"""Scientific Baseline 0 campaign orchestrator.

Runs closed-loop scenarios with FRESH processes per run (sim server + zenoh
router + ROS 2 launch), collects validator JSON + logs into
`experiments/simulation/scientific_baseline_0/`, and aggregates.

Scenarios:
  A  straight, no obstacle          (control test)
  B  one obstacle, oracle relay     (committed avoidance)
  C  = B + deterministic detector dropout during the maneuver

Run from the sourced ROS 2 environment (see scripts/source_ros2_windows.bat);
the sim server subprocess uses the holoocean conda env python (OCEAN_PYTHON
env var or the known local default).

Usage:
    python scripts/run_baseline0_campaign.py --scenarios A,B,C --runs 3
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIO_DIR = os.path.join(
    REPO, "src", "rov_obstacle_sim_bridge", "config", "holoocean_scenarios")

SCENARIOS = {
    "A": {
        "yaml": "bluerov2_baseline0_straight.yaml",
        "duration_s": 45.0,
        "dropout": False,
        "expect_obstacle": False,
        "desc": "straight transit, no obstacle",
    },
    "B": {
        "yaml": "bluerov2_baseline0_obstacle.yaml",
        "duration_s": 120.0,
        "dropout": False,
        "expect_obstacle": True,
        "desc": "committed avoidance, oracle perception",
    },
    "C": {
        "yaml": "bluerov2_baseline0_obstacle.yaml",
        "duration_s": 120.0,
        "dropout": True,
        "expect_obstacle": True,
        "desc": "committed avoidance with deterministic detector dropout",
    },
}


def ocean_python() -> str:
    cand = os.environ.get("OCEAN_PYTHON")
    if cand and os.path.isfile(cand):
        return cand
    default = os.path.join(os.environ.get("USERPROFILE", ""), "miniconda3",
                           "envs", "holoocean_joystick", "python.exe")
    if os.path.isfile(default):
        return default
    raise RuntimeError("holoocean python not found; set OCEAN_PYTHON")


def git_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"],
                                       cwd=REPO, text=True).strip()
    except Exception:
        return "unknown"


def wait_for_graph_liveness(validator_json: str, timeout_s: float) -> bool:
    """True once the validator reports bridge traffic (dynamics_debug > 10)."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            with open(validator_json) as f:
                m = json.load(f)
            if (m.get("msg_counts") or {}).get("dynamics_debug", 0) > 10:
                return True
        except (OSError, json.JSONDecodeError):
            pass
        time.sleep(2.0)
    return False


def wait_for_port(host: str, port: int, timeout_s: float) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            time.sleep(2.0)
    return False


def start(cmd, log_path, env=None):
    log = open(log_path, "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            cwd=REPO, env=env), log


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                   capture_output=True)


def run_once(scenario_key: str, run_idx: int, out_root: str) -> dict:
    spec = SCENARIOS[scenario_key]
    run_dir = os.path.join(out_root, "runs", f"{scenario_key}_{run_idx}")
    os.makedirs(run_dir, exist_ok=True)
    validator_out = os.path.join(run_dir, "validation.json")
    scenario_yaml = os.path.join(SCENARIO_DIR, spec["yaml"])

    procs = []
    logs = []
    result = {"scenario": scenario_key, "run": run_idx, "ok": False,
              "desc": spec["desc"]}
    try:
        # 1. Fresh sim server (fresh engine process).
        p, lg = start([ocean_python(),
                       os.path.join(REPO, "src", "rov_obstacle_sim_bridge",
                                    "holoocean_server",
                                    "holoocean_sim_server.py"),
                       "--config", scenario_yaml, "--serve"],
                      os.path.join(run_dir, "sim_server.log"))
        procs.append(p); logs.append(lg)
        if not wait_for_port("127.0.0.1", 47654, timeout_s=300):
            result["error"] = "sim server port never opened"
            return result

        # 2. Zenoh router (matches the repo's tested RMW setup).
        env = dict(os.environ)
        env.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        # A node whose zenoh session initializes before the router is
        # reachable proceeds ISOLATED (publishes to nobody). Force every
        # node to retry the router check until it succeeds.
        env.setdefault("ZENOH_ROUTER_CHECK_ATTEMPTS", "20")
        p, lg = start(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                      os.path.join(run_dir, "zenoh.log"), env=env)
        procs.append(p); logs.append(lg)
        time.sleep(6.0)

        # 3. ROS 2 launch (bridge, relay, odometry, nominal, planner, validator)
        #    with a liveness gate: the bridge's zenoh session occasionally
        #    starts isolated (seen as run A_2: one state message then
        #    silence); detect a dead graph within 30 s and retry once.
        launch_cmd = [
            "ros2", "launch", "rov_obstacle_sim_bridge",
            "holoocean_baseline0.launch.py",
            f"dropout_enabled:={'true' if spec['dropout'] else 'false'}",
            f"validator_output:={validator_out}",
            f"label:=baseline0_{scenario_key}_{run_idx}",
        ]
        launched_ok = False
        for attempt in (1, 2):
            p, lg = start(launch_cmd,
                          os.path.join(run_dir, f"ros2_launch_{attempt}.log"),
                          env=env)
            procs.append(p); logs.append(lg)
            if wait_for_graph_liveness(validator_out, timeout_s=30.0):
                launched_ok = True
                break
            print(f"[campaign] graph not live (attempt {attempt}); "
                  "restarting launch", flush=True)
            stop(p)
            time.sleep(3.0)
        if not launched_ok:
            result["error"] = "graph liveness failed twice (technical invalid)"
            result["technical_invalid"] = True
            return result

        # 4. Let the run happen.
        time.sleep(spec["duration_s"])
        result["ok"] = True
    finally:
        for p in reversed(procs):
            stop(p)
        for lg in logs:
            try:
                lg.close()
            except Exception:
                pass
        time.sleep(2.0)

    # 5. Collect validator output.
    if os.path.isfile(validator_out):
        with open(validator_out) as f:
            result["metrics"] = json.load(f)
    else:
        result["ok"] = False
        result["error"] = "validator output missing"
    return result


def acceptance(scenario: str, m: dict) -> tuple[bool, list]:
    """Pre-registered per-scenario acceptance checks."""
    problems = []
    if not m:
        return False, ["no metrics"]
    def num(key, default):
        v = m.get(key)
        return default if v is None else v

    if scenario == "A":
        if num("forward_progress_m", 0.0) < 8.0:
            problems.append("forward progress < 8 m")
        if num("max_lateral_deviation_m", 99.0) > 1.0:
            problems.append("lateral deviation > 1 m on straight run")
        if abs(num("final_yaw_error_deg", 99.0)) > 10:
            problems.append("heading unstable")
        if num("roll_max_deg", 99.0) > 15 or num("pitch_max_deg", 99.0) > 15:
            problems.append("roll/pitch instability")
        if m.get("avoidance_entries", 0) != 0:
            problems.append("spurious avoidance without obstacle")
    else:
        if m.get("collision") is not False:
            problems.append("collision or clearance not computed")
        if num("min_clearance_m", -1.0) < 0.3:
            problems.append("min clearance < 0.3 m")
        if m.get("avoidance_entries", 0) < 1:
            problems.append("no avoidance maneuver")
        if m.get("side_switches", 99) > 0:
            problems.append("side switching occurred")
        if m.get("returned_to_original_line") is not True:
            problems.append("did not return to original line")
        seq = [s["state"] for s in m.get("state_sequence", [])]
        if not any(s.startswith("AVOIDING") for s in seq):
            problems.append("state machine never entered AVOIDING")
    if scenario == "C":
        if not any(e.get("event") == "dropout_start"
                   for e in m.get("dropout_events", [])):
            problems.append("dropout was never injected")
    return (len(problems) == 0), problems


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", default="A,B,C")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--out", default=os.path.join(
        REPO, "experiments", "simulation", "scientific_baseline_0"))
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    all_results = []
    t0 = time.time()
    for sc in [s.strip().upper() for s in args.scenarios.split(",") if s.strip()]:
        for i in range(1, args.runs + 1):
            print(f"[campaign] === scenario {sc} run {i}/{args.runs} ===",
                  flush=True)
            r = run_once(sc, i, args.out)
            m = r.get("metrics") or {}
            ok, problems = acceptance(sc, m)
            r["accepted"] = ok
            r["problems"] = problems
            print(f"[campaign] {sc}{i}: accepted={ok} problems={problems} "
                  f"clearance={m.get('min_clearance_m')} "
                  f"final_lat={m.get('final_lateral_error_m')} "
                  f"odo_err_max={m.get('odo_err_max_m')}", flush=True)
            all_results.append(r)

    manifest = {
        "campaign": "scientific_baseline_0",
        "test_type": "simulation dynamics integration baseline",
        "perception": "oracle relay (primitive stand-in obstacle)",
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "commit_sha": git_sha(),
        "wall_time_s": round(time.time() - t0, 1),
        "scenario_specs": SCENARIOS,
        "results": all_results,
        "summary": {
            sc: {
                "runs": sum(1 for r in all_results if r["scenario"] == sc),
                "accepted": sum(1 for r in all_results
                                if r["scenario"] == sc and r["accepted"]),
            }
            for sc in set(r["scenario"] for r in all_results)
        },
    }
    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"[campaign] manifest -> {args.out}/manifest.json", flush=True)
    print(json.dumps(manifest["summary"], indent=2), flush=True)

    failed = [r for r in all_results if not r["accepted"]]
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
