"""Phase 7 closed-loop campaign: E0-E4 scenarios x T0-T3 estimators.

Everything from Scientific Baseline 0 is kept identical (BlueROV2 Heavy,
dynamics mode, no teleport, no-DVL commanded odometry, CURRENT committed
planner, same nominal command, same obstacle, same start): the ONLY varied
factor is the temporal estimator between the perception relay and the
planner. Planner parameters are NOT changed per method.

Scenarios (dropout timing relative to AVOIDING commitment):
  E0  normal detection (no dropout)
  E1  EARLY 2 s detector SILENCE, 1 s after commitment
      == the original Baseline-0 attempt-1 C_3 failing condition (regression)
  E2  mid-maneuver 2 s silence (3 s after commitment)
  E3  repeated 0.5 s silences (1.0 s on / 0.5 s off for 6 s)
  E4  one large bbox outlier injected 0.15 s after the first detection
      (closed-loop engagement is near-immediate, so this is effectively
      'before/at engagement'), no dropout

Usage (sourced ROS env):
    python scripts/run_temporal_closedloop_campaign.py \
        --methods t0,t1,t2,t3 --scenarios E0,E1,E2,E3,E4 --runs 3
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIO_YAML = os.path.join(
    REPO, "src", "rov_obstacle_sim_bridge", "config", "holoocean_scenarios",
    "bluerov2_baseline0_obstacle.yaml")

# Reuse the baseline0 orchestrator's process helpers.
spec = importlib.util.spec_from_file_location(
    "b0", os.path.join(REPO, "scripts", "run_baseline0_campaign.py"))
b0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b0)

E_SCENARIOS = {
    "E0": {"desc": "normal detection", "launch_args": [
        "dropout_enabled:=false"]},
    "E1": {"desc": "EARLY 2s silence 1s after commit (C_3 regression)",
           "launch_args": ["dropout_enabled:=true", "dropout_mode:=single",
                           "dropout_delay_s:=1.0", "dropout_duration_s:=2.0"]},
    "E2": {"desc": "mid-maneuver 2s silence (3s after commit)",
           "launch_args": ["dropout_enabled:=true", "dropout_mode:=single",
                           "dropout_delay_s:=3.0", "dropout_duration_s:=2.0"]},
    "E3": {"desc": "repeated 0.5s silences for 6s",
           "launch_args": ["dropout_enabled:=true", "dropout_mode:=repeated",
                           "dropout_delay_s:=1.0", "repeat_on_s:=1.0",
                           "repeat_off_s:=0.5", "repeat_total_s:=6.0"]},
    "E4": {"desc": "one large outlier at first detection +0.15s",
           "launch_args": ["dropout_enabled:=false", "outlier_at_s:=0.15"]},
}
DURATION_S = 120.0


def run_once(method: str, scenario: str, run_idx: int, out_root: str) -> dict:
    spec_e = E_SCENARIOS[scenario]
    run_dir = os.path.join(out_root, "runs", f"{scenario}_{method}_{run_idx}")
    os.makedirs(run_dir, exist_ok=True)
    validator_out = os.path.join(run_dir, "validation.json")

    procs, logs = [], []
    result = {"scenario": scenario, "method": method, "run": run_idx,
              "ok": False, "desc": spec_e["desc"]}
    try:
        p, lg = b0.start([b0.ocean_python(),
                          os.path.join(REPO, "src", "rov_obstacle_sim_bridge",
                                       "holoocean_server",
                                       "holoocean_sim_server.py"),
                          "--config", SCENARIO_YAML, "--serve"],
                         os.path.join(run_dir, "sim_server.log"))
        procs.append(p); logs.append(lg)
        if not b0.wait_for_port("127.0.0.1", 47654, timeout_s=300):
            result["error"] = "sim server port never opened"
            return result

        env = dict(os.environ)
        env.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        env.setdefault("ZENOH_ROUTER_CHECK_ATTEMPTS", "20")
        p, lg = b0.start(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                         os.path.join(run_dir, "zenoh.log"), env=env)
        procs.append(p); logs.append(lg)
        time.sleep(6.0)

        launch_cmd = [
            "ros2", "launch", "rov_obstacle_sim_bridge",
            "holoocean_baseline0.launch.py",
            f"estimator_method:={method}",
            f"validator_output:={validator_out}",
            f"label:=temporal_{scenario}_{method}_{run_idx}",
        ] + spec_e["launch_args"]
        launched = False
        for attempt in (1, 2):
            p, lg = b0.start(launch_cmd,
                             os.path.join(run_dir, f"ros2_launch_{attempt}.log"),
                             env=env)
            procs.append(p); logs.append(lg)
            if b0.wait_for_graph_liveness(validator_out, timeout_s=30.0):
                launched = True
                break
            print(f"[temporal] graph not live (attempt {attempt}); retry",
                  flush=True)
            b0.stop(p)
            time.sleep(3.0)
        if not launched:
            result["error"] = "graph liveness failed twice (technical invalid)"
            result["technical_invalid"] = True
            return result

        time.sleep(DURATION_S)
        result["ok"] = True
    finally:
        for p in reversed(procs):
            b0.stop(p)
        for lg in logs:
            try:
                lg.close()
            except Exception:
                pass
        time.sleep(2.0)

    if os.path.isfile(validator_out):
        try:
            with open(validator_out) as f:
                result["metrics"] = json.load(f)
        except json.JSONDecodeError as exc:
            result["ok"] = False
            result["error"] = f"validator output corrupt: {exc}"
    else:
        result["ok"] = False
        result["error"] = "validator output missing"
    return result


def assess(scenario: str, m: dict) -> dict:
    """Descriptive outcome (NOT pass/fail thresholds tuned per method —
    a failed method is a valid scientific result)."""
    if not m:
        return {"outcome": "no_data"}
    seq = [s["state"] for s in m.get("state_sequence", [])]
    aborted = False
    for i in range(1, len(seq)):
        if seq[i] == "NORMAL" and seq[i - 1].startswith("AVOIDING"):
            aborted = True  # AVOIDING -> NORMAL without RECOVERING
    return {
        "collision": m.get("collision"),
        "min_clearance_m": m.get("min_clearance_m"),
        "returned": m.get("returned_to_original_line"),
        "avoidance_entries": m.get("avoidance_entries"),
        "reengagements": max(0, (m.get("avoidance_entries") or 0) - 1),
        "maneuver_aborted_to_normal": aborted,
        "side_switches": m.get("side_switches"),
        "final_lateral_m": m.get("final_lateral_error_m"),
        "final_yaw_deg": m.get("final_yaw_error_deg"),
        "max_lat_dev_m": m.get("max_lateral_deviation_m"),
        "prediction_ticks": m.get("estimator_prediction_ticks"),
        "max_time_since_meas_s": m.get("estimator_max_time_since_meas_s"),
        "odo_err_max_m": m.get("odo_err_max_m"),
        "thruster_peak_n": m.get("thruster_peak_n"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--methods", default="t0,t1,t2,t3")
    parser.add_argument("--scenarios", default="E0,E1,E2,E3,E4")
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--out", default=os.path.join(
        REPO, "experiments", "simulation", "temporal_estimators",
        "closedloop"))
    args = parser.parse_args()

    methods = [m.strip() for m in args.methods.split(",") if m.strip()]
    scenarios = [s.strip().upper() for s in args.scenarios.split(",")
                 if s.strip()]
    os.makedirs(args.out, exist_ok=True)
    results = []
    t0 = time.time()
    total = len(methods) * len(scenarios) * args.runs
    n = 0
    for sc in scenarios:
        for method in methods:
            for i in range(1, args.runs + 1):
                n += 1
                print(f"[temporal] === {n}/{total}: {sc} {method} "
                      f"run {i} ===", flush=True)
                r = run_once(method, sc, i, args.out)
                r["assessment"] = assess(sc, r.get("metrics") or {})
                a = r["assessment"]
                print(f"[temporal] {sc}/{method}/{i}: {json.dumps(a)}",
                      flush=True)
                results.append(r)
                # Keep the manifest current so progress survives any crash.
                with open(os.path.join(args.out, "manifest.json"), "w") as f:
                    json.dump({
                        "campaign": "temporal_estimators_closedloop",
                        "test_type": "simulation dynamics integration baseline"
                                     " + temporal estimation (oracle "
                                     "perception, NOT a visual result)",
                        "generated_utc": time.strftime(
                            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                        "commit_sha": b0.git_sha(),
                        "wall_time_s": round(time.time() - t0, 1),
                        "scenarios": {k: v["desc"]
                                      for k, v in E_SCENARIOS.items()},
                        "duration_s": DURATION_S,
                        "results": results,
                    }, f, indent=2)
    print(f"[temporal] campaign done: {args.out}/manifest.json", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
