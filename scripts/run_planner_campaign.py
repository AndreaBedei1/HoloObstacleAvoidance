"""Phase 8 planner campaign: committed (C) vs holonomic DWA (D), paired.

Everything except the planner is fixed: T2 + Phase-7B qualification, same
scenarios, same nominal, same dynamics. Scenario geometry comes from
per-case scenario YAMLs (planner_F*.yaml); perception perturbations (F7
dropout, F8 outlier) from relay launch args; F10 higher approach speed from
a nominal override.

Usage (sourced ROS env):
    python scripts/run_planner_campaign.py --planners committed,dwa \
        --scenarios F0..F10 --runs N --out <dir> [--dwa-args k:=v ...]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCEN = os.path.join(REPO, "src", "rov_obstacle_sim_bridge", "config",
                    "holoocean_scenarios")

spec = importlib.util.spec_from_file_location(
    "b0", os.path.join(REPO, "scripts", "run_baseline0_campaign.py"))
b0 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b0)

F_SCENARIOS = {
    "F0": {"yaml": "planner_F0.yaml", "args": [], "desc": "central obstacle"},
    "F1": {"yaml": "planner_F1.yaml", "args": [], "desc": "obstacle 1.5 m left"},
    "F2": {"yaml": "planner_F2.yaml", "args": [], "desc": "obstacle 1.5 m right"},
    "F3": {"yaml": "planner_F3.yaml", "args": [], "desc": "diagonal surrogate left 3 m"},
    "F4": {"yaml": "planner_F4.yaml", "args": [], "desc": "diagonal surrogate right 3 m"},
    "F5": {"yaml": "planner_F5.yaml", "args": [], "desc": "smaller-extent surrogate"},
    "F6": {"yaml": "planner_F6.yaml", "args": [], "desc": "reduced left clearance"},
    "F7": {"yaml": "planner_F0.yaml",
           "args": ["dropout_enabled:=true", "dropout_mode:=single",
                    "dropout_delay_s:=3.0", "dropout_duration_s:=2.0"],
           "desc": "mid-maneuver 2 s perception dropout"},
    "F8": {"yaml": "planner_F0.yaml", "args": ["outlier_at_s:=0.15"],
           "desc": "young-track giant outlier"},
    "F9": {"yaml": "planner_F9.yaml", "args": [],
           "desc": "initial heading error +15 deg"},
    "F10": {"yaml": "planner_F0.yaml", "args": ["nominal_surge:=0.4"],
            "desc": "higher approach speed 0.4 m/s"},
}
DURATION_S = 120.0


def run_once(planner: str, scenario: str, run_idx: int, out_root: str,
             dwa_args) -> dict:
    fs = F_SCENARIOS[scenario]
    run_dir = os.path.join(out_root, "runs",
                           f"{scenario}_{planner}_{run_idx}")
    os.makedirs(run_dir, exist_ok=True)
    validator_out = os.path.join(run_dir, "validation.json")
    scenario_yaml = os.path.join(SCEN, fs["yaml"])

    procs, logs = [], []
    result = {"scenario": scenario, "planner": planner, "run": run_idx,
              "ok": False, "desc": fs["desc"]}
    try:
        p, lg = b0.start([b0.ocean_python(),
                          os.path.join(REPO, "src", "rov_obstacle_sim_bridge",
                                       "holoocean_server",
                                       "holoocean_sim_server.py"),
                          "--config", scenario_yaml, "--serve"],
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
            f"planner:={planner}",
            "estimator_method:=t2",
            f"validator_output:={validator_out}",
            f"label:=planner_{scenario}_{planner}_{run_idx}",
        ] + fs["args"] + list(dwa_args)
        launched = False
        for attempt in (1, 2):
            p, lg = b0.start(launch_cmd,
                             os.path.join(run_dir,
                                          f"ros2_launch_{attempt}.log"),
                             env=env)
            procs.append(p); logs.append(lg)
            if b0.wait_for_graph_liveness(validator_out, timeout_s=30.0):
                launched = True
                break
            print(f"[planner8] graph not live (attempt {attempt}); retry",
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


def assess(m: dict) -> dict:
    if not m:
        return {"outcome": "no_data"}
    seq = [s["state"] for s in m.get("state_sequence", [])]
    aborted = any(seq[i] == "NORMAL" and seq[i - 1].startswith("AVOIDING")
                  for i in range(1, len(seq)))
    return {
        "collision": m.get("collision"),
        "min_clearance_m": m.get("min_clearance_m"),
        "returned": m.get("returned_to_original_line"),
        "final_lateral_m": m.get("final_lateral_error_m"),
        "final_yaw_deg": m.get("final_yaw_error_deg"),
        "max_lat_dev_m": m.get("max_lateral_deviation_m"),
        "path_length_m": m.get("path_length_m"),
        "forward_progress_m": m.get("forward_progress_m"),
        "maneuver_time_s": m.get("maneuver_time_s"),
        "avoidance_entries": m.get("avoidance_entries"),
        "side_switches": m.get("side_switches"),
        "maneuver_aborted_to_normal": aborted,
        "thruster_peak_n": m.get("thruster_peak_n"),
        "thruster_mean_abs_n": m.get("thruster_mean_abs_n"),
        "cmd_smoothness": m.get("cmd_smoothness_mean_delta"),
        "odo_err_max_m": m.get("odo_err_max_m"),
        "infra_freeze": m.get("infra_freeze_detected"),
        "dist_at_commit_m": m.get("distance_at_commitment_m"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planners", default="committed,dwa")
    parser.add_argument("--scenarios", default=",".join(F_SCENARIOS))
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--out", required=True)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--dwa-args", nargs="*", default=[])
    args = parser.parse_args()
    global DURATION_S
    if args.duration:
        DURATION_S = args.duration

    planners = [p.strip() for p in args.planners.split(",") if p.strip()]
    scenarios = [s.strip().upper() for s in args.scenarios.split(",")
                 if s.strip()]
    os.makedirs(args.out, exist_ok=True)
    results = []
    t0 = time.time()
    total = len(planners) * len(scenarios) * args.runs
    n = 0
    for sc in scenarios:
        for planner in planners:
            for i in range(1, args.runs + 1):
                n += 1
                print(f"[planner8] === {n}/{total}: {sc} {planner} run {i} "
                      "===", flush=True)
                r = run_once(planner, sc, i, args.out, args.dwa_args)
                r["assessment"] = assess(r.get("metrics") or {})
                print(f"[planner8] {sc}/{planner}/{i}: "
                      f"{json.dumps(r['assessment'])}", flush=True)
                results.append(r)
                with open(os.path.join(args.out, "manifest.json"), "w") as f:
                    json.dump({
                        "campaign": "phase8_planner_comparison",
                        "test_type": "simulation dynamics integration "
                                     "baseline + planner comparison "
                                     "(oracle perception, NOT visual)",
                        "generated_utc": time.strftime(
                            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                        "commit_sha": b0.git_sha(),
                        "wall_time_s": round(time.time() - t0, 1),
                        "perception_stack": "t2 + phase7b qualification",
                        "scenarios": {k: v["desc"]
                                      for k, v in F_SCENARIOS.items()},
                        "dwa_args": list(args.dwa_args),
                        "duration_s": DURATION_S,
                        "results": results,
                    }, f, indent=2)
    print(f"[planner8] campaign done -> {args.out}/manifest.json", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
