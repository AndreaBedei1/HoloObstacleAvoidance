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
    # Development-only tuning geometries (never in the final campaign).
    "P0": {"yaml": "planner_P0.yaml", "args": [],
           "desc": "TUNING: no obstacle (route-hold sanity)"},
    "P1": {"yaml": "planner_P1.yaml", "args": [],
           "desc": "TUNING: central obstacle at 11 m"},
    "P2": {"yaml": "planner_P2.yaml", "args": [],
           "desc": "TUNING: obstacle 2 m left at 11 m"},
    # Pool-scale feasibility profile (K-series): ~8 m pool geometry, small
    # obstacle class, shared monocular constants scaled for BOTH planners;
    # planner-specific tuned parameters stay frozen (as-is transfer test).
    "K0": {"duration_s": 90.0, "yaml": "planner_K0.yaml",
           "args": ["nominal_surge:=0.15", "target_obstacle_height_m:=0.5",
                    "dwa_obstacle_radius_m:=0.25",
                    "dwa_goal_lookahead_m:=4.0"],
           "desc": "POOL: central 0.5 m obstacle at 3.5 m, 0.15 m/s"},
    "K1": {"duration_s": 90.0, "yaml": "planner_K1.yaml",
           "args": ["nominal_surge:=0.15", "target_obstacle_height_m:=0.5",
                    "dwa_obstacle_radius_m:=0.25",
                    "dwa_goal_lookahead_m:=4.0"],
           "desc": "POOL: 0.5 m obstacle 0.75 m left at 3.5 m, 0.15 m/s"},
}
DURATION_S = 120.0


def run_once(planner: str, scenario: str, run_idx: int, out_root: str,
             dwa_args) -> dict:
    fs = F_SCENARIOS[scenario]
    duration = fs.get("duration_s", DURATION_S)
    run_dir = os.path.join(out_root, "runs",
                           f"{scenario}_{planner}_{run_idx}")
    os.makedirs(run_dir, exist_ok=True)
    validator_out = os.path.join(run_dir, "validation.json")
    scenario_yaml = os.path.join(SCEN, fs["yaml"])

    procs, logs = [], []
    result = {"scenario": scenario, "planner": planner, "run": run_idx,
              "ok": False, "desc": fs["desc"]}
    try:
        env = dict(os.environ)
        env.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
        env.setdefault("ZENOH_ROUTER_CHECK_ATTEMPTS", "20")
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
            # FULL environment start per attempt, sim server included: a
            # retry that reuses the running engine inherits whatever pose,
            # yaw, and velocity attempt 1 left behind, so the route the
            # planner/validator capture at startup is poisoned (observed:
            # a diagonal route from a -22 deg residual yaw).
            p, lg = b0.start([b0.ocean_python(),
                              os.path.join(REPO, "src",
                                           "rov_obstacle_sim_bridge",
                                           "holoocean_server",
                                           "holoocean_sim_server.py"),
                              "--config", scenario_yaml, "--serve"],
                             os.path.join(run_dir,
                                          f"sim_server_{attempt}.log"))
            procs.append(p); logs.append(lg)
            if not b0.wait_for_port("127.0.0.1", 47654, timeout_s=300):
                result["error"] = "sim server port never opened"
                return result
            p, lg = b0.start(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                             os.path.join(run_dir, f"zenoh_{attempt}.log"),
                             env=env)
            procs.append(p); logs.append(lg)
            time.sleep(6.0)
            p, lg = b0.start(launch_cmd,
                             os.path.join(run_dir,
                                          f"ros2_launch_{attempt}.log"),
                             env=env)
            procs.append(p); logs.append(lg)
            if b0.wait_for_graph_liveness(validator_out, timeout_s=30.0):
                launched = True
                break
            print(f"[planner8] graph not live (attempt {attempt}); "
                  "FULL restart incl. sim server", flush=True)
            for pp in reversed(procs):
                b0.stop(pp)
            procs.clear()
            time.sleep(3.0)
        if not launched:
            result["error"] = "graph liveness failed twice (technical invalid)"
            result["technical_invalid"] = True
            return result

        time.sleep(duration)
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


def is_technical_invalid(r: dict) -> str | None:
    """Objective technical-invalid signatures (protocol section 6)."""
    if r.get("technical_invalid"):
        return r.get("error", "launch")
    m = r.get("metrics")
    if not m:
        return r.get("error", "no validator output")
    if m.get("infra_freeze_detected"):
        return "infra_freeze"
    if m.get("cmd_path_dead_detected"):
        return "cmd_path_dead"
    return None


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
        # Alternate planners inside the scenario block (protocol section 4)
        # so drift in engine load affects both planners symmetrically.
        for i in range(1, args.runs + 1):
            for planner in planners:
                n += 1
                print(f"[planner8] === {n}/{total}: {sc} {planner} run {i} "
                      "===", flush=True)
                r = run_once(planner, sc, i, args.out, args.dwa_args)
                why = is_technical_invalid(r)
                if why:
                    # Protocol section 4: technical-invalid runs are
                    # excluded and re-run (ONCE); algorithm failures never.
                    print(f"[planner8] technical invalid ({why}) -> "
                          "one re-run", flush=True)
                    r_retry = run_once(planner, sc, i, args.out,
                                       args.dwa_args)
                    r_retry["replaced_technical_invalid"] = why
                    if not is_technical_invalid(r_retry):
                        r = r_retry
                    else:
                        r["also_retry_invalid"] = True
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
