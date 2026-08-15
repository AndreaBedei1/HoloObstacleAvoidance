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

# Phase 10 calibration ladder, appended to every launch. Set by --calib
# in main(); empty means S0, the historical simulator untouched.
CALIB_ARGS: list = []
CALIB_LEVEL = "S0"

# The fit files are REQUIRED at S1 and above: the relay refuses to run
# without them rather than silently falling back to an uncalibrated
# stand-in, which would produce results labelled "calibrated" that are
# nothing of the kind.
# Forward slashes: these paths travel as `key:=value` launch arguments,
# where a Windows backslash is an escape character and the path arrives
# mangled. The node then cannot find the file, raises in its constructor
# and dies silently -- the run still completes, because the rest of the
# graph is alive, and it looks like a calibrated result that is not one.
S1_FIT = os.path.join(REPO, "config", "calibration",
                      "s1_observation_fit.json").replace("\\", "/")
S2_FIT = os.path.join(REPO, "config", "calibration",
                      "s2_timing_fit.json").replace("\\", "/")


S3_FIT = os.path.join(REPO, "config", "calibration",
                      "s3_vehicle.json").replace("\\", "/")


# S3 is a PLANT model applied downstream of /planner/cmd_vel_safe. It
# passes the fit FILE to the actuation model rather than deriving
# planner limits from it: changing the planner at a calibration rung
# would change the controller and the simulator together and destroy the
# causal reading of S0 -> S3.


def calibration_args(level: str) -> list:
    level = level.upper().strip()
    if level == "S0":
        return ["calibration_level:=S0"]
    if level in ("S1", "S2", "S3"):
        out = [f"calibration_level:={level}", f"s1_fit_path:={S1_FIT}"]
        if level in ("S2", "S3"):
            out.append(f"s2_fit_path:={S2_FIT}")
        if level == "S3":
            out.append(f"s3_fit_path:={S3_FIT}")
        return out
    raise SystemExit("unknown calibration level: %s" % level)


def pool_benchmark() -> dict:
    """The frozen pool configuration, read from the single shared file.

    The simulated campaign and the real pipeline both read this, so the
    matched experiment cannot drift apart between the two domains. The
    values are not restated here.
    """
    import yaml
    with open(os.path.join(REPO, "config",
                           "pool_benchmark_FROZEN.yaml")) as f:
        return yaml.safe_load(f)["pool_benchmark"]


def pool_args() -> list:
    b = pool_benchmark()
    return [f"nominal_surge:={b['nominal_surge']}",
            f"target_obstacle_height_m:={b['target_obstacle_height_m']}",
            f"dwa_obstacle_radius_m:={b['dwa_obstacle_radius_m']}",
            f"dwa_goal_lookahead_m:={b['dwa_goal_lookahead_m']}",
            f"engage_distance_m:={b['engage_distance_m']}"]


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
           "args": None,          # filled from the frozen pool benchmark
           "desc": "POOL: central 0.5 m obstacle at 3.5 m, 0.15 m/s"},
    "K1": {"duration_s": 90.0, "yaml": "planner_K1.yaml",
           "args": None,          # filled from the frozen pool benchmark
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
            "relay_status_path:=" + os.path.join(
                run_dir, "relay_status.json").replace("\\", "/"),
            "plant_status_path:=" + os.path.join(
                run_dir, "plant_status.json").replace("\\", "/"),
            f"label:=planner_{scenario}_{planner}_{run_idx}",
        ] + (fs["args"] if fs["args"] is not None else pool_args())             + list(dwa_args) + list(CALIB_ARGS)
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
        result["calib_level"] = CALIB_LEVEL
        # The full launch invocation is recorded so rung ownership can be
        # AUDITED from the stored data rather than trusted: per-run paths
        # are dropped because they legitimately differ run to run.
        result["launch_args"] = sorted(
            a for a in launch_cmd[4:]
            if not a.startswith(("validator_output:=", "label:=",
                                 "relay_status_path:=",
                                 "plant_status_path:="))) 
    finally:
        for p in reversed(procs):
            b0.stop(p)
        for lg in logs:
            try:
                lg.close()
            except Exception:
                pass
        time.sleep(2.0)

    # Read AFTER the log files are closed: reading them while the launch
    # was still running returned None for a relay that had in fact
    # announced itself correctly, because the line had not been flushed
    # to disk yet -- a false alarm from the very guard meant to catch
    # false results. Recorded per run so the level is auditable from the
    # manifest alone, not only from the logs.
    result["relay_level"] = relay_level_from_logs(run_dir)

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
    # The calibrated relay must be ALIVE and at the requested level.
    # It once died in its constructor on a mangled path and printed
    # nothing: the rest of the graph stayed up, the run produced a
    # complete set of metrics, and the result was indistinguishable from
    # a calibrated one. A campaign of such runs would be labelled S1 and
    # be pure S0. This check makes that failure loud.
    if r.get("relay_level") != r.get("calib_level"):
        return ("calibrated relay reported %r, expected %r"
                % (r.get("relay_level"), r.get("calib_level")))
    m = r.get("metrics")
    if not m:
        return r.get("error", "no validator output")
    if m.get("infra_freeze_detected"):
        return "infra_freeze"
    if m.get("cmd_path_dead_detected"):
        return "cmd_path_dead"
    return None


def relay_level_from_logs(run_dir: str):
    """The level the relay actually ran at, from its sentinel file.

    Read from a file the node writes at construction, not from the
    launch log: the startup line is not reliably flushed before the
    process is killed at teardown, so log parsing failed runs that were
    perfectly calibrated and doubled the campaign's cost by re-running
    them.
    """
    path = os.path.join(run_dir, "relay_status.json")
    try:
        with open(path) as f:
            return json.load(f).get("level")
    except (OSError, json.JSONDecodeError):
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
        # Output-derived, so they exist for BOTH planners and can be
        # recomputed identically from the real runs.
        "lateral_commit_dist_m": m.get("lateral_commit_dist_m"),
        "lateral_maneuver_s": m.get("lateral_maneuver_s"),
        "lateral_peak_m_s": m.get("lateral_peak_m_s"),
    }


def has_valid_outcome(r: dict) -> bool:
    """A tuple is DONE only if its final record is a usable experimental
    outcome: the run completed and is not technical-invalid. Algorithm
    failures (collision, abort, no return) ARE valid outcomes and must never
    be re-run (protocol section 4)."""
    if not r.get("ok"):
        return False
    if r.get("technical_invalid") or r.get("also_retry_invalid"):
        return False
    return bool(r.get("metrics"))


def load_prior(out_root: str) -> list:
    """Read already-completed runs so a resumed campaign continues instead of
    restarting. Manifest first (it carries the assessments); per-run
    validation.json is the fallback if the manifest was lost or truncated by
    a kill. Never invents outcomes."""
    manifest = os.path.join(out_root, "manifest.json")
    prior = []
    if os.path.isfile(manifest):
        try:
            with open(manifest) as f:
                prior = json.load(f).get("results", [])
            print(f"[planner8] resume: manifest has {len(prior)} records",
                  flush=True)
        except (json.JSONDecodeError, OSError) as exc:
            print(f"[planner8] resume: manifest unreadable ({exc}); "
                  "falling back to per-run artifacts", flush=True)
            prior = []
    have = {(r["scenario"], r["planner"], r["run"]) for r in prior}
    runs_dir = os.path.join(out_root, "runs")
    recovered = 0
    if os.path.isdir(runs_dir):
        for name in sorted(os.listdir(runs_dir)):
            vj = os.path.join(runs_dir, name, "validation.json")
            if not os.path.isfile(vj):
                continue
            try:
                scenario, planner, idx = name.rsplit("_", 2)
                key = (scenario, planner, int(idx))
            except ValueError:
                continue
            if key in have:
                continue
            try:
                with open(vj) as f:
                    metrics = json.load(f)
            except (json.JSONDecodeError, OSError):
                continue
            rec = {"scenario": scenario, "planner": planner, "run": int(idx),
                   "ok": True, "desc": F_SCENARIOS.get(scenario, {})
                   .get("desc", ""), "metrics": metrics,
                   "recovered_from_run_dir": True}
            rec["assessment"] = assess(metrics)
            why = is_technical_invalid(rec)
            if why:
                rec["ok"] = False
                rec["technical_invalid"] = True
                rec["error"] = why
            prior.append(rec)
            have.add(key)
            recovered += 1
    if recovered:
        print(f"[planner8] resume: recovered {recovered} run(s) from "
              "per-run artifacts", flush=True)
    return prior


def preserve_partial(run_dir: str) -> None:
    """Move an existing run directory's artifacts aside before re-running the
    same planned tuple, so an interrupted or technical-invalid attempt is
    never silently overwritten (protocol: preserve, label, re-run)."""
    if not os.path.isdir(run_dir):
        return
    entries = [e for e in os.listdir(run_dir)
               if not e.startswith("superseded_")]
    if not entries:
        return
    n = 1
    while os.path.exists(os.path.join(run_dir, f"superseded_{n}")):
        n += 1
    dest = os.path.join(run_dir, f"superseded_{n}")
    os.makedirs(dest, exist_ok=True)
    for e in entries:
        try:
            os.replace(os.path.join(run_dir, e), os.path.join(dest, e))
        except OSError:
            pass
    with open(os.path.join(dest, "REASON.txt"), "w") as f:
        f.write("interrupted_session / technical-invalid attempt preserved "
                "before a protocol re-run of the same planned tuple.\n"
                "Not counted as an algorithm success or failure.\n")
    print(f"[planner8] preserved prior artifacts -> {dest}", flush=True)


def write_manifest(path: str, payload: dict) -> None:
    """Atomic manifest write: a kill mid-write must not destroy the record of
    every completed run."""
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f, indent=2)
    os.replace(tmp, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--planners", default="committed,dwa")
    parser.add_argument("--scenarios", default=",".join(F_SCENARIOS))
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--out", required=True)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--dwa-args", nargs="*", default=[])
    parser.add_argument("--calib", default="S0",
                        choices=["S0", "S1", "S2", "S3"],
                        help="Phase 10 calibration level. S0 is the "
                             "historical simulator; S1 adds the measured "
                             "observation model, S2 its timing and burst "
                             "structure, S3 the vehicle profile.")
    parser.add_argument(
        "--resume", action="store_true",
        help="continue an interrupted campaign: keep every completed run, "
             "execute only planned tuples without a valid outcome. Does not "
             "change any experimental parameter.")
    args = parser.parse_args()
    global DURATION_S, CALIB_ARGS, CALIB_LEVEL
    CALIB_ARGS = calibration_args(args.calib)
    CALIB_LEVEL = args.calib.upper().strip()
    for f in (S1_FIT if args.calib != "S0" else None,
              S2_FIT if args.calib in ("S2", "S3") else None):
        if f and not os.path.isfile(f):
            raise SystemExit("missing calibration fit: %s" % f)
    if args.duration:
        DURATION_S = args.duration

    planners = [p.strip() for p in args.planners.split(",") if p.strip()]
    scenarios = [s.strip().upper() for s in args.scenarios.split(",")
                 if s.strip()]
    os.makedirs(args.out, exist_ok=True)
    results = load_prior(args.out) if args.resume else []
    done = {(r["scenario"], r["planner"], r["run"])
            for r in results if has_valid_outcome(r)}
    # A tuple that exists only as a technical-invalid record still owes a
    # valid outcome: drop the stale record so the re-run replaces it.
    if args.resume:
        stale = [(r["scenario"], r["planner"], r["run"]) for r in results
                 if not has_valid_outcome(r)]
        if stale:
            print(f"[planner8] resume: {len(stale)} tuple(s) without a valid "
                  f"outcome will be re-run: {sorted(set(stale))}", flush=True)
        results = [r for r in results if has_valid_outcome(r)]
        print(f"[planner8] resume: {len(done)} tuple(s) already complete",
              flush=True)
    t0 = time.time()
    total = len(planners) * len(scenarios) * args.runs
    n = 0
    for sc in scenarios:
        # Alternate planners inside the scenario block (protocol section 4)
        # so drift in engine load affects both planners symmetrically.
        for i in range(1, args.runs + 1):
            for planner in planners:
                n += 1
                if (sc, planner, i) in done:
                    print(f"[planner8] --- {n}/{total}: {sc} {planner} run "
                          f"{i} already complete, skipping ---", flush=True)
                    continue
                print(f"[planner8] === {n}/{total}: {sc} {planner} run {i} "
                      "===", flush=True)
                if args.resume:
                    preserve_partial(os.path.join(args.out, "runs",
                                                  f"{sc}_{planner}_{i}"))
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
                write_manifest(os.path.join(args.out, "manifest.json"), {
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
                    "resumed": bool(args.resume),
                    "results": results,
                })
    print(f"[planner8] campaign done -> {args.out}/manifest.json", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
