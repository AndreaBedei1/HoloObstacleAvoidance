"""Post-deployment diagnostic: reintroduce each real-session change alone.

NOT A PREDICTION. Every campaign this script runs happened AFTER the real
runs were flown and after their outcome was known. None of it may be
reported as pre-registered. The 80 frozen runs are not touched.

WHAT IT ASKS. A stack developed against simulated perception did not work
on the vehicle, and a specific list of changes was made to it in the pool
until it did. Which of those changes actually mattered cannot be read off
the real runs -- they were all changed together, in one morning, with no
repetitions of the intermediate states. Simulation can do what the pool
cannot: apply them ONE AT A TIME, with repetitions, everything else held
at its frozen value.

THE LADDER. Each rung adds one documented change to the frozen
configuration; the last rung is all of them together, which is the
configuration the vehicle actually flew.

    A0  frozen configuration, as the 80 predictions ran it
    A1  + qualifier relaxed          warmup 20->2, confirm 3->1
    A2  + risk thresholds lowered    0.55/0.30 -> 0.30/0.15
    A3  + avoidance hold lengthened  1.0 -> 4.0 s
    A4  + engagement distance        1.5 -> 1.8 m
    A5  + relay vfov corrected       60 -> 90 deg
    A6  + pool-feasible manoeuvre    offset 2.5->0.6 m, pass 4.0->1.0 m
    A7  = all of the above

A4 is 1.8 m and not the 5.0 m that config/real_session_20260816.yaml
records, because 1.8 m is what the vehicle actually ran: the runner's
wrapper forwarded only three arguments, so the engagement distance never
left its default. Every run's result.json says 1.8. The session file is
wrong on this point and is corrected rather than followed.

A5 is not a real-session change at all -- it is a defect found in the
frozen campaign during the final audit. The relay decides whether a
detection survives by converting apparent height to range at 60 degrees
while the oracle projects and the planner invert at 90, so the measured
detection-probability curve was applied about 1.5x too far out in the 60
runs at S1 and above. It is included here to measure how much that cost.

A6 is not a parameter anyone changed on the day. It is included because
the frozen planner asks for a 2.5 m lateral offset and a 4 m run past the
obstacle, and the pool is 2.7-3.0 m wide with the anchor 1.86 m from the
start: the real vehicle was asked for a manoeuvre the basin could not
contain, and one run ended against the wall. A6 asks what the frozen
planner would have done had it been given a manoeuvre that fits.

Usage (sourced ROS env):
    python scripts/run_deployment_gap_ablation.py [--runs 5] [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

QUALIFIER = ["warmup_min_updates:=2", "confirm_min_updates:=1"]
RISK = ["risk_enter_threshold:=0.30", "risk_exit_threshold:=0.15"]
HOLD = ["min_avoidance_hold_s:=4.0"]
ENGAGE = ["engage_distance_m:=1.8"]
VFOV = ["relay_vfov_deg:=90.0"]
# 0.6 m of lateral offset and 1.0 m of run-past: the widest manoeuvre that
# fits a 2.7 m basin with a 0.3 m vehicle radius and keeps the far wall at
# arm's length, rather than a round number chosen for looks.
GEOMETRY = ["clearance_offset_m:=0.6", "pass_margin_m:=1.0"]

LADDER = [
    ("A0_frozen", [], "frozen configuration"),
    ("A1_qualifier", QUALIFIER, "qualifier relaxed"),
    ("A2_risk", RISK, "risk thresholds lowered"),
    ("A3_hold", HOLD, "avoidance hold lengthened"),
    ("A4_engage", ENGAGE, "engagement distance as flown"),
    ("A5_vfov", VFOV, "relay vfov corrected"),
    ("A6_geometry", GEOMETRY, "pool-feasible manoeuvre geometry"),
    ("A7_deployed", QUALIFIER + RISK + HOLD + ENGAGE + VFOV + GEOMETRY,
     "everything together: the configuration that flew"),
]


def campaign(out, scenarios, planners, runs, extra, calib, dry):
    cmd = [sys.executable, os.path.join(REPO, "scripts",
                                        "run_planner_campaign.py"),
           "--planners", planners, "--scenarios", scenarios,
           "--runs", str(runs), "--out", out, "--calib", calib]
    if extra:
        cmd += ["--extra-args"] + extra
    print("\n>>> " + " ".join(cmd), flush=True)
    if dry:
        return 0
    t0 = time.time()
    r = subprocess.run(cmd, cwd=REPO)
    print("<<< uscita %d in %.0f s" % (r.returncode, time.time() - t0),
          flush=True)
    return r.returncode


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--calib", default="S3")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--only", default="",
                    help="comma-separated rung names to run")
    a = ap.parse_args()

    base = os.path.join(REPO, "experiments", "simulation", "diagnostic")
    os.makedirs(base, exist_ok=True)
    only = {s.strip() for s in a.only.split(",") if s.strip()}

    index = []
    for name, extra, desc in LADDER:
        if only and name not in only:
            continue
        out = os.path.join(base, "ablation_" + name)
        rc = campaign(out, "K0", "committed", a.runs, extra, a.calib,
                      a.dry_run)
        index.append({"rung": name, "description": desc,
                      "extra_args": extra, "calibration": a.calib,
                      "scenario": "K0", "planner": "committed",
                      "runs": a.runs, "out": os.path.relpath(out, REPO)
                      .replace("\\", "/"), "returncode": rc})

    p = os.path.join(base, "ablation_index.json")
    with open(p, "w") as f:
        json.dump({
            "category": "post-deployment diagnostic simulation",
            "pre_registered": False,
            "note": ("Run after the real campaign, with its outcome known. "
                     "Not a prediction and not comparable to the frozen "
                     "80 as if it were one."),
            "ladder": index,
        }, f, indent=1)
    print("\n->", os.path.relpath(p, REPO).replace("\\", "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
