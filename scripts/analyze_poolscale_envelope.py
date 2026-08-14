"""Pool-scale workspace envelope analysis (protocol section 5 addendum).

Computes, from the GROUND-TRUTH trace of each run, the physical workspace an
avoidance maneuver actually consumes, so that pool feasibility can be decided
from measurement instead of intuition:

    longitudinal workspace   max(gt_dx) - min(gt_dx)  (+ vehicle radius)
    lateral workspace        max(gt_dy) - min(gt_dy)  (+ vehicle diameter)
    start-to-obstacle        obstacle x in the route frame
    avoidance width          peak |gt_dy| excursion
    recovery length          along-track distance from the obstacle plane to
                             the point where |gt_dy| settles below a threshold
    min clearance            from the validator

The route frame has dx along the nominal route and dy across it, with the
start at the origin, so these are directly the dimensions a pool must supply.

IMPORTANT: this script never invents pool dimensions. It reports the REQUIRED
workspace unconditionally. Pass --pool-length / --pool-width only once the
pool has actually been surveyed; without them the feasibility columns read
"unmeasured" rather than guessing.

This is deliberately a SEPARATE script: scripts/aggregate_planner_campaign.py
was committed before the campaign as the pre-registered analysis and is not
modified.

Usage:
    python scripts/analyze_poolscale_envelope.py <manifest.json> [...] \
        [--scenarios K0,K1] [--pool-length L] [--pool-width W] [--out FILE]
"""

from __future__ import annotations

import argparse
import json
import statistics
from collections import defaultdict

# Circumscribed BlueROV2 Heavy footprint used throughout the project.
VEHICLE_RADIUS_M = 0.40
# |lateral| below this counts as "returned to the route" for recovery length.
RETURN_TOL_M = 0.30


def envelope(metrics: dict) -> dict | None:
    """Workspace envelope of a single run from its GT trace."""
    series = metrics.get("odo_series") or []
    if not series:
        return None
    dx = [p["gt_dx"] for p in series if p.get("gt_dx") is not None]
    dy = [p["gt_dy"] for p in series if p.get("gt_dy") is not None]
    if not dx or not dy:
        return None

    geom = (metrics.get("obstacle_geometry") or [{}])[0]
    obstacle_dx = geom.get("x")
    obstacle_r = geom.get("radius_m")

    peak_i = max(range(len(dy)), key=lambda i: abs(dy[i]))
    peak_lat = dy[peak_i]

    # Recovery: first sample AFTER the lateral peak whose |dy| is back inside
    # the tolerance. Measured from the obstacle plane, not from the peak, so
    # it answers "how much pool do I need downstream of the obstacle".
    recovery_dx = None
    for i in range(peak_i, len(dy)):
        if abs(dy[i]) <= RETURN_TOL_M:
            recovery_dx = dx[i]
            break
    recovery_len = (None if recovery_dx is None or obstacle_dx is None
                    else round(recovery_dx - obstacle_dx, 3))

    return {
        "longitudinal_span_m": round(max(dx) - min(dx), 3),
        "longitudinal_required_m": round(max(dx) - min(dx)
                                         + 2 * VEHICLE_RADIUS_M, 3),
        "lateral_span_m": round(max(dy) - min(dy), 3),
        "lateral_required_m": round(max(dy) - min(dy)
                                    + 2 * VEHICLE_RADIUS_M, 3),
        "start_to_obstacle_m": obstacle_dx,
        "obstacle_radius_m": obstacle_r,
        "avoidance_width_m": round(abs(peak_lat), 3),
        "avoidance_side": "left" if peak_lat > 0 else "right",
        "recovery_length_m": recovery_len,
        "returned_within_tol": recovery_dx is not None,
        "min_clearance_m": metrics.get("min_clearance_m"),
        "collision": metrics.get("collision"),
        "max_dx_m": round(max(dx), 3),
    }


def summarize(vals: list[float]) -> dict | None:
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return {
        "n": len(vals),
        "median": round(statistics.median(vals), 3),
        "min": round(min(vals), 3),
        "max": round(max(vals), 3),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("manifests", nargs="+")
    ap.add_argument("--scenarios", default="K0,K1")
    ap.add_argument("--pool-length", type=float, default=None,
                    help="MEASURED pool length [m]; omit if not yet surveyed")
    ap.add_argument("--pool-width", type=float, default=None,
                    help="MEASURED pool width [m]; omit if not yet surveyed")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    want = {s.strip().upper() for s in args.scenarios.split(",") if s.strip()}
    runs = []
    for mf in args.manifests:
        with open(mf) as f:
            runs.extend(json.load(f).get("results", []))

    by_cell = defaultdict(list)
    skipped = 0
    for r in runs:
        if r.get("scenario") not in want:
            continue
        if not r.get("ok") or r.get("technical_invalid"):
            skipped += 1
            continue
        env = envelope(r.get("metrics") or {})
        if env is None:
            skipped += 1
            continue
        env["run"] = r.get("run")
        by_cell[(r["scenario"], r["planner"])].append(env)

    report: dict = {"scenarios_requested": sorted(want),
                    "excluded_runs": skipped,
                    "vehicle_radius_m": VEHICLE_RADIUS_M,
                    "return_tolerance_m": RETURN_TOL_M,
                    "cells": {}}

    worst_long, worst_lat = {}, {}
    for (sc, pl), envs in sorted(by_cell.items()):
        cell = {
            "n": len(envs),
            "longitudinal_required_m": summarize(
                [e["longitudinal_required_m"] for e in envs]),
            "lateral_required_m": summarize(
                [e["lateral_required_m"] for e in envs]),
            "avoidance_width_m": summarize(
                [e["avoidance_width_m"] for e in envs]),
            "recovery_length_m": summarize(
                [e["recovery_length_m"] for e in envs]),
            "min_clearance_m": summarize(
                [e["min_clearance_m"] for e in envs]),
            "start_to_obstacle_m": envs[0]["start_to_obstacle_m"],
            "collisions": sum(1 for e in envs if e["collision"]),
            "returned_within_tol": sum(1 for e in envs
                                       if e["returned_within_tol"]),
            "runs": envs,
        }
        report["cells"][f"{sc}/{pl}"] = cell
        lo = cell["longitudinal_required_m"]
        la = cell["lateral_required_m"]
        if lo:
            worst_long[pl] = max(worst_long.get(pl, 0.0), lo["max"])
        if la:
            worst_lat[pl] = max(worst_lat.get(pl, 0.0), la["max"])

    feas = {}
    for pl in sorted(set(worst_long) | set(worst_lat)):
        entry = {
            "worst_case_longitudinal_required_m": round(
                worst_long.get(pl, 0.0), 3),
            "worst_case_lateral_required_m": round(worst_lat.get(pl, 0.0), 3),
        }
        if args.pool_length is None:
            entry["fits_pool_length"] = "unmeasured -- survey the pool first"
        else:
            margin = args.pool_length - worst_long.get(pl, 0.0)
            entry["pool_length_m"] = args.pool_length
            entry["longitudinal_margin_m"] = round(margin, 3)
            entry["fits_pool_length"] = margin > 0
        if args.pool_width is None:
            entry["fits_pool_width"] = "unmeasured -- survey the pool first"
        else:
            margin = args.pool_width - worst_lat.get(pl, 0.0)
            entry["pool_width_m"] = args.pool_width
            entry["lateral_margin_m"] = round(margin, 3)
            entry["fits_pool_width"] = margin > 0
        feas[pl] = entry
    report["feasibility"] = feas
    report["note"] = ("Required workspace is measured; pool fit is reported "
                      "only against MEASURED pool dimensions supplied on the "
                      "command line. No pool dimension is assumed.")

    text = json.dumps(report, indent=2)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
        print(f"[poolscale] wrote {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
