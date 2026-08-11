"""Phase 8 planner-comparison aggregation (pre-registered analysis).

Reads one or more campaign manifests produced by
`scripts/run_planner_campaign.py` and emits the per-scenario comparison
tables defined in docs/PHASE8_DWA_CAMPAIGN_PROTOCOL.md §7: exact
collision counts, success proportions with 95% Wilson CIs, median + IQR
for continuous metrics, and Mann-Whitney U (normal approximation with tie
correction) with Holm correction across scenarios. Technical-invalid runs
(protocol §6) are excluded and counted separately; nothing else is
excluded.

Self-contained (no scipy/numpy) so it runs under any python.

Usage:
    python scripts/aggregate_planner_campaign.py <manifest.json ...> \
        [--out results.json]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import defaultdict

CONTINUOUS = [
    "min_clearance_m", "max_lateral_deviation_m", "path_length_m",
    "forward_progress_m", "maneuver_time_s", "thruster_peak_n",
    "thruster_mean_abs_n", "odo_err_max_m",
]
SUCCESS_RETURN_TOL_M = 1.0


def technical_invalid(run: dict) -> str | None:
    if run.get("technical_invalid"):
        return run.get("error", "technical_invalid")
    m = run.get("metrics")
    if not m:
        return run.get("error", "validator output missing")
    if m.get("infra_freeze_detected"):
        return "infra_freeze"
    if m.get("cmd_path_dead_detected"):
        return "cmd_path_dead"
    return None


def success(m: dict, obstacle_x: float | None) -> bool:
    if m.get("collision"):
        return False
    fp = m.get("forward_progress_m") or 0.0
    if obstacle_x is not None and fp <= obstacle_x:
        return False
    lat = m.get("final_lateral_error_m")
    if lat is None:
        return bool(m.get("returned_to_original_line"))
    return abs(lat) <= SUCCESS_RETURN_TOL_M


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def mann_whitney(a: list[float], b: list[float]) -> float | None:
    """Two-sided p, normal approximation with tie correction."""
    na, nb = len(a), len(b)
    if na < 2 or nb < 2:
        return None
    allv = sorted((v, 0) for v in a) + sorted((v, 1) for v in b)
    allv.sort(key=lambda t: t[0])
    ranks: dict[int, float] = {}
    i = 0
    tie_term = 0.0
    while i < len(allv):
        j = i
        while j < len(allv) and allv[j][0] == allv[i][0]:
            j += 1
        r = (i + j + 1) / 2.0
        for k in range(i, j):
            ranks[k] = r
        t = j - i
        tie_term += t ** 3 - t
        i = j
    ra = sum(ranks[i] for i, (_, g) in enumerate(allv) if g == 0)
    u = ra - na * (na + 1) / 2.0
    n = na + nb
    mu = na * nb / 2.0
    sig2 = na * nb / 12.0 * ((n + 1) - tie_term / (n * (n - 1)))
    if sig2 <= 0:
        return 1.0
    zval = (u - mu - (0.5 if u > mu else -0.5)) / math.sqrt(sig2)
    return math.erfc(abs(zval) / math.sqrt(2.0))


def med_iqr(vals: list[float]) -> dict | None:
    vs = sorted(v for v in vals if v is not None)
    if not vs:
        return None

    def q(p: float) -> float:
        i = p * (len(vs) - 1)
        lo = int(math.floor(i))
        hi = min(lo + 1, len(vs) - 1)
        return vs[lo] + (vs[hi] - vs[lo]) * (i - lo)

    return {"median": round(q(0.5), 3), "q1": round(q(0.25), 3),
            "q3": round(q(0.75), 3), "n": len(vs)}


def holm(pairs: list[tuple[str, float]]) -> dict[str, float]:
    """Holm step-down adjusted p-values keyed by scenario."""
    srt = sorted((p, s) for s, p in pairs if p is not None)
    out: dict[str, float] = {}
    m = len(srt)
    running = 0.0
    for i, (p, s) in enumerate(srt):
        adj = min(1.0, (m - i) * p)
        running = max(running, adj)
        out[s] = round(running, 5)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("manifests", nargs="+")
    ap.add_argument("--out", default=None)
    ap.add_argument("--obstacle-x", type=float, default=None,
                    help="forward coordinate of the obstacle plane used in "
                         "the success criterion (default: from scenario "
                         "name, 11.0 for F*, 3.5 for K*)")
    args = ap.parse_args()

    runs = []
    for mf in args.manifests:
        with open(mf) as f:
            data = json.load(f)
        runs.extend(data.get("results", []))

    by_cell = defaultdict(list)
    invalid = defaultdict(int)
    for r in runs:
        cell = (r.get("scenario"), r.get("planner"))
        why = technical_invalid(r)
        if why:
            invalid[(cell, why)] += 1
            continue
        by_cell[cell].append(r["metrics"])

    scenarios = sorted({s for s, _ in by_cell})
    planners = sorted({p for _, p in by_cell})
    report = {"scenarios": {}, "technical_invalid": {
        f"{s}/{p}: {w}": n for ((s, p), w), n in invalid.items()}}
    mwu_by_metric: dict[str, list[tuple[str, float]]] = defaultdict(list)

    for sc in scenarios:
        ox = args.obstacle_x
        if ox is None:
            ox = 3.5 if sc.startswith("K") else 11.0
        row: dict = {}
        for pl in planners:
            ms = by_cell.get((sc, pl), [])
            n = len(ms)
            coll = sum(1 for m in ms if m.get("collision"))
            succ = sum(1 for m in ms if success(m, ox))
            lo, hi = wilson(succ, n)
            row[pl] = {
                "n": n, "collisions": coll, "successes": succ,
                "success_rate": round(succ / n, 3) if n else None,
                "success_ci95": [round(lo, 3), round(hi, 3)],
                "metrics": {k: med_iqr([m.get(k) for m in ms])
                            for k in CONTINUOUS},
            }
        if len(planners) == 2:
            pa, pb = planners
            row["mwu_p_raw"] = {}
            for k in CONTINUOUS:
                a = [m.get(k) for m in by_cell.get((sc, pa), [])
                     if m.get(k) is not None]
                b = [m.get(k) for m in by_cell.get((sc, pb), [])
                     if m.get(k) is not None]
                p = mann_whitney(a, b)
                row["mwu_p_raw"][k] = round(p, 5) if p is not None else None
                mwu_by_metric[k].append((sc, p))
        report["scenarios"][sc] = row

    report["mwu_p_holm_across_scenarios"] = {
        k: holm(v) for k, v in mwu_by_metric.items()}

    text = json.dumps(report, indent=2)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)),
                    exist_ok=True)
        with open(args.out, "w") as f:
            f.write(text)
        print(f"[aggregate] -> {args.out}")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
