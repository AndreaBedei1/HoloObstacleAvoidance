"""Aggregate the Phase-7 closed-loop campaign into tables + figures.

Reads one or more campaign manifests (main + optional patch reruns; later
entries override earlier ones for the same scenario/method/run key), flags
technically-invalid runs (dead estimator = estimator_debug_msgs 0/None),
and writes:

  aggregate.json, aggregate.md, fig_closedloop_comparison.png

Usage:
    python scripts/analyze_temporal_campaign.py \
        --manifests experiments/simulation/temporal_estimators/closedloop/manifest.json \
                    [experiments/.../patch/manifest.json ...]
"""

from __future__ import annotations

import argparse
import json
import os
from collections import defaultdict

METHODS = ["t0", "t1", "t2", "t3"]
SCENARIOS = ["E0", "E1", "E2", "E3", "E4"]


def load_runs(manifest_paths):
    runs = {}
    for path in manifest_paths:
        with open(path) as f:
            m = json.load(f)
        for r in m["results"]:
            key = (r["scenario"], r["method"], r["run"])
            runs[key] = r          # later manifests override (patch reruns)
    return runs


def is_technical_invalid(r):
    if r.get("technical_invalid"):
        return True
    mm = r.get("metrics") or {}
    if not mm:
        return True
    # Estimator never alive (e.g. crashed node) => run had no perception path.
    if not mm.get("estimator_debug_msgs"):
        return True
    return False


def outcome(r):
    a = r.get("assessment") or {}
    ok = (a.get("collision") is False
          and (a.get("min_clearance_m") or -1) > 0.3
          and a.get("returned") is True
          and (a.get("side_switches") or 0) == 0
          and not a.get("maneuver_aborted_to_normal"))
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifests", nargs="+", required=True)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    out_dir = args.out or os.path.dirname(os.path.abspath(args.manifests[0]))

    runs = load_runs(args.manifests)
    table = defaultdict(dict)
    details = []
    for sc in SCENARIOS:
        for me in METHODS:
            sel = [r for (s, m, _), r in sorted(runs.items())
                   if s == sc and m == me]
            valid = [r for r in sel if not is_technical_invalid(r)]
            succ = [r for r in valid if outcome(r)]
            aborted = [r for r in valid
                       if (r.get("assessment") or {}).get(
                           "maneuver_aborted_to_normal")]
            clr = [a for r in valid
                   if (a := (r.get("assessment") or {}).get(
                       "min_clearance_m")) is not None]
            flat = [abs(a) for r in valid
                    if (a := (r.get("assessment") or {}).get(
                        "final_lateral_m")) is not None]
            table[sc][me] = {
                "runs": len(sel),
                "technical_invalid": len(sel) - len(valid),
                "success": f"{len(succ)}/{len(valid)}",
                "aborts": len(aborted),
                "collisions": sum(1 for r in valid
                                  if (r.get("assessment") or {}).get(
                                      "collision")),
                "reengage_total": sum((r.get("assessment") or {}).get(
                    "reengagements") or 0 for r in valid),
                "min_clearance_min": round(min(clr), 3) if clr else None,
                "final_lat_worst": round(max(flat), 3) if flat else None,
            }
            for r in sel:
                details.append({
                    "key": f"{sc}/{me}/{r['run']}",
                    "technical_invalid": is_technical_invalid(r),
                    "success": outcome(r) if not is_technical_invalid(r)
                               else None,
                    **(r.get("assessment") or {}),
                })

    agg = {"table": {k: dict(v) for k, v in table.items()},
           "details": details}
    with open(os.path.join(out_dir, "aggregate.json"), "w") as f:
        json.dump(agg, f, indent=2)

    lines = ["# Phase 7 closed-loop campaign — aggregate",
             "",
             "success = no collision AND clearance>0.3 AND returned AND no "
             "side switch AND no maneuver abort; technical-invalid runs "
             "(dead estimator/graph) excluded and counted separately.",
             ""]
    header = "| Scenario | " + " | ".join(METHODS) + " |"
    lines += [header, "|---" * (len(METHODS) + 1) + "|"]
    for sc in SCENARIOS:
        cells = []
        for me in METHODS:
            c = table[sc].get(me, {})
            cell = (f"{c.get('success','-')} ok"
                    f"{', ' + str(c.get('aborts')) + ' abort' if c.get('aborts') else ''}"
                    f"{', ' + str(c.get('collisions')) + ' COLL' if c.get('collisions') else ''}"
                    f"{', ' + str(c.get('technical_invalid')) + ' techinv' if c.get('technical_invalid') else ''}")
            cells.append(cell)
        lines.append(f"| {sc} | " + " | ".join(cells) + " |")
    with open(os.path.join(out_dir, "aggregate.md"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))

    try:
        _plot(table, out_dir)
    except Exception as exc:
        print("plot skipped:", exc)
    print("wrote", out_dir)
    return 0


def _plot(table, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    fig, ax = plt.subplots(figsize=(11, 5))
    x = np.arange(len(SCENARIOS))
    width = 0.2
    for i, me in enumerate(METHODS):
        vals = []
        for sc in SCENARIOS:
            c = table[sc].get(me, {})
            s = c.get("success", "0/0").split("/")
            vals.append(int(s[0]) / max(1, int(s[1])))
        ax.bar(x + (i - 1.5) * width, vals, width, label=me)
    ax.set_xticks(x)
    ax.set_xticklabels(SCENARIOS)
    ax.set_ylabel("success rate (valid runs)")
    ax.set_ylim(0, 1.05)
    ax.grid(alpha=0.3, axis="y")
    ax.legend()
    ax.set_title("Closed-loop success by scenario and temporal estimator "
                 "(oracle perception, BlueROV2 dynamics)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig_closedloop_comparison.png"),
                dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
