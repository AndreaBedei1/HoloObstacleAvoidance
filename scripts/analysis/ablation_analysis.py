"""Analyse the post-deployment deployment-gap ablation, and emit its table.

POST-HOC. Every run this reads was executed after the physical campaign,
with its outcome known. Nothing here is a prediction and nothing here is
compared against the 80 frozen runs, for the reason given in the paper:
the frozen campaign is not reproducible, so its numbers cannot serve as a
baseline for anything run later. The ablation's own A0 rung, executed in
the same session on the same machine, is the baseline.

The outcome is bimodal -- a run either completes an avoidance or wanders
-- so the reporting is medians with interquartile ranges plus the count
of runs that manoeuvred at all, never means. A mean over a bimodal
distribution describes a state the system never occupies.

Usage:
    python scripts/analysis/ablation_analysis.py [--latex]
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
DIAG = os.path.join(_ROOT, "experiments", "simulation", "diagnostic")

RUNGS = [
    ("A0_frozen", "A0", "frozen"),
    ("A1_qualifier", "A1", "qualifier"),
    ("A2_risk", "A2", "risk thresholds"),
    ("A3_hold", "A3", "hold"),
    ("A4_engage", "A4", "engagement"),
    ("A5_vfov", "A5", "relay FOV fix"),
    ("A6_geometry", "A6", "tested pool rescaling"),
    ("A7_deployed", "A7", "combined diagnostic"),
]


def load(dirname):
    base = os.path.join(DIAG, "ablation_" + dirname, "runs")
    rows = []
    if not os.path.isdir(base):
        return rows
    for name in sorted(os.listdir(base)):
        p = os.path.join(base, name, "validation.json")
        if not os.path.exists(p):
            continue
        try:
            with open(p) as f:
                v = json.load(f)
        except (OSError, ValueError):
            continue
        tr = v.get("cmd_trace") or []
        peak_lat = max((abs(r["y"]) for r in tr), default=0.0)
        rows.append({
            "run": name,
            "clearance": v.get("min_clearance_m"),
            "collision": bool(v.get("collision")),
            "commit": v.get("lateral_commit_dist_m"),
            "lat_dev": v.get("max_lateral_deviation_m"),
            "path": v.get("path_length_m"),
            "span": v.get("lateral_maneuver_s"),
            "entries": v.get("avoidance_entries"),
            "switches": v.get("side_switches"),
            "qual_range": v.get("distance_at_first_planner_valid_m"),
            "peak_lat": round(float(peak_lat), 4),
            # The strafe phase commands 0.20 m/s; only the go-around
            # phase reaches 0.30. This is the same observable the paper
            # uses on the physical runs, where no external reference
            # exists, so the two domains are read the same way.
            "reached_go_around": bool(peak_lat > 0.25),
            "maneuvered": v.get("lateral_commit_dist_m") is not None
                          or (v.get("max_lateral_deviation_m") or 0) > 0.1,
        })
    return rows


def q(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return None, None, None
    return (float(np.median(v)), float(np.percentile(v, 25)),
            float(np.percentile(v, 75)))


def fmt(m, lo, hi, nd=2):
    if m is None:
        return "\\nomeasure"
    return "%.*f \\tiny[%.*f--%.*f]" % (nd, m, nd, lo, nd, hi)


def main() -> int:
    latex = "--latex" in sys.argv
    table = []
    for dirname, tag, desc in RUNGS:
        rows = load(dirname)
        if not rows:
            print("%-14s nessun run" % tag)
            continue
        cl = q([r["clearance"] for r in rows])
        lat = q([r["lat_dev"] for r in rows])
        span = q([r["span"] for r in rows])
        qr = q([r["qual_range"] for r in rows])
        e = {
            "rung": tag, "description": desc, "n": len(rows),
            "clearance_median": cl[0], "clearance_iqr": [cl[1], cl[2]],
            "lat_dev_median": lat[0], "lat_dev_iqr": [lat[1], lat[2]],
            "span_median": span[0], "span_iqr": [span[1], span[2]],
            "qual_range_median": qr[0],
            "n_maneuvered": sum(1 for r in rows if r["maneuvered"]),
            "n_go_around": sum(1 for r in rows if r["reached_go_around"]),
            "n_collision": sum(1 for r in rows if r["collision"]),
        }
        table.append(e)
        print("%-4s n=%2d  clearance %s  lat_dev %s  manovrate %d/%d  "
              "aggiramento %d/%d  collisioni %d"
              % (tag, e["n"],
                 ("%.2f" % cl[0]) if cl[0] is not None else "-",
                 ("%.2f" % lat[0]) if lat[0] is not None else "-",
                 e["n_maneuvered"], e["n"], e["n_go_around"], e["n"],
                 e["n_collision"]))

    out = os.path.join(DIAG, "ablation_analysis.json")
    with open(out, "w") as f:
        json.dump({"category": "post-deployment diagnostic",
                   "pre_registered": False,
                   "baseline": "A0, run in the same session",
                   "rungs": table}, f, indent=1)
    print("\n->", os.path.relpath(out, _ROOT).replace("\\", "/"))

    if latex and table:
        tex = os.path.join(_ROOT, "paper", "robovis2027", "sections",
                           "08b_ablation_table.tex")
        with open(tex, "w") as f:
            f.write(render(table))
        print("->", os.path.relpath(tex, _ROOT).replace("\\", "/"))
    return 0


def render(table) -> str:
    lines = []
    lines.append("""
\\begin{table}[t]
\\centering
\\caption{Deployment-gap ablation \\diagmark, ten runs per configuration.
Continuous values are medians with interquartile ranges. \\emph{go-around}
counts runs reaching the second manoeuvre phase; Table~\\ref{tab:ablation-design}
defines each change.}
\\label{tab:ablation}
\\scriptsize
\\setlength{\\tabcolsep}{2.5pt}
\\begin{tabular}{llrrrrr}
\\toprule
rung & change & clear. (\\si{\\metre}) & lateral (\\si{\\metre})
& manoeuvred & go-around & coll. \\\\
\\midrule""")
    for e in table:
        lines.append(
            "%s & %s & %s & %s & %d/%d & %d/%d & %d \\\\"
            % (e["rung"], e["description"],
               fmt(e["clearance_median"], *e["clearance_iqr"]),
               fmt(e["lat_dev_median"], *e["lat_dev_iqr"]),
               e["n_maneuvered"], e["n"], e["n_go_around"], e["n"],
               e["n_collision"]))
    lines.append("""\\bottomrule
\\end{tabular}
\\end{table}
""")
    return "\n".join(lines)


if __name__ == "__main__":
    sys.exit(main())
