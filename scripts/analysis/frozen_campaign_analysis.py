"""Analyse the 80 frozen pre-real simulated runs.

These runs were executed and hashed BEFORE any real run and are never
modified. This script only reads them.

WHAT THESE RUNS ARE. Four calibration levels (S0 historical simulator,
S1 + measured static observation model, S2 + measured timing and burst
structure, S3 + measured vehicle response downstream of the planner),
two planners (committed, DWA), two geometries (K0 centred, K1 offset
0.35 m), five repetitions: 80 runs. Perception is an ORACLE relay
degraded by the measured statistics, not a visual detector -- the
recorded perception_source says so in every run, and the distinction
matters for every claim made from these numbers.

WHAT THEY CAN AND CANNOT SUPPORT. They are a controlled study of how a
closed loop responds to progressively more realistic mismatch IN
SIMULATION. They are not, on their own, evidence that any level is a
better predictor of reality: that would need a matched real campaign of
the same stack, which does not exist. Read the headline as "behaviour
under mismatch", never as "predictive validity".

Usage:
    python scripts/analysis/frozen_campaign_analysis.py
"""

from __future__ import annotations

import json
import os
import sys
from collections import defaultdict

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

LEVELS = ["S0", "S1", "S2", "S3"]
PLANNERS = ["committed", "dwa"]
GEOMS = ["K0", "K1"]

# Metrics taken straight from the stored validation, with the name the
# paper will use. "None" is a real value for several of them -- a run
# that never manoeuvred has no commitment distance, and recording that
# as zero would put a fictitious 0 m into a median.
METRICS = [
    ("min_clearance_m", "minimum clearance", "m"),
    ("lateral_commit_dist_m", "commitment distance", "m"),
    ("max_lateral_deviation_m", "maximum lateral excursion", "m"),
    ("path_length_m", "path length", "m"),
    ("lateral_maneuver_s", "manoeuvre span", "s"),
    ("maneuver_time_s", "active manoeuvre time", "s"),
    ("forward_progress_m", "forward progress", "m"),
    ("avoidance_entries", "avoidance entries", ""),
    ("side_switches", "side switches", ""),
    ("distance_at_first_planner_valid_m", "range at first qualified obs", "m"),
    ("confirmation_delay_s", "qualification delay", "s"),
    ("lateral_peak_m_s", "commanded lateral peak", "m/s"),
]


def load_all():
    rows = []
    for lvl in LEVELS:
        base = os.path.join(_ROOT, "experiments", "simulation",
                            "phase10_" + lvl, "runs")
        if not os.path.isdir(base):
            continue
        for name in sorted(os.listdir(base)):
            p = os.path.join(base, name, "validation.json")
            if not os.path.exists(p):
                continue
            with open(p) as f:
                v = json.load(f)
            geom, planner, idx = name.rsplit("_", 2)
            rec = {"level": lvl, "geometry": geom, "planner": planner,
                   "run": int(idx), "dir": name}
            for k, _lab, _u in METRICS:
                rec[k] = v.get(k)
            rec["collision"] = bool(v.get("collision"))
            rec["infra_freeze"] = bool(v.get("infra_freeze_detected"))
            rec["cmd_path_dead"] = bool(v.get("cmd_path_dead_detected"))
            rec["maneuvered"] = rec.get("lateral_commit_dist_m") is not None
            rec["elapsed_s"] = v.get("elapsed_s")
            rec["perception_source"] = v.get("perception_source")
            rows.append(rec)
    return rows


def med(vals):
    vals = [v for v in vals if v is not None]
    return float(np.median(vals)) if vals else None


def iqr(vals):
    vals = [v for v in vals if v is not None]
    if len(vals) < 2:
        return None
    return (float(np.percentile(vals, 25)), float(np.percentile(vals, 75)))


def fmt(x, nd=3):
    return "-" if x is None else ("%.*f" % (nd, x))


def main() -> int:
    rows = load_all()
    print("run caricati: %d" % len(rows))
    if not rows:
        print("nessun run trovato")
        return 2

    # ---- integrity -------------------------------------------------------
    cells = defaultdict(list)
    for r in rows:
        cells[(r["level"], r["geometry"], r["planner"])].append(r)
    bad = [(k, len(v)) for k, v in sorted(cells.items()) if len(v) != 5]
    print("celle con conteggio diverso da 5: %s" % (bad or "nessuna"))
    print("collisioni: %d | infra_freeze: %d | cmd_path_dead: %d"
          % (sum(r["collision"] for r in rows),
             sum(r["infra_freeze"] for r in rows),
             sum(r["cmd_path_dead"] for r in rows)))
    srcs = {r["perception_source"] for r in rows}
    print("perception_source: %s" % srcs)
    print("run che NON hanno mai manovrato: %d"
          % sum(1 for r in rows if not r["maneuvered"]))

    # ---- per-cell medians ------------------------------------------------
    table = {}
    for (lvl, geom, pl), v in cells.items():
        e = {}
        for k, _lab, _u in METRICS:
            e[k] = med([r[k] for r in v])
            e[k + "_iqr"] = iqr([r[k] for r in v])
        e["n_maneuvered"] = sum(1 for r in v if r["maneuvered"])
        e["n_collision"] = sum(1 for r in v if r["collision"])
        table["%s|%s|%s" % (lvl, geom, pl)] = e

    for k, lab, unit in METRICS:
        print("\n=== %s (%s) — mediana su 5 ripetizioni ===" % (lab, unit))
        print("%-10s %-10s %8s %8s %8s %8s" % ("geometria", "planner",
                                               *LEVELS))
        for geom in GEOMS:
            for pl in PLANNERS:
                vals = [table["%s|%s|%s" % (lv, geom, pl)][k]
                        for lv in LEVELS]
                print("%-10s %-10s %8s %8s %8s %8s"
                      % (geom, pl, *[fmt(x) for x in vals]))

    # ---- how many repetitions manoeuvred at all --------------------------
    print("\n=== ripetizioni che hanno manovrato (su 5) ===")
    print("%-10s %-10s %8s %8s %8s %8s" % ("geometria", "planner", *LEVELS))
    for geom in GEOMS:
        for pl in PLANNERS:
            vals = [table["%s|%s|%s" % (lv, geom, pl)]["n_maneuvered"]
                    for lv in LEVELS]
            print("%-10s %-10s %8d %8d %8d %8d" % (geom, pl, *vals))

    out = os.path.join(_ROOT, "experiments", "simulation",
                       "phase10_predictions", "frozen_analysis.json")
    with open(out, "w") as f:
        json.dump({
            "note": ("Analysis of the 80 frozen pre-real runs. The runs "
                     "themselves are unmodified; this file is derived."),
            "perception_source": sorted(srcs),
            "design": {"levels": LEVELS, "planners": PLANNERS,
                       "geometries": GEOMS, "repetitions": 5},
            "runs": rows,
            "cell_medians": table,
        }, f, indent=1)
    print("\n->", os.path.relpath(out, _ROOT).replace("\\", "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
