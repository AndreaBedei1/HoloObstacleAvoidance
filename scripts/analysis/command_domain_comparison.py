"""Compare simulation and reality where the two are actually comparable.

The command stream is recorded on the SAME topic in both domains,
/planner/cmd_vel_safe, at the frozen boundary. It is therefore the one
place where a simulated and a physical run measure the same thing with
the same instrument, and it is the only place this paper compares them
numerically.

Everything spatial is excluded by construction. The physical runs have no
external tracking, so simulated clearance, path length and lateral
excursion have no physical counterpart to be compared against, and
producing one by integrating the physical command stream is refused
elsewhere in this analysis and refused here.

WHAT IS AND IS NOT BEING ASKED. This is not a test of predictive
validity. The frozen simulated runs and the physical runs are not the
same stack -- seven documented differences separate them -- so agreement
would not prove the calibration worked and disagreement would not prove
it failed. The question is narrower and answerable: on the axis where
both domains measure the same signal, how far apart are they, and does
the distance change across the calibration levels?

Usage:
    python scripts/analysis/command_domain_comparison.py
"""

from __future__ import annotations

import json
import os
import sys
from collections import defaultdict

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

LATERAL_ACTIVE = 0.02        # m/s, "a lateral command is present"
COMMIT_THRESHOLD = 0.05
COMMIT_HOLD_S = 1.0


def commitment(t, y):
    for i in range(len(t)):
        if abs(y[i]) <= COMMIT_THRESHOLD:
            continue
        s = np.sign(y[i])
        j = i
        while j < len(t) and np.sign(y[j]) == s and abs(y[j]) > COMMIT_THRESHOLD:
            j += 1
        if t[j - 1] - t[i] >= COMMIT_HOLD_S:
            return ("right" if s > 0 else "left"), float(t[i])
    return None, None


def stats(t, x, y):
    lat = np.abs(y) > LATERAL_ACTIVE
    sg = np.sign(y[lat])
    side, tc = commitment(t, y)
    return {
        "duration_s": round(float(t[-1]), 2),
        "mean_surge": round(float(x.mean()), 4),
        "median_surge": round(float(np.median(x)), 4),
        "zero_surge_fraction": round(float((x < 0.01).mean()), 3),
        "lateral_fraction": round(float(lat.mean()), 3),
        "lateral_peak": round(float(np.abs(y).max()), 4),
        "reversals": int((np.diff(sg) != 0).sum()) if len(sg) > 1 else 0,
        "commit_side": side,
        "commit_time_s": None if tc is None else round(tc, 2),
        "commit_censored": bool(tc is not None and tc <= 0.0),
    }


def sim_runs():
    out = defaultdict(list)
    for lvl in ("S0", "S1", "S2", "S3"):
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
            tr = v.get("cmd_trace") or []
            if len(tr) < 5:
                continue
            t = np.array([r["t"] for r in tr], float)
            x = np.array([r["x"] for r in tr], float)
            y = np.array([r["y"] for r in tr], float)
            geom, planner, _ = name.rsplit("_", 2)
            out[(lvl, geom, planner)].append(stats(t, x, y))
    return out


def real_runs():
    p = os.path.join(_ROOT, "experiments", "real",
                     "campaign_provenance.json")
    with open(p) as f:
        d = json.load(f)
    out = defaultdict(list)
    for r in d["runs"]:
        if not r.get("admissible"):
            continue
        # The lateral PEAK is recomputed from the trace rather than taken
        # from the provenance file, because it is the discriminating
        # number of this comparison: the planner strafes at 0.20 m/s and
        # only reaches 0.30 m/s once it enters the go-around phase, so
        # the peak says whether the manoeuvre ever got that far.
        peak_lat = peak_surge = None
        tr = os.path.join(_ROOT, r["directory"], "cmd_trace_19hz.jsonl")
        if os.path.exists(tr):
            rows = [json.loads(x) for x in open(tr) if x.strip()]
            if rows:
                peak_lat = round(float(max(abs(q["y"]) for q in rows)), 4)
                peak_surge = round(float(max(q["x"] for q in rows)), 4)
        out[r["geometry"]].append({
            "lateral_peak": peak_lat,
            "surge_peak": peak_surge,
            "mean_surge": r.get("mean_commanded_surge_m_s"),
            "median_surge": r.get("median_commanded_surge_m_s"),
            "zero_surge_fraction": r.get("zero_surge_fraction"),
            "lateral_fraction": r.get("lateral_command_fraction"),
            "reversals": r.get("sign_reversals"),
            "commit_side": r.get("commitment_side"),
            "commit_time_s": r.get("commitment_time_s"),
            "commit_censored": r.get("commitment_time_left_censored"),
            "duration_s": r.get("trace_duration_s"),
        })
    return out


def med(rows, key):
    v = [r[key] for r in rows if r.get(key) is not None]
    return float(np.median(v)) if v else None


def main() -> int:
    sim, real = sim_runs(), real_runs()

    print("=== SIMULAZIONE CONGELATA, planner committed, dominio comandi ===")
    print("%-4s %-4s %3s %8s %9s %9s %8s %6s"
          % ("liv", "geo", "n", "surge", "lat_frac", "zero_frac",
             "picco", "inv"))
    for lvl in ("S0", "S1", "S2", "S3"):
        for geom in ("K0", "K1"):
            rows = sim.get((lvl, geom, "committed"), [])
            if not rows:
                continue
            print("%-4s %-4s %3d %8.4f %9.3f %9.3f %8.3f %6.1f"
                  % (lvl, geom, len(rows), med(rows, "mean_surge"),
                     med(rows, "lateral_fraction"),
                     med(rows, "zero_surge_fraction"),
                     med(rows, "lateral_peak"), med(rows, "reversals")))

    print("\n=== REALE, planner committed ===")
    print("%-4s %-4s %3s %8s %9s %9s %8s %6s"
          % ("", "geo", "n", "surge", "lat_frac", "zero_frac", "picco",
             "inv"))
    for geom in ("K0", "K1", "K1M"):
        rows = real.get(geom, [])
        if not rows:
            continue
        print("%-4s %-4s %3d %8.4f %9.3f %9.3f %8.3f %6.1f"
              % ("", geom, len(rows), med(rows, "mean_surge"),
                 med(rows, "lateral_fraction"),
                 med(rows, "zero_surge_fraction"),
                 med(rows, "lateral_peak"), med(rows, "reversals")))

    print("\n=== LA MANOVRA E' ARRIVATA ALL'AGGIRAMENTO? ===")
    print("Lo scostamento iniziale comanda 0.20 m/s di laterale; solo la "
          "fase di aggiramento arriva a 0.30, con surge 0.40.")
    for geom in ("K0", "K1", "K1M"):
        rows = real.get(geom, [])
        if not rows:
            continue
        got = sum(1 for r in rows if (r.get("lateral_peak") or 0) > 0.25)
        print("  reale %-4s: %d prove su %d hanno superato 0.25 m/s "
              "di laterale" % (geom, got, len(rows)))
    for lvl in ("S0", "S1", "S2", "S3"):
        rows = sim.get((lvl, "K0", "committed"), [])
        got = sum(1 for r in rows if (r.get("lateral_peak") or 0) > 0.25)
        print("  sim %s K0 : %d prove su %d hanno superato 0.25 m/s "
              "di laterale" % (lvl, got, len(rows)))

    print("\n=== SCARTO ASSOLUTO |sim - reale| sul surge medio, K0 ===")
    rk0 = med(real.get("K0", []), "mean_surge")
    for lvl in ("S0", "S1", "S2", "S3"):
        s = med(sim.get((lvl, "K0", "committed"), []), "mean_surge")
        if s is not None and rk0 is not None:
            print("  %s  sim %.4f  reale %.4f  scarto %.4f"
                  % (lvl, s, rk0, abs(s - rk0)))

    out = {
        "note": ("Command-domain comparison only. The two domains record "
                 "the same topic at the same boundary. Nothing spatial is "
                 "compared, because the physical runs have no external "
                 "reference and integrating their command stream would "
                 "manufacture the quantity under test."),
        "not_a_predictivity_test": (
            "The frozen simulated stack and the flown stack differ in "
            "seven documented ways; agreement here would not validate the "
            "calibration and disagreement would not refute it."),
        "simulation_frozen": {"|".join(k): v for k, v in sim.items()},
        "real": {k: v for k, v in real.items()},
    }
    p = os.path.join(_ROOT, "experiments", "command_domain_comparison.json")
    with open(p, "w") as f:
        json.dump(out, f, indent=1)
    print("\n->", os.path.relpath(p, _ROOT).replace("\\", "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
