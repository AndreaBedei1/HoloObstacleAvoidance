"""Fit the S2 timing model: rate, jitter, and the BURST structure of
missing observations.

S1 makes each observation as wrong as a real one but still delivers it
every tick. S2 adds when-and-whether it arrives, and nothing else.

THE CONSTRAINT THAT PREVENTS DOUBLE COUNTING. S1 owns the MARGINAL
probability that a frame yields a usable observation. A two-state
visible/blind process has a marginal of its own, so fitting it freely
would thin the stream twice: once through S1's detection probability and
again through S2's blind runs. The burst process is therefore fitted
SUBJECT TO reproducing the marginal, never in addition to it:

    mean_visible_run / (mean_visible_run + mean_blind_run) == p

WHY A TWO-STATE PROCESS AND NOT INDEPENDENT DROPOUT. Independent
per-frame dropout at the same marginal produces short, evenly scattered
gaps. The real stream goes blind in RUNS -- dome bubbles, glare, the
anchor leaving the frame during a turn -- and a run of consecutive
misses is what actually starves a temporal estimator. This script
measures both and reports the comparison, so the choice is made by the
data rather than assumed: if the observed runs are not longer than
independent dropout would give, the simpler model is the honest one.

Source: the pilot mission logs, which record `accepted` per control
cycle with its timestamp.

Usage:  python scripts/analysis/fit_s2_timing.py
"""

from __future__ import annotations

import glob
import json
import os

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
MISSIONS = os.path.join(_ROOT, "experiments", "real", "missions")
OUT = os.path.join(_ROOT, "config", "calibration", "s2_timing_fit.json")


def runs(seq):
    """Lengths of consecutive equal values: [(value, length), ...]."""
    out = []
    for v in seq:
        if out and out[-1][0] == v:
            out[-1][1] += 1
        else:
            out.append([v, 1])
    return [(v, n) for v, n in out]


def main() -> int:
    cycles, dts, per_run = [], [], []
    for path in sorted(glob.glob(os.path.join(MISSIONS, "*", "log.json"))):
        with open(path) as f:
            d = json.load(f)
        log = d.get("log") or []
        if len(log) < 5:
            continue
        acc = [bool(r.get("accepted")) for r in log]
        ts = [r.get("t") for r in log if r.get("t") is not None]
        cycles.extend(acc)
        dts.extend(np.diff(ts).tolist())
        per_run.append({"run": os.path.basename(os.path.dirname(path)),
                        "n": len(acc), "p": round(np.mean(acc), 3)})

    if not cycles:
        print("nessun log utilizzabile")
        return 1

    p = float(np.mean(cycles))
    dt = np.array(dts, float)
    dt = dt[(dt > 0) & (dt < 5.0)]
    print("run %d, cicli %d" % (len(per_run), len(cycles)))
    print("\n--- ritmo del ciclo ---")
    print("  periodo mediano %.3f s  = %.1f Hz" % (np.median(dt),
                                                   1.0 / np.median(dt)))
    print("  jitter (scarto robusto) %.3f s"
          % (float(np.median(np.abs(dt - np.median(dt)))) * 1.4826))
    print("  coda: 95mo percentile %.3f s, massimo %.3f s"
          % (np.percentile(dt, 95), dt.max()))

    print("\n--- accettazione dell'osservazione ---")
    print("  marginale p = %.3f su %d cicli" % (p, len(cycles)))
    for r in per_run:
        print("    %s  n=%3d  p=%.3f" % (r["run"], r["n"], r["p"]))

    # ---- burst structure --------------------------------------------
    rr = runs(cycles)
    vis = [n for v, n in rr if v]
    bli = [n for v, n in rr if not v]
    print("\n--- struttura a raffiche ---")
    if vis and bli:
        mv, mb = float(np.mean(vis)), float(np.mean(bli))
        print("  tratti VISIBILI: %d, media %.2f cicli, massimo %d"
              % (len(vis), mv, max(vis)))
        print("  tratti CIECHI:   %d, media %.2f cicli, massimo %d"
              % (len(bli), mb, max(bli)))
        print("  marginale implicito %.3f (deve valere %.3f)"
              % (mv / (mv + mb), p))
        # independent-dropout comparison at the SAME marginal
        rngs = np.random.default_rng(12345)
        sim = rngs.random(len(cycles)) < p
        sr = runs(sim.tolist())
        sb = [n for v, n in sr if not v]
        print("  se le mancate fossero indipendenti: tratto cieco medio "
              "%.2f, massimo %d" % (float(np.mean(sb)), max(sb)))
        bursty = np.mean(bli) > 1.3 * float(np.mean(sb))
        print("  -> %s" % ("RAFFICHE reali: i tratti ciechi sono piu "
                           "lunghi di quanto darebbe il caso"
                           if bursty else
                           "nessuna struttura a raffiche oltre il caso: "
                           "il modello indipendente e sufficiente"))
    else:
        mv = mb = None
        bursty = False
        print("  stream tutto visibile o tutto cieco: struttura non "
              "stimabile")

    out = {
        "source": "experiments/real/missions/*/log.json",
        "n_runs": len(per_run), "n_cycles": len(cycles),
        "cycle_period_s": round(float(np.median(dt)), 4),
        "cycle_rate_hz": round(1.0 / float(np.median(dt)), 2),
        "cycle_jitter_s": round(float(np.median(np.abs(
            dt - np.median(dt)))) * 1.4826, 4),
        "cycle_period_p95_s": round(float(np.percentile(dt, 95)), 4),
        "marginal_p_accept": round(p, 4),
        "mean_visible_run_cycles": (None if mv is None else round(mv, 3)),
        "mean_blind_run_cycles": (None if mb is None else round(mb, 3)),
        "longest_blind_run_cycles": (max(bli) if bli else None),
        "bursty": bool(bursty),
        "constraint": "mean_visible/(mean_visible+mean_blind) reproduces "
                      "the marginal; the burst process is fitted SUBJECT "
                      "TO it, never in addition, or the stream would be "
                      "thinned twice",
        "per_run": per_run,
        "caveat": "the marginal here is the QUALIFIER acceptance rate at "
                  "the planner's input, which is the stream S2 models. It "
                  "is not the raw detection rate fitted in S1: the "
                  "qualifier rejects observations that the detector did "
                  "produce.",
    }
    with open(OUT, "w") as f:
        json.dump(out, f, indent=2)
    print("\n->", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
