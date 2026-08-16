"""Resample every real command trace onto the nominal 19 Hz grid.

The command topic is nominally published at 19 Hz. Some recordings
contain more samples than that grid holds. Where the extra samples are
exact duplicates they carry no information, and collapsing them onto the
grid is resampling: no recorded value is altered, only repetitions are
removed. Where they are NOT duplicates the extra samples are real
divergence, collapsing them would invent a trace that was never
commanded, and the run is reported as failing the criterion instead.

The criterion is stated before it is applied, and is measured against
the data itself rather than picked:

    a run passes if the mean within-cell spread of its command samples
    does not exceed the within-cell spread of a natively 19 Hz run.

Runs already at 19 Hz pass trivially and pass through unchanged apart
from the grid alignment.

Raw traces are never modified. Output goes to cmd_trace_19hz.jsonl
beside each cmd_trace.jsonl, so every number in the analysis can be
traced back to the recording it came from.

Usage:
    python scripts/analysis/normalize_command_traces.py
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

GRID_HZ = 19.0

# Baseline: measured, not chosen. A recording made AT the grid rate has
# exactly one planner by construction, so whatever within-cell spread
# those recordings show is what a single planner looks like on this
# grid. The baseline is the largest such spread in the campaign, so the
# criterion cannot reject a run that is indistinguishable from the
# known-clean ones. Hard-coding a number instead would let the threshold
# be tuned until it gave the answer one wanted, which is the failure
# mode this whole file exists to avoid.
NATIVE_RATE_TOLERANCE = 0.15

CAMPAIGN = [
    (1, "K0", "20260816_120927_01_K0_committed_v2"),
    (2, "K0", "20260816_122520_run02_K0_committed"),
    (3, "K0", "20260816_122752_run03_K0_committed"),
    (4, "K0", "20260816_123213_run04_K0_committed"),
    (5, "K0", "20260816_123406_run05_K0_committed"),
    (6, "K1", "20260816_124312_run06_K1_committed"),
    (7, "K1", "20260816_124803_run07_K1_committed"),
    (8, "K1M", "20260816_125238_run08_K1M_committed"),
    (9, "K1M", "20260816_125518_run09_K1M_committed"),
]


def load(path):
    with open(path) as f:
        rows = [json.loads(line) for line in f if line.strip()]
    t = np.array([r["t"] for r in rows], float)
    x = np.array([r["x"] for r in rows], float)
    y = np.array([r["y"] for r in rows], float)
    r_ = np.array([r.get("r", 0.0) for r in rows], float)
    return t, x, y, r_


def spread(t, x, y):
    """Mean within-cell range of the command, over cells holding >1 sample."""
    cell = np.floor(t * GRID_HZ).astype(int)
    sy, sx = [], []
    for k in np.unique(cell):
        m = cell == k
        if m.sum() < 2:
            continue
        sy.append(y[m].max() - y[m].min())
        sx.append(x[m].max() - x[m].min())
    if not sy:
        return 0.0, 0.0, 1.0
    sy = np.array(sy)
    return float(sy.mean()), float(np.mean(sx)), float((sy == 0).mean())


def collapse(t, x, y, r_):
    """One sample per grid cell: the median, which equals the value itself
    when the samples in the cell are duplicates."""
    cell = np.floor(t * GRID_HZ).astype(int)
    out = []
    for k in np.unique(cell):
        m = cell == k
        out.append({
            "t": round(float(k) / GRID_HZ, 4),
            "x": round(float(np.median(x[m])), 4),
            "y": round(float(np.median(y[m])), 4),
            "r": round(float(np.median(r_[m])), 4),
        })
    return out


def main() -> int:
    base = os.path.join(_ROOT, "experiments", "real", "quick_runs")

    # First pass: measure every run, and derive the baseline from those
    # recorded at the grid rate.
    measured = []
    for run, geom, name in CAMPAIGN:
        src = os.path.join(base, name, "cmd_trace.jsonl")
        if not os.path.exists(src):
            print("prova %d: traccia mancante" % run)
            continue
        t, x, y, r_ = load(src)
        hz = len(t) / max(t[-1], 1e-6)
        sy, sx, dup = spread(t, x, y)
        measured.append((run, geom, name, t, x, y, r_, hz, sy, dup))

    native = [m[8] for m in measured
              if abs(m[7] - GRID_HZ) <= NATIVE_RATE_TOLERANCE * GRID_HZ]
    baseline = max(native) if native else 0.0
    print("riferimento da %d registrazioni native a %.0f Hz: "
          "dispersione massima %.4f m/s\n" % (len(native), GRID_HZ, baseline))

    report = []
    for run, geom, name, t, x, y, r_, hz, sy, dup in measured:
        d = os.path.join(base, name)
        ok = sy <= baseline
        rec = {
            "run": run, "geometry": geom, "directory": name,
            "recorded_rate_hz": round(hz, 1),
            "within_cell_spread_m_s": round(sy, 4),
            "identical_cells": round(dup, 3),
            "passes": bool(ok),
        }
        if ok:
            rows = collapse(t, x, y, r_)
            dst = os.path.join(d, "cmd_trace_19hz.jsonl")
            with open(dst, "w") as f:
                for row in rows:
                    f.write(json.dumps(row) + "\n")
            rec["samples_out"] = len(rows)
            rec["output"] = os.path.relpath(dst, _ROOT).replace("\\", "/")
        report.append(rec)
        print("prova %d (%s): %5.0f Hz  spread %.4f  identiche %3.0f%%  -> %s"
              % (run, geom, hz, sy, 100 * dup,
                 "19 Hz, %d campioni" % rec.get("samples_out", 0) if ok
                 else "NON SUPERA IL CRITERIO"))

    out = os.path.join(_ROOT, "experiments", "real",
                       "command_trace_normalization.json")
    with open(out, "w") as f:
        json.dump({
            "grid_hz": GRID_HZ,
            "baseline_spread_m_s": round(baseline, 4),
            "baseline_source": ("largest within-cell spread among the "
                                "recordings made at the grid rate"),
            "criterion": ("within-cell command spread at or below the "
                          "spread of a natively 19 Hz recording"),
            "runs": report,
        }, f, indent=1)
    n_ok = sum(1 for r in report if r["passes"])
    print("\n%d prove su %d normalizzate a 19 Hz" % (n_ok, len(report)))
    print("->", os.path.relpath(out, _ROOT).replace("\\", "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
