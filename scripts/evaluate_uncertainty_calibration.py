"""Evaluate whether estimator covariance actually predicts observed error.

Runs a (possibly calibrated) T3 and the fixed T2 on held-out replay data and
reports/plots the probabilistic-consistency evidence:

  - NIS distribution vs the chi-square 4-dof reference
  - empirical 1-sigma / 2-sigma coverage vs 68.3% / 95.4%
  - predicted sigma vs empirical |error| (binned)
  - residual vs confidence and vs bbox area

Usage:
    python scripts/evaluate_uncertainty_calibration.py \
        --data <dir with *.jsonl> [--noise-model config/t3_noise_model.json] \
        --out experiments/simulation/temporal_estimators/uncertainty_eval
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "rov_obstacle_tracking"))

from rov_obstacle_tracking.metrics import evaluate, metrics_to_dict  # noqa: E402
from rov_obstacle_tracking.replay import load_dataset, run_replay  # noqa: E402
from rov_obstacle_tracking.temporal_core import make_estimator  # noqa: E402


def gather(results):
    """Per-tick arrays for plots: sigma, |err|, nis, conf, area."""
    rows = []
    for r in results:
        if not (r.output.obstacles and r.gt and r.gt.get("present")):
            continue
        ob = r.output.obstacles[0]
        covd = ob.debug.get("cov_diag")
        if not covd:
            continue
        err = math.hypot(ob.cx - r.gt["cx"], ob.cy - r.gt["cy"])
        sigma = math.sqrt(max(covd[0], 1e-12) + max(covd[1], 1e-12))
        rows.append({
            "sigma": sigma, "err": err,
            "nis": ob.debug.get("nis"),
            "conf": ob.confidence,
            "area": ob.w * ob.h,
            "predicted": ob.is_predicted,
        })
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True)
    parser.add_argument("--noise-model", default=None)
    parser.add_argument("--out", default=os.path.join(
        REPO, "experiments", "simulation", "temporal_estimators",
        "uncertainty_eval"))
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.data, "*.jsonl")))
    if not files:
        print("no data"); return 2
    os.makedirs(args.out, exist_ok=True)

    report = {"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                             time.gmtime()),
              "noise_model": args.noise_model or "default (theta=0)",
              "files": [os.path.basename(f) for f in files],
              "methods": {}}
    all_rows = {}
    for method in ("t2", "t3"):
        rows = []
        mets = []
        for f in files:
            est = make_estimator(method, noise_model_path=(
                args.noise_model if method == "t3" else None))
            res = run_replay(est, load_dataset(f))
            rows.extend(gather(res))
            mets.append(metrics_to_dict(evaluate(
                res, method=method, dataset=os.path.basename(f))))
        all_rows[method] = rows
        nis = [r["nis"] for r in rows if r["nis"] is not None]
        cover1 = (sum(1 for r in rows if r["err"] <= r["sigma"]) / len(rows)
                  if rows else None)
        cover2 = (sum(1 for r in rows if r["err"] <= 2 * r["sigma"])
                  / len(rows) if rows else None)
        report["methods"][method] = {
            "samples": len(rows),
            "nis_mean": (sum(nis) / len(nis)) if nis else None,
            "nis_target": 4.0,
            "coverage_1s": cover1, "coverage_1s_target": 0.6827,
            "coverage_2s": cover2, "coverage_2s_target": 0.9545,
            "per_file": mets,
        }
        print(method, json.dumps({k: v for k, v in
                                  report["methods"][method].items()
                                  if k != "per_file"}))

    with open(os.path.join(args.out, "uncertainty_report.json"), "w") as f:
        json.dump(report, f, indent=2)
    _plots(all_rows, args.out)
    print("wrote", args.out)
    return 0


def _plots(all_rows, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for method, rows in all_rows.items():
        if not rows:
            continue
        sig = np.array([r["sigma"] for r in rows])
        err = np.array([r["err"] for r in rows])
        conf = np.array([r["conf"] for r in rows])
        area = np.array([r["area"] for r in rows])
        nis = np.array([r["nis"] for r in rows if r["nis"] is not None])

        # sigma vs |err| (binned means)
        bins = np.quantile(sig, np.linspace(0, 1, 8))
        idx = np.digitize(sig, bins[1:-1])
        bx = [sig[idx == i].mean() for i in range(len(bins) - 1)
              if (idx == i).any()]
        by = [err[idx == i].mean() for i in range(len(bins) - 1)
              if (idx == i).any()]
        axes[0, 0].plot(bx, by, "o-", label=method)
        axes[0, 1].hist(nis, bins=40, density=True, alpha=0.5, label=method)
        axes[1, 0].scatter(conf, err, s=3, alpha=0.25, label=method)
        axes[1, 1].scatter(area, err, s=3, alpha=0.25, label=method)

    lim = axes[0, 0].get_xlim()
    axes[0, 0].plot(lim, lim, "k:", linewidth=0.8, label="ideal (err = sigma)")
    axes[0, 0].set_xlabel("predicted sigma"); axes[0, 0].set_ylabel("mean |err|")
    # Chi-square 4-dof reference density for the NIS histogram.
    x = np.linspace(0.05, 15, 200)
    chi2_4 = x / 4.0 * np.exp(-x / 2.0)  # k=4: f(x) = x*exp(-x/2)/4
    axes[0, 1].plot(x, chi2_4, "k:", linewidth=1.0, label="chi2 (4 dof)")
    axes[0, 1].set_xlabel("NIS"); axes[0, 1].set_ylabel("density")
    axes[1, 0].set_xlabel("confidence"); axes[1, 0].set_ylabel("|err|")
    axes[1, 1].set_xlabel("bbox area"); axes[1, 1].set_ylabel("|err|")
    for ax in axes.flat:
        ax.grid(alpha=0.3); ax.legend(fontsize=8)
    fig.suptitle("Uncertainty consistency: does covariance predict error?")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_uncertainty_consistency.png"), dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
