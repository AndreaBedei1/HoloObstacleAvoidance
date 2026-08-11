"""Offline replay evaluation: D0-D14 controlled cases x T0-T3 estimators.

Generates the deterministic datasets, runs every estimator on every case,
writes per-run metrics + an aggregate table + plots. No ROS, no Unreal.

Run (ros2_lyrical env):
    python scripts/run_temporal_replay_eval.py
Outputs:
    experiments/simulation/temporal_estimators/replay_eval/
        datasets/D*.jsonl
        metrics.json
        summary.md
        fig_*.png
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "rov_obstacle_tracking"))

from rov_obstacle_tracking.metrics import evaluate, metrics_to_dict  # noqa: E402
from rov_obstacle_tracking.replay import run_replay, save_dataset  # noqa: E402
from rov_obstacle_tracking.temporal_core import make_estimator  # noqa: E402
from rov_obstacle_tracking.test_cases import ALL_CASES, generate_case  # noqa: E402

METHODS = ["t0", "t1", "t2", "t3"]
OUT = os.path.join(REPO, "experiments", "simulation", "temporal_estimators",
                   "replay_eval")


def git_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"],
                                       cwd=REPO, text=True).strip()
    except Exception:
        return "unknown"


def main() -> int:
    os.makedirs(os.path.join(OUT, "datasets"), exist_ok=True)
    all_metrics = []
    for case in ALL_CASES:
        records = generate_case(case)
        save_dataset(records, os.path.join(OUT, "datasets", f"{case}.jsonl"))
        for method in METHODS:
            est = make_estimator(method)
            results = run_replay(est, records)
            m = evaluate(results, method=method, dataset=case)
            all_metrics.append(metrics_to_dict(m))
            print(f"{case} {method}: avail={m.availability} "
                  f"rmse_c={m.rmse_center} ghost={round(m.ghost_track_time_s,2)} "
                  f"pred_rmse={m.rmse_center_predicted} "
                  f"nis={m.nis_mean}", flush=True)

    manifest = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "commit_sha": git_sha(),
        "methods": METHODS,
        "cases": ALL_CASES,
        "note": ("T3 runs with the DEFAULT UNCALIBRATED noise model "
                 "(theta=0, identical to T2) until B1/B2 assets allow real "
                 "residual calibration"),
        "metrics": all_metrics,
    }
    with open(os.path.join(OUT, "metrics.json"), "w") as f:
        json.dump(manifest, f, indent=2)

    # Aggregate summary table (markdown).
    lines = ["# Temporal estimator replay evaluation\n",
             f"Commit {git_sha()[:9]} — datasets deterministic (seed 7)\n",
             "| Case | Method | Availability | RMSE center | RMSE pred-only "
             "| Ghost [s] | Longest gap [s] | NIS mean | 1σ cov | 2σ cov |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for m in all_metrics:
        lines.append(
            f"| {m['dataset']} | {m['method']} | {m['availability']} "
            f"| {m['rmse_center']} | {m['rmse_center_predicted']} "
            f"| {m['ghost_track_time_s']} | {m['longest_gap_s']} "
            f"| {m['nis_mean']} | {m['coverage_1s']} | {m['coverage_2s']} |")
    with open(os.path.join(OUT, "summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    _plots(all_metrics)
    print(f"wrote {OUT}")
    return 0


def _plots(all_metrics) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    cases = ALL_CASES
    fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=True)
    x = np.arange(len(cases))
    width = 0.2
    for i, method in enumerate(METHODS):
        sel = {m["dataset"]: m for m in all_metrics if m["method"] == method}
        avail = [sel[c].get("availability") or 0 for c in cases]
        rmse = [sel[c].get("rmse_center") or 0 for c in cases]
        ghost = [sel[c].get("ghost_track_time_s") or 0 for c in cases]
        axes[0].bar(x + (i - 1.5) * width, avail, width, label=method)
        axes[1].bar(x + (i - 1.5) * width, rmse, width, label=method)
        axes[2].bar(x + (i - 1.5) * width, ghost, width, label=method)
    axes[0].set_ylabel("availability\n(output while GT present)")
    axes[1].set_ylabel("RMSE center [img units]")
    axes[2].set_ylabel("ghost track time [s]")
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(cases)
    for ax in axes:
        ax.grid(alpha=0.3, axis="y")
        ax.legend(fontsize=8, ncol=4)
    fig.suptitle("Temporal estimators on controlled cases (T3 uncalibrated == T2)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_case_comparison.png"), dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
