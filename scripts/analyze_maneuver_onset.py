"""Why does T1 (hold) beat T2 (CV Kalman) during maneuver-onset dropouts?

Phase-7 negative result to explain (D5: dropout starting 1 s after avoidance
engagement): T1 pred-RMSE 0.028 vs T2 0.063. Hypothesis: the CV model
extrapolates the PRE-maneuver image velocity through a sudden change in
vehicle motion and overshoots.

This script quantifies it on the deterministic D5/D6 cases (+ exploratory
T2b damped variant) and produces the time-series plot: measured cx, T1/T2/
T2b estimates, ground truth, with the engagement/dropout phases marked.

Development-only analysis: no closed loop, no parameter changes to T2.
Output: experiments/simulation/temporal_estimators_phase7b/onset_analysis/
"""

from __future__ import annotations

import json
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "rov_obstacle_tracking"))

from rov_obstacle_tracking.metrics import evaluate  # noqa: E402
from rov_obstacle_tracking.replay import run_replay  # noqa: E402
from rov_obstacle_tracking.temporal_core import make_estimator  # noqa: E402
from rov_obstacle_tracking.test_cases import (  # noqa: E402
    T_AVOID_END,
    T_ENGAGE,
    generate_case,
    gt_state,
)

OUT = os.path.join(REPO, "experiments", "simulation",
                   "temporal_estimators_phase7b", "onset_analysis")
METHODS = ["t1", "t2", "t2b"]


def series(results):
    t, est = [], []
    for r in results:
        if r.output.obstacles:
            t.append(r.t)
            est.append(r.output.obstacles[0].cx)
    return np.array(t), np.array(est)


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    report = {"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                             time.gmtime()),
              "hypothesis": "CV extrapolates pre-maneuver image velocity "
                            "through the motion change and overshoots",
              "cases": {}}

    # 1. Quantify the GT image-velocity discontinuity at engagement.
    eps = 0.2
    v_before = (gt_state(T_ENGAGE - 0.01)["cx"]
                - gt_state(T_ENGAGE - eps - 0.01)["cx"]) / eps
    v_after = (gt_state(T_ENGAGE + eps)["cx"] - gt_state(T_ENGAGE)["cx"]) / eps
    report["gt_cx_velocity_before_engage"] = round(v_before, 4)
    report["gt_cx_velocity_after_engage"] = round(v_after, 4)
    report["velocity_discontinuity"] = round(v_after - v_before, 4)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for case, window in (("D5", (T_ENGAGE + 1.0, T_ENGAGE + 3.0)),
                         ("D6", (T_ENGAGE + 6.0, T_ENGAGE + 8.0))):
        records = generate_case(case)
        gt_t = [r.t for r in records if r.gt and r.gt.get("present")]
        gt_cx = [r.gt["cx"] for r in records if r.gt and r.gt.get("present")]
        meas_t = [r.t for r in records if r.detection_present]
        meas_cx = [r.meas["cx"] for r in records if r.detection_present]

        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(gt_t, gt_cx, "k-", linewidth=1.6, label="ground truth cx")
        ax.plot(meas_t, meas_cx, ".", markersize=2.5, alpha=0.45,
                label="measured cx")
        case_report = {}
        for m in METHODS:
            res = run_replay(make_estimator(m), records)
            t, est = series(res)
            ax.plot(t, est, linewidth=1.1, label=f"{m} estimate")
            met = evaluate(res, method=m, dataset=case)
            case_report[m] = {
                "rmse_center": round(met.rmse_center or 0, 5),
                "rmse_center_predicted": (round(met.rmse_center_predicted, 5)
                                          if met.rmse_center_predicted
                                          else None),
            }
            # Peak cx deviation from GT inside the dropout window.
            sel = [(tt, ee) for tt, ee in zip(t, est)
                   if window[0] <= tt <= window[1]]
            if sel:
                errs = [abs(ee - gt_state(tt)["cx"]) for tt, ee in sel
                        if gt_state(tt)]
                case_report[m]["peak_cx_error_in_dropout"] = round(
                    max(errs), 4) if errs else None
        ax.axvline(T_ENGAGE, color="tab:red", linestyle=":",
                   label="avoidance engagement")
        ax.axvspan(window[0], window[1], color="gray", alpha=0.2,
                   label="detector silence")
        ax.set_xlim(T_ENGAGE - 3, min(T_AVOID_END, window[1] + 4))
        ax.set_xlabel("t [s]")
        ax.set_ylabel("normalized image cx")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title(f"{case}: estimates through maneuver-phase dropout")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"fig_onset_{case}.png"), dpi=140)
        plt.close(fig)
        report["cases"][case] = case_report

    with open(os.path.join(OUT, "onset_analysis.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
