"""Evaluation metrics for temporal estimators (offline replay layer).

Ground truth is allowed HERE and only here (evaluation is not runtime).

Answers, per replay run:
  - accuracy: state RMSE vs GT (overall / measured-only / prediction-only)
  - availability: output continuity, missed time, reacquisition delay
  - safety: ghost-track duration (output while GT absent)
  - probabilistic consistency (KF methods): NIS statistics vs chi-square
    bounds, empirical 1-sigma / 2-sigma coverage of the position estimate
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .replay import TickResult

# Chi-square 4-dof mean = 4; 95% interval for a single NIS sample.
CHI2_4DOF_P025 = 0.484
CHI2_4DOF_P975 = 11.143
# Per-axis Gaussian coverage targets.
COVER_1S = 0.6827
COVER_2S = 0.9545


def _sq(v: float) -> float:
    return v * v


@dataclass
class ReplayMetrics:
    method: str = ""
    dataset: str = ""
    ticks: int = 0
    # Accuracy (position = normalized image units).
    rmse_center: Optional[float] = None
    rmse_center_measured: Optional[float] = None
    rmse_center_predicted: Optional[float] = None
    rmse_size: Optional[float] = None
    # Availability.
    gt_present_time_s: float = 0.0
    output_during_gt_s: float = 0.0
    availability: Optional[float] = None
    prediction_only_time_s: float = 0.0
    longest_gap_s: float = 0.0
    reacquisition_delay_s: Optional[float] = None
    # Safety.
    ghost_track_time_s: float = 0.0
    # Probabilistic consistency (KF only).
    nis_samples: int = 0
    nis_mean: Optional[float] = None
    nis_in_95_band_frac: Optional[float] = None
    coverage_1s: Optional[float] = None
    coverage_2s: Optional[float] = None
    details: Dict[str, Any] = field(default_factory=dict)


def evaluate(results: List[TickResult], method: str = "",
             dataset: str = "") -> ReplayMetrics:
    m = ReplayMetrics(method=method, dataset=dataset, ticks=len(results))
    if not results:
        return m
    dt = (results[-1].t - results[0].t) / max(1, len(results) - 1)

    se_c: List[float] = []
    se_c_meas: List[float] = []
    se_c_pred: List[float] = []
    se_s: List[float] = []
    nis_vals: List[float] = []
    cov1 = cov2 = cov_n = 0
    gap = 0.0
    dropout_started: Optional[float] = None
    reacq: List[float] = []
    in_dropout = False

    for r in results:
        gt = r.gt
        gt_present = bool(gt and gt.get("present"))
        out_present = r.output.publish and len(r.output.obstacles) > 0
        ob = r.output.obstacles[0] if out_present else None

        if gt_present:
            m.gt_present_time_s += dt
            if out_present:
                m.output_during_gt_s += dt
                gap = 0.0
            else:
                gap += dt
                m.longest_gap_s = max(m.longest_gap_s, gap)
        elif out_present:
            m.ghost_track_time_s += dt

        # Reacquisition delay: dropout (no upstream detection while GT
        # present) -> time until upstream detection returns AND output valid.
        if gt_present and not r.upstream_detection and not in_dropout:
            in_dropout = True
            dropout_started = r.t
        if in_dropout and r.upstream_detection:
            in_dropout = False
            if out_present and dropout_started is not None:
                reacq.append(0.0)  # output already valid at reacquisition
            elif dropout_started is not None:
                reacq.append(r.t - dropout_started)

        if ob is not None and gt_present:
            ec = math.hypot(ob.cx - gt["cx"], ob.cy - gt["cy"])
            es = math.hypot(ob.w - gt["w"], ob.h - gt["h"])
            se_c.append(_sq(ec))
            se_s.append(_sq(es))
            if ob.is_predicted:
                se_c_pred.append(_sq(ec))
                m.prediction_only_time_s += dt
            else:
                se_c_meas.append(_sq(ec))

            # Coverage from reported covariance (KF debug), position axes.
            covd = ob.debug.get("cov_diag")
            if covd:
                for axis, key in ((0, "cx"), (1, "cy")):
                    sig = math.sqrt(max(covd[axis], 1e-12))
                    err = abs((ob.cx if axis == 0 else ob.cy) - gt[key])
                    cov_n += 1
                    if err <= sig:
                        cov1 += 1
                    if err <= 2 * sig:
                        cov2 += 1

        nis = (ob.debug.get("nis") if ob is not None else None)
        if nis is not None and not ob.is_predicted:
            nis_vals.append(float(nis))

    def rmse(vals: List[float]) -> Optional[float]:
        return math.sqrt(sum(vals) / len(vals)) if vals else None

    m.rmse_center = rmse(se_c)
    m.rmse_center_measured = rmse(se_c_meas)
    m.rmse_center_predicted = rmse(se_c_pred)
    m.rmse_size = rmse(se_s)
    m.availability = (m.output_during_gt_s / m.gt_present_time_s
                      if m.gt_present_time_s > 0 else None)
    m.reacquisition_delay_s = (sum(reacq) / len(reacq)) if reacq else None
    if nis_vals:
        m.nis_samples = len(nis_vals)
        m.nis_mean = sum(nis_vals) / len(nis_vals)
        m.nis_in_95_band_frac = (
            sum(1 for v in nis_vals
                if CHI2_4DOF_P025 <= v <= CHI2_4DOF_P975) / len(nis_vals))
    if cov_n:
        m.coverage_1s = cov1 / cov_n
        m.coverage_2s = cov2 / cov_n
    return m


def metrics_to_dict(m: ReplayMetrics) -> Dict[str, Any]:
    d = dict(m.__dict__)
    d.pop("details", None)
    for k, v in list(d.items()):
        if isinstance(v, float):
            d[k] = round(v, 5)
    return d
