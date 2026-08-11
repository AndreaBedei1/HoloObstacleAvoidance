"""Temporal obstacle estimation core (T0-T3) — pure Python, offline-runnable.

Common contract for all methods so the comparison is unbiased:

  upstream --Obstacle2DArray--> [estimator] --Obstacle2DArray--> planner
            (raw detections)                  (planner-unchanged)

Three upstream conditions are EXPLICITLY distinguished (Baseline-0 C_3
lesson: they are not equivalent):

  A. fresh detection        -> on_message(event with detections)
  B. fresh EMPTY array      -> on_message(event with [])  (evidence of absence)
  C. SILENCE                -> no on_message call at all; only tick(t) runs

Methods:
  T0  raw passthrough (control baseline; reproduces current behavior,
      including the early-dropout weakness — deliberately NOT improved)
  T1  last-valid hold + optional EMA, deterministic expiry
  T2  fixed-noise linear Kalman filter, CV image-space model, gating,
      dropout prediction with bounded horizon
  T3  T2 + adaptive measurement noise R = R0 * exp(theta . phi(features));
      theta comes from an empirical calibration file — DEFAULT theta = 0
      (T3 == T2) with calibrated=False until real residual calibration data
      exists (blockers B1/B2). No fake coefficients.

Ground truth NEVER enters this module's runtime path; it exists only in the
replay/evaluation layer for offline metrics.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

CHI2_GATE_4DOF_099 = 13.277  # 99% gate for 4-dof innovation


# ---------------------------------------------------------------------------
# Data contract
# ---------------------------------------------------------------------------

@dataclass
class Detection:
    """One raw image-space detection (mirrors rov_obstacle_msgs/Obstacle2D)."""
    class_name: str
    confidence: float
    cx: float
    cy: float
    w: float
    h: float

    def as_measurement(self) -> np.ndarray:
        w = max(self.w, 1e-4)
        h = max(self.h, 1e-4)
        return np.array([self.cx, self.cy, math.log(w), math.log(h)])


@dataclass
class DetectionEvent:
    """A fresh upstream MESSAGE (may carry zero detections = fresh-empty)."""
    t: float
    detections: List[Detection]

    @property
    def is_empty(self) -> bool:
        return len(self.detections) == 0


@dataclass
class EstimatedObstacle:
    """Planner-facing estimate + debug fields (debug never reaches planner)."""
    class_name: str
    confidence: float
    cx: float
    cy: float
    w: float
    h: float
    is_predicted: bool          # True when held/predicted (no fresh meas)
    track_id: int
    age_s: float                # since track creation
    time_since_meas_s: float    # since last accepted real measurement
    debug: Dict[str, Any] = field(default_factory=dict)


@dataclass
class EstimatorOutput:
    """What tick() wants published downstream this cycle."""
    publish: bool                       # False = stay silent this tick (T0)
    obstacles: List[EstimatedObstacle] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------

class TemporalEstimator:
    """Common interface. Subclasses must not require ROS or ground truth."""

    name = "base"

    def on_message(self, event: DetectionEvent) -> None:
        raise NotImplementedError

    def tick(self, t: float) -> EstimatorOutput:
        raise NotImplementedError

    def state_debug(self) -> Dict[str, Any]:
        return {}


# ---------------------------------------------------------------------------
# T0 — raw passthrough
# ---------------------------------------------------------------------------

class T0Raw(TemporalEstimator):
    """Frame-by-frame control baseline. Republishes each upstream message
    exactly once; SILENT when upstream is silent. No persistence, no
    prediction, no hold. Reproduces the current pipeline behavior including
    its dropout weakness — do not 'improve' this."""

    name = "t0_raw"

    def __init__(self) -> None:
        self._pending: Optional[DetectionEvent] = None
        self._counter = 0

    def on_message(self, event: DetectionEvent) -> None:
        self._pending = event

    def tick(self, t: float) -> EstimatorOutput:
        if self._pending is None:
            return EstimatorOutput(publish=False)
        ev = self._pending
        self._pending = None
        obstacles = [
            EstimatedObstacle(
                class_name=d.class_name, confidence=d.confidence,
                cx=d.cx, cy=d.cy, w=d.w, h=d.h,
                is_predicted=False, track_id=i,
                age_s=0.0, time_since_meas_s=0.0,
                debug={"method": self.name},
            )
            for i, d in enumerate(ev.detections)
        ]
        return EstimatorOutput(publish=True, obstacles=obstacles)


# ---------------------------------------------------------------------------
# T1 — hold / EMA
# ---------------------------------------------------------------------------

@dataclass
class T1Config:
    # Tuned on the D-series DEVELOPMENT cases (max dropout D4 = 2.0 s):
    # bridge up to 2 s of silence with margin. Never tuned on E-runs.
    hold_duration_s: float = 2.5
    ema_alpha: float = 0.4           # 1.0 = no smoothing (alpha on new meas)
    empty_misses_to_drop: int = 3    # consecutive fresh-empty msgs -> drop
    max_track_age_s: float = 120.0


class T1HoldEMA(TemporalEstimator):
    """Simple non-probabilistic temporal baseline: hold the last valid
    detection (optionally EMA-smoothed) for a bounded time during SILENCE;
    fresh-empty messages are evidence of absence and drop the track after a
    few consecutive misses. Downstream publication is fixed-rate."""

    name = "t1_hold_ema"

    def __init__(self, config: Optional[T1Config] = None) -> None:
        self.cfg = config or T1Config()
        self._est: Optional[Detection] = None
        self._t_created: Optional[float] = None
        self._t_meas: Optional[float] = None
        self._empty_streak = 0
        self._track_id = 0

    def on_message(self, event: DetectionEvent) -> None:
        if event.is_empty:
            self._empty_streak += 1
            if self._est is not None \
                    and self._empty_streak >= self.cfg.empty_misses_to_drop:
                self._drop("fresh_empty_streak")
            return
        self._empty_streak = 0
        d = max(event.detections, key=lambda x: x.confidence)
        a = self.cfg.ema_alpha
        if self._est is None:
            self._est = Detection(d.class_name, d.confidence, d.cx, d.cy,
                                  d.w, d.h)
            self._t_created = event.t
            self._track_id += 1
        else:
            e = self._est
            e.cx = a * d.cx + (1 - a) * e.cx
            e.cy = a * d.cy + (1 - a) * e.cy
            e.w = a * d.w + (1 - a) * e.w
            e.h = a * d.h + (1 - a) * e.h
            e.confidence = d.confidence
            e.class_name = d.class_name
        self._t_meas = event.t

    def _drop(self, reason: str) -> None:
        self._est = None
        self._t_created = None
        self._t_meas = None
        self._last_drop_reason = reason

    def tick(self, t: float) -> EstimatorOutput:
        if self._est is None or self._t_meas is None:
            return EstimatorOutput(publish=True, obstacles=[])
        since = t - self._t_meas
        age = t - (self._t_created or t)
        if since > self.cfg.hold_duration_s or age > self.cfg.max_track_age_s:
            self._drop("hold_expired" if since > self.cfg.hold_duration_s
                       else "max_age")
            return EstimatorOutput(publish=True, obstacles=[])
        e = self._est
        # 'Predicted/held' means genuinely bridging missing measurements
        # (>~3 frame intervals), not the ordinary sub-frame interleave
        # between message arrival and the output tick.
        return EstimatorOutput(publish=True, obstacles=[EstimatedObstacle(
            class_name=e.class_name, confidence=e.confidence,
            cx=e.cx, cy=e.cy, w=e.w, h=e.h,
            is_predicted=since > 0.1,
            track_id=self._track_id, age_s=age, time_since_meas_s=since,
            debug={"method": self.name, "held": since > 1e-3},
        )])


# ---------------------------------------------------------------------------
# T2 — fixed-noise Kalman filter (CV image-space model)
# ---------------------------------------------------------------------------

@dataclass
class KFConfig:
    # Process noise: white-acceleration PSD per axis pair (pos, size).
    q_center: float = 0.30    # (units/s^2)^2 spectral density for cx, cy
    q_logsize: float = 0.30   # for log w, log h
    # Fixed measurement noise std devs (normalized units / log units).
    r_center: float = 0.015
    r_logsize: float = 0.05
    # Initial velocity variance.
    p0_vel: float = 0.25
    gate_chi2: float = CHI2_GATE_4DOF_099
    # Tuned on the D-series DEVELOPMENT cases (max dropout D4 = 2.0 s);
    # silences longer than this still defeat T2 by design.
    max_prediction_horizon_s: float = 2.5
    empty_misses_to_drop: int = 3
    max_track_age_s: float = 120.0
    # Prediction confidence decay (reported confidence during dropout).
    conf_decay_per_s: float = 0.5


def cv_transition(dt: float) -> np.ndarray:
    F = np.eye(8)
    for i in range(4):
        F[i, i + 4] = dt
    return F


def cv_process_noise(dt: float, q_center: float, q_logsize: float) -> np.ndarray:
    """Discrete white-noise-acceleration Q for 4 independent CV pairs."""
    Q = np.zeros((8, 8))
    q11 = dt ** 3 / 3.0
    q12 = dt ** 2 / 2.0
    q22 = dt
    for i, q in enumerate([q_center, q_center, q_logsize, q_logsize]):
        Q[i, i] = q * q11
        Q[i, i + 4] = q * q12
        Q[i + 4, i] = q * q12
        Q[i + 4, i + 4] = q * q22
    return Q


H = np.hstack([np.eye(4), np.zeros((4, 4))])


class Track:
    _next_id = 1

    def __init__(self, det: Detection, t: float, cfg: KFConfig,
                 R0: np.ndarray) -> None:
        self.id = Track._next_id
        Track._next_id += 1
        self.class_name = det.class_name
        self.x = np.zeros(8)
        self.x[:4] = det.as_measurement()
        self.P = np.zeros((8, 8))
        self.P[:4, :4] = R0 * 4.0
        for i in range(4, 8):
            self.P[i, i] = cfg.p0_vel
        self.t_created = t
        self.t_meas = t
        self.t_state = t
        self.confidence = det.confidence
        self.empty_streak = 0
        self.last_innovation: Optional[np.ndarray] = None
        self.last_nis: Optional[float] = None
        self.gated_out_count = 0

    def predict_to(self, t: float, cfg: KFConfig) -> None:
        dt = t - self.t_state
        if dt <= 0:
            return
        F = cv_transition(dt)
        Q = cv_process_noise(dt, cfg.q_center, cfg.q_logsize)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t_state = t

    def update(self, det: Detection, t: float, R: np.ndarray,
               cfg: KFConfig) -> bool:
        """Predict to t then gated measurement update. Returns accepted."""
        self.predict_to(t, cfg)
        z = det.as_measurement()
        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        nis = float(innovation @ S_inv @ innovation)
        self.last_innovation = innovation
        self.last_nis = nis
        if nis > cfg.gate_chi2:
            self.gated_out_count += 1
            return False
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ innovation
        self.P = (np.eye(8) - K @ H) @ self.P
        self.t_meas = t
        self.confidence = det.confidence
        self.class_name = det.class_name
        self.empty_streak = 0
        return True

    def estimate(self, t: float) -> Tuple[float, float, float, float]:
        cx, cy, lw, lh = self.x[:4]
        return (float(cx), float(cy),
                float(math.exp(lw)), float(math.exp(lh)))


class T2FixedKalman(TemporalEstimator):
    """Fixed-Q/R linear Kalman on [cx, cy, log w, log h] + rates.

    Silence  -> predict-only up to max_prediction_horizon_s, then delete.
    Fresh-empty -> miss counter; N consecutive -> delete (absence evidence).
    Outliers -> chi-square innovation gating (do not update, keep predicting).
    """

    name = "t2_fixed_kf"

    def __init__(self, config: Optional[KFConfig] = None) -> None:
        self.cfg = config or KFConfig()
        self.track: Optional[Track] = None
        self.last_drop_reason = ""

    # -- noise model (overridden by T3) ------------------------------------
    def measurement_noise(self, det: Detection,
                          event: DetectionEvent) -> np.ndarray:
        c = self.cfg
        return np.diag([c.r_center ** 2, c.r_center ** 2,
                        c.r_logsize ** 2, c.r_logsize ** 2])

    def _drop(self, reason: str) -> None:
        self.track = None
        self.last_drop_reason = reason

    def on_message(self, event: DetectionEvent) -> None:
        if event.is_empty:
            if self.track is not None:
                self.track.empty_streak += 1
                if self.track.empty_streak >= self.cfg.empty_misses_to_drop:
                    self._drop("fresh_empty_streak")
            return
        det = max(event.detections, key=lambda d: d.confidence)
        if self.track is not None \
                and det.class_name != self.track.class_name:
            # Class inconsistency: treat as a different object; replace only
            # if the current track is already in prolonged dropout.
            since = event.t - self.track.t_meas
            if since > self.cfg.max_prediction_horizon_s / 2:
                self._drop("class_mismatch_replace")
            else:
                return  # ignore inconsistent detection (single-track phase)
        if self.track is None:
            R = self.measurement_noise(det, event)
            self.track = Track(det, event.t, self.cfg, R)
            return
        R = self.measurement_noise(det, event)
        self.track.update(det, event.t, R, self.cfg)

    def tick(self, t: float) -> EstimatorOutput:
        if self.track is None:
            return EstimatorOutput(publish=True, obstacles=[])
        tr = self.track
        since = t - tr.t_meas
        age = t - tr.t_created
        if since > self.cfg.max_prediction_horizon_s:
            self._drop("prediction_horizon_exceeded")
            return EstimatorOutput(publish=True, obstacles=[])
        if age > self.cfg.max_track_age_s:
            self._drop("max_age")
            return EstimatorOutput(publish=True, obstacles=[])
        tr.predict_to(t, self.cfg)
        cx, cy, w, h = tr.estimate(t)
        if not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
            self._drop("predicted_out_of_image")
            return EstimatorOutput(publish=True, obstacles=[])
        predicted = since > 0.1  # > ~3 frame intervals = real dropout
        conf = tr.confidence * max(
            0.0, 1.0 - self.cfg.conf_decay_per_s * max(0.0, since))
        return EstimatorOutput(publish=True, obstacles=[EstimatedObstacle(
            class_name=tr.class_name, confidence=conf,
            cx=cx, cy=cy, w=min(w, 1.0), h=min(h, 1.0),
            is_predicted=predicted, track_id=tr.id,
            age_s=age, time_since_meas_s=since,
            debug={
                "method": self.name,
                "nis": tr.last_nis,
                "innovation": (list(map(float, tr.last_innovation))
                               if tr.last_innovation is not None else None),
                "cov_diag": [float(v) for v in np.diag(tr.P)[:4]],
                "gated_out_count": tr.gated_out_count,
                "prediction_only": predicted,
            },
        )])


# ---------------------------------------------------------------------------
# T3 — adaptive measurement noise (calibration-driven framework)
# ---------------------------------------------------------------------------

def detection_features(det: Detection, event: DetectionEvent,
                       prev_event_t: Optional[float]) -> Dict[str, float]:
    """Observable quality features for the measurement-noise model.
    (Blur/contrast enter later when image-quality signals are wired in.)"""
    area = det.w * det.h
    border = min(det.cx - det.w / 2, det.cy - det.h / 2,
                 1.0 - (det.cx + det.w / 2), 1.0 - (det.cy + det.h / 2))
    return {
        "one_minus_conf": 1.0 - min(1.0, max(0.0, det.confidence)),
        "inv_sqrt_area": 1.0 / math.sqrt(max(area, 1e-6)),
        "border_proximity": max(0.0, -border) + max(0.0, 0.05 - max(border, 0.0)),
        "frame_interval": (event.t - prev_event_t) if prev_event_t else 0.0,
    }


FEATURE_ORDER = ["one_minus_conf", "inv_sqrt_area", "border_proximity",
                 "frame_interval"]


@dataclass
class AdaptiveNoiseModel:
    """log-linear R scaling: R = R0 * exp(theta . phi(features)).

    theta comes from empirical residual calibration
    (scripts/calibrate_visual_measurement_noise.py). Default theta = 0 means
    NO adaptation (T3 degenerates to T2) and calibrated = False. We do NOT
    invent coefficients — see docs/TEMPORAL_OBSTACLE_ESTIMATION.md.
    """

    theta_center: List[float] = field(
        default_factory=lambda: [0.0] * len(FEATURE_ORDER))
    theta_logsize: List[float] = field(
        default_factory=lambda: [0.0] * len(FEATURE_ORDER))
    calibrated: bool = False
    source: str = "default (uncalibrated: theta=0, T3 == T2)"
    max_scale: float = 25.0

    @classmethod
    def from_file(cls, path: str) -> "AdaptiveNoiseModel":
        with open(path) as f:
            d = json.load(f)
        return cls(
            theta_center=list(d["theta_center"]),
            theta_logsize=list(d["theta_logsize"]),
            calibrated=bool(d.get("calibrated", False)),
            source=str(d.get("source", path)),
            max_scale=float(d.get("max_scale", 25.0)),
        )

    def scales(self, features: Dict[str, float]) -> Tuple[float, float]:
        phi = np.array([features[k] for k in FEATURE_ORDER])
        sc = math.exp(float(np.dot(self.theta_center, phi)))
        ss = math.exp(float(np.dot(self.theta_logsize, phi)))
        clamp = lambda v: min(max(v, 1.0 / self.max_scale), self.max_scale)
        return clamp(sc), clamp(ss)


class T3AdaptiveKalman(T2FixedKalman):
    """T2 with feature-adaptive measurement covariance. Identical state
    model, gains, gating and lifecycle so the T2-vs-T3 ablation isolates
    exactly the measurement-noise adaptation."""

    name = "t3_adaptive_kf"

    def __init__(self, config: Optional[KFConfig] = None,
                 noise_model: Optional[AdaptiveNoiseModel] = None) -> None:
        super().__init__(config)
        self.noise_model = noise_model or AdaptiveNoiseModel()
        self._prev_event_t: Optional[float] = None
        self.last_features: Optional[Dict[str, float]] = None
        self.last_scales: Tuple[float, float] = (1.0, 1.0)

    def measurement_noise(self, det: Detection,
                          event: DetectionEvent) -> np.ndarray:
        feats = detection_features(det, event, self._prev_event_t)
        self.last_features = feats
        sc, ss = self.noise_model.scales(feats)
        self.last_scales = (sc, ss)
        c = self.cfg
        return np.diag([
            (c.r_center ** 2) * sc, (c.r_center ** 2) * sc,
            (c.r_logsize ** 2) * ss, (c.r_logsize ** 2) * ss,
        ])

    def on_message(self, event: DetectionEvent) -> None:
        super().on_message(event)
        self._prev_event_t = event.t

    def tick(self, t: float) -> EstimatorOutput:
        out = super().tick(t)
        for ob in out.obstacles:
            ob.debug["method"] = self.name
            ob.debug["noise_calibrated"] = self.noise_model.calibrated
            ob.debug["noise_source"] = self.noise_model.source
            ob.debug["noise_scales"] = list(self.last_scales)
            ob.debug["features"] = self.last_features
        return out


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def make_estimator(method: str,
                   noise_model_path: Optional[str] = None) -> TemporalEstimator:
    method = method.lower()
    if method in ("t0", "t0_raw", "raw"):
        return T0Raw()
    if method in ("t1", "t1_hold_ema", "hold"):
        return T1HoldEMA()
    if method in ("t2", "t2_fixed_kf", "kf"):
        return T2FixedKalman()
    if method in ("t3", "t3_adaptive_kf", "adaptive"):
        nm = (AdaptiveNoiseModel.from_file(noise_model_path)
              if noise_model_path else AdaptiveNoiseModel())
        return T3AdaptiveKalman(noise_model=nm)
    raise ValueError(f"unknown estimator method {method!r}")
