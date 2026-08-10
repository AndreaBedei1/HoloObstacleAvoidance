"""Transferable no-DVL odometry: commanded-motion dead reckoning.

The real BlueROV2 has NO DVL and no velocity sensor (hardware inventory
2026-08-10). This estimator therefore uses ONLY quantities the real vehicle
can provide, with identical conceptual inputs in simulation and reality:

  * measured attitude (yaw)      — real: ArduSub ATTITUDE (EKF/compass);
                                   sim: /rov/attitude_measured
  * measured pressure depth      — real: SCALED_PRESSURE2; sim: /rov/depth
  * commanded body velocity      — /planner/cmd_vel_safe in both domains

Body velocity is ESTIMATED (not measured) by passing the commanded velocity
through a first-order vehicle-response model (time constants from the
step-response identification, docs/BLUEROV2_SIMULATION_DYNAMICS.md), then
integrated with the measured yaw into x/y. Depth is a direct measurement.

Explicitly FORBIDDEN inputs (enforced by design — this module has no access
to them): simulator ground-truth x/y, simulator VelocitySensor, oracle
vehicle pose, external RealSense. The sim VelocitySensor is used ONLY inside
the low-level simulated PI thruster controller (part of the simulated plant),
never here.

Expected drift: dead reckoning drifts with response-model error and yaw
error; the committed-circumnavigation planner was designed to tolerate
short-horizon drift. Quantifying this drift versus ground truth is a
validator task (odometry error metrics in Scientific Baseline 0).

Pure Python module (no rclpy) for unit testing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple


FORBIDDEN_TOPIC_SUBSTRINGS = (
    "ground_truth", "pose_ground", "oracle", "/rov/velocity",
)


def assert_topics_allowed(topics) -> None:
    """Raise if any runtime input topic is a forbidden (non-transferable)
    source: ground truth, oracle, or the synthetic-DVL velocity stream."""
    for t in topics:
        for bad in FORBIDDEN_TOPIC_SUBSTRINGS:
            if bad in t:
                raise RuntimeError(
                    f"commanded_odometry must not consume {t!r}: ground-truth"
                    "/oracle/synthetic-DVL inputs are forbidden in the "
                    "transferable estimator"
                )


@dataclass
class CommandedOdometryConfig:
    # First-order response time constants [s], from the S0 step responses
    # (10-90% rise ≈ 3.0/2.9/2.0 s incl. the 0.4 s setpoint ramp -> tau ≈
    # (rise - ramp)/2.2).
    tau_surge_s: float = 1.2
    tau_sway_s: float = 1.15
    tau_heave_s: float = 0.8
    # Commanded velocities are trusted only while fresh; a stale command means
    # the vehicle-side watchdog has zeroed the real command, so the model
    # target decays to zero too.
    cmd_timeout_s: float = 1.0
    # Reject non-finite or absurd inputs.
    max_speed_mps: float = 2.0


@dataclass
class CommandedOdometry:
    """Dead-reckoned planar pose from commanded motion + measured yaw/depth."""

    config: CommandedOdometryConfig = field(default_factory=CommandedOdometryConfig)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    # Model-estimated body velocity (first-order response state).
    v_surge: float = 0.0
    v_sway: float = 0.0
    v_heave: float = 0.0
    _last_cmd: Optional[Tuple[float, float, float]] = None
    _last_cmd_time: Optional[float] = None
    _have_yaw: bool = False

    def set_command(self, surge: float, sway: float, heave: float,
                    now_s: float) -> None:
        c = self.config
        vals = []
        for v in (surge, sway, heave):
            v = float(v)
            if not math.isfinite(v):
                v = 0.0
            vals.append(max(-c.max_speed_mps, min(c.max_speed_mps, v)))
        self._last_cmd = (vals[0], vals[1], vals[2])
        self._last_cmd_time = float(now_s)

    def set_measured_yaw(self, yaw_rad: float) -> None:
        if math.isfinite(yaw_rad):
            self.yaw = float(yaw_rad)
            self._have_yaw = True

    def set_measured_depth(self, z: float) -> None:
        if math.isfinite(z):
            self.z = float(z)

    def _target(self, now_s: float) -> Tuple[float, float, float]:
        if self._last_cmd is None or self._last_cmd_time is None:
            return (0.0, 0.0, 0.0)
        if now_s - self._last_cmd_time > self.config.cmd_timeout_s:
            return (0.0, 0.0, 0.0)
        return self._last_cmd

    def update(self, dt: float, now_s: float) -> None:
        """Advance the response model and integrate the planar position."""
        if dt <= 0.0 or not math.isfinite(dt):
            return
        c = self.config
        tgt = self._target(now_s)
        self.v_surge += (tgt[0] - self.v_surge) * min(1.0, dt / c.tau_surge_s)
        self.v_sway += (tgt[1] - self.v_sway) * min(1.0, dt / c.tau_sway_s)
        self.v_heave += (tgt[2] - self.v_heave) * min(1.0, dt / c.tau_heave_s)

        # Integrate with the MEASURED yaw (transferable attitude), x fwd,
        # y left (REP-103 / HoloOcean client frame).
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.x += (cos_y * self.v_surge - sin_y * self.v_sway) * dt
        self.y += (sin_y * self.v_surge + cos_y * self.v_sway) * dt
        # z comes from the depth MEASUREMENT (set_measured_depth), not from
        # integration; v_heave exists only for completeness/diagnostics.

    def pose(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "yaw": self.yaw}
