"""Twist (SI, ROS frame) -> ArduSub MANUAL_CONTROL (counts, vehicle frame).

This module is the ONLY place where the scientific command abstraction
crosses into this particular vehicle's control interface. It is pure
Python (no rclpy, no pymavlink) so every rule below is unit-testable
without hardware.

FRAMES AND SIGNS
----------------
ROS body frame (REP-103), the frame of /planner/cmd_vel_safe:
    linear.x  > 0  forward (surge)
    linear.y  > 0  to PORT / LEFT (sway)
    angular.z > 0  counter-clockwise seen from above (yaw, to port)

ArduSub MANUAL_CONTROL, verified in water on this vehicle 2026-08-14
(see docs/real_vehicle_reference/README.md):
    x > 0  forward
    y > 0  to STARBOARD / RIGHT
    r > 0  CLOCKWISE (to starboard)
    z      heave, 0..1000 with 500 neutral (NOT used: ALT_HOLD owns depth)

Therefore sway and yaw require an EXPLICIT sign inversion. It is written
out here as a named constant rather than buried in a negative gain, so a
reader can check it against the frame definitions above:

    x_counts = +f_surge(linear.x)
    y_counts = -f_sway (linear.y)      # ROS left -> MC right
    r_counts = -f_yaw  (angular.z)     # ROS CCW  -> MC CW

FAIL-CLOSED CONTRACT
--------------------
Every actuator input must fail closed. `twist_to_manual_control` rejects
non-finite values (NaN, +Inf, -Inf) BEFORE any clamping and returns the
neutral command with a reason. This is not hypothetical: Python's
`min(0.3, float('nan'))` returns 0.3, so the naive clamp
`max(-limit, min(limit, v))` turns a NaN into FULL FORWARD surge, while
the simulator's `max(lo, min(v, hi))` turns the same NaN into FULL
REVERSE. A silent, opposite-sign divergence between the two domains.

DEADBAND
--------
The measured vehicle does not move below a command threshold (tether
drag and stiction): commands under `min_command` produce thruster noise
but no motion. Emitting them would also corrupt any dead-reckoning that
integrates the commanded velocity. The mapping therefore returns exactly
zero below the per-axis minimum and reports `deadband_applied`, instead
of emitting a command the vehicle cannot execute.

The affine form above the deadband is

    counts = sign(v) * (db + |v| / k)

with `db` the deadband in counts and `k` the SI-units-per-count slope
above it, both estimated per axis AND PER SIGN (the measured yaw
asymmetry is a factor ~2, so a symmetric model is not defensible).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

MC_MIN = -1000
MC_MAX = 1000
Z_NEUTRAL = 500

# Explicit frame conversion, see module docstring.
SWAY_ROS_TO_MC = -1.0
YAW_ROS_TO_MC = -1.0


@dataclass
class AxisCalibration:
    """Signed command mapping for one degree of freedom.

    `k_pos`/`k_neg` are SI units per count ABOVE the deadband, for the
    positive and negative ROS direction respectively; `db_pos`/`db_neg`
    are the deadband sizes in counts. `min_command` is the smallest SI
    magnitude the vehicle can be trusted to execute; below it the axis
    is commanded to zero.

    Defaults are deliberately UNCALIBRATED placeholders flagged by
    `calibrated=False`: a node must refuse LIVE actuation with an
    uncalibrated axis.
    """

    k_pos: float = 1.0e-3
    k_neg: float = 1.0e-3
    db_pos: float = 0.0
    db_neg: float = 0.0
    min_command: float = 0.0
    max_command: float = 1.0
    calibrated: bool = False
    source: str = "uncalibrated placeholder"

    def to_counts(self, value: float) -> Tuple[int, bool]:
        """SI value -> counts. Returns (counts, deadband_applied)."""
        if not math.isfinite(value):
            return 0, True
        v = max(-self.max_command, min(self.max_command, value))
        if abs(v) < self.min_command or v == 0.0:
            return 0, abs(v) > 0.0
        if v > 0:
            counts = self.db_pos + v / self.k_pos
        else:
            counts = -(self.db_neg + (-v) / self.k_neg)
        counts = int(round(max(MC_MIN, min(MC_MAX, counts))))
        return counts, False


@dataclass
class CommandMapping:
    """Full Twist -> MANUAL_CONTROL mapping for this vehicle."""

    surge: AxisCalibration = field(default_factory=AxisCalibration)
    sway: AxisCalibration = field(default_factory=AxisCalibration)
    yaw: AxisCalibration = field(default_factory=AxisCalibration)
    calibration_id: str = "uncalibrated"

    @property
    def fully_calibrated(self) -> bool:
        return all(a.calibrated for a in (self.surge, self.sway, self.yaw))

    def uncalibrated_axes(self):
        return [n for n, a in (("surge", self.surge), ("sway", self.sway),
                               ("yaw", self.yaw)) if not a.calibrated]


NEUTRAL = {"x": 0, "y": 0, "z": Z_NEUTRAL, "r": 0}


def twist_to_manual_control(linear_x: float, linear_y: float,
                            angular_z: float,
                            mapping: CommandMapping) -> Dict:
    """Convert one Twist to MANUAL_CONTROL counts, failing closed.

    Returns a dict with the counts, the applied conversions and the
    reason for any rejection, so the caller can log exactly what the
    vehicle was told and why.
    """
    values = {"linear_x": linear_x, "linear_y": linear_y,
              "angular_z": angular_z}
    bad = [k for k, v in values.items()
           if not isinstance(v, (int, float)) or not math.isfinite(v)]
    if bad:
        return {**NEUTRAL, "accepted": False,
                "reason": f"non-finite command component(s): {bad}",
                "deadband": [], "input": values}

    x_counts, x_db = mapping.surge.to_counts(linear_x)
    y_si = SWAY_ROS_TO_MC * linear_y      # ROS left -> MC right
    y_counts, y_db = mapping.sway.to_counts(y_si)
    r_si = YAW_ROS_TO_MC * angular_z      # ROS CCW -> MC CW
    r_counts, r_db = mapping.yaw.to_counts(r_si)

    deadband = [n for n, applied in (("surge", x_db), ("sway", y_db),
                                     ("yaw", r_db)) if applied]
    return {"x": x_counts, "y": y_counts, "z": Z_NEUTRAL, "r": r_counts,
            "accepted": True, "reason": "ok", "deadband": deadband,
            "input": values,
            "saturated": [n for n, c in (("surge", x_counts),
                                         ("sway", y_counts),
                                         ("yaw", r_counts))
                          if abs(c) >= MC_MAX]}
