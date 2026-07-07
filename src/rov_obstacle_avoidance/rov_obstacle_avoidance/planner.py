"""Deterministic local obstacle avoidance planner logic.

The planner is stateful and body-frame by default, but it is also
*pose-aware*: when the wrapping node feeds it the vehicle pose it captures the
original straight path (position + heading) the vehicle was following before it
had to avoid, and drives a closed-loop recovery that returns the vehicle to
that path -- not merely to body-frame forward motion.  Avoidance therefore
prefers lateral (sway) motion with a small, limited yaw drift, and any residual
heading/lateral error accumulated during the maneuver is actively nulled during
recovery.

If no pose is provided the planner degrades gracefully to the previous
time-based recovery blend so it remains usable without odometry.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class PlannerState(str, Enum):
    NORMAL = "NORMAL"
    APPROACH_OBSTACLE = "APPROACH_OBSTACLE"
    AVOIDING_LEFT = "AVOIDING_LEFT"
    AVOIDING_RIGHT = "AVOIDING_RIGHT"
    RECOVERING = "RECOVERING"


class AvoidanceSide(str, Enum):
    NONE = "none"
    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True)
class ObstacleObservation:
    class_name: str = "obstacle"
    confidence: float = 0.0
    center_x: float = 0.5
    center_y: float = 0.5
    width: float = 0.0
    height: float = 0.0
    bearing_rad: float = 0.0
    apparent_area: float = 0.0
    risk: float = 0.0
    is_tracking_valid: bool = True


@dataclass(frozen=True)
class VelocityCommand:
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class VehiclePose:
    """Planar world-frame pose used for path-relative recovery."""

    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class PlannerConfig:
    risk_enter_threshold: float = 0.55
    risk_exit_threshold: float = 0.30
    central_zone_min_x: float = 0.35
    central_zone_max_x: float = 0.65
    min_avoidance_hold_s: float = 1.0
    recovery_time_s: float = 2.0
    max_surge: float = 0.5
    min_surge_during_avoidance: float = 0.08
    avoidance_sway: float = 0.20
    # Limited yaw drift during avoidance: prefer lateral (sway) avoidance and
    # keep the heading close to the original path so recovery is a short lateral
    # return rather than a large heading correction.  Sway (above) stays the
    # dominant avoidance action; this small yaw only helps clear the obstacle
    # from view and any residual drift is nulled during recovery.
    avoidance_yaw_rate: float = 0.1
    command_timeout_s: float = 1.0
    nominal_timeout_behavior: str = "stop"
    # Pose-aware recovery gains/limits.  When a reference path and pose are
    # available the vehicle is steered back onto the original line and heading.
    recovery_lateral_gain: float = 0.8
    recovery_yaw_gain: float = 1.0
    recovery_max_sway: float = 0.25
    recovery_max_yaw_rate: float = 0.30
    recovery_lateral_tolerance_m: float = 0.20
    recovery_yaw_tolerance_deg: float = 5.0
    recovery_max_time_s: float = 15.0
    # Minimum forward command magnitude required before the planner locks in the
    # "original path" reference (avoids anchoring the line while idle).
    forward_reference_min_surge: float = 0.02


@dataclass(frozen=True)
class PlannerOutput:
    command: VelocityCommand
    state: PlannerState
    selected_side: AvoidanceSide
    risk: float
    cross_track_error_m: float = 0.0
    yaw_error_rad: float = 0.0


CENTRALITY_RISK_WEIGHT = 0.6
AREA_RISK_WEIGHT = 0.4
AREA_RISK_GAIN = 4.0
# A nominal command with yaw-rate / sway above this is the operator deliberately
# steering, so the "original path" reference is re-anchored to follow it.
STEER_EPS = 1e-3
CLASS_RISK_WEIGHTS = {
    "anchor": 1.25,
    "gate": 1.15,
    "box_obstacle": 1.10,
    "box": 1.05,
    "sphere": 1.00,
    "unknown_obstacle": 1.00,
}


class LocalAvoidancePlanner:
    """Stateful obstacle avoidance planner that outputs a safe velocity command."""

    def __init__(self, config: PlannerConfig | None = None) -> None:
        self.config = config or PlannerConfig()
        self.state = PlannerState.NORMAL
        self.selected_side = AvoidanceSide.NONE
        self._nominal_command = VelocityCommand()
        self._last_nominal_time_s: float | None = None
        self._obstacles: list[ObstacleObservation] = []
        self._last_obstacle_time_s: float | None = None
        self._side_selected_time_s: float | None = None
        self._recovery_start_time_s: float | None = None
        # Pose-aware recovery state.
        self._latest_pose: VehiclePose | None = None
        self._reference_path: VehiclePose | None = None

    def update_nominal_command(self, command: VelocityCommand, now_s: float) -> None:
        self._nominal_command = command
        self._last_nominal_time_s = now_s

    def update_obstacles(self, obstacles: list[ObstacleObservation], now_s: float) -> None:
        self._obstacles = obstacles
        self._last_obstacle_time_s = now_s

    def update_pose(self, x: float, y: float, yaw_rad: float, now_s: float) -> None:
        """Feed the latest planar world pose used for path-relative recovery."""
        self._latest_pose = VehiclePose(
            x=_finite_or_zero(x),
            y=_finite_or_zero(y),
            yaw_rad=_finite_or_zero(yaw_rad),
        )

    @property
    def reference_path(self) -> VehiclePose | None:
        return self._reference_path

    def compute(self, now_s: float) -> PlannerOutput:
        nominal = self._current_nominal(now_s)
        obstacle, risk = self._most_dangerous_obstacle(now_s)
        dangerous = obstacle is not None and risk >= self.config.risk_enter_threshold
        clear = obstacle is None or risk < self.config.risk_exit_threshold

        # While cruising normally, keep the "original path" reference anchored to
        # where the vehicle is and which way it is heading.  It freezes the
        # instant we leave NORMAL, capturing the path to return to.
        self._maybe_capture_reference(nominal)

        if self.state == PlannerState.NORMAL:
            if dangerous:
                self._select_side(obstacle, now_s)
                self.state = PlannerState.APPROACH_OBSTACLE
            else:
                return self._normal_output(nominal, risk)

        elif self.state == PlannerState.APPROACH_OBSTACLE:
            if dangerous:
                self.state = self._state_for_selected_side()
            elif clear and self._hold_elapsed(now_s):
                self._start_recovery(now_s)

        elif self.state in {PlannerState.AVOIDING_LEFT, PlannerState.AVOIDING_RIGHT}:
            if dangerous and self._hold_elapsed(now_s):
                self._select_side(obstacle, now_s)
                self.state = self._state_for_selected_side()
            elif clear and self._hold_elapsed(now_s):
                self._start_recovery(now_s)

        elif self.state == PlannerState.RECOVERING:
            if dangerous:
                self._select_side(obstacle, now_s)
                self.state = PlannerState.APPROACH_OBSTACLE
                self._recovery_start_time_s = None

        if self.state == PlannerState.RECOVERING:
            return self._recovery_output(nominal, now_s, risk)

        command = self._avoidance_command(nominal)
        cross, yaw_err = self._path_errors_or_zero()
        return PlannerOutput(command, self.state, self.selected_side, risk, cross, yaw_err)

    # -- inputs / bookkeeping ------------------------------------------------
    def _current_nominal(self, now_s: float) -> VelocityCommand:
        if self._last_nominal_time_s is None:
            return VelocityCommand()
        if now_s - self._last_nominal_time_s > self.config.command_timeout_s:
            if self.config.nominal_timeout_behavior.strip().lower() in {"hold", "hold_last"}:
                return _sanitize_command(self._nominal_command, self.config.max_surge)
            return VelocityCommand()
        return _sanitize_command(self._nominal_command, self.config.max_surge)

    def _most_dangerous_obstacle(
        self,
        now_s: float,
    ) -> tuple[ObstacleObservation | None, float]:
        if self._last_obstacle_time_s is None:
            return None, 0.0
        if now_s - self._last_obstacle_time_s > self.config.command_timeout_s:
            return None, 0.0

        best_obstacle: ObstacleObservation | None = None
        best_risk = 0.0
        for obstacle in self._obstacles:
            if not obstacle.is_tracking_valid:
                continue
            risk = compute_obstacle_risk(obstacle)
            if best_obstacle is None or risk > best_risk:
                best_obstacle = obstacle
                best_risk = risk
        return best_obstacle, best_risk

    def _select_side(self, obstacle: ObstacleObservation, now_s: float) -> None:
        desired_side = choose_avoidance_side(
            obstacle,
            self.config.central_zone_min_x,
            self.config.central_zone_max_x,
        )
        if self.selected_side == AvoidanceSide.NONE or self._hold_elapsed(now_s):
            self.selected_side = desired_side
            self._side_selected_time_s = now_s

    def _hold_elapsed(self, now_s: float) -> bool:
        if self._side_selected_time_s is None:
            return True
        return now_s - self._side_selected_time_s >= self.config.min_avoidance_hold_s

    def _state_for_selected_side(self) -> PlannerState:
        if self.selected_side == AvoidanceSide.RIGHT:
            return PlannerState.AVOIDING_RIGHT
        return PlannerState.AVOIDING_LEFT

    def _start_recovery(self, now_s: float) -> None:
        self.state = PlannerState.RECOVERING
        self._recovery_start_time_s = now_s

    def _complete_recovery(self) -> None:
        self.state = PlannerState.NORMAL
        self.selected_side = AvoidanceSide.NONE
        self._side_selected_time_s = None
        self._recovery_start_time_s = None

    # -- reference path (original straight line) -----------------------------
    def _maybe_capture_reference(self, nominal: VelocityCommand) -> None:
        if self.state != PlannerState.NORMAL or self._latest_pose is None:
            return
        if nominal.surge <= self.config.forward_reference_min_surge:
            return
        operator_steering = (
            abs(nominal.yaw_rate) > STEER_EPS or abs(nominal.sway) > STEER_EPS
        )
        # Lock the original straight path once; only re-anchor it if the operator
        # is actively steering (so we follow a deliberate course change rather
        # than fight it, and so a residual offset after recovery is NOT forgiven).
        if self._reference_path is None or operator_steering:
            self._reference_path = self._latest_pose

    def _path_errors(self, pose: VehiclePose) -> tuple[float, float]:
        """Signed cross-track error (m, +left) and yaw error (rad) vs the path."""
        assert self._reference_path is not None
        ref = self._reference_path
        dx = pose.x - ref.x
        dy = pose.y - ref.y
        # REP-103 body/world: +y is left; path-left unit vector is (-sin, cos).
        left_x = -math.sin(ref.yaw_rad)
        left_y = math.cos(ref.yaw_rad)
        cross = dx * left_x + dy * left_y
        yaw_err = _wrap_to_pi(pose.yaw_rad - ref.yaw_rad)
        return cross, yaw_err

    def _path_errors_or_zero(self) -> tuple[float, float]:
        if self._reference_path is None or self._latest_pose is None:
            return 0.0, 0.0
        return self._path_errors(self._latest_pose)

    # -- command generation --------------------------------------------------
    def _normal_output(self, nominal: VelocityCommand, risk: float) -> PlannerOutput:
        """NORMAL cruise: hold the original line when pose is available."""
        if self._reference_path is not None and self._latest_pose is not None:
            cross, yaw_err = self._path_errors(self._latest_pose)
            command = self._line_keeping_command(nominal, self._latest_pose, cross, yaw_err)
            return PlannerOutput(
                command, PlannerState.NORMAL, self.selected_side, risk, cross, yaw_err
            )
        return self._passthrough_output(nominal, risk)

    def _passthrough_output(self, nominal: VelocityCommand, risk: float) -> PlannerOutput:
        cross, yaw_err = self._path_errors_or_zero()
        return PlannerOutput(
            nominal, self.state, self.selected_side, risk, cross, yaw_err
        )

    def _avoidance_command(self, nominal: VelocityCommand) -> VelocityCommand:
        side_sign = 1.0 if self.selected_side == AvoidanceSide.LEFT else -1.0
        max_surge = abs(_finite_or_zero(self.config.max_surge))
        surge = _clamp(nominal.surge, -max_surge, max_surge)
        if surge > self.config.min_surge_during_avoidance:
            surge = self.config.min_surge_during_avoidance
        sway = side_sign * abs(_finite_or_zero(self.config.avoidance_sway))
        yaw_rate = side_sign * abs(_finite_or_zero(self.config.avoidance_yaw_rate))
        return VelocityCommand(
            surge=surge,
            sway=sway,
            heave=nominal.heave,
            roll_rate=nominal.roll_rate,
            pitch_rate=nominal.pitch_rate,
            yaw_rate=yaw_rate,
        )

    def _recovery_output(
        self, nominal: VelocityCommand, now_s: float, risk: float
    ) -> PlannerOutput:
        if self._recovery_start_time_s is None:
            self._recovery_start_time_s = now_s

        if self._reference_path is not None and self._latest_pose is not None:
            cross, yaw_err = self._path_errors(self._latest_pose)
            elapsed = now_s - self._recovery_start_time_s
            yaw_tol = math.radians(self.config.recovery_yaw_tolerance_deg)
            converged = (
                abs(cross) <= self.config.recovery_lateral_tolerance_m
                and abs(yaw_err) <= yaw_tol
            )
            timed_out = (
                self.config.recovery_max_time_s > 0.0
                and elapsed >= self.config.recovery_max_time_s
            )
            if converged or timed_out:
                self._complete_recovery()
                return PlannerOutput(
                    nominal, self.state, self.selected_side, risk, cross, yaw_err
                )
            command = self._line_keeping_command(nominal, self._latest_pose, cross, yaw_err)
            return PlannerOutput(
                command, PlannerState.RECOVERING, self.selected_side, risk, cross, yaw_err
            )

        # No pose: fall back to the legacy time-based blend to nominal.
        recovered = self._recovery_blend(nominal, now_s)
        if recovered is not None:
            return PlannerOutput(
                recovered, PlannerState.RECOVERING, self.selected_side, risk, 0.0, 0.0
            )
        self._complete_recovery()
        return PlannerOutput(nominal, self.state, self.selected_side, risk, 0.0, 0.0)

    def _line_keeping_command(
        self,
        nominal: VelocityCommand,
        pose: VehiclePose,
        cross: float,
        yaw_err: float,
    ) -> VelocityCommand:
        """Steer onto/along the original line and heading (holonomic control).

        Used both to recover after avoidance and to hold the line during normal
        cruise; when the vehicle is already on the line the corrections vanish
        and the command equals the nominal forward command.
        """
        assert self._reference_path is not None
        ref = self._reference_path
        max_surge = abs(_finite_or_zero(self.config.max_surge))
        forward = _clamp(nominal.surge, -max_surge, max_surge)

        # Correction velocity (world frame) toward the line, along path-left.
        lat_corr = _clamp(
            -self.config.recovery_lateral_gain * cross,
            -self.config.recovery_max_sway,
            self.config.recovery_max_sway,
        )
        fx = math.cos(ref.yaw_rad)
        fy = math.sin(ref.yaw_rad)
        lx = -math.sin(ref.yaw_rad)
        ly = math.cos(ref.yaw_rad)
        world_vx = forward * fx + lat_corr * lx
        world_vy = forward * fy + lat_corr * ly

        # Rotate the desired world velocity into the current body frame.
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        surge = world_vx * cy + world_vy * sy
        sway = -world_vx * sy + world_vy * cy

        surge = _clamp(surge, -max_surge, max_surge)
        sway = _clamp(sway, -max_surge, max_surge)
        yaw_rate = _clamp(
            -self.config.recovery_yaw_gain * yaw_err,
            -self.config.recovery_max_yaw_rate,
            self.config.recovery_max_yaw_rate,
        )
        return VelocityCommand(
            surge=surge,
            sway=sway,
            heave=nominal.heave,
            roll_rate=nominal.roll_rate,
            pitch_rate=nominal.pitch_rate,
            yaw_rate=yaw_rate,
        )

    def _recovery_blend(self, nominal: VelocityCommand, now_s: float) -> VelocityCommand | None:
        if self._recovery_start_time_s is None:
            self._recovery_start_time_s = now_s
        if self.config.recovery_time_s <= 0.0:
            return None

        alpha = (now_s - self._recovery_start_time_s) / self.config.recovery_time_s
        if alpha >= 1.0:
            return None
        alpha = _clamp(alpha, 0.0, 1.0)
        avoidance = self._avoidance_command(nominal)
        return VelocityCommand(
            surge=_blend(avoidance.surge, nominal.surge, alpha),
            sway=_blend(avoidance.sway, nominal.sway, alpha),
            heave=nominal.heave,
            roll_rate=nominal.roll_rate,
            pitch_rate=nominal.pitch_rate,
            yaw_rate=_blend(avoidance.yaw_rate, nominal.yaw_rate, alpha),
        )


def compute_obstacle_risk(obstacle: ObstacleObservation) -> float:
    if obstacle.risk > 0.0:
        return _clamp(obstacle.risk * _class_risk_weight(obstacle.class_name), 0.0, 1.0)

    confidence = _clamp(obstacle.confidence, 0.0, 1.0)
    centrality = _clamp(1.0 - abs(_clamp(obstacle.center_x, 0.0, 1.0) - 0.5) * 2.0, 0.0, 1.0)
    apparent_area = obstacle.apparent_area if obstacle.apparent_area > 0.0 else obstacle.width * obstacle.height
    area_score = _clamp(apparent_area * AREA_RISK_GAIN, 0.0, 1.0)
    return _clamp(
        confidence
        * _class_risk_weight(obstacle.class_name)
        * (CENTRALITY_RISK_WEIGHT * centrality + AREA_RISK_WEIGHT * area_score),
        0.0,
        1.0,
    )


def choose_avoidance_side(
    obstacle: ObstacleObservation,
    central_zone_min_x: float,
    central_zone_max_x: float,
) -> AvoidanceSide:
    if obstacle.center_x < central_zone_min_x:
        return AvoidanceSide.RIGHT
    if obstacle.center_x > central_zone_max_x:
        return AvoidanceSide.LEFT

    left_edge = _clamp(obstacle.center_x - obstacle.width * 0.5, 0.0, 1.0)
    right_edge = _clamp(obstacle.center_x + obstacle.width * 0.5, 0.0, 1.0)
    left_space = left_edge
    right_space = 1.0 - right_edge
    if left_space >= right_space:
        return AvoidanceSide.LEFT
    return AvoidanceSide.RIGHT


def _blend(start: float, end: float, alpha: float) -> float:
    return start + (end - start) * alpha


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _sanitize_command(command: VelocityCommand, max_surge: float) -> VelocityCommand:
    max_surge = abs(_finite_or_zero(max_surge))
    return VelocityCommand(
        surge=_clamp(_finite_or_zero(command.surge), -max_surge, max_surge),
        sway=_finite_or_zero(command.sway),
        heave=_finite_or_zero(command.heave),
        roll_rate=_finite_or_zero(command.roll_rate),
        pitch_rate=_finite_or_zero(command.pitch_rate),
        yaw_rate=_finite_or_zero(command.yaw_rate),
    )


def _finite_or_zero(value: float) -> float:
    value = float(value)
    return value if math.isfinite(value) else 0.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _class_risk_weight(class_name: str) -> float:
    return CLASS_RISK_WEIGHTS.get(class_name.strip().lower(), 1.0)
