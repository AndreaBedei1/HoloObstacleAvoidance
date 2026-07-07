"""Deterministic local obstacle avoidance planner logic.

The planner performs a *committed circumnavigation* rather than a reactive
wobble:

  1. NORMAL      -- cruise, holding the original straight path (line-keeping on
                    the estimated odometry).
  2. APPROACH    -- an obstacle is detected ahead; keep closing straight while
                    estimating its RANGE (monocular: bbox size + known target
                    height + camera FOV -- realistic sensors only).
  3. AVOIDING    -- once within engage distance, commit to one side, strafe out
                    to a clearing offset, then run PARALLEL to the path past the
                    obstacle.  The decision that the obstacle has been passed is
                    driven by ODOMETRY forward-progress past the estimated
                    obstacle position (not by loss of detection), so a detection
                    dropout can never make the vehicle turn back into it.
  4. RECOVERING  -- once passed, return to the original line and heading (a
                    diagonal return that keeps moving forward), then NORMAL.

This avoids both failure modes: the continuous left/right oscillation of a
reactive controller, and driving into the obstacle when the detector blinks out.

Pose here is the ESTIMATED odometry (drifting, dead-reckoned from DVL+gyro), not
simulator ground truth.  Without pose the planner degrades to a simpler
body-frame fallback so it stays usable.
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
    """Planar world/odom-frame pose used for path-relative navigation."""

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
    avoidance_yaw_rate: float = 0.1
    command_timeout_s: float = 1.0
    nominal_timeout_behavior: str = "stop"
    # Pose-aware recovery / line-keeping gains and limits.
    recovery_lateral_gain: float = 0.8
    recovery_yaw_gain: float = 1.0
    recovery_max_sway: float = 0.25
    recovery_max_yaw_rate: float = 0.30
    recovery_lateral_tolerance_m: float = 0.20
    recovery_yaw_tolerance_deg: float = 5.0
    recovery_max_time_s: float = 15.0
    forward_reference_min_surge: float = 0.02
    # Monocular range estimation (realistic: known target height + camera FOV).
    camera_vertical_fov_deg: float = 90.0
    target_obstacle_height_m: float = 3.5
    max_range_m: float = 40.0
    # Committed circumnavigation.
    engage_distance_m: float = 9.0
    clearance_offset_m: float = 2.5
    pass_margin_m: float = 4.0
    go_around_surge: float = 0.4
    go_around_max_sway: float = 0.3
    offset_reached_tol_m: float = 0.4
    ahead_bearing_deg: float = 55.0


@dataclass(frozen=True)
class PlannerOutput:
    command: VelocityCommand
    state: PlannerState
    selected_side: AvoidanceSide
    risk: float
    cross_track_error_m: float = 0.0
    yaw_error_rad: float = 0.0
    estimated_range_m: float = 0.0


CENTRALITY_RISK_WEIGHT = 0.6
AREA_RISK_WEIGHT = 0.4
AREA_RISK_GAIN = 4.0
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
        self._recovery_start_time_s: float | None = None
        # Pose / path state.
        self._latest_pose: VehiclePose | None = None
        self._reference_path: VehiclePose | None = None
        # Committed-maneuver state.
        self._commit_offset_m: float = 0.0
        self._anchor_along_m: float | None = None
        self._commit_time_s: float | None = None

    # -- inputs --------------------------------------------------------------
    def update_nominal_command(self, command: VelocityCommand, now_s: float) -> None:
        self._nominal_command = command
        self._last_nominal_time_s = now_s

    def update_obstacles(self, obstacles: list[ObstacleObservation], now_s: float) -> None:
        self._obstacles = obstacles
        self._last_obstacle_time_s = now_s

    def update_pose(self, x: float, y: float, yaw_rad: float, now_s: float) -> None:
        self._latest_pose = VehiclePose(
            x=_finite_or_zero(x), y=_finite_or_zero(y), yaw_rad=_finite_or_zero(yaw_rad)
        )

    @property
    def reference_path(self) -> VehiclePose | None:
        return self._reference_path

    # -- main step -----------------------------------------------------------
    def compute(self, now_s: float) -> PlannerOutput:
        nominal = self._current_nominal(now_s)
        obstacle, risk = self._most_dangerous_obstacle(now_s)
        dangerous = obstacle is not None and risk >= self.config.risk_enter_threshold
        self._maybe_capture_reference(nominal)

        rng = (
            estimate_range(
                obstacle,
                self._vfov_rad(),
                self.config.target_obstacle_height_m,
                self.config.max_range_m,
            )
            if obstacle is not None
            else self.config.max_range_m
        )
        ahead = (
            dangerous
            and obstacle is not None
            and abs(obstacle.bearing_rad) <= math.radians(self.config.ahead_bearing_deg)
        )
        # Commit once within engage range, OR when risk is high enough that the
        # obstacle is clearly close (a safety net if the monocular range estimate
        # is biased high, so we never fail to start the go-around in time).
        in_engage_range = dangerous and (
            rng <= self.config.engage_distance_m
            or risk >= self.config.risk_enter_threshold + 0.2
        )

        if self.state == PlannerState.NORMAL:
            if dangerous:
                self.state = PlannerState.APPROACH_OBSTACLE

        elif self.state == PlannerState.APPROACH_OBSTACLE:
            if not dangerous:
                self.state = PlannerState.NORMAL
            elif in_engage_range and obstacle is not None:
                self._commit(obstacle, rng, now_s)
                self.state = self._state_for_selected_side()

        elif self.state in {PlannerState.AVOIDING_LEFT, PlannerState.AVOIDING_RIGHT}:
            if self._passed_obstacle(ahead, now_s):
                self._start_recovery(now_s)

        elif self.state == PlannerState.RECOVERING:
            if in_engage_range and obstacle is not None:
                self._commit(obstacle, rng, now_s)
                self.state = self._state_for_selected_side()
                self._recovery_start_time_s = None

        return self._output_for_state(nominal, risk, rng)

    # -- state helpers -------------------------------------------------------
    def _output_for_state(
        self, nominal: VelocityCommand, risk: float, rng: float
    ) -> PlannerOutput:
        if self.state in {PlannerState.NORMAL, PlannerState.APPROACH_OBSTACLE}:
            return self._line_output(nominal, risk, rng)
        if self.state in {PlannerState.AVOIDING_LEFT, PlannerState.AVOIDING_RIGHT}:
            return self._go_around_output(nominal, risk, rng)
        return self._recovery_output(nominal, risk, rng)

    def _commit(self, obstacle: ObstacleObservation, rng: float, now_s: float) -> None:
        side = choose_avoidance_side(
            obstacle, self.config.central_zone_min_x, self.config.central_zone_max_x
        )
        self.selected_side = side
        sign = 1.0 if side == AvoidanceSide.LEFT else -1.0
        self._commit_offset_m = sign * abs(self.config.clearance_offset_m)
        self._commit_time_s = now_s
        if self._reference_path is not None and self._latest_pose is not None:
            along, _cross = self._path_progress(self._latest_pose)
            self._anchor_along_m = along + rng * math.cos(obstacle.bearing_rad)
        else:
            self._anchor_along_m = None

    def _passed_obstacle(self, ahead: bool, now_s: float) -> bool:
        if ahead:
            return False  # never turn back while the obstacle is still in front
        if (
            self._reference_path is not None
            and self._latest_pose is not None
            and self._anchor_along_m is not None
        ):
            along, _cross = self._path_progress(self._latest_pose)
            return along >= self._anchor_along_m + self.config.pass_margin_m
        # Pose-less fallback: hold the offset for a minimum time, then pass.
        if self._commit_time_s is None:
            return True
        return now_s - self._commit_time_s >= max(1.0, self.config.min_avoidance_hold_s)

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
        self._recovery_start_time_s = None
        self._anchor_along_m = None
        self._commit_offset_m = 0.0
        self._commit_time_s = None

    # -- reference path ------------------------------------------------------
    def _maybe_capture_reference(self, nominal: VelocityCommand) -> None:
        if self.state != PlannerState.NORMAL or self._latest_pose is None:
            return
        if nominal.surge <= self.config.forward_reference_min_surge:
            return
        operator_steering = (
            abs(nominal.yaw_rate) > STEER_EPS or abs(nominal.sway) > STEER_EPS
        )
        if self._reference_path is None or operator_steering:
            self._reference_path = self._latest_pose

    def _path_progress(self, pose: VehiclePose) -> tuple[float, float]:
        """(along-track, cross-track) of pose relative to the reference line."""
        assert self._reference_path is not None
        ref = self._reference_path
        dx = pose.x - ref.x
        dy = pose.y - ref.y
        forward = math.cos(ref.yaw_rad) * dx + math.sin(ref.yaw_rad) * dy
        cross = -math.sin(ref.yaw_rad) * dx + math.cos(ref.yaw_rad) * dy
        return forward, cross

    def _path_errors_or_zero(self) -> tuple[float, float]:
        if self._reference_path is None or self._latest_pose is None:
            return 0.0, 0.0
        _along, cross = self._path_progress(self._latest_pose)
        yaw_err = _wrap_to_pi(self._latest_pose.yaw_rad - self._reference_path.yaw_rad)
        return cross, yaw_err

    def _vfov_rad(self) -> float:
        return math.radians(self.config.camera_vertical_fov_deg)

    # -- command generation --------------------------------------------------
    def _line_output(
        self, nominal: VelocityCommand, risk: float, rng: float
    ) -> PlannerOutput:
        """NORMAL / APPROACH: hold the original line (offset 0) while cruising."""
        if self._reference_path is not None and self._latest_pose is not None:
            _along, cross = self._path_progress(self._latest_pose)
            yaw_err = _wrap_to_pi(
                self._latest_pose.yaw_rad - self._reference_path.yaw_rad
            )
            forward = _clamp(
                nominal.surge, -abs(self.config.max_surge), abs(self.config.max_surge)
            )
            command = self._steer(
                nominal, cross, yaw_err, forward, self.config.recovery_max_sway
            )
            return PlannerOutput(
                command, self.state, self.selected_side, risk, cross, yaw_err, rng
            )
        return PlannerOutput(
            nominal, self.state, self.selected_side, risk, 0.0, 0.0, rng
        )

    def _go_around_output(
        self, nominal: VelocityCommand, risk: float, rng: float
    ) -> PlannerOutput:
        """AVOIDING: strafe to the clearing offset, then run parallel past it."""
        if self._reference_path is None or self._latest_pose is None:
            return PlannerOutput(
                self._fallback_avoidance_command(nominal),
                self.state,
                self.selected_side,
                risk,
                0.0,
                0.0,
                rng,
            )
        _along, cross = self._path_progress(self._latest_pose)
        yaw_err = _wrap_to_pi(self._latest_pose.yaw_rad - self._reference_path.yaw_rad)
        cross_err = cross - self._commit_offset_m
        reaching = abs(cross_err) > self.config.offset_reached_tol_m
        if nominal.surge <= self.config.forward_reference_min_surge:
            forward = 0.0
        elif reaching:
            # Prioritise strafing out before advancing (don't creep into it).
            forward = min(self.config.min_surge_during_avoidance, abs(nominal.surge))
        else:
            forward = min(self.config.go_around_surge, abs(nominal.surge))
        command = self._steer(
            nominal, cross_err, yaw_err, forward, self.config.go_around_max_sway
        )
        return PlannerOutput(
            command, self.state, self.selected_side, risk, cross, yaw_err, rng
        )

    def _recovery_output(
        self, nominal: VelocityCommand, risk: float, rng: float
    ) -> PlannerOutput:
        if self._recovery_start_time_s is None:
            self._recovery_start_time_s = 0.0
        if self._reference_path is not None and self._latest_pose is not None:
            _along, cross = self._path_progress(self._latest_pose)
            yaw_err = _wrap_to_pi(
                self._latest_pose.yaw_rad - self._reference_path.yaw_rad
            )
            yaw_tol = math.radians(self.config.recovery_yaw_tolerance_deg)
            if (
                abs(cross) <= self.config.recovery_lateral_tolerance_m
                and abs(yaw_err) <= yaw_tol
            ):
                self._complete_recovery()
                return PlannerOutput(
                    nominal, self.state, self.selected_side, risk, cross, yaw_err, rng
                )
            forward = _clamp(
                nominal.surge, -abs(self.config.max_surge), abs(self.config.max_surge)
            )
            command = self._steer(
                nominal, cross, yaw_err, forward, self.config.recovery_max_sway
            )
            return PlannerOutput(
                command,
                PlannerState.RECOVERING,
                self.selected_side,
                risk,
                cross,
                yaw_err,
                rng,
            )
        # Pose-less fallback: no odometry -> just resume nominal.
        self._complete_recovery()
        return PlannerOutput(nominal, self.state, self.selected_side, risk, 0.0, 0.0, rng)

    def _steer(
        self,
        nominal: VelocityCommand,
        cross_error: float,
        yaw_err: float,
        forward_surge: float,
        max_sway: float,
    ) -> VelocityCommand:
        """Holonomic controller: drive cross_error and yaw_err to zero while
        progressing ``forward_surge`` along the reference path."""
        assert self._reference_path is not None and self._latest_pose is not None
        ref = self._reference_path
        pose = self._latest_pose
        max_surge = abs(_finite_or_zero(self.config.max_surge))
        lat_corr = _clamp(
            -self.config.recovery_lateral_gain * cross_error,
            -abs(max_sway),
            abs(max_sway),
        )
        fx = math.cos(ref.yaw_rad)
        fy = math.sin(ref.yaw_rad)
        lx = -math.sin(ref.yaw_rad)
        ly = math.cos(ref.yaw_rad)
        world_vx = forward_surge * fx + lat_corr * lx
        world_vy = forward_surge * fy + lat_corr * ly
        cy = math.cos(pose.yaw_rad)
        sy = math.sin(pose.yaw_rad)
        surge = _clamp(world_vx * cy + world_vy * sy, -max_surge, max_surge)
        sway = _clamp(-world_vx * sy + world_vy * cy, -max_surge, max_surge)
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

    def _fallback_avoidance_command(self, nominal: VelocityCommand) -> VelocityCommand:
        side_sign = 1.0 if self.selected_side == AvoidanceSide.LEFT else -1.0
        max_surge = abs(_finite_or_zero(self.config.max_surge))
        surge = _clamp(nominal.surge, -max_surge, max_surge)
        if surge > self.config.min_surge_during_avoidance:
            surge = self.config.min_surge_during_avoidance
        return VelocityCommand(
            surge=surge,
            sway=side_sign * abs(_finite_or_zero(self.config.avoidance_sway)),
            heave=nominal.heave,
            roll_rate=nominal.roll_rate,
            pitch_rate=nominal.pitch_rate,
            yaw_rate=side_sign * abs(_finite_or_zero(self.config.avoidance_yaw_rate)),
        )

    def _current_nominal(self, now_s: float) -> VelocityCommand:
        if self._last_nominal_time_s is None:
            return VelocityCommand()
        if now_s - self._last_nominal_time_s > self.config.command_timeout_s:
            if self.config.nominal_timeout_behavior.strip().lower() in {"hold", "hold_last"}:
                return _sanitize_command(self._nominal_command, self.config.max_surge)
            return VelocityCommand()
        return _sanitize_command(self._nominal_command, self.config.max_surge)

    def _most_dangerous_obstacle(
        self, now_s: float
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


def estimate_range(
    obstacle: ObstacleObservation,
    vfov_rad: float,
    target_height_m: float,
    max_range_m: float,
) -> float:
    """Monocular range from apparent (normalized) bbox height.

    An object of physical height ``H`` at range ``R`` subtends a vertical angle
    ``theta`` filling ``height`` of the image, so ``theta = height * vfov`` and
    ``R = (H/2) / tan(theta/2)``.  Realistic (no ground truth): needs only the
    detection, the camera FOV and an assumed target size.
    """
    h = _clamp(obstacle.height, 0.0, 1.0)
    if h <= 1e-4 or vfov_rad <= 0.0:
        return max_range_m
    half_theta = 0.5 * h * vfov_rad
    t = math.tan(half_theta)
    if t <= 1e-6:
        return max_range_m
    return _clamp((target_height_m * 0.5) / t, 0.0, max_range_m)


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
