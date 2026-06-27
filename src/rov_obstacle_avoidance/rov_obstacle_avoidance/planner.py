"""Deterministic local obstacle avoidance planner logic."""

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
    avoidance_yaw_rate: float = 0.35
    command_timeout_s: float = 1.0
    nominal_timeout_behavior: str = "stop"


@dataclass(frozen=True)
class PlannerOutput:
    command: VelocityCommand
    state: PlannerState
    selected_side: AvoidanceSide
    risk: float


CENTRALITY_RISK_WEIGHT = 0.6
AREA_RISK_WEIGHT = 0.4
AREA_RISK_GAIN = 4.0


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

    def update_nominal_command(self, command: VelocityCommand, now_s: float) -> None:
        self._nominal_command = command
        self._last_nominal_time_s = now_s

    def update_obstacles(self, obstacles: list[ObstacleObservation], now_s: float) -> None:
        self._obstacles = obstacles
        self._last_obstacle_time_s = now_s

    def compute(self, now_s: float) -> PlannerOutput:
        nominal = self._current_nominal(now_s)
        obstacle, risk = self._most_dangerous_obstacle(now_s)
        dangerous = obstacle is not None and risk >= self.config.risk_enter_threshold
        clear = obstacle is None or risk < self.config.risk_exit_threshold

        if self.state == PlannerState.NORMAL:
            if dangerous:
                self._select_side(obstacle, now_s)
                self.state = PlannerState.APPROACH_OBSTACLE
            else:
                return PlannerOutput(nominal, self.state, self.selected_side, risk)

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
            else:
                recovered = self._recovery_blend(nominal, now_s)
                if recovered is not None:
                    return PlannerOutput(recovered, self.state, self.selected_side, risk)
                self.state = PlannerState.NORMAL
                self.selected_side = AvoidanceSide.NONE
                self._side_selected_time_s = None
                self._recovery_start_time_s = None
                return PlannerOutput(nominal, self.state, self.selected_side, risk)

        if self.state == PlannerState.RECOVERING:
            recovered = self._recovery_blend(nominal, now_s)
            if recovered is not None:
                return PlannerOutput(recovered, self.state, self.selected_side, risk)
            self.state = PlannerState.NORMAL
            self.selected_side = AvoidanceSide.NONE
            return PlannerOutput(nominal, self.state, self.selected_side, risk)

        command = self._avoidance_command(nominal)
        return PlannerOutput(command, self.state, self.selected_side, risk)

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
        return _clamp(obstacle.risk, 0.0, 1.0)

    confidence = _clamp(obstacle.confidence, 0.0, 1.0)
    centrality = _clamp(1.0 - abs(_clamp(obstacle.center_x, 0.0, 1.0) - 0.5) * 2.0, 0.0, 1.0)
    apparent_area = obstacle.apparent_area if obstacle.apparent_area > 0.0 else obstacle.width * obstacle.height
    area_score = _clamp(apparent_area * AREA_RISK_GAIN, 0.0, 1.0)
    return _clamp(confidence * (CENTRALITY_RISK_WEIGHT * centrality + AREA_RISK_WEIGHT * area_score), 0.0, 1.0)


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
