"""Closed-loop tests proving the planner returns to the ORIGINAL path.

These tests drive the deterministic ``LocalAvoidancePlanner`` through a
holonomic kinematic simulation that mirrors the HoloOcean sim server motion
model exactly (see ``holoocean_sim_server.body_to_world`` /
``_integrate_and_teleport``)::

    yaw   += yaw_rate * dt
    x     += surge*cos(yaw) - sway*sin(yaw)   (times dt)
    y     += surge*sin(yaw) + sway*cos(yaw)   (times dt)

The obstacle "detector" here is a deliberately simple, clearly-labelled
stand-in for YOLO: its only job is to trigger the state machine so the
recovery / return-to-line behaviour (the actual fix) can be validated
deterministically.  Realistic detection is covered by the perception package
tests and the real closed-loop run.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import unittest

from rov_obstacle_avoidance.planner import (
    AvoidanceSide,
    LocalAvoidancePlanner,
    ObstacleObservation,
    PlannerConfig,
    PlannerState,
    VehiclePose,
    VelocityCommand,
)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def _step(pose: Pose, cmd: VelocityCommand, dt: float) -> Pose:
    """One kinematic teleport step, identical to the sim server."""
    yaw = pose.yaw + cmd.yaw_rate * dt
    dx = cmd.surge * dt * math.cos(yaw) - cmd.sway * dt * math.sin(yaw)
    dy = cmd.surge * dt * math.sin(yaw) + cmd.sway * dt * math.cos(yaw)
    return Pose(pose.x + dx, pose.y + dy, yaw)


class _SyntheticAnchor:
    """A centered obstacle the vehicle must move laterally around.

    Becomes "dangerous" once the vehicle reaches ``trigger_x`` and stays so
    until the vehicle has strafed ``clearance_m`` off the line, after which it
    latches cleared (the vehicle has gone around it) and never re-triggers.
    """

    def __init__(self, trigger_x: float, clearance_m: float) -> None:
        self.trigger_x = trigger_x
        self.clearance_m = clearance_m
        self.armed = False
        self.cleared = False

    def detect(self, pose: Pose) -> list[ObstacleObservation]:
        if self.cleared:
            return []
        if not self.armed:
            if pose.x < self.trigger_x:
                return []  # not yet encountered -> straight cruise
            self.armed = True
        # Once encountered it stays a threat until the vehicle has strafed
        # around it (latched cleared), regardless of forward creep/yaw.
        if abs(pose.y) >= self.clearance_m:
            self.cleared = True
            return []
        return [
            ObstacleObservation(
                class_name="anchor",
                confidence=0.9,
                center_x=0.5,
                center_y=0.5,
                width=0.30,
                height=0.40,
                apparent_area=0.12,
                risk=0.8,
                is_tracking_valid=True,
            )
        ]


def _wrap_deg(angle_deg: float) -> float:
    return math.degrees(math.atan2(math.sin(math.radians(angle_deg)),
                                   math.cos(math.radians(angle_deg))))


def run_closed_loop(
    config: PlannerConfig,
    *,
    anchor: _SyntheticAnchor,
    nominal: VelocityCommand,
    duration_s: float = 60.0,
    dt: float = 0.05,
) -> dict:
    planner = LocalAvoidancePlanner(config)
    pose = Pose(0.0, 0.0, 0.0)
    states: list[str] = []
    safe_diff_from_nominal = 0
    max_lateral = 0.0
    ticks = int(duration_s / dt)
    for k in range(ticks):
        now = k * dt
        planner.update_nominal_command(nominal, now)
        planner.update_obstacles(anchor.detect(pose), now)
        planner.update_pose(pose.x, pose.y, pose.yaw, now)
        out = planner.compute(now)

        if not states or states[-1] != out.state.value:
            states.append(out.state.value)
        if (
            abs(out.command.surge - nominal.surge) > 0.02
            or abs(out.command.sway - nominal.sway) > 0.02
            or abs(out.command.yaw_rate - nominal.yaw_rate) > 0.02
        ):
            safe_diff_from_nominal += 1
        max_lateral = max(max_lateral, abs(pose.y))
        pose = _step(pose, out.command, dt)

    initial_yaw = 0.0
    final_yaw = pose.yaw
    # Lateral error vs the original line (through origin, heading initial_yaw).
    lateral = abs(-math.sin(initial_yaw) * pose.x + math.cos(initial_yaw) * pose.y)
    yaw_err_deg = _wrap_deg(math.degrees(final_yaw - initial_yaw))
    returned = lateral < 0.5 and abs(yaw_err_deg) < 10.0
    return {
        "states": states,
        "safe_diff_from_nominal_msgs": safe_diff_from_nominal,
        "final_pose": pose,
        "initial_yaw_rad": initial_yaw,
        "final_yaw_rad": final_yaw,
        "final_lateral_error_m": lateral,
        "final_yaw_error_deg": yaw_err_deg,
        "max_lateral_deviation_m": max_lateral,
        "returned_to_original_line": returned,
    }


PROD_CONFIG = PlannerConfig(
    risk_enter_threshold=0.55,
    risk_exit_threshold=0.30,
    min_avoidance_hold_s=1.0,
    max_surge=0.5,
    min_surge_during_avoidance=0.08,
    avoidance_sway=0.20,
    avoidance_yaw_rate=0.1,
    command_timeout_s=1.0,
    recovery_lateral_gain=0.8,
    recovery_yaw_gain=1.0,
    recovery_max_sway=0.25,
    recovery_max_yaw_rate=0.30,
    recovery_lateral_tolerance_m=0.20,
    recovery_yaw_tolerance_deg=5.0,
    recovery_max_time_s=20.0,
)


class ReferenceCaptureTest(unittest.TestCase):
    def test_reference_line_captured_while_cruising_forward(self):
        planner = LocalAvoidancePlanner(PROD_CONFIG)
        planner.update_nominal_command(VelocityCommand(surge=0.4), now_s=0.0)
        planner.update_pose(1.0, 2.0, 0.5, now_s=0.0)
        planner.compute(now_s=0.0)
        ref = planner.reference_path
        self.assertIsNotNone(ref)
        assert ref is not None
        self.assertAlmostEqual(ref.x, 1.0)
        self.assertAlmostEqual(ref.y, 2.0)
        self.assertAlmostEqual(ref.yaw_rad, 0.5)

    def test_reference_not_captured_when_idle(self):
        planner = LocalAvoidancePlanner(PROD_CONFIG)
        planner.update_nominal_command(VelocityCommand(surge=0.0), now_s=0.0)
        planner.update_pose(1.0, 2.0, 0.0, now_s=0.0)
        planner.compute(now_s=0.0)
        self.assertIsNone(planner.reference_path)


class RecoveryControllerDirectionTest(unittest.TestCase):
    """White-box sign checks for the pose-aware recovery controller."""

    def _planner_in_recovery(self, cross_pose: VehiclePose) -> LocalAvoidancePlanner:
        planner = LocalAvoidancePlanner(PROD_CONFIG)
        planner._reference_path = VehiclePose(0.0, 0.0, 0.0)
        planner._latest_pose = cross_pose
        planner.state = PlannerState.RECOVERING
        planner._recovery_start_time_s = 0.0
        planner.update_nominal_command(VelocityCommand(surge=0.4), now_s=0.1)
        return planner

    def test_left_offset_commands_rightward_sway(self):
        # Vehicle left of the line (y>0) should be pushed right (sway<0).
        planner = self._planner_in_recovery(VehiclePose(1.0, 1.0, 0.0))
        out = planner.compute(now_s=0.1)
        self.assertEqual(out.state, PlannerState.RECOVERING)
        self.assertGreater(out.cross_track_error_m, 0.0)
        self.assertLess(out.command.sway, 0.0)
        self.assertGreater(out.command.surge, 0.0)

    def test_right_offset_commands_leftward_sway(self):
        planner = self._planner_in_recovery(VehiclePose(1.0, -1.0, 0.0))
        out = planner.compute(now_s=0.1)
        self.assertLess(out.cross_track_error_m, 0.0)
        self.assertGreater(out.command.sway, 0.0)

    def test_positive_yaw_error_commands_negative_yaw_rate(self):
        planner = self._planner_in_recovery(VehiclePose(0.0, 0.0, 0.6))
        out = planner.compute(now_s=0.1)
        self.assertGreater(out.yaw_error_rad, 0.0)
        self.assertLess(out.command.yaw_rate, 0.0)


class ClosedLoopReturnTest(unittest.TestCase):
    def test_lateral_avoidance_returns_to_original_line(self):
        result = run_closed_loop(
            PROD_CONFIG,
            anchor=_SyntheticAnchor(trigger_x=2.5, clearance_m=1.6),
            nominal=VelocityCommand(surge=0.4),
        )
        states = result["states"]
        self.assertIn("APPROACH_OBSTACLE", states)
        self.assertTrue(any(s.startswith("AVOIDING_") for s in states))
        self.assertIn("RECOVERING", states)
        # ends back in NORMAL, and RECOVERING happens before that final NORMAL
        self.assertEqual(states[-1], "NORMAL")
        self.assertLess(states.index("RECOVERING"), len(states) - 1)
        self.assertGreater(result["safe_diff_from_nominal_msgs"], 0)
        # The maneuver actually left the line before returning.
        self.assertGreater(result["max_lateral_deviation_m"], 0.5)
        # ...and came back.
        self.assertLess(result["final_lateral_error_m"], 0.5)
        self.assertLess(abs(result["final_yaw_error_deg"]), 10.0)
        self.assertTrue(result["returned_to_original_line"])

    def test_recovery_corrects_induced_yaw_drift(self):
        """Reproduce the original bug (heading runaway) and prove it is fixed.

        With a non-zero avoidance yaw rate the vehicle accrues a large heading
        drift during avoidance (the old behaviour kept that drift forever).
        The pose-aware recovery must drive the heading back to the original.
        """
        drifting = PlannerConfig(
            risk_enter_threshold=0.55,
            risk_exit_threshold=0.30,
            min_avoidance_hold_s=1.0,
            max_surge=0.5,
            min_surge_during_avoidance=0.10,
            avoidance_sway=0.20,
            avoidance_yaw_rate=0.20,  # deliberately induce heading drift
            command_timeout_s=1.0,
            recovery_lateral_gain=0.8,
            recovery_yaw_gain=1.0,
            recovery_max_sway=0.30,
            recovery_max_yaw_rate=0.40,
            recovery_lateral_tolerance_m=0.20,
            recovery_yaw_tolerance_deg=5.0,
            recovery_max_time_s=25.0,
        )
        result = run_closed_loop(
            drifting,
            anchor=_SyntheticAnchor(trigger_x=2.5, clearance_m=1.2),
            nominal=VelocityCommand(surge=0.4),
            duration_s=60.0,
        )
        # Heading drifted meaningfully during the maneuver...
        self.assertGreater(result["max_lateral_deviation_m"], 0.5)
        # ...but recovery brought heading AND position back to the line.
        self.assertLess(abs(result["final_yaw_error_deg"]), 10.0)
        self.assertLess(result["final_lateral_error_m"], 0.5)
        self.assertTrue(result["returned_to_original_line"])
        self.assertEqual(result["states"][-1], "NORMAL")


if __name__ == "__main__":
    unittest.main()
