import math
from pathlib import Path
import unittest

from rov_obstacle_avoidance.planner import (
    AvoidanceSide,
    LocalAvoidancePlanner,
    ObstacleObservation,
    PlannerConfig,
    PlannerState,
    VelocityCommand,
    compute_obstacle_risk,
)


class PlannerRiskTest(unittest.TestCase):
    def test_uses_detector_risk_when_positive(self):
        obstacle = ObstacleObservation(confidence=0.1, center_x=0.0, width=0.1, height=0.1, risk=0.73)

        self.assertAlmostEqual(compute_obstacle_risk(obstacle), 0.73)

    def test_fallback_risk_uses_confidence_centrality_and_area(self):
        central = ObstacleObservation(confidence=0.9, center_x=0.5, width=0.4, height=0.4, risk=0.0)
        edge = ObstacleObservation(confidence=0.9, center_x=0.0, width=0.4, height=0.4, risk=0.0)

        self.assertGreater(compute_obstacle_risk(central), compute_obstacle_risk(edge))
        self.assertGreater(compute_obstacle_risk(central), 0.5)

    def test_anchor_class_has_higher_generic_risk_than_sphere(self):
        anchor = ObstacleObservation(
            class_name="anchor",
            confidence=0.9,
            center_x=0.5,
            width=0.25,
            height=0.35,
            apparent_area=0.25 * 0.35,
            risk=0.0,
        )
        sphere = ObstacleObservation(
            class_name="sphere",
            confidence=0.9,
            center_x=0.5,
            width=0.25,
            height=0.35,
            apparent_area=0.25 * 0.35,
            risk=0.0,
        )

        self.assertGreater(compute_obstacle_risk(anchor), compute_obstacle_risk(sphere))

    def test_detector_risk_is_class_weighted_and_clamped(self):
        anchor = ObstacleObservation(class_name="anchor", confidence=1.0, risk=0.5)
        very_high_anchor = ObstacleObservation(class_name="anchor", confidence=1.0, risk=0.95)

        self.assertGreater(compute_obstacle_risk(anchor), 0.5)
        self.assertLessEqual(compute_obstacle_risk(very_high_anchor), 1.0)


class PlannerBehaviorTest(unittest.TestCase):
    def setUp(self):
        self.config = PlannerConfig(
            risk_enter_threshold=0.55,
            risk_exit_threshold=0.30,
            min_avoidance_hold_s=1.0,
            recovery_time_s=2.0,
            command_timeout_s=1.0,
        )
        self.nominal = VelocityCommand(surge=0.3, sway=0.0, heave=0.1, yaw_rate=0.0)

    def test_no_obstacle_passes_through_nominal_command(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)
        planner.update_obstacles([], now_s=0.0)

        output = planner.compute(now_s=0.1)

        self.assertEqual(output.state, PlannerState.NORMAL)
        self.assertEqual(output.command, self.nominal)

    def test_central_obstacle_triggers_avoidance(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)
        planner.update_obstacles([_obstacle(center_x=0.5)], now_s=0.0)

        first = planner.compute(now_s=0.1)
        second = planner.compute(now_s=0.2)

        self.assertEqual(first.state, PlannerState.APPROACH_OBSTACLE)
        self.assertIn(second.state, {PlannerState.AVOIDING_LEFT, PlannerState.AVOIDING_RIGHT})
        self.assertLess(second.command.surge, self.nominal.surge)

    def test_left_obstacle_causes_right_avoidance(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)
        planner.update_obstacles([_obstacle(center_x=0.2)], now_s=0.0)

        output = planner.compute(now_s=0.1)

        self.assertEqual(output.selected_side, AvoidanceSide.RIGHT)
        self.assertLess(output.command.sway, 0.0)
        self.assertLess(output.command.yaw_rate, 0.0)

    def test_right_obstacle_causes_left_avoidance(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)
        planner.update_obstacles([_obstacle(center_x=0.8)], now_s=0.0)

        output = planner.compute(now_s=0.1)

        self.assertEqual(output.selected_side, AvoidanceSide.LEFT)
        self.assertGreater(output.command.sway, 0.0)
        self.assertGreater(output.command.yaw_rate, 0.0)

    def test_state_transitions_through_recovery_to_normal(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)
        planner.update_obstacles([_obstacle(center_x=0.5)], now_s=0.0)

        planner.compute(now_s=0.1)
        avoiding = planner.compute(now_s=0.2)
        self.assertIn(avoiding.state, {PlannerState.AVOIDING_LEFT, PlannerState.AVOIDING_RIGHT})

        planner.update_obstacles([], now_s=1.3)
        recovering = planner.compute(now_s=1.3)
        self.assertEqual(recovering.state, PlannerState.RECOVERING)

        planner.update_nominal_command(self.nominal, now_s=3.4)
        normal = planner.compute(now_s=3.4)
        self.assertEqual(normal.state, PlannerState.NORMAL)
        self.assertEqual(normal.selected_side, AvoidanceSide.NONE)
        self.assertEqual(normal.command, self.nominal)

    def test_stale_obstacle_detections_cause_recovery_and_pass_through(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)
        planner.update_obstacles([_obstacle(center_x=0.5)], now_s=0.0)

        planner.compute(now_s=0.1)
        planner.compute(now_s=0.2)
        planner.compute(now_s=1.2)
        planner.update_nominal_command(self.nominal, now_s=3.4)
        output = planner.compute(now_s=3.4)

        self.assertEqual(output.state, PlannerState.NORMAL)
        self.assertEqual(output.command, self.nominal)

    def test_stale_nominal_command_triggers_safe_stop(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)

        output = planner.compute(now_s=2.0)

        self.assertEqual(output.command, VelocityCommand())

    def test_selected_avoidance_side_does_not_flip_every_frame(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(self.nominal, now_s=0.0)
        planner.update_obstacles([_obstacle(center_x=0.2)], now_s=0.0)
        first = planner.compute(now_s=0.1)

        planner.update_obstacles([_obstacle(center_x=0.8)], now_s=0.2)
        second = planner.compute(now_s=0.2)

        self.assertEqual(first.selected_side, AvoidanceSide.RIGHT)
        self.assertEqual(second.selected_side, AvoidanceSide.RIGHT)

    def test_generated_twist_commands_are_finite_and_surge_clamped(self):
        planner = LocalAvoidancePlanner(self.config)
        planner.update_nominal_command(
            VelocityCommand(
                surge=2.0,
                sway=float("nan"),
                heave=float("inf"),
                roll_rate=0.0,
                pitch_rate=0.0,
                yaw_rate=float("-inf"),
            ),
            now_s=0.0,
        )
        planner.update_obstacles([], now_s=0.0)

        output = planner.compute(now_s=0.1)

        values = (
            output.command.surge,
            output.command.sway,
            output.command.heave,
            output.command.roll_rate,
            output.command.pitch_rate,
            output.command.yaw_rate,
        )
        self.assertTrue(all(math.isfinite(value) for value in values))
        self.assertLessEqual(abs(output.command.surge), self.config.max_surge)


class PlannerConfigTest(unittest.TestCase):
    def test_planner_config_contains_topic_parameters(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "config" / "avoidance_planner.yaml").read_text(encoding="utf-8")

        self.assertIn('obstacle_topic: "/perception/obstacles"', text)
        self.assertIn('nominal_cmd_topic: "/cmd_vel_nominal"', text)
        self.assertIn('safe_cmd_topic: "/planner/cmd_vel_safe"', text)
        self.assertIn('debug_topic: "/avoidance/debug"', text)
        self.assertIn('debug_frame_id: "front_camera"', text)
        self.assertIn('nominal_timeout_behavior: "stop"', text)

    def test_nominal_publisher_config_contains_output_topic(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "config" / "nominal_cmd_publisher.yaml").read_text(
            encoding="utf-8"
        )

        self.assertIn('output_topic: "/cmd_vel_nominal"', text)


def _obstacle(center_x: float) -> ObstacleObservation:
    return ObstacleObservation(
        confidence=0.9,
        center_x=center_x,
        center_y=0.5,
        width=0.25,
        height=0.35,
        apparent_area=0.25 * 0.35,
        risk=0.8,
        is_tracking_valid=True,
    )


if __name__ == "__main__":
    unittest.main()
