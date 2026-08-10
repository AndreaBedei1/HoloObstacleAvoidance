"""Unit tests for the transferable no-DVL commanded-motion odometry.

Pure Python (no rclpy for the model tests). Also asserts the forbidden-input
contract at the source level.
"""

from __future__ import annotations

import math
import unittest
from pathlib import Path

from rov_obstacle_sim_bridge.commanded_odometry import (
    CommandedOdometry,
    CommandedOdometryConfig,
)

DT = 1.0 / 20.0


def run(odo: CommandedOdometry, seconds: float, t0: float = 0.0) -> float:
    t = t0
    n = int(seconds / DT)
    for _ in range(n):
        t += DT
        odo.update(dt=DT, now_s=t)
    return t


class TestResponseModel(unittest.TestCase):
    def test_velocity_converges_to_command(self):
        odo = CommandedOdometry()
        odo.set_command(0.4, 0.0, 0.0, now_s=0.0)
        t = 0.0
        while t < 6.0:
            t += DT
            odo.set_command(0.4, 0.0, 0.0, now_s=t)  # keep fresh
            odo.update(dt=DT, now_s=t)
        self.assertAlmostEqual(odo.v_surge, 0.4, delta=0.01)

    def test_first_order_time_constant(self):
        cfg = CommandedOdometryConfig(tau_surge_s=1.2)
        odo = CommandedOdometry(config=cfg)
        t = 0.0
        while t < 1.2:  # one time constant
            t += DT
            odo.set_command(1.0, 0.0, 0.0, now_s=t)
            odo.update(dt=DT, now_s=t)
        # ~63% after one tau (discrete integration tolerance).
        self.assertAlmostEqual(odo.v_surge, 0.63, delta=0.05)

    def test_straight_line_integration_with_yaw(self):
        odo = CommandedOdometry()
        odo.set_measured_yaw(math.pi / 2)  # facing +y (left)
        t = 0.0
        while t < 10.0:
            t += DT
            odo.set_command(0.4, 0.0, 0.0, now_s=t)
            odo.update(dt=DT, now_s=t)
        # All displacement in +y, none in x.
        self.assertAlmostEqual(odo.x, 0.0, delta=1e-9)
        self.assertGreater(odo.y, 3.0)

    def test_depth_is_direct_measurement(self):
        odo = CommandedOdometry()
        odo.set_measured_depth(-7.5)
        run(odo, 1.0)
        self.assertEqual(odo.pose()["z"], -7.5)

    def test_stale_command_decays_to_zero(self):
        odo = CommandedOdometry()
        odo.set_command(0.5, 0.0, 0.0, now_s=0.0)
        t = 0.0
        while t < 3.0:  # keep fresh for 3 s
            t += DT
            odo.set_command(0.5, 0.0, 0.0, now_s=t)
            odo.update(dt=DT, now_s=t)
        self.assertGreater(odo.v_surge, 0.4)
        # Now stop sending commands: after cmd_timeout_s the target is zero.
        end = run(odo, 6.0, t0=t)
        self.assertLess(abs(odo.v_surge), 0.02,
                        "estimated velocity must decay after command loss")

    def test_nonfinite_inputs_rejected(self):
        odo = CommandedOdometry()
        odo.set_command(float("nan"), float("inf"), 0.0, now_s=0.0)
        odo.set_measured_yaw(float("nan"))
        odo.set_measured_depth(float("inf"))
        run(odo, 1.0)
        self.assertEqual(odo.pose()["z"], 0.0)
        self.assertTrue(math.isfinite(odo.x))
        self.assertAlmostEqual(odo.v_surge, 0.0, delta=1e-6)

    def test_speed_clamp(self):
        odo = CommandedOdometry()
        t = 0.0
        while t < 20.0:
            t += DT
            odo.set_command(99.0, 0.0, 0.0, now_s=t)
            odo.update(dt=DT, now_s=t)
        self.assertLessEqual(odo.v_surge, odo.config.max_speed_mps + 1e-6)

    def test_zero_or_negative_dt_ignored(self):
        odo = CommandedOdometry()
        odo.set_command(0.4, 0.0, 0.0, now_s=0.0)
        odo.update(dt=0.0, now_s=0.1)
        odo.update(dt=-1.0, now_s=0.2)
        self.assertEqual(odo.x, 0.0)


class TestForbiddenInputContract(unittest.TestCase):
    def test_guard_rejects_forbidden_topics(self):
        from rov_obstacle_sim_bridge.commanded_odometry import (
            assert_topics_allowed,
        )
        for bad in ("/rov/pose_ground_truth", "/ground_truth/external/pose",
                    "/perception/obstacles_oracle", "/rov/velocity"):
            with self.assertRaises(RuntimeError, msg=bad):
                assert_topics_allowed(["/rov/depth", bad])

    def test_guard_allows_transferable_topics(self):
        from rov_obstacle_sim_bridge.commanded_odometry import (
            assert_topics_allowed,
        )
        assert_topics_allowed([
            "/planner/cmd_vel_safe", "/rov/attitude_measured", "/rov/depth",
        ])

    def test_node_calls_guard(self):
        src = (Path(__file__).resolve().parents[1] / "rov_obstacle_sim_bridge"
               / "commanded_odometry_node.py").read_text()
        self.assertIn("assert_topics_allowed(topics)", src)

    def test_model_module_has_no_ros_or_socket_access(self):
        src = (Path(__file__).resolve().parents[1] / "rov_obstacle_sim_bridge"
               / "commanded_odometry.py").read_text()
        self.assertNotIn("import rclpy", src)
        self.assertNotIn("import socket", src)


if __name__ == "__main__":
    unittest.main()
