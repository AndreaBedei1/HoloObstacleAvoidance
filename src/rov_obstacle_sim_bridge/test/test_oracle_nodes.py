"""Tests for the ROS 2 oracle bridge nodes."""

import csv
import math
import os
import tempfile
import unittest

from geometry_msgs.msg import Quaternion

from rov_obstacle_sim_bridge.cmd_vel_safe_logger_node import CmdVelSafeLoggerNode
from rov_obstacle_sim_bridge.holoocean_obstacle_oracle_node import (
    HolooceanObstacleOracleNode,
    yaw_from_quaternion,
)
from rov_obstacle_sim_bridge.simulated_rover_pose_publisher_node import (
    SimulatedRoverPosePublisherNode,
    quaternion_from_yaw,
)


def _get_parameter():
    """Lazy import for rclpy.parameter.Parameter."""
    from rclpy.parameter import Parameter
    return Parameter


# ---------------------------------------------------------------------------
# Warm-up rclpy so the (flaky) logging-DLL load happens once at import time
# rather than inside the first test's setUp.  On Windows Humble the first
# rclpy.init() may fail when loading rcl_logging_spdlog.dll; after that
# failure the global ref-count prevents re-trying, so subsequent inits succeed.
# ---------------------------------------------------------------------------
try:
    import rclpy as _rclpy
    _warm_ctx = _rclpy.context.Context()
    _rclpy.init(context=_warm_ctx)
    _rclpy.shutdown(context=_warm_ctx)
except Exception:  # noqa: BLE001 – any failure is acceptable here
    pass


class TestQuaternionHelpers(unittest.TestCase):
    """Test the quaternion conversion helpers in both nodes."""

    def test_quaternion_from_yaw_zero(self):
        q = quaternion_from_yaw(0.0)
        self.assertAlmostEqual(q.w, 1.0, places=6)
        self.assertAlmostEqual(q.x, 0.0, places=6)
        self.assertAlmostEqual(q.y, 0.0, places=6)
        self.assertAlmostEqual(q.z, 0.0, places=6)

    def test_quaternion_from_yaw_90_deg(self):
        q = quaternion_from_yaw(math.pi / 2)
        self.assertAlmostEqual(q.w, math.cos(math.pi / 4), places=6)
        self.assertAlmostEqual(q.z, math.sin(math.pi / 4), places=6)

    def test_yaw_from_quaternion_zero(self):
        q = Quaternion()
        q.w = 1.0
        yaw = yaw_from_quaternion(q)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    def test_yaw_from_quaternion_90_deg(self):
        """Reconstruct a 90-degree yaw quaternion and extract back."""
        q = quaternion_from_yaw(math.pi / 2)
        yaw = yaw_from_quaternion(q)
        self.assertAlmostEqual(yaw, math.pi / 2, places=6)

    def test_roundtrip_consistency(self):
        """quaternion_from_yaw -> yaw_from_quaternion should be identity."""
        for angle in [0.0, math.pi / 6, math.pi / 4, math.pi / 2, math.pi, -math.pi / 3]:
            q = quaternion_from_yaw(angle)
            recovered = yaw_from_quaternion(q)
            self.assertAlmostEqual(recovered, angle, places=6, msg=f"Failed at {angle}")


class TestNodeParameters(unittest.TestCase):
    """Verify default parameters are declared correctly."""

    def setUp(self):
        import rclpy
        self._ctx = rclpy.context.Context()
        rclpy.init(context=self._ctx)

    def tearDown(self):
        import rclpy
        rclpy.shutdown(context=self._ctx)

    def test_simulated_pose_publisher_defaults(self):
        node = SimulatedRoverPosePublisherNode(context=self._ctx)
        self.assertEqual(
            str(node.get_parameter("output_topic").value), "/sim/rov_pose"
        )
        self.assertAlmostEqual(node.get_parameter("publish_rate_hz").value, 20.0)
        self.assertEqual(str(node.get_parameter("frame_id").value), "world")
        self.assertEqual(str(node.get_parameter("motion_mode").value), "forward")
        node.destroy_node()

    def test_oracle_node_defaults(self):
        Parameter = _get_parameter()
        fh = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        try:
            fh.write("obstacles:\n")
            fh.write("  - name: test\n    class_name: rock\n")
            fh.write("    position: [3.0, 0.0, 0.0]\n    radius_m: 0.4\n")
            fh.flush()
            fh.close()
            node = HolooceanObstacleOracleNode(
                context=self._ctx,
                parameter_overrides=[
                    Parameter("obstacle_config_file", Parameter.Type.STRING, fh.name)
                ],
            )
            self.assertEqual(
                str(node.get_parameter("rover_pose_topic").value), "/sim/rov_pose"
            )
            self.assertEqual(
                str(node.get_parameter("output_topic").value), "/perception/obstacles"
            )
            self.assertAlmostEqual(node.get_parameter("confidence").value, 1.0)
            node.destroy_node()
        finally:
            os.unlink(fh.name)

    def test_cmd_vel_logger_defaults(self):
        node = CmdVelSafeLoggerNode(context=self._ctx)
        self.assertEqual(
            str(node.get_parameter("input_topic").value), "/planner/cmd_vel_safe"
        )
        self.assertEqual(str(node.get_parameter("log_file").value), "")
        self.assertAlmostEqual(node.get_parameter("flush_interval_s").value, 1.0)
        node.destroy_node()


class TestCmdVelLoggerCsv(unittest.TestCase):
    """Verify the CSV logger writes correct rows."""

    def setUp(self):
        import rclpy
        self._ctx = rclpy.context.Context()
        rclpy.init(context=self._ctx)

    def tearDown(self):
        import rclpy
        rclpy.shutdown(context=self._ctx)

    def test_csv_header_and_content(self):
        Parameter = _get_parameter()
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".csv", delete=False
        ) as tmp:
            tmp_path = tmp.name

        try:
            node = CmdVelSafeLoggerNode(
                context=self._ctx,
                parameter_overrides=[
                    Parameter("log_file", Parameter.Type.STRING, tmp_path)
                ],
            )

            from geometry_msgs.msg import Twist

            twist = Twist()
            twist.linear.x = 1.0
            twist.linear.y = -0.5
            twist.angular.z = 0.3

            node._on_cmd_vel(twist)
            node._flush()
            node.on_shutdown()
            node.destroy_node()

            with open(tmp_path, newline="") as f:
                reader = csv.DictReader(f)
                rows = list(reader)

            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(float(rows[0]["linear_x"]), 1.0)
            self.assertAlmostEqual(float(rows[0]["linear_y"]), -0.5)
            self.assertAlmostEqual(float(rows[0]["angular_z"]), 0.3)
        finally:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)


if __name__ == "__main__":
    unittest.main()
