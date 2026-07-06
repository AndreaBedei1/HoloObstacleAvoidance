"""Tests for the ROS 2 HoloOcean bridge node.

These verify the bridge works WITHOUT HoloOcean installed (it never imports
holoocean) and that the simulation-only oracle projection respects the
empirically-calibrated HoloOcean frame: facing +X, a world +Y obstacle appears
on the LEFT of the image (center_x < 0.5).
"""

import unittest

from rov_obstacle_msgs.msg import Obstacle2DArray

from rov_obstacle_sim_bridge.holoocean_bridge_node import (
    HolooceanBridgeNode,
    quaternion_from_yaw,
)
from rov_obstacle_sim_bridge.sim_bridge_protocol import MSG_STATE


# Warm up rclpy once (Windows logging-DLL flakiness, mirrors other test files).
try:
    import rclpy as _rclpy
    _warm = _rclpy.context.Context()
    _rclpy.init(context=_warm)
    _rclpy.shutdown(context=_warm)
except Exception:  # noqa: BLE001
    pass


class TestBridgeNode(unittest.TestCase):
    def setUp(self):
        import rclpy
        self._ctx = rclpy.context.Context()
        rclpy.init(context=self._ctx)

    def tearDown(self):
        import rclpy
        rclpy.shutdown(context=self._ctx)

    def test_defaults(self):
        node = HolooceanBridgeNode(context=self._ctx)
        self.assertEqual(str(node.get_parameter("host").value), "127.0.0.1")
        self.assertEqual(int(node.get_parameter("port").value), 47654)
        self.assertEqual(
            str(node.get_parameter("image_topic").value), "/camera/front/image_raw"
        )
        self.assertEqual(str(node.get_parameter("pose_topic").value), "/rov/pose")
        self.assertEqual(
            str(node.get_parameter("oracle_topic").value),
            "/perception/obstacles_oracle",
        )
        node.destroy_node()

    def test_graceful_without_server(self):
        """No sim server running: the node must not crash, just stay disconnected."""
        node = HolooceanBridgeNode(context=self._ctx)
        # Should return False and not raise.
        self.assertFalse(node._ensure_connected())
        # A timer tick with no connection must be a no-op.
        node._on_timer()
        self.assertIsNone(node._stream)
        node.destroy_node()

    def test_cmd_vel_callback_sets_pending(self):
        from geometry_msgs.msg import Twist
        node = HolooceanBridgeNode(context=self._ctx)
        msg = Twist()
        msg.linear.x = 0.4
        msg.linear.y = -0.2
        msg.angular.z = 0.1
        node._on_cmd_vel(msg)
        self.assertIsNotNone(node._pending_cmd)
        self.assertAlmostEqual(node._pending_cmd["surge"], 0.4)
        self.assertAlmostEqual(node._pending_cmd["sway"], -0.2)
        self.assertAlmostEqual(node._pending_cmd["yaw_rate"], 0.1)
        node.destroy_node()

    def test_oracle_plus_y_is_left_of_image(self):
        """Calibrated convention: facing +X (yaw=0), world +Y -> image LEFT."""
        import rclpy
        from rclpy.node import Node

        received = []

        node = HolooceanBridgeNode(
            context=self._ctx,
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "oracle_topic", rclpy.parameter.Parameter.Type.STRING,
                    "/test/oracle"),
            ],
        )
        sub_node = Node("oracle_sub", context=self._ctx)
        sub_node.create_subscription(
            Obstacle2DArray, "/test/oracle", lambda m: received.append(m), 10)

        # Synthetic state: rover at origin facing +X (yaw=0), one obstacle 10 m
        # ahead and 5 m to the LEFT (world +Y).
        header = {
            "type": MSG_STATE,
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "depth": 0.0,
            "camera": {"horizontal_fov_deg": 90.0, "vertical_fov_deg": 90.0},
            "image": None,
            "obstacles": [
                {"name": "s", "class_name": "sphere",
                 "position": [10.0, 5.0, 0.0], "radius_m": 1.0},
            ],
        }
        node._publish_state(header, b"")

        exe = rclpy.executors.SingleThreadedExecutor(context=self._ctx)
        exe.add_node(node)
        exe.add_node(sub_node)
        for _ in range(50):
            exe.spin_once(timeout_sec=0.02)
            if received:
                break

        self.assertTrue(received, "no oracle message received")
        arr = received[-1]
        self.assertEqual(len(arr.obstacles), 1)
        self.assertLess(arr.obstacles[0].center_x, 0.5,
                        "world +Y must project to the LEFT half of the image")

        node.destroy_node()
        sub_node.destroy_node()

    def test_oracle_semantic_anchor_bounds_publish_one_detection(self):
        import rclpy
        from rclpy.node import Node

        received = []

        node = HolooceanBridgeNode(
            context=self._ctx,
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "oracle_topic", rclpy.parameter.Parameter.Type.STRING,
                    "/test/anchor_oracle"),
            ],
        )
        sub_node = Node("anchor_oracle_sub", context=self._ctx)
        sub_node.create_subscription(
            Obstacle2DArray, "/test/anchor_oracle", lambda m: received.append(m), 10)

        header = {
            "type": MSG_STATE,
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "velocity": {"x": 0.3, "y": 0.0, "z": 0.0},
            "depth": 0.0,
            "camera": {"horizontal_fov_deg": 90.0, "vertical_fov_deg": 90.0},
            "image": None,
            "obstacles": [
                {
                    "name": "anchor_center",
                    "class_name": "anchor",
                    "position": [8.0, 0.0, 0.0],
                    "radius_m": 2.4,
                    "bounds": {
                        "min": [7.8, -1.8, -1.8],
                        "max": [8.2, 1.8, 1.8],
                    },
                    "part_count": 6,
                },
            ],
        }
        node._publish_state(header, b"")

        exe = rclpy.executors.SingleThreadedExecutor(context=self._ctx)
        exe.add_node(node)
        exe.add_node(sub_node)
        for _ in range(50):
            exe.spin_once(timeout_sec=0.02)
            if received:
                break

        self.assertTrue(received, "no oracle message received")
        arr = received[-1]
        self.assertEqual(len(arr.obstacles), 1)
        obstacle = arr.obstacles[0]
        self.assertEqual(obstacle.class_name, "anchor")
        self.assertGreater(obstacle.width, 0.0)
        self.assertGreater(obstacle.height, 0.0)
        self.assertGreater(obstacle.risk, 0.55)
        self.assertTrue(obstacle.is_tracking_valid)

        node.destroy_node()
        sub_node.destroy_node()

    def test_quaternion_from_yaw(self):
        import math
        q = quaternion_from_yaw(math.pi / 2)
        self.assertAlmostEqual(q.w, math.cos(math.pi / 4), places=6)
        self.assertAlmostEqual(q.z, math.sin(math.pi / 4), places=6)


if __name__ == "__main__":
    unittest.main()
