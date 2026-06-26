"""Publish a simulated rover PoseStamped so the oracle node can run without HoloOcean."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.node import Node


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    """Return a yaw-only quaternion (zero pitch/roll)."""
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.z = math.sin(yaw_rad / 2.0)
    return q


class SimulatedRoverPosePublisherNode(Node):
    """Publish a simple /sim/rov_pose PoseStamped for oracle demos."""

    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "simulated_rover_pose_publisher",
            context=context,
            parameter_overrides=parameter_overrides,
        )

        self._declare_parameters()

        output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        self._publisher = self.create_publisher(PoseStamped, output_topic, 10)

        self._start_x = float(self.get_parameter("start_x").value)
        self._start_y = float(self.get_parameter("start_y").value)
        self._start_z = float(self.get_parameter("start_z").value)
        self._velocity_x = float(self.get_parameter("velocity_x").value)
        self._velocity_y = float(self.get_parameter("velocity_y").value)
        self._velocity_z = float(self.get_parameter("velocity_z").value)
        self._yaw_deg = float(self.get_parameter("yaw_deg").value)
        self._motion_mode = str(self.get_parameter("motion_mode").value)

        rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_pose)

        self._start_time = self.get_clock().now()

        self.get_logger().info(
            f"Simulated rover pose publisher on {output_topic} "
            f"(mode={self._motion_mode})."
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter("output_topic", "/sim/rov_pose")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("velocity_x", 0.2)
        self.declare_parameter("velocity_y", 0.0)
        self.declare_parameter("velocity_z", 0.0)
        self.declare_parameter("yaw_deg", 0.0)
        self.declare_parameter("motion_mode", "forward")

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _publish_pose(self) -> None:
        t = self._elapsed()

        if self._motion_mode == "static":
            x, y, z = self._start_x, self._start_y, self._start_z
            yaw_rad = math.radians(self._yaw_deg)
        elif self._motion_mode == "forward":
            x = self._start_x + self._velocity_x * t
            y = self._start_y + self._velocity_y * t
            z = self._start_z + self._velocity_z * t
            yaw_rad = math.radians(self._yaw_deg)
        elif self._motion_mode == "lateral":
            x = self._start_x + self._velocity_x * t
            y = self._start_y + self._velocity_y * math.sin(0.5 * t)
            z = self._start_z
            yaw_rad = math.radians(self._yaw_deg)
        elif self._motion_mode == "yaw_scan":
            x, y, z = self._start_x, self._start_y, self._start_z
            yaw_rad = math.radians(self._yaw_deg) + 0.3 * math.sin(0.4 * t)
        else:
            self.get_logger().warn_throttle(
                5.0, f"Unknown motion_mode '{self._motion_mode}', falling back to static."
            )
            x, y, z = self._start_x, self._start_y, self._start_z
            yaw_rad = math.radians(self._yaw_deg)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = quaternion_from_yaw(yaw_rad)

        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulatedRoverPosePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
