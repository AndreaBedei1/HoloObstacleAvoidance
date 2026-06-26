"""Publish a constant nominal velocity command for local avoidance demos."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class NominalCmdPublisherNode(Node):
    """Publish a constant forward /cmd_vel_nominal command."""

    def __init__(self) -> None:
        super().__init__("nominal_cmd_publisher")
        self.declare_parameter("output_topic", "/cmd_vel_nominal")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("surge", 0.3)
        self.declare_parameter("sway", 0.0)
        self.declare_parameter("heave", 0.0)
        self.declare_parameter("yaw_rate", 0.0)

        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish)
        self.get_logger().info(f"Nominal command publisher sending {output_topic}.")

    def _publish(self) -> None:
        msg = Twist()
        msg.linear.x = float(self.get_parameter("surge").value)
        msg.linear.y = float(self.get_parameter("sway").value)
        msg.linear.z = float(self.get_parameter("heave").value)
        msg.angular.z = float(self.get_parameter("yaw_rate").value)
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NominalCmdPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
