"""ROS 2 node for the transferable no-DVL commanded-motion odometry.

Publishes `/rov/odom_estimated` (PoseStamped, frame `odom`) — the SAME topic
the local avoidance planner already consumes, so the planner needs no change.

Runtime inputs (all with real-vehicle counterparts):
  * /planner/cmd_vel_safe      geometry_msgs/Twist   (commanded body velocity)
  * /rov/attitude_measured     geometry_msgs/Vector3Stamped (roll,pitch,yaw)
  * /rov/depth                 std_msgs/Float32

It does NOT subscribe to /rov/pose_ground_truth, /rov/velocity (synthetic
DVL), /perception/obstacles_oracle, or /ground_truth/* — enforced at startup
via forbidden-topic assertion and covered by tests.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Float32

import math

from .commanded_odometry import (
    CommandedOdometry,
    CommandedOdometryConfig,
    assert_topics_allowed,
)


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.z = math.sin(yaw_rad / 2.0)
    return q


class CommandedOdometryNode(Node):
    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "commanded_odometry",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("cmd_topic", "/planner/cmd_vel_safe")
        self.declare_parameter("attitude_topic", "/rov/attitude_measured")
        self.declare_parameter("depth_topic", "/rov/depth")
        self.declare_parameter("output_topic", "/rov/odom_estimated")
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("tau_surge_s", 1.2)
        self.declare_parameter("tau_sway_s", 1.15)
        self.declare_parameter("tau_heave_s", 0.8)
        self.declare_parameter("cmd_timeout_s", 1.0)
        # Clock source: "attitude_messages" integrates one step per attitude
        # message with dt = sensor_period_s (sensor-clocked dead reckoning,
        # standard on real vehicles and immune to sim-time dilation when the
        # engine falls below real time); "wall" uses the node timer.
        self.declare_parameter("clock_source", "attitude_messages")
        self.declare_parameter("sensor_period_s", 1.0 / 30.0)

        topics = [
            str(self.get_parameter("cmd_topic").value),
            str(self.get_parameter("attitude_topic").value),
            str(self.get_parameter("depth_topic").value),
        ]
        assert_topics_allowed(topics)

        cfg = CommandedOdometryConfig(
            tau_surge_s=float(self.get_parameter("tau_surge_s").value),
            tau_sway_s=float(self.get_parameter("tau_sway_s").value),
            tau_heave_s=float(self.get_parameter("tau_heave_s").value),
            cmd_timeout_s=float(self.get_parameter("cmd_timeout_s").value),
        )
        self._odo = CommandedOdometry(config=cfg)
        self._last_update_s: float | None = None
        self._sensor_clocked = (str(
            self.get_parameter("clock_source").value) == "attitude_messages")
        self._sensor_period = float(
            self.get_parameter("sensor_period_s").value)

        self.create_subscription(Twist, topics[0], self._on_cmd, 10)
        self.create_subscription(Vector3Stamped, topics[1], self._on_attitude, 10)
        self.create_subscription(Float32, topics[2], self._on_depth, 10)
        self._pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("output_topic").value), 10)

        rate = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            "commanded odometry (no-DVL transferable dead reckoning): "
            f"inputs {topics} -> {self.get_parameter('output_topic').value}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist) -> None:
        self._odo.set_command(msg.linear.x, msg.linear.y, msg.linear.z,
                              now_s=self._now_s())

    def _on_attitude(self, msg: Vector3Stamped) -> None:
        self._odo.set_measured_yaw(msg.vector.z)
        if self._sensor_clocked:
            # One integration step per sensor message at the nominal sensor
            # period: sim-time-consistent under engine load, wall-time-
            # consistent on the real vehicle.
            self._odo.update(dt=self._sensor_period, now_s=self._now_s())

    def _on_depth(self, msg: Float32) -> None:
        self._odo.set_measured_depth(msg.data)

    def _on_timer(self) -> None:
        now = self._now_s()
        if not self._sensor_clocked:
            if self._last_update_s is not None:
                self._odo.update(dt=now - self._last_update_s, now_s=now)
            self._last_update_s = now

        pose = self._odo.pose()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = pose["x"]
        msg.pose.position.y = pose["y"]
        msg.pose.position.z = pose["z"]
        msg.pose.orientation = quaternion_from_yaw(pose["yaw"])
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandedOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
