"""ROS 2 node: realistic dead-reckoning odometry estimator.

Subscribes to the body-frame velocity + yaw rate (``/rov/velocity``, the DVL +
gyro signal from the bridge) and integrates it into a drifting pose estimate on
``/rov/odom_estimated``.  This is what the runtime planner consumes -- it never
sees the simulator ground-truth pose (``/rov/pose_ground_truth``), which is kept
for validation / debug only.

Sensor bias / scale / noise are configurable so the estimate accumulates
realistic odometry drift.  No HoloOcean, no MAVLink, no thrusters.
"""

from __future__ import annotations

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from rclpy.node import Node

from .odometry_estimator import OdometryEstimator, OdometryNoiseConfig


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.z = math.sin(yaw_rad / 2.0)
    return q


class OdometryEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("odometry_estimator")
        self._declare_parameters()

        rng = random.Random(int(self.get_parameter("seed").value))
        self._estimator = OdometryEstimator(
            self._noise_config(),
            x0=float(self.get_parameter("initial_x").value),
            y0=float(self.get_parameter("initial_y").value),
            z0=float(self.get_parameter("initial_z").value),
            yaw0=float(self.get_parameter("initial_yaw").value),
            rng=lambda: rng.gauss(0.0, 1.0),
        )
        self._frame_id = str(self.get_parameter("frame_id").value)
        output_topic = str(self.get_parameter("output_odom_topic").value)
        input_topic = str(self.get_parameter("input_velocity_topic").value)
        self._pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.create_subscription(TwistStamped, input_topic, self._on_velocity, 10)
        self._last_stamp_s: float | None = None
        self.get_logger().info(
            f"Odometry estimator: {input_topic} (DVL+gyro) -> {output_topic} "
            "(estimated, drifting). Ground truth is NOT used at runtime."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_velocity_topic", "/rov/velocity")
        self.declare_parameter("output_odom_topic", "/rov/odom_estimated")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("gyro_bias_rad_s", 0.0002)
        self.declare_parameter("gyro_noise_std_rad_s", 0.0015)
        self.declare_parameter("gyro_scale_error", 0.0)
        self.declare_parameter("dvl_bias_x_ms", 0.004)
        self.declare_parameter("dvl_bias_y_ms", 0.0015)
        self.declare_parameter("dvl_noise_std_ms", 0.01)
        self.declare_parameter("dvl_scale_error", 0.01)
        self.declare_parameter("seed", 12345)

    def _noise_config(self) -> OdometryNoiseConfig:
        return OdometryNoiseConfig(
            gyro_bias_rad_s=float(self.get_parameter("gyro_bias_rad_s").value),
            gyro_noise_std_rad_s=float(self.get_parameter("gyro_noise_std_rad_s").value),
            gyro_scale_error=float(self.get_parameter("gyro_scale_error").value),
            dvl_bias_x_ms=float(self.get_parameter("dvl_bias_x_ms").value),
            dvl_bias_y_ms=float(self.get_parameter("dvl_bias_y_ms").value),
            dvl_noise_std_ms=float(self.get_parameter("dvl_noise_std_ms").value),
            dvl_scale_error=float(self.get_parameter("dvl_scale_error").value),
        )

    def _on_velocity(self, msg: TwistStamped) -> None:
        stamp = msg.header.stamp
        now_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if now_s <= 0.0:
            now_s = self.get_clock().now().nanoseconds * 1e-9
        if self._last_stamp_s is None:
            self._last_stamp_s = now_s
            self._publish(msg.header.stamp)
            return
        dt = now_s - self._last_stamp_s
        self._last_stamp_s = now_s
        if dt <= 0.0:
            return
        self._estimator.update(
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
            float(msg.twist.angular.z),
            dt,
        )
        self._publish(msg.header.stamp)

    def _publish(self, stamp) -> None:
        state = self._estimator.state()
        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self._frame_id
        out.pose.position.x = state.x
        out.pose.position.y = state.y
        out.pose.position.z = state.z
        out.pose.orientation = quaternion_from_yaw(state.yaw)
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdometryEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
