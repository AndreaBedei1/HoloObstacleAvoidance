"""ROS 2 node wrapping the local obstacle avoidance planner."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rov_obstacle_msgs.msg import AvoidanceDebug, Obstacle2DArray

from .planner import (
    LocalAvoidancePlanner,
    ObstacleObservation,
    PlannerConfig,
    VelocityCommand,
)


class LocalAvoidancePlannerNode(Node):
    """Subscribe to nominal velocity and obstacle detections, publish safe velocity."""

    def __init__(self) -> None:
        super().__init__("local_avoidance_planner")
        self._declare_parameters()
        self._planner = LocalAvoidancePlanner(self._planner_config())
        obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        nominal_cmd_topic = str(self.get_parameter("nominal_cmd_topic").value)
        safe_cmd_topic = str(self.get_parameter("safe_cmd_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)
        self._debug_frame_id = str(self.get_parameter("debug_frame_id").value)
        self._safe_publisher = self.create_publisher(Twist, safe_cmd_topic, 10)
        self._debug_publisher = self.create_publisher(AvoidanceDebug, debug_topic, 10)
        self.create_subscription(Obstacle2DArray, obstacle_topic, self._on_obstacles, 10)
        self.create_subscription(Twist, nominal_cmd_topic, self._on_nominal_command, 10)
        planner_rate_hz = max(1.0, float(self.get_parameter("planner_rate_hz").value))
        self._timer = self.create_timer(1.0 / planner_rate_hz, self._publish_safe_command)
        self.get_logger().info(f"Local avoidance planner publishing {safe_cmd_topic}.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("obstacle_topic", "/perception/obstacles")
        self.declare_parameter("nominal_cmd_topic", "/cmd_vel_nominal")
        self.declare_parameter("safe_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("debug_topic", "/avoidance/debug")
        self.declare_parameter("debug_frame_id", "front_camera")
        self.declare_parameter("risk_enter_threshold", 0.55)
        self.declare_parameter("risk_exit_threshold", 0.30)
        self.declare_parameter("central_zone_min_x", 0.35)
        self.declare_parameter("central_zone_max_x", 0.65)
        self.declare_parameter("min_avoidance_hold_s", 1.0)
        self.declare_parameter("recovery_time_s", 2.0)
        self.declare_parameter("max_surge", 0.5)
        self.declare_parameter("min_surge_during_avoidance", 0.08)
        self.declare_parameter("avoidance_sway", 0.20)
        self.declare_parameter("avoidance_yaw_rate", 0.35)
        self.declare_parameter("command_timeout_s", 1.0)
        self.declare_parameter("planner_rate_hz", 20.0)

    def _planner_config(self) -> PlannerConfig:
        return PlannerConfig(
            risk_enter_threshold=float(self.get_parameter("risk_enter_threshold").value),
            risk_exit_threshold=float(self.get_parameter("risk_exit_threshold").value),
            central_zone_min_x=float(self.get_parameter("central_zone_min_x").value),
            central_zone_max_x=float(self.get_parameter("central_zone_max_x").value),
            min_avoidance_hold_s=float(self.get_parameter("min_avoidance_hold_s").value),
            recovery_time_s=float(self.get_parameter("recovery_time_s").value),
            max_surge=float(self.get_parameter("max_surge").value),
            min_surge_during_avoidance=float(
                self.get_parameter("min_surge_during_avoidance").value
            ),
            avoidance_sway=float(self.get_parameter("avoidance_sway").value),
            avoidance_yaw_rate=float(self.get_parameter("avoidance_yaw_rate").value),
            command_timeout_s=float(self.get_parameter("command_timeout_s").value),
        )

    def _on_nominal_command(self, msg: Twist) -> None:
        self._planner.update_nominal_command(_velocity_from_twist(msg), self._now_s())

    def _on_obstacles(self, msg: Obstacle2DArray) -> None:
        obstacles = [
            ObstacleObservation(
                class_name=obstacle.class_name,
                confidence=float(obstacle.confidence),
                center_x=float(obstacle.center_x),
                center_y=float(obstacle.center_y),
                width=float(obstacle.width),
                height=float(obstacle.height),
                bearing_rad=float(obstacle.bearing_rad),
                apparent_area=float(obstacle.apparent_area),
                risk=float(obstacle.risk),
                is_tracking_valid=bool(obstacle.is_tracking_valid),
            )
            for obstacle in msg.obstacles
        ]
        self._planner.update_obstacles(obstacles, self._now_s())

    def _publish_safe_command(self) -> None:
        output = self._planner.compute(self._now_s())
        safe_command = _twist_from_velocity(output.command)
        self._safe_publisher.publish(safe_command)

        debug = AvoidanceDebug()
        debug.header.stamp = self.get_clock().now().to_msg()
        debug.header.frame_id = self._debug_frame_id
        debug.current_state = output.state.value
        debug.selected_side = output.selected_side.value
        debug.risk = float(output.risk)
        debug.desired_surge = float(output.command.surge)
        debug.desired_sway = float(output.command.sway)
        debug.desired_yaw_rate = float(output.command.yaw_rate)
        self._debug_publisher.publish(debug)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _velocity_from_twist(msg: Twist) -> VelocityCommand:
    return VelocityCommand(
        surge=float(msg.linear.x),
        sway=float(msg.linear.y),
        heave=float(msg.linear.z),
        roll_rate=float(msg.angular.x),
        pitch_rate=float(msg.angular.y),
        yaw_rate=float(msg.angular.z),
    )


def _twist_from_velocity(command: VelocityCommand) -> Twist:
    msg = Twist()
    msg.linear.x = float(command.surge)
    msg.linear.y = float(command.sway)
    msg.linear.z = float(command.heave)
    msg.angular.x = float(command.roll_rate)
    msg.angular.y = float(command.pitch_rate)
    msg.angular.z = float(command.yaw_rate)
    return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalAvoidancePlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
