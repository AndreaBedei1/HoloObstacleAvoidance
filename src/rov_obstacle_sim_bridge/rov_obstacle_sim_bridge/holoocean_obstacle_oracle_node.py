"""ROS 2 node that projects known obstacle positions into camera-space detections.

Subscribes to a simulated rover pose and publishes Obstacle2DArray using the
pure-Python oracle geometry module.  No HoloOcean dependency required.
"""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2D, Obstacle2DArray
from std_msgs.msg import Header

from .oracle_geometry import (
    CameraConfig,
    ObstacleConfig,
    ProjectedObstacle,
    RoverPose2D,
    load_obstacle_config_yaml,
    project_obstacles,
)


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw (rad) from a yaw-only quaternion (rotation about Z).

    yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _projected_to_obstacle_2d(
    proj: ProjectedObstacle,
    header: Header,
) -> Obstacle2D:
    """Convert a ProjectedObstacle dataclass into an Obstacle2D ROS message."""
    msg = Obstacle2D()
    msg.header = header
    msg.class_name = proj.class_name
    msg.confidence = float(proj.confidence)
    msg.center_x = float(proj.center_x)
    msg.center_y = float(proj.center_y)
    msg.width = float(proj.width)
    msg.height = float(proj.height)
    msg.bearing_rad = float(proj.bearing_rad)
    msg.apparent_area = float(proj.apparent_area)
    msg.risk = float(proj.risk)
    msg.is_tracking_valid = True
    return msg


class HolooceanObstacleOracleNode(Node):
    """Subscribe to simulated rover pose, publish Obstacle2DArray via oracle geometry."""

    def __init__(self) -> None:
        super().__init__("holoocean_obstacle_oracle")

        self._declare_parameters()

        # --- Load obstacle config at startup ---
        config_file = str(self.get_parameter("obstacle_config_file").value)
        if not config_file:
            raise RuntimeError(
                "Parameter 'obstacle_config_file' is empty. "
                "Provide a path to an obstacle YAML configuration file."
            )

        config_path = Path(config_file)
        if not config_path.is_file():
            raise RuntimeError(
                f"Obstacle config file not found: {config_file}"
            )

        try:
            self._obstacles: list[ObstacleConfig] = load_obstacle_config_yaml(config_path)
        except Exception as exc:
            raise RuntimeError(f"Failed to load obstacle config: {exc}") from exc

        self.get_logger().info(
            f"Loaded {len(self._obstacles)} obstacle(s) from {config_file}"
        )

        # --- Camera config from parameters ---
        self._camera = CameraConfig(
            horizontal_fov_deg=float(self.get_parameter("camera_horizontal_fov_deg").value),
            vertical_fov_deg=float(self.get_parameter("camera_vertical_fov_deg").value),
            min_detection_range_m=float(self.get_parameter("min_detection_range_m").value),
            max_detection_range_m=float(self.get_parameter("max_detection_range_m").value),
            confidence=float(self.get_parameter("confidence").value),
            risk_area_gain=float(self.get_parameter("risk_area_gain").value),
        )

        # --- Topic I/O ---
        rover_pose_topic = str(self.get_parameter("rover_pose_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        self.create_subscription(
            PoseStamped, rover_pose_topic, self._on_rover_pose, 10
        )

        self._publisher = self.create_publisher(Obstacle2DArray, output_topic, 10)

        # --- Latest pose cache ---
        self._latest_pose: PoseStamped | None = None

        # --- Timer ---
        rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_callback)

        self.get_logger().info(
            f"Oracle node subscribing to {rover_pose_topic}, "
            f"publishing {output_topic} at {rate_hz:.1f} Hz."
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter("rover_pose_topic", "/sim/rov_pose")
        self.declare_parameter("output_topic", "/perception/obstacles")
        self.declare_parameter("obstacle_config_file", "")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_id", "front_camera")
        self.declare_parameter("camera_horizontal_fov_deg", 90.0)
        self.declare_parameter("camera_vertical_fov_deg", 60.0)
        self.declare_parameter("min_detection_range_m", 0.2)
        self.declare_parameter("max_detection_range_m", 10.0)
        self.declare_parameter("confidence", 1.0)
        self.declare_parameter("risk_area_gain", 4.0)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_rover_pose(self, msg: PoseStamped) -> None:
        self._latest_pose = msg

    def _timer_callback(self) -> None:
        if self._latest_pose is None:
            self.get_logger().warn_throttle(
                5.0, "No rover pose received yet; skipping publish."
            )
            return

        header = self._latest_pose.header

        rover = RoverPose2D(
            x=self._latest_pose.pose.position.x,
            y=self._latest_pose.pose.position.y,
            z=self._latest_pose.pose.position.z,
            yaw_rad=yaw_from_quaternion(self._latest_pose.pose.orientation),
        )

        projected = project_obstacles(self._obstacles, rover, self._camera)

        array_msg = Obstacle2DArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = self._frame_id

        for p in projected:
            array_msg.obstacles.append(_projected_to_obstacle_2d(p, array_msg.header))

        self._publisher.publish(array_msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HolooceanObstacleOracleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
