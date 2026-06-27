"""Fake obstacle detector used before the camera neural detector exists."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import rclpy
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2D, Obstacle2DArray


SCENARIO_NONE = "none"
SCENARIO_CENTRAL_STATIC = "central_static"
SCENARIO_LEFT_STATIC = "left_static"
SCENARIO_RIGHT_STATIC = "right_static"
SCENARIO_CROSSING_LEFT_TO_RIGHT = "crossing_left_to_right"
SCENARIO_CROSSING_RIGHT_TO_LEFT = "crossing_right_to_left"
SCENARIO_APPROACHING = "approaching"
SCENARIO_DISAPPEARING = "disappearing"
SCENARIO_INTERMITTENT = "intermittent"

SUPPORTED_SCENARIOS = {
    SCENARIO_NONE,
    SCENARIO_CENTRAL_STATIC,
    SCENARIO_LEFT_STATIC,
    SCENARIO_RIGHT_STATIC,
    SCENARIO_CROSSING_LEFT_TO_RIGHT,
    SCENARIO_CROSSING_RIGHT_TO_LEFT,
    SCENARIO_APPROACHING,
    SCENARIO_DISAPPEARING,
    SCENARIO_INTERMITTENT,
}


@dataclass(frozen=True)
class FakeObstacleFields:
    center_x: float
    center_y: float
    width: float
    height: float
    bearing_rad: float
    risk: float


class FakeObstacleDetectorNode(Node):
    """Publish deterministic camera-space obstacle detections for planner testing."""

    def __init__(self) -> None:
        super().__init__("fake_obstacle_detector")
        self._declare_parameters()
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(Obstacle2DArray, output_topic, 10)
        self._start_time = time.monotonic()
        publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._publish)
        self.get_logger().info(f"Fake obstacle detector publishing {output_topic}.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("output_topic", "/perception/obstacles")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("obstacle_class", "obstacle")
        self.declare_parameter("confidence", 0.9)
        self.declare_parameter("center_x", 0.5)
        self.declare_parameter("center_y", 0.5)
        self.declare_parameter("width", 0.25)
        self.declare_parameter("height", 0.35)
        self.declare_parameter("bearing_rad", 0.0)
        self.declare_parameter("use_bearing_override", False)
        self.declare_parameter("horizontal_fov_deg", 90.0)
        self.declare_parameter("risk", 0.8)
        self.declare_parameter("scenario_mode", SCENARIO_CENTRAL_STATIC)
        self.declare_parameter("disappearing_after_s", 5.0)
        self.declare_parameter("intermittent_period_s", 2.0)
        self.declare_parameter("intermittent_visible_fraction", 0.5)

    def _publish(self) -> None:
        scenario = str(self.get_parameter("scenario_mode").value)
        if scenario not in SUPPORTED_SCENARIOS:
            self.get_logger().warning(
                f"Unsupported scenario_mode '{scenario}', publishing no obstacles."
            )
            scenario = SCENARIO_NONE

        message = Obstacle2DArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "front_camera"

        if scenario != SCENARIO_NONE:
            obstacle = self._build_obstacle(scenario)
            if obstacle is not None:
                obstacle.header = message.header
                message.obstacles.append(obstacle)

        self._publisher.publish(message)

    def _build_obstacle(self, scenario: str) -> Obstacle2D | None:
        elapsed_s = time.monotonic() - self._start_time
        class_name = str(self.get_parameter("obstacle_class").value)
        confidence = float(self.get_parameter("confidence").value)
        center_x = float(self.get_parameter("center_x").value)
        center_y = float(self.get_parameter("center_y").value)
        width = float(self.get_parameter("width").value)
        height = float(self.get_parameter("height").value)
        risk = float(self.get_parameter("risk").value)
        horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        bearing_override = None
        if bool(self.get_parameter("use_bearing_override").value):
            bearing_override = float(self.get_parameter("bearing_rad").value)

        fields = build_fake_obstacle_fields(
            scenario=scenario,
            elapsed_s=elapsed_s,
            center_x=center_x,
            center_y=center_y,
            width=width,
            height=height,
            risk=risk,
            horizontal_fov_deg=horizontal_fov_deg,
            bearing_override_rad=bearing_override,
            disappearing_after_s=float(self.get_parameter("disappearing_after_s").value),
            intermittent_period_s=float(self.get_parameter("intermittent_period_s").value),
            intermittent_visible_fraction=float(
                self.get_parameter("intermittent_visible_fraction").value
            ),
        )
        if fields is None:
            return None

        obstacle = Obstacle2D()
        obstacle.class_name = class_name
        obstacle.confidence = _clamp(confidence, 0.0, 1.0)
        obstacle.center_x = fields.center_x
        obstacle.center_y = fields.center_y
        obstacle.width = fields.width
        obstacle.height = fields.height
        obstacle.bearing_rad = fields.bearing_rad
        obstacle.apparent_area = fields.width * fields.height
        obstacle.risk = fields.risk
        obstacle.is_tracking_valid = True
        return obstacle


def build_fake_obstacle_fields(
    *,
    scenario: str,
    elapsed_s: float,
    center_x: float,
    center_y: float,
    width: float,
    height: float,
    risk: float,
    horizontal_fov_deg: float,
    bearing_override_rad: float | None = None,
    disappearing_after_s: float = 5.0,
    intermittent_period_s: float = 2.0,
    intermittent_visible_fraction: float = 0.5,
) -> FakeObstacleFields | None:
    if scenario == SCENARIO_DISAPPEARING and elapsed_s >= max(0.0, disappearing_after_s):
        return None
    if scenario == SCENARIO_INTERMITTENT:
        period_s = max(0.1, intermittent_period_s)
        visible_fraction = _clamp(intermittent_visible_fraction, 0.0, 1.0)
        phase = (elapsed_s % period_s) / period_s
        if phase >= visible_fraction:
            return None

    if scenario == SCENARIO_LEFT_STATIC:
        center_x = 0.25
    elif scenario == SCENARIO_RIGHT_STATIC:
        center_x = 0.75
    elif scenario == SCENARIO_CROSSING_LEFT_TO_RIGHT:
        center_x = _triangle_wave(elapsed_s, period_s=8.0)
    elif scenario == SCENARIO_CROSSING_RIGHT_TO_LEFT:
        center_x = 1.0 - _triangle_wave(elapsed_s, period_s=8.0)
    elif scenario == SCENARIO_APPROACHING:
        phase = 0.5 + 0.5 * math.sin(elapsed_s * 0.6)
        width = _clamp(width + phase * 0.25, 0.05, 0.9)
        height = _clamp(height + phase * 0.30, 0.05, 0.9)
        risk = _clamp(max(risk, 0.35 + phase * 0.6), 0.0, 1.0)

    center_x = _clamp(center_x, 0.0, 1.0)
    center_y = _clamp(center_y, 0.0, 1.0)
    width = _clamp(width, 0.0, 1.0)
    height = _clamp(height, 0.0, 1.0)

    # Camera bearing convention: left image side is negative, right image side is positive.
    bearing_rad = (
        bearing_override_rad
        if bearing_override_rad is not None
        else center_x_to_bearing_rad(center_x, horizontal_fov_deg)
    )
    return FakeObstacleFields(
        center_x=center_x,
        center_y=center_y,
        width=width,
        height=height,
        bearing_rad=bearing_rad,
        risk=_clamp(risk, 0.0, 1.0),
    )


def center_x_to_bearing_rad(center_x: float, horizontal_fov_deg: float) -> float:
    horizontal_fov_rad = math.radians(horizontal_fov_deg)
    return (_clamp(center_x, 0.0, 1.0) - 0.5) * horizontal_fov_rad


def _triangle_wave(elapsed_s: float, period_s: float) -> float:
    phase = (elapsed_s % period_s) / period_s
    if phase < 0.5:
        return phase * 2.0
    return 2.0 - phase * 2.0


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeObstacleDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
