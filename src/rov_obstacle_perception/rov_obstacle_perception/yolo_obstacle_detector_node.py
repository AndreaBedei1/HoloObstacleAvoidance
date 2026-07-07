"""YOLO-based visual obstacle detector (simulation-only).

Subscribes to the simulator camera (``/camera/front/image_raw``), runs YOLO
inference when the optional ``ultralytics`` package is installed, and
publishes ``rov_obstacle_msgs/Obstacle2DArray`` on ``/perception/obstacles``
- the same interface the oracle relay uses, so the planner stays unchanged.

Skeleton behaviour without ``ultralytics``/``torch`` installed: the node
starts, subscribes and logs a throttled warning, but publishes nothing (it
never publishes empty arrays it did not infer, so it cannot mask another
detector's output).

When running the YOLO detector against the HoloOcean bridge, disable the
bridge's oracle relay (``relay_oracle_topic:=''``) so ``/perception/obstacles``
has a single publisher.

The default configuration points to the fine-tuned custom-object model in
``training/yolo_custom_objects/``. The ``class_map`` parameter can still map
detector names to planner names when testing alternative weights.

Simulation-only: this node never commands any real vehicle hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from rov_obstacle_msgs.msg import Obstacle2D, Obstacle2DArray


@dataclass(frozen=True)
class YoloDetection:
    """Plain detection record (normalized xyxy box), ultralytics-free."""

    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(value, hi))


def center_x_to_bearing_rad(center_x: float, horizontal_fov_deg: float) -> float:
    """Same convention as the fake detector: 0.5 -> 0 rad, edges -> +/- fov/2."""
    horizontal_fov_rad = math.radians(horizontal_fov_deg)
    return (_clamp(center_x, 0.0, 1.0) - 0.5) * horizontal_fov_rad


def normalize_class_map(spec: str) -> dict:
    """Parse ``"boat:anchor, ship:anchor"`` into ``{"boat": "anchor", ...}``.

    Keys are lower-cased detector class names, values are planner class
    names.  Blank or malformed entries are ignored.
    """
    mapping: dict = {}
    for chunk in str(spec or "").split(","):
        if ":" not in chunk:
            continue
        src, _, dst = chunk.partition(":")
        src = src.strip().lower()
        dst = dst.strip()
        if src and dst:
            mapping[src] = dst
    return mapping


def detection_risk(
    center_x: float,
    apparent_area: float,
    confidence: float,
    risk_area_gain: float,
) -> float:
    """Camera-only risk in [0, 1] (no range available from a mono detector).

    55% centrality + 45% apparent area (saturated by ``risk_area_gain``),
    scaled by detection confidence — mirrors the oracle weighting minus the
    range/closing-speed terms it cannot know.
    """
    centrality = 1.0 - 2.0 * abs(_clamp(center_x, 0.0, 1.0) - 0.5)
    area_term = _clamp(apparent_area * max(0.0, risk_area_gain), 0.0, 1.0)
    risk = confidence * (0.55 * centrality + 0.45 * area_term)
    return _clamp(risk, 0.0, 1.0)


def convert_detections(
    detections,
    *,
    confidence_threshold: float,
    target_classes,
    class_map: dict,
    horizontal_fov_deg: float,
    risk_area_gain: float,
) -> list:
    """Filter + convert :class:`YoloDetection` records to obstacle field dicts.

    Pure helper (unit-testable without ultralytics or rclpy).
    """
    wanted = {str(c).strip().lower() for c in (target_classes or []) if str(c).strip()}
    fields: list = []
    for det in detections:
        name = str(det.class_name).strip().lower()
        if det.confidence < confidence_threshold:
            continue
        if wanted and name not in wanted:
            continue
        x_min = _clamp(min(det.x_min, det.x_max), 0.0, 1.0)
        x_max = _clamp(max(det.x_min, det.x_max), 0.0, 1.0)
        y_min = _clamp(min(det.y_min, det.y_max), 0.0, 1.0)
        y_max = _clamp(max(det.y_min, det.y_max), 0.0, 1.0)
        width = x_max - x_min
        height = y_max - y_min
        if width <= 0.0 or height <= 0.0:
            continue
        center_x = x_min + width / 2.0
        center_y = y_min + height / 2.0
        apparent_area = width * height
        fields.append(
            {
                "class_name": class_map.get(name, name),
                "confidence": float(det.confidence),
                "center_x": center_x,
                "center_y": center_y,
                "width": width,
                "height": height,
                "bearing_rad": center_x_to_bearing_rad(center_x, horizontal_fov_deg),
                "apparent_area": apparent_area,
                "risk": detection_risk(
                    center_x, apparent_area, float(det.confidence), risk_area_gain
                ),
            }
        )
    return fields


def image_msg_to_rgb_array(msg: Image) -> np.ndarray:
    """sensor_msgs/Image (rgb8/bgr8) -> HxWx3 uint8 RGB array."""
    encoding = str(msg.encoding).lower()
    if encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding: {msg.encoding!r}")
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    array = data.reshape(int(msg.height), int(msg.width), 3)
    if encoding == "bgr8":
        array = array[:, :, ::-1]
    return array


class YoloObstacleDetectorNode(Node):
    """Camera -> YOLO -> /perception/obstacles (planner unchanged)."""

    def __init__(self) -> None:
        super().__init__("yolo_obstacle_detector")
        self._declare_parameters()

        image_topic = str(self.get_parameter("image_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(Obstacle2DArray, output_topic, 10)
        self._subscription = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )

        self._model = None
        self._model_error: str = ""
        self._frame_counter = 0
        self._load_model()

        mode = "YOLO inference" if self._model is not None else (
            "SKELETON mode (ultralytics not available; publishing nothing)"
        )
        self.get_logger().info(
            f"yolo_obstacle_detector: {image_topic} -> {output_topic} [{mode}]"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/front/image_raw")
        self.declare_parameter("output_topic", "/perception/obstacles")
        self.declare_parameter("frame_id", "front_camera")
        # Any ultralytics-loadable weights: pretrained YOLO weights or the
        # fine-tuned custom-object model path.
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("confidence_threshold", 0.25)
        # Empty list = keep every class the model reports.
        self.declare_parameter("target_classes", [""])
        # "detector_name:planner_name" pairs, e.g. "boat:anchor".
        self.declare_parameter("class_map", "")
        self.declare_parameter("horizontal_fov_deg", 90.0)
        self.declare_parameter("risk_area_gain", 4.0)
        # Run inference every Nth frame (>=1); keeps CPU load sane at 30 fps.
        self.declare_parameter("inference_stride", 3)

    def _load_model(self) -> None:
        model_path = str(self.get_parameter("model_path").value)
        try:
            from ultralytics import YOLO  # optional dependency

            self._model = YOLO(model_path)
        except ImportError as exc:
            self._model = None
            self._model_error = f"ultralytics not installed ({exc})"
        except Exception as exc:  # bad weights path, torch issues, ...
            self._model = None
            self._model_error = f"model load failed ({exc})"

    def _on_image(self, msg: Image) -> None:
        self._frame_counter += 1
        if self._model is None:
            self.get_logger().warning(
                f"YOLO unavailable: {self._model_error}; not publishing "
                "(install ultralytics or set model_path to valid weights)",
                throttle_duration_sec=30.0,
            )
            return
        stride = max(1, int(self.get_parameter("inference_stride").value))
        if (self._frame_counter - 1) % stride != 0:
            return

        try:
            rgb = image_msg_to_rgb_array(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=30.0)
            return

        detections = self._infer(rgb)
        fields = convert_detections(
            detections,
            confidence_threshold=float(
                self.get_parameter("confidence_threshold").value
            ),
            target_classes=[
                str(v) for v in (self.get_parameter("target_classes").value or [])
            ],
            class_map=normalize_class_map(
                str(self.get_parameter("class_map").value)
            ),
            horizontal_fov_deg=float(
                self.get_parameter("horizontal_fov_deg").value
            ),
            risk_area_gain=float(self.get_parameter("risk_area_gain").value),
        )

        message = Obstacle2DArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        for f in fields:
            obstacle = Obstacle2D()
            obstacle.header = message.header
            obstacle.class_name = f["class_name"]
            obstacle.confidence = float(f["confidence"])
            obstacle.center_x = float(f["center_x"])
            obstacle.center_y = float(f["center_y"])
            obstacle.width = float(f["width"])
            obstacle.height = float(f["height"])
            obstacle.bearing_rad = float(f["bearing_rad"])
            obstacle.apparent_area = float(f["apparent_area"])
            obstacle.risk = float(f["risk"])
            obstacle.is_tracking_valid = True
            message.obstacles.append(obstacle)
        self._publisher.publish(message)

    def _infer(self, rgb: np.ndarray) -> list:
        """Run ultralytics on an RGB array, return YoloDetection records."""
        height, width = rgb.shape[0], rgb.shape[1]
        results = self._model.predict(
            source=rgb[:, :, ::-1],  # ultralytics expects BGR ndarrays
            device=str(self.get_parameter("device").value),
            verbose=False,
        )
        detections: list = []
        for result in results:
            names = result.names
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                xyxy = [float(v) for v in box.xyxy[0]]
                detections.append(
                    YoloDetection(
                        class_name=str(names.get(cls_id, cls_id)),
                        confidence=float(box.conf[0]),
                        x_min=xyxy[0] / width,
                        y_min=xyxy[1] / height,
                        x_max=xyxy[2] / width,
                        y_max=xyxy[3] / height,
                    )
                )
        return detections


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
