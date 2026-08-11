"""ROS 2 wrapper for the T0-T3 temporal estimators.

Pipeline position (planner unchanged):

    /perception/obstacles_raw  (relay/detector output)
      -> THIS NODE (method t0|t1|t2|t3)
      -> /perception/obstacles (planner input, same message type)
      -> /tracking/obstacles_debug (String JSON, validator-only)

Upstream conditions map 1:1 onto the core contract: a subscription callback
with detections = fresh; with an empty array = fresh-empty; no callback =
silence (only the output timer runs).

Planner-facing bearing/apparent_area/risk are recomputed IDENTICALLY for all
methods from (cx, cy, w, h, confidence) using the same formula as the YOLO
detector node, so no method gets an unfair representation.

Ground truth is never consumed here (guard on the input topic name).
"""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2D, Obstacle2DArray
from std_msgs.msg import String

from .temporal_core import (
    Detection,
    DetectionEvent,
    EstimatedObstacle,
    make_estimator,
)

FORBIDDEN_INPUT_SUBSTRINGS = ("ground_truth", "pose_ground", "oracle_pose")


def detection_risk(confidence: float, cx: float, area: float,
                   risk_area_gain: float = 4.0) -> float:
    """Same heuristic as the YOLO detector node (parity across methods)."""
    centrality = 1.0 - min(1.0, 2.0 * abs(cx - 0.5))
    area_term = min(1.0, max(0.0, area) * risk_area_gain)
    risk = confidence * (0.55 * centrality + 0.45 * area_term)
    return min(1.0, max(0.0, risk))


class TemporalEstimatorNode(Node):
    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "temporal_estimator",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("method", "t0")
        self.declare_parameter("input_topic", "/perception/obstacles_raw")
        self.declare_parameter("output_topic", "/perception/obstacles")
        self.declare_parameter("debug_topic", "/tracking/obstacles_debug")
        self.declare_parameter("output_rate_hz", 30.0)
        self.declare_parameter("horizontal_fov_deg", 90.0)
        self.declare_parameter("risk_area_gain", 4.0)
        self.declare_parameter("noise_model_path", "")

        input_topic = str(self.get_parameter("input_topic").value)
        for bad in FORBIDDEN_INPUT_SUBSTRINGS:
            if bad in input_topic:
                raise RuntimeError(
                    f"temporal estimator must not consume {input_topic!r}")

        self._method = str(self.get_parameter("method").value)
        nm_path = str(self.get_parameter("noise_model_path").value) or None
        self._est = make_estimator(self._method, noise_model_path=nm_path)
        self._hfov = math.radians(
            float(self.get_parameter("horizontal_fov_deg").value))
        self._gain = float(self.get_parameter("risk_area_gain").value)

        self.create_subscription(
            Obstacle2DArray, input_topic, self._on_raw, 10)
        self._pub = self.create_publisher(
            Obstacle2DArray, str(self.get_parameter("output_topic").value), 10)
        self._pub_debug = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10)

        rate = max(1.0, float(self.get_parameter("output_rate_hz").value))
        self.create_timer(1.0 / rate, self._on_tick)
        self._msg_count = 0
        self._empty_count = 0
        self.get_logger().info(
            f"temporal estimator method={self._est.name} "
            f"{input_topic} -> {self.get_parameter('output_topic').value}")
        if self._method.startswith("t3"):
            nm = getattr(self._est, "noise_model", None)
            self.get_logger().warn(
                f"T3 noise model: calibrated={getattr(nm, 'calibrated', False)} "
                f"source={getattr(nm, 'source', '?')}")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_raw(self, msg: Obstacle2DArray) -> None:
        self._msg_count += 1
        dets = [
            Detection(
                class_name=ob.class_name,
                confidence=float(ob.confidence),
                cx=float(ob.center_x), cy=float(ob.center_y),
                w=float(ob.width), h=float(ob.height),
            )
            for ob in msg.obstacles
        ]
        if not dets:
            self._empty_count += 1
        self._est.on_message(DetectionEvent(t=self._now_s(), detections=dets))

    def _to_msg(self, est: EstimatedObstacle, stamp) -> Obstacle2D:
        m = Obstacle2D()
        m.header.stamp = stamp
        m.header.frame_id = "front_camera"
        m.class_name = est.class_name
        m.confidence = float(est.confidence)
        m.center_x = float(est.cx)
        m.center_y = float(est.cy)
        m.width = float(min(1.0, est.w))
        m.height = float(min(1.0, est.h))
        m.bearing_rad = (est.cx - 0.5) * self._hfov
        m.apparent_area = float(est.w * est.h)
        m.risk = detection_risk(est.confidence, est.cx, m.apparent_area,
                                self._gain)
        # Estimator contract: a PUBLISHED track is planner-usable by
        # definition (bridging dropouts is the whole point of temporal
        # estimation); expired tracks are simply not published. The planner
        # drops is_tracking_valid=False obstacles (planner.py:493), so
        # held/predicted estimates must stay True. Predicted-ness is
        # reported on /tracking/obstacles_debug for the validator.
        m.is_tracking_valid = True
        return m

    def _on_tick(self) -> None:
        t = self._now_s()
        out = self._est.tick(t)
        if not out.publish:
            return  # T0 silence semantics: publish nothing at all
        stamp = self.get_clock().now().to_msg()
        arr = Obstacle2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = "front_camera"
        arr.obstacles = [self._to_msg(ob, stamp) for ob in out.obstacles]
        self._pub.publish(arr)

        dbg = String()
        dbg.data = json.dumps({
            "t": t,
            "method": self._est.name,
            "msg_count": self._msg_count,
            "empty_count": self._empty_count,
            "tracks": [
                {
                    "id": ob.track_id,
                    "class": ob.class_name,
                    "cx": round(ob.cx, 4), "cy": round(ob.cy, 4),
                    "w": round(ob.w, 4), "h": round(ob.h, 4),
                    "confidence": round(ob.confidence, 3),
                    "is_predicted": ob.is_predicted,
                    "age_s": round(ob.age_s, 3),
                    "time_since_meas_s": round(ob.time_since_meas_s, 3),
                    "debug": ob.debug,
                }
                for ob in out.obstacles
            ],
        })
        self._pub_debug.publish(dbg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TemporalEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
