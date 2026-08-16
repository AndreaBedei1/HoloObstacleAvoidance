"""Real visual detector -> /perception/obstacles_raw (Phase 10).

This node is the REAL side of the observation domain. It turns BlueROV2
camera frames into exactly the same `Obstacle2DArray` contract that the
simulator's observation model publishes, so that everything downstream
(Phase-7B qualification, T2, Planner C/D) cannot tell the two apart.
See docs/EXPERIMENTAL_BOUNDARY.md.

The detector itself is the classical shape-prior pipeline validated in
the pilot session (`scripts/real/anchor_detect.py`): no trained weights
exist for this setup, and the simulator never had a detector at all, so
the detector is characterized as a front end rather than transferred.

Contract published (rov_obstacle_msgs/Obstacle2D), all image-space
fractions in [0, 1] except bearing:
    class_name        obstacle class label (parameter, default "anchor")
    confidence        detector score mapped to [0, 1]
    center_x/center_y bbox centre as a fraction of image width/height
    width/height      bbox size as a fraction of image width/height
    bearing_rad       (center_x - 0.5) * HFOV, + to the RIGHT of centre
                      (same convention the simulated relay uses)
    apparent_area     width * height
    risk              left at 0.0: the downstream estimator owns risk
    is_tracking_valid FALSE here by construction — this is a RAW
                      observation; validity is decided by the Phase-7B
                      qualifier downstream, which is the shared code

An EMPTY Obstacle2DArray is published on every cycle with no accepted
detection, because the qualifier's warm-up logic counts stream health
from empty messages: silence and "nothing seen" are different states.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2D, Obstacle2DArray
from std_msgs.msg import String

# The detector lives with the other real-vehicle tooling; import it by
# path so the ROS package does not have to vendor a copy (a copy would
# silently drift from the version used to characterize the front end).
_REAL_SCRIPTS = os.environ.get(
    "ROV_REAL_SCRIPTS",
    os.path.join(os.path.expanduser("~"), "Desktop",
                 "HoloObstacleAvoidance", "scripts", "real"))
if _REAL_SCRIPTS not in sys.path:
    sys.path.insert(0, _REAL_SCRIPTS)


class RealDetectorNode(Node):
    def __init__(self, *, context=None, parameter_overrides=None) -> None:
        super().__init__("real_detector", context=context,
                         parameter_overrides=parameter_overrides)
        self.declare_parameter("output_topic", "/perception/obstacles_raw")
        self.declare_parameter("health_topic", "/real/detector_health")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("class_name", "anchor")
        self.declare_parameter("frame_id", "front_camera")
        self.declare_parameter("camera_horizontal_fov_deg", 74.0)
        # Acceptance gate of the RAW detector (shape/quality only). The
        # temporal/coherence gate is the Phase-7B qualifier downstream.
        self.declare_parameter("min_score", 0.55)
        # Annotated recording of WHAT THE DETECTOR SAW. The operator
        # watches the vehicle, not the camera, so a run that fails to
        # avoid leaves no way to tell whether the anchor was out of
        # frame, seen and rejected, or seen too late. This node is the
        # only process that can write it: it owns the video port, and a
        # second reader cannot bind it.
        self.declare_parameter("annotated_video_path", "")
        self.declare_parameter("min_height_frac", 0.12)
        self.declare_parameter("min_width_frac", 0.02)
        self.declare_parameter("score_scale", 3.0)
        self.declare_parameter("warmup_s", 8.0)

        self._hfov = math.radians(
            float(self.get_parameter("camera_horizontal_fov_deg").value))
        self._pub = self.create_publisher(
            Obstacle2DArray,
            str(self.get_parameter("output_topic").value), 10)
        self._pub_health = self.create_publisher(
            String, str(self.get_parameter("health_topic").value), 10)

        from anchor_detect import detect_anchor          # noqa: E402
        from camera_stream import CameraStream           # noqa: E402
        self._detect = detect_anchor
        self._cam = CameraStream(
            warmup_s=float(self.get_parameter("warmup_s").value))
        self.get_logger().info(
            "real detector up: camera stream open, publishing "
            f"{self.get_parameter('output_topic').value}")

        self._n_pub = 0
        self._n_det = 0
        self._last_ms = 0.0
        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._on_timer)

    def _accept(self, det) -> bool:
        return bool(
            det.get("found")
            and det.get("score", 0.0)
            >= float(self.get_parameter("min_score").value)
            and det.get("height", 0.0)
            >= float(self.get_parameter("min_height_frac").value)
            and det.get("width", 0.0)
            >= float(self.get_parameter("min_width_frac").value))

    def _write_annotated(self, frame, det) -> None:
        """Draw the detection on the frame and append it to the video."""
        path = str(self.get_parameter("annotated_video_path").value or "")
        if not path:
            return
        try:
            import cv2
            if getattr(self, "_vw", None) is None:
                h, w = frame.shape[:2]
                # MJPG in AVI, not H.264 in MP4. An MP4 needs its index
                # written at close, and these runs are routinely killed
                # mid-flight: the first recorded video was unopenable
                # ("moov atom not found", zero readable frames). MJPG
                # stores each frame independently, so a truncated file
                # still plays up to where it stopped.
                if path.lower().endswith(".mp4"):
                    path = path[:-4] + ".avi"
                self._vw = cv2.VideoWriter(
                    path, cv2.VideoWriter_fourcc(*"MJPG"), 4.0, (w, h))
                self.get_logger().info("video rilevamenti -> %s" % path)
            vis = frame.copy()
            ok = bool(det and det.get("found"))
            acc = bool(det and self._accept(det))
            if ok:
                b = det.get("bbox_px")
                col = (0, 255, 0) if acc else (0, 165, 255)
                if b:
                    cv2.rectangle(vis, (b[0], b[1]),
                                  (b[0] + b[2], b[1] + b[3]), col, 4)
                cv2.putText(vis, "score %.2f  %s%s"
                            % (det.get("score") or 0.0,
                               "ACCETTATA" if acc else "scartata",
                               "  (ripiego)" if det.get("fallback") else ""),
                            (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, 3)
            else:
                cv2.putText(vis, "nessun rilevamento", (30, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            self._vw.write(vis)
        except Exception:
            pass

    def _on_timer(self) -> None:
        frame, t_frame = self._cam.latest()
        msg = Obstacle2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        det = None
        if frame is not None:
            det = self._detect(frame)
            self._write_annotated(frame, det)
            self._last_ms = float(det.get("ms", 0.0))
            if self._accept(det):
                o = Obstacle2D()
                o.header = msg.header
                o.class_name = str(self.get_parameter("class_name").value)
                scale = float(self.get_parameter("score_scale").value)
                o.confidence = float(
                    max(0.0, min(1.0, det["score"] / max(scale, 1e-6))))
                o.center_x = float(det["center_x"])
                o.center_y = float(det["center_y"])
                o.width = float(det["width"])
                o.height = float(det["height"])
                # + bearing to the RIGHT of image centre, matching the
                # convention of the simulated observation source.
                o.bearing_rad = float((det["center_x"] - 0.5) * self._hfov)
                o.apparent_area = float(det["width"] * det["height"])
                o.risk = 0.0
                o.is_tracking_valid = False   # RAW: qualifier decides
                msg.obstacles.append(o)
                self._n_det += 1
        # Always publish, even empty: the qualifier's warm-up counts
        # stream health, and "nothing seen" is not the same as silence.
        self._pub.publish(msg)
        self._n_pub += 1
        if self._n_pub % 10 == 0:
            health = String()
            health.data = json.dumps({
                "t": time.time(),
                "published": self._n_pub,
                "with_detection": self._n_det,
                "acceptance": round(self._n_det / max(1, self._n_pub), 3),
                "detector_ms": self._last_ms,
                "frames_decoded": self._cam.frames_decoded,
                "frame_age_s": (None if t_frame == 0 else
                                round(time.time() - t_frame, 3)),
                "last_score": None if det is None else det.get("score"),
            })
            self._pub_health.publish(health)

    def destroy_node(self) -> bool:
        try:
            self._cam.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
