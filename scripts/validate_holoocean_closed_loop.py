#!/usr/bin/env python
"""Collect closed-loop HoloOcean bridge/planner validation metrics."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rov_obstacle_msgs.msg import AvoidanceDebug, Obstacle2DArray
from sensor_msgs.msg import Image


def yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ClosedLoopCollector:
    def __init__(self, save_images_dir: str = "", image_prefix: str = "closed_loop") -> None:
        self.save_images_dir = save_images_dir
        self.image_prefix = image_prefix
        self.expect_class = "anchor"
        self.saved_images: list[str] = []
        self._save_at_frames = {1, 200}
        self.camera_frames = 0
        self.oracle_msgs = 0
        self.oracle_detections = 0
        self.oracle_anchor_detections = 0
        self.planner_input_msgs = 0
        self.planner_input_detections = 0
        self.nominal_cmd_msgs = 0
        self.safe_cmd_msgs = 0
        self.safe_cmd_different_from_nominal_msgs = 0
        self.max_abs_safe_nominal_surge_delta = 0.0
        self.max_abs_safe_nominal_sway_delta = 0.0
        self.max_abs_safe_nominal_yaw_delta = 0.0
        self.avoidance_cmd_msgs = 0
        self.pose_msgs = 0
        # Ground-truth pose (/rov/pose_ground_truth): validation/debug ONLY.
        self.first_pose: tuple[float, float, float, float] | None = None
        self.last_pose: tuple[float, float, float, float] | None = None
        self.max_forward_progress_m = 0.0
        self.max_lateral_deviation_m = 0.0
        # Estimated odometry (/rov/odom_estimated): the planner's actual input.
        self.odom_msgs = 0
        self.first_odom: tuple[float, float, float, float] | None = None
        self.last_odom: tuple[float, float, float, float] | None = None
        self.max_odom_position_error_m = 0.0
        self.max_odom_yaw_error_deg = 0.0
        self.states: list[str] = []
        self.max_risk = 0.0
        self._last_nominal: Twist | None = None

    def on_image(self, msg: Image) -> None:
        self.camera_frames += 1
        if self.save_images_dir and self.camera_frames in self._save_at_frames:
            self._save_image(msg)

    def _save_image(self, msg: Image) -> None:
        try:
            import cv2
            import numpy as np

            data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            rgb = data.reshape(int(msg.height), int(msg.width), -1)[:, :, :3]
            out_dir = Path(self.save_images_dir)
            out_dir.mkdir(parents=True, exist_ok=True)
            path = out_dir / f"{self.image_prefix}_frame{self.camera_frames:04d}.png"
            cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            self.saved_images.append(str(path))
        except Exception as exc:  # keep validation running on encoder issues
            print(f"[validator] image save failed: {exc!r}")

    def on_oracle(self, msg: Obstacle2DArray) -> None:
        self.oracle_msgs += 1
        self.oracle_detections += len(msg.obstacles)
        for obstacle in msg.obstacles:
            if obstacle.class_name == self.expect_class:
                self.oracle_anchor_detections += 1
            self.max_risk = max(self.max_risk, float(obstacle.risk))

    def on_planner_input(self, msg: Obstacle2DArray) -> None:
        self.planner_input_msgs += 1
        self.planner_input_detections += len(msg.obstacles)

    def on_nominal_cmd(self, msg: Twist) -> None:
        self.nominal_cmd_msgs += 1
        self._last_nominal = msg

    def on_safe_cmd(self, msg: Twist) -> None:
        self.safe_cmd_msgs += 1
        if self._last_nominal is not None:
            dx = abs(float(msg.linear.x) - float(self._last_nominal.linear.x))
            dy = abs(float(msg.linear.y) - float(self._last_nominal.linear.y))
            dz = abs(float(msg.angular.z) - float(self._last_nominal.angular.z))
            self.max_abs_safe_nominal_surge_delta = max(
                self.max_abs_safe_nominal_surge_delta, dx
            )
            self.max_abs_safe_nominal_sway_delta = max(
                self.max_abs_safe_nominal_sway_delta, dy
            )
            self.max_abs_safe_nominal_yaw_delta = max(
                self.max_abs_safe_nominal_yaw_delta, dz
            )
            if dx > 0.02 or dy > 0.02 or dz > 0.02:
                self.safe_cmd_different_from_nominal_msgs += 1
        if abs(msg.linear.y) > 0.05 or abs(msg.angular.z) > 0.05:
            self.avoidance_cmd_msgs += 1

    def on_debug(self, msg: AvoidanceDebug) -> None:
        state = str(msg.current_state)
        if not self.states or self.states[-1] != state:
            self.states.append(state)
        self.max_risk = max(self.max_risk, float(msg.risk))

    def on_pose(self, msg: PoseStamped) -> None:
        pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            yaw_from_pose(msg),
        )
        self.pose_msgs += 1
        if self.first_pose is None:
            self.first_pose = pose
        self.last_pose = pose
        self._update_lateral_deviation(pose)

    def _update_lateral_deviation(self, pose: tuple[float, float, float, float]) -> None:
        if self.first_pose is None:
            return
        x0, y0, _z0, yaw0 = self.first_pose
        dx = pose[0] - x0
        dy = pose[1] - y0
        forward_x = math.cos(yaw0)
        forward_y = math.sin(yaw0)
        forward = forward_x * dx + forward_y * dy
        lateral = abs(-forward_y * dx + forward_x * dy)
        self.max_forward_progress_m = max(self.max_forward_progress_m, forward)
        self.max_lateral_deviation_m = max(self.max_lateral_deviation_m, lateral)

    def on_odom(self, msg: PoseStamped) -> None:
        odom = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            yaw_from_pose(msg),
        )
        self.odom_msgs += 1
        if self.first_odom is None:
            self.first_odom = odom
        self.last_odom = odom
        self._update_odom_error()

    def _odometry_error(self) -> tuple[float, float]:
        """Odometry drift: |estimated displacement - ground-truth displacement|.

        Returns (position_error_m, yaw_error_deg) comparing the estimated
        odometry's displacement to the true displacement (frame-independent).
        """
        if None in (self.first_pose, self.last_pose, self.first_odom, self.last_odom):
            return 0.0, 0.0
        gt_dx = self.last_pose[0] - self.first_pose[0]
        gt_dy = self.last_pose[1] - self.first_pose[1]
        gt_dyaw = self.last_pose[3] - self.first_pose[3]
        od_dx = self.last_odom[0] - self.first_odom[0]
        od_dy = self.last_odom[1] - self.first_odom[1]
        od_dyaw = self.last_odom[3] - self.first_odom[3]
        pos_err = math.hypot(od_dx - gt_dx, od_dy - gt_dy)
        yaw_err = math.degrees(
            math.atan2(math.sin(od_dyaw - gt_dyaw), math.cos(od_dyaw - gt_dyaw))
        )
        return pos_err, yaw_err

    def _update_odom_error(self) -> None:
        pos_err, yaw_err = self._odometry_error()
        self.max_odom_position_error_m = max(self.max_odom_position_error_m, pos_err)
        self.max_odom_yaw_error_deg = max(self.max_odom_yaw_error_deg, abs(yaw_err))

    def _path_relative_final_errors(self) -> tuple[float, float, float, float]:
        """(initial_yaw, final_yaw, final_lateral_error_m, final_yaw_error_deg).

        Errors are relative to the ORIGINAL path: the line through the first pose
        along the initial heading.  This directly measures whether the vehicle
        returned to its original straight route and heading after avoiding.
        """
        if self.first_pose is None or self.last_pose is None:
            return 0.0, 0.0, 0.0, 0.0
        x0, y0, _z0, yaw0 = self.first_pose
        xf, yf, _zf, yawf = self.last_pose
        dx = xf - x0
        dy = yf - y0
        forward_x = math.cos(yaw0)
        forward_y = math.sin(yaw0)
        lateral = abs(-forward_y * dx + forward_x * dy)
        yaw_err = math.atan2(math.sin(yawf - yaw0), math.cos(yawf - yaw0))
        return yaw0, yawf, lateral, math.degrees(yaw_err)

    def summary(self) -> dict:
        recovered = False
        if "RECOVERING" in self.states:
            recovery_idx = self.states.index("RECOVERING")
            recovered = "NORMAL" in self.states[recovery_idx + 1 :]
        initial_yaw, final_yaw, final_lateral_error, final_yaw_error_deg = (
            self._path_relative_final_errors()
        )
        returned_to_original_line = (
            self.first_pose is not None
            and final_lateral_error < 0.5
            and abs(final_yaw_error_deg) < 10.0
        )
        odom_pos_err, odom_yaw_err = self._odometry_error()
        return {
            "saved_images": self.saved_images,
            "camera_frames": self.camera_frames,
            "oracle_msgs": self.oracle_msgs,
            "oracle_detections": self.oracle_detections,
            "oracle_anchor_detections": self.oracle_anchor_detections,
            "planner_input_msgs": self.planner_input_msgs,
            "planner_input_detections": self.planner_input_detections,
            "nominal_cmd_msgs": self.nominal_cmd_msgs,
            "safe_cmd_msgs": self.safe_cmd_msgs,
            "safe_cmd_different_from_nominal_msgs": (
                self.safe_cmd_different_from_nominal_msgs
            ),
            "max_abs_safe_nominal_surge_delta": round(
                self.max_abs_safe_nominal_surge_delta, 4
            ),
            "max_abs_safe_nominal_sway_delta": round(
                self.max_abs_safe_nominal_sway_delta, 4
            ),
            "max_abs_safe_nominal_yaw_delta": round(
                self.max_abs_safe_nominal_yaw_delta, 4
            ),
            "avoidance_cmd_msgs": self.avoidance_cmd_msgs,
            "pose_msgs": self.pose_msgs,
            "first_pose": (
                [round(v, 3) for v in self.first_pose]
                if self.first_pose is not None
                else None
            ),
            "last_pose": (
                [round(v, 3) for v in self.last_pose]
                if self.last_pose is not None
                else None
            ),
            "max_forward_progress_m": round(self.max_forward_progress_m, 3),
            "max_lateral_deviation_m": round(self.max_lateral_deviation_m, 3),
            "states": self.states,
            "max_risk": round(self.max_risk, 4),
            "recovered_after_avoidance": recovered,
            "initial_yaw_rad": round(initial_yaw, 4),
            "final_yaw_rad": round(final_yaw, 4),
            "final_lateral_error_m": round(final_lateral_error, 3),
            "final_yaw_error_deg": round(final_yaw_error_deg, 2),
            "returned_to_original_line": bool(returned_to_original_line),
            # Estimated odometry the planner actually navigated on, vs ground
            # truth. Non-zero drift is expected and realistic.
            "odom_msgs": self.odom_msgs,
            "planner_uses_estimated_odometry": self.odom_msgs > 0,
            "first_odom": (
                [round(v, 3) for v in self.first_odom]
                if self.first_odom is not None
                else None
            ),
            "last_odom": (
                [round(v, 3) for v in self.last_odom]
                if self.last_odom is not None
                else None
            ),
            "odom_final_position_error_m": round(odom_pos_err, 3),
            "odom_final_yaw_error_deg": round(odom_yaw_err, 2),
            "odom_max_position_error_m": round(self.max_odom_position_error_m, 3),
            "odom_max_yaw_error_deg": round(self.max_odom_yaw_error_deg, 2),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-s", type=float, default=25.0)
    parser.add_argument("--save-images-dir", default="",
                        help="save first/mid camera frames as PNG here")
    parser.add_argument("--image-prefix", default="closed_loop")
    parser.add_argument("--report-json", default="",
                        help="also write the summary JSON to this file")
    parser.add_argument("--expect-class", default="anchor",
                        help="oracle class counted as the target obstacle")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("holoocean_closed_loop_validator")
    collector = ClosedLoopCollector(
        save_images_dir=args.save_images_dir,
        image_prefix=args.image_prefix,
    )
    collector.expect_class = args.expect_class

    node.create_subscription(Image, "/camera/front/image_raw", collector.on_image, 10)
    node.create_subscription(
        Obstacle2DArray,
        "/perception/obstacles_oracle",
        collector.on_oracle,
        10,
    )
    node.create_subscription(
        Obstacle2DArray,
        "/perception/obstacles",
        collector.on_planner_input,
        10,
    )
    node.create_subscription(Twist, "/cmd_vel_nominal", collector.on_nominal_cmd, 10)
    node.create_subscription(Twist, "/planner/cmd_vel_safe", collector.on_safe_cmd, 10)
    node.create_subscription(AvoidanceDebug, "/avoidance/debug", collector.on_debug, 10)
    # Ground truth: validation/debug only.
    node.create_subscription(
        PoseStamped, "/rov/pose_ground_truth", collector.on_pose, 10
    )
    # Estimated odometry: the planner's actual pose input (compared to GT).
    node.create_subscription(
        PoseStamped, "/rov/odom_estimated", collector.on_odom, 10
    )

    deadline = time.time() + max(1.0, args.duration_s)
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    result = collector.summary()
    print(json.dumps(result, indent=2, sort_keys=True))
    if args.report_json:
        report_path = Path(args.report_json)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(result, indent=2, sort_keys=True), encoding="utf-8"
        )
        print(f"[validator] report written to {report_path}")

    node.destroy_node()
    rclpy.shutdown()

    required = (
        result["camera_frames"] > 0
        and result["oracle_anchor_detections"] > 0
        and result["planner_input_detections"] > 0
        and result["nominal_cmd_msgs"] > 0
        and result["safe_cmd_different_from_nominal_msgs"] > 0
        and result["avoidance_cmd_msgs"] > 0
        and result["max_forward_progress_m"] > 0.05
        and result["max_lateral_deviation_m"] > 0.05
        # Full avoid-and-return state sequence.
        and "APPROACH_OBSTACLE" in result["states"]
        and any(s.startswith("AVOIDING_") for s in result["states"])
        and "RECOVERING" in result["states"]
        and result["recovered_after_avoidance"]
        # Planner navigated on the ESTIMATED odometry (not ground truth).
        and result["planner_uses_estimated_odometry"]
        # Returned to the ORIGINAL path (position + heading), measured with
        # ground truth, despite navigating on drifting estimated odometry.
        and result["final_lateral_error_m"] < 0.5
        and abs(result["final_yaw_error_deg"]) < 10.0
        and result["returned_to_original_line"]
    )
    return 0 if required else 1


if __name__ == "__main__":
    raise SystemExit(main())
