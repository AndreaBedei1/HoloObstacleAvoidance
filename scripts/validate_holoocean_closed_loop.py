#!/usr/bin/env python
"""Collect closed-loop HoloOcean bridge/planner validation metrics."""

from __future__ import annotations

import argparse
import json
import math
import time

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
    def __init__(self) -> None:
        self.camera_frames = 0
        self.oracle_msgs = 0
        self.oracle_detections = 0
        self.oracle_anchor_detections = 0
        self.planner_input_msgs = 0
        self.planner_input_detections = 0
        self.safe_cmd_msgs = 0
        self.avoidance_cmd_msgs = 0
        self.pose_msgs = 0
        self.first_pose: tuple[float, float, float, float] | None = None
        self.last_pose: tuple[float, float, float, float] | None = None
        self.max_lateral_deviation_m = 0.0
        self.states: list[str] = []
        self.max_risk = 0.0

    def on_image(self, _msg: Image) -> None:
        self.camera_frames += 1

    def on_oracle(self, msg: Obstacle2DArray) -> None:
        self.oracle_msgs += 1
        self.oracle_detections += len(msg.obstacles)
        for obstacle in msg.obstacles:
            if obstacle.class_name == "anchor":
                self.oracle_anchor_detections += 1
            self.max_risk = max(self.max_risk, float(obstacle.risk))

    def on_planner_input(self, msg: Obstacle2DArray) -> None:
        self.planner_input_msgs += 1
        self.planner_input_detections += len(msg.obstacles)

    def on_safe_cmd(self, msg: Twist) -> None:
        self.safe_cmd_msgs += 1
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
        lateral = abs(-forward_y * dx + forward_x * dy)
        self.max_lateral_deviation_m = max(self.max_lateral_deviation_m, lateral)

    def summary(self) -> dict:
        recovered = False
        if "RECOVERING" in self.states:
            recovery_idx = self.states.index("RECOVERING")
            recovered = "NORMAL" in self.states[recovery_idx + 1 :]
        return {
            "camera_frames": self.camera_frames,
            "oracle_msgs": self.oracle_msgs,
            "oracle_detections": self.oracle_detections,
            "oracle_anchor_detections": self.oracle_anchor_detections,
            "planner_input_msgs": self.planner_input_msgs,
            "planner_input_detections": self.planner_input_detections,
            "safe_cmd_msgs": self.safe_cmd_msgs,
            "avoidance_cmd_msgs": self.avoidance_cmd_msgs,
            "pose_msgs": self.pose_msgs,
            "max_lateral_deviation_m": round(self.max_lateral_deviation_m, 3),
            "states": self.states,
            "max_risk": round(self.max_risk, 4),
            "recovered_after_avoidance": recovered,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-s", type=float, default=25.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("holoocean_closed_loop_validator")
    collector = ClosedLoopCollector()

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
    node.create_subscription(Twist, "/planner/cmd_vel_safe", collector.on_safe_cmd, 10)
    node.create_subscription(AvoidanceDebug, "/avoidance/debug", collector.on_debug, 10)
    node.create_subscription(PoseStamped, "/rov/pose", collector.on_pose, 10)

    deadline = time.time() + max(1.0, args.duration_s)
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    result = collector.summary()
    print(json.dumps(result, indent=2, sort_keys=True))

    node.destroy_node()
    rclpy.shutdown()

    required = (
        result["camera_frames"] > 0
        and result["oracle_anchor_detections"] > 0
        and result["planner_input_detections"] > 0
        and result["avoidance_cmd_msgs"] > 0
        and result["max_lateral_deviation_m"] > 0.05
        and "APPROACH_OBSTACLE" in result["states"]
        and any(s.startswith("AVOIDING_") for s in result["states"])
    )
    return 0 if required else 1


if __name__ == "__main__":
    raise SystemExit(main())
