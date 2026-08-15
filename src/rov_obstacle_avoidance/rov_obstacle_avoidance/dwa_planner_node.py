"""ROS 2 node for the holonomic DWA baseline (Planner D).

INPUT PARITY with the committed planner (see docs/PLANNER_INPUT_EQUIVALENCE.md):
subscribes exactly `/perception/obstacles`, `/cmd_vel_nominal`,
`/rov/odom_estimated`; publishes `/planner/cmd_vel_safe` (Twist) and a
validator-only `/dwa/debug` JSON stream. No ground truth, no oracle pose,
no VelocitySensor, no thruster access — the comparison ends at Twist.

Safety semantics (mirroring the committed planner):
  - stale nominal command (>1 s)  -> zero output;
  - stale perception stream (>1 s; the T2 stack publishes continuously,
    silence means infrastructure failure) -> zero output;
  - no admissible candidate -> zero output + first-class logged event.
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2DArray
from std_msgs.msg import String

from .dwa_planner import (
    ObstacleMemory,
    DWAConfig,
    HolonomicDWA,
    ResponseVelocityEstimator,
    obstacle_from_detection,
)

FORBIDDEN = ("ground_truth", "pose_ground", "oracle", "obstacles_world")


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DWAPlannerNode(Node):
    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "dwa_planner",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("obstacle_topic", "/perception/obstacles")
        self.declare_parameter("nominal_topic", "/cmd_vel_nominal")
        self.declare_parameter("pose_topic", "/rov/odom_estimated")
        self.declare_parameter("output_topic", "/planner/cmd_vel_safe")
        self.declare_parameter("debug_topic", "/dwa/debug")
        self.declare_parameter("planner_rate_hz", 10.0)
        self.declare_parameter("command_timeout_s", 1.0)
        # Core DWA parameters exposed for the tuning phase.
        self.declare_parameter("w_clearance", 0.5)
        self.declare_parameter("w_progress", 1.0)
        self.declare_parameter("w_speed", 0.3)
        self.declare_parameter("w_route", 0.5)
        self.declare_parameter("w_smooth", 0.1)
        self.declare_parameter("safety_margin_m", 0.80)
        self.declare_parameter("horizon_s", 6.0)
        self.declare_parameter("clearance_saturation_m", 2.0)
        self.declare_parameter("max_surge", 0.5)
        # Exposed so the S3 vehicle profile can bound BOTH planners with
        # the same measured limits. Bounding only the committed planner
        # would confound the C-vs-D comparison with a constraint applied
        # to one side.
        self.declare_parameter("max_sway", 0.3)
        # Scenario class constants (pool-scale profile): shared monocular
        # assumptions, kept IDENTICAL to the committed planner per scenario.
        self.declare_parameter("target_obstacle_height_m", 3.5)
        self.declare_parameter("obstacle_radius_m", 1.75)
        self.declare_parameter("goal_lookahead_m", 4.0)

        topics = [str(self.get_parameter(p).value)
                  for p in ("obstacle_topic", "nominal_topic", "pose_topic")]
        for t in topics:
            for bad in FORBIDDEN:
                if bad in t:
                    raise RuntimeError(f"DWA must not consume {t!r}")

        cfg = DWAConfig(
            w_clearance=float(self.get_parameter("w_clearance").value),
            w_progress=float(self.get_parameter("w_progress").value),
            w_speed=float(self.get_parameter("w_speed").value),
            w_route=float(self.get_parameter("w_route").value),
            w_smooth=float(self.get_parameter("w_smooth").value),
            safety_margin_m=float(self.get_parameter("safety_margin_m").value),
            horizon_s=float(self.get_parameter("horizon_s").value),
            clearance_saturation_m=float(
                self.get_parameter("clearance_saturation_m").value),
            max_surge=float(self.get_parameter("max_surge").value),
            max_sway=float(self.get_parameter("max_sway").value),
            target_obstacle_height_m=float(
                self.get_parameter("target_obstacle_height_m").value),
            obstacle_radius_m=float(
                self.get_parameter("obstacle_radius_m").value),
            goal_lookahead_m=float(
                self.get_parameter("goal_lookahead_m").value),
        )
        self._dwa = HolonomicDWA(cfg)
        self._vel_est = ResponseVelocityEstimator(cfg)
        # Rolling odom-frame obstacle memory (see ObstacleMemory doc):
        # parity with the committed planner's commitment-state memory.
        self.declare_parameter("obstacle_memory_ttl_s", 30.0)
        self._memory = ObstacleMemory(
            ttl_s=float(self.get_parameter("obstacle_memory_ttl_s").value))
        self._cfg = cfg

        self._pose = None                 # (x, y, yaw)
        self._obstacles_msg = None
        self._obstacles_t = None
        self._nominal = None
        self._nominal_t = None
        self._route = None                # (x0, y0, yaw0)
        self._last_obstacles = []
        self._timeout = float(self.get_parameter("command_timeout_s").value)
        self._last_plan_t = None
        self._no_admissible_events = 0

        self.create_subscription(Obstacle2DArray, topics[0],
                                 self._on_obstacles, 10)
        self.create_subscription(Twist, topics[1], self._on_nominal, 10)
        self.create_subscription(PoseStamped, topics[2], self._on_pose, 20)
        self._pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10)
        self._pub_debug = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10)

        rate = max(1.0, float(self.get_parameter("planner_rate_hz").value))
        self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            "holonomic DWA baseline (Planner D): transferable inputs only "
            f"{topics} -> {self.get_parameter('output_topic').value}")

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_obstacles(self, msg: Obstacle2DArray) -> None:
        self._obstacles_msg = msg
        self._obstacles_t = self._now_s()

    def _on_nominal(self, msg: Twist) -> None:
        self._nominal = msg
        self._nominal_t = self._now_s()
        if self._route is None and abs(msg.linear.x) > 1e-3 \
                and self._pose is not None:
            self._route = self._pose
            self.get_logger().info(
                f"route captured at ({self._route[0]:.2f}, "
                f"{self._route[1]:.2f}) yaw "
                f"{math.degrees(self._route[2]):.1f} deg")

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = (msg.pose.position.x, msg.pose.position.y,
                      yaw_from_quaternion(msg.pose.orientation))

    def _publish(self, u: float, v: float, r: float) -> None:
        cmd = Twist()
        cmd.linear.x = float(u)
        cmd.linear.y = float(v)
        cmd.angular.z = float(r)
        self._pub.publish(cmd)
        now = self._now_s()
        if self._last_plan_t is not None:
            self._vel_est.update(u, v, r, now - self._last_plan_t)
        self._last_plan_t = now

    def _on_timer(self) -> None:
        now = self._now_s()
        # Staleness safety (same contract as the committed planner).
        if self._nominal is None or self._nominal_t is None \
                or now - self._nominal_t > self._timeout:
            self._publish(0.0, 0.0, 0.0)
            return
        if self._obstacles_t is None or now - self._obstacles_t > self._timeout:
            self._publish(0.0, 0.0, 0.0)
            self._emit_debug(None, reason="perception_stream_stale")
            return
        if self._pose is None:
            self._publish(0.0, 0.0, 0.0)
            return

        obstacles = []
        for ob in self._obstacles_msg.obstacles:
            # Edge-clip guard: when the bbox touches an image border the
            # apparent height under-measures and the monocular range
            # OVER-estimates (obstacle believed farther exactly during the
            # close pass). Hold the last reliable estimate instead.
            # Same-information rule, declared in PLANNER_INPUT_EQUIVALENCE.
            clipped = (ob.center_y + ob.height / 2.0 > 0.98
                       or ob.center_y - ob.height / 2.0 < 0.02
                       or ob.center_x + ob.width / 2.0 > 0.98
                       or ob.center_x - ob.width / 2.0 < 0.02)
            if clipped and self._last_obstacles:
                obstacles = list(self._last_obstacles)
                break
            obstacles.append(obstacle_from_detection(
                cx=float(ob.center_x), bbox_h=float(ob.height),
                pose_x=self._pose[0], pose_y=self._pose[1],
                pose_yaw=self._pose[2], cfg=self._cfg))
        if obstacles:
            self._last_obstacles = list(obstacles)
        self._memory.update(obstacles, now)

        nominal_surge = float(self._nominal.linear.x)
        res = self._dwa.plan(
            pose=self._pose,
            vel_est=(self._vel_est.u, self._vel_est.v, self._vel_est.r),
            obstacles=self._memory.active(now),
            route=self._route,
            nominal_surge=nominal_surge,
        )
        if res.no_admissible:
            self._no_admissible_events += 1
            self.get_logger().warning(
                "no_admissible_candidate — safe stop "
                f"(#{self._no_admissible_events})")
        self._publish(res.u, res.v, res.r)
        self._emit_debug(res)

    def _emit_debug(self, res, reason: str | None = None) -> None:
        msg = String()
        payload = {"t": self._now_s(), "planner": "dwa_holonomic",
                   "no_admissible_events": self._no_admissible_events}
        if reason:
            payload["reason"] = reason
        if res is not None:
            payload.update({
                "cmd": [res.u, res.v, res.r],
                "admissible": res.admissible_count,
                "candidates": res.candidate_count,
                "no_admissible": res.no_admissible,
                "best_cost": res.best_cost,
                "planning_time_ms": round(res.planning_time_ms, 2),
                "min_predicted_clearance": res.min_predicted_clearance,
                "vel_est": [round(self._vel_est.u, 3),
                            round(self._vel_est.v, 3),
                            round(self._vel_est.r, 3)],
            })
        msg.data = json.dumps(payload)
        self._pub_debug.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DWAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
