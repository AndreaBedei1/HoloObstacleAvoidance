"""Record the full oracle demo pipeline to CSV for quantitative validation.

Subscribes to every topic in the oracle pipeline and writes one row per sample
interval.  No commands are published — this node is a passive logger only.
"""

from __future__ import annotations

import csv
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rov_obstacle_msgs.msg import AvoidanceDebug, Obstacle2D, Obstacle2DArray


# ---------------------------------------------------------------------------
# Pure-Python helpers (testable without ROS)
# ---------------------------------------------------------------------------

def summarize_obstacles(obstacles: list[Obstacle2D]) -> dict[str, float]:
    """Return obstacle_count, max_obstacle_risk, most_dangerous_center_x, most_dangerous_bearing_rad.

    When the list is empty all numeric fields are 0.0.
    """
    if not obstacles:
        return {
            "obstacle_count": 0,
            "max_obstacle_risk": 0.0,
            "most_dangerous_center_x": 0.0,
            "most_dangerous_bearing_rad": 0.0,
        }

    worst = max(obstacles, key=lambda o: o.risk)
    return {
        "obstacle_count": len(obstacles),
        "max_obstacle_risk": float(worst.risk),
        "most_dangerous_center_x": float(worst.center_x),
        "most_dangerous_bearing_rad": float(worst.bearing_rad),
    }


def twist_to_command_fields(twist: Twist | None) -> dict[str, float]:
    """Extract surge, sway, yaw_rate from a Twist; return 0.0 when None."""
    if twist is None:
        return {"surge": 0.0, "sway": 0.0, "yaw_rate": 0.0}
    return {
        "surge": float(twist.linear.x),
        "sway": float(twist.linear.y),
        "yaw_rate": float(twist.angular.z),
    }


def build_csv_row(
    timestamp_s: float,
    rov_x: float | None,
    rov_y: float | None,
    rov_z: float | None,
    obstacle_summary: dict[str, float],
    nominal_fields: dict[str, float],
    safe_fields: dict[str, float],
    planner_state: str | None,
    selected_side: str | None,
    debug_risk: float | None,
) -> list[float | int | str]:
    """Assemble one CSV row from collected fields, filling missing values safely."""
    return [
        round(timestamp_s, 6),
        rov_x if rov_x is not None else 0.0,
        rov_y if rov_y is not None else 0.0,
        rov_z if rov_z is not None else 0.0,
        obstacle_summary.get("obstacle_count", 0),
        obstacle_summary.get("max_obstacle_risk", 0.0),
        obstacle_summary.get("most_dangerous_center_x", 0.0),
        obstacle_summary.get("most_dangerous_bearing_rad", 0.0),
        nominal_fields.get("surge", 0.0),
        nominal_fields.get("sway", 0.0),
        nominal_fields.get("yaw_rate", 0.0),
        safe_fields.get("surge", 0.0),
        safe_fields.get("sway", 0.0),
        safe_fields.get("yaw_rate", 0.0),
        planner_state if planner_state is not None else "",
        selected_side if selected_side is not None else "",
        debug_risk if debug_risk is not None else 0.0,
    ]


CSV_HEADER = [
    "timestamp_s",
    "rov_x",
    "rov_y",
    "rov_z",
    "obstacle_count",
    "max_obstacle_risk",
    "most_dangerous_center_x",
    "most_dangerous_bearing_rad",
    "nominal_surge",
    "nominal_sway",
    "nominal_yaw_rate",
    "safe_surge",
    "safe_sway",
    "safe_yaw_rate",
    "planner_state",
    "selected_side",
    "debug_risk",
]


class OracleDemoRecorderNode(Node):
    """Passive CSV recorder for the full oracle demo pipeline."""

    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "oracle_demo_recorder",
            context=context,
            parameter_overrides=parameter_overrides,
        )

        self._declare_parameters()

        # --- Parameters ---
        output_csv = str(self.get_parameter("output_csv").value)
        sample_rate_hz = max(0.1, float(self.get_parameter("sample_rate_hz").value))
        duration_s = float(self.get_parameter("duration_s").value)
        self._auto_shutdown = bool(self.get_parameter("auto_shutdown").value)

        # --- CSV setup ---
        self._csv_path = Path(output_csv)
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self._csv_path.open("w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CSV_HEADER)
        self._lock = threading.Lock()

        self.get_logger().info(f"Recording to {self._csv_path}")

        # --- Latest message cache (protected by _lock) ---
        self._rov_x: float | None = None
        self._rov_y: float | None = None
        self._rov_z: float | None = None
        self._obstacles: list[Obstacle2D] = []
        self._nominal_twist: Twist | None = None
        self._safe_twist: Twist | None = None
        self._planner_state: str | None = None
        self._selected_side: str | None = None
        self._debug_risk: float | None = None

        # --- Subscriptions ---
        self.create_subscription(
            PoseStamped, "/sim/rov_pose", self._on_rov_pose, 10
        )
        self.create_subscription(
            Obstacle2DArray, "/perception/obstacles", self._on_obstacles, 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_nominal", self._on_nominal, 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_safe", self._on_safe, 10
        )
        self.create_subscription(
            AvoidanceDebug, "/avoidance/debug", self._on_debug, 10
        )

        # --- Sampling timer ---
        self._start_time = self.get_clock().now()
        self._sample_timer = self.create_timer(1.0 / sample_rate_hz, self._sample)

        # --- Auto-shutdown timer (optional) ---
        if self._auto_shutdown and duration_s > 0.0:
            self._shutdown_timer = self.create_timer(duration_s, self._shutdown_once)
            self.get_logger().info(f"Auto-shutdown enabled after {duration_s}s")
        else:
            self._shutdown_timer: rclpy.timer.Timer | None = None

        self.get_logger().info(
            f"Oracle demo recorder started at {sample_rate_hz:.1f} Hz."
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter("output_csv", "logs/oracle_demo_record.csv")
        self.declare_parameter("sample_rate_hz", 10.0)
        self.declare_parameter("duration_s", 20.0)
        self.declare_parameter("auto_shutdown", False)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_rov_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._rov_x = msg.pose.position.x
            self._rov_y = msg.pose.position.y
            self._rov_z = msg.pose.position.z

    def _on_obstacles(self, msg: Obstacle2DArray) -> None:
        with self._lock:
            self._obstacles = list(msg.obstacles)

    def _on_nominal(self, msg: Twist) -> None:
        with self._lock:
            self._nominal_twist = msg

    def _on_safe(self, msg: Twist) -> None:
        with self._lock:
            self._safe_twist = msg

    def _on_debug(self, msg: AvoidanceDebug) -> None:
        with self._lock:
            self._planner_state = msg.current_state
            self._selected_side = msg.selected_side
            self._debug_risk = float(msg.risk)

    # ------------------------------------------------------------------
    # Sampling
    # ------------------------------------------------------------------
    def _sample(self) -> None:
        ts = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

        with self._lock:
            obstacle_summary = summarize_obstacles(self._obstacles)
            nominal_fields = twist_to_command_fields(self._nominal_twist)
            safe_fields = twist_to_command_fields(self._safe_twist)

            row = build_csv_row(
                timestamp_s=ts,
                rov_x=self._rov_x,
                rov_y=self._rov_y,
                rov_z=self._rov_z,
                obstacle_summary=obstacle_summary,
                nominal_fields=nominal_fields,
                safe_fields=safe_fields,
                planner_state=self._planner_state,
                selected_side=self._selected_side,
                debug_risk=self._debug_risk,
            )

        self._csv_writer.writerow(row)
        self._csv_file.flush()

    def _shutdown_once(self) -> None:
        self.get_logger().info("Duration reached, shutting down recorder.")
        self.on_shutdown()
        rclpy.shutdown()

    def on_shutdown(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
            self.get_logger().info(f"CSV closed: {self._csv_path}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OracleDemoRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
