"""Log /cmd_vel_safe commands for offline review during oracle demos."""

from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelSafeLoggerNode(Node):
    """Subscribe to /cmd_vel_safe and append every command to a CSV file."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_safe_logger")

        self._declare_parameters()

        topic = str(self.get_parameter("input_topic").value)
        log_file = str(self.get_parameter("log_file").value)
        flush_interval_s = float(self.get_parameter("flush_interval_s").value)

        self.create_subscription(Twist, topic, self._on_cmd_vel, 10)

        self._entries: list[dict[str, float]] = []
        self._path = Path(log_file) if log_file else None
        self._csv_file: Path | None = None
        self._csv_writer: csv.writer | None = None

        if self._path is not None:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = self._path.open("a", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "timestamp_s",
                "linear_x", "linear_y", "linear_z",
                "angular_x", "angular_y", "angular_z",
            ])
            self.get_logger().info(f"Logging /cmd_vel_safe to {self._path}")
        else:
            self.get_logger().info("Logging disabled (empty log_file parameter).")

        if flush_interval_s > 0.0 and self._csv_writer is not None:
            self.create_timer(flush_interval_s, self._flush)

        self.get_logger().info(f"Subscribed to {topic}.")

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/cmd_vel_safe")
        self.declare_parameter("log_file", "")
        self.declare_parameter("flush_interval_s", 1.0)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        ts = self.get_clock().now().nanoseconds * 1e-9

        entry = {
            "timestamp_s": round(ts, 6),
            "linear_x": round(msg.linear.x, 6),
            "linear_y": round(msg.linear.y, 6),
            "linear_z": round(msg.linear.z, 6),
            "angular_x": round(msg.angular.x, 6),
            "angular_y": round(msg.angular.y, 6),
            "angular_z": round(msg.angular.z, 6),
        }

        self._entries.append(entry)

        if len(self._entries) >= 10:
            self._flush()

    def _flush(self) -> None:
        if self._csv_writer is None or not self._entries:
            return

        for row in self._entries:
            self._csv_writer.writerow(list(row.values()))

        self._entries.clear()

        if self._csv_file is not None:
            self._csv_file.flush()

    def on_shutdown(self) -> None:
        self._flush()
        if self._csv_file is not None:
            self._csv_file.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CmdVelSafeLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
