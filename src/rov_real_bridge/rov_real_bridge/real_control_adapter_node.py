"""Real-vehicle control adapter — SHADOW-FIRST, fail-closed.

Subscribes `/planner/cmd_vel_safe`, converts it to the real-vehicle command
representation, and:

- in `shadow` mode (DEFAULT): publishes the would-be command on
  `/real/shadow_cmd` and logs it. ZERO actuation.
- in `live` mode: requires `vehicle_in_water:=true` AND
  `allow_real_actuation:=true`; otherwise behaves exactly like shadow.

IMPORTANT: the actual MAVLink transmit layer is INTENTIONALLY NOT IMPLEMENTED
yet. Even with every interlock open, this node cannot move the vehicle today;
`_transmit()` raises NotImplementedError behind the gate. The transmit code
will be added only after the wet-test checklist
(docs/REAL_WET_TEST_SAFETY_CHECKLIST.md) is executed with Andrea present.

Telemetry/camera RECEPTION is unaffected by the interlock (read-only paths).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .safety_interlock import SafetyInterlock, forbid_ground_truth_topics

CMD_TOPIC = "/planner/cmd_vel_safe"
SHADOW_TOPIC = "/real/shadow_cmd"


class RealControlAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("real_control_adapter")

        self.declare_parameter("vehicle_in_water", False)
        self.declare_parameter("allow_real_actuation", False)
        self.declare_parameter("real_control_mode", "shadow")
        self.declare_parameter("max_surge", 0.3)
        self.declare_parameter("max_sway", 0.2)
        self.declare_parameter("max_yaw_rate", 0.3)

        params = {
            "vehicle_in_water": self.get_parameter("vehicle_in_water").value,
            "allow_real_actuation": self.get_parameter("allow_real_actuation").value,
            "real_control_mode": self.get_parameter("real_control_mode").value,
        }
        self.interlock = SafetyInterlock.from_params(params)

        subscriptions = [CMD_TOPIC]
        err = forbid_ground_truth_topics(subscriptions)
        if err:
            raise RuntimeError(err)

        self._sub = self.create_subscription(Twist, CMD_TOPIC, self._on_cmd, 10)
        self._shadow_pub = self.create_publisher(String, SHADOW_TOPIC, 10)

        decision = self.interlock.evaluate()
        self.get_logger().info(
            f"real control adapter up: mode={self.interlock.real_control_mode} "
            f"in_water={self.interlock.vehicle_in_water} "
            f"allow={self.interlock.allow_real_actuation} -> "
            f"may_actuate={decision.may_actuate} ({decision.reason})"
        )
        if decision.may_actuate:
            self.get_logger().warning(
                "LIVE actuation nominally enabled — but the transmit layer is "
                "not implemented; commands will still not be sent."
            )

    def _clamp(self, v: float, limit: float) -> float:
        return max(-limit, min(limit, float(v)))

    def _on_cmd(self, msg: Twist) -> None:
        surge = self._clamp(msg.linear.x, self.get_parameter("max_surge").value)
        sway = self._clamp(msg.linear.y, self.get_parameter("max_sway").value)
        yaw_rate = self._clamp(msg.angular.z,
                               self.get_parameter("max_yaw_rate").value)
        desc = f"surge={surge:.3f} sway={sway:.3f} yaw_rate={yaw_rate:.3f}"

        decision = self.interlock.gate(desc)
        shadow = String()
        shadow.data = (
            f"{'WOULD-SEND' if decision.may_actuate else 'BLOCKED'} {desc} "
            f"[{decision.reason}]"
        )
        self._shadow_pub.publish(shadow)

        if decision.may_actuate:
            self._transmit(surge, sway, yaw_rate)

    def _transmit(self, surge: float, sway: float, yaw_rate: float) -> None:
        # Deliberately unimplemented: see module docstring. Do NOT implement
        # without the wet-test checklist and explicit authorization.
        raise NotImplementedError(
            "Real actuation transmit layer is intentionally absent. "
            "See docs/REAL_WET_TEST_SAFETY_CHECKLIST.md."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealControlAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
