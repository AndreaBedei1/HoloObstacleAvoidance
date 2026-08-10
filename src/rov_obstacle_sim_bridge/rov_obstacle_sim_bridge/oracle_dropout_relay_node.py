"""Oracle→planner perception relay with deterministic dropout injection.

Republishes `/perception/obstacles_oracle` onto `/perception/obstacles`
(keeping exactly ONE publisher on the planner's perception input) and can
inject a deterministic detector dropout for robustness testing:

  * `dropout_enabled:=true`
  * when the planner state (on `/avoidance/debug`) first matches
    `trigger_state_prefix` (default "AVOIDING"), a timer starts;
  * after `dropout_delay_s`, the relay publishes NOTHING for
    `dropout_duration_s` (mimicking YOLO losing the object mid-maneuver:
    silence, not empty arrays);
  * afterwards the relay resumes.

The window (trigger/start/end times) is logged and published as JSON on
`/sim/dropout_debug` for the validator.

Scientific labeling: this is part of the `simulation dynamics integration
baseline` — oracle perception, NOT a visual avoidance result.
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node
from rov_obstacle_msgs.msg import AvoidanceDebug, Obstacle2DArray
from std_msgs.msg import String


class OracleDropoutRelayNode(Node):
    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "oracle_dropout_relay",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("input_topic", "/perception/obstacles_oracle")
        self.declare_parameter("output_topic", "/perception/obstacles")
        self.declare_parameter("debug_topic", "/sim/dropout_debug")
        self.declare_parameter("dropout_enabled", False)
        self.declare_parameter("trigger_state_prefix", "AVOIDING")
        # Dropout is injected DURING the maneuver (3 s after AVOIDING starts,
        # vehicle committed and moving) matching the planner's design intent
        # and the closed-loop unit-test scenario. Injecting at ~1 s exposed a
        # known current-planner weakness (silence-staleness aborts an early
        # maneuver) — documented in docs/SCIENTIFIC_BASELINE_0.md, to be
        # addressed by the Phase 7 temporal estimators, not hidden here.
        self.declare_parameter("dropout_delay_s", 3.0)
        self.declare_parameter("dropout_duration_s", 2.0)
        # Graph-liveness gate: do not feed the planner until the runtime
        # navigation inputs are alive (prevents pose-less engagement races
        # seen as baseline0 run B_2).
        self.declare_parameter("require_nav_liveness", True)

        self._enabled = bool(self.get_parameter("dropout_enabled").value)
        self._prefix = str(self.get_parameter("trigger_state_prefix").value)
        self._delay = float(self.get_parameter("dropout_delay_s").value)
        self._duration = float(self.get_parameter("dropout_duration_s").value)

        self._trigger_time: float | None = None
        self._dropout_done = False
        self._was_dropping = False
        self._require_liveness = bool(
            self.get_parameter("require_nav_liveness").value)
        self._odom_alive = False
        self._attitude_alive = False
        self._liveness_logged = False

        self._pub = self.create_publisher(
            Obstacle2DArray, str(self.get_parameter("output_topic").value), 10)
        self._pub_debug = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10)
        self.create_subscription(
            Obstacle2DArray, str(self.get_parameter("input_topic").value),
            self._on_obstacles, 10)
        self.create_subscription(
            AvoidanceDebug, "/avoidance/debug", self._on_debug, 10)
        if self._require_liveness:
            self.create_subscription(
                PoseStamped, "/rov/odom_estimated", self._on_odom_alive, 1)
            self.create_subscription(
                Vector3Stamped, "/rov/attitude_measured",
                self._on_attitude_alive, 1)

        self.get_logger().info(
            f"oracle relay: dropout_enabled={self._enabled} "
            f"(trigger '{self._prefix}*' +{self._delay:.1f}s "
            f"for {self._duration:.1f}s)"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom_alive(self, _msg) -> None:
        self._odom_alive = True

    def _on_attitude_alive(self, _msg) -> None:
        self._attitude_alive = True

    def _nav_live(self) -> bool:
        if not self._require_liveness:
            return True
        live = self._odom_alive and self._attitude_alive
        if live and not self._liveness_logged:
            self._liveness_logged = True
            self.get_logger().info(
                "nav liveness confirmed (odom + attitude); relaying "
                "perception to the planner")
        return live

    def _on_debug(self, msg: AvoidanceDebug) -> None:
        if (self._enabled and self._trigger_time is None
                and msg.current_state.startswith(self._prefix)):
            self._trigger_time = self._now_s()
            self.get_logger().info(
                f"dropout trigger: state {msg.current_state}; window "
                f"[{self._delay:.1f}, {self._delay + self._duration:.1f}]s "
                "from now")
            self._emit_debug("triggered")

    def _dropping(self) -> bool:
        if not self._enabled or self._trigger_time is None or self._dropout_done:
            return False
        dt = self._now_s() - self._trigger_time
        if dt < self._delay:
            return False
        if dt < self._delay + self._duration:
            return True
        self._dropout_done = True
        return False

    def _emit_debug(self, event: str) -> None:
        msg = String()
        msg.data = json.dumps({
            "event": event,
            "t": self._now_s(),
            "trigger_time": self._trigger_time,
            "delay_s": self._delay,
            "duration_s": self._duration,
        })
        self._pub_debug.publish(msg)

    def _on_obstacles(self, msg: Obstacle2DArray) -> None:
        if not self._nav_live():
            return
        dropping = self._dropping()
        if dropping != self._was_dropping:
            self._emit_debug("dropout_start" if dropping else "dropout_end")
            self.get_logger().info(
                "dropout START (relay silent)" if dropping else "dropout END")
            self._was_dropping = dropping
        if not dropping:
            self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OracleDropoutRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
