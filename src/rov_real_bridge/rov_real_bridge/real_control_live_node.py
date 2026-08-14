"""Real-vehicle adapter BELOW /planner/cmd_vel_safe (Phase 10).

This is the ONLY platform-specific layer of the real scientific stack.
Everything above /planner/cmd_vel_safe — the Phase-7B qualification, the
T2 estimator and Planner C/D — is the same source code that runs in
simulation (see docs/EXPERIMENTAL_BOUNDARY.md).

Why this is a separate node from `real_control_adapter_node.py`: that
node is the fail-closed reference whose transmit layer is intentionally
absent, and `test/test_safety_interlock.py` asserts by reading its source
that it contains no MAVLink transport. That contract must not be broken,
so the live transport lives here instead, behind the same interlock.

SAFETY, in the order the checks run
-----------------------------------
1. `SafetyInterlock`: LIVE mode AND vehicle_in_water AND
   allow_real_actuation, otherwise shadow (compute, publish, log, do not
   transmit). Anything unparseable fails closed.
2. Calibration gate: LIVE actuation is refused unless every axis of the
   command mapping is marked calibrated. An uncalibrated axis means the
   commanded m/s has no measured meaning on this vehicle.
3. Non-finite rejection BEFORE clamping (`command_mapping`), because a
   naive clamp turns NaN into full-scale forward.
4. Per-axis clamp to the pool envelope, in SI, before conversion.
5. Command watchdog: a `/planner/cmd_vel_safe` older than
   `command_timeout_s` produces neutral.
6. Fixed-rate transmission independent of planner arrival: ArduSub needs
   MANUAL_CONTROL at >= 4 Hz and disarms on FS_PILOT_TIMEOUT (3 s) if the
   stream stops — which is also the emergency stop of last resort.
7. Any exception in the transmit path produces neutral and latches a
   fault.

The vehicle runs in ALT_HOLD: ArduSub holds depth and attitude, the
scientific planner owns horizontal motion. `linear.z` from the planner is
therefore ignored BY DESIGN and the omission is logged, not silently
dropped.
"""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .command_mapping import (
    NEUTRAL,
    AxisCalibration,
    CommandMapping,
    twist_to_manual_control,
)
from .safety_interlock import SafetyInterlock, forbid_ground_truth_topics


class RealControlLiveNode(Node):
    def __init__(self, *, context=None, parameter_overrides=None) -> None:
        super().__init__("real_control_live",
                         context=context,
                         parameter_overrides=parameter_overrides)
        self.declare_parameter("command_topic", "/planner/cmd_vel_safe")
        self.declare_parameter("debug_topic", "/real/adapter_debug")
        self.declare_parameter("shadow_topic", "/real/shadow_cmd")
        self.declare_parameter("command_timeout_s", 1.0)
        self.declare_parameter("stream_rate_hz", 20.0)
        # SI envelope (pool profile); clamped BEFORE conversion.
        self.declare_parameter("max_surge_ms", 0.30)
        self.declare_parameter("max_sway_ms", 0.25)
        self.declare_parameter("max_yaw_rate_rads", 0.30)
        # Signed command mapping (see command_mapping.AxisCalibration).
        for axis in ("surge", "sway", "yaw"):
            self.declare_parameter(f"{axis}_k_pos", 0.0)
            self.declare_parameter(f"{axis}_k_neg", 0.0)
            self.declare_parameter(f"{axis}_db_pos", 0.0)
            self.declare_parameter(f"{axis}_db_neg", 0.0)
            self.declare_parameter(f"{axis}_min_command", 0.0)
            self.declare_parameter(f"{axis}_calibrated", False)
        self.declare_parameter("calibration_id", "uncalibrated")
        # Interlock
        self.declare_parameter("vehicle_in_water", False)
        self.declare_parameter("allow_real_actuation", False)
        self.declare_parameter("real_control_mode", "shadow")
        # MAVLink
        self.declare_parameter("mavlink_url", "udpin:0.0.0.0:14550")
        self.declare_parameter("ardusub_mode", "ALT_HOLD")

        self._interlock = SafetyInterlock.from_params({
            "vehicle_in_water": self.get_parameter("vehicle_in_water").value,
            "allow_real_actuation":
                self.get_parameter("allow_real_actuation").value,
            "real_control_mode":
                self.get_parameter("real_control_mode").value,
        })
        self._mapping = self._build_mapping()

        cmd_topic = str(self.get_parameter("command_topic").value)
        err = forbid_ground_truth_topics([cmd_topic])
        if err:
            raise RuntimeError(err)

        self._timeout = float(self.get_parameter("command_timeout_s").value)
        self._last_twist = None
        self._last_twist_t = None
        self._lock = threading.Lock()
        self._fault = None
        self._link = None
        self._tx_count = 0
        self._reject_count = 0
        self._watchdog_trips = 0

        self.create_subscription(Twist, cmd_topic, self._on_cmd, 10)
        self._pub_debug = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10)
        self._pub_shadow = self.create_publisher(
            String, str(self.get_parameter("shadow_topic").value), 10)

        decision = self._interlock.evaluate()
        self._live = decision.may_actuate and self._mapping.fully_calibrated
        if decision.may_actuate and not self._mapping.fully_calibrated:
            self.get_logger().error(
                "LIVE refused: uncalibrated axes "
                f"{self._mapping.uncalibrated_axes()} — a commanded m/s "
                "has no measured meaning on this vehicle. Staying in "
                "shadow.")
        if self._live:
            self._open_link()
        self.get_logger().info(
            f"real adapter: {'LIVE' if self._live else 'SHADOW'} "
            f"({decision.reason}); calibration="
            f"{self._mapping.calibration_id}")

        rate = max(4.0, float(self.get_parameter("stream_rate_hz").value))
        self.create_timer(1.0 / rate, self._on_timer)

    # -- setup ---------------------------------------------------------
    def _build_mapping(self) -> CommandMapping:
        def axis(name, max_cmd):
            return AxisCalibration(
                k_pos=float(self.get_parameter(f"{name}_k_pos").value)
                or 1.0,
                k_neg=float(self.get_parameter(f"{name}_k_neg").value)
                or 1.0,
                db_pos=float(self.get_parameter(f"{name}_db_pos").value),
                db_neg=float(self.get_parameter(f"{name}_db_neg").value),
                min_command=float(
                    self.get_parameter(f"{name}_min_command").value),
                max_command=max_cmd,
                calibrated=bool(
                    self.get_parameter(f"{name}_calibrated").value),
                source=str(self.get_parameter("calibration_id").value),
            )
        return CommandMapping(
            surge=axis("surge",
                       float(self.get_parameter("max_surge_ms").value)),
            sway=axis("sway",
                      float(self.get_parameter("max_sway_ms").value)),
            yaw=axis("yaw",
                     float(self.get_parameter("max_yaw_rate_rads").value)),
            calibration_id=str(self.get_parameter("calibration_id").value),
        )

    def _open_link(self) -> None:
        try:
            from pymavlink import mavutil
            url = str(self.get_parameter("mavlink_url").value)
            self._link = mavutil.mavlink_connection(
                url, source_system=255, source_component=190)
            if self._link.wait_heartbeat(timeout=15) is None:
                raise RuntimeError(f"no vehicle heartbeat on {url}")
            self._hb_stop = threading.Event()
            t = threading.Thread(target=self._hb_loop, daemon=True)
            t.start()
            self.get_logger().info(f"MAVLink link up on {url}")
        except Exception as exc:            # fail closed
            self._fault = f"link open failed: {exc}"
            self._live = False
            self._link = None
            self.get_logger().error(self._fault)

    def _hb_loop(self) -> None:
        from pymavlink import mavutil
        while not self._hb_stop.is_set():
            try:
                self._link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            time.sleep(1.0)

    # -- runtime -------------------------------------------------------
    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._last_twist = msg
            self._last_twist_t = time.time()

    def _on_timer(self) -> None:
        with self._lock:
            twist = self._last_twist
            t_cmd = self._last_twist_t
        now = time.time()
        stale = (t_cmd is None) or (now - t_cmd > self._timeout)
        if stale and t_cmd is not None:
            self._watchdog_trips += 1

        if twist is None or stale:
            mc = dict(NEUTRAL)
            mc.update({"accepted": False,
                       "reason": "no command yet" if t_cmd is None
                                 else "stale command", "deadband": [],
                       "saturated": []})
            heave_in = 0.0
        else:
            mc = twist_to_manual_control(twist.linear.x, twist.linear.y,
                                         twist.angular.z, self._mapping)
            heave_in = twist.linear.z
            if not mc["accepted"]:
                self._reject_count += 1
                self.get_logger().warning(
                    f"rejected command: {mc['reason']}")

        sent = False
        if self._live and self._link is not None:
            try:
                self._link.mav.manual_control_send(
                    self._link.target_system, int(mc["x"]), int(mc["y"]),
                    int(mc["z"]), int(mc["r"]), 0)
                self._tx_count += 1
                sent = True
            except Exception as exc:        # fail closed
                self._fault = f"transmit failed: {exc}"
                self._live = False
                self.get_logger().error(self._fault)

        payload = {
            "t": now, "mode": "live" if self._live else "shadow",
            "sent": sent, "mc": {k: mc[k] for k in ("x", "y", "z", "r")},
            "accepted": mc["accepted"], "reason": mc["reason"],
            "deadband": mc.get("deadband", []),
            "saturated": mc.get("saturated", []),
            "twist_in": (None if twist is None else
                         [twist.linear.x, twist.linear.y, twist.angular.z]),
            "heave_ignored_by_design": heave_in,
            "watchdog_active": bool(stale),
            "watchdog_trips": self._watchdog_trips,
            "rejects": self._reject_count, "tx": self._tx_count,
            "calibration_id": self._mapping.calibration_id,
            "fault": self._fault,
        }
        out = String()
        out.data = json.dumps(payload)
        self._pub_debug.publish(out)
        self._pub_shadow.publish(out)

    def destroy_node(self) -> bool:
        try:
            if self._link is not None:
                for _ in range(5):
                    self._link.mav.manual_control_send(
                        self._link.target_system, 0, 0, NEUTRAL["z"], 0, 0)
                    time.sleep(0.05)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealControlLiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
