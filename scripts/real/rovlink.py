"""Shared pymavlink connection helper for the real BlueROV2 (Phase 9).

Connection: BlueOS "GCS Client Link" pushes MAVLink to this PC at
udp 0.0.0.0:14550 (verified 2026-08-14). We answer on the same socket,
identify as a GCS (sysid 255), and stream OUR heartbeat at 1 Hz — this
clears the GCS failsafe (vehicle showed MAV_STATE_CRITICAL without one).

SAFETY CONTRACT (every user of this module inherits it):
  * `RovLink` is a context manager: on ANY exit (including exceptions)
    it sends neutral MANUAL_CONTROL, then disarms, then closes.
  * ArduSub's own pilot-input failsafe (FS_PILOT_INPUT) is the hardware
    backstop: if our process dies mid-motion, the vehicle stops when the
    MANUAL_CONTROL stream stops.
  * The imaging/side-scan sonar (Cerulean, 192.168.2.86) is NEVER
    touched by anything in this package.

ArduSub custom modes (MAV_TYPE_SUBMARINE):
  0 STABILIZE, 1 ACRO, 2 ALT_HOLD, 3 AUTO, 4 GUIDED, 7 CIRCLE,
  9 SURFACE, 16 POSHOLD, 19 MANUAL, 20 MOTOR_DETECT
MANUAL_CONTROL axes (ArduSub): x surge [-1000..1000], y sway
[-1000..1000] (+ = right), z heave [0..1000] with 500 NEUTRAL
(0 = full down), r yaw [-1000..1000] (+ = clockwise/right).
"""

from __future__ import annotations

import json
import threading
import time

from pymavlink import mavutil

SUB_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
             4: "GUIDED", 7: "CIRCLE", 9: "SURFACE", 16: "POSHOLD",
             19: "MANUAL", 20: "MOTOR_DETECT"}
MODE_IDS = {v: k for k, v in SUB_MODES.items()}

Z_NEUTRAL = 500


class RovLink:
    def __init__(self, url: str = "udpin:0.0.0.0:14550",
                 heartbeat_hz: float = 1.0, timeout_s: float = 15.0):
        self.master = mavutil.mavlink_connection(
            url, source_system=255, source_component=190)
        hb = self.master.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            raise RuntimeError("no vehicle heartbeat on " + url)
        self._stop = threading.Event()
        self._hb_thread = threading.Thread(
            target=self._hb_loop, args=(heartbeat_hz,), daemon=True)
        self._hb_thread.start()

    # -- lifecycle -----------------------------------------------------
    def _hb_loop(self, hz: float) -> None:
        while not self._stop.is_set():
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1.0 / hz)

    def __enter__(self) -> "RovLink":
        return self

    def __exit__(self, *exc) -> None:
        try:
            self.neutral()
            time.sleep(0.2)
            self.disarm()
        finally:
            self._stop.set()
            self.master.close()

    # -- state ---------------------------------------------------------
    def recv_match(self, mtype, timeout=2.0, blocking=True):
        return self.master.recv_match(type=mtype, blocking=blocking,
                                      timeout=timeout)

    def heartbeat(self, timeout=3.0):
        m = self.recv_match("HEARTBEAT", timeout)
        if m is None:
            return None
        armed = bool(m.base_mode
                     & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return {"mode": SUB_MODES.get(m.custom_mode, m.custom_mode),
                "armed": armed,
                "system_status": m.system_status}

    def request_message(self, msg_id: int) -> None:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            float(msg_id), 0, 0, 0, 0, 0, 0)

    def set_message_interval(self, msg_id: int, hz: float) -> None:
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            float(msg_id), 1e6 / hz, 0, 0, 0, 0, 0)

    def get_param(self, name: str, timeout=3.0):
        self.master.param_fetch_one(name)
        m = self.recv_match("PARAM_VALUE", timeout)
        deadline = time.time() + timeout
        while m is not None and m.param_id != name \
                and time.time() < deadline:
            m = self.recv_match("PARAM_VALUE", timeout=0.5)
        return None if m is None or m.param_id != name else m.param_value

    # -- mode / arming -------------------------------------------------
    def set_mode(self, name: str, settle_s: float = 1.0) -> dict | None:
        self.master.set_mode(MODE_IDS[name])
        time.sleep(settle_s)
        return self.heartbeat()

    def arm(self, settle_s: float = 1.0) -> dict | None:
        self.master.arducopter_arm()
        time.sleep(settle_s)
        return self.heartbeat()

    def disarm(self, settle_s: float = 1.5) -> dict | None:
        self.master.arducopter_disarm()
        time.sleep(settle_s)
        return self.heartbeat()

    # -- motion --------------------------------------------------------
    def manual(self, x=0, y=0, z=Z_NEUTRAL, r=0, buttons=0) -> None:
        """One MANUAL_CONTROL frame (call repeatedly, >= 4 Hz)."""
        self.master.mav.manual_control_send(
            self.master.target_system, int(x), int(y), int(z), int(r),
            int(buttons))

    def neutral(self) -> None:
        self.manual(0, 0, Z_NEUTRAL, 0)

    def pulse(self, x=0, y=0, z=Z_NEUTRAL, r=0, duration_s=1.0,
              rate_hz=10.0, on_sample=None) -> None:
        """Hold a MANUAL_CONTROL setpoint for duration_s, then neutral.
        on_sample(t) is called every cycle for logging."""
        t0 = time.time()
        period = 1.0 / rate_hz
        while time.time() - t0 < duration_s:
            self.manual(x, y, z, r)
            if on_sample:
                on_sample(time.time() - t0)
            time.sleep(period)
        self.neutral()

    # -- lights (RC channel 9 override, ArduSub Lights1) ---------------
    def lights(self, level: float) -> None:
        """level 0.0..1.0 -> RC9 PWM 1100..1900."""
        pwm = int(1100 + max(0.0, min(1.0, level)) * 800)
        ch = [65535] * 18
        ch[8] = pwm
        self.master.mav.rc_channels_override_send(
            self.master.target_system, self.master.target_component,
            *ch[:18])


def jdump(path: str, obj) -> None:
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, default=str)
