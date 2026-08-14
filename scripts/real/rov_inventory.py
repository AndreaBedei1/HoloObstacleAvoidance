"""Phase 9 step 1: full re-inventory of the real BlueROV2 over MAVLink.

Read-only w.r.t. the vehicle (no arming, no motion, no parameter writes):
connects as GCS, collects versions, mode, arm state, key parameters,
sensor stream rates, and writes the machine-readable inventory to
docs/real_vehicle_reference/data/mavlink_inventory.json.
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rovlink import RovLink, SUB_MODES, jdump  # noqa: E402

from pymavlink import mavutil  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "docs", "real_vehicle_reference", "data")

PARAMS = [
    "FRAME_CONFIG", "FS_PILOT_INPUT", "FS_PILOT_TIMEOUT", "FS_GCS_ENABLE",
    "FS_LEAK_ENABLE", "FS_PRESS_ENABLE", "FS_TEMP_ENABLE",
    "ARMING_CHECK", "BATT_MONITOR", "BATT_LOW_VOLT", "BATT_CAPACITY",
    "MOT_PWM_MIN", "MOT_PWM_MAX", "PILOT_SPEED_UP", "PILOT_SPEED_DN",
    "WPNAV_SPEED", "GND_EXT_BUS", "RNGFND1_TYPE", "RNGFND1_ORIENT",
    "RNGFND1_MAX_CM", "RNGFND1_MIN_CM",
] + [f"SERVO{i}_FUNCTION" for i in range(1, 17)]


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    inv = {"generated_local": time.strftime("%Y-%m-%dT%H:%M:%S"),
           "connection": "udpin:0.0.0.0:14550 (BlueOS GCS Client Link)"}

    with RovLink() as rov:
        hb = rov.heartbeat()
        inv["heartbeat"] = hb
        print("heartbeat:", hb)

        rov.request_message(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
        av = rov.recv_match("AUTOPILOT_VERSION", timeout=5)
        if av is not None:
            fw = av.flight_sw_version
            inv["ardusub_version"] = "%d.%d.%d" % (
                (fw >> 24) & 0xFF, (fw >> 16) & 0xFF, (fw >> 8) & 0xFF)
            inv["board_version"] = av.board_version
            inv["capabilities_bits"] = av.capabilities
            try:
                inv["flight_custom_version"] = bytes(
                    av.flight_custom_version).decode(errors="ignore")
            except Exception:
                pass
        print("ardusub:", inv.get("ardusub_version"))

        params = {}
        for p in PARAMS:
            v = rov.get_param(p)
            params[p] = v
            print(f"  {p} = {v}")
        inv["parameters"] = params

        # Measure actual stream rates for the messages the pipeline needs.
        rates = {}
        for name, msg_id, want in [
                ("ATTITUDE", mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
                ("SCALED_PRESSURE2",
                 mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 10),
                ("VFR_HUD", mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10),
                ("SERVO_OUTPUT_RAW",
                 mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10),
                ("SYS_STATUS", mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)]:
            rov.set_message_interval(msg_id, want)
        time.sleep(1.0)
        t0 = time.time()
        counts = {}
        while time.time() - t0 < 5.0:
            m = rov.master.recv_match(blocking=True, timeout=1.0)
            if m is None:
                continue
            counts[m.get_type()] = counts.get(m.get_type(), 0) + 1
        for k in ("ATTITUDE", "SCALED_PRESSURE2", "VFR_HUD",
                  "SERVO_OUTPUT_RAW", "SYS_STATUS", "HEARTBEAT"):
            rates[k] = round(counts.get(k, 0) / 5.0, 2)
        inv["measured_stream_hz"] = rates
        print("stream rates:", rates)

        att = rov.recv_match("ATTITUDE", timeout=2)
        if att:
            inv["attitude_sample"] = {"roll": att.roll, "pitch": att.pitch,
                                      "yaw": att.yaw}
        vfr = rov.recv_match("VFR_HUD", timeout=2)
        if vfr:
            inv["vfr_sample"] = {"alt_(-depth)_m": vfr.alt,
                                 "heading_deg": vfr.heading}
        pr = rov.recv_match("SCALED_PRESSURE2", timeout=2)
        if pr:
            inv["pressure2_sample"] = {"press_abs_mbar": pr.press_abs,
                                       "temperature_cdeg": pr.temperature}

    out = os.path.join(OUT, "mavlink_inventory.json")
    jdump(out, inv)
    print("->", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
