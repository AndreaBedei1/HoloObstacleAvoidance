"""Phase 9 progressive real-motion tests for the BlueROV2 (in water ONLY).

Safety model
------------
* RovLink context manager: neutral + disarm on ANY exit path.
* ArduSub FS_PILOT_INPUT=2 / FS_PILOT_TIMEOUT=3 s: if this process dies,
  the vehicle disarms itself 3 s after the MANUAL_CONTROL stream stops.
* Every sequence: arm -> short neutral -> ONE bounded pulse -> neutral
  coast -> disarm. Default power 15% (150/1000), duration <= 2 s.
* Full telemetry log (JSONL) for response reconstruction: ATTITUDE,
  VFR_HUD (depth/heading), SCALED_PRESSURE2, SERVO_OUTPUT_RAW (8 thruster
  PWMs), DISTANCE_SENSOR (Ping1D), HEARTBEAT (mode/arm), our commands.

Sequences
---------
  static   arm, 4 s neutral (verify thrusters stay 1500), disarm
  lights   flash Lights1 0 -> 50% -> 0 (no arming needed)
  surge+ / surge- / sway+ / sway- / yaw+ / yaw- / heave-
           one pulse on one axis (sign per ArduSub convention:
           sway+ = right, yaw+ = clockwise, heave- = down)
  step_surge / step_sway / step_yaw
           calibrated step: longer pulse (default 3 s) for time-constant
           estimation, still bounded

Usage:
  python scripts/real/rov_motion_test.py --seq static
  python scripts/real/rov_motion_test.py --seq surge+ --power 0.15 --dur 1.2
"""

import argparse
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rovlink import RovLink, Z_NEUTRAL  # noqa: E402

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "..", "..", "experiments", "real", "motion_tests")

AXES = {
    "surge+": dict(x=+1), "surge-": dict(x=-1),
    "sway+": dict(y=+1), "sway-": dict(y=-1),
    "yaw+": dict(r=+1), "yaw-": dict(r=-1),
    "heave-": dict(z=-1),   # down; heave+ (up) intentionally omitted at
                            # the surface — the vehicle already floats up
}


class TelemetryLogger:
    WANTED = ("ATTITUDE", "VFR_HUD", "SCALED_PRESSURE2",
              "SERVO_OUTPUT_RAW", "DISTANCE_SENSOR", "HEARTBEAT",
              "SYS_STATUS", "STATUSTEXT")

    def __init__(self, rov, path):
        self.rov = rov
        self.f = open(path, "w")
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self.t = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.t.start()

    def write(self, kind, payload):
        with self.lock:
            self.f.write(json.dumps(
                {"t": time.time(), "kind": kind, **payload},
                default=str) + "\n")

    def _loop(self):
        while not self._stop.is_set():
            m = self.rov.master.recv_match(blocking=True, timeout=0.5)
            if m is None or m.get_type() not in self.WANTED:
                continue
            d = m.to_dict()
            d.pop("mavpackettype", None)
            self.write(m.get_type(), d)

    def stop(self):
        self._stop.set()
        self.t.join(timeout=2)
        self.f.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seq", required=True,
                    choices=["static", "lights"] + list(AXES) +
                            ["step_surge", "step_sway", "step_yaw"])
    ap.add_argument("--power", type=float, default=0.15,
                    help="fraction of full authority (0..1), default 0.15")
    ap.add_argument("--dur", type=float, default=1.2,
                    help="pulse duration seconds")
    ap.add_argument("--coast", type=float, default=4.0,
                    help="post-pulse neutral observation seconds")
    ap.add_argument("--mode", default="MANUAL",
                    help="ArduSub mode for the test (MANUAL/STABILIZE/...)")
    args = ap.parse_args()
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    logpath = os.path.join(LOG_DIR, f"{args.seq}_{stamp}.jsonl")

    power = max(0.0, min(0.4, args.power))       # hard cap 40%
    dur = max(0.1, min(4.0, args.dur))           # hard cap 4 s
    mag = int(power * 1000)

    with RovLink() as rov:
        log = TelemetryLogger(rov, logpath)
        # ask for dense streams during the test
        from pymavlink import mavutil
        for mid, hz in [
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15),
                (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 15),
                (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 15),
                (mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 15),
                (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)]:
            rov.set_message_interval(mid, hz)
        log.start()
        log.write("meta", {"seq": args.seq, "power": power, "dur": dur,
                           "mode": args.mode})

        if args.seq == "lights":
            for level in (0.0, 0.5, 0.0):
                rov.lights(level)
                log.write("cmd", {"lights": level})
                time.sleep(1.5)
            log.stop()
            print("lights flashed; log:", logpath)
            return 0

        hb = rov.set_mode(args.mode)
        log.write("cmd", {"set_mode": args.mode, "result": hb})
        print("mode:", hb)
        hb = rov.arm()
        log.write("cmd", {"arm": True, "result": hb})
        print("armed:", hb)
        if not hb or not hb["armed"]:
            print("ARMING FAILED — aborting sequence")
            log.stop()
            return 1

        try:
            if args.seq == "static":
                t0 = time.time()
                while time.time() - t0 < 4.0:
                    rov.neutral()
                    time.sleep(0.1)
            else:
                base = args.seq.replace("step_", "")
                ax = AXES[base + "+"] if args.seq.startswith("step_") \
                    else AXES[args.seq]
                x = ax.get("x", 0) * mag
                y = ax.get("y", 0) * mag
                r = ax.get("r", 0) * mag
                z = Z_NEUTRAL + ax.get("z", 0) * mag
                if args.seq.startswith("step_"):
                    dur = max(dur, 3.0)
                # 1 s settle at neutral
                t0 = time.time()
                while time.time() - t0 < 1.0:
                    rov.neutral()
                    time.sleep(0.1)
                log.write("cmd", {"pulse": {"x": x, "y": y, "z": z,
                                            "r": r, "dur": dur}})
                print(f"pulse x={x} y={y} z={z} r={r} for {dur}s")
                rov.pulse(x=x, y=y, z=z, r=r, duration_s=dur, rate_hz=10)
                log.write("cmd", {"pulse_end": True})
                t0 = time.time()
                while time.time() - t0 < args.coast:
                    rov.neutral()
                    time.sleep(0.1)
        finally:
            hb = rov.disarm()
            log.write("cmd", {"disarm": True, "result": hb})
            print("disarmed:", hb)
            log.stop()
    print("log:", logpath)
    return 0


if __name__ == "__main__":
    sys.exit(main())
