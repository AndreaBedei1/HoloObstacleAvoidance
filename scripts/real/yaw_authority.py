"""Measure real yaw authority vs command (deadband characterization).

Observation 2026-08-14: at 12-15% command the thrusters spin but the
vehicle does not rotate — tether drag plus hull damping exceed the
produced torque. This sweeps the command level, measuring the actual
compass rotation per bounded pulse, with an ACTIVE heading hold between
levels (a disarmed or idle vehicle keeps drifting).

Output: degrees turned and mean rate per command level -> the deadband
and the usable control range for the servo/planner on the real vehicle.
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rovlink import RovLink  # noqa: E402

from pymavlink import mavutil  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "experiments", "real", "yaw_authority")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--levels", default="0.15,0.25,0.35,0.45")
    ap.add_argument("--pulse", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=6.0)
    ap.add_argument("--mode", default="STABILIZE")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    levels = [float(x) for x in args.levels.split(",")]
    results = []

    with RovLink() as rov:
        rov.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
        rov.set_mode(args.mode)
        hb = rov.arm(mode=args.mode)
        print("armed:", hb)
        if not hb or not hb["armed"]:
            return 1
        try:
            for lv in levels:
                mag = int(max(0.0, min(0.6, lv)) * 1000)
                att = rov.recv_match("ATTITUDE", timeout=2)
                if att is None:
                    print("no attitude; abort")
                    break
                y0 = att.yaw
                accum = 0.0
                prev = y0
                peak_rate = 0.0
                t0 = time.time()
                while time.time() - t0 < args.pulse:
                    rov.manual(r=mag)
                    a = rov.recv_match("ATTITUDE", timeout=0.3)
                    if a is not None:
                        accum += abs(wrap(a.yaw - prev))
                        prev = a.yaw
                        peak_rate = max(peak_rate, abs(a.yawspeed))
                    time.sleep(0.05)
                rov.neutral()
                # coast: how much does it keep turning after the stop?
                t1 = time.time()
                coast = 0.0
                pv = prev
                while time.time() - t1 < 3.0:
                    a = rov.recv_match("ATTITUDE", timeout=0.3)
                    if a is not None:
                        coast += abs(wrap(a.yaw - pv))
                        pv = a.yaw
                    rov.neutral()
                    time.sleep(0.05)
                rec = {
                    "level": lv, "cmd": mag,
                    "turned_deg": round(math.degrees(accum), 1),
                    "rate_deg_s": round(math.degrees(accum) / args.pulse, 1),
                    "peak_rate_deg_s": round(math.degrees(peak_rate), 1),
                    "coast_deg_3s": round(math.degrees(coast), 1),
                }
                results.append(rec)
                print(f"  {lv:.2f} -> {rec['turned_deg']:6.1f} deg in "
                      f"{args.pulse}s ({rec['rate_deg_s']:5.1f} deg/s, "
                      f"peak {rec['peak_rate_deg_s']:.1f}), "
                      f"coast {rec['coast_deg_3s']:.1f} deg")
                rov.hold_heading(args.hold)
        finally:
            rov.hold_heading(4.0)
            print("disarmed:", rov.disarm())
    stamp = time.strftime("%Y%m%d_%H%M%S")
    with open(os.path.join(OUT, f"yaw_authority_{stamp}.json"), "w") as f:
        json.dump(results, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
