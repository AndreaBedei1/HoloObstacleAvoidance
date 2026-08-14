"""Turn the ROV by a requested yaw delta using compass feedback.

STABILIZE mode, gentle rate, stops when the accumulated yaw change
reaches the target (or on timeout). Then optionally grabs a frame.

Usage: python scripts/real/rov_yaw_to.py --delta 180 [--power 0.15]
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rovlink import RovLink  # noqa: E402


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--delta", type=float, required=True,
                    help="signed yaw change in degrees (+ = clockwise)")
    ap.add_argument("--power", type=float, default=0.15)
    ap.add_argument("--timeout", type=float, default=45.0)
    args = ap.parse_args()
    target = math.radians(abs(args.delta))
    sign = 1 if args.delta >= 0 else -1
    mag = int(max(0.0, min(0.3, args.power)) * 1000)

    with RovLink() as rov:
        from pymavlink import mavutil
        rov.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)
        hb = rov.set_mode("STABILIZE")
        hb = rov.arm(mode="STABILIZE")
        print("armed:", hb)
        if not hb or not hb["armed"]:
            print("ARMING FAILED")
            return 1
        att = rov.recv_match("ATTITUDE", timeout=2)
        yaw0 = att.yaw
        accum = 0.0
        prev = yaw0
        t0 = time.time()
        try:
            while time.time() - t0 < args.timeout:
                rov.manual(r=sign * mag)
                att = rov.recv_match("ATTITUDE", timeout=0.5)
                if att is None:
                    continue
                accum += abs(wrap(att.yaw - prev))
                prev = att.yaw
                if accum >= target:
                    break
                time.sleep(0.05)
        finally:
            rov.neutral()
            time.sleep(1.0)
            hb = rov.disarm()
            print("disarmed:", hb)
        print(f"turned {math.degrees(accum):.0f} deg "
              f"(target {args.delta:+.0f}) in {time.time()-t0:.1f} s; "
              f"final yaw {math.degrees(prev):.0f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
