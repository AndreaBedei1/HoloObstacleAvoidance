"""Active station-keeping: hold current heading for N seconds, then stop.

Use this instead of disarming when the vehicle must STAY PUT: with the
thrusters idle the ROV keeps rotating on its own inertia and a disarmed
vehicle has no control at all.

Usage: python scripts/real/rov_hold.py --seconds 30 [--keep-armed]
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rovlink import RovLink
from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--keep-armed", action="store_true")
    args = ap.parse_args()
    with RovLink() as rov:
        rov.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)
        rov.set_mode(args.mode)
        hb = rov.arm(mode=args.mode)
        print("armed:", hb)
        if not hb or not hb["armed"]:
            return 1
        samples = []
        yaw = rov.hold_heading(
            args.seconds,
            on_sample=lambda t, e, c: samples.append((round(t, 1),
                                                      round(e, 1), c)))
        print("hold done; final yaw deg:",
              None if yaw is None else round(yaw * 57.2958, 1))
        errs = [abs(e) for _, e, _ in samples]
        if errs:
            print(f"heading error: max {max(errs):.1f} deg, "
                  f"mean {sum(errs)/len(errs):.1f} deg, n={len(errs)}")
        if not args.keep_armed:
            print("disarmed:", rov.disarm())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
