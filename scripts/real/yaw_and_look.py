"""Turn by a small yaw increment, then grab an onboard-camera frame.

Overhead-supervised: aborts if the ROV leaves the frame (safe box).
Used to bring the anchor into the onboard camera field of view.

Usage: python scripts/real/yaw_and_look.py --delta 20 [--shots 1]
"""

import argparse
import math
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overhead_track import OverheadTracker  # noqa: E402
from rovlink import RovLink  # noqa: E402

import cv2  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "visualizations", "real_rov_camera",
                   "yaw_scan")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def grab_frame(tag: str) -> str:
    os.makedirs(OUT, exist_ok=True)
    env = dict(os.environ)
    env["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = \
        "protocol_whitelist;file,rtp,udp|fflags;nobuffer|flags;low_delay"
    sub = os.path.join(OUT, tag)
    os.makedirs(sub, exist_ok=True)
    subprocess.run([sys.executable, os.path.join(HERE, "camera_grab.py"),
                    "--frames", "1", "--out", sub],
                   env=env, capture_output=True, timeout=60)
    files = sorted(os.listdir(sub))
    return os.path.join(sub, files[-1]) if files else ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--delta", type=float, default=20.0)
    ap.add_argument("--power", type=float, default=0.15)
    ap.add_argument("--margin", type=float, default=0.02)
    args = ap.parse_args()

    tracker = OverheadTracker()
    try:
        ref, _ = tracker.grab()
        mag = int(max(0.0, min(0.3, args.power)) * 1000)
        sign = 1 if args.delta >= 0 else -1
        target = math.radians(abs(args.delta))

        with RovLink() as rov:
            from pymavlink import mavutil
            rov.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)
            rov.set_mode("STABILIZE")
            hb = rov.arm(mode="STABILIZE")
            print("armed:", hb)
            if not hb or not hb["armed"]:
                return 1
            att = rov.recv_match("ATTITUDE", timeout=2)
            prev = att.yaw
            accum = 0.0
            t0 = time.time()
            last_check = 0.0
            try:
                while time.time() - t0 < 40 and accum < target:
                    rov.manual(r=sign * mag)
                    att = rov.recv_match("ATTITUDE", timeout=0.4)
                    if att:
                        accum += abs(wrap(att.yaw - prev))
                        prev = att.yaw
                    if time.time() - last_check > 1.0:
                        last_check = time.time()
                        cur, _ = tracker.grab()
                        det = tracker.detect_motion(ref, cur)
                        if det.get("found"):
                            fx, fy = det["frac_x"], det["frac_y"]
                            m = args.margin
                            if not (m < fx < 1 - m and m < fy < 1 - m):
                                print("!! ABORT: ROV at frame edge",
                                      round(fx, 3), round(fy, 3))
                                break
                    time.sleep(0.05)
            finally:
                rov.neutral()
                time.sleep(1.5)
                hb = rov.disarm()
                print("disarmed:", hb)
            print(f"turned {math.degrees(accum):.0f} deg, "
                  f"final yaw {math.degrees(prev):.0f} deg")
        tag = time.strftime("%H%M%S")
        path = grab_frame(tag)
        print("onboard frame:", path)
        cur, _ = tracker.grab()
        cv2.imwrite(os.path.join(OUT, f"overhead_{tag}.png"), cur)
    finally:
        tracker.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
