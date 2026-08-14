"""Overhead-supervised ROV motion (Phase 9).

The overhead RealSense is the safety supervisor AND the observer:
  * before the move: reference frame + ROV located by motion differencing
  * during the move: frames captured continuously; if the ROV centroid
    leaves the SAFE BOX (default: 8-92% of the frame) the motion is
    ABORTED immediately (neutral + disarm)
  * after the move: displacement/rotation measured from the overhead view

The vehicle must stay inside the camera view: outside it there is
shallow water where it grounds (operator constraint 2026-08-14).

Usage:
  python scripts/real/supervised_move.py --axis yaw+ --power 0.15 --dur 4
  python scripts/real/supervised_move.py --axis surge+ --power 0.15 --dur 2
"""

import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overhead_track import OverheadTracker  # noqa: E402
from rovlink import RovLink, Z_NEUTRAL  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "experiments", "real", "supervised")

AXES = {"surge+": dict(x=+1), "surge-": dict(x=-1),
        "sway+": dict(y=+1), "sway-": dict(y=-1),
        "yaw+": dict(r=+1), "yaw-": dict(r=-1)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", required=True, choices=list(AXES))
    ap.add_argument("--power", type=float, default=0.15)
    ap.add_argument("--dur", type=float, default=3.0)
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--safe-margin", type=float, default=0.08,
                    help="fraction of frame kept as forbidden border")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, f"{args.axis}_{stamp}")
    os.makedirs(sess, exist_ok=True)

    mag = int(max(0.0, min(0.3, args.power)) * 1000)
    dur = max(0.2, min(8.0, args.dur))
    ax = AXES[args.axis]

    tracker = OverheadTracker()
    events = []
    try:
        ref, _ = tracker.grab()
        cv2.imwrite(os.path.join(sess, "ref.png"), ref)
        abort = threading.Event()
        frames = []

        last_px = [None]

        def watcher():
            while not abort.is_set():
                cur, _ = tracker.grab()
                if cur is None:
                    continue
                det = tracker.detect_motion(ref, cur, near=last_px[0])
                if det.get("found"):
                    last_px[0] = det["pixel"]
                det["wall"] = time.time()
                frames.append((det, cur))
                if det.get("found"):
                    fx, fy = det["frac_x"], det["frac_y"]
                    m = args.safe_margin
                    if not (m < fx < 1 - m and m < fy < 1 - m):
                        det["ABORT"] = "left safe box"
                        abort.set()
                        return
                time.sleep(0.15)

        with RovLink() as rov:
            from pymavlink import mavutil
            rov.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15)
            hb = rov.set_mode(args.mode)
            hb = rov.arm(mode=args.mode)
            print("armed:", hb)
            if not hb or not hb["armed"]:
                print("ARMING FAILED")
                return 1
            att0 = rov.recv_match("ATTITUDE", timeout=2)
            yaw0 = att0.yaw if att0 else None

            w = threading.Thread(target=watcher, daemon=True)
            w.start()
            t0 = time.time()
            aborted = False
            while time.time() - t0 < dur:
                if abort.is_set():
                    aborted = True
                    print("!! ABORT: ROV reached the safe-box boundary")
                    break
                rov.manual(x=ax.get("x", 0) * mag,
                           y=ax.get("y", 0) * mag,
                           z=Z_NEUTRAL,
                           r=ax.get("r", 0) * mag)
                time.sleep(0.1)
            rov.neutral()
            # settle and observe
            t1 = time.time()
            while time.time() - t1 < 3.0:
                rov.neutral()
                time.sleep(0.1)
            att1 = rov.recv_match("ATTITUDE", timeout=2)
            yaw1 = att1.yaw if att1 else None
            abort.set()
            w.join(timeout=3)
            hb = rov.disarm()
            print("disarmed:", hb)

        # post-analysis
        after, _ = tracker.grab()
        cv2.imwrite(os.path.join(sess, "after.png"), after)
        det_final = tracker.detect_motion(
            ref, after, save_path=os.path.join(sess, "after_det.png"),
            near=last_px[0])
        dets = [d for d, _ in frames if d.get("found")]
        for i, (d, img) in enumerate(frames[::max(1, len(frames)//6)]):
            cv2.imwrite(os.path.join(sess, f"seq_{i}.png"), img)
        summary = {
            "axis": args.axis, "power": args.power, "dur": dur,
            "mode": args.mode, "aborted": aborted,
            "yaw_change_deg": (None if yaw0 is None or yaw1 is None else
                               round(math.degrees(
                                   math.atan2(math.sin(yaw1 - yaw0),
                                              math.cos(yaw1 - yaw0))), 1)),
            "overhead_start_px": dets[0]["pixel"] if dets else None,
            "overhead_end_px": det_final.get("pixel"),
            "n_detections": len(dets),
            "track": [{"t": round(d["wall"], 2), "px": d.get("pixel"),
                       "area": d.get("area_px")} for d, _ in frames],
        }
        if dets and det_final.get("found"):
            p0 = np.array(dets[0]["pixel"])
            p1 = np.array(det_final["pixel"])
            summary["overhead_px_displacement"] = round(
                float(np.linalg.norm(p1 - p0)), 1)
        with open(os.path.join(sess, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        print(json.dumps({k: v for k, v in summary.items()
                          if k != "track"}, indent=1))
        print("session:", sess)
    finally:
        tracker.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
