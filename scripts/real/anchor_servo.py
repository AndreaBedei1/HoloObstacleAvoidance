"""Visual search-and-centre on the real anchor (Phase 9).

Closed loop, camera -> yaw, with the overhead RealSense as an
independent safety supervisor. This is the first real perception->control
loop of the project and the building block for shadow mode and for the
autonomous avoidance trials.

Behaviour
  SEARCH   no acceptable detection: rotate slowly (fixed rate, one
           direction, bounded total rotation) until the anchor appears
  CENTRE   detection accepted: yaw proportional to the bearing error
           (center_x - 0.5), deadband 0.06, saturated at `power`
  DONE     |error| < 0.06 on 3 consecutive frames

Acceptance gate (mirrors the Phase-7B qualification philosophy):
  score >= 0.55 AND bbox height fraction >= 0.12 AND width fraction
  >= 0.02 -- filters the small dark specks that the classical detector
  otherwise latches onto when the anchor is out of view.

SAFETY: overhead safe box (frame border) aborts; neutral+disarm on any
exit; ArduSub pilot-input failsafe is the backstop.
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_stream import CameraStream, ensure_env  # noqa: E402

ensure_env()

from anchor_detect import annotate, detect_anchor  # noqa: E402
from overhead_track import OverheadTracker  # noqa: E402
from rovlink import RovLink  # noqa: E402

import cv2  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "experiments", "real", "servo")

MIN_SCORE = 0.55
MIN_H = 0.12
MIN_W = 0.02


def accept(det) -> bool:
    return bool(det.get("found")
                and det.get("score", 0) >= MIN_SCORE
                and det.get("height", 0) >= MIN_H
                and det.get("width", 0) >= MIN_W)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--power", type=float, default=0.12)
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--search-dir", type=float, default=1.0)
    ap.add_argument("--margin", type=float, default=0.02)
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--hold-s", type=float, default=8.0,
                    help="closed-loop heading hold after the maneuver")
    ap.add_argument("--stay-armed", action="store_true",
                    help="keep holding at the end instead of disarming")
    ap.add_argument("--dry-run", action="store_true",
                    help="perception only: no arming, no motion")
    args = ap.parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, stamp)
    os.makedirs(sess, exist_ok=True)

    cam = CameraStream()
    tracker = OverheadTracker()
    log = []
    print("camera + overhead ready")
    try:
        ov = tracker.detect()
        near = ov.get("pixel") if ov and ov.get("found") else None
        print("overhead start:", ov.get("pixel"), ov.get("xyz_camera_m"))

        if args.dry_run:
            for i in range(12):
                fr, t = cam.latest()
                det = detect_anchor(fr)
                det["accepted"] = accept(det)
                log.append({"i": i, "onboard": det})
                print(f"  {i}: found={det['found']} score="
                      f"{det.get('score')} h={det.get('height')} "
                      f"cx={det.get('center_x')} acc={det['accepted']}")
                if i % 4 == 0:
                    cv2.imwrite(os.path.join(sess, f"dry_{i}.png"),
                                annotate(fr, det))
                time.sleep(0.4)
            with open(os.path.join(sess, "log.json"), "w") as f:
                json.dump(log, f, indent=2)
            print("session:", sess)
            return 0

        mag = int(max(0.0, min(0.25, args.power)) * 1000)
        state = "SEARCH"
        centred = 0
        with RovLink() as rov:
            rov.set_mode(args.mode)
            hb = rov.arm(mode=args.mode)
            print("armed:", hb)
            if not hb or not hb["armed"]:
                return 1
            t0 = time.time()
            last_ov = 0.0
            try:
                while time.time() - t0 < args.timeout:
                    fr, ft = cam.latest()
                    det = detect_anchor(fr)
                    ok = accept(det)
                    if ok:
                        err = det["center_x"] - 0.5
                        if abs(err) < 0.06:
                            centred += 1
                            r = 0
                            state = "CENTRED"
                        else:
                            centred = 0
                            state = "CENTRE"
                            r = int(max(-mag, min(mag, err * 4.0 * mag)))
                    else:
                        centred = 0
                        state = "SEARCH"
                        r = int(args.search_dir * mag * 0.8)
                    rov.manual(r=r)
                    log.append({"t": time.time() - t0, "state": state,
                                "r": r, "score": det.get("score"),
                                "cx": det.get("center_x"),
                                "h": det.get("height"),
                                "accepted": ok})
                    if len(log) % 10 == 1:
                        print(f"  {state:8s} r={r:+5d} score="
                              f"{det.get('score')} cx={det.get('center_x')}"
                              f" h={det.get('height')}")
                        cv2.imwrite(
                            os.path.join(sess, f"f{len(log):04d}.png"),
                            annotate(fr, det))
                    if centred >= 3:
                        print("== anchor CENTRED")
                        break
                    if time.time() - last_ov > 1.0:
                        last_ov = time.time()
                        ovd = tracker.detect(near=near)
                        if ovd and ovd.get("found"):
                            near = ovd["pixel"]
                            fx, fy = ovd["frac_x"], ovd["frac_y"]
                            m = args.margin
                            if not (m < fx < 1 - m and m < fy < 1 - m):
                                print("!! ABORT: overhead safe box",
                                      round(fx, 3), round(fy, 3))
                                break
                    time.sleep(0.1)
            finally:
                # ACTIVE stop (operator observation 2026-08-14): the
                # vehicle keeps rotating after the thrusters stop, and
                # disarming removes all control. Hold heading closed-loop
                # so it stays where the maneuver left it, THEN disarm.
                print("holding heading...")
                yaw_final = rov.hold_heading(args.hold_s)
                print("held; final yaw deg:",
                      None if yaw_final is None else round(
                          yaw_final * 57.2958, 1))
                if not args.stay_armed:
                    hb = rov.disarm()
                    print("disarmed:", hb)
                else:
                    print("LEFT ARMED holding (use rov_stop.py to end)")
        fr, _ = cam.latest()
        det = detect_anchor(fr)
        cv2.imwrite(os.path.join(sess, "final.png"), annotate(fr, det))
        print("final detection:", json.dumps(
            {k: det.get(k) for k in ("found", "score", "center_x",
                                     "height", "width")}))
    finally:
        cam.close()
        tracker.close()
    with open(os.path.join(sess, "log.json"), "w") as f:
        json.dump(log, f, indent=2)
    print("session:", sess)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
