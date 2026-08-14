"""FIRST REAL CAMERA-BASED OBSTACLE AVOIDANCE RUN (Phase 9).

Simplified protocol agreed with the operator (2026-08-14): the ROV is
placed facing the anchor, in view; this script drives it forward and
avoids the anchor using ONLY the onboard camera.

Loop (10 Hz):
  APPROACH  surge forward at `--surge`, heading held closed-loop on the
            initial compass heading; the weight-free anchor detector
            runs on every frame; monocular range from the apparent
            width (one-point calibration, D-015)
  AVOID     range <= trigger -> add sway AWAY from the anchor bearing
            while keeping a reduced surge; the maneuver commits to the
            side chosen at trigger time (the committed-planner idea)
  CLEAR     anchor lost or bearing beyond `--clear-bearing` -> keep the
            sway briefly to pass the obstacle
  HOLD      active closed-loop heading hold: the vehicle coasts and
            keeps rotating if simply stopped, and disarming leaves it
            uncontrolled, so every run ENDS in an active hold

Safety
  * overhead RealSense supervisor: abort if the ROV reaches the frame
    border (outside the view the pool is too shallow and it grounds)
  * bounded power/duration, active hold on every exit path
  * ArduSub pilot-input failsafe as the ultimate backstop
"""

import argparse
import json
import math
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
from pymavlink import mavutil  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "experiments", "real", "avoid_runs")

# D-015 (one-point range calibration, 2026-08-14): the anchor's arm span
# is 0.80 m (operator) and measured 399 px wide at 2.56 m of overhead
# ground-truth distance -> f_px = 399*2.56/0.80 = 1277 px (HFOV ~74 deg
# underwater, consistent with the BlueROV2 low-light camera).
F_PX = 1277.0
ANCHOR_WIDTH_M = 0.80
IMG_W = 1920.0

MIN_SCORE = 0.55
MIN_H = 0.12
MIN_W = 0.02


def accept(det):
    return bool(det.get("found") and det.get("score", 0) >= MIN_SCORE
                and det.get("height", 0) >= MIN_H
                and det.get("width", 0) >= MIN_W)


def range_from_width(det):
    w_px = det["width"] * IMG_W
    if w_px < 5:
        return None
    return ANCHOR_WIDTH_M * F_PX / w_px


def bearing_deg(det):
    """Horizontal bearing of the anchor centre (deg, + = right)."""
    dx_px = (det["center_x"] - 0.5) * IMG_W
    return math.degrees(math.atan2(dx_px, F_PX))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--surge", type=float, default=0.18)
    ap.add_argument("--sway", type=float, default=0.30)
    ap.add_argument("--trigger-m", type=float, default=1.30)
    ap.add_argument("--clear-bearing", type=float, default=32.0)
    ap.add_argument("--pass-s", type=float, default=3.0)
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--margin", type=float, default=0.03)
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--hold-s", type=float, default=12.0)
    ap.add_argument("--shadow", action="store_true",
                    help="SHADOW MODE: compute everything, command nothing")
    args = ap.parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, ("shadow_" if args.shadow else "run_") + stamp)
    os.makedirs(sess, exist_ok=True)

    surge = int(max(0.0, min(0.30, args.surge)) * 1000)
    sway = int(max(0.0, min(0.40, args.sway)) * 1000)

    cam = CameraStream()
    tracker = OverheadTracker()
    log = []
    print("camera + overhead ready", flush=True)
    try:
        ov = tracker.detect()
        near = ov.get("pixel") if ov.get("found") else None
        print("overhead start:", near, ov.get("xyz_camera_m"), flush=True)

        with RovLink() as rov:
            rov.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
            if not args.shadow:
                rov.set_mode(args.mode)
                hb = rov.arm(mode=args.mode)
                print("armed:", hb, flush=True)
                if not hb or not hb["armed"]:
                    print("ARMING FAILED — aborting")
                    return 1
            att = rov.recv_match("ATTITUDE", timeout=2)
            yaw_target = att.yaw if att else 0.0
            state = "APPROACH"
            side = 0
            t_clear = None
            t0 = time.time()
            last_ov = 0.0
            aborted = None
            try:
                while time.time() - t0 < args.timeout:
                    fr, _ = cam.latest()
                    det = detect_anchor(fr)
                    ok = accept(det)
                    rng = range_from_width(det) if ok else None
                    brg = bearing_deg(det) if ok else None

                    att = rov.recv_match("ATTITUDE", timeout=0.15)
                    yaw = att.yaw if att else yaw_target
                    yrate = getattr(att, "yawspeed", 0.0) if att else 0.0
                    yerr = wrap(yaw_target - yaw)
                    r = 6.0 * math.degrees(yerr) - 2.0 * math.degrees(yrate)
                    if abs(math.degrees(yerr)) < 1.5 and abs(yrate) < 0.03:
                        r = 0.0
                    r = int(max(-200, min(200, r)))

                    x, y = surge, 0
                    if state == "APPROACH":
                        if rng is not None and rng <= args.trigger_m:
                            # commit to the side AWAY from the obstacle
                            side = -1 if (brg or 0) >= 0 else +1
                            state = "AVOID"
                            print(f"== TRIGGER at {rng:.2f} m, bearing "
                                  f"{brg:+.0f} deg -> avoid to the "
                                  f"{'right' if side > 0 else 'left'}",
                                  flush=True)
                    if state == "AVOID":
                        x = int(surge * 0.6)
                        y = side * sway
                        if (not ok) or (brg is not None
                                        and abs(brg) >= args.clear_bearing):
                            state = "CLEAR"
                            t_clear = time.time()
                            print("== CLEAR (anchor passed/lost)",
                                  flush=True)
                    if state == "CLEAR":
                        x = int(surge * 0.6)
                        y = side * sway
                        if time.time() - t_clear > args.pass_s:
                            print("== PASS COMPLETE", flush=True)
                            break

                    if not args.shadow:
                        rov.manual(x=x, y=y, r=r)

                    rec = {"t": round(time.time() - t0, 2), "state": state,
                           "x": x, "y": y, "r": r,
                           "det": {k: det.get(k) for k in
                                   ("found", "score", "center_x",
                                    "height", "width")},
                           "accepted": ok,
                           "range_m": None if rng is None else round(rng, 2),
                           "bearing_deg": (None if brg is None
                                           else round(brg, 1)),
                           "yaw_deg": round(math.degrees(yaw), 1)}
                    log.append(rec)
                    if len(log) % 8 == 1:
                        print(f"  {state:8s} rng={rec['range_m']} "
                              f"brg={rec['bearing_deg']} x={x} y={y} r={r}",
                              flush=True)
                        cv2.imwrite(os.path.join(
                            sess, f"f{len(log):04d}.png"),
                            annotate(fr, det))

                    if time.time() - last_ov > 0.7:
                        last_ov = time.time()
                        ovd = tracker.detect(near=near)
                        if ovd and ovd.get("found"):
                            near = ovd["pixel"]
                            rec["overhead_px"] = near
                            fx, fy = ovd["frac_x"], ovd["frac_y"]
                            m = args.margin
                            if not (m < fx < 1 - m and m < fy < 1 - m):
                                aborted = "overhead safe box"
                                print("!! ABORT: safe box", round(fx, 2),
                                      round(fy, 2), flush=True)
                                break
                    time.sleep(0.1)
            finally:
                if not args.shadow:
                    print("active hold...", flush=True)
                    rov.hold_heading(args.hold_s, target_yaw=yaw_target)
                    print("hold done; disarm:", rov.disarm(), flush=True)
        fr, _ = cam.latest()
        det = detect_anchor(fr)
        cv2.imwrite(os.path.join(sess, "final.png"), annotate(fr, det))
        ovf = tracker.detect(near=near,
                             save_path=os.path.join(sess, "overhead_end.png"))
        summary = {"aborted": aborted, "final_state": state,
                   "side": side, "samples": len(log),
                   "overhead_end_px": ovf.get("pixel"),
                   "overhead_end_xyz": ovf.get("xyz_camera_m"),
                   "params": vars(args)}
        with open(os.path.join(sess, "log.json"), "w") as f:
            json.dump({"summary": summary, "log": log}, f, indent=2,
                      default=str)
        print(json.dumps(summary, indent=1, default=str), flush=True)
        print("session:", sess, flush=True)
    finally:
        cam.close()
        tracker.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
