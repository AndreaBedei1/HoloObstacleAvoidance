"""Continuous station keeping for the real BlueROV2 (Phase 9).

The vehicle NEVER holds still on its own: with the thrusters idle it
keeps coasting on its own momentum and on tether pull, and a disarmed
vehicle has no control at all (operator observations 2026-08-14). This
daemon therefore keeps it ARMED and CLOSED-LOOP indefinitely:

  * heading: P+D on the compass (nulls both error and residual rate)
  * position: P+D on the OVERHEAD camera pixel position, rotated into
    the body frame using the compass yaw plus a measured image/compass
    offset (calibrated once by a short surge probe, then cached)
  * lost overhead detection -> heading-only hold (fail-safe degradation)
  * never disarms while running; stop it with TaskStop / Ctrl-C, after
    which ArduSub's pilot-input failsafe disarms within 3 s

The overhead camera is an OPERATIONAL aid here (station keeping between
experiments). It is NEVER an input to the obstacle-avoidance planner:
the scientific runs use only onboard perception.

Usage:
  python scripts/real/station_keep.py                 # hold where it is
  python scripts/real/station_keep.py --target-frac 0.45 0.5
  python scripts/real/station_keep.py --recalibrate
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from overhead_track import OverheadTracker  # noqa: E402
from rovlink import RovLink  # noqa: E402

from pymavlink import mavutil  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CAL_PATH = os.path.join(HERE, "image_heading_offset.json")
LOG_DIR = os.path.join(HERE, "..", "..", "experiments", "real",
                       "station_keep")

# Control gains (pixels -> MANUAL_CONTROL units). Deliberately soft: the
# vehicle is slow to respond and the pool is small.
KP_POS = 0.55
KD_POS = 1.8
MAX_TRANS = 220           # 22% authority
KP_YAW = 6.0
KD_YAW = 2.0
MAX_YAW = 220
DEADBAND_PX = 35.0
LOST_TIMEOUT = 3.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def calibrate_offset(rov, tracker, near=None, power=0.25, dur=2.5):
    """Measure the angle between 'body forward' and the image +x axis."""
    d0 = tracker.detect(near=near)
    if not d0.get("found"):
        return None
    att = rov.recv_match("ATTITUDE", timeout=2)
    if att is None:
        return None
    yaw0 = att.yaw
    p0 = d0["pixel"]
    mag = int(power * 1000)
    t0 = time.time()
    while time.time() - t0 < dur:
        rov.manual(x=mag)
        time.sleep(0.1)
    rov.neutral()
    time.sleep(1.2)
    d1 = tracker.detect(near=p0)
    if not d1.get("found"):
        return None
    p1 = d1["pixel"]
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    dist = math.hypot(dx, dy)
    if dist < 25:
        return None
    img_dir = math.atan2(dy, dx)
    offset = wrap(img_dir - yaw0)
    return {"offset_rad": offset, "probe_px": dist,
            "yaw_at_probe_deg": math.degrees(yaw0),
            "img_dir_deg": math.degrees(img_dir)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-frac", nargs=2, type=float, default=None,
                    help="target position as frame fractions (x y)")
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--recalibrate", action="store_true")
    ap.add_argument("--max-seconds", type=float, default=3600.0)
    args = ap.parse_args()
    os.makedirs(LOG_DIR, exist_ok=True)
    logf = open(os.path.join(
        LOG_DIR, time.strftime("station_%Y%m%d_%H%M%S.jsonl")), "w")

    tracker = OverheadTracker()
    try:
        with RovLink() as rov:
            rov.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
            rov.set_mode(args.mode)
            hb = rov.arm(mode=args.mode)
            print("armed:", hb, flush=True)
            if not hb or not hb["armed"]:
                return 1

            det = tracker.detect()
            near = det.get("pixel") if det.get("found") else None
            print("start pixel:", near, flush=True)

            cal = None
            if os.path.isfile(CAL_PATH) and not args.recalibrate:
                cal = json.load(open(CAL_PATH))
                print("using cached image/compass offset "
                      f"{math.degrees(cal['offset_rad']):.0f} deg",
                      flush=True)
            if cal is None:
                print("calibrating image/compass offset...", flush=True)
                cal = calibrate_offset(rov, tracker, near=near)
                if cal:
                    json.dump(cal, open(CAL_PATH, "w"), indent=2)
                    print("offset "
                          f"{math.degrees(cal['offset_rad']):.0f} deg "
                          f"(probe {cal['probe_px']:.0f} px)", flush=True)
                else:
                    print("calibration failed -> heading-only hold",
                          flush=True)
                det = tracker.detect(near=near)
                if det.get("found"):
                    near = det["pixel"]

            img = None
            target = None
            if args.target_frac and tracker.intr is not None:
                target = [args.target_frac[0] * 1920,
                          args.target_frac[1] * 1080]
            elif near is not None:
                target = list(near)
            att = rov.recv_match("ATTITUDE", timeout=2)
            yaw_target = att.yaw if att else 0.0
            print("target px:", target, "| yaw target deg:",
                  round(math.degrees(yaw_target), 1), flush=True)

            t_start = time.time()
            last_det_t = time.time()
            prev_err = None
            prev_t = None
            while time.time() - t_start < args.max_seconds:
                att = rov.recv_match("ATTITUDE", timeout=0.25)
                if att is not None:
                    yaw = att.yaw
                    yaw_rate = getattr(att, "yawspeed", 0.0)
                else:
                    yaw = yaw_target
                    yaw_rate = 0.0
                yerr = wrap(yaw_target - yaw)
                rcmd = KP_YAW * math.degrees(yerr) - \
                    KD_YAW * math.degrees(yaw_rate)
                if abs(math.degrees(yerr)) < 1.5 and abs(yaw_rate) < 0.03:
                    rcmd = 0.0
                rcmd = int(max(-MAX_YAW, min(MAX_YAW, rcmd)))

                x = y = 0
                det = tracker.detect(near=near)
                now = time.time()
                if det and det.get("found"):
                    near = det["pixel"]
                    last_det_t = now
                    if target is not None and cal is not None:
                        ex = target[0] - near[0]
                        ey = target[1] - near[1]
                        dist = math.hypot(ex, ey)
                        # rotate image error into the body frame
                        ang = yaw + cal["offset_rad"]
                        fwd = ex * math.cos(ang) + ey * math.sin(ang)
                        rgt = -ex * math.sin(ang) + ey * math.cos(ang)
                        dfwd = dgt = 0.0
                        if prev_err is not None and prev_t is not None:
                            dt = max(1e-2, now - prev_t)
                            dfwd = (fwd - prev_err[0]) / dt
                            dgt = (rgt - prev_err[1]) / dt
                        prev_err = (fwd, rgt)
                        prev_t = now
                        if dist > DEADBAND_PX:
                            x = int(max(-MAX_TRANS, min(
                                MAX_TRANS, KP_POS * fwd + KD_POS * dfwd)))
                            y = int(max(-MAX_TRANS, min(
                                MAX_TRANS, KP_POS * rgt + KD_POS * dgt)))
                elif now - last_det_t > LOST_TIMEOUT:
                    x = y = 0     # heading-only fail-safe
                    prev_err = None

                rov.manual(x=x, y=y, r=rcmd)
                logf.write(json.dumps({
                    "t": round(now - t_start, 2),
                    "px": near, "x": x, "y": y, "r": rcmd,
                    "yaw_err_deg": round(math.degrees(yerr), 1),
                    "found": bool(det and det.get("found")),
                }) + "\n")
                logf.flush()
                time.sleep(0.08)
    finally:
        tracker.close()
        logf.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
