"""Minimal adaptive actuator identification with overhead ground truth.

Implements docs/ACTUATOR_ID_PROTOCOL.md: NOT a fixed grid. For each
signed axis it climbs a short ladder until the vehicle demonstrably
moves, confirms the bracket with a repeat, then takes one operating
point above it. Trials stop on geometry and safety, never on a fixed
duration.

The speed comes from the OVERHEAD ground truth (pixels/second converted
with the frozen pool scale), because the vehicle has no DVL and the
commanded velocity is exactly the unknown being measured.

Safety, checked every cycle:
  * displacement from the trial start          -> stop
  * distance to the frame border (proxy wall)  -> stop
  * overhead track lost > 1 s                  -> stop
  * roll/pitch beyond limits                   -> stop
  * hard duration cap                          -> stop
Every attempted trial is recorded, including aborts and trials where the
vehicle did not move: a non-moving trial is the primary evidence for the
deadband.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)

import numpy as np                                    # noqa: E402

from overhead_track import OverheadTracker            # noqa: E402
from rovlink import RovLink, Z_NEUTRAL                # noqa: E402

OUT = os.path.join(ROOT, "experiments", "real", "actuator_id")

AXES = {"surge+": ("x", +1), "surge-": ("x", -1),
        "sway+": ("y", +1), "sway-": ("y", -1),
        "yaw+": ("r", +1), "yaw-": ("r", -1)}

MOVE_THRESHOLD_M = 0.06        # displacement that counts as "it moved"
MOVE_THRESHOLD_DEG = 6.0       # for yaw


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def run_trial(rov, tracker, axis, level, scale, max_s=5.0,
              max_disp_m=0.6, margin=0.06):
    """One bounded pulse. Returns the measurement and why it ended."""
    chan, sign = AXES[axis]
    mag = int(max(0.0, min(0.5, level)) * 1000)
    kw = {"x": 0, "y": 0, "r": 0}
    kw[chan] = sign * mag

    d0 = tracker.detect()
    if not d0.get("found"):
        return {"axis": axis, "level": level, "ended": "no_ground_truth",
                "moved": None}
    p0 = np.array(d0["pixel"], dtype=float)
    att = rov.newest("ATTITUDE", timeout=1.0)
    yaw0 = att.yaw if att else None

    track, ended = [], "duration"
    t0 = time.time()
    last_seen = t0
    while time.time() - t0 < max_s:
        rov.manual(z=Z_NEUTRAL, **kw)
        a = rov.newest("ATTITUDE", timeout=0.12)
        if a is not None and (abs(a.roll) > 0.26 or abs(a.pitch) > 0.26):
            ended = "attitude_limit"
            break
        d = tracker.detect()
        now = time.time()
        if d.get("found"):
            last_seen = now
            p = np.array(d["pixel"], dtype=float)
            track.append({"t": round(now - t0, 3), "px": list(p),
                          "yaw": None if a is None else a.yaw})
            disp = float(np.linalg.norm(p - p0)) / scale
            if disp > max_disp_m:
                ended = "max_displacement"
                break
            fx, fy = d["frac_x"], d["frac_y"]
            if not (margin < fx < 1 - margin and margin < fy < 1 - margin):
                ended = "frame_border"
                break
        elif now - last_seen > 1.0:
            ended = "track_lost"
            break
        time.sleep(0.08)

    rov.neutral()
    # coast: how far does it keep going after the command stops?
    t1 = time.time()
    coast = []
    while time.time() - t1 < 2.5:
        rov.neutral()
        d = tracker.detect()
        if d.get("found"):
            coast.append({"t": round(time.time() - t1, 3),
                          "px": list(d["pixel"])})
        time.sleep(0.1)

    out = {"axis": axis, "level": level, "counts": sign * mag,
           "ended": ended, "duration_s": round(time.time() - t0, 2),
           "track": track, "coast": coast}
    if len(track) >= 3:
        p_end = np.array(track[-1]["px"], dtype=float)
        disp_m = float(np.linalg.norm(p_end - p0)) / scale
        dt = track[-1]["t"] - track[0]["t"]
        out["displacement_m"] = round(disp_m, 4)
        out["mean_speed_ms"] = round(disp_m / dt, 4) if dt > 0.5 else None
        out["direction_px"] = [round(float(v), 1) for v in (p_end - p0)]
        if yaw0 is not None and track[-1]["yaw"] is not None:
            turned = abs(math.degrees(wrap(track[-1]["yaw"] - yaw0)))
            out["turned_deg"] = round(turned, 1)
            out["yaw_rate_deg_s"] = (round(turned / dt, 2)
                                     if dt > 0.5 else None)
        if axis.startswith("yaw"):
            out["moved"] = bool(out.get("turned_deg", 0)
                                >= MOVE_THRESHOLD_DEG)
        else:
            out["moved"] = bool(disp_m >= MOVE_THRESHOLD_M)
    else:
        out["moved"] = None
    if len(coast) >= 3:
        c0 = np.array(coast[0]["px"], dtype=float)
        c1 = np.array(coast[-1]["px"], dtype=float)
        out["coast_m"] = round(float(np.linalg.norm(c1 - c0)) / scale, 4)
    return out


def learn_image_offset(rov, tracker, level=0.30, dur=2.5):
    """Learn the angle between 'body forward' and the image +x axis.

    Needed to steer the vehicle BACK to the centre between trials. The
    vehicle is not trimmed and drifts backwards continuously (operator,
    2026-08-15), so without an active return every trial starts further
    out until the vehicle leaves the camera view and nothing can be
    measured at all.

    One short surge pulse, the displacement direction in the image, and
    the compass yaw at that moment give the offset; from then on any
    desired image direction can be converted into body commands.
    """
    d0 = tracker.detect()
    if not d0.get("found"):
        return None
    a0 = rov.newest("ATTITUDE", timeout=1.5)
    if a0 is None:
        return None
    p0 = np.array(d0["pixel"], dtype=float)
    mag = int(level * 1000)
    t0 = time.time()
    while time.time() - t0 < dur:
        rov.manual(x=mag, z=Z_NEUTRAL)
        time.sleep(0.08)
    rov.neutral()
    time.sleep(1.0)
    d1 = tracker.detect()
    if not d1.get("found"):
        return None
    p1 = np.array(d1["pixel"], dtype=float)
    v = p1 - p0
    if np.linalg.norm(v) < 20:
        return None
    img_dir = math.atan2(v[1], v[0])
    return {"offset_rad": wrap(img_dir - a0.yaw),
            "probe_px": float(np.linalg.norm(v))}


def recenter(rov, tracker, target_px, cal, scale, max_s=14.0,
             tol_px=90.0, level=0.28):
    """Drive the vehicle back toward `target_px` in closed loop."""
    if cal is None:
        return "no_calibration"
    t0 = time.time()
    while time.time() - t0 < max_s:
        d = tracker.detect()
        if not d.get("found"):
            rov.neutral()
            time.sleep(0.15)
            continue
        p = np.array(d["pixel"], dtype=float)
        e = np.array(target_px, dtype=float) - p
        if np.linalg.norm(e) < tol_px:
            rov.neutral()
            return "centred"
        a = rov.newest("ATTITUDE", timeout=0.2)
        yaw = a.yaw if a is not None else 0.0
        ang = yaw + cal["offset_rad"]
        fwd = e[0] * math.cos(ang) + e[1] * math.sin(ang)
        rgt = -e[0] * math.sin(ang) + e[1] * math.cos(ang)
        n = max(1.0, math.hypot(fwd, rgt))
        mag = int(level * 1000)
        rov.manual(x=int(mag * fwd / n), y=int(mag * rgt / n),
                   z=Z_NEUTRAL)
        time.sleep(0.1)
    rov.neutral()
    return "timeout"


def undo(rov, axis, level, duration_s):
    """Return the vehicle roughly to where the trial started.

    The vehicle is not perfectly trimmed and drifts backwards when left
    alone (operator, 2026-08-15), so consecutive trials accumulate
    displacement and walk it out of the camera view. An equal-and-
    opposite pulse cancels most of the trial's own displacement without
    needing the command->speed mapping that these very trials are
    measuring.
    """
    if duration_s <= 0.2:
        return
    chan, sign = AXES[axis]
    mag = int(max(0.0, min(0.5, level)) * 1000)
    kw = {"x": 0, "y": 0, "r": 0}
    kw[chan] = -sign * mag
    t0 = time.time()
    while time.time() - t0 < min(duration_s, 4.0):
        rov.manual(z=Z_NEUTRAL, **kw)
        time.sleep(0.08)
    rov.neutral()
    time.sleep(0.5)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--axes", default="sway+,sway-,surge+,surge-,"
                                      "yaw+,yaw-")
    ap.add_argument("--ladder", default="0.15,0.25,0.35")
    ap.add_argument("--pulse-s", type=float, default=3.0)
    ap.add_argument("--max-disp-m", type=float, default=0.55)
    ap.add_argument("--mode", default="ALT_HOLD")
    ap.add_argument("--pool-frame",
                    default=os.path.join(ROOT, "config", "real_pool",
                                         "pool_frame_FROZEN.json"))
    args = ap.parse_args()
    with open(args.pool_frame) as f:
        scale = json.load(f)["scale"]["px_per_m_rod"]
    levels = [float(x) for x in args.ladder.split(",")]
    axes = [a.strip() for a in args.axes.split(",") if a.strip()]
    os.makedirs(OUT, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    tracker = OverheadTracker()
    trials = []
    try:
        with RovLink() as rov:
            from pymavlink import mavutil
            rov.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
            rov.set_mode(args.mode)
            hb = rov.arm(mode=args.mode)
            print("armed:", hb, flush=True)
            if not hb or not hb["armed"]:
                print("ARMING FAILED")
                return 1
            d_start = tracker.detect()
            home = (list(d_start["pixel"]) if d_start.get("found")
                    else [960.0, 540.0])
            print("home pixel:", [round(v) for v in home], flush=True)
            cal = learn_image_offset(rov, tracker)
            print("image/compass offset:",
                  None if cal is None else
                  round(math.degrees(cal["offset_rad"])), "deg", flush=True)
            recenter(rov, tracker, home, cal, scale)

            for axis in axes:
                moved_at = None
                for lv in levels:
                    t = run_trial(rov, tracker, axis, lv, scale,
                                  max_s=args.pulse_s,
                                  max_disp_m=args.max_disp_m)
                    trials.append(t)
                    print(f"  {axis} @{lv:.2f}: moved={t.get('moved')} "
                          f"disp={t.get('displacement_m')} m "
                          f"speed={t.get('mean_speed_ms')} m/s "
                          f"turned={t.get('turned_deg')} deg "
                          f"end={t['ended']}", flush=True)
                    undo(rov, axis, lv, t.get("duration_s", 0.0))
                    st = recenter(rov, tracker, home, cal, scale)
                    print(f"     recentre: {st}", flush=True)
                    if t.get("moved"):
                        moved_at = lv
                        break
                if moved_at is not None:
                    # one repeat at the moving level: repeatability, and
                    # the second point the slope needs
                    t = run_trial(rov, tracker, axis, moved_at, scale,
                                  max_s=args.pulse_s,
                                  max_disp_m=args.max_disp_m)
                    t["repeat"] = True
                    trials.append(t)
                    print(f"  {axis} @{moved_at:.2f} REPEAT: "
                          f"disp={t.get('displacement_m')} "
                          f"speed={t.get('mean_speed_ms')} "
                          f"turned={t.get('turned_deg')}", flush=True)
                    undo(rov, axis, moved_at, t.get("duration_s", 0.0))
                    st = recenter(rov, tracker, home, cal, scale)
                    print(f"     recentre: {st}", flush=True)
            rov.hold_heading(4.0)
            print("disarm:", rov.disarm(), flush=True)
    finally:
        tracker.close()
        out = os.path.join(OUT, f"actuator_id_{stamp}.json")
        with open(out, "w") as f:
            json.dump({"meta": {"stamp": stamp, "scale_px_per_m": scale,
                                "ladder": levels, "pulse_s": args.pulse_s,
                                "note": "CALIBRATION data, never final "
                                        "validation"},
                       "trials": trials}, f, indent=2)
        print("->", out, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
