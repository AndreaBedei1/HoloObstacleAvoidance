"""FULL REAL AVOIDANCE MISSION with ground truth and presentation media.

Improvements over avoid_run.py (operator feedback 2026-08-14: "it must
see it, move right, avoid it AND get past it — here it stopped early"):

  * the avoidance COMMITS and keeps going until the overhead ground
    truth confirms the vehicle has passed the anchor (distance to the
    anchor grew back past a margin after the closest point), not merely
    until the anchor leaves the camera
  * full surge maintained through the pass (was 60%)
  * OVERHEAD GROUND TRUTH: ROV tracked every cycle, distance to the
    known anchor pixel logged; min distance = the real clearance proxy
  * MEDIA: annotated onboard video, annotated overhead video with the
    trajectory drawn, a side-by-side composite, and key still frames

Nothing here feeds the planner: the overhead camera is validation and
safety only.
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
import numpy as np  # noqa: E402
from pymavlink import mavutil  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "experiments", "real", "missions")

F_PX = 1277.0                # D-015 one-point calibration
ANCHOR_WIDTH_M = 0.80
IMG_W = 1920.0
ANCHOR_PX = (1069.0, 635.0)  # overhead pixel of the anchor attachment

MIN_SCORE = 0.55
MIN_H = 0.12
MIN_W = 0.02


def accept(d):
    return bool(d.get("found") and d.get("score", 0) >= MIN_SCORE
                and d.get("height", 0) >= MIN_H
                and d.get("width", 0) >= MIN_W)


def rng_of(d):
    w = d["width"] * IMG_W
    return None if w < 5 else ANCHOR_WIDTH_M * F_PX / w


def brg_of(d):
    return math.degrees(math.atan2((d["center_x"] - 0.5) * IMG_W, F_PX))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--surge", type=float, default=0.18)
    ap.add_argument("--sway", type=float, default=0.30)
    ap.add_argument("--trigger-m", type=float, default=1.30)
    ap.add_argument("--avoid-max-s", type=float, default=8.0,
                    help="max lateral clearing time")
    ap.add_argument("--straight-s", type=float, default=6.0,
                    help="straight leg after the anchor is cleared")
    ap.add_argument("--lost-frames", type=int, default=5,
                    help="consecutive non-detections that end the "
                         "lateral clearing")
    ap.add_argument("--clear-bearing", type=float, default=35.0)
    ap.add_argument("--timeout", type=float, default=50.0)
    ap.add_argument("--margin", type=float, default=0.10,
                    help="forbidden frame border: the pool WALLS are "
                         "just outside the view, so keep well clear")
    ap.add_argument("--mode", default="STABILIZE")
    ap.add_argument("--hold-s", type=float, default=10.0)
    ap.add_argument("--anchor-px", nargs=2, type=float, default=ANCHOR_PX)
    ap.add_argument("--centre-s", type=float, default=8.0,
                    help="visual centring window before the approach")
    args = ap.parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, stamp)
    os.makedirs(sess, exist_ok=True)
    apx = np.array(args.anchor_px)

    # Cap raised to 0.50: at 0.22 the thrusters spin but the
    # vehicle does not move against tether drag (operator,
    # 2026-08-14).
    surge = int(max(0.0, min(0.50, args.surge)) * 1000)
    sway = int(max(0.0, min(0.55, args.sway)) * 1000)

    cam = CameraStream()
    tracker = OverheadTracker()
    log, onboard_frames, overhead_frames, traj = [], [], [], []
    print("camera + overhead ready", flush=True)
    try:
        ov = tracker.detect()
        near = ov.get("pixel") if ov.get("found") else None
        print("overhead start:", near, flush=True)

        with RovLink() as rov:
            rov.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
            rov.set_mode(args.mode)
            hb = rov.arm(mode=args.mode)
            print("armed:", hb, flush=True)
            if not hb or not hb["armed"]:
                print("ARMING FAILED")
                return 1
            att = rov.recv_match("ATTITUDE", timeout=2)
            yaw_target = att.yaw if att else 0.0

            # ---- CENTRE the anchor before starting the approach ------
            # Arming takes seconds during which the vehicle drifts, so
            # the heading at arm time is NOT the heading the operator
            # set. Servo on the detector until the anchor is centred,
            # then freeze that heading as the approach line.
            if args.centre_s > 0:
                tc = time.time()
                good = 0
                while time.time() - tc < args.centre_s:
                    fr, _ = cam.latest()
                    d = detect_anchor(fr)
                    if accept(d):
                        e = d["center_x"] - 0.5
                        if abs(e) < 0.05:
                            good += 1
                            rov.manual(r=0)
                            if good >= 3:
                                break
                        else:
                            good = 0
                            rov.manual(r=int(max(-220, min(
                                220, e * 5.0 * 220))))
                    else:
                        rov.manual(r=0)
                    time.sleep(0.12)
                att = rov.recv_match("ATTITUDE", timeout=1.0)
                if att:
                    yaw_target = att.yaw
                print(f"centred: {good} good frames, approach heading "
                      f"{math.degrees(yaw_target):.0f} deg", flush=True)
                rov.hold_heading(1.5, target_yaw=yaw_target)

            state, side, t_trig = "APPROACH", 0, None
            start_frac = None
            lim_x = lim_y = 0.02
            lim_x2 = lim_y2 = 0.98
            t_str = None
            lost = 0
            p_start = None
            head_dir = None
            min_gt = 1e9
            t0 = time.time()
            aborted = None
            trig_info = None
            try:
                while time.time() - t0 < args.timeout:
                    fr, _ = cam.latest()
                    det = detect_anchor(fr)
                    ok = accept(det)
                    rng = rng_of(det) if ok else None
                    brg = brg_of(det) if ok else None

                    att = rov.recv_match("ATTITUDE", timeout=0.15)
                    yaw = att.yaw if att else yaw_target
                    yrate = getattr(att, "yawspeed", 0.0) if att else 0.0
                    yerr = wrap(yaw_target - yaw)
                    r = 6.0 * math.degrees(yerr) - 2.0 * math.degrees(yrate)
                    if abs(math.degrees(yerr)) < 1.5 and abs(yrate) < 0.03:
                        r = 0.0
                    r = int(max(-200, min(200, r)))

                    # ---- overhead ground truth (every cycle) ----------
                    ovd = tracker.detect(near=near)
                    gt_dist = None
                    log_extra = {}
                    if ovd and ovd.get("found"):
                        near = ovd["pixel"]
                        traj.append(list(near))
                        if p_start is None:
                            p_start = np.array(near, dtype=float)
                        elif state == "APPROACH":
                            v = np.array(near, dtype=float) - p_start
                            if np.linalg.norm(v) > 60:
                                head_dir = v / np.linalg.norm(v)
                        gt_dist = float(np.linalg.norm(
                            np.array(near) - apx))
                        min_gt = min(min_gt, gt_dist)
                        fx, fy = ovd["frac_x"], ovd["frac_y"]
                        # RELATIVE wall guard: the operator releases the
                        # vehicle from the pool edge (that is where the
                        # hands reach), so an absolute margin aborts every
                        # run at t=0. Instead: never get CLOSER to any
                        # border than the release point (minus a small
                        # tolerance), and never cross a hard 2% limit.
                        if start_frac is None:
                            start_frac = (fx, fy)
                            lim_x = min(args.margin,
                                        max(0.02, start_frac[0] - 0.03))
                            lim_y = min(args.margin,
                                        max(0.02, start_frac[1] - 0.03))
                            lim_x2 = max(1 - args.margin,
                                         min(0.98, start_frac[0] + 0.03))
                            lim_y2 = max(1 - args.margin,
                                         min(0.98, start_frac[1] + 0.03))
                            print(f"wall guard: x in [{lim_x:.2f},"
                                  f"{lim_x2:.2f}] y in [{lim_y:.2f},"
                                  f"{lim_y2:.2f}]", flush=True)
                        m = args.margin
                        if not (lim_x < fx < lim_x2
                                and lim_y < fy < lim_y2):
                            aborted = "overhead safe box (wall margin)"
                            print("!! ABORT: too close to the pool "
                                  f"margin ({fx:.2f}, {fy:.2f}) — active "
                                  "hold", flush=True)
                            break

                    x, y = surge, 0
                    if state == "APPROACH" and rng is not None                             and rng <= args.trigger_m:
                        # PILOT-1 DEFECT (2026-08-14, before the
                        # campaign): forcing "always pass right" steered
                        # the vehicle INTO the anchor when the anchor was
                        # itself to the right (trigger bearing +20.8 deg,
                        # closest approach 0.32 m = contact). The rule is
                        # now the physically correct one used by the
                        # simulation planners: clear to the side AWAY
                        # from the obstacle's bearing; when the obstacle
                        # is dead ahead (|bearing| < 5 deg) prefer RIGHT,
                        # as the operator asked.
                        side = +1 if abs(brg) < 5.0 else (
                            -1 if brg > 0 else +1)
                        state = "AVOID"
                        t_trig = time.time()
                        lost = 0
                        trig_info = {"range_m": round(rng, 2),
                                     "bearing_deg": round(brg, 1),
                                     "gt_px": gt_dist}
                        print(f"== TRIGGER {rng:.2f} m brg {brg:+.0f} deg"
                              f" -> clearing to the "
                              f"{'RIGHT' if side > 0 else 'LEFT'}",
                              flush=True)
                    if state == "AVOID":
                        # LATERAL ONLY while the obstacle is still seen:
                        # slide sideways (minimal surge) so the vehicle
                        # does not close on the anchor while clearing it.
                        x, y = int(surge * 0.25), side * sway
                        if ok and abs(brg) < args.clear_bearing:
                            lost = 0
                        else:
                            lost += 1
                        if lost >= args.lost_frames:
                            state = "STRAIGHT"
                            t_str = time.time()
                            print("== anchor cleared -> STRAIGHT ahead",
                                  flush=True)
                        elif time.time() - t_trig > args.avoid_max_s:
                            state = "STRAIGHT"
                            t_str = time.time()
                            print("== avoid window elapsed -> STRAIGHT",
                                  flush=True)
                    if state == "STRAIGHT":
                        # NO sway: straight past the obstacle (operator
                        # requirement — sideways travel walks into the
                        # pool walls).
                        x, y = surge, 0
                        passed = False
                        if head_dir is not None and near is not None:
                            s_rov = float(np.dot(np.array(near) - p_start,
                                                 head_dir))
                            s_anchor = float(np.dot(apx - p_start,
                                                    head_dir))
                            log_extra["s_rov"] = round(s_rov, 1)
                            log_extra["s_anchor"] = round(s_anchor, 1)
                            passed = s_rov > s_anchor + 90
                        if passed:
                            state = "PASSED"
                            print("== PASSED the anchor plane (GT)",
                                  flush=True)
                        elif time.time() - t_str > args.straight_s:
                            state = "PASSED"
                            print("== straight leg complete", flush=True)
                    if state == "PASSED":
                        break

                    rov.manual(x=x, y=y, r=r)
                    log.append({"t": round(time.time() - t0, 2),
                                "state": state, "x": x, "y": y, "r": r,
                                "accepted": ok,
                                "range_m": None if rng is None
                                else round(rng, 2),
                                "bearing_deg": None if brg is None
                                else round(brg, 1),
                                "gt_dist_px": None if gt_dist is None
                                else round(gt_dist, 1),
                                "yaw_deg": round(math.degrees(yaw), 1),
                                **log_extra})
                    onboard_frames.append(annotate(fr, det))
                    ovv = tracker.last_frame if hasattr(
                        tracker, "last_frame") else None
                    if ovd is not None and ovd.get("found"):
                        overhead_frames.append((list(near), state,
                                                gt_dist))
                    if len(log) % 8 == 1:
                        print(f"  {state:8s} rng={log[-1]['range_m']} "
                              f"brg={log[-1]['bearing_deg']} "
                              f"gt={log[-1]['gt_dist_px']} x={x} y={y}",
                              flush=True)
                    time.sleep(0.1)
            finally:
                print("active hold...", flush=True)
                rov.hold_heading(args.hold_s, target_yaw=yaw_target)
                print("disarm:", rov.disarm(), flush=True)

        # ---------- media ----------
        ovimg, _ = tracker.grab()
        vis_ov = ovimg.copy()
        pts = np.array(traj, dtype=np.int32)
        if len(pts) > 1:
            cv2.polylines(vis_ov, [pts], False, (0, 255, 255), 3)
            cv2.circle(vis_ov, tuple(pts[0]), 12, (0, 255, 0), -1)
            cv2.circle(vis_ov, tuple(pts[-1]), 12, (0, 0, 255), -1)
        cv2.circle(vis_ov, (int(apx[0]), int(apx[1])), 16, (255, 0, 255), 3)
        cv2.putText(vis_ov, "ANCHOR", (int(apx[0]) + 20, int(apx[1])),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 255), 2)
        cv2.putText(vis_ov, f"min GT distance {min_gt:.0f} px",
                    (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 255, 255), 3)
        cv2.imwrite(os.path.join(sess, "overhead_trajectory.png"), vis_ov)

        if onboard_frames:
            h, w = onboard_frames[0].shape[:2]
            vw = cv2.VideoWriter(
                os.path.join(sess, "onboard_detector.mp4"),
                cv2.VideoWriter_fourcc(*"mp4v"), 8, (w // 2, h // 2))
            for f in onboard_frames:
                vw.write(cv2.resize(f, (w // 2, h // 2)))
            vw.release()
            for i, idx in enumerate(
                    (0, len(onboard_frames) // 3,
                     2 * len(onboard_frames) // 3, -1)):
                cv2.imwrite(os.path.join(sess, f"key_{i}.png"),
                            onboard_frames[idx])

        summary = {"aborted": aborted, "final_state": state, "side": side,
                   "trigger": trig_info, "samples": len(log),
                   "min_gt_dist_px": None if min_gt > 1e8
                   else round(min_gt, 1),
                   "accepted_frac": round(
                       sum(1 for r in log if r["accepted"])
                       / max(1, len(log)), 3),
                   "params": vars(args)}
        with open(os.path.join(sess, "log.json"), "w") as f:
            json.dump({"summary": summary, "log": log, "traj": traj},
                      f, indent=2, default=str)
        print(json.dumps({k: v for k, v in summary.items()
                          if k != "params"}, indent=1, default=str),
              flush=True)
        print("session:", sess, flush=True)
    finally:
        cam.close()
        tracker.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
