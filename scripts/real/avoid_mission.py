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
from gt_guard import PoseGuard  # noqa: E402
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


def overhead_scale(default=402.0):
    """Pixel/metre scale for the overhead camera.

    Prefers the value measured by scripts/real/pool_remap.py after the
    last camera move; falls back to the 2026-08-13 survey number, which
    is only valid while the camera has not been touched."""
    try:
        from pool_remap import latest_pool_frame
        found = latest_pool_frame()
        if found:
            path, doc = found
            scale = doc.get("scale", {}).get("px_per_m_anchor_plane")
            if scale:
                print(f"overhead scale {scale:.0f} px/m from "
                      f"{os.path.basename(path)}", flush=True)
                return float(scale)
    except Exception as exc:                       # never block a run
        print("pool frame unavailable:", exc, flush=True)
    print(f"overhead scale {default:.0f} px/m (2026-08-13 survey; run "
          "pool_remap.py if the camera moved)", flush=True)
    return default


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
    ap.add_argument("--preroll-s", type=float, default=4.0,
                    help="record the stationary start pose before moving")
    ap.add_argument("--outrun-s", type=float, default=7.0,
                    help="keep going straight AFTER the anchor is passed "
                         "so the departure leg is visible")
    ap.add_argument("--confirm", type=int, default=3,
                    help="consecutive confirmations below the trigger "
                         "range before committing")
    ap.add_argument("--range-window", type=int, default=5,
                    help="median filter length on the monocular range")
    args = ap.parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, stamp)
    os.makedirs(sess, exist_ok=True)
    apx = np.array(args.anchor_px)
    px_per_m = overhead_scale()
    # ONE abort predicate, shared with the offline harness and the unit
    # tests (scripts/real/gt_guard.py). Thresholds: docs/GT_VALIDATION.md.
    guard = PoseGuard(margin=args.margin, px_per_m=px_per_m,
                      t0=time.time())

    # Cap raised to 0.50: at 0.22 the thrusters spin but the
    # vehicle does not move against tether drag (operator,
    # 2026-08-14).
    surge = int(max(0.0, min(0.50, args.surge)) * 1000)
    sway = int(max(0.0, min(0.55, args.sway)) * 1000)

    cam = CameraStream()
    tracker = OverheadTracker()
    log, onboard_frames, overhead_frames, traj = [], [], [], []
    # Overhead video is written INCREMENTALLY (a run's worth of 1080p
    # frames does not fit in RAM). RECORDING ONLY: no effect on
    # perception or control, so the D-016 freeze is unaffected.
    ov_w, ov_h = 960, 540
    ov_writer = cv2.VideoWriter(
        os.path.join(sess, "overhead_live.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"), 7, (ov_w, ov_h))
    ov_scale = ov_w / 1920.0

    def write_overhead(det_ov, state, rng, gt_dist, t_el):
        img = tracker.last_frame
        if img is None:
            return
        vis = cv2.resize(img, (ov_w, ov_h))
        ax_, ay_ = int(apx[0] * ov_scale), int(apx[1] * ov_scale)
        cv2.circle(vis, (ax_, ay_), 13, (255, 0, 255), 2)
        cv2.line(vis, (ax_ - 18, ay_), (ax_ + 18, ay_), (255, 0, 255), 1)
        cv2.line(vis, (ax_, ay_ - 18), (ax_, ay_ + 18), (255, 0, 255), 1)
        cv2.putText(vis, "ANCHOR", (ax_ + 16, ay_ - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        if len(traj) > 1:
            pts = (np.array(traj) * ov_scale).astype(np.int32)
            cv2.polylines(vis, [pts], False, (0, 255, 255), 2)
            cv2.circle(vis, tuple(pts[0]), 7, (0, 255, 0), -1)
        if det_ov and det_ov.get("found"):
            bx, by, bw_, bh_ = det_ov["bbox"]
            p1 = (int(bx * ov_scale), int(by * ov_scale))
            p2_ = (int((bx + bw_) * ov_scale), int((by + bh_) * ov_scale))
            cv2.rectangle(vis, p1, p2_, (0, 0, 255), 2)
            cx_, cy_ = det_ov["pixel"]
            cv2.circle(vis, (int(cx_ * ov_scale), int(cy_ * ov_scale)),
                       5, (0, 0, 255), -1)
        cv2.rectangle(vis, (0, 0), (ov_w, 30), (0, 0, 0), -1)
        txt = f"t={t_el:5.1f}s  {state:9s}"
        if rng is not None:
            txt += f"  camera range {rng:.2f} m"
        if gt_dist is not None:
            txt += f"  |  GT {gt_dist / px_per_m:.2f} m"
        cv2.putText(vis, txt, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1)
        ov_writer.write(vis)
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

            state, side, t_trig = "PREROLL", 0, None
            t_pre = time.time()
            t_out = None
            rng_hist = []
            confirm = 0
            start_frac = None
            t_str = None
            lost = 0
            p_start = None
            head_dir = None
            min_gt = 1e9
            t0 = time.time()
            # The guard's no-pose timeout runs from HERE, not from
            # construction: camera warm-up, arming and the centring
            # window take tens of seconds, and the guard must not spend
            # its acquisition budget while nothing is looking yet.
            guard.reset(t0)
            aborted = None
            trig_info = None
            try:
                while time.time() - t0 < args.timeout:
                    fr, _ = cam.latest()
                    det = detect_anchor(fr)
                    ok = accept(det)
                    rng_raw = rng_of(det) if ok else None
                    brg = brg_of(det) if ok else None
                    # MEDIAN-FILTERED range + CONFIRMATION (imported from
                    # the Phase-7B qualification used in simulation).
                    # Pilot evidence 2026-08-14: a single noisy sample
                    # (monocular MAE 0.44 m) fired the trigger on the
                    # FIRST frame that saw the anchor, so the vehicle
                    # curved immediately instead of running straight
                    # first (run 19:41 triggered at t=1.06 s).
                    if rng_raw is not None:
                        rng_hist.append(rng_raw)
                        del rng_hist[:-args.range_window]
                    rng = (float(np.median(rng_hist)) if rng_hist
                           else None)

                    att = rov.recv_match("ATTITUDE", timeout=0.15)
                    yaw = att.yaw if att else yaw_target
                    yrate = getattr(att, "yawspeed", 0.0) if att else 0.0
                    yerr = wrap(yaw_target - yaw)
                    r = 6.0 * math.degrees(yerr) - 2.0 * math.degrees(yrate)
                    if abs(math.degrees(yerr)) < 1.5 and abs(yrate) < 0.03:
                        r = 0.0
                    r = int(max(-200, min(200, r)))

                    # ---- overhead ground truth (every cycle) ----------
                    # The guard owns EVERY abort decision (wall margin,
                    # missing/stale pose, impossible jump, low-confidence
                    # blob). It is the same object the offline harness
                    # and the unit tests drive, so its behaviour is
                    # qualified before the vehicle is in the water.
                    ovd = tracker.detect(near=near)
                    verdict = guard.update(ovd, t=time.time())
                    gt_dist = None
                    log_extra = {"gt": verdict.reason or verdict.level}
                    if verdict.abort:
                        aborted = f"GT guard: {verdict.reason}"
                        print(f"!! ABORT ({verdict.reason}): "
                              f"{json.dumps(verdict.detail, default=str)}"
                              " — active hold", flush=True)
                        break
                    if verdict.ok:
                        if start_frac is None:
                            start_frac = guard.snapshot()["start_frac"]
                            print("wall guard box (frame fractions): "
                                  f"{guard.snapshot()['box']}", flush=True)
                        near = ovd["pixel"]
                        traj.append(list(near))
                        log_extra["area_px"] = ovd.get("area_px")
                        if p_start is None:
                            p_start = np.array(near, dtype=float)
                        elif state == "APPROACH":
                            v = np.array(near, dtype=float) - p_start
                            if np.linalg.norm(v) > 60:
                                head_dir = v / np.linalg.norm(v)
                        gt_dist = float(np.linalg.norm(
                            np.array(near) - apx))
                        min_gt = min(min_gt, gt_dist)

                    x, y = surge, 0
                    if state == "PREROLL":
                        # stationary, recorded: the video must show where
                        # the vehicle starts before anything happens
                        x, y = 0, 0
                        # ... and do NOT start moving before the overhead
                        # ground truth has a lock: without it the wall
                        # guard cannot see an excursion. Worst measured
                        # acquisition was 7.4 s (mission 19:38), well
                        # inside the guard's no-pose timeout.
                        if not guard.has_fix:
                            t_pre = time.time()
                        elif time.time() - t_pre > args.preroll_s:
                            state = "APPROACH"
                            print("== APPROACH (straight)", flush=True)
                    elif state == "APPROACH" and rng is not None                             and rng <= args.trigger_m:
                        confirm += 1
                        if confirm < args.confirm:
                            print(f"  candidate {confirm}/{args.confirm} "
                                  f"at {rng:.2f} m", flush=True)
                    elif state == "APPROACH":
                        confirm = 0
                    if state == "APPROACH" and confirm >= args.confirm:
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
                        state = "OUTRUN"
                        t_out = time.time()
                        print("== OUTRUN: straight departure leg",
                              flush=True)
                    if state == "OUTRUN":
                        x, y = surge, 0
                        if time.time() - t_out > args.outrun_s:
                            state = "DONE"
                            print("== DONE", flush=True)
                            break

                    rov.manual(x=x, y=y, r=r)
                    write_overhead(ovd, state, rng, gt_dist,
                                   time.time() - t0)
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
                    if verdict.ok:
                        overhead_frames.append((list(near), state,
                                                gt_dist))
                    if len(log) % 8 == 1:
                        print(f"  {state:8s} rng={log[-1]['range_m']} "
                              f"brg={log[-1]['bearing_deg']} "
                              f"gt={log[-1]['gt_dist_px']} x={x} y={y}",
                              flush=True)
                    time.sleep(0.1)
            finally:
                print("active hold (recorded)...", flush=True)

                def _hold_sample(el, err, cmd):
                    if int(el * 7) != getattr(_hold_sample, "_k", -1):
                        _hold_sample._k = int(el * 7)
                        ovh = tracker.detect(near=near)
                        if ovh and ovh.get("found"):
                            traj.append(list(ovh["pixel"]))
                        write_overhead(ovh, "HOLD", None, None,
                                       time.time() - t0)

                rov.hold_heading(args.hold_s, target_yaw=yaw_target,
                                 on_sample=_hold_sample)
                print("disarm:", rov.disarm(), flush=True)

        # ---------- media ----------
        for _ in range(10):        # a couple of seconds of the hold
            ovd2 = tracker.detect(near=near)
            if ovd2 and ovd2.get("found"):
                near = ovd2["pixel"]
                traj.append(list(near))
            write_overhead(ovd2, "HOLD", None,
                           None if not ovd2 or not ovd2.get("found")
                           else float(np.linalg.norm(
                               np.array(near) - apx)),
                           time.time() - t0)
        ov_writer.release()
        ovimg, _ = tracker.grab()
        vis_ov = ovimg.copy()
        ovf_probe = tracker.detect(near=near)
        ovf_box = ovf_probe.get("bbox") if ovf_probe.get("found") else None
        pts = np.array(traj, dtype=np.int32)
        if len(pts) > 1:
            # thicker trail + direction arrows so the motion reads at a
            # glance (a thin polyline was hard to see)
            cv2.polylines(vis_ov, [pts], False, (0, 0, 0), 9)
            cv2.polylines(vis_ov, [pts], False, (0, 255, 255), 5)
            for i in range(0, len(pts) - 1, max(1, len(pts) // 8)):
                cv2.arrowedLine(vis_ov, tuple(pts[i]), tuple(pts[i + 1]),
                                (0, 200, 255), 3, tipLength=1.2)
            cv2.circle(vis_ov, tuple(pts[0]), 16, (0, 0, 0), -1)
            cv2.circle(vis_ov, tuple(pts[0]), 13, (0, 255, 0), -1)
            cv2.putText(vis_ov, "START", (pts[0][0] + 18, pts[0][1] + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            cv2.circle(vis_ov, tuple(pts[-1]), 16, (0, 0, 0), -1)
            cv2.circle(vis_ov, tuple(pts[-1]), 13, (0, 0, 255), -1)
            cv2.putText(vis_ov, "END", (pts[-1][0] + 18, pts[-1][1] + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        if ovf_box is not None:
            bx, by, bw_, bh_ = ovf_box
            cv2.rectangle(vis_ov, (bx, by), (bx + bw_, by + bh_),
                          (0, 0, 255), 3)
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
                   "min_gt_dist_m": None if min_gt > 1e8
                   else round(min_gt / px_per_m, 2),
                   "px_per_m": px_per_m,
                   "accepted_frac": round(
                       sum(1 for r in log if r["accepted"])
                       / max(1, len(log)), 3),
                   "gt_guard": guard.snapshot(),
                   "gt_guard_config": guard.cfg.as_dict(),
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
