"""Stepped approach toward the anchor: apparent size vs true distance.

Produces the data that calibrates the REAL monocular range model (the
sim used H_ref/(2 h tan(VFOV/2)); the real camera's focal length in
pixels is measured here instead of assumed).

Per step:
  * short surge pulse (bounded), then settle
  * overhead RealSense: ROV pixel + metric position (external GT)
  * onboard camera: one frame -> weight-free anchor detector -> bbox
Everything logged to a session folder; the fit is done offline by
scripts/real/fit_range_model.py.

SAFETY: overhead safe-box supervision aborts the whole sequence if the
ROV approaches the frame border (outside the view it grounds).
"""

import argparse
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from anchor_detect import annotate, detect_anchor  # noqa: E402
from overhead_track import OverheadTracker  # noqa: E402
from rovlink import RovLink  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyrealsense2 as rs  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "experiments", "real",
                   "approach_dataset")


def grab_onboard(dst_dir: str, tag: str):
    env = dict(os.environ)
    env["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = \
        "protocol_whitelist;file,rtp,udp|fflags;nobuffer|flags;low_delay"
    sub = os.path.join(dst_dir, tag)
    os.makedirs(sub, exist_ok=True)
    subprocess.run([sys.executable, os.path.join(HERE, "camera_grab.py"),
                    "--frames", "1", "--out", sub],
                   env=env, capture_output=True, timeout=60)
    files = [f for f in sorted(os.listdir(sub)) if f.endswith(".png")]
    if not files:
        return None, None
    p = os.path.join(sub, files[-1])
    return p, cv2.imread(p)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=5)
    ap.add_argument("--power", type=float, default=0.12)
    ap.add_argument("--pulse", type=float, default=1.2)
    ap.add_argument("--settle", type=float, default=4.0)
    ap.add_argument("--margin", type=float, default=0.02)
    ap.add_argument("--mode", default="ALT_HOLD")
    args = ap.parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, stamp)
    os.makedirs(sess, exist_ok=True)

    tracker = OverheadTracker()
    records = []
    try:
        ref, _ = tracker.grab()
        cv2.imwrite(os.path.join(sess, "overhead_ref.png"), ref)
        last_px = None

        def observe(step: int, phase: str):
            nonlocal last_px
            cur, depth = tracker.grab()
            det_ov = tracker.detect_motion(ref, cur, near=last_px)
            if det_ov.get("found"):
                last_px = det_ov["pixel"]
                if depth is not None and tracker.intr is not None:
                    cx, cy = det_ov["pixel"]
                    win = depth[max(0, int(cy) - 12):int(cy) + 12,
                                max(0, int(cx) - 12):int(cx) + 12]
                    win = win[win > 0]
                    if win.size:
                        z = float(np.median(win))
                        det_ov["xyz_camera_m"] = [
                            round(v, 3) for v in
                            rs.rs2_deproject_pixel_to_point(
                                tracker.intr, [cx, cy], z)]
            cv2.imwrite(os.path.join(sess, f"overhead_{step}_{phase}.png"),
                        cur)
            path, img = grab_onboard(sess, f"onboard_{step}_{phase}")
            det_cam = detect_anchor(img) if img is not None else {
                "found": False}
            if img is not None:
                cv2.imwrite(
                    os.path.join(sess, f"det_{step}_{phase}.png"),
                    annotate(img, det_cam))
            rec = {"step": step, "phase": phase, "t": time.time(),
                   "overhead": det_ov, "onboard": det_cam,
                   "onboard_path": path}
            records.append(rec)
            ov = det_ov.get("xyz_camera_m")
            print(f"  step {step} {phase}: overhead {ov} | "
                  f"anchor bbox h={det_cam.get('height')} "
                  f"w={det_cam.get('width')} cx={det_cam.get('center_x')}")
            return det_ov

        print("== baseline observation")
        d0 = observe(0, "start")
        if not d0.get("found"):
            print("WARNING: ROV not located from overhead at start")

        mag = int(max(0.0, min(0.25, args.power)) * 1000)
        with RovLink() as rov:
            from pymavlink import mavutil
            for mid, hz in [
                    (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
                    (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10)]:
                rov.set_message_interval(mid, hz)
            rov.set_mode(args.mode)
            hb = rov.arm(mode=args.mode)
            print("armed:", hb)
            if not hb or not hb["armed"]:
                return 1
            if hb["mode"] != args.mode:
                rov.set_mode(args.mode)
            try:
                for step in range(1, args.steps + 1):
                    print(f"== step {step}: surge pulse")
                    rov.pulse(x=mag, duration_s=args.pulse, rate_hz=10)
                    t0 = time.time()
                    while time.time() - t0 < args.settle:
                        rov.neutral()
                        time.sleep(0.1)
                    att = rov.recv_match("ATTITUDE", timeout=1.5)
                    vfr = rov.recv_match("VFR_HUD", timeout=1.5)
                    d = observe(step, "after")
                    records[-1]["attitude"] = (
                        {"yaw": att.yaw, "pitch": att.pitch,
                         "roll": att.roll} if att else None)
                    records[-1]["depth_m"] = (-vfr.alt if vfr else None)
                    if d.get("found"):
                        fx, fy = d["frac_x"], d["frac_y"]
                        m = args.margin
                        if not (m < fx < 1 - m and m < fy < 1 - m):
                            print("!! ABORT: safe-box boundary")
                            break
            finally:
                rov.neutral()
                time.sleep(1.0)
                hb = rov.disarm()
                print("disarmed:", hb)
    finally:
        tracker.close()

    with open(os.path.join(sess, "records.json"), "w") as f:
        json.dump(records, f, indent=2, default=str)
    print("session:", sess)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
