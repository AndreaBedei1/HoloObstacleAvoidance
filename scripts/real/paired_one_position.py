"""Record ONE position of the paired observation dataset.

Run once per operator position: the operator moves the vehicle with the
Cockpit between calls, so no interactive prompt is needed and the
collector never contends for the MAVLink port (telemetry is read over
the BlueOS HTTP API).

Each record pairs, on ONE timestamp: overhead ground truth (ROV pose in
the pool frame, true range and bearing to the anchor), the vehicle's
attitude and depth, the full detector bbox, and the range from the
SHARED planner.estimate_range().
"""

import argparse
import json
import math
import os
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "src", "rov_obstacle_avoidance"))

import cv2                                            # noqa: E402
import numpy as np                                    # noqa: E402

from anchor_detect import detect_anchor               # noqa: E402
from camera_stream import CameraStream                # noqa: E402
from overhead_track import OverheadTracker            # noqa: E402
from rov_obstacle_avoidance.planner import (          # noqa: E402
    ObstacleObservation, estimate_range)

BASE = "http://192.168.2.2:6040/v1/mavlink/vehicles/1/components/1/messages"


def vehicle_state():
    out = {}
    for name in ("ATTITUDE", "VFR_HUD"):
        try:
            with urllib.request.urlopen(f"{BASE}/{name}", timeout=1.0) as r:
                m = json.loads(r.read().decode())["message"]
            if name == "ATTITUDE":
                out.update({"roll": m["roll"], "pitch": m["pitch"],
                            "yaw": m["yaw"]})
            else:
                out.update({"depth_m": -m["alt"], "heading_deg": m["heading"]})
        except Exception:
            pass
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--position", type=int, required=True)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--rate-hz", type=float, default=4.0)
    ap.add_argument("--vfov-deg", type=float, default=60.0)
    ap.add_argument("--target-height-m", type=float, default=0.75)
    ap.add_argument("--anchor-px", nargs=2, type=float,
                    default=[1044.0, 626.0])
    ap.add_argument("--pool-frame",
                    default=os.path.join(ROOT, "config", "real_pool",
                                         "pool_frame_FROZEN.json"))
    args = ap.parse_args()

    sess = os.path.join(ROOT, "experiments", "real", "paired_dataset",
                        args.session)
    os.makedirs(os.path.join(sess, "frames"), exist_ok=True)
    with open(args.pool_frame) as f:
        pf = json.load(f)
    rr = pf["R_rows_pool_axes_in_camera"]
    R = np.array([rr["x_approach"], rr["y_along_rod"], rr["z_up"]])
    t = np.array(pf["t_pool_origin_in_camera"], dtype=float)
    anchor_pool = np.array([pf["anchor_pose_pool_m"]["x"],
                            pf["anchor_pose_pool_m"]["y"]], dtype=float)
    apx = np.array(args.anchor_px, dtype=float)
    scale = pf["scale"]["px_per_m_rod"]
    vfov = math.radians(args.vfov_deg)

    cam = CameraStream()
    tracker = OverheadTracker()
    recs, n_det, n_gt = [], 0, 0
    try:
        t0 = time.time()
        k = 0
        while time.time() - t0 < args.seconds:
            frame, t_frame = cam.latest()
            det = detect_anchor(frame) if frame is not None else {}
            ov = tracker.detect()
            vs = vehicle_state()
            rec = {"t": time.time(), "position": args.position,
                   "vehicle": vs}
            if ov and ov.get("found"):
                n_gt += 1
                rec["rov_px"] = ov["pixel"]
                if ov.get("xyz_camera_m"):
                    p = R @ (np.array(ov["xyz_camera_m"]) - t)
                    rec["rov_pool_xyz_m"] = [round(float(v), 4) for v in p]
                    d = anchor_pool - p[:2]
                    rec["gt_range_pool_m"] = round(
                        float(np.linalg.norm(d)), 4)
                    rec["gt_bearing_pool_deg"] = round(
                        math.degrees(math.atan2(d[1], d[0])), 2)
                d_px = float(np.linalg.norm(np.array(ov["pixel"]) - apx))
                rec["gt_dist_px"] = round(d_px, 1)
                rec["gt_range_px_scale_m"] = round(d_px / scale, 4)
            if det.get("found"):
                n_det += 1
                rec["det"] = {k2: det.get(k2) for k2 in
                              ("score", "center_x", "center_y", "width",
                               "height", "shank_width", "mean_dark",
                               "wander", "ms")}
                obs = ObstacleObservation(
                    class_name="anchor", confidence=float(det["score"]),
                    center_x=float(det["center_x"]),
                    center_y=float(det["center_y"]),
                    width=float(det["width"]), height=float(det["height"]),
                    bearing_rad=0.0, apparent_area=0.0, risk=0.0,
                    is_tracking_valid=False)
                rec["shared_range_m"] = round(
                    estimate_range(obs, vfov, args.target_height_m, 6.0), 3)
            else:
                rec["det"] = {"found": False}
            # Save EVERY frame: the detector is still being adapted to
            # the real lighting, so the pairing must be re-computable
            # offline from raw frames rather than depending on the
            # detector version that happened to run live.
            if frame is not None:
                fp = os.path.join(sess, "frames",
                                  f"p{args.position}_{k}.png")
                cv2.imwrite(fp, frame)
                rec["frame_file"] = os.path.relpath(fp, sess)
            recs.append(rec)
            k += 1
            time.sleep(1.0 / args.rate_hz)
    finally:
        cam.close()
        tracker.close()

    out = os.path.join(sess, f"position_{args.position}.json")
    with open(out, "w") as f:
        json.dump({"meta": {"position": args.position,
                            "vfov_deg": args.vfov_deg,
                            "target_height_m": args.target_height_m,
                            "anchor_px": args.anchor_px,
                            "pool_frame": os.path.basename(args.pool_frame),
                            "note": "PILOT/CALIBRATION data"},
                   "records": recs}, f, indent=2)
    paired = [r for r in recs if r.get("det", {}).get("score")
              and r.get("gt_range_pool_m")]
    print(f"posizione {args.position}: {len(recs)} cicli | "
          f"{n_det} detezioni | {n_gt} ground truth | "
          f"{len(paired)} PAIRED")
    if paired:
        gts = [r["gt_range_pool_m"] for r in paired]
        shs = [r["shared_range_m"] for r in paired]
        hs = [r["det"]["height"] for r in paired]
        print(f"  GT range {min(gts):.2f}-{max(gts):.2f} m | "
              f"bbox height {min(hs):.3f}-{max(hs):.3f} | "
              f"shared range {min(shs):.2f}-{max(shs):.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
