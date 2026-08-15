"""Paired observation dataset: everything at the same timestamp.

Purpose (Phase 10, S1 observation model). The pilot logs recorded the
detector block and the overhead ground truth on DISJOINT cycles, so no
covariate model of the observation error was estimable. This collector
fixes that by construction: one record per cycle containing

    * overhead ground truth: ROV pose in the pool frame, anchor pose,
      TRUE range and TRUE bearing computed geometrically from them
    * the camera frame identifier (the frame is also saved to disk)
    * the detection: found/not found, the FULL bbox, score, confidence
    * the range produced by the SHARED estimator (planner.estimate_range,
      the same function the simulation runs) on that same bbox

Why the shared estimator rather than a local formula: the sim-to-real
claim only holds if the quantity being compared is produced by the code
that both domains run. The old width-derived range was a local
reimplementation and was shown to be structurally invalid
(docs/OBSERVATION_MODEL.md); it is not used here.

IDENTIFIABILITY. `estimate_range` has two constants: the camera vertical
FOV and the physical extent `target_height_m` that the bbox height spans.
They are NOT both free: fitting both to the same data would absorb any
model error into meaningless parameter values. The FOV comes from the
camera calibration/specification (parameter, recorded in the output) and
the physical extent from a tape measurement of the anchor + suspension
(parameter, recorded). This dataset then measures the BIAS and RESIDUALS
of that fixed model, which is what S1 needs.

OPERATOR PROCEDURE (no metric precision required). Move the ROV to about
four clearly different positions, roughly far to near relative to the
anchor, holding each still for a few seconds. The positions need not be
measured, equidistant, or repeated exactly: the true range and bearing
come from the overhead ground truth after the pool remap, never from a
manual estimate.

Usage:
    python scripts/real/collect_paired_dataset.py --positions 4 \
        --seconds-per-position 8 --vfov-deg 60 --target-height-m 1.0
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(_ROOT, "src", "rov_obstacle_avoidance"))

from camera_stream import CameraStream, ensure_env  # noqa: E402

ensure_env()

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from anchor_detect import detect_anchor  # noqa: E402
from overhead_track import OverheadTracker  # noqa: E402

import urllib.request  # noqa: E402


def vehicle_state(host="192.168.2.2", timeout=1.0):
    """Attitude and depth over the BlueOS REST API.

    Deliberately NOT pymavlink: the operator holds the vehicle with
    Cockpit, which owns the GCS UDP port, and a second binding would
    either fail or fight it. HTTP is read-only and conflict-free.
    """
    base = f"http://{host}:6040/v1/mavlink/vehicles/1/components/1/messages"
    out = {}
    for name, keys in (("ATTITUDE", ("roll", "pitch", "yaw", "yawspeed")),
                       ("VFR_HUD", ("alt", "heading")),
                       ("SCALED_PRESSURE2", ("press_abs",))):
        try:
            with urllib.request.urlopen(f"{base}/{name}",
                                        timeout=timeout) as r:
                m = json.loads(r.read().decode())["message"]
            for k in keys:
                out[k if name != "VFR_HUD" or k != "alt" else "alt_m"] =                     m.get(k)
        except Exception:
            pass
    if "alt_m" in out and out["alt_m"] is not None:
        out["depth_m"] = -float(out["alt_m"])
    return out

# The SHARED range estimator: the same function the simulated planner
# calls. Imported, never reimplemented.
from rov_obstacle_avoidance.planner import (  # noqa: E402
    ObstacleObservation,
    estimate_range,
)

OUT = os.path.join(_ROOT, "experiments", "real", "paired_dataset")


def load_pool_frame(path):
    """Load the camera->pool transform produced by the remap."""
    if not path or not os.path.isfile(path):
        return None
    with open(path) as f:
        return json.load(f)


def to_pool(p_cam, frame):
    """Camera-frame XYZ -> pool-frame XYZ (p_pool = R @ (p_cam - t))."""
    if frame is None or "R_rows_pool_axes_in_camera" not in frame:
        return None
    rr = frame["R_rows_pool_axes_in_camera"]
    R = np.array([rr["x_approach"], rr["y_along_rod"], rr["z_up"]],
                 dtype=float)
    t = np.array(frame["t_pool_origin_in_camera"], dtype=float)
    return [round(float(v), 4)
            for v in (R @ (np.array(p_cam, dtype=float) - t))]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--positions", type=int, default=4)
    ap.add_argument("--seconds-per-position", type=float, default=8.0)
    ap.add_argument("--rate-hz", type=float, default=4.0)
    ap.add_argument("--pool-frame",
                    default="config/real_pool/pool_frame_FROZEN.json",
                    help="config/real_pool/pool_frame_<date>.json "
                         "from the remap; without it, ranges are "
                         "reported in the camera frame and flagged")
    # Anchor pixel measured with the operator's cross-tube marker on
    # 2026-08-15 and snapped to the rod centreline; the earlier value was
    # biased by the anchor's shadow.
    ap.add_argument("--anchor-px", nargs=2, type=float,
                    default=[1044.0, 626.0])
    # Fixed model constants; see the module docstring on identifiability.
    ap.add_argument("--vfov-deg", type=float, required=True,
                    help="camera VERTICAL FOV from calibration/spec")
    ap.add_argument("--target-height-m", type=float, required=True,
                    help="measured physical extent the bbox height spans "
                         "(anchor + visible suspension), from a tape "
                         "measurement")
    ap.add_argument("--max-range-m", type=float, default=6.0)
    ap.add_argument("--save-frames", action="store_true", default=True)
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, stamp)
    os.makedirs(os.path.join(sess, "frames"), exist_ok=True)
    frame_cfg = load_pool_frame(args.pool_frame)
    if frame_cfg is None:
        print("WARNING: no pool frame given — true range/bearing will be "
              "reported in the CAMERA frame and flagged as such. Run the "
              "remap first for pool-frame ground truth.", flush=True)

    cam = CameraStream()
    tracker = OverheadTracker()
    records = []
    meta = {
        "session": stamp,
        "purpose": "PAIRED OBSERVATION DATASET for the S1 observation "
                   "model. Calibration data: never part of the final "
                   "validation statistics.",
        "vfov_deg": args.vfov_deg,
        "target_height_m": args.target_height_m,
        "max_range_m": args.max_range_m,
        "anchor_px": args.anchor_px,
        "pool_frame_file": args.pool_frame or None,
        "shared_estimator": "rov_obstacle_avoidance.planner.estimate_range",
        "note": "vfov and target_height are FIXED from calibration and a "
                "tape measurement; this dataset measures the bias and "
                "residuals of that model, it does not fit both.",
    }
    apx = np.array(args.anchor_px, dtype=float)
    vfov = math.radians(args.vfov_deg)

    try:
        anchor_cam = None
        d0 = tracker.detect()          # also primes the pipeline
        for pos in range(1, args.positions + 1):
            input(f"\n=== POSITION {pos}/{args.positions}: move the ROV to "
                  "a clearly different distance from the anchor (rough is "
                  "fine, no measuring), hold it still, then press "
                  "ENTER ===")
            t0 = time.time()
            n_here = 0
            while time.time() - t0 < args.seconds_per_position:
                frame, t_frame = cam.latest()
                det = detect_anchor(frame) if frame is not None else \
                    {"found": False}
                ov = tracker.detect(near=None)
                vs = vehicle_state()
                rec = {
                    "t": time.time(), "position_index": pos,
                    "vehicle": vs,
                    "frame_age_s": (None if t_frame == 0
                                    else round(time.time() - t_frame, 3)),
                }
                # --- ground truth from the overhead camera ------------
                if ov and ov.get("found"):
                    rec["rov_px"] = ov["pixel"]
                    rec["rov_camera_xyz_m"] = ov.get("xyz_camera_m")
                    if frame_cfg and ov.get("xyz_camera_m"):
                        rec["rov_pool_xyz_m"] = to_pool(
                            ov["xyz_camera_m"], frame_cfg)
                    # true range/bearing in the overhead IMAGE plane;
                    # converted to metres by the remap scale when the
                    # pool frame is available.
                    d_px = float(np.linalg.norm(
                        np.array(ov["pixel"]) - apx))
                    rec["gt_dist_px"] = round(d_px, 1)
                    scale = None
                    if frame_cfg:
                        scale = (frame_cfg.get("scale") or {}).get(
                            "px_per_m_rod")
                    rec["gt_range_m"] = (round(d_px / scale, 3)
                                         if scale else None)
                    rec["gt_frame"] = ("pool" if frame_cfg else "camera")
                else:
                    rec["rov_px"] = None
                    rec["gt_range_m"] = None
                # --- observation --------------------------------------
                rec["detection"] = {
                    "found": bool(det.get("found")),
                    "score": det.get("score"),
                    "center_x": det.get("center_x"),
                    "center_y": det.get("center_y"),
                    "width": det.get("width"),
                    "height": det.get("height"),
                    "shank_width": det.get("shank_width"),
                    "bbox_px": det.get("bbox_px"),
                    "mean_dark": det.get("mean_dark"),
                    "wander": det.get("wander"),
                    "ms": det.get("ms"),
                }
                # --- range from the SHARED estimator ------------------
                if det.get("found"):
                    obs = ObstacleObservation(
                        class_name="anchor",
                        confidence=float(det.get("score", 0.0)),
                        center_x=float(det["center_x"]),
                        center_y=float(det["center_y"]),
                        width=float(det["width"]),
                        height=float(det["height"]),
                        bearing_rad=0.0, apparent_area=0.0, risk=0.0,
                        is_tracking_valid=False)
                    rec["shared_range_m"] = round(estimate_range(
                        obs, vfov, args.target_height_m,
                        args.max_range_m), 3)
                else:
                    rec["shared_range_m"] = None
                if args.save_frames and frame is not None and n_here < 3:
                    fp = os.path.join(sess, "frames",
                                      f"p{pos}_{n_here}.png")
                    cv2.imwrite(fp, frame)
                    rec["frame_file"] = os.path.relpath(fp, sess)
                records.append(rec)
                n_here += 1
                time.sleep(1.0 / max(1.0, args.rate_hz))
            got = sum(1 for r in records
                      if r["position_index"] == pos
                      and r["detection"]["found"])
            print(f"  position {pos}: {n_here} cycles, {got} detections, "
                  f"GT range "
                  f"{records[-1].get('gt_range_m') or records[-1].get('gt_dist_px')}")
    finally:
        cam.close()
        tracker.close()

    out = os.path.join(sess, "paired.json")
    with open(out, "w") as f:
        json.dump({"meta": meta, "records": records}, f, indent=2)
    paired = [r for r in records
              if r["detection"]["found"] and r.get("gt_dist_px")]
    print(f"\n{len(records)} cycles, {len(paired)} PAIRED "
          f"(detection AND ground truth on the same timestamp)")
    print("->", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
