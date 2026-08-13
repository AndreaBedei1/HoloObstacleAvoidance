"""Metric measurement of the real pool scene from the overhead RealSense.

Read-only capture: aligns depth to color, medians N frames for stability,
deprojects a set of annotated pixels to 3D (camera frame, meters), and
reports pairwise distances for the pool geometry (length to the rod, pool
width, camera height). Saves the median RGB, a depth visualization, and a
JSON report. Never writes camera settings.

NOTE: stereo depth through water is refraction-biased and IR-attenuated —
underwater distances (water depths) are NOT measured here; they come from
the operator's tape measurements. Only above-water geometry is verified.

Usage:
    python scripts/measure_pool_geometry.py --out visualizations/pool_setup_20260812
"""

import argparse
import json
import os
import time

import numpy as np
import pyrealsense2 as rs

try:
    import cv2
except ImportError:
    cv2 = None

# Pixels of interest in the 1920x1080 COLOR image, chosen on the sample
# frame. Each entry: name -> (u, v). Updated by inspection of the scene.
POINTS = {
    "pool_left_end_waterline": (60, 520),
    "rod_top": (1258, 40),
    "rod_mid": (1262, 540),
    "rod_bottom": (1252, 1000),
    "near_rim_at_rod": (1250, 1035),
    "far_rim_at_rod": (1255, 15),
    "near_rim_left": (400, 1005),
    "far_rim_left": (390, 25),
    "right_image_edge_water": (1890, 540),
    "water_surface_left": (600, 520),
    "water_surface_right": (1600, 520),
}

PAIRS = [
    ("pool_left_end_waterline", "rod_mid", "left_end_to_rod"),
    ("rod_mid", "right_image_edge_water", "rod_to_right_image_edge"),
    ("pool_left_end_waterline", "right_image_edge_water", "visible_length"),
    ("far_rim_at_rod", "near_rim_at_rod", "pool_width_at_rod"),
    ("far_rim_left", "near_rim_left", "pool_width_left"),
    ("rod_top", "rod_bottom", "rod_visible_span"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="visualizations/pool_setup_20260812")
    ap.add_argument("--frames", type=int, default=30)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    for _ in range(15):
        pipe.wait_for_frames(5000)

    depth_acc = []
    color_img = None
    intr = None
    for _ in range(args.frames):
        frames = align.process(pipe.wait_for_frames(5000))
        d = frames.get_depth_frame()
        c = frames.get_color_frame()
        if not d or not c:
            continue
        if intr is None:
            intr = c.get_profile().as_video_stream_profile().get_intrinsics()
        depth_acc.append(np.asanyarray(d.get_data()).astype(np.float32))
        color_img = np.asanyarray(c.get_data()).copy()
    pipe.stop()

    depth = np.stack(depth_acc)
    depth[depth == 0] = np.nan
    med = np.nanmedian(depth, axis=0) * depth_scale  # meters
    valid_frac = float(np.mean(~np.isnan(depth)).round(3))

    def sample(uv, win=9):
        u, v = uv
        patch = med[max(0, v - win):v + win, max(0, u - win):u + win]
        vals = patch[np.isfinite(patch)]
        return float(np.median(vals)) if vals.size else None

    report = {"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
              "depth_valid_fraction": valid_frac,
              "points": {}, "distances_m": {}}
    pts3d = {}
    for name, uv in POINTS.items():
        z = sample(uv)
        entry = {"pixel": uv, "depth_m": None if z is None else round(z, 3)}
        if z is not None:
            x, y, zz = rs.rs2_deproject_pixel_to_point(
                intr, [float(uv[0]), float(uv[1])], z)
            pts3d[name] = np.array([x, y, zz])
            entry["xyz_m"] = [round(x, 3), round(y, 3), round(zz, 3)]
        report["points"][name] = entry

    for a, b, label in PAIRS:
        if a in pts3d and b in pts3d:
            report["distances_m"][label] = round(
                float(np.linalg.norm(pts3d[a] - pts3d[b])), 3)

    if cv2 is not None and color_img is not None:
        for name, uv in POINTS.items():
            cv2.drawMarker(color_img, uv, (0, 0, 255),
                           cv2.MARKER_CROSS, 24, 3)
            cv2.putText(color_img, name, (uv[0] + 8, uv[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.imwrite(os.path.join(args.out, "annotated_rgb.png"), color_img)
        d8 = np.nan_to_num(med, nan=0.0)
        d8 = np.clip(d8 / 8.0 * 255.0, 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(args.out, "median_depth_aligned.png"),
                    cv2.applyColorMap(d8, cv2.COLORMAP_JET))

    out = os.path.join(args.out, "pool_geometry.json")
    with open(out, "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
