"""Overhead RealSense capture with exposure control for the pool remap.

Applies VOLATILE runtime settings only (auto-exposure off + manual
exposure for the RGB sensor while streaming; nothing persisted to the
camera). Saves median-filtered aligned depth + RGB at several exposures
so the least-glared frame can be picked for reference-point picking.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="visualizations/pool_remap_20260814")
    ap.add_argument("--exposures", default="auto,160,80,40",
                    help="comma list: 'auto' or manual exposure values")
    ap.add_argument("--frames", type=int, default=20)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    dev = profile.get_device()
    color_sensor = next(s for s in dev.query_sensors()
                        if s.get_info(rs.camera_info.name) == "RGB Camera")
    scale = dev.first_depth_sensor().get_depth_scale()

    report = {"captures": []}
    for exp in args.exposures.split(","):
        exp = exp.strip()
        if exp == "auto":
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        else:
            color_sensor.set_option(rs.option.enable_auto_exposure, 0)
            color_sensor.set_option(rs.option.exposure, float(exp))
        for _ in range(12):   # settle
            pipe.wait_for_frames(5000)
        acc = []
        color_img = None
        intr = None
        for _ in range(args.frames):
            fr = align.process(pipe.wait_for_frames(5000))
            d, c = fr.get_depth_frame(), fr.get_color_frame()
            if not d or not c:
                continue
            if intr is None:
                p = c.get_profile().as_video_stream_profile().get_intrinsics()
                intr = {"fx": p.fx, "fy": p.fy, "ppx": p.ppx, "ppy": p.ppy,
                        "w": p.width, "h": p.height}
            acc.append(np.asanyarray(d.get_data()).astype(np.float32))
            color_img = np.asanyarray(c.get_data()).copy()
        med = np.stack(acc)
        med[med == 0] = np.nan
        med = np.nanmedian(med, axis=0) * scale
        tag = f"exp_{exp}"
        if cv2 is not None:
            cv2.imwrite(os.path.join(args.out, f"rgb_{tag}.png"), color_img)
            d8 = np.clip(np.nan_to_num(med) / 6.0 * 255, 0, 255).astype(
                np.uint8)
            cv2.imwrite(os.path.join(args.out, f"depth_{tag}.png"),
                        cv2.applyColorMap(d8, cv2.COLORMAP_JET))
        np.save(os.path.join(args.out, f"depth_{tag}.npy"), med)
        mean_lum = float(color_img.mean())
        report["captures"].append({"exposure": exp, "mean_lum": mean_lum,
                                   "intrinsics": intr})
        print(f"{tag}: mean luminance {mean_lum:.0f}")
    # restore auto exposure (volatile session default)
    color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    pipe.stop()
    with open(os.path.join(args.out, "captures.json"), "w") as f:
        json.dump(report, f, indent=2)


if __name__ == "__main__":
    main()
