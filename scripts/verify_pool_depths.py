"""Refraction-corrected water-depth cross-check from the overhead RealSense.

The stereo depth through calm clear water returns the POOL BOTTOM with the
classic refraction bias (apparent depth ~ real/1.33, paraxial). This script
fits the water-surface plane from above-water rim points, samples bottom
returns in the three zones the operator measured with a tape (left 60 cm,
rod/anchor 130 cm, right 80 cm), and reports the refraction-corrected
estimates alongside. A CROSS-CHECK, not a replacement for the tape.

Read-only capture; no camera settings are written.
"""

import argparse
import json
import os
import time

import numpy as np
import pyrealsense2 as rs

RIM_PIXELS = [(390, 25), (400, 1005), (1255, 15), (1250, 1035),
              (60, 520)]
ZONES = {
    "left_60cm": [(500, 420), (600, 520), (700, 620), (550, 700)],
    "rod_130cm": [(1130, 460), (1150, 540), (1140, 640), (1330, 540)],
    "right_80cm": [(1820, 460), (1890, 540), (1860, 660), (1780, 380)],
}
N_REFRACT = 1.333


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="visualizations/pool_setup_20260812")
    ap.add_argument("--frames", type=int, default=40)
    args = ap.parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    for _ in range(15):
        pipe.wait_for_frames(5000)
    acc = []
    intr = None
    for _ in range(args.frames):
        fr = align.process(pipe.wait_for_frames(5000))
        d, c = fr.get_depth_frame(), fr.get_color_frame()
        if not d or not c:
            continue
        if intr is None:
            intr = c.get_profile().as_video_stream_profile().get_intrinsics()
        acc.append(np.asanyarray(d.get_data()).astype(np.float32))
    pipe.stop()
    med = np.stack(acc)
    med[med == 0] = np.nan
    med = np.nanmedian(med, axis=0) * scale

    def p3(uv, win=9):
        u, v = uv
        patch = med[max(0, v - win):v + win, max(0, u - win):u + win]
        vals = patch[np.isfinite(patch)]
        if not vals.size:
            return None
        z = float(np.median(vals))
        return np.array(rs.rs2_deproject_pixel_to_point(
            intr, [float(u), float(v)], z))

    rims = [p for p in (p3(uv) for uv in RIM_PIXELS) if p is not None]
    A = np.c_[[r[:2] for r in rims], np.ones(len(rims))]
    zs = np.array([r[2] for r in rims])
    coef, *_ = np.linalg.lstsq(A, zs, rcond=None)

    def surface_z(x, y):
        return coef[0] * x + coef[1] * y + coef[2]

    report = {"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                             time.gmtime()),
              "surface_plane_coef": [round(float(c), 5) for c in coef],
              "rim_fit_residual_m": round(float(np.std(A @ coef - zs)), 3),
              "zones": {}}
    for zone, pixels in ZONES.items():
        ests = []
        for uv in pixels:
            p = p3(uv)
            if p is None:
                continue
            apparent = p[2] - surface_z(p[0], p[1])
            if apparent > 0.05:
                ests.append(apparent * N_REFRACT)
        report["zones"][zone] = {
            "n_valid": len(ests),
            "corrected_depth_m": (round(float(np.median(ests)), 3)
                                  if ests else None),
            "spread_m": (round(float(np.ptp(ests)), 3) if len(ests) > 1
                         else None),
        }
    out = os.path.join(args.out, "pool_depth_crosscheck.json")
    with open(out, "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
