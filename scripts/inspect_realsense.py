"""Read-only inventory of the connected Intel RealSense camera.

Enumerates device info, supported stream profiles, intrinsics/extrinsics,
depth scale, default sensor options, and measures the actually achievable
RGB frame rate at the highest resolution. Saves a machine-readable JSON
inventory and sample RGB frames.

This script only reads and streams; it never changes persistent camera
settings (no option writes, no firmware operations).

Usage:
    python scripts/inspect_realsense.py [--out visualizations/realsense_inventory]
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import pyrealsense2 as rs

try:
    import cv2
except ImportError:
    cv2 = None


def intrinsics_to_dict(intr):
    return {
        "width": intr.width,
        "height": intr.height,
        "fx": intr.fx,
        "fy": intr.fy,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "distortion_model": str(intr.model),
        "distortion_coeffs": list(intr.coeffs),
    }


def profile_key(p):
    v = p.as_video_stream_profile()
    return (str(p.stream_type()), p.stream_index(), v.width(), v.height(), p.fps(), str(p.format()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="visualizations/realsense_inventory")
    parser.add_argument("--fps-test-seconds", type=float, default=6.0)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("No RealSense device found.")
        sys.exit(1)

    inventory = {"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "devices": []}

    for dev in devices:
        info = {}
        for name in (
            "name", "serial_number", "firmware_version", "recommended_firmware_version",
            "physical_port", "usb_type_descriptor", "product_id", "product_line",
        ):
            field = getattr(rs.camera_info, name, None)
            if field is not None and dev.supports(field):
                info[name] = dev.get_info(field)

        sensors_out = []
        for sensor in dev.query_sensors():
            s_entry = {"name": sensor.get_info(rs.camera_info.name), "profiles": [], "options": {}}

            seen = set()
            for p in sensor.get_stream_profiles():
                if not p.is_video_stream_profile():
                    continue
                k = profile_key(p)
                if k in seen:
                    continue
                seen.add(k)
                stype, sidx, w, h, fps, fmt = k
                s_entry["profiles"].append(
                    {"stream": stype, "index": sidx, "width": w, "height": h, "fps": fps, "format": fmt}
                )

            for opt in sensor.get_supported_options():
                try:
                    rng = sensor.get_option_range(opt)
                    s_entry["options"][str(opt)] = {
                        "current": sensor.get_option(opt),
                        "default": rng.default,
                        "min": rng.min,
                        "max": rng.max,
                    }
                except Exception:
                    pass

            if sensor.is_depth_sensor():
                s_entry["depth_scale_m"] = sensor.as_depth_sensor().get_depth_scale()

            sensors_out.append(s_entry)

        info["sensors"] = sensors_out
        inventory["devices"].append(info)

    # --- Stream test: highest-resolution RGB + default depth, measure real FPS,
    # record intrinsics/extrinsics of the actually-streamed profiles.
    stream_report = {}
    pipeline = rs.pipeline(ctx)
    config = rs.config()
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        stream_report["error_1080p"] = str(exc)
        profile = None

    if profile is not None:
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        stream_report["color_intrinsics"] = intrinsics_to_dict(color_profile.get_intrinsics())
        stream_report["depth_intrinsics"] = intrinsics_to_dict(depth_profile.get_intrinsics())
        ext = depth_profile.get_extrinsics_to(color_profile)
        stream_report["depth_to_color_extrinsics"] = {
            "rotation_row_major": list(ext.rotation),
            "translation_m": list(ext.translation),
        }

        # warmup
        for _ in range(15):
            pipeline.wait_for_frames(5000)

        n = 0
        ts_domains = set()
        hw_ts = []
        t0 = time.perf_counter()
        saved = False
        while time.perf_counter() - t0 < args.fps_test_seconds:
            frames = pipeline.wait_for_frames(5000)
            color = frames.get_color_frame()
            if not color:
                continue
            n += 1
            ts_domains.add(str(color.get_frame_timestamp_domain()))
            hw_ts.append(color.get_timestamp())
            if not saved and n >= 5:
                img = np.asanyarray(color.get_data())
                if cv2 is not None:
                    cv2.imwrite(os.path.join(args.out, "sample_rgb_1920x1080.png"), img)
                depth = frames.get_depth_frame()
                if depth is not None and cv2 is not None:
                    d = np.asanyarray(depth.get_data())
                    d8 = cv2.convertScaleAbs(d, alpha=0.03)
                    cv2.imwrite(os.path.join(args.out, "sample_depth_848x480_colorized.png"),
                                cv2.applyColorMap(d8, cv2.COLORMAP_JET))
                saved = True
        elapsed = time.perf_counter() - t0
        pipeline.stop()

        stream_report["rgb_1080p_requested_fps"] = 30
        stream_report["rgb_1080p_measured_fps"] = round(n / elapsed, 2)
        stream_report["timestamp_domains"] = sorted(ts_domains)
        if len(hw_ts) > 2:
            dts = np.diff(hw_ts)
            stream_report["frame_interval_ms"] = {
                "mean": float(np.mean(dts)),
                "std": float(np.std(dts)),
                "p95": float(np.percentile(dts, 95)),
            }

    inventory["stream_test"] = stream_report

    out_json = os.path.join(args.out, "realsense_inventory.json")
    with open(out_json, "w") as f:
        json.dump(inventory, f, indent=2)
    print(f"Inventory written to {out_json}")
    print(json.dumps({k: v for k, v in stream_report.items() if "intrinsics" not in k}, indent=2))


if __name__ == "__main__":
    main()
