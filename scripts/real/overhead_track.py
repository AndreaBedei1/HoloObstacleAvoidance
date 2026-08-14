"""Overhead RealSense tracker for the ROV (external ground truth AND
safety supervisor).

The ROV is a dark, compact object against the bright pool bottom: we
segment it by darkness + size, take the largest plausible blob, and
report pixel centroid, orientation (blob principal axis), and metric
position via the depth stream. Runs either as a one-shot probe or as a
background supervisor that another script can poll.

NEVER used as a runtime input to the obstacle-avoidance planner
(validation/safety only, per the scientific protocol).
"""

from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import pyrealsense2 as rs

try:
    import cv2
except ImportError:
    cv2 = None

EXPOSURE = 5.0          # manual exposure that tames the sun glare
MIN_AREA = 4000         # px, ROV at 1080p (measured 4k-25k)
MAX_AREA = 60000


class OverheadTracker:
    def __init__(self, exposure: float = EXPOSURE):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
        self.profile = self.pipe.start(cfg)
        self.align = rs.align(rs.stream.color)
        dev = self.profile.get_device()
        self.color_sensor = next(
            s for s in dev.query_sensors()
            if s.get_info(rs.camera_info.name) == "RGB Camera")
        if exposure:
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 0)
            self.color_sensor.set_option(rs.option.exposure, exposure)
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.intr = None
        for _ in range(12):
            self.pipe.wait_for_frames(5000)
        self._lock = threading.Lock()
        self._latest = None
        self._stop = threading.Event()
        self._thread = None
        self.last_frame = None

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        try:
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        except Exception:
            pass
        self.pipe.stop()

    @staticmethod
    def _pick(cnts, img, near, gate_px):
        best = None
        for c in cnts:
            a = cv2.contourArea(c)
            if a < MIN_AREA or a > MAX_AREA:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if x <= 2 or y <= 2 or x + w >= img.shape[1] - 2                     or y + h >= img.shape[0] - 2:
                continue
            if not (0.5 < w / float(h) < 2.0):
                continue
            # COMPACTNESS: the ROV is a solid rectangular body (fill
            # ~0.6-0.8 of its bounding box); the shaded band along the
            # pool edge has a similar area but is diffuse and was being
            # picked as the vehicle (2026-08-14).
            if a / float(w * h) < 0.45:
                continue
            M = cv2.moments(c)
            if M["m00"] <= 0:
                continue
            ccx, ccy = M["m10"] / M["m00"], M["m01"] / M["m00"]
            if near is not None and math.hypot(
                    ccx - near[0], ccy - near[1]) > gate_px:
                continue
            if best is None or a > best[0]:
                best = (a, c, (x, y, w, h), (ccx, ccy))
        return best

    # -- detection ------------------------------------------------------
    def detect(self, save_path: str | None = None, near=None,
               gate_px: float = 500.0) -> dict | None:
        """Locate the ROV as the darkest LARGE COMPACT blob in the pool.

        Validated 2026-08-14: the vehicle is by far the darkest object of
        its size in the water (area ~5k-40k px at 1080p); the rim shadow
        and the liner are excluded by area/border tests, and `near`
        gating keeps the lock across frames."""
        frames = self.align.process(self.pipe.wait_for_frames(5000))
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color:
            return None
        if self.intr is None:
            self.intr = color.get_profile().as_video_stream_profile()                 .get_intrinsics()
        img = np.asanyarray(color.get_data())
        self.last_frame = img          # for overlay rendering
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 0)
        # Adaptive darkness percentile: at a loose threshold the vehicle
        # merges with the shaded band along the pool edge (verified
        # 2026-08-14: 2% merged to the top border, 1% isolated the ROV
        # cleanly), so tighten until a NON-border blob appears.
        best = None
        for pct in (1.0, 0.6, 1.5, 2.0):
            thr = float(np.percentile(blur, pct))
            mask = (blur < thr).astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    np.ones((9, 9), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    np.ones((21, 21), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            best = self._pick(cnts, img, near, gate_px)
            if best is None and near is not None:
                # Fast motion can exceed the gate: retry ungated rather
                # than losing the vehicle (observed 2026-08-14: the
                # trajectory stopped while the ROV kept going).
                best = self._pick(cnts, img, None, gate_px)
            if best is not None:
                break
        if best is None:
            if save_path is not None and cv2 is not None:
                cv2.imwrite(save_path, img)
            return {"found": False, "t": time.time(), "threshold": thr}
        area, cnt, (x, y, w, h), (cx, cy) = best
        (_, _), (_, _), angle = cv2.minAreaRect(cnt)
        out = {"found": True, "t": time.time(), "pixel": [cx, cy],
               "bbox": [x, y, w, h], "area_px": area,
               "blob_angle_deg": angle,
               "frac_x": cx / img.shape[1], "frac_y": cy / img.shape[0]}
        if depth:
            d = np.asanyarray(depth.get_data()).astype(np.float32)
            win = d[max(0, int(cy) - 12):int(cy) + 12,
                    max(0, int(cx) - 12):int(cx) + 12]
            win = win[win > 0]
            if win.size:
                z = float(np.median(win)) * self.depth_scale
                p = rs.rs2_deproject_pixel_to_point(self.intr, [cx, cy], z)
                out["xyz_camera_m"] = [round(v, 3) for v in p]
        if save_path is not None and cv2 is not None:
            vis = img.copy()
            cv2.drawContours(vis, [cnt], -1, (0, 0, 255), 3)
            cv2.circle(vis, (int(cx), int(cy)), 8, (0, 255, 255), -1)
            cv2.putText(vis, f"ROV a={area:.0f}", (x, max(30, y - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(save_path, vis)
        return out

    def grab(self):
        """Raw aligned color frame (BGR) + depth in meters."""
        frames = self.align.process(self.pipe.wait_for_frames(5000))
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color:
            return None, None
        if self.intr is None:
            self.intr = color.get_profile().as_video_stream_profile()                 .get_intrinsics()
        img = np.asanyarray(color.get_data()).copy()
        d = (np.asanyarray(depth.get_data()).astype(np.float32)
             * self.depth_scale) if depth else None
        return img, d

    def detect_motion(self, ref_img, cur_img, min_area=600,
                      save_path=None, near=None, gate_px=400):
        """near: (x, y) last known ROV pixel; candidates farther than
        gate_px are rejected (the surface wake and sun ripples produce
        large spurious motion blobs elsewhere in the pool)."""
        """Motion-based ROV detection: the ONLY thing that moves in the
        pool is the vehicle. Far more robust than darkness segmentation
        (the rim shadow and the liner are dark too)."""
        g0 = cv2.GaussianBlur(cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY),
                              (7, 7), 0).astype(np.int16)
        g1 = cv2.GaussianBlur(cv2.cvtColor(cur_img, cv2.COLOR_BGR2GRAY),
                              (7, 7), 0).astype(np.int16)
        diff = np.abs(g1 - g0).astype(np.uint8)
        thr = max(12, int(np.percentile(diff, 99.3)))
        mask = (diff > thr).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                np.ones((25, 25), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in cnts:
            a = cv2.contourArea(c)
            if a < min_area:
                continue
            M = cv2.moments(c)
            if M["m00"] <= 0:
                continue
            ccx, ccy = M["m10"] / M["m00"], M["m01"] / M["m00"]
            if near is not None:
                if math.hypot(ccx - near[0], ccy - near[1]) > gate_px:
                    continue
            score = a
            if best is None or score > best[0]:
                best = (score, c)
        out = {"t": time.time(), "found": best is not None,
               "diff_threshold": thr}
        if best:
            a, c = best
            M = cv2.moments(c)
            cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
            x, y, w, h = cv2.boundingRect(c)
            out.update({"pixel": [cx, cy], "bbox": [x, y, w, h],
                        "area_px": a,
                        "frac_x": cx / cur_img.shape[1],
                        "frac_y": cy / cur_img.shape[0]})
        if save_path and cv2 is not None:
            vis = cur_img.copy()
            if best:
                cv2.drawContours(vis, [best[1]], -1, (0, 0, 255), 3)
                cv2.circle(vis, (int(out["pixel"][0]),
                                 int(out["pixel"][1])), 10,
                           (0, 255, 255), -1)
            cv2.imwrite(save_path, vis)
            cv2.imwrite(save_path.replace(".png", "_mask.png"), mask)
        return out

    # -- supervisor -----------------------------------------------------
    def start_supervisor(self, hz: float = 4.0, log_path: str | None = None):
        self._log = open(log_path, "w") if log_path else None

        def loop():
            while not self._stop.is_set():
                try:
                    det = self.detect()
                except Exception as exc:  # keep supervising
                    det = {"found": False, "error": str(exc),
                           "t": time.time()}
                with self._lock:
                    self._latest = det
                if self._log and det:
                    self._log.write(json.dumps(det) + "\n")
                    self._log.flush()
                time.sleep(1.0 / hz)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def latest(self):
        with self._lock:
            return self._latest


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="visualizations/pool_remap_20260814")
    ap.add_argument("--n", type=int, default=1)
    args = ap.parse_args()
    import os
    os.makedirs(args.out, exist_ok=True)
    t = OverheadTracker()
    try:
        for i in range(args.n):
            det = t.detect(save_path=f"{args.out}/rov_track_{i}.png")
            print(json.dumps(det, indent=1))
            time.sleep(0.5)
    finally:
        t.close()


if __name__ == "__main__":
    main()
