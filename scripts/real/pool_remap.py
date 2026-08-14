"""Re-measure the camera -> pool transform after the camera has moved.

WHY: config/real_pool/pool_geometry.yaml holds a rigid camera->pool
transform surveyed on 2026-08-13. The overhead RealSense was physically
moved TWICE on 2026-08-14, so every number in that file that is expressed
in camera coordinates (the transform, the anchor pixel, the surface
plane) is stale, and with it the metric meaning of every overhead
ground-truth measurement. This script rebuilds the transform from a
single capture, in about a minute, so a camera bump costs a minute
instead of a survey.

Pool frame produced here (the frame the missions and the analysis use):

    origin  = the anchor attachment point on the rod
    +Y      = along the rod, cross-pool, toward the FAR rim
    +X      = perpendicular to the rod in the surface plane, the
              approach direction (along-pool, toward the right end)
    +Z      = up (surface normal, pointing back at the camera)

    p_pool = R @ (p_cam - t)      R rows = pool axes in camera coords

Same convention as pool_geometry.yaml, so downstream code does not have
to learn a second one. The rod is found automatically (long, bright,
near-vertical structure in the image); the anchor attachment PIXEL is a
parameter because nothing in the image marks it - default 1069 635, the
value avoid_mission.py has been using since the second camera move.

READ-ONLY with respect to the vehicle: this opens the RealSense only. It
does NOT open a MAVLink connection and does not command anything. The
overhead camera stays validation/safety only, never a planner input.

Usage (tomorrow, with the camera live):
    python scripts/real/pool_remap.py
    python scripts/real/pool_remap.py --anchor-px 1069 635 --frames 30

Usage now, with no hardware at all (geometry + rod detector exercised on
a synthetic scene, nothing written to config/):
    python scripts/real/pool_remap.py --self-test
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import pyrealsense2 as rs
except ImportError:               # importable on a machine with no SDK
    rs = None

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
CFG_DIR = os.path.join(ROOT, "config", "real_pool")
VIS_DIR = os.path.join(ROOT, "visualizations")

EXPOSURE = 5.0                 # manual exposure validated 2026-08-14
COLOR_WH = (1920, 1080)
DEPTH_WH = (848, 480)
ANCHOR_PX = (1069.0, 635.0)    # anchor attachment, second camera pose

# A float64 temporal median over 30 full 1080p colour frames would
# allocate ~1.5 GB; the colour median only needs enough frames to reject
# ripple glint, so it is capped and taken one channel at a time.
MEDIAN_COLOR_FRAMES = 9

AXIS_LEN_M = 0.5               # length of the drawn axes
ROD_SAMPLE_T = (0.15, 0.85)    # where along the rod depth is sampled
PLANE_BAND_M = 0.35            # depth band around the rod kept for the
                               # surface-plane fit


# ---------------------------------------------------------------------
# pure geometry (no hardware, no OpenCV) - unit-checkable
# ---------------------------------------------------------------------

def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        raise ValueError("cannot normalise a zero-length vector")
    return v / n


def intr_to_dict(intr) -> dict:
    """RealSense intrinsics -> plain dict (also accepts a dict)."""
    if isinstance(intr, dict):
        return dict(intr)
    return {"width": int(intr.width), "height": int(intr.height),
            "fx": float(intr.fx), "fy": float(intr.fy),
            "ppx": float(intr.ppx), "ppy": float(intr.ppy),
            "model": str(intr.model), "coeffs": list(intr.coeffs)}


def deproject(intr, uv, z) -> np.ndarray:
    """Pixel + depth -> 3D point in camera coordinates (metres).

    Uses the SDK (which applies the distortion model) when a live
    intrinsics object is available, and the pinhole model otherwise, so
    the same code path runs in the self-test."""
    if rs is not None and isinstance(intr, rs.intrinsics):
        p = rs.rs2_deproject_pixel_to_point(
            intr, [float(uv[0]), float(uv[1])], float(z))
        return np.asarray(p, dtype=float)
    d = intr_to_dict(intr)
    x = (float(uv[0]) - d["ppx"]) / d["fx"] * float(z)
    y = (float(uv[1]) - d["ppy"]) / d["fy"] * float(z)
    return np.array([x, y, float(z)])


def project(intr, p) -> tuple[float, float]:
    """3D camera point -> pixel. Pinhole: used for the axis overlay and
    for the handedness checks, where a sub-pixel distortion term is
    irrelevant."""
    d = intr_to_dict(intr)
    p = np.asarray(p, dtype=float)
    z = max(1e-6, float(p[2]))
    return (d["ppx"] + d["fx"] * float(p[0]) / z,
            d["ppy"] + d["fy"] * float(p[1]) / z)


def fit_plane(points, trims: int = 2, sigma: float = 2.5) -> dict:
    """Robust least-squares plane through 3D points.

    Trimmed refit rather than full RANSAC: the selected points are
    already depth-banded around the rod, so the outliers are a thin tail
    (ripple glint, the rod itself), not a second surface."""
    pts = np.asarray(points, dtype=float)
    if pts.shape[0] < 16:
        raise ValueError(f"plane fit needs >= 16 points, got {pts.shape[0]}")
    keep = np.ones(pts.shape[0], dtype=bool)
    normal = centroid = None
    rms = float("nan")
    for _ in range(trims + 1):
        sel = pts[keep]
        centroid = sel.mean(axis=0)
        # Smallest singular direction = plane normal.
        _, _, vt = np.linalg.svd(sel - centroid, full_matrices=False)
        normal = vt[-1]
        resid = (pts - centroid) @ normal
        rms = float(np.sqrt(np.mean(resid[keep] ** 2)))
        if rms < 1e-9:
            break
        keep = np.abs(resid) < sigma * rms
        if keep.sum() < 16:
            break
    return {"normal": unit(normal), "centroid": np.asarray(centroid),
            "rms_m": rms, "n_points": int(pts.shape[0]),
            "n_inliers": int(keep.sum())}


def build_pool_frame(origin_cam, rod_dir_cam, up_cam) -> dict:
    """Orthonormal pool frame from the origin, the rod and the vertical.

    +Y is the rod direction with its out-of-plane component removed (the
    rod is nominally horizontal, but the depth samples on a thin bar are
    noisy and there is no reason to let that noise tilt the frame).
    +X = Y x Z is then forced by right-handedness; for a downward-looking
    camera with +Y toward the top of the image that lands on
    image-right = along-pool, which is the approach direction."""
    z = unit(up_cam)
    y_raw = np.asarray(rod_dir_cam, dtype=float)
    y_perp = y_raw - float(np.dot(y_raw, z)) * z
    if np.linalg.norm(y_perp) < 1e-6:
        raise ValueError("the rod is parallel to the surface normal; "
                         "the rod detection or the plane fit is wrong")
    y = unit(y_perp)
    x = unit(np.cross(y, z))
    R = np.vstack([x, y, z])
    return {"R": R, "t": np.asarray(origin_cam, dtype=float),
            "rod_out_of_plane_deg": float(math.degrees(math.asin(
                max(-1.0, min(1.0, float(np.dot(unit(y_raw), z))))))),
            "det_R": float(np.linalg.det(R)),
            "orthonormality_err": float(np.max(np.abs(
                R @ R.T - np.eye(3))))}


def camera_to_pool(frame: dict, p_cam) -> np.ndarray:
    """p_pool = R @ (p_cam - t). `frame` is a loaded pool_frame JSON or
    the dict returned by build_pool_frame."""
    R = np.asarray(frame["R"] if "R" in frame
                   else [frame["R_rows_pool_axes_in_camera"][k]
                         for k in ("x_approach", "y_along_rod", "z_up")],
                   dtype=float)
    t = np.asarray(frame["t"] if "t" in frame
                   else frame["t_pool_origin_in_camera"], dtype=float)
    return R @ (np.asarray(p_cam, dtype=float) - t)


def load_pool_frame(path: str) -> dict:
    with open(path) as f:
        return json.load(f)


def latest_pool_frame(cfg_dir: str = CFG_DIR) -> tuple[str, dict] | None:
    """Newest pool_frame_<date>.json, or None if the camera has never
    been remapped. Consumers (avoid_mission, the analysis scripts) use
    this to pick up the measured scale instead of a hard-coded one."""
    import glob
    paths = sorted(glob.glob(os.path.join(cfg_dir, "pool_frame_*.json")))
    for path in reversed(paths):
        try:
            doc = load_pool_frame(path)
        except (OSError, ValueError):
            continue
        if doc.get("valid"):
            return path, doc
    return None


# ---------------------------------------------------------------------
# rod detection (image only - runs on any array, hence testable)
# ---------------------------------------------------------------------

def _fit_segment(pts) -> dict:
    """Total-least-squares line through a point set, with the extent
    along and across the line. Deliberately not minAreaRect: its angle
    convention has changed between OpenCV versions."""
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
    vx, vy, x0, y0 = [float(v) for v in
                      cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()]
    d = np.array([vx, vy])
    if d[1] < 0:               # orient every segment downward in v
        d = -d
    n = np.array([-d[1], d[0]])
    rel = pts - np.array([x0, y0])
    s = rel @ d
    w = rel @ n
    p_lo = np.array([x0, y0]) + s.min() * d
    p_hi = np.array([x0, y0]) + s.max() * d
    return {"dir": d, "point": np.array([x0, y0]),
            "length_px": float(s.max() - s.min()),
            "width_px": float(max(1.0, w.max() - w.min())),
            "p_top": p_lo, "p_bottom": p_hi,
            "tilt_deg": float(math.degrees(math.atan2(abs(d[0]), abs(d[1]))))}


def _point_line_dist(seg: dict, uv) -> float:
    rel = np.asarray(uv, dtype=float) - seg["point"]
    n = np.array([-seg["dir"][1], seg["dir"][0]])
    return float(abs(rel @ n))


def _merge_collinear(segs, pts_by_seg, max_angle_deg=8.0,
                     max_offset_px=25.0):
    """The anchor suspension and glare break the rod into fragments;
    fragments that lie on the same line are one structure."""
    used = [False] * len(segs)
    merged = []
    for i, si in enumerate(segs):
        if used[i]:
            continue
        group = [i]
        used[i] = True
        for j in range(i + 1, len(segs)):
            if used[j]:
                continue
            sj = segs[j]
            ang = abs(si["tilt_deg"] - sj["tilt_deg"])
            off = _point_line_dist(si, sj["point"])
            if ang < max_angle_deg and off < max_offset_px:
                group.append(j)
                used[j] = True
        pts = np.vstack([pts_by_seg[k] for k in group])
        merged.append((_fit_segment(pts), pts))
    return merged


def find_rod(gray, anchor_px=None, min_len_frac=0.35, max_tilt_deg=30.0,
             min_elongation=6.0, anchor_gate_px=220.0,
             percentiles=(0.5, 1.0, 2.0, 5.0)) -> dict:
    """Locate the transverse rod: the long, bright, near-vertical
    structure crossing the pool.

    Adaptive brightness percentile, in the spirit of the darkness
    percentile in overhead_track.py: sun glint on the water is brighter
    than the rod in places, so a single fixed threshold either drowns in
    glint or loses the rod. Tighten until an elongated, near-vertical,
    long-enough structure appears.

    Returns the fitted segment plus the mask that produced it."""
    if cv2 is None:
        raise RuntimeError("OpenCV is required for rod detection")
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    min_len = min_len_frac * h
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    vker = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 25))
    best = None
    tried = []
    for pct in percentiles:
        thr = float(np.percentile(blur, 100.0 - pct))
        mask = (blur >= thr).astype(np.uint8) * 255
        # Close ALONG the rod to bridge the fragments the anchor
        # suspension cuts out, then open to drop glint speckle.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, vker)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_NONE)
        segs, pts_by_seg = [], []
        for c in cnts:
            if len(c) < 8:
                continue
            pts = c.reshape(-1, 2).astype(np.float32)
            seg = _fit_segment(pts)
            if seg["length_px"] < 0.25 * min_len:
                continue
            segs.append(seg)
            pts_by_seg.append(pts)
        cands = []
        for seg, pts in _merge_collinear(segs, pts_by_seg):
            if seg["length_px"] < min_len:
                continue
            if seg["tilt_deg"] > max_tilt_deg:
                continue
            if seg["length_px"] / seg["width_px"] < min_elongation:
                continue
            seg["anchor_dist_px"] = (
                None if anchor_px is None
                else _point_line_dist(seg, anchor_px))
            seg["threshold"] = thr
            seg["percentile"] = pct
            seg["n_points"] = int(pts.shape[0])
            cands.append(seg)
        tried.append({"percentile": pct, "threshold": round(thr, 1),
                      "candidates": len(cands)})
        if not cands:
            continue
        gated = [c for c in cands
                 if c["anchor_dist_px"] is None
                 or c["anchor_dist_px"] <= anchor_gate_px]
        pool = gated or cands
        best = max(pool, key=lambda c: c["length_px"])
        best["gated_by_anchor"] = bool(gated)
        best["mask"] = mask
        break
    if best is None:
        raise RuntimeError(
            "rod not found: no bright, elongated, near-vertical "
            f"structure longer than {min_len:.0f} px. Tried {tried}. "
            "Check the exposure and --min-len-frac / --max-tilt-deg.")
    best["attempts"] = tried
    return best


# ---------------------------------------------------------------------
# depth sampling
# ---------------------------------------------------------------------

def sample_depth(depth_m, uv, win=9, max_win=41) -> float | None:
    """Median finite depth in a window, widening until something is
    there. Thin bright structures return sparse stereo, so a fixed
    window on the rod frequently comes back empty."""
    h, w = depth_m.shape[:2]
    u, v = int(round(uv[0])), int(round(uv[1]))
    while win <= max_win:
        patch = depth_m[max(0, v - win):v + win + 1,
                        max(0, u - win):u + win + 1]
        vals = patch[np.isfinite(patch) & (patch > 0)]
        if vals.size >= 8:
            return float(np.median(vals))
        win *= 2
    return None


def surface_points(depth_m, intr, ref_depth, band=PLANE_BAND_M, step=16):
    """Deprojected points within `band` of the rod depth.

    Selecting a depth band around the ROD is what makes this fit
    well-posed: the rod spans the pool at rim level, so returns at the
    same range are the surface and the rims, while the pool bottom sits
    ~1 m further away (and is refraction-biased, hence useless for a
    vertical)."""
    h, w = depth_m.shape[:2]
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth_m[vs, us]
    ok = np.isfinite(z) & (z > 0) & (np.abs(z - ref_depth) < band)
    us, vs, z = us[ok], vs[ok], z[ok]
    d = intr_to_dict(intr)
    x = (us - d["ppx"]) / d["fx"] * z
    y = (vs - d["ppy"]) / d["fy"] * z
    return np.stack([x, y, z], axis=1)


# ---------------------------------------------------------------------
# the analysis proper
# ---------------------------------------------------------------------

def analyse(color, depth_m, intr, anchor_px=ANCHOR_PX, **rod_kw) -> dict:
    """Everything between "we have a frame" and "we have a transform".

    Separated from the capture so it can be exercised on a synthetic
    scene with no camera attached (--self-test)."""
    warnings = []
    rod = find_rod(color, anchor_px=anchor_px, **rod_kw)
    p_top, p_bot = rod["p_top"], rod["p_bottom"]
    span = p_bot - p_top

    # Depth on the rod, sampled inside the ends: the extreme endpoints
    # mix the rod with whatever is behind it.
    uv_lo = p_top + ROD_SAMPLE_T[0] * span
    uv_hi = p_top + ROD_SAMPLE_T[1] * span
    z_lo, z_hi = sample_depth(depth_m, uv_lo), sample_depth(depth_m, uv_hi)
    if z_lo is None or z_hi is None:
        raise RuntimeError(
            "no valid depth on the rod at "
            f"{uv_lo.round(0).tolist()} / {uv_hi.round(0).tolist()}; "
            "the stereo pair sees no texture there - move the camera "
            "or raise --frames")
    P_lo = deproject(intr, uv_lo, z_lo)      # toward the image top
    P_hi = deproject(intr, uv_hi, z_hi)

    # +Y = along the rod toward the FAR rim = toward the image top.
    rod_dir = unit(P_lo - P_hi)
    rod_sampled_span_m = float(np.linalg.norm(P_lo - P_hi))
    frac = ROD_SAMPLE_T[1] - ROD_SAMPLE_T[0]
    rod_len_m = rod_sampled_span_m / frac

    # Origin: the anchor attachment, snapped onto the rod line. The
    # origin must lie ON the rod or the frame is inconsistent; the snap
    # distance is a direct check that the supplied pixel still matches
    # this camera pose.
    n_hat = np.array([-rod["dir"][1], rod["dir"][0]])
    rel = np.asarray(anchor_px, dtype=float) - rod["point"]
    anchor_snap = np.asarray(anchor_px, dtype=float) - (rel @ n_hat) * n_hat
    snap_px = float(abs(rel @ n_hat))
    if snap_px > 40.0:
        warnings.append(
            f"the anchor pixel {list(anchor_px)} is {snap_px:.0f} px off "
            "the detected rod - confirm it against the reference image "
            "before trusting the origin")

    z_anchor = sample_depth(depth_m, anchor_snap)
    anchor_depth_source = "direct"
    if z_anchor is None:
        # Interpolate the depth along the rod instead of failing: the
        # rod is a straight rigid bar, so a linear interpolation between
        # two good samples is better than no origin at all.
        s_lo = float((uv_lo - rod["point"]) @ rod["dir"])
        s_hi = float((uv_hi - rod["point"]) @ rod["dir"])
        s_a = float((anchor_snap - rod["point"]) @ rod["dir"])
        w = 0.5 if abs(s_hi - s_lo) < 1e-6 else (s_a - s_lo) / (s_hi - s_lo)
        z_anchor = float(z_lo + w * (z_hi - z_lo))
        anchor_depth_source = "interpolated_along_rod"
        warnings.append("no direct depth at the anchor pixel; the origin "
                        "depth was interpolated along the rod")
    origin = deproject(intr, anchor_snap, z_anchor)

    # Vertical from the surface plane.
    ref_depth = 0.5 * (z_lo + z_hi)
    pts = surface_points(depth_m, intr, ref_depth)
    z_source = "surface_plane_fit"
    plane = None
    try:
        plane = fit_plane(pts)
        up = plane["normal"]
        if float(np.dot(up, origin)) > 0:
            up = -up             # orient back toward the camera = up
        if plane["rms_m"] > 0.06:
            warnings.append(
                f"surface plane fit rms {plane['rms_m']*100:.1f} cm - "
                "ripple or a mixed surface/bottom selection; the "
                "vertical is the weakest axis of this frame")
    except (ValueError, np.linalg.LinAlgError) as exc:
        # Honest degradation: the camera's own axis is not the pool
        # vertical (the mount is tilted ~17 deg), so this frame is
        # flagged invalid rather than quietly wrong.
        up = np.array([0.0, 0.0, -1.0])
        z_source = "camera_axis_fallback"
        warnings.append(f"surface plane fit failed ({exc}); +Z fell back "
                        "to the camera axis and IS NOT the pool vertical "
                        "- do not use this frame for metric ground truth")

    frame = build_pool_frame(origin, rod_dir, up)
    R = frame["R"]

    # Handedness / convention check, in pixels: +X must run toward the
    # right of the image (along-pool, the approach direction).
    u0, v0 = project(intr, origin)
    ux, vx = project(intr, origin + AXIS_LEN_M * R[0])
    uy, vy = project(intr, origin + AXIS_LEN_M * R[1])
    if ux <= u0:
        warnings.append(
            "+X points toward the LEFT of the image: the camera pose or "
            "the rod orientation is not what this frame assumes - check "
            "the reference image before using +X as the approach axis")
    if vy >= v0:
        warnings.append("+Y does not point toward the top of the image "
                        "(the far rim); check the reference image")

    # Scale. The camera is oblique, so the pixel/metre scale is not one
    # number: report the scale measured ALONG the rod and the local
    # pinhole scale at the anchor plane, and let the caller see the
    # difference rather than hiding it.
    px_span = float(np.linalg.norm(uv_hi - uv_lo))
    px_per_m_rod = px_span / max(1e-6, rod_sampled_span_m)
    d = intr_to_dict(intr)
    px_per_m_anchor = 0.5 * (d["fx"] + d["fy"]) / max(1e-6, z_anchor)

    return {
        "warnings": warnings,
        "rod": rod, "plane": plane, "z_source": z_source,
        "anchor_depth_source": anchor_depth_source,
        "origin_cam": origin, "rod_dir_cam": rod_dir, "up_cam": up,
        "frame": frame,
        "anchor_px_given": [float(anchor_px[0]), float(anchor_px[1])],
        "anchor_px_on_rod": anchor_snap,
        "anchor_snap_px": snap_px,
        "anchor_depth_m": float(z_anchor),
        "rod_sample_uv": (uv_lo, uv_hi),
        "rod_sample_depth_m": (float(z_lo), float(z_hi)),
        "rod_sampled_span_m": rod_sampled_span_m,
        "rod_length_m": rod_len_m,
        "px_per_m_rod": float(px_per_m_rod),
        "px_per_m_anchor_plane": float(px_per_m_anchor),
        "intrinsics": d,
    }


# ---------------------------------------------------------------------
# rendering + serialisation
# ---------------------------------------------------------------------

def draw_reference(color, res) -> np.ndarray:
    """Reference image: the rod, the origin and the three pool axes.

    This is the artefact a human checks before believing the JSON."""
    vis = color.copy()
    intr = res["intrinsics"]
    R, origin = res["frame"]["R"], res["origin_cam"]
    p_top = tuple(int(v) for v in res["rod"]["p_top"])
    p_bot = tuple(int(v) for v in res["rod"]["p_bottom"])
    cv2.line(vis, p_top, p_bot, (0, 255, 255), 2)
    for p in (p_top, p_bot):
        cv2.circle(vis, p, 10, (0, 255, 255), 2)
    cv2.putText(vis, "ROD", (p_top[0] + 14, p_top[1] + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    o = tuple(int(v) for v in project(intr, origin))
    axes = [(R[0], (0, 0, 255), "+X approach"),
            (R[1], (0, 255, 0), "+Y rod / far rim"),
            (R[2], (255, 128, 0), "+Z up")]
    for axis, colour, label in axes:
        tip = project(intr, origin + AXIS_LEN_M * axis)
        tip = (int(tip[0]), int(tip[1]))
        cv2.arrowedLine(vis, o, tip, (0, 0, 0), 7, tipLength=0.18)
        cv2.arrowedLine(vis, o, tip, colour, 4, tipLength=0.18)
        cv2.putText(vis, label, (tip[0] + 8, tip[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(vis, label, (tip[0] + 8, tip[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    cv2.drawMarker(vis, o, (255, 0, 255), cv2.MARKER_CROSS, 34, 3)
    cv2.putText(vis, "ORIGIN (anchor attachment)", (o[0] + 16, o[1] + 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

    lines = [f"axes drawn at {AXIS_LEN_M:.2f} m",
             f"rod {res['rod_length_m']:.2f} m  "
             f"anchor depth {res['anchor_depth_m']:.2f} m",
             f"scale {res['px_per_m_rod']:.0f} px/m (rod)  "
             f"{res['px_per_m_anchor_plane']:.0f} px/m (anchor plane)",
             f"+Z from {res['z_source']}"]
    if res["plane"] is not None:
        lines[-1] += f"  rms {res['plane']['rms_m']*1000:.0f} mm"
    y = 40
    for text in lines:
        cv2.putText(vis, text, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 0), 5)
        cv2.putText(vis, text, (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)
        y += 36
    for wmsg in res["warnings"]:
        cv2.putText(vis, "WARNING: " + wmsg[:78], (24, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 5)
        cv2.putText(vis, "WARNING: " + wmsg[:78], (24, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        y += 32
    return vis


def to_json(res, meta: dict, image_paths: dict) -> dict:
    R, t = res["frame"]["R"], res["frame"]["t"]
    plane = res["plane"]
    return {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "produced_by": "scripts/real/pool_remap.py",
        "supersedes": "config/real_pool/pool_geometry.yaml "
                      "(camera-frame entries only; the pool dimensions "
                      "in that file are unaffected by a camera move)",
        "valid": res["z_source"] != "camera_axis_fallback",
        "warnings": res["warnings"],
        "frame_definition": {
            "origin": "anchor attachment point on the rod",
            "x_approach": "perpendicular to the rod in the surface "
                          "plane, along-pool approach direction",
            "y_along_rod": "along the rod, cross-pool, toward the far rim",
            "z_up": "surface normal, pointing back at the camera",
            "usage": "p_pool = R @ (p_cam - t)"},
        "R_rows_pool_axes_in_camera": {
            "x_approach": [round(float(v), 6) for v in R[0]],
            "y_along_rod": [round(float(v), 6) for v in R[1]],
            "z_up": [round(float(v), 6) for v in R[2]]},
        "t_pool_origin_in_camera": [round(float(v), 5) for v in t],
        "frame_checks": {
            "det_R": round(res["frame"]["det_R"], 9),
            "orthonormality_err": round(
                res["frame"]["orthonormality_err"], 9),
            "rod_out_of_plane_deg": round(
                res["frame"]["rod_out_of_plane_deg"], 3),
            "z_source": res["z_source"],
            "surface_plane_rms_m": (None if plane is None
                                    else round(plane["rms_m"], 4)),
            "surface_plane_inliers": (None if plane is None
                                      else plane["n_inliers"])},
        "anchor": {
            "pixel_given": res["anchor_px_given"],
            "pixel_on_rod": [round(float(v), 1)
                             for v in res["anchor_px_on_rod"]],
            "snap_distance_px": round(res["anchor_snap_px"], 1),
            "depth_m": round(res["anchor_depth_m"], 4),
            "depth_source": res["anchor_depth_source"],
            "xyz_camera_m": [round(float(v), 4)
                             for v in res["origin_cam"]]},
        "rod": {
            "endpoint_top_px": [round(float(v), 1)
                                for v in res["rod"]["p_top"]],
            "endpoint_bottom_px": [round(float(v), 1)
                                   for v in res["rod"]["p_bottom"]],
            "sample_px": [[round(float(v), 1) for v in uv]
                          for uv in res["rod_sample_uv"]],
            "sample_depth_m": [round(v, 4)
                               for v in res["rod_sample_depth_m"]],
            "sampled_span_m": round(res["rod_sampled_span_m"], 4),
            "estimated_length_m": round(res["rod_length_m"], 4),
            "tilt_from_image_vertical_deg": round(
                res["rod"]["tilt_deg"], 2),
            "width_px": round(res["rod"]["width_px"], 1),
            "threshold_percentile": res["rod"].get("percentile"),
            "gated_by_anchor_pixel": res["rod"].get("gated_by_anchor")},
        "scale": {
            "px_per_m_rod": round(res["px_per_m_rod"], 1),
            "px_per_m_anchor_plane": round(
                res["px_per_m_anchor_plane"], 1),
            "note": "the camera is oblique, so a single px/m is an "
                    "approximation; use the transform for anything "
                    "metric and these scales only for quick reads"},
        "intrinsics": res["intrinsics"],
        "capture": meta,
        "images": image_paths,
    }


# ---------------------------------------------------------------------
# hardware capture
# ---------------------------------------------------------------------

def capture(frames: int = 30, exposure: float = EXPOSURE,
            warmup: int = 15) -> dict:
    """Median-filtered aligned depth + colour from the overhead D435.

    Read-only with respect to the vehicle. Auto-exposure is restored on
    the way out so the next script starts from a known state."""
    if rs is None:
        raise RuntimeError("pyrealsense2 is not installed")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, COLOR_WH[0], COLOR_WH[1],
                      rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, DEPTH_WH[0], DEPTH_WH[1],
                      rs.format.z16, 30)
    profile = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    dev = profile.get_device()
    colour_sensor = next(s for s in dev.query_sensors()
                         if s.get_info(rs.camera_info.name) == "RGB Camera")
    serial = dev.get_info(rs.camera_info.serial_number)
    try:
        if exposure:
            colour_sensor.set_option(rs.option.enable_auto_exposure, 0)
            colour_sensor.set_option(rs.option.exposure, exposure)
        depth_scale = dev.first_depth_sensor().get_depth_scale()
        for _ in range(warmup):
            pipe.wait_for_frames(5000)
        depth_acc, colour_acc, intr = [], [], None
        for _ in range(frames):
            aligned = align.process(pipe.wait_for_frames(5000))
            d, c = aligned.get_depth_frame(), aligned.get_color_frame()
            if not d or not c:
                continue
            if intr is None:
                intr = c.get_profile().as_video_stream_profile() \
                    .get_intrinsics()
            depth_acc.append(np.asanyarray(d.get_data()).astype(np.float32))
            if len(colour_acc) < MEDIAN_COLOR_FRAMES:
                colour_acc.append(np.asanyarray(c.get_data()).copy())
    finally:
        try:
            colour_sensor.set_option(rs.option.enable_auto_exposure, 1)
        except Exception:
            pass
        pipe.stop()
    if not depth_acc or intr is None:
        raise RuntimeError("no usable frames from the RealSense")

    stack = np.stack(depth_acc)
    stack[stack == 0] = np.nan
    depth_m = np.nanmedian(stack, axis=0) * depth_scale
    valid = float(np.mean(np.isfinite(stack)))

    colour = np.empty_like(colour_acc[0])
    for ch in range(colour.shape[2]):
        band = np.stack([f[:, :, ch] for f in colour_acc])
        colour[:, :, ch] = np.median(band, axis=0).astype(np.uint8)

    return {"color": colour, "depth_m": depth_m, "intr": intr,
            "meta": {"serial": serial, "frames": len(depth_acc),
                     "exposure": exposure, "depth_scale": depth_scale,
                     "depth_valid_fraction": round(valid, 3),
                     "color_median_frames": len(colour_acc),
                     "color_stream": f"{COLOR_WH[0]}x{COLOR_WH[1]}",
                     "depth_stream": f"{DEPTH_WH[0]}x{DEPTH_WH[1]}"}}


# ---------------------------------------------------------------------
# synthetic scene (self-test; no hardware)
# ---------------------------------------------------------------------

def synthetic_scene(tilt_deg=15.0, height_m=3.7, rod_len_m=2.6,
                    yaw_deg=6.0, noise=6.0, seed=0) -> dict:
    """A pool-like scene with a KNOWN answer.

    Builds a tilted surface plane at a known range, projects a bright rod
    lying in that plane, renders the matching depth map, and returns the
    truth axes so the recovered frame can be scored. This is what lets
    the geometry be verified with the camera in its box."""
    rng = np.random.default_rng(seed)
    w, h = COLOR_WH
    intr = {"width": w, "height": h, "fx": 1380.0, "fy": 1380.0,
            "ppx": 960.0, "ppy": 540.0, "model": "synthetic",
            "coeffs": [0.0] * 5}

    # Truth: pool up-vector tilted away from the camera axis.
    tilt = math.radians(tilt_deg)
    up = unit([math.sin(tilt) * 0.6, math.sin(tilt) * 0.8, -math.cos(tilt)])
    # A rod direction in the plane, roughly toward the image top.
    yaw = math.radians(yaw_deg)
    guess = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
    rod_dir = unit(guess - float(np.dot(guess, up)) * up)
    origin = np.array([0.12, -0.18, height_m])

    # Depth map: every pixel ray meets the plane through `origin`.
    us, vs = np.meshgrid(np.arange(w, dtype=np.float32),
                         np.arange(h, dtype=np.float32))
    dx = (us - intr["ppx"]) / intr["fx"]
    dy = (vs - intr["ppy"]) / intr["fy"]
    denom = dx * up[0] + dy * up[1] + up[2]
    denom[np.abs(denom) < 1e-6] = np.nan
    depth = float(np.dot(origin, up)) / denom
    depth[~np.isfinite(depth)] = 0.0
    depth[depth <= 0] = 0.0
    depth += rng.normal(0.0, 0.004, depth.shape)   # stereo noise

    colour = np.full((h, w, 3), 38, np.uint8)
    colour += rng.integers(0, 18, colour.shape, dtype=np.int64) \
        .astype(np.uint8)
    end_a = origin - 0.5 * rod_len_m * rod_dir
    end_b = origin + 0.5 * rod_len_m * rod_dir
    pa = tuple(int(v) for v in project(intr, end_a))
    pb = tuple(int(v) for v in project(intr, end_b))
    if cv2 is not None:
        cv2.line(colour, pa, pb, (238, 240, 244), 13)
        # Sun glint: bright but small and round - the elongation and
        # length filters must reject it.
        for _ in range(60):
            c = (int(rng.integers(60, w - 60)), int(rng.integers(60, h - 60)))
            cv2.circle(colour, c, int(rng.integers(4, 16)),
                       (250, 250, 250), -1)
        # The anchor suspension cuts the rod: the merge step must
        # reassemble the fragments.
        mid = ((pa[0] + pb[0]) // 2, (pa[1] + pb[1]) // 2)
        cv2.circle(colour, mid, 26, (38, 38, 38), -1)
        colour = cv2.GaussianBlur(colour, (3, 3), 0)
    colour = np.clip(colour.astype(np.int16)
                     + rng.normal(0, noise, colour.shape).astype(np.int16),
                     0, 255).astype(np.uint8)

    anchor_px = project(intr, origin)
    truth = build_pool_frame(origin, rod_dir, up)
    return {"color": colour, "depth_m": depth.astype(np.float32),
            "intr": intr, "anchor_px": anchor_px, "truth": truth,
            "rod_ends_px": (pa, pb),
            "meta": {"synthetic": True, "tilt_deg": tilt_deg,
                     "height_m": height_m, "rod_len_m": rod_len_m}}


def self_test(out_dir: str | None = None, verbose: bool = True) -> int:
    """Run the whole geometry path on the synthetic scene and score it
    against the known answer. No camera, no vehicle, nothing written to
    config/."""
    if cv2 is None:
        print("FAIL: OpenCV is required")
        return 1
    scene = synthetic_scene()
    res = analyse(scene["color"], scene["depth_m"], scene["intr"],
                  anchor_px=scene["anchor_px"])
    truth = scene["truth"]
    ang = [math.degrees(math.acos(max(-1.0, min(1.0, float(
        np.dot(res["frame"]["R"][i], truth["R"][i]))))))
        for i in range(3)]
    dorigin = float(np.linalg.norm(res["origin_cam"] - truth["t"]))
    rod_err = abs(res["rod_length_m"] - scene["meta"]["rod_len_m"])

    # Exercise the serialisation too: the JSON is what tomorrow's
    # scripts consume, so a doc that cannot round-trip is as broken as a
    # wrong transform.
    doc = json.loads(json.dumps(to_json(res, {"synthetic": True}, {})))
    p_probe = res["origin_cam"] + 0.75 * res["frame"]["R"][1]
    round_trip = camera_to_pool(doc, p_probe)      # expect (0, 0.75, 0)

    checks = [
        ("rod found", res["rod"]["length_px"] > 300, ""),
        ("json doc is valid", doc["valid"], ""),
        ("json round trip 0.75 m along +Y",
         float(np.linalg.norm(round_trip - [0, 0.75, 0])) < 1e-4,
         f"{np.round(round_trip, 5).tolist()}"),
        ("+X axis error < 1.5 deg", ang[0] < 1.5, f"{ang[0]:.3f} deg"),
        ("+Y axis error < 1.5 deg", ang[1] < 1.5, f"{ang[1]:.3f} deg"),
        ("+Z axis error < 1.5 deg", ang[2] < 1.5, f"{ang[2]:.3f} deg"),
        ("origin error < 30 mm", dorigin < 0.030, f"{dorigin*1000:.1f} mm"),
        ("rod length error < 0.15 m", rod_err < 0.15, f"{rod_err:.3f} m"),
        ("frame right-handed",
         abs(res["frame"]["det_R"] - 1.0) < 1e-6,
         f"det={res['frame']['det_R']:.9f}"),
        ("frame orthonormal",
         res["frame"]["orthonormality_err"] < 1e-9, ""),
        ("+X points image-right",
         not any("LEFT" in w for w in res["warnings"]), ""),
        ("round trip origin -> (0,0,0)",
         float(np.linalg.norm(camera_to_pool(res["frame"],
                                             res["origin_cam"]))) < 1e-9,
         ""),
    ]
    bad = [c for c in checks if not c[1]]
    if verbose:
        print("pool_remap self-test (synthetic scene, no hardware)")
        for name, ok, note in checks:
            print(f"  [{'ok' if ok else 'FAIL'}] {name:34s} {note}")
        print(f"  scale: {res['px_per_m_rod']:.0f} px/m along the rod, "
              f"{res['px_per_m_anchor_plane']:.0f} px/m at the anchor plane")
        for wmsg in res["warnings"]:
            print("  warning:", wmsg)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, "self_test_axes.png")
        cv2.imwrite(path, draw_reference(scene["color"], res))
        cv2.imwrite(os.path.join(out_dir, "self_test_mask.png"),
                    res["rod"]["mask"])
        if verbose:
            print("  reference image ->", path)
    print("SELF-TEST:", "PASS" if not bad else f"FAIL ({len(bad)} checks)")
    return 0 if not bad else 1


# ---------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Rebuild the camera->pool transform from the "
                    "overhead RealSense (read-only capture).")
    ap.add_argument("--anchor-px", nargs=2, type=float, default=ANCHOR_PX,
                    help="anchor attachment pixel in the 1080p colour "
                         "image (default: %(default)s)")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--exposure", type=float, default=EXPOSURE)
    ap.add_argument("--date", default=None,
                    help="tag for the output files (default: today)")
    ap.add_argument("--out-dir", default=CFG_DIR)
    ap.add_argument("--vis-dir", default=None)
    ap.add_argument("--min-len-frac", type=float, default=0.35,
                    help="minimum rod length as a fraction of the "
                         "image height")
    ap.add_argument("--max-tilt-deg", type=float, default=30.0,
                    help="maximum rod tilt from the image vertical")
    ap.add_argument("--anchor-gate-px", type=float, default=220.0,
                    help="reject rod candidates whose line lies further "
                         "than this from the anchor pixel")
    ap.add_argument("--self-test", action="store_true",
                    help="run the geometry on a synthetic scene and "
                         "exit; touches no hardware and writes no config")
    args = ap.parse_args()

    date = args.date or time.strftime("%Y%m%d")
    vis_dir = args.vis_dir or os.path.join(VIS_DIR, f"pool_remap_{date}")

    if args.self_test:
        return self_test(out_dir=vis_dir)

    if cv2 is None:
        print("OpenCV is required")
        return 1
    cap = capture(frames=args.frames, exposure=args.exposure)
    res = analyse(cap["color"], cap["depth_m"], cap["intr"],
                  anchor_px=tuple(args.anchor_px),
                  min_len_frac=args.min_len_frac,
                  max_tilt_deg=args.max_tilt_deg,
                  anchor_gate_px=args.anchor_gate_px)

    os.makedirs(vis_dir, exist_ok=True)
    os.makedirs(args.out_dir, exist_ok=True)
    ref = draw_reference(cap["color"], res)
    paths = {
        "reference": os.path.join(vis_dir, "pool_frame_axes.png"),
        "reference_beside_config": os.path.join(
            args.out_dir, f"pool_frame_{date}.png"),
        "median_rgb": os.path.join(vis_dir, "median_rgb.png"),
        "median_depth": os.path.join(vis_dir, "median_depth.png"),
        "rod_mask": os.path.join(vis_dir, "rod_mask.png")}
    cv2.imwrite(paths["reference"], ref)
    cv2.imwrite(paths["reference_beside_config"], ref)
    cv2.imwrite(paths["median_rgb"], cap["color"])
    d8 = np.clip(np.nan_to_num(cap["depth_m"], nan=0.0) / 8.0 * 255.0,
                 0, 255).astype(np.uint8)
    cv2.imwrite(paths["median_depth"], cv2.applyColorMap(
        d8, cv2.COLORMAP_JET))
    cv2.imwrite(paths["rod_mask"], res["rod"]["mask"])

    rel = {k: os.path.relpath(v, ROOT).replace("\\", "/")
           for k, v in paths.items()}
    doc = to_json(res, cap["meta"], rel)
    out = os.path.join(args.out_dir, f"pool_frame_{date}.json")
    with open(out, "w") as f:
        json.dump(doc, f, indent=2)

    print(json.dumps({k: v for k, v in doc.items()
                      if k not in ("intrinsics", "images")}, indent=2))
    for wmsg in doc["warnings"]:
        print("WARNING:", wmsg)
    print("\npool frame ->", out)
    print("reference image ->", paths["reference"])
    if not doc["valid"]:
        print("!! this frame is NOT metrically valid - see the warnings")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
