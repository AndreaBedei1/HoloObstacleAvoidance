"""Weight-free anchor detector for the real BlueROV2 camera (Phase 9).

No trained weights exist for the real setup (the lab machine holds the
simulation-era YOLO). The anchor is nevertheless a strong classical
target: a DARK, THIN, VERTICALLY-DOMINANT metal structure against a
brighter, low-frequency background (sunlit pool bottom / water column).

Pipeline (all classical, deterministic, ~15 ms/frame at 1080p):
  1. grayscale + illumination flattening: subtract a heavily blurred
     copy of the image (background model) -> local darkness map
  2. adaptive threshold on the darkness map (percentile-based, so it
     adapts to turbidity/lighting without hand-tuned absolute levels)
  3. morphological closing with a VERTICAL kernel: reconnects the shank
     and the arms of the anchor while suppressing horizontal ripple
  4. connected components; score candidates by
        darkness * verticality * compactness-in-x
     rejecting the surface band (top rows) and image-border blobs
  5. the winner's bounding box is the detection

Output matches the simulation's Obstacle2D convention so the SAME
downstream stack (T2 estimator + Phase-7B qualifier + planners) can
consume it: center_x, center_y, width, height as image fractions.

Usage:
  python scripts/real/anchor_detect.py IMG [IMG ...] --out DIR
  python scripts/real/anchor_detect.py --video V.mp4 --out DIR
"""

from __future__ import annotations

import argparse
import json
import os
import time

import cv2
import numpy as np

# Fraction of the image height ignored at the top: the water surface +
# its bright specular band is not a target and produces dark banding.
SURFACE_BAND = 0.12
MIN_AREA_FRAC = 2.0e-4      # of image area
MAX_AREA_FRAC = 0.25


def detect_anchor(img: np.ndarray, debug: bool = False) -> dict:
    """Detect the suspended anchor.

    Shape prior (v2, 2026-08-14): the anchor is a LONG THIN VERTICAL dark
    structure (the shank, spanning a large fraction of the frame height)
    with a horizontal arm span at its bottom. Dome bubbles — the dominant
    real-world clutter — are small round blobs and are annihilated by a
    tall-thin morphological opening, whereas the shank survives it. v1
    (plain darkness + closing) latched onto bubble clusters whenever the
    anchor's contrast dropped with turbidity/backlight.
    """
    t0 = time.perf_counter()
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    bg = cv2.GaussianBlur(gray, (0, 0), sigmaX=w / 25.0)
    dark = np.clip(bg - gray, 0, None)
    # normalize so the threshold does not depend on absolute contrast
    dmax = float(np.percentile(dark, 99.9)) or 1.0
    dn = np.clip(dark / dmax, 0, 1)

    # --- shank: long vertical dark structure -------------------------
    vert_k = cv2.getStructuringElement(
        cv2.MORPH_RECT, (3, max(31, int(0.12 * h)) | 1))
    shank = cv2.morphologyEx((dn * 255).astype(np.uint8),
                             cv2.MORPH_OPEN, vert_k)
    # 85th pct (not 92nd): in backlit/turbid frames the shank contrast
    # collapses and a tighter threshold drops it entirely.
    thr = max(12.0, float(np.percentile(shank[shank > 0], 85))
              if np.any(shank > 0) else 255.0)
    smask = (shank > thr).astype(np.uint8) * 255
    smask[:int(SURFACE_BAND * h), :] = 0
    smask = cv2.morphologyEx(
        smask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (9, 121)))

    n, labels, stats, _ = cv2.connectedComponentsWithStats(smask, 8)
    best = None
    cand = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        # The real shank spans 0.4-0.65 of the frame height at working
        # distances; 0.12 admitted bubble streaks and rim structures.
        if bh < 0.25 * h:
            continue
        if bw > 0.35 * w:                     # too wide: not a shank
            continue
        comp = labels[y:y + bh, x:x + bw] == i
        mean_dark = float(dark[y:y + bh, x:x + bw][comp].mean())
        vertical = bh / float(bw + 1e-6)
        # STRAIGHT-THIN-LINE test: the shank is a single continuous line,
        # so per row it is a few pixels wide and its centroid barely
        # moves. A bubble cloud has similar height/darkness but wanders
        # and is wide per row -- this is what separates them (the
        # dominant real-world false positive, 2026-08-14).
        rowsum = comp.sum(axis=1)
        covered = float((rowsum > 0).mean())
        if covered < 0.75:
            continue
        idx = np.arange(bw)
        cxs = np.array([(idx * r).sum() / r.sum()
                        for r in comp if r.sum() > 0])
        wander = float(cxs.std())
        row_w = float(np.median(rowsum[rowsum > 0]))
        if row_w > 0.035 * w or wander > 0.02 * w:
            continue
        # ISOLATION test: a real shank has BRIGHTER water on BOTH sides;
        # the shaded band along the pool edge is dark on one side only
        # and was the remaining false positive (2026-08-14).
        pad = max(12, int(0.02 * w))
        xl0, xl1 = max(0, x - 3 * pad), max(1, x - pad)
        xr0, xr1 = min(w - 1, x + bw + pad), min(w, x + bw + 3 * pad)
        band_l = gray[y:y + bh, xl0:xl1]
        band_r = gray[y:y + bh, xr0:xr1]
        core = gray[y:y + bh, x:x + bw]
        if band_l.size == 0 or band_r.size == 0:
            continue
        cm = float(core.mean())
        lm, rm = float(band_l.mean()), float(band_r.mean())
        if not (lm > cm + 4 and rm > cm + 4):
            continue
        straight = 1.0 / (1.0 + wander / 6.0)
        score = (mean_dark / 12.0) * min(vertical, 8.0) / 4.0 * straight
        cand.append({"bbox": [int(x), int(y), int(bw), int(bh)],
                     "area": int(area), "mean_dark": round(mean_dark, 2),
                     "vertical": round(vertical, 2),
                     "row_w": round(row_w, 1),
                     "wander": round(wander, 1),
                     "covered": round(covered, 2),
                     "iso_lr": [round(lm - cm, 1), round(rm - cm, 1)],
                     "score": round(score, 3)})
        if best is None or score > best["score"]:
            best = cand[-1]

    dt = (time.perf_counter() - t0) * 1000.0
    out = {"found": best is not None, "ms": round(dt, 1),
           "threshold": round(thr, 2), "n_candidates": len(cand)}
    if best:
        x, y, bw, bh = best["bbox"]
        # --- arm span: widest dark row in the lower part of the shank
        y1 = int(y + 0.62 * bh)
        y2 = min(h, int(y + bh + 0.06 * h))
        # Search the arm span only in a window centred on the shank:
        # a full-row scan measured the whole dark background band.
        xc = x + bw / 2.0
        half = int(0.22 * w)
        x0w = max(0, int(xc - half))
        x1w = min(w, int(xc + half))
        band = dn[y1:y2, x0w:x1w]
        rows = (band > 0.30).astype(np.uint8)
        widths = []
        for rr in rows:
            idx = np.flatnonzero(rr)
            if idx.size:
                widths.append((idx[-1] - idx[0] + 1,
                               x0w + int(idx[0]), x0w + int(idx[-1])))
        arm_w, arm_x0, arm_x1 = max(widths) if widths else (bw, x, x + bw)
        arm_w = max(arm_w, bw)
        out.update({
            "bbox_px": [int(min(x, arm_x0)), int(y),
                        int(max(x + bw, arm_x1) - min(x, arm_x0)), int(bh)],
            "center_x": round((min(x, arm_x0)
                               + max(x + bw, arm_x1)) / 2.0 / w, 4),
            "center_y": round((y + bh / 2) / h, 4),
            "width": round(arm_w / w, 4),
            "shank_width": round(bw / w, 4),
            "height": round(bh / h, 4),
            "score": best["score"], "mean_dark": best["mean_dark"],
            "vertical": best["vertical"],
        })
    if debug:
        out["candidates"] = sorted(cand, key=lambda c: -c["score"])[:5]
    return out


def annotate(img, det):
    vis = img.copy()
    if det.get("found"):
        x, y, w, h = det["bbox_px"]
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 3)
        cv2.putText(vis, f"anchor s={det['score']:.2f} "
                         f"h={det['height']:.3f}", (x, max(30, y - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
    else:
        cv2.putText(vis, "no detection", (40, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)
    return vis


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="*")
    ap.add_argument("--video")
    ap.add_argument("--out", default="visualizations/real_anchor_detect")
    ap.add_argument("--stride", type=int, default=10)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    results = []

    if args.video:
        cap = cv2.VideoCapture(args.video)
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        idx = 0
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            if idx % args.stride == 0:
                det = detect_anchor(fr)
                det["frame"] = idx
                results.append(det)
                if len(results) % 20 == 1:
                    cv2.imwrite(os.path.join(args.out, f"v{idx:05d}.png"),
                                annotate(fr, det))
            idx += 1
        cap.release()
        found = [r for r in results if r["found"]]
        print(f"video {os.path.basename(args.video)}: "
              f"{len(found)}/{len(results)} frames with detection "
              f"({100*len(found)/max(1,len(results)):.0f}%), "
              f"median {np.median([r['ms'] for r in results]):.1f} ms")
        if found:
            hs = [r["height"] for r in found]
            cxs = [r["center_x"] for r in found]
            print(f"  bbox height frac: median {np.median(hs):.3f} "
                  f"IQR {np.percentile(hs,25):.3f}-{np.percentile(hs,75):.3f}")
            print(f"  center_x: median {np.median(cxs):.3f} "
                  f"std {np.std(cxs):.3f}")
    for p in args.images:
        img = cv2.imread(p)
        if img is None:
            continue
        det = detect_anchor(img, debug=True)
        cand = det.pop("candidates", None)
        det.pop("_mask", None)
        det["image"] = os.path.basename(p)
        results.append(det)
        cv2.imwrite(os.path.join(args.out,
                                 "det_" + os.path.basename(p)),
                    annotate(img, det))
        print(json.dumps(det, indent=1))
        if cand:
            print("  top candidates:", json.dumps(cand[:3]))
    with open(os.path.join(args.out, "detections.json"), "w") as f:
        json.dump(results, f, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
