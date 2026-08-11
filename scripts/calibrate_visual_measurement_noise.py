"""Fit the T3 adaptive measurement-noise model from empirical residuals.

Input: replay-format JSONL files where BOTH `meas` (detector output) and
`gt` (oracle geometry projected to image space) are present for many frames
— i.e. real YOLO-vs-oracle logs (pending B1/B2 asset/model transfer) or any
synchronized detector/ground-truth recording. The synthetic D-cases are NOT
valid calibration inputs for the paper (their noise is generated, not
observed); they may be used only to smoke-test this pipeline.

Model:  log(residual_var / base_var) = theta . phi(features)
fitted by least squares on log squared residuals, separately for the center
axes and the log-size axes.

Splits: files are assigned round-robin to calibration / validation /
final-evaluation. Coefficients are fitted ONLY on the calibration split;
the validation split reports goodness; the final split must stay untouched
until the paper's final evaluation.

Output: JSON consumable by AdaptiveNoiseModel.from_file, with calibrated=true
and provenance. Until real data exists, DO NOT commit a calibrated file —
T3 stays theta=0 (== T2), clearly labeled.

Usage:
    python scripts/calibrate_visual_measurement_noise.py \
        --data <dir with *.jsonl> --out config/t3_noise_model.json
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "src", "rov_obstacle_tracking"))

from rov_obstacle_tracking.replay import load_dataset  # noqa: E402
from rov_obstacle_tracking.temporal_core import (  # noqa: E402
    FEATURE_ORDER,
    Detection,
    DetectionEvent,
    detection_features,
)


def extract_samples(path: str):
    """Yield (features_vec, sq_residual_center, sq_residual_logsize)."""
    records = load_dataset(path)
    prev_t = None
    for r in records:
        if not (r.message_present and r.detection_present and r.meas
                and r.gt and r.gt.get("present")):
            if r.message_present:
                prev_t = r.t
            continue
        m, g = r.meas, r.gt
        det = Detection(class_name=str(m["class_name"]),
                        confidence=float(m["confidence"]),
                        cx=float(m["cx"]), cy=float(m["cy"]),
                        w=float(m["w"]), h=float(m["h"]))
        feats = detection_features(
            det, DetectionEvent(t=r.t, detections=[det]), prev_t)
        phi = np.array([feats[k] for k in FEATURE_ORDER])
        rc = (m["cx"] - g["cx"]) ** 2 + (m["cy"] - g["cy"]) ** 2
        rs = ((math.log(max(m["w"], 1e-4)) - math.log(max(g["w"], 1e-4))) ** 2
              + (math.log(max(m["h"], 1e-4)) - math.log(max(g["h"], 1e-4))) ** 2)
        prev_t = r.t
        yield phi, rc, rs


def fit_theta(phis: np.ndarray, sq_res: np.ndarray,
              base_var: float) -> np.ndarray:
    """Least squares on log(residual^2 / base_var); floor tiny residuals."""
    y = np.log(np.maximum(sq_res, 1e-10) / base_var)
    theta, *_ = np.linalg.lstsq(phis, y, rcond=None)
    return theta


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", required=True,
                        help="directory containing replay JSONL with meas+gt")
    parser.add_argument("--out", required=True)
    parser.add_argument("--r-center", type=float, default=0.015)
    parser.add_argument("--r-logsize", type=float, default=0.05)
    parser.add_argument("--min-samples", type=int, default=500)
    parser.add_argument("--allow-synthetic", action="store_true",
                        help="permit fitting on synthetic D-cases "
                             "(pipeline smoke test ONLY, never for the paper)")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.data, "*.jsonl")))
    if not files:
        print(f"FATAL: no JSONL files in {args.data}")
        return 2
    synth = [f for f in files if os.path.basename(f).startswith("D")]
    if synth and not args.allow_synthetic:
        print("FATAL: input looks synthetic (D*.jsonl). Synthetic cases have "
              "GENERATED noise and are not valid calibration data for the "
              "paper. Pass --allow-synthetic only for pipeline smoke tests.")
        return 2

    splits = {"calibration": files[0::3] + files[1::3], "validation": files[2::3]}
    print(f"calibration files: {len(splits['calibration'])}, "
          f"validation files: {len(splits['validation'])}")

    def collect(file_list):
        P, RC, RS = [], [], []
        for f in file_list:
            for phi, rc, rs in extract_samples(f):
                P.append(phi); RC.append(rc); RS.append(rs)
        return np.array(P), np.array(RC), np.array(RS)

    P, RC, RS = collect(splits["calibration"])
    if len(P) < args.min_samples:
        print(f"FATAL: only {len(P)} paired samples (<{args.min_samples}). "
              "Not enough data for honest calibration — refusing to emit "
              "coefficients.")
        return 3

    base_c = args.r_center ** 2 * 2      # 2 axes
    base_s = args.r_logsize ** 2 * 2
    theta_c = fit_theta(P, RC, base_c)
    theta_s = fit_theta(P, RS, base_s)

    # Validation goodness: correlation between predicted and observed log var.
    Pv, RCv, RSv = collect(splits["validation"])
    val = {}
    if len(Pv) > 50:
        pred = Pv @ theta_c
        obs = np.log(np.maximum(RCv, 1e-10) / base_c)
        val["center_corr"] = float(np.corrcoef(pred, obs)[0, 1])
        pred_s = Pv @ theta_s
        obs_s = np.log(np.maximum(RSv, 1e-10) / base_s)
        val["logsize_corr"] = float(np.corrcoef(pred_s, obs_s)[0, 1])

    out = {
        "theta_center": [float(v) for v in theta_c],
        "theta_logsize": [float(v) for v in theta_s],
        "feature_order": FEATURE_ORDER,
        "calibrated": True,
        "synthetic_smoke_test": bool(synth),
        "source": (f"fitted {time.strftime('%Y-%m-%d')} on "
                   f"{len(splits['calibration'])} files / {len(P)} samples; "
                   f"validation: {val}"),
        "base_r_center": args.r_center,
        "base_r_logsize": args.r_logsize,
        "max_scale": 25.0,
    }
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    print(f"wrote {args.out}")
    print(json.dumps(out, indent=2))
    if synth:
        print("WARNING: fitted on SYNTHETIC data — smoke test only, "
              "never use for the paper.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
