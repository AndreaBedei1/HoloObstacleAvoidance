"""Controlled temporal test-case generation (D0-D14), deterministic/seeded.

Base scenario mirrors the Baseline-0 geometry in image space: an obstacle
dead ahead is approached at constant speed (bbox grows), the vehicle then
strafes (bbox center drifts toward the image edge), passes the obstacle
(bbox leaves the FOV — a LEGITIMATE disappearance). Detector noise is
Gaussian and seeded. Cases modify the base stream with dropouts (silence),
fresh-empty segments, outliers, noise bursts, confidence collapse, timestamp
jitter, and false positives.

All quantities are normalized image units; rate 30 Hz.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np

from .replay import ReplayRecord

RATE_HZ = 30.0
DT = 1.0 / RATE_HZ

# Geometry constants (match the sim scenario: 3.5 m object, VFOV 90 deg,
# approach from 12 m at 0.3 m/s, engage at ~9 m).
OBJ_H_M = 3.5
VFOV_RAD = math.radians(90.0)
APPROACH_RANGE0_M = 12.0
APPROACH_SPEED = 0.3
ENGAGE_RANGE_M = 9.0
T_ENGAGE = (APPROACH_RANGE0_M - ENGAGE_RANGE_M) / APPROACH_SPEED  # 10.0 s
T_AVOID_END = T_ENGAGE + 12.0     # strafe phase duration
T_END = T_AVOID_END + 4.0         # short tail after disappearance


def apparent_height(range_m: float) -> float:
    return min(0.9, OBJ_H_M / (2.0 * max(range_m, 0.5)
                               * math.tan(VFOV_RAD / 2.0)))


def gt_state(t: float) -> Optional[Dict[str, float]]:
    """Scripted ground-truth bbox; None once the obstacle left the FOV."""
    if t < T_ENGAGE:
        rng = APPROACH_RANGE0_M - APPROACH_SPEED * t
        cx = 0.5
    elif t < T_AVOID_END:
        # Strafing: range shrinks slower, center drifts toward image edge.
        rng = ENGAGE_RANGE_M - 0.15 * (t - T_ENGAGE)
        cx = 0.5 - 0.38 * (t - T_ENGAGE) / (T_AVOID_END - T_ENGAGE)
    else:
        return None  # passed: legitimately out of FOV
    h = apparent_height(rng)
    w = h * 0.9
    if cx - w / 2 < 0.02:
        return None
    return {"cx": cx, "cy": 0.5, "w": w, "h": h}


def base_stream(seed: int, noise_scale: float = 1.0,
                conf_mean: float = 0.85) -> List[ReplayRecord]:
    rng = np.random.default_rng(seed)
    records = []
    t = 0.0
    while t <= T_END:
        gt = gt_state(t)
        if gt is None:
            records.append(ReplayRecord(
                t=t, message_present=True, detection_present=False,
                meas=None, gt={"present": False}))
        else:
            meas = {
                "class_name": "anchor",
                "confidence": float(np.clip(
                    conf_mean + rng.normal(0, 0.05), 0.05, 1.0)),
                "cx": float(gt["cx"] + rng.normal(0, 0.008 * noise_scale)),
                "cy": float(gt["cy"] + rng.normal(0, 0.008 * noise_scale)),
                "w": float(gt["w"] * math.exp(rng.normal(0, 0.03 * noise_scale))),
                "h": float(gt["h"] * math.exp(rng.normal(0, 0.03 * noise_scale))),
            }
            records.append(ReplayRecord(
                t=t, message_present=True, detection_present=True,
                meas=meas, gt={"present": True, **gt}))
        t += DT
    return records


def _window(records: List[ReplayRecord], t0: float, t1: float,
            mode: str) -> None:
    """Apply 'silence' or 'empty' to records in [t0, t1)."""
    for r in records:
        if t0 <= r.t < t1:
            if mode == "silence":
                r.message_present = False
                r.detection_present = False
                r.meas = None
            elif mode == "empty":
                r.detection_present = False
                r.meas = None


def generate_case(case: str, seed: int = 7) -> List[ReplayRecord]:
    """D0-D14 controlled cases. Deterministic for a given (case, seed)."""
    recs = base_stream(seed)

    if case == "D0":                       # clean continuous detection
        pass
    elif case == "D1":
        _window(recs, 5.0, 5.25, "silence")
    elif case == "D2":
        _window(recs, 5.0, 5.5, "silence")
    elif case == "D3":
        _window(recs, 5.0, 6.0, "silence")
    elif case == "D4":
        _window(recs, 5.0, 7.0, "silence")
    elif case == "D5":                     # EARLY dropout ~1 s after engage
        _window(recs, T_ENGAGE + 1.0, T_ENGAGE + 3.0, "silence")
    elif case == "D6":                     # mid-maneuver dropout
        _window(recs, T_ENGAGE + 6.0, T_ENGAGE + 8.0, "silence")
    elif case == "D7":                     # repeated short dropouts
        for k in range(6):
            _window(recs, 4.0 + k * 1.5, 4.5 + k * 1.5, "silence")
    elif case == "D8":                     # fresh EMPTY instead of silence
        _window(recs, T_ENGAGE + 1.0, T_ENGAGE + 3.0, "empty")
    elif case == "D9":                     # one large bbox outlier
        for r in recs:
            if abs(r.t - 6.0) < DT / 2 and r.meas:
                r.meas["cx"] = min(0.95, r.meas["cx"] + 0.30)
                r.meas["cy"] = min(0.95, r.meas["cy"] + 0.25)
                r.meas["w"] *= 2.5
                r.meas["h"] *= 2.5
    elif case == "D10":                    # noisy segment
        noisy = base_stream(seed + 1, noise_scale=5.0)
        for i, r in enumerate(recs):
            if 6.0 <= r.t < 8.0:
                recs[i] = noisy[i]
    elif case == "D11":                    # confidence collapse then dropout
        for r in recs:
            if 8.0 <= r.t < 9.0 and r.meas:
                frac = (r.t - 8.0) / 1.0
                r.meas["confidence"] = float(
                    max(0.15, 0.85 - 0.7 * frac))
        _window(recs, 9.0, 10.5, "silence")
    elif case == "D12":                    # irregular frame interval
        rng = np.random.default_rng(seed + 2)
        t_acc = 0.0
        for r in recs:
            r.t = t_acc
            t_acc += DT * float(rng.uniform(0.6, 1.4))
    elif case == "D13":                    # brief false positive, no real obj
        recs = []
        t = 0.0
        rng = np.random.default_rng(seed + 3)
        while t <= 8.0:
            if 2.0 <= t < 2.15:            # 4-5 frames of FP
                meas = {"class_name": "anchor", "confidence": 0.6,
                        "cx": 0.7 + float(rng.normal(0, 0.01)),
                        "cy": 0.4, "w": 0.08, "h": 0.1}
                recs.append(ReplayRecord(t=t, message_present=True,
                                         detection_present=True, meas=meas,
                                         gt={"present": False}))
            else:
                recs.append(ReplayRecord(t=t, message_present=True,
                                         detection_present=False, meas=None,
                                         gt={"present": False}))
            t += DT
    elif case == "D14":                    # legitimate disappearance
        # base_stream already ends with fresh-empty after the pass — extend
        # the tail so ghost behavior is measurable.
        t_last = recs[-1].t
        t = t_last + DT
        while t <= t_last + 6.0:
            recs.append(ReplayRecord(t=t, message_present=True,
                                     detection_present=False, meas=None,
                                     gt={"present": False}))
            t += DT
    else:
        raise ValueError(f"unknown case {case!r}")
    return recs


ALL_CASES = [f"D{i}" for i in range(15)]


# ---------------------------------------------------------------------------
# Phase-7B young-track / startup cases (Y0-Y9, S0)
# ---------------------------------------------------------------------------

def _clean_snippet(seed: int, n: int, t0: float = 0.0,
                   cx: float = 0.5, rng_m: float = 10.0) -> List[ReplayRecord]:
    """n clean frames of a static-ish target at range rng_m."""
    rng = np.random.default_rng(seed)
    out = []
    t = t0
    h = apparent_height(rng_m)
    for _ in range(n):
        meas = {"class_name": "anchor",
                "confidence": float(np.clip(0.85 + rng.normal(0, 0.05),
                                            0.05, 1.0)),
                "cx": float(cx + rng.normal(0, 0.008)),
                "cy": float(0.5 + rng.normal(0, 0.008)),
                "w": float(h * 0.9 * math.exp(rng.normal(0, 0.03))),
                "h": float(h * math.exp(rng.normal(0, 0.03)))}
        out.append(ReplayRecord(t=t, message_present=True,
                                detection_present=True, meas=meas,
                                gt={"present": True, "cx": cx, "cy": 0.5,
                                    "w": h * 0.9, "h": h}))
        t += DT
    return out


def _giant_outlier(t: float) -> ReplayRecord:
    h = apparent_height(10.0)
    return ReplayRecord(t=t, message_present=True, detection_present=True,
                        meas={"class_name": "anchor", "confidence": 0.7,
                              "cx": 0.82, "cy": 0.78,
                              "w": min(0.9, h * 0.9 * 2.5),
                              "h": min(0.9, h * 2.5)},
                        gt={"present": True, "cx": 0.5, "cy": 0.5,
                            "w": h * 0.9, "h": h})


def generate_y_case(case: str, seed: int = 11) -> List[ReplayRecord]:
    """Young-track and startup qualification cases (deterministic)."""
    if case == "Y0":                      # clean first measurements
        return _clean_snippet(seed, 90)
    if case in ("Y1", "Y2", "Y3", "Y4"):  # outlier at update k (1..0..2..3)
        k = {"Y1": 1, "Y2": 0, "Y3": 2, "Y4": 3}[case]
        recs = _clean_snippet(seed, 90)
        recs[k] = _giant_outlier(recs[k].t)
        return recs
    if case == "Y5":                      # 3 consistent then outlier
        recs = _clean_snippet(seed, 90)
        recs[3] = _giant_outlier(recs[3].t)
        return recs
    if case == "Y6":                      # alternating inconsistent
        recs = []
        t = 0.0
        for i in range(60):
            cx = 0.3 if i % 2 == 0 else 0.7
            snip = _clean_snippet(seed + i, 1, t0=t, cx=cx)
            recs.append(snip[0])
            t += DT
        return recs
    if case == "Y7":                      # short false-positive track
        recs = []
        t = 0.0
        while t <= 4.0:
            if 1.0 <= t < 1.0 + 4 * DT:   # 4 FP frames
                recs.extend(_clean_snippet(seed, 1, t0=t, cx=0.7))
            else:
                recs.append(ReplayRecord(t=t, message_present=True,
                                         detection_present=False, meas=None,
                                         gt={"present": False}))
            t += DT
        return recs
    if case == "Y8":                      # sudden legit close-range object
        recs = [ReplayRecord(t=i * DT, message_present=True,
                             detection_present=False, meas=None,
                             gt={"present": False}) for i in range(60)]
        recs += _clean_snippet(seed, 60, t0=60 * DT, rng_m=4.0)
        return recs
    if case == "Y9":                      # very small/far legitimate object
        return _clean_snippet(seed, 120, rng_m=35.0)
    if case == "S0":                      # startup garbage then clean
        recs = []
        garbage = [(0.9, 0.95, 0.97), (0.1, 0.02, 0.9), (0.7, 0.5, 0.5),
                   (0.2, 0.85, 0.1)]
        t = 0.0
        for cx, w, h in garbage:
            recs.append(ReplayRecord(
                t=t, message_present=True, detection_present=True,
                meas={"class_name": "anchor", "confidence": 0.5,
                      "cx": cx, "cy": 0.5, "w": w, "h": h},
                gt={"present": False}))
            t += DT
        recs += _clean_snippet(seed, 120, t0=t)
        return recs
    raise ValueError(f"unknown Y case {case!r}")


Y_CASES = [f"Y{i}" for i in range(10)] + ["S0"]
