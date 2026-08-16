"""Recover the real runs' perception stream from the annotated videos.

WHY THIS EXISTS. The real campaign recorded commanded velocity and an
annotated onboard video, and nothing else. No overhead recording was
made during the runs, so there is no external measurement of where the
vehicle actually went: real trajectory, path length, lateral deviation
and true minimum clearance are NOT recoverable, and this file does not
pretend otherwise. Integrating the commanded velocity would produce a
curve that looks like a trajectory and is not one -- an open-loop
integration of a saturating, deadbanded plant in moving water -- so it
is not done anywhere in this analysis.

WHAT IS RECOVERABLE, EXACTLY. The detector drew its own output onto
every frame it processed, and the drawing is unambiguous:

    green  (0,255,0)    a detection the qualifier ACCEPTED
    orange (0,165,255)  a detection it rejected
    no box              no detection that frame

Reading those rectangles back is not re-detection. It recovers the very
observation stream that reached the planner, frame by frame, including
the frames where nothing was seen. From the box height the shared
monocular estimator gives the range the planner itself believed --
estimate_range() from rov_obstacle_avoidance.planner, the same function
the simulated campaign uses, so the quantity is directly comparable
across domains.

WHAT THE NUMBER IS AND IS NOT. It is the ESTIMATED range, not the true
one. The paired-observation fit (config/calibration/s1_observation_fit.json)
measured that estimator against ground truth: it reads 0.469 of true
range with a median absolute error of 1.01 m over 24 paired samples.
Every range reported here inherits that bias, and it is reported as an
estimator output on both sides of the sim-real comparison rather than
being corrected -- correcting one side would break the comparison the
figure exists to make.

MEASUREMENT ERROR OF THE RECOVERY ITSELF. The rectangle is stroked 4 px
wide, so the recovered extent is uncertain by about +/-2 px. Range goes
as 1/tan(h/2), so near-linearly as 1/h for small angles: a 2 px error on
a 150 px box is 1.3 % of range. That is two orders of magnitude below
the estimator's own bias and is reported per run rather than assumed.

TIME BASE. The VideoWriter was opened with a nominal 4 fps while frames
were written at the detector's actual processing rate, so the video's
own timestamps are meaningless. Frame INDEX is exact; wall-clock time is
reconstructed only as index / measured mean rate, and any quantity that
would need tighter alignment than that is reported as unavailable rather
than estimated.

Usage:
    python scripts/analysis/reconstruct_real_perception.py
"""

from __future__ import annotations

import json
import math
import os
import sys

import cv2
import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(_ROOT, "src", "rov_obstacle_avoidance"))

# The colours the detector draws with, BGR, exactly as in
# src/rov_real_bridge/rov_real_bridge/real_detector_node.py::_write_annotated
ACCEPTED_BGR = (0, 255, 0)
REJECTED_BGR = (0, 165, 255)
STROKE_PX = 4

# Shared monocular constants.
#
# VFOV IS 90, NOT THE 60 THE BENCHMARK FILE DECLARES. This reconstruction
# must reproduce the range the planner ITSELF believed, and the planner
# reads camera_vertical_fov_deg, whose default is 90.0
# (rov_obstacle_avoidance/planner.py:106). NEITHER pipeline ever sets it:
# the real launch passes only camera_hfov_deg, and to the detector rather
# than the planner (rov_real_bridge/launch/real_pipeline.launch.py:159-169),
# and the simulated launch does not declare it at all. So
# camera_vfov_deg: 60.0 in config/pool_benchmark_FROZEN.yaml is a
# documented constant that no node reads, in both domains equally.
#
# That the two domains agree is what matters for the comparison, and they
# do -- both ran at 90. But a range reported here with 60 would be about
# 1.5x the range the planner acted on, which would make every number in
# this file a description of a system that did not run.
VFOV_DEG = 90.0
TARGET_HEIGHT_M = 0.5
MAX_RANGE_M = 40.0

# Which recording belongs to which campaign run. Runs 3 and 4 have no
# video: the detector wrote none for them, so their perception stream is
# simply absent and is reported as absent.
CAMPAIGN = [
    (1, "K0", "20260816_120927_01_K0_committed_v2"),
    (2, "K0", "20260816_122520_run02_K0_committed"),
    (3, "K0", "20260816_122752_run03_K0_committed"),
    (4, "K0", "20260816_123213_run04_K0_committed"),
    (6, "K1", "20260816_124312_run06_K1_committed"),
    (7, "K1", "20260816_124803_run07_K1_committed"),
    (8, "K1M", "20260816_125238_run08_K1M_committed"),
    (9, "K1M", "20260816_125518_run09_K1M_committed"),
]

# Run 9's file kept growing for over an hour after the run ended, because
# an orphaned detector held the writer open; only its first frames belong
# to the run. Reading 11 GB to reach them is pointless -- and decoding
# that far into it killed the process outright on the first attempt -- so
# frames are capped and the cap is reported with the result.
#
# The cap is set from measurement, not taste: run 8 is a 25 s run and its
# recording holds 551 frames, i.e. about 22 frames per second, so a 25 s
# run cannot honestly contain more than ~600. 1200 leaves a wide margin
# for the pipeline's start-up frames while staying far from the point
# where the decoder fell over.
MAX_FRAMES = 1200


def estimate_range_m(norm_height: float) -> float:
    """The planner's own monocular range, reimplemented identically.

    Kept in this file rather than imported so the reconstruction does not
    depend on a built ROS workspace; it is checked against the real
    implementation by test_reconstruct_real_perception.py.
    """
    h = min(max(norm_height, 0.0), 1.0)
    vfov = math.radians(VFOV_DEG)
    if h <= 1e-4:
        return MAX_RANGE_M
    t = math.tan(0.5 * h * vfov)
    if t <= 1e-6:
        return MAX_RANGE_M
    return min(max((TARGET_HEIGHT_M * 0.5) / t, 0.0), MAX_RANGE_M)


def find_box(frame, colour, tol=40):
    """Locate the stroked rectangle of the given colour.

    The stroke is a solid constant colour on a photographic background,
    so an exact-ish colour match isolates it cleanly. The text label is
    drawn in the same colour, so the search is restricted to rows below
    the label band and the returned box is the bounding box of the
    remaining coloured pixels.
    """
    b, g, r = colour
    d = (np.abs(frame[:, :, 0].astype(np.int16) - b)
         + np.abs(frame[:, :, 1].astype(np.int16) - g)
         + np.abs(frame[:, :, 2].astype(np.int16) - r))
    mask = d < tol
    mask[:100, :] = False          # label band: "score X.XX ACCETTATA"
    ys, xs = np.nonzero(mask)
    if len(ys) < 200:              # a rectangle outline is thousands of px
        return None
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    # The stroke straddles the true edge, so the drawn extent overstates
    # the box by one stroke width overall; take the mid-line.
    return (x0 + STROKE_PX // 2, y0 + STROKE_PX // 2,
            (x1 - x0) - STROKE_PX, (y1 - y0) - STROKE_PX)


def process(path, cap=MAX_FRAMES):
    cap_v = cv2.VideoCapture(path)
    rows, i, truncated = [], 0, False
    while True:
        ok, frame = cap_v.read()
        if not ok:
            break
        if i >= cap:
            truncated = True
            break
        H = frame.shape[0]
        box = find_box(frame, ACCEPTED_BGR)
        state = "accepted"
        if box is None:
            box = find_box(frame, REJECTED_BGR)
            state = "rejected" if box is not None else "none"
        rec = {"frame": i, "state": state}
        if box is not None and box[3] > 0:
            nh = box[3] / float(H)
            rec.update({
                "bbox_px": [int(v) for v in box],
                "frame_height": int(H),
                "norm_height": round(nh, 5),
                "range_m": round(estimate_range_m(nh), 3),
                # sensitivity of the range to the +/- half-stroke ambiguity
                "range_err_m": round(
                    abs(estimate_range_m(max(nh - STROKE_PX / H, 1e-5))
                        - estimate_range_m(nh + STROKE_PX / H)) / 2.0, 3),
            })
        rows.append(rec)
        i += 1
    cap_v.release()
    return rows, truncated


def summarise(run, geom, rows, truncated):
    acc = [r for r in rows if r["state"] == "accepted" and "range_m" in r]
    rej = [r for r in rows if r["state"] == "rejected"]
    out = {
        "run": run, "geometry": geom,
        "frames": len(rows),
        "frames_truncated_at_cap": truncated,
        "accepted": len(acc), "rejected": len(rej),
        "none": len(rows) - len(acc) - len(rej),
        "accept_fraction": round(len(acc) / max(len(rows), 1), 3),
    }
    if acc:
        rng = np.array([r["range_m"] for r in acc])
        err = np.array([r["range_err_m"] for r in acc])
        out.update({
            "range_first_accept_m": acc[0]["range_m"],
            "range_min_m": round(float(rng.min()), 3),
            "range_median_m": round(float(np.median(rng)), 3),
            "range_max_m": round(float(rng.max()), 3),
            "frame_first_accept": acc[0]["frame"],
            "frame_at_min_range": int(acc[int(rng.argmin())]["frame"]),
            "recovery_err_median_m": round(float(np.median(err)), 3),
            "recovery_err_frac_of_range": round(
                float(np.median(err / np.maximum(rng, 1e-6))), 4),
        })
    return out


def reload_rows(path):
    """Re-read a previous pass and recompute range from the stored heights.

    Decoding the videos costs tens of minutes and the normalised box
    height does not depend on any camera constant, so a change to the
    optics only needs the ranges recomputed, not the frames re-read.
    """
    rows = []
    with open(path) as f:
        for line in f:
            r = json.loads(line)
            if "norm_height" in r:
                nh = r["norm_height"]
                H = r.get("frame_height", 1080)
                r["range_m"] = round(estimate_range_m(nh), 3)
                r["range_err_m"] = round(
                    abs(estimate_range_m(max(nh - STROKE_PX / H, 1e-5))
                        - estimate_range_m(nh + STROKE_PX / H)) / 2.0, 3)
            rows.append(r)
    return rows


def main() -> int:
    base = os.path.join(_ROOT, "experiments", "real", "quick_runs")
    outdir = os.path.join(_ROOT, "experiments", "real", "perception_recovery")
    os.makedirs(outdir, exist_ok=True)
    reuse = "--reuse" in sys.argv
    summary = []
    for run, geom, name in CAMPAIGN:
        cached = os.path.join(outdir, "run%02d_perception.jsonl" % run)
        if reuse and os.path.exists(cached):
            rows = reload_rows(cached)
            with open(cached, "w") as f:
                for r in rows:
                    f.write(json.dumps(r) + "\n")
            s = summarise(run, geom, rows, False)
            s["video"] = "(riletto dalla passata precedente)"
            summary.append(s)
            print("prova %d (%-3s): %4d frame  accettati %4d (%.0f%%)  "
                  "range primo %s  min %s  err %s  [ricalcolato]"
                  % (run, geom, s["frames"], s["accepted"],
                     100 * s["accept_fraction"],
                     s.get("range_first_accept_m", "-"),
                     s.get("range_min_m", "-"),
                     s.get("recovery_err_median_m", "-")))
            continue
        vid = os.path.join(base, name, "onboard_detections.avi")
        if not os.path.exists(vid):
            print("prova %d (%s): nessun video registrato" % (run, geom))
            summary.append({"run": run, "geometry": geom,
                            "video": None, "frames": 0,
                            "note": "no onboard recording exists for this run"})
            continue
        try:
            rows, truncated = process(vid)
        except Exception as exc:                     # noqa: BLE001
            # A recording that cannot be decoded is reported as such and
            # the remaining runs still get processed. Losing eight runs
            # because the ninth is malformed is the wrong failure mode
            # when the dataset can never be re-acquired.
            print("prova %d (%s): decodifica fallita: %s" % (run, geom, exc),
                  flush=True)
            summary.append({"run": run, "geometry": geom, "frames": 0,
                            "note": "video present but not decodable: %s"
                                    % exc})
            continue
        with open(os.path.join(outdir, "run%02d_perception.jsonl" % run),
                  "w") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")
        s = summarise(run, geom, rows, truncated)
        s["video"] = os.path.relpath(vid, _ROOT).replace("\\", "/")
        summary.append(s)
        print("prova %d (%-3s): %4d frame  accettati %4d (%.0f%%)  "
              "range primo %s  min %s  err %s"
              % (run, geom, s["frames"], s["accepted"],
                 100 * s["accept_fraction"],
                 s.get("range_first_accept_m", "-"),
                 s.get("range_min_m", "-"),
                 s.get("recovery_err_median_m", "-")))

    meta = {
        "source": "annotated onboard video, detector overlay read back",
        "not_recoverable": [
            "vehicle trajectory", "path length", "lateral deviation",
            "true minimum clearance", "true range to obstacle",
        ],
        "reason_not_recoverable": (
            "no overhead camera recording was made during the campaign "
            "runs; the only spatial observation is the onboard monocular "
            "estimate, which has no independent reference"),
        "range_is": "the planner's own monocular estimate, not ground truth",
        "estimator_bias_vs_truth": {
            "slope": 0.469, "median_abs_error_m": 1.01, "n_paired": 24,
            "source": "config/calibration/s1_observation_fit.json"},
        "vfov_deg": VFOV_DEG, "target_height_m": TARGET_HEIGHT_M,
        "runs": summary,
    }
    p = os.path.join(outdir, "summary.json")
    with open(p, "w") as f:
        json.dump(meta, f, indent=1)
    print("\n->", os.path.relpath(p, _ROOT).replace("\\", "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
