"""Fit the S1 static observation model in the BBOX DOMAIN.

S1 replaces the simulator's noiseless projection of ground truth at
`/perception/obstacles_raw` with the MEASURED observation process, and
changes nothing else. What crosses that boundary is a bounding box, so
this fits the bbox itself, not a derived range:

    * detection probability as a function of true range (the marginal;
      the BURST structure of the misses belongs to S2 and is not touched
      here, or the thinning would be counted twice)
    * bbox CENTRE error, which is what the planner turns into bearing
    * bbox HEIGHT error, which the shared estimator turns into range
    * bbox WIDTH and aspect, recorded because the qualifier gates on them
    * outliers, kept as a separate rate rather than folded into a
      Gaussian that would then fit neither the bulk nor the tail

WHY NOT A RANGE-RESIDUAL MODEL. The earlier attempt modelled the error
of a width-derived range. That range was shown to be structurally
invalid (docs/OBSERVATION_MODEL.md), and more importantly the range is
not what crosses the shared boundary: modelling it would inject the
error at the wrong place and let the simulated planner see a bbox that
could never have produced it.

IDENTIFIABILITY. The vertical FOV and the physical extent the bbox
height spans are FIXED from calibration and a tape measurement. This
script measures the BIAS and SCATTER of that fixed model; it does not
fit both, which would absorb model error into meaningless parameters.

Usage:  python scripts/analysis/fit_s1_observation.py
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
DATA = os.path.join(_ROOT, "experiments", "real", "paired_dataset")
OUT = os.path.join(_ROOT, "config", "calibration", "s1_observation_fit.json")


def load():
    recs = []
    # Prefer the REPLAY files: the original recording was made in
    # morning haze with absolute detector thresholds and contains zero
    # detections. redetect_paired.py replays the corrected detector over
    # the same stored frames; the ground truth is untouched because it
    # comes from the overhead camera, not the detector.
    paths = sorted(glob.glob(os.path.join(DATA, "*",
                                          "position_replay_*.json")))
    replay = bool(paths)
    if not paths:
        paths = sorted(glob.glob(os.path.join(DATA, "*",
                                              "position_*.json")))
    for path in paths:
        with open(path) as f:
            d = json.load(f)
        meta = d.get("meta", {})
        for r in d["records"]:
            r["_meta"] = meta
            if replay and "det_replay" in r:
                r["det"] = r["det_replay"]
            recs.append(r)
    print("sorgente:", "REPLAY del rilevatore corretto" if replay
          else "registrazione originale")
    return recs


def true_bearing_deg(r):
    return r.get("gt_bearing_pool_deg")


def main() -> int:
    recs = load()
    rng = [r for r in recs if r.get("gt_range_pool_m")]
    print("record %d, con ground truth %d" % (len(recs), len(rng)))

    # ---- 1. detection probability vs true range ----------------------
    edges = [0.0, 0.8, 1.2, 1.6, 2.0, 10.0]
    print("\n--- probabilita di rilevamento per fascia di distanza ---")
    det_curve = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        band = [r for r in rng if lo <= r["gt_range_pool_m"] < hi]
        if not band:
            continue
        k = sum(1 for r in band if r["det"].get("found"))
        p = k / len(band)
        det_curve.append({"range_lo_m": lo, "range_hi_m": hi,
                          "n": len(band), "detections": k,
                          "p_detect": round(p, 3)})
        print("  %.1f-%.1f m: %2d/%2d = %.2f" % (lo, hi, k, len(band), p))

    paired = [r for r in rng if r["det"].get("found")]
    print("\ncoppie utilizzabili (detection E ground truth): %d" % len(paired))
    if not paired:
        print("nessuna coppia: S1 non fittabile")
        return 1

    # ---- 2. bbox centre error -> bearing ------------------------------
    # The detector's horizontal centre is what becomes bearing. Compare
    # it with the bearing the overhead ground truth says is true.
    meta = paired[0]["_meta"]
    W = meta.get("image_width") or 1920
    hfov = math.radians(meta.get("hfov_deg") or 80.0)
    berr, hgt, wid, asp, ranges = [], [], [], [], []
    for r in paired:
        det = r["det"]
        cx = det.get("center_x")
        if cx is None:
            continue
        # bearing implied by the bbox centre, small-angle pinhole
        b_obs = math.degrees(math.atan2((cx - W / 2.0),
                                        (W / 2.0) / math.tan(hfov / 2.0)))
        b_true = true_bearing_deg(r)
        if b_true is not None:
            berr.append(b_obs - b_true)
        h, w = det.get("height"), det.get("width")
        if h:
            hgt.append(h)
            ranges.append(r["gt_range_pool_m"])
        if h and w:
            wid.append(w)
            asp.append(w / float(h))

    def stats(v, name, unit=""):
        if not v:
            print("  %-22s nessun dato" % name)
            return None
        a = np.array(v, float)
        med = float(np.median(a))
        mad = float(np.median(np.abs(a - med))) * 1.4826
        out = int(np.sum(np.abs(a - med) > 3 * max(mad, 1e-9)))
        print("  %-22s mediana %8.3f%s  scarto robusto %7.3f  "
              "outlier %d/%d" % (name, med, unit, mad, out, len(a)))
        return {"median": round(med, 4), "robust_sd": round(mad, 4),
                "n": len(a), "outliers": out,
                "outlier_rate": round(out / len(a), 3),
                "min": round(float(a.min()), 4),
                "max": round(float(a.max()), 4)}

    print("\n--- errore della bbox ---")
    # The bearing offset is a FRAME CONVENTION difference, not a
    # detector bias: the ground truth bearing is in the pool frame while
    # the bbox centre gives bearing relative to the vehicle heading, and
    # the operator held the vehicle pointed at the anchor throughout.
    # The evidence is the spread -- a systematic median with 0.3 deg of
    # robust scatter cannot be detector error. The offset is reported
    # separately; the SCATTER is what S1 injects.
    s_bear = stats(berr, "rotta grezza", " deg")
    if berr:
        _off = float(np.median(berr))
        _res = stats([b - _off for b in berr], "rotta senza offset", " deg")
        if _res is not None:
            _res["frame_offset_deg"] = round(_off, 3)
            _res["caveat"] = (
                "the vehicle was held pointing at the anchor throughout, so "
                "this scatter spans a narrow range of headings and "
                "understates the variability across a real run")
            s_bear = _res
    s_h = stats(hgt, "altezza bbox", " px")
    s_w = stats(wid, "larghezza bbox", " px")
    s_a = stats(asp, "rapporto larg/alt")

    # ---- 3. height vs range: is the fixed model biased? ---------------
    # The shared estimator inverts h = k / R. Fit k on the real data and
    # compare with the value the FIXED constants imply: the ratio is the
    # scale bias S1 must inject, and it is a single number, not a free
    # reparameterisation of the estimator.
    scale_bias = None
    if len(hgt) >= 4:
        h = np.array(hgt, float)
        R = np.array(ranges, float)
        k_fit = float(np.sum(h * R) / len(h))       # mean of h*R
        vfov = math.radians(meta.get("vfov_deg") or 60.0)
        target_h = meta.get("target_height_m") or 0.75
        # The detector reports the bbox in NORMALISED image units (the
        # median height is 0.76, not 760 px), so the model constant must
        # be normalised too: h_norm = target_h / (2 R tan(vfov/2)).
        # Comparing a normalised measurement against a pixel model gave a
        # scale bias of 0.002, which is a unit error, not a finding.
        k_model = target_h / (2.0 * math.tan(vfov / 2.0))
        scale_bias = k_fit / k_model if k_model else None
        resid = h - k_fit / R
        print("\n--- altezza bbox contro distanza vera ---")
        print("  k misurato   %.3f (normalizzato)*m" % k_fit)
        print("  k del modello %.3f (vfov %.0f deg, altezza %.2f m)"
              % (k_model, math.degrees(vfov), target_h))
        print("  bias di scala %.3f  (1.0 = modello corretto)"
              % (scale_bias or float("nan")))
        print("  residuo altezza: mediana %.4f, scarto %.4f (normalizzato)"
              % (float(np.median(resid)), float(np.std(resid))))

    # ---- 4. the SHARED estimator against truth -----------------------
    # This is the quantity that matters: not a local formula, but what
    # rov_obstacle_avoidance.planner.estimate_range -- the function the
    # simulated planner calls -- produces from these real bounding boxes.
    est = None
    try:
        sys.path.insert(0, os.path.join(_ROOT, "src",
                                        "rov_obstacle_avoidance"))
        from rov_obstacle_avoidance.planner import estimate_range
        vfov = math.radians(meta.get("vfov_deg") or 60.0)
        th = meta.get("target_height_m") or 0.75
        pairs = []
        for r in paired:
            h = r["det"].get("height")
            if not h:
                continue
            class _O:
                height = float(h)
            R_est = estimate_range(_O(), vfov, th, 6.0)
            pairs.append((r["gt_range_pool_m"], R_est))
        if pairs:
            gt = np.array([p[0] for p in pairs], float)
            es = np.array([p[1] for p in pairs], float)
            ratio = es / gt
            print("\n--- stimatore CONDIVISO contro distanza vera ---")
            print("  n %d, vero %.2f-%.2f m, stimato %.2f-%.2f m"
                  % (len(pairs), gt.min(), gt.max(), es.min(), es.max()))
            print("  rapporto stimato/vero: mediana %.3f  "
                  "(1.0 = nessun bias)" % float(np.median(ratio)))
            print("  errore assoluto mediano %.3f m"
                  % float(np.median(np.abs(es - gt))))
            est = {"n": len(pairs),
                   "ratio_median": round(float(np.median(ratio)), 4),
                   "ratio_robust_sd": round(float(np.median(
                       np.abs(ratio - np.median(ratio)))) * 1.4826, 4),
                   "abs_error_median_m": round(
                       float(np.median(np.abs(es - gt))), 4),
                   "gt_range_m": [round(float(gt.min()), 3),
                                  round(float(gt.max()), 3)],
                   "note": "bias of the FIXED-constant monocular model as "
                           "run by the shared planner code; S1 injects "
                           "this ratio and its scatter, it does not "
                           "re-fit the estimator's constants"}
    except Exception as e:
        print("stimatore condiviso non valutabile:", e)

    out = {
        "shared_estimator_vs_truth": est,
        "source": "experiments/real/paired_dataset/*/position_*.json",
        "domain": "bounding box at /perception/obstacles_raw",
        "n_records": len(recs), "n_with_ground_truth": len(rng),
        "n_paired": len(paired),
        "detection_probability_vs_range": det_curve,
        "marginal_p_detect": round(
            sum(1 for r in rng if r["det"].get("found")) / max(len(rng), 1), 3),
        "bearing_error_deg": s_bear,
        "bbox_height_px": s_h,
        "bbox_width_px": s_w,
        "bbox_aspect": s_a,
        "height_scale_bias": (None if scale_bias is None
                              else round(scale_bias, 4)),
        "note": "MARGINAL detection probability only. The burst structure "
                "of the misses is S2's and is deliberately not fitted here; "
                "fitting both would thin the observation stream twice.",
        "identifiability": "vfov and target_height are FIXED inputs; this "
                           "file measures the bias and scatter of that "
                           "fixed model.",
    }
    with open(OUT, "w") as f:
        json.dump(out, f, indent=2)
    print("\n->", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
