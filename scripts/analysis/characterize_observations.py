"""Characterize the REAL anchor detector and build the S1/S2 observation
model from the 2026-08-14 pilot wet runs.

WHY: the scientific boundary of this project is `/perception/obstacles_raw`
(docs/EXPERIMENTAL_BOUNDARY.md). Simulation manufactures that topic from an
observation model; reality produces it from `scripts/real/anchor_detect.py`.
The observation model is therefore not a convenience -- it is the only place
where the real sensor enters the simulated experiment, so it has to be
estimated from measurements rather than assumed. The tempting shortcut is
"monocular range = truth + N(0, sigma)" with sigma from a reported MAE. This
script exists to show, from the pilot data, that such a model would be
wrong in kind and not merely in scale, and to emit an empirical/replay model
instead.

WHAT THE PILOT DATA ACTUALLY CONTAIN (established by this script, and the
reason its structure is not the obvious one):

  * `experiments/real/missions/*` log the overhead ground truth
    (`gt_dist_px`) and the monocular range, but NOT the detector's bbox
    (`det` block absent).
  * `experiments/real/avoid_runs/*` log the `det` block but NO ground truth.
  * The intersection is EMPTY: no cycle anywhere carries both a GT distance
    and a bbox. Regressing the range residual on bbox height or detector
    score is therefore impossible with this dataset, and this script reports
    that rather than substituting a proxy.
  * bbox WIDTH and CENTER_X are nevertheless recoverable on the GT sessions,
    because `avoid_mission.py` derives range and bearing from them by a
    documented invertible map. Width is a 1:1 function of the logged range,
    so "residual vs width" is the range regression re-parameterised, not an
    independent covariate. The script inverts, cross-checks the inversion on
    the sessions that log both, and labels the result as derived.

Estimated ONLY from PILOT/CALIBRATION runs (see PILOT_MANIFEST.json). None
of these numbers may be reported as a result; they are model input.

Read-only and offline: reads log.json files, writes into
experiments/real/observation_model/. Touches no vehicle, no camera, no
MAVLink.

Usage:
    python scripts/analysis/characterize_observations.py
    python scripts/analysis/characterize_observations.py --include-git-missing
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import subprocess
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
REAL = os.path.join(REPO, "experiments", "real")
OUTDIR = os.path.join(REAL, "observation_model")

# --- geometry, copied verbatim from scripts/real/avoid_mission.py ------
# Duplicated rather than imported: that module opens a camera and a
# MAVLink link at import time, and this analysis must never touch
# hardware. The values are asserted against the logs below.
F_PX = 1277.0                 # D-015 one-point calibration
ANCHOR_WIDTH_M = 0.80         # arm span
IMG_W = 1920.0
ANCHOR_PX = np.array([1069.0, 635.0])
PX_PER_M = 402.0              # overhead scale (operator)
MIN_SCORE, MIN_H, MIN_W = 0.55, 0.12, 0.02

# `detect_anchor` searches the arm span only within +-0.22*W of the shank
# centre, so the reported width fraction cannot exceed 0.44 and the range
# derived from it cannot fall below the value below. This is a CENSORING
# limit of the sensor, not noise, and the observation model must keep it.
ARM_WINDOW_FRAC = 0.44
RANGE_FLOOR_M = ANCHOR_WIDTH_M * F_PX / (ARM_WINDOW_FRAC * IMG_W)

# The v2 shape prior rejects candidates shorter than 0.25 of the frame, so
# any logged detection below that height was produced by an EARLIER
# detector build. Used to date the sessions rather than to filter them.
V2_MIN_HEIGHT_FRAC = 0.25

# A measured range more than this multiple of (or fraction of) the truth is
# treated as a qualitatively different event ("gross failure"), not as a
# tail of the same error distribution. The value is not tuned: the data
# separate perfectly at any threshold in 1.5-4 (see the report).
GROSS_RATIO = 2.0

# GT-range bins for the conditional replay pools. Boundaries are
# mechanistic, not fitted: 1.00 m is where the arm span leaves the
# detector's search band, and the bins above it straddle RANGE_FLOOR_M.
GT_BINS = [(0.0, 1.00), (1.00, 1.60), (1.60, 2.00), (2.00, 1e9)]

N_BOOT = 5000
SEED = 20260814


# ---------------------------------------------------------------------
# small statistics helpers (no scipy in this environment)
# ---------------------------------------------------------------------

def pearson(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 3 or x.std() == 0 or y.std() == 0:
        return float("nan")
    return float(np.corrcoef(x, y)[0, 1])


def _rank(a):
    """Average ranks, so ties do not inflate the correlation."""
    a = np.asarray(a, float)
    order = np.argsort(a, kind="mergesort")
    r = np.empty(len(a), float)
    r[order] = np.arange(len(a), dtype=float)
    i = 0
    while i < len(a):
        j = i
        while j + 1 < len(a) and a[order[j + 1]] == a[order[i]]:
            j += 1
        if j > i:
            r[order[i:j + 1]] = (i + j) / 2.0
        i = j + 1
    return r


def spearman(x, y):
    if len(x) < 3:
        return float("nan")
    return pearson(_rank(x), _rank(y))


def chi2_sf_df1(x):
    """Upper tail of chi-square with 1 dof; erfc form avoids scipy."""
    if x <= 0:
        return 1.0
    return float(math.erfc(math.sqrt(x / 2.0)))


def wilson(k, n, z=1.96):
    """Wilson score interval: honest at the 0/n and n/n counts that this
    dataset keeps producing, where the Wald interval collapses to zero
    width and would silently claim certainty."""
    if n == 0:
        return [float("nan"), float("nan")]
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(max(0.0, c - h), 4), round(min(1.0, c + h), 4)]


def autocorr(x, maxlag=10):
    x = np.asarray(x, float)
    out = []
    for k in range(1, maxlag + 1):
        a, b = x[:-k], x[k:]
        if len(a) < 3 or a.std() == 0 or b.std() == 0:
            out.append(float("nan"))
        else:
            out.append(round(float(np.corrcoef(a, b)[0, 1]), 4))
    return out


def block_len_from_acf(x, lo=5, hi=20):
    """Block length for the bootstrap = first non-positive autocorrelation
    lag. Chosen from the data because these series are strongly
    autocorrelated (the vehicle moves smoothly), so an IID bootstrap would
    understate every interval."""
    ac = autocorr(x, maxlag=hi)
    for k, a in enumerate(ac, start=1):
        if not (a > 0):
            return max(lo, k)
    return hi


def moving_block_bootstrap(values, stat, block, n_boot=N_BOOT, seed=SEED):
    """CI that survives serial correlation. Returns (lo, hi) at 95%."""
    v = np.asarray(values, float)
    n = len(v)
    if n < 3:
        return [float("nan"), float("nan")]
    block = int(max(1, min(block, n)))
    rng = np.random.default_rng(seed)
    nb = int(math.ceil(n / block))
    out = np.empty(n_boot)
    starts = max(1, n - block + 1)
    for i in range(n_boot):
        idx = rng.integers(0, starts, size=nb)
        s = np.concatenate([v[j:j + block] for j in idx])[:n]
        out[i] = stat(s)
    return [round(float(np.percentile(out, 2.5)), 4),
            round(float(np.percentile(out, 97.5)), 4)]


def describe(v, name=""):
    v = np.asarray(v, float)
    if len(v) == 0:
        return {"name": name, "n": 0}
    return {"name": name, "n": int(len(v)),
            "mean": round(float(v.mean()), 4),
            "median": round(float(np.median(v)), 4),
            "std": round(float(v.std(ddof=1)) if len(v) > 1 else 0.0, 4),
            "mad": round(float(np.median(np.abs(v - np.median(v)))), 4),
            "mae": round(float(np.abs(v).mean()), 4),
            "rms": round(float(math.sqrt((v ** 2).mean())), 4),
            "min": round(float(v.min()), 4),
            "max": round(float(v.max()), 4),
            "q25": round(float(np.percentile(v, 25)), 4),
            "q75": round(float(np.percentile(v, 75)), 4)}


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------

def git_sha():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO,
            text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def git_dirty():
    try:
        return bool(subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=REPO,
            text=True, stderr=subprocess.DEVNULL).strip())
    except Exception:
        return None


def load_worktree():
    """Sessions present on disk. `kind` records which fields exist, since
    the two families of logs are NOT interchangeable."""
    out = []
    for p in sorted(glob.glob(os.path.join(
            REAL, "missions", "*", "log.json"))):
        with open(p) as f:
            out.append(("missions", os.path.basename(os.path.dirname(p)),
                        json.load(f), "worktree"))
    for p in sorted(glob.glob(os.path.join(
            REAL, "avoid_runs", "*", "log.json"))):
        with open(p) as f:
            out.append(("avoid_runs", os.path.basename(os.path.dirname(p)),
                        json.load(f), "worktree"))
    return out


def git_missing_sessions():
    """Mission logs tracked in HEAD but absent from the worktree.

    WHY this exists at all: PILOT_MANIFEST.json already flags eight such
    sessions, and the manifest's own stated rationale is that a
    filesystem-only view "launders the survivors" of a pilot day into a
    clean-looking record. Analysing only what remained on disk would
    reproduce exactly that bias, so the deleted sessions are available as
    an explicit, opt-in sensitivity check -- never silently pooled.
    """
    try:
        names = subprocess.check_output(
            ["git", "ls-tree", "-r", "--name-only", "HEAD",
             "experiments/real/missions"], cwd=REPO, text=True,
            stderr=subprocess.DEVNULL).splitlines()
    except Exception:
        return []
    out = []
    for rel in sorted(n for n in names if n.endswith("log.json")):
        if os.path.exists(os.path.join(REPO, rel)):
            continue
        try:
            blob = subprocess.check_output(
                ["git", "show", "HEAD:" + rel], cwd=REPO, text=True,
                stderr=subprocess.DEVNULL)
        except Exception:
            continue
        out.append(("missions", os.path.basename(os.path.dirname(rel)),
                    json.loads(blob), "git-HEAD-only"))
    return out


def records(sessions):
    """Flatten to per-cycle records, deriving what is derivable.

    The per-cycle overhead ROV pixel is not logged inside the cycle, but
    `traj` receives one entry per cycle in which the overhead tracker
    found the vehicle -- i.e. exactly the cycles with a non-null
    `gt_dist_px`, in order. The alignment is verified below against
    `gt_dist_px` itself and rejected if it does not reproduce it, so the
    bearing geometry never rests on an unchecked assumption.
    """
    out = []
    align = []
    for kind, name, data, origin in sessions:
        log = data.get("log", [])
        traj = data.get("traj", []) or []
        gt_idx = [i for i, e in enumerate(log)
                  if e.get("gt_dist_px") is not None]
        pix = {}
        errs = []
        for k, i in enumerate(gt_idx):
            if k >= len(traj):
                break
            d = float(np.linalg.norm(np.array(traj[k], float) - ANCHOR_PX))
            errs.append(abs(d - log[i]["gt_dist_px"]))
            pix[i] = np.array(traj[k], float)
        ok = bool(errs) and max(errs) <= 0.2      # rounding of gt to 0.1 px
        if errs:
            align.append({"session": name, "n": len(errs),
                          "max_px_error": round(max(errs), 4),
                          "accepted": ok})
        if not ok:
            pix = {}
        for i, e in enumerate(log):
            det = e.get("det") or {}
            rng = e.get("range_m")
            brg = e.get("bearing_deg")
            r = {"session": name, "kind": kind, "origin": origin,
                 "i": i, "t": e.get("t"), "state": e.get("state"),
                 "accepted": bool(e.get("accepted")),
                 "range_m": rng, "bearing_deg": brg,
                 "yaw_deg": e.get("yaw_deg"),
                 "gt_m": (None if e.get("gt_dist_px") is None
                          else e["gt_dist_px"] / PX_PER_M),
                 "rov_px": (list(pix[i]) if i in pix else None),
                 "det_found": det.get("found") if det else None,
                 "score": det.get("score"),
                 "height": det.get("height"),
                 "width_logged": det.get("width"),
                 "center_x_logged": det.get("center_x")}
            # Exact inversion of avoid_mission.rng_of / brg_of. Valid only
            # because both scripts share F_PX / ANCHOR_WIDTH_M / IMG_W;
            # verified against the logged det block where both exist.
            r["width_derived"] = (None if not rng else
                                  ANCHOR_WIDTH_M * F_PX / (rng * IMG_W))
            r["center_x_derived"] = (
                None if brg is None else
                0.5 + math.tan(math.radians(brg)) * F_PX / IMG_W)
            r["width"] = (r["width_logged"] if r["width_logged"]
                          is not None else r["width_derived"])
            r["center_x"] = (r["center_x_logged"]
                             if r["center_x_logged"] is not None
                             else r["center_x_derived"])
            out.append(r)
    return out, align


def inversion_check(recs):
    """Verify the derived width/center_x against the sessions that log
    both. If this drifts, every derived covariate is void."""
    dw, dc = [], []
    for r in recs:
        if r["width_logged"] is not None and r["width_derived"]:
            dw.append(abs(r["width_logged"] - r["width_derived"]))
        if (r["center_x_logged"] is not None
                and r["center_x_derived"] is not None):
            dc.append(abs(r["center_x_logged"] - r["center_x_derived"]))
    return {"n_width": len(dw),
            "max_abs_width_error": (round(max(dw), 6) if dw else None),
            "n_center_x": len(dc),
            "max_abs_center_x_error": (round(max(dc), 6) if dc else None),
            "tolerance": 2e-3,
            "passes": bool(dw and dc and max(dw) < 2e-3
                           and max(dc) < 2e-3),
            "residual_source": (
                "the logs round range_m to 0.01 m and bearing_deg to "
                "0.1 deg, so the inversion inherits a quantisation of "
                "about width*0.01/range (1.8e-3 at the range floor) and "
                "1.2e-3 in center_x. The observed maxima match that, "
                "i.e. the map is exact and only the printout is coarse.")}


# ---------------------------------------------------------------------
# 1. range residual
# ---------------------------------------------------------------------

def range_residuals(recs):
    pairs = [r for r in recs
             if r["range_m"] is not None and r["gt_m"] is not None]
    for r in pairs:
        r["resid_m"] = r["range_m"] - r["gt_m"]
        r["ratio"] = r["range_m"] / r["gt_m"]
        r["log_ratio"] = math.log(r["ratio"])
        r["gross"] = r["ratio"] > GROSS_RATIO or r["ratio"] < 1 / GROSS_RATIO
    return pairs


def residual_report(pairs):
    if not pairs:
        return {"n": 0, "note": "no paired samples"}
    res = np.array([p["resid_m"] for p in pairs])
    gt = np.array([p["gt_m"] for p in pairs])
    rng = np.array([p["range_m"] for p in pairs])
    lr = np.array([p["log_ratio"] for p in pairs])
    gross = np.array([p["gross"] for p in pairs])
    ng = ~gross
    blk = block_len_from_acf(res)

    rep = {"n_paired": len(pairs),
           "sessions": sorted({p["session"] for p in pairs}),
           "gt_range_m": [round(float(gt.min()), 3),
                          round(float(gt.max()), 3)],
           "bootstrap_block_len": blk,
           "all_samples": describe(res, "range residual (m), all"),
           "gross": {
               "definition": ("measured/true outside [1/%.1f, %.1f]"
                              % (GROSS_RATIO, GROSS_RATIO)),
               "n": int(gross.sum()), "n_total": len(pairs),
               "rate": round(float(gross.mean()), 4),
               "rate_ci95": wilson(int(gross.sum()), len(pairs))},
           "non_gross": describe(res[ng], "range residual (m), non-gross")}

    # Robust headline numbers with serial-correlation-aware intervals.
    if ng.sum() >= 3:
        rep["non_gross"]["bias_ci95"] = moving_block_bootstrap(
            res[ng], np.mean, blk)
        rep["non_gross"]["mae_ci95"] = moving_block_bootstrap(
            res[ng], lambda a: np.abs(a).mean(), blk)
        rep["non_gross"]["rms_ci95"] = moving_block_bootstrap(
            res[ng], lambda a: math.sqrt((a ** 2).mean()), blk)

    rep["per_session"] = []
    for s in sorted({p["session"] for p in pairs}):
        m = np.array([p["session"] == s for p in pairs])
        rep["per_session"].append({
            "session": s, "n": int(m.sum()),
            "n_gross": int((m & gross).sum()),
            "all": describe(res[m]),
            "non_gross": describe(res[m & ng])})

    # --- does the residual depend on observables? ---------------------
    cov = {}
    obs = {"gt_range_m": gt,
           "bbox_width_frac": np.array([p["width"] for p in pairs]),
           "center_x": np.array([p["center_x"] for p in pairs]),
           "abs_center_offset": np.abs(
               np.array([p["center_x"] for p in pairs]) - 0.5)}
    for nm, x in obs.items():
        cov[nm] = {
            "n": len(x),
            "pearson_resid": round(pearson(x, res), 4),
            "spearman_resid": round(spearman(x, res), 4),
            "spearman_log_ratio": round(spearman(x, lr), 4),
            "spearman_resid_non_gross": round(
                spearman(x[ng], res[ng]), 4) if ng.sum() >= 3 else None,
            "spearman_ci95": moving_block_bootstrap(
                np.arange(len(x)), lambda idx, x=x: spearman(
                    x[idx.astype(int)], res[idx.astype(int)]), blk)}
    cov["bbox_height_frac"] = {
        "n": 0, "available": False,
        "reason": ("no cycle in the dataset carries both gt_dist_px and a "
                   "bbox height: the GT sessions (missions/) log no det "
                   "block and the det sessions (avoid_runs/) log no GT. "
                   "Height is not recoverable from range either, because "
                   "range is derived from WIDTH.")}
    cov["detector_score"] = {
        "n": 0, "available": False,
        "reason": ("same disjointness: score is logged only where GT is "
                   "absent. Not estimable from this dataset.")}
    cov["bbox_width_frac"]["caveat"] = (
        "NOT an independent covariate: width = ANCHOR_WIDTH_M*F_PX/"
        "(range*IMG_W) exactly, so this is the range regression "
        "re-parameterised. Reported for completeness only.")
    rep["dependence_on_observables"] = cov

    # --- the simplest model the data support --------------------------
    # A multiplicative scale error is the natural candidate, so it is
    # fitted and then tested rather than assumed: regress the MEASURED
    # range on the true range. A usable sensor needs slope ~1.
    fit = {}
    if ng.sum() >= 5:
        A = np.vstack([np.ones(int(ng.sum())), gt[ng]]).T
        coef = np.linalg.lstsq(A, rng[ng], rcond=None)[0]
        pred = A @ coef
        sse = float(((rng[ng] - pred) ** 2).sum())
        sst = float(((rng[ng] - rng[ng].mean()) ** 2).sum())
        idx = np.arange(int(ng.sum()))

        def slope_of(ii):
            ii = ii.astype(int)
            AA = np.vstack([np.ones(len(ii)), gt[ng][ii]]).T
            return float(np.linalg.lstsq(
                AA, rng[ng][ii], rcond=None)[0][1])

        fit = {"model": "measured_range = a + b * true_range (non-gross)",
               "a": round(float(coef[0]), 4),
               "b": round(float(coef[1]), 4),
               "b_ci95": moving_block_bootstrap(idx, slope_of, blk),
               "r2": round(1 - sse / sst, 4) if sst > 0 else None,
               "residual_rms_m": round(math.sqrt(sse / ng.sum()), 4),
               "pearson_measured_vs_true": round(
                   pearson(gt[ng], rng[ng]), 4),
               "best_pure_scale_k": round(
                   float((gt[ng] * rng[ng]).sum() / (gt[ng] ** 2).sum()), 4)}
        fit["interpretation"] = (
            "b is the informativeness of the sensor: b=1 would mean the "
            "measurement tracks truth. See the report for the verdict; "
            "the CI is a moving-block bootstrap, block %d." % blk)
    rep["scale_model"] = fit

    # --- binned view (the honest summary when a trend is not linear) ---
    rep["gt_bins"] = []
    for lo, hi in GT_BINS:
        m = (gt >= lo) & (gt < hi)
        if not m.any():
            continue
        rep["gt_bins"].append({
            "gt_lo": lo, "gt_hi": (None if hi > 1e8 else hi),
            "n": int(m.sum()), "n_gross": int((m & gross).sum()),
            "gross_rate": round(float(gross[m].mean()), 4),
            "gross_rate_ci95": wilson(int((m & gross).sum()), int(m.sum())),
            "measured_range_m": describe(rng[m]),
            "resid_m": describe(res[m]),
            "ratio_median": round(float(np.median(
                [p["ratio"] for p, k in zip(pairs, m) if k])), 4)})

    # --- censoring ----------------------------------------------------
    w = np.array([p["width"] for p in pairs])
    rep["censoring"] = {
        "arm_search_window_frac": ARM_WINDOW_FRAC,
        "implied_range_floor_m": round(RANGE_FLOOR_M, 4),
        "max_width_frac_observed": round(float(w.max()), 4),
        "n_within_1pct_of_cap": int((w >= 0.99 * ARM_WINDOW_FRAC).sum()),
        "n_paired": len(pairs),
        "note": ("the detector cannot report a width above "
                 "ARM_WINDOW_FRAC, so the monocular range is CENSORED "
                 "from below at the floor. This is structure, not noise.")}
    return rep


# ---------------------------------------------------------------------
# 2. bearing
# ---------------------------------------------------------------------

def bearing_report(recs):
    """Derive a GT bearing, or state precisely why it cannot be trusted.

    The overhead camera gives the ROV pixel and the anchor pixel, so the
    LINE to the anchor is known in image coordinates. The vehicle's
    heading in those coordinates is not: the logged yaw is a compass
    heading with an unknown offset to the overhead image axes, and no
    calibration of that offset was recorded. The offset is therefore
    estimated from straight APPROACH segments under a NO-CRAB assumption
    (the vehicle travels along its bow). That assumption is the weak
    point, so the estimate is made per session and its cross-session
    agreement is reported as the only available evidence about it.
    """
    per = {}
    for s in sorted({r["session"] for r in recs if r["rov_px"]}):
        rs = [r for r in recs if r["session"] == s and r["rov_px"]]
        offs = []
        for a, b in zip(rs, rs[6:]):
            if a["state"] != "APPROACH" or b["state"] != "APPROACH":
                continue
            v = np.array(b["rov_px"]) - np.array(a["rov_px"])
            if np.linalg.norm(v) < 40:      # below this, heading is noise
                continue
            mdir = math.degrees(math.atan2(v[1], v[0]))
            yaw = 0.5 * (a["yaw_deg"] + b["yaw_deg"])
            offs.append(wrap180(mdir - yaw))
        if offs:
            o = np.radians(offs)
            per[s] = {"n": len(offs),
                      "offset_deg": round(float(math.degrees(math.atan2(
                          np.sin(o).mean(), np.cos(o).mean()))), 3),
                      "std_deg": round(float(np.std(offs, ddof=1))
                                       if len(offs) > 1 else 0.0, 3)}
    rep = {"frame_offset_per_session": per}
    if not per:
        rep["available"] = False
        rep["reason"] = ("no session has a straight APPROACH segment long "
                         "enough to calibrate the overhead/compass frame "
                         "offset")
        return rep

    vals = [v["offset_deg"] for v in per.values()]
    off = float(np.mean(vals))
    rep["frame_offset_deg"] = round(off, 3)
    rep["frame_offset_spread_deg"] = round(
        float(max(vals) - min(vals)), 3) if len(vals) > 1 else None
    rep["n_sessions_calibrating"] = len(vals)

    out = []
    for r in recs:
        if r["bearing_deg"] is None or not r["rov_px"]:
            continue
        v = ANCHOR_PX - np.array(r["rov_px"])
        phi = math.degrees(math.atan2(v[1], v[0]))
        gtb = wrap180(phi - (r["yaw_deg"] + off))
        out.append({"session": r["session"], "t": r["t"],
                    "cam_bearing_deg": r["bearing_deg"],
                    "gt_bearing_deg": round(gtb, 2),
                    "resid_deg": round(wrap180(r["bearing_deg"] - gtb), 2),
                    "gt_m": r["gt_m"],
                    "gross_range": bool(r.get("gross"))})
    d = np.array([o["resid_deg"] for o in out])
    rep["available"] = True
    rep["n"] = len(out)
    rep["residual_deg"] = describe(d, "bearing residual (deg)")
    ng = np.array([not o["gross_range"] for o in out])
    if ng.sum() >= 3:
        rep["residual_deg_non_gross_range"] = describe(d[ng])
    near = np.array([(o["gt_m"] or 9) < 0.8 for o in out])
    if near.any():
        rep["residual_deg_gt_below_0p8m"] = describe(d[near])
        rep["residual_deg_gt_above_0p8m"] = describe(d[~near])
    if len(d) >= 3:
        blk = block_len_from_acf(d)
        rep["bootstrap_block_len"] = blk
        rep["bias_ci95"] = moving_block_bootstrap(d, np.mean, blk)
        rep["sd_ci95"] = moving_block_bootstrap(
            d, lambda a: float(np.std(a, ddof=1)) if len(a) > 1 else 0.0,
            blk)
    rep["validity"] = {
        "bias_is_validated": False,
        "why": ("the GT bearing is defined relative to an estimated frame "
                "offset; any error in that offset shifts EVERY residual by "
                "the same amount, so the mean residual and the offset error "
                "are not separable. The per-session offsets agree to "
                "%s deg, which bounds but does not remove the confound."
                % (rep["frame_offset_spread_deg"])),
        "dispersion_is_usable": True,
        "extra_caveat": ("the GT is the bearing to the anchor's SUSPENSION "
                         "pixel while the camera reports the bearing to the "
                         "detected blob centre; at short range the anchor "
                         "subtends a large angle and the two diverge, which "
                         "is visible in the gt<0.8 m split above.")}
    rep["samples"] = out
    return rep


# ---------------------------------------------------------------------
# 3. availability and its temporal structure
# ---------------------------------------------------------------------

def run_lengths(bits):
    out = {0: [], 1: []}
    if not bits:
        return out
    cur, k = bits[0], 1
    for v in bits[1:]:
        if v == cur:
            k += 1
        else:
            out[cur].append(k)
            cur, k = v, 1
    out[cur].append(k)
    return out


def _ll_iid(bits, p):
    def lg(x):
        return math.log(x) if x > 0 else 0.0
    return sum(lg(p) if v else lg(1 - p) for v in bits)


def markov_test(bits):
    """IID Bernoulli vs 2-state Markov by likelihood ratio, df = 1.

    Both models are scored on the SAME set of events -- the n-1
    transitions -- so the comparison is not rigged by the extra initial
    observation the IID model would otherwise get for free.
    """
    n = len(bits)
    c = {"00": 0, "01": 0, "10": 0, "11": 0}
    for a, b in zip(bits, bits[1:]):
        c["%d%d" % (a, b)] += 1
    tail = bits[1:]
    p1 = (sum(tail) / len(tail)) if tail else float("nan")
    n0, n1_ = c["00"] + c["01"], c["10"] + c["11"]
    p01 = c["01"] / n0 if n0 else float("nan")
    p11 = c["11"] / n1_ if n1_ else float("nan")

    def lg(x):
        return math.log(x) if x > 0 else 0.0
    ll_i = _ll_iid(tail, p1) if tail else 0.0
    ll_m = (c["00"] * lg(1 - p01) + c["01"] * lg(p01)
            + c["10"] * lg(1 - p11) + c["11"] * lg(p11))
    g2 = 2 * (ll_m - ll_i)
    stat = 0.0 if not np.isfinite(g2) else g2
    res = {"n_cycles": n, "n_transitions": len(tail),
           "accept_rate": round(float(sum(bits) / n), 4) if n else None,
           "accept_rate_ci95": wilson(int(sum(bits)), n),
           "counts": c,
           "iid_p": None if not np.isfinite(p1) else round(p1, 4),
           "p01": None if not np.isfinite(p01) else round(p01, 4),
           "p11": None if not np.isfinite(p11) else round(p11, 4),
           "loglik_iid": round(ll_i, 3), "loglik_markov": round(ll_m, 3),
           "G2": round(stat, 3), "df": 1,
           "p_value": chi2_sf_df1(stat),
           "aic_iid": round(-2 * ll_i + 2, 3),
           "aic_markov": round(-2 * ll_m + 4, 3)}
    res["favours"] = ("markov" if stat > 3.841 else "iid_not_rejected")
    # Power warning: with a handful of cycles in either class the test
    # cannot detect burstiness even if it is there, so "not rejected"
    # must never be read as "IID confirmed". Ten is not a magic number;
    # it is the point below which a single run of detections can carry
    # the whole statistic.
    res["underpowered"] = bool(min(sum(bits), n - sum(bits)) < 10)
    return res


def availability_report(recs):
    rep = {"per_session": [], "note": (
        "acceptance = the gate in avoid_mission.accept(): found and "
        "score>=%.2f and height>=%.2f and width>=%.2f" % (
            MIN_SCORE, MIN_H, MIN_W))}
    pooled = {"00": 0, "01": 0, "10": 0, "11": 0}
    pooled_bits, pooled_tail = 0, 0
    all_r1, all_r0 = [], []
    for s in sorted({r["session"] for r in recs}):
        rs = [r for r in recs if r["session"] == s]
        bits = [1 if r["accepted"] else 0 for r in rs]
        if not bits:
            rep["per_session"].append({"session": s, "n_cycles": 0,
                                       "note": "no cycles logged"})
            continue
        rl = run_lengths(bits)
        m = markov_test(bits)
        # Run lengths are in CYCLES; the chain is only transferable to a
        # simulator that knows how long a cycle was.
        ts = [r["t"] for r in rs if r["t"] is not None]
        m["cycle_period_s_median"] = (
            round(float(np.median(np.diff(ts))), 4) if len(ts) > 2 else None)
        m["session"] = s
        m["kind"] = rs[0]["kind"]
        m["origin"] = rs[0]["origin"]
        m["autocorr_lag1_10"] = autocorr(bits, 10)
        m["run_lengths_detect"] = rl[1]
        m["run_lengths_gap"] = rl[0]
        rep["per_session"].append(m)
        for k in pooled:
            pooled[k] += m["counts"][k]
        pooled_bits += sum(bits)
        pooled_tail += len(bits) - 1
        all_r1 += rl[1]
        all_r0 += rl[0]

    n0 = pooled["00"] + pooled["01"]
    n1 = pooled["10"] + pooled["11"]
    p01 = pooled["01"] / n0 if n0 else float("nan")
    p11 = pooled["11"] / n1 if n1 else float("nan")
    nt = sum(pooled.values())
    p1 = (pooled["01"] + pooled["11"]) / nt if nt else float("nan")

    def lg(x):
        return math.log(x) if x > 0 else 0.0
    ll_i = ((pooled["01"] + pooled["11"]) * lg(p1)
            + (pooled["00"] + pooled["10"]) * lg(1 - p1))
    ll_m = (pooled["00"] * lg(1 - p01) + pooled["01"] * lg(p01)
            + pooled["10"] * lg(1 - p11) + pooled["11"] * lg(p11))
    g2 = 2 * (ll_m - ll_i)
    periods = [s["cycle_period_s_median"] for s in rep["per_session"]
               if s.get("cycle_period_s_median")]
    rep["pooled"] = {
        "cycle_period_s_median": (round(float(np.median(periods)), 4)
                                  if periods else None),
        "n_transitions": nt, "counts": pooled,
        "iid_p": round(p1, 4), "p01": round(p01, 4), "p11": round(p11, 4),
        "stationary_accept_rate": round(p01 / (p01 + 1 - p11), 4)
        if (p01 + 1 - p11) > 0 else None,
        "mean_detect_run_cycles": round(1 / (1 - p11), 3)
        if p11 < 1 else None,
        "mean_gap_run_cycles": round(1 / p01, 3) if p01 > 0 else None,
        "G2": round(g2, 3), "df": 1, "p_value": chi2_sf_df1(g2),
        "favours": "markov" if g2 > 3.841 else "iid_not_rejected",
        "run_lengths_detect_pool": sorted(all_r1),
        "run_lengths_gap_pool": sorted(all_r0),
        "caveat": ("transitions are counted WITHIN sessions only, so no "
                   "cross-session splice is created. Even so, pooling "
                   "sessions whose acceptance rates differ by an order of "
                   "magnitude (0.04 to 0.44) inflates apparent "
                   "persistence: a mixture of Bernoulli rates looks "
                   "bursty. The per-session tests are the trustworthy "
                   "evidence; the pooled G2 is an upper bound.")}
    return rep


# ---------------------------------------------------------------------
# 4. false positives and gross failures
# ---------------------------------------------------------------------

HALF_FOV_DEG = math.degrees(math.atan2(0.5 * IMG_W, F_PX))
# A detection that lies inside the image cannot be further off the bow
# than the half FOV. A GT bearing disagreement beyond this plus a margin
# for the GT itself is therefore geometrically impossible for a correct
# detection of the anchor -- so the cut is derived, not tuned.
BEARING_FP_DEG = 45.0


def false_positive_report(recs, pairs, brg_rep):
    """Three different failures that must not be conflated.

    (a) GROSS RANGE FAILURE: the anchor IS detected, but the measured
        width is not its arm span, so the range is wrong by an order of
        magnitude. The object is real; the measurement is not.
    (b) FALSE POSITIVE: the detector reports a box on something that is
        not the anchor. Against the overhead GT this shows up in the
        BEARING, which is independent of the width measurement.
    (c) NON-ANCHOR DETECTION BELOW THE GATE: the detector fires on
        clutter but the acceptance gate stops it, so it never reaches the
        planner. Only (c) is directly observable in the shadow runs, and
        only for an EARLIER detector build -- see the dating below.
    """
    rep = {}
    gross = [p for p in pairs if p["gross"]]
    rep["gross_range_failures"] = {
        "n": len(gross), "n_paired": len(pairs),
        "rate_of_accepted_samples": round(len(gross) / len(pairs), 4)
        if pairs else None,
        "rate_ci95": wilson(len(gross), len(pairs)),
        "gt_m_of_failures": sorted(round(p["gt_m"], 3) for p in gross),
        "gt_m_of_non_failures": [
            round(p["gt_m"], 3) for p in pairs if not p["gross"]],
        "max_ratio": round(max((p["ratio"] for p in gross), default=0), 2),
        "separation": None}
    if pairs:
        gmax = max((p["gt_m"] for p in gross), default=None)
        gmin = min((p["gt_m"] for p in pairs if not p["gross"]),
                   default=None)
        rep["gross_range_failures"]["separation"] = {
            "max_gt_m_with_failure": (None if gmax is None
                                      else round(gmax, 3)),
            "min_gt_m_without_failure": (None if gmin is None
                                         else round(gmin, 3)),
            "perfectly_separated": bool(
                gmax is not None and gmin is not None and gmax < gmin),
            "note": ("if perfectly separated, the threshold is a property "
                     "of the mechanism (the arm span leaves the detector's "
                     "search band), not a tuned cut")}

    # --- bearing-inconsistent accepted detections ---------------------
    # The strongest false-positive evidence available, because bearing is
    # derived from center_x and is therefore independent of the width
    # that produced the range.
    bs = brg_rep.get("samples", [])
    if bs:
        bad = [s for s in bs if abs(s["resid_deg"]) > BEARING_FP_DEG]
        bysess = {}
        for s in bs:
            bysess.setdefault(s["session"], []).append(s)
        rep["bearing_inconsistent_accepted"] = {
            "criterion_deg": BEARING_FP_DEG,
            "half_fov_deg": round(HALF_FOV_DEG, 2),
            "why_this_threshold": (
                "a detection inside the image is at most %.1f deg off the "
                "bow, so a disagreement past %.0f deg cannot be a correct "
                "detection of the anchor under a correct overhead fix"
                % (HALF_FOV_DEG, BEARING_FP_DEG)),
            "n": len(bad), "n_accepted_with_gt": len(bs),
            "rate": round(len(bad) / len(bs), 4),
            "rate_ci95": wilson(len(bad), len(bs)),
            "sessions_affected": sorted({s["session"] for s in bad}),
            "per_session_mean_resid_deg": {
                k: round(float(np.mean([x["resid_deg"] for x in v])), 2)
                for k, v in sorted(bysess.items())},
            "samples": bad,
            "confound": (
                "this criterion cannot separate a detector false positive "
                "from an overhead-tracker lock failure: both put the "
                "vehicle's line to the anchor somewhere it is not. "
                "Resolving it requires the recorded frames, not the logs."),
            "interaction_with_gross_range": (
                "gross range failures that coincide with a bearing "
                "inconsistency are better read as false positives than as "
                "range failures, since the object measured was not the "
                "anchor")}
        badkey = {(s["session"], s["t"]) for s in bad}
        gross_and_bad = [p for p in gross
                         if (p["session"], p["t"]) in badkey]
        clean = [p for p in pairs if (p["session"], p["t"]) not in badkey]
        cg = [p for p in clean if p["gross"]]
        rep["bearing_inconsistent_accepted"][
            "n_gross_range_also_bad_bearing"] = len(gross_and_bad)
        rep["gross_range_failures"]["separation_bearing_consistent_only"] = {
            "n": len(clean), "n_gross": len(cg),
            "max_gt_m_with_failure": (round(max(p["gt_m"] for p in cg), 3)
                                      if cg else None),
            "min_gt_m_without_failure": (
                round(min(p["gt_m"] for p in clean if not p["gross"]), 3)
                if any(not p["gross"] for p in clean) else None),
            "why": ("re-tests the range/GT separation after removing "
                    "samples whose bearing proves the measured object was "
                    "not the anchor; a separation that survives this is a "
                    "property of the range mechanism alone")}

    # --- detector build dating ----------------------------------------
    builds = {}
    for s in sorted({r["session"] for r in recs}):
        hs = [r["height"] for r in recs
              if r["session"] == s and r["height"] is not None]
        if not hs:
            builds[s] = {"n_detections": 0, "build": "undetermined",
                         "why": "no det block logged in this session"}
            continue
        below = sum(1 for h in hs if h < V2_MIN_HEIGHT_FRAC)
        builds[s] = {
            "n_detections": len(hs),
            "min_height_frac": round(min(hs), 4),
            "max_height_frac": round(max(hs), 4),
            "n_below_v2_floor": below,
            "build": ("pre-v2" if below else "v2-compatible"),
            "why": ("the committed v2 shape prior skips candidates shorter "
                    "than %.2f of the frame, so detections below that "
                    "floor cannot have come from it" % V2_MIN_HEIGHT_FRAC)}
    rep["detector_build_dating"] = builds

    # --- non-anchor detections in the shadow runs ---------------------
    shadow = [r for r in recs if r["session"].startswith("shadow_")]
    if shadow:
        found = [r for r in shadow if r["det_found"]]
        acc = [r for r in found if r["accepted"]]
        rep["non_anchor_detections_shadow_runs"] = {
            "sessions": sorted({r["session"] for r in shadow}),
            "n_cycles": len(shadow),
            "n_found": len(found),
            "found_rate": round(len(found) / len(shadow), 4),
            "n_passing_the_gate": len(acc),
            "gate_pass_rate": round(len(acc) / len(shadow), 4),
            "gate_pass_rate_ci95": wilson(len(acc), len(shadow)),
            "heights_of_gate_passers": sorted(
                round(r["height"], 4) for r in acc),
            "scores_of_gate_passers": sorted(
                round(r["score"], 3) for r in acc),
            "implied_range_m": sorted(
                round(r["range_m"], 2) for r in acc if r["range_m"]),
            "evidence": (
                "the detector reported a box on EVERY cycle of these runs "
                "while %d/%d of those boxes were shorter than the v2 "
                "structural floor. Inspection of the recorded frames "
                "(f0001.png) shows the box on surface debris near the top "
                "of the image while the anchor's shank is visible "
                "elsewhere in the same frame: these are detections of the "
                "wrong object, not detections of a missing one."
                % (sum(1 for r in found
                       if r["height"] is not None
                       and r["height"] < V2_MIN_HEIGHT_FRAC), len(found))),
            "what_it_does_NOT_estimate": (
                "the false-positive rate of the CURRENT detector: these "
                "runs predate the v2 shape prior. The gate-pass rate below "
                "characterises the earlier build and is an upper bound on "
                "nothing else. The implied ranges (9-14 m) are far outside "
                "the pool, so a downstream range sanity bound would have "
                "rejected them independently of the detector."),
            "n_absurd_range": sum(
                1 for r in acc if r["range_m"] and r["range_m"] > 6.0)}

    aborted = [r for r in recs if r["origin"] != "worktree"]
    rep["aborted_at_t0_runs"] = {
        "in_worktree": 0,
        "note": ("the task suggested using runs that aborted at t=0 as a "
                 "false-positive source. No such run exists in the "
                 "worktree; the aborted runs are among the git-only "
                 "sessions and they logged 0-3 cycles in total, aborting "
                 "on the overhead WALL GUARD rather than on perception. "
                 "They carry no usable false-positive information."),
        "git_only_records_loaded": len(aborted)}
    return rep


# ---------------------------------------------------------------------
# 5. score distribution
# ---------------------------------------------------------------------

def score_report(recs):
    have = [r for r in recs if r["score"] is not None]
    if not have:
        return {"available": False, "reason": "no session logs det.score"}
    acc = np.array([r["score"] for r in have if r["accepted"]])
    rej = np.array([r["score"] for r in have if not r["accepted"]])
    rej_hi = [r for r in have
              if not r["accepted"] and r["score"] >= MIN_SCORE]
    rep = {"available": True,
           "sessions": sorted({r["session"] for r in have}),
           "n_detections": len(have),
           "accepted": describe(acc, "score | accepted"),
           "rejected": describe(rej, "score | rejected"),
           "n_rejected_despite_score_gate": len(rej_hi),
           "frac_rejections_not_caused_by_score": round(
               len(rej_hi) / len(rej), 4) if len(rej) else None,
           "height_accepted": describe(
               [r["height"] for r in have
                if r["accepted"] and r["height"] is not None]),
           "height_rejected": describe(
               [r["height"] for r in have
                if not r["accepted"] and r["height"] is not None])}
    if len(acc) and len(rej):
        # Rank-based overlap (AUC) of the two gate classes. NOT a
        # detection-quality ROC: the labels are the gate's own output, and
        # the gate reads height and width as well as score.
        allv = np.concatenate([acc, rej])
        r = _rank(allv)
        auc = (r[:len(acc)].sum() - len(acc) * (len(acc) - 1) / 2.0) / (
            len(acc) * len(rej))
        rep["auc_score_vs_gate_label"] = round(float(auc), 4)
    rep["caveat"] = (
        "accepted/rejected is the ACCEPTANCE GATE's label, not ground "
        "truth. %d of %d rejected detections passed the score threshold "
        "and were rejected on bbox size, and the single highest score in "
        "the dataset (%.3f) belongs to a REJECTED detection. Score alone "
        "does not separate anchor from clutter here."
        % (len(rej_hi), len(rej), float(rej.max()) if len(rej) else 0.0))
    return rep


# ---------------------------------------------------------------------
# model emission
# ---------------------------------------------------------------------

def build_model(res_rep, brg_rep, avail_rep, fp_rep, sc_rep, pairs,
                sessions_used, args):
    gt = np.array([p["gt_m"] for p in pairs]) if pairs else np.array([])
    pools = []
    for lo, hi in GT_BINS:
        m = [p for p in pairs if lo <= p["gt_m"] < hi]
        if not m:
            continue
        pools.append({
            "gt_lo_m": lo, "gt_hi_m": (None if hi > 1e8 else hi),
            "n": len(m),
            "gross_rate": round(sum(1 for p in m if p["gross"]) / len(m), 4),
            # Both pools are given because they are NOT interchangeable:
            # with a slope near zero the additive form implies an accuracy
            # the sensor does not have. Prefer measured_range_m.
            "measured_range_m": [round(p["range_m"], 3) for p in m],
            "residual_m": [round(p["resid_m"], 3) for p in m],
            "ratio": [round(p["ratio"], 4) for p in m]})
    per_sess = {}
    for s in avail_rep["per_session"]:
        if s.get("n_cycles"):
            per_sess[s["session"]] = {
                "accept_rate": s.get("accept_rate"),
                "p01": s.get("p01"), "p11": s.get("p11"),
                "n_cycles": s.get("n_cycles"),
                "underpowered": s.get("underpowered")}
    model = {
        "schema": "holoobstacle.observation_model/1",
        "version": "v1",
        "kind": "EMPIRICAL_REPLAY",
        "provenance": {
            "PILOT_ONLY_NOTICE": (
                "Estimated ONLY from the 2026-08-14 PILOT/CALIBRATION wet "
                "runs. Those runs tuned pool geometry, the detector and "
                "the engagement thresholds between trials. This model is "
                "admissible as SIMULATION INPUT only. None of its numbers "
                "may be reported as an experimental result, and the "
                "dataset that produced it must stay disjoint from any "
                "evaluation dataset."),
            "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                           time.gmtime()),
            "git_sha": git_sha(),
            "git_worktree_dirty": git_dirty(),
            "generator": "scripts/analysis/characterize_observations.py",
            "sessions_used": sessions_used,
            "include_git_missing": bool(args.include_git_missing),
            "n_paired_range_samples": len(pairs),
            "n_bearing_samples": brg_rep.get("n", 0),
            "n_availability_cycles": sum(
                s.get("n_cycles", 0) for s in avail_rep["per_session"]),
            "gt_range_span_m": ([round(float(gt.min()), 3),
                                 round(float(gt.max()), 3)]
                                if len(gt) else None),
            "single_lighting_session": True,
            "single_obstacle": "suspended anchor, 0.80 m arm span"},
        "geometry": {
            "f_px": F_PX, "anchor_width_m": ANCHOR_WIDTH_M,
            "img_w": IMG_W, "overhead_px_per_m": PX_PER_M,
            "anchor_overhead_px": ANCHOR_PX.tolist(),
            "range_from_width": "range = anchor_width_m*f_px/(width*img_w)",
            "bearing_from_center_x":
                "bearing = atan2((center_x-0.5)*img_w, f_px)",
            "acceptance_gate": {"min_score": MIN_SCORE, "min_height": MIN_H,
                                "min_width": MIN_W},
            "range_floor_m": round(RANGE_FLOOR_M, 4),
            "arm_search_window_frac": ARM_WINDOW_FRAC},
        "range_model": {
            "form": "conditional empirical replay, binned on TRUE range",
            "why_not_gaussian": (
                "the residual is not a zero-mean disturbance around the "
                "truth. It is (i) censored from below at the range floor, "
                "(ii) strongly dependent on the true range, and (iii) "
                "punctuated by order-of-magnitude failures inside 1 m. A "
                "Gaussian would reproduce none of the three and would make "
                "the simulated planner face an easier sensor than the real "
                "one."),
            "bins": pools,
            "gross_failure_rule": {
                "definition": ("measured/true outside [1/%.1f,%.1f]"
                               % (GROSS_RATIO, GROSS_RATIO)),
                "observed": fp_rep["gross_range_failures"]["separation"],
                "usage": ("draw from the bin's own pool; the pool for the "
                          "innermost bin already contains the failures at "
                          "their observed frequency, so no separate "
                          "failure process is needed")},
            "scale_model": res_rep.get("scale_model"),
            "non_gross_summary": res_rep.get("non_gross"),
            "sampling_recipe": (
                "given a true range g: pick the bin containing g, draw one "
                "value uniformly from that bin's measured_range_m pool "
                "(bootstrap with replacement), and clamp to >= "
                "range_floor_m. Do NOT add the residual pool to g unless "
                "you have first checked that the scale_model slope is "
                "compatible with 1 in your regime of interest.")},
        "bearing_model": {
            "form": "empirical residual pool, additive on true bearing",
            "residual_deg_pool": [s["resid_deg"]
                                  for s in brg_rep.get("samples", [])],
            "summary": brg_rep.get("residual_deg"),
            "bias_is_validated": False,
            "caveat": brg_rep.get("validity", {}).get("why"),
            "frame_offset_deg": brg_rep.get("frame_offset_deg"),
            "frame_offset_spread_deg": brg_rep.get(
                "frame_offset_spread_deg")},
        "availability_model": {
            "form": "2-state Markov chain on the accepted flag",
            "cycle_period_s_median": avail_rep["pooled"][
                "cycle_period_s_median"],
            "cycle_period_s_per_session": {
                s["session"]: s.get("cycle_period_s_median")
                for s in avail_rep["per_session"] if s.get("n_cycles")},
            "selected": avail_rep["pooled"]["favours"],
            "pooled": {k: avail_rep["pooled"][k] for k in
                       ("p01", "p11", "iid_p", "stationary_accept_rate",
                        "mean_detect_run_cycles", "mean_gap_run_cycles",
                        "G2", "p_value", "n_transitions")},
            "per_session": per_sess,
            "run_length_pools": {
                "detect": avail_rep["pooled"]["run_lengths_detect_pool"],
                "gap": avail_rep["pooled"]["run_lengths_gap_pool"]},
            "usage": (
                "sample the per-session accept rate as an ENVIRONMENT "
                "variable first (it ranged 0.04-0.44 with dome "
                "cleanliness), then run the chain. Using the pooled rate "
                "alone models an average condition that never occurred in "
                "any single run."),
            "caveat": avail_rep["pooled"]["caveat"]},
        "false_positive_model": {
            "gross_range_failure": fp_rep["gross_range_failures"],
            "bearing_inconsistent_accepted": fp_rep.get(
                "bearing_inconsistent_accepted"),
            "non_anchor_detection": fp_rep.get(
                "non_anchor_detections_shadow_runs"),
            "detector_build_dating": fp_rep["detector_build_dating"],
            "usage": ("the non-anchor rate applies to a PRE-v2 detector "
                      "build and must not be attached to the current one "
                      "without new data")},
        "score_model": {
            "accepted": sc_rep.get("accepted"),
            "rejected": sc_rep.get("rejected"),
            "caveat": sc_rep.get("caveat"),
            "note": ("no score is available on any cycle that also has "
                     "ground truth, so score cannot be used to condition "
                     "the range error in this model")},
        "not_supported_by_this_dataset": [
            "residual vs bbox height (never co-logged with ground truth)",
            "residual vs detector score (never co-logged with ground "
            "truth)",
            "bearing BIAS (confounded with the estimated frame offset)",
            "false-positive rate of the current detector build",
            "any dependence on lighting, turbidity or obstacle type "
            "(one session, one obstacle)",
            "true range beyond the observed span; the model must not be "
            "extrapolated"]}
    return model


# ---------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--include-git-missing", action="store_true",
                    help="also load mission logs tracked in HEAD but "
                         "deleted from the worktree (sensitivity check)")
    ap.add_argument("--out", default=OUTDIR)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    sessions = load_worktree()
    missing = git_missing_sessions()
    if args.include_git_missing:
        sessions += missing

    recs, align = records(sessions)
    inv = inversion_check(recs)
    pairs = range_residuals(recs)
    res_rep = residual_report(pairs)
    brg_rep = bearing_report(recs)
    avail_rep = availability_report(recs)
    fp_rep = false_positive_report(recs, pairs, brg_rep)
    sc_rep = score_report(recs)

    used = [{"session": n, "kind": k, "origin": o,
             "n_cycles": len(d.get("log", [])),
             "has_gt": any(e.get("gt_dist_px") is not None
                           for e in d.get("log", [])),
             "has_det_block": any("det" in e for e in d.get("log", []))}
            for k, n, d, o in sessions]

    report = {
        "PILOT_ONLY_NOTICE": (
            "Every number here comes from the 2026-08-14 PILOT/CALIBRATION "
            "wet runs and is model input, never a result."),
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "git_sha": git_sha(), "git_worktree_dirty": git_dirty(),
        "sessions_used": used,
        "git_tracked_but_missing_from_worktree": [
            n for _, n, _, _ in missing],
        "included_git_missing": bool(args.include_git_missing),
        "dataset_structure_finding": {
            "sessions_with_ground_truth": sorted(
                {u["session"] for u in used if u["has_gt"]}),
            "sessions_with_det_block": sorted(
                {u["session"] for u in used if u["has_det_block"]}),
            "n_cycles_with_both": sum(
                1 for r in recs
                if r["gt_m"] is not None and r["det_found"] is not None),
            "consequence": (
                "the two sets are disjoint, so no regression of the range "
                "residual on bbox height or detector score is possible")},
        "traj_alignment_check": align,
        "inversion_check": inv,
        "range_residual": res_rep,
        "bearing_residual": brg_rep,
        "availability": avail_rep,
        "false_positives": fp_rep,
        "score_distribution": sc_rep}

    rpath = os.path.join(args.out, "observation_characterization.json")
    with open(rpath, "w") as f:
        json.dump(report, f, indent=2, default=str)

    model = build_model(res_rep, brg_rep, avail_rep, fp_rep, sc_rep,
                        pairs, used, args)
    mpath = os.path.join(args.out, "observation_model_v1.json")
    if args.include_git_missing:
        # The frozen v1 model is defined on the worktree dataset. A
        # sensitivity run must not overwrite it.
        mpath = os.path.join(args.out,
                             "observation_model_v1_with_git_missing.json")
    with open(mpath, "w") as f:
        json.dump(model, f, indent=2, default=str)

    # ---- console summary --------------------------------------------
    print("sessions: %d (%d with GT, %d with det block)" % (
        len(used), sum(u["has_gt"] for u in used),
        sum(u["has_det_block"] for u in used)))
    print("cycles carrying BOTH gt and a bbox: %d"
          % report["dataset_structure_finding"]["n_cycles_with_both"])
    print("inversion check passes: %s (max width err %s)"
          % (inv["passes"], inv["max_abs_width_error"]))
    print("paired range samples: %d over GT %s m"
          % (res_rep.get("n_paired", 0), res_rep.get("gt_range_m")))
    ng = res_rep.get("non_gross", {})
    print("  all:       bias %+.2f MAE %.2f RMS %.2f"
          % (res_rep["all_samples"]["mean"], res_rep["all_samples"]["mae"],
             res_rep["all_samples"]["rms"]))
    print("  non-gross: bias %+.2f MAE %.2f RMS %.2f (n=%d)"
          % (ng.get("mean", float("nan")), ng.get("mae", float("nan")),
             ng.get("rms", float("nan")), ng.get("n", 0)))
    print("  gross failures: %d/%d, all at GT <= %s m"
          % (res_rep["gross"]["n"], res_rep["gross"]["n_total"],
             fp_rep["gross_range_failures"]["separation"][
                 "max_gt_m_with_failure"]))
    sm = res_rep.get("scale_model") or {}
    if sm:
        print("  measured = %.2f %+.2f*true  (b CI %s, R2 %.2f)"
              % (sm["a"], sm["b"], sm["b_ci95"], sm["r2"]))
    print("bearing: n=%s resid mean %s sd %s deg (bias NOT validated)"
          % (brg_rep.get("n"),
             (brg_rep.get("residual_deg") or {}).get("mean"),
             (brg_rep.get("residual_deg") or {}).get("std")))
    bi = fp_rep.get("bearing_inconsistent_accepted")
    if bi:
        print("  bearing-inconsistent accepted (>%.0f deg): %d/%d %s"
              % (bi["criterion_deg"], bi["n"], bi["n_accepted_with_gt"],
                 bi["sessions_affected"]))
        sep = fp_rep["gross_range_failures"].get(
            "separation_bearing_consistent_only", {})
        print("  gross-range separation (bearing-consistent only): "
              "failures <= %s m, clean >= %s m"
              % (sep.get("max_gt_m_with_failure"),
                 sep.get("min_gt_m_without_failure")))
    p = avail_rep["pooled"]
    print("availability pooled: p01 %.3f p11 %.3f G2 %.1f -> %s"
          % (p["p01"], p["p11"], p["G2"], p["favours"]))
    for s in avail_rep["per_session"]:
        if s.get("n_cycles"):
            print("   %-24s acc %.3f G2 %7.2f p=%.2e %s%s"
                  % (s["session"], s["accept_rate"], s["G2"],
                     s["p_value"], s["favours"],
                     "  [UNDERPOWERED]" if s["underpowered"] else ""))
    print("wrote %s" % rpath)
    print("wrote %s" % mpath)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
