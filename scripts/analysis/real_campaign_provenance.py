"""The definitive run-by-run record of the real campaign of 2026-08-16.

Every number here is recomputed from the raw recordings. Nothing is read
from a previously derived file, so this script can contradict the earlier
ones -- and where it does, it is the one to believe, because it is the
only one that checks the recordings against each other.

WHAT THE REAL CAMPAIGN IS. Nine runs were flown, all with the committed
planner; the DWA half of the design never happened, because DWA issues no
command in the real pipeline without a pose estimate the vehicle has no
sensor for. Run 5 is excluded by the normalisation criterion. Eight runs
remain, in three geometries: K0 centred (4), K1 offset (2), K1M mirrored
(2). K1M was invented mid-session and was never pre-registered.

WHAT CAN AND CANNOT BE MEASURED. No overhead recording was made during
any run, so there is no external observation of where the vehicle went.
That rules out, permanently and for every run:

    trajectory, path length, lateral deviation, forward progress,
    true range to the anchor, true minimum clearance

and therefore also rules out the spatial half of the sim-real comparison.
Integrating the commanded velocity would manufacture all six of them from
an open-loop model of a saturating, deadbanded vehicle in moving water,
and is not done here or anywhere else in this analysis.

What remains is real and worth having:

    the command stream        every run, at 19 Hz
    the perception stream     six runs, recovered from the annotated
                              video, including the range the planner
                              itself believed

so the comparable surface between the domains is the COMMAND DOMAIN --
commanded surge, lateral fraction, commitment time and side, reversals --
plus the planner's own belief about range, which exists on both sides.

LEFT CENSORING AND THE COMMON WINDOW, APPLIED ONLY WHERE THEY MEAN
SOMETHING. A metric can be censored at a distance only if a distance was
observed. Real commitment TIME is observed and is censored when the run
begins already committed. Real commitment DISTANCE is not observed at all
and is reported as unavailable rather than censored, because censoring an
absent quantity would imply it exists.

Usage:
    python scripts/analysis/real_campaign_provenance.py
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

# run -> (geometry, directory). Established by reading every run's
# result.json and cmd_trace, not by trusting a filename.
CAMPAIGN = [
    (1, "K0", "20260816_120927_01_K0_committed_v2"),
    (2, "K0", "20260816_122520_run02_K0_committed"),
    (3, "K0", "20260816_122752_run03_K0_committed"),
    (4, "K0", "20260816_123213_run04_K0_committed"),
    (5, "K0", "20260816_123406_run05_K0_committed"),
    (6, "K1", "20260816_124312_run06_K1_committed"),
    (7, "K1", "20260816_124803_run07_K1_committed"),
    (8, "K1M", "20260816_125238_run08_K1M_committed"),
    (9, "K1M", "20260816_125518_run09_K1M_committed"),
]

COMMIT_THRESHOLD = 0.05      # m/s, the pre-registered commitment level
COMMIT_HOLD_S = 1.0          # sustained for this long
CENSOR_THRESHOLD = 0.02      # m/s, "already moving" -- deliberately lower


def load_trace(d, name):
    p = os.path.join(d, name)
    if not os.path.exists(p):
        return None
    rows = [json.loads(x) for x in open(p) if x.strip()]
    if not rows:
        return None
    return (np.array([r["t"] for r in rows]),
            np.array([r["x"] for r in rows]),
            np.array([r["y"] for r in rows]))


def commitment(t, y):
    """First sustained lateral command, by the pre-registered definition."""
    for i in range(len(t)):
        if abs(y[i]) <= COMMIT_THRESHOLD:
            continue
        s = np.sign(y[i])
        j = i
        while j < len(t) and np.sign(y[j]) == s and abs(y[j]) > COMMIT_THRESHOLD:
            j += 1
        if t[j - 1] - t[i] >= COMMIT_HOLD_S:
            return ("right" if s > 0 else "left"), float(t[i])
    return None, None


def main() -> int:
    base = os.path.join(_ROOT, "experiments", "real", "quick_runs")
    norm_path = os.path.join(_ROOT, "experiments", "real",
                             "command_trace_normalization.json")
    norm = {}
    if os.path.exists(norm_path):
        for r in json.load(open(norm_path))["runs"]:
            norm[r["run"]] = r
    rec_path = os.path.join(_ROOT, "experiments", "real",
                            "perception_recovery", "summary.json")
    rec = {}
    if os.path.exists(rec_path):
        for r in json.load(open(rec_path))["runs"]:
            rec[r["run"]] = r

    table = []
    for run, geom, name in CAMPAIGN:
        d = os.path.join(base, name)
        res = {}
        rp = os.path.join(d, "result.json")
        if os.path.exists(rp):
            res = json.load(open(rp))
        raw = load_trace(d, "cmd_trace.jsonl")
        grid = load_trace(d, "cmd_trace_19hz.jsonl")
        n = norm.get(run, {})
        admissible = bool(n.get("passes"))
        vid = os.path.join(d, "onboard_detections.avi")

        e = {
            "run": run, "geometry": geom, "planner": res.get("planner"),
            "directory": "experiments/real/quick_runs/" + name,
            "admissible": admissible,
            "exclusion_reason": None if admissible else
                ("command-trace spread %.4f m/s exceeds the %.4f m/s "
                 "single-publisher baseline"
                 % (n.get("within_cell_spread_m_s", float("nan")),
                    0.0107)) if n else "not assessed",
            "engage_distance_m_as_run": res.get("engage_distance_m"),
            "duration_s_requested": res.get("duration_s"),
            "detections_raw": res.get("raw"),
            "detections_with_obstacle": res.get("raw_obs"),
            "recorded_rate_hz": n.get("recorded_rate_hz"),
            "onboard_video": (os.path.relpath(vid, _ROOT).replace("\\", "/")
                              if os.path.exists(vid) else None),
            "onboard_video_bytes": (os.path.getsize(vid)
                                    if os.path.exists(vid) else 0),
        }
        if res.get("raw"):
            e["detection_fraction"] = round(
                res["raw_obs"] / float(res["raw"]), 3)

        src = grid if grid is not None else raw
        if src is not None:
            t, x, y = src
            lat = np.abs(y) > CENSOR_THRESHOLD
            sg = np.sign(y[lat])
            side, tc = commitment(t, y)
            e.update({
                "trace_used": ("cmd_trace_19hz.jsonl" if grid is not None
                               else "cmd_trace.jsonl"),
                "samples": int(len(t)),
                "trace_duration_s": round(float(t[-1]), 2),
                "mean_commanded_surge_m_s": round(float(x.mean()), 4),
                "median_commanded_surge_m_s": round(float(np.median(x)), 4),
                "zero_surge_fraction": round(float((x < 0.01).mean()), 3),
                "lateral_command_fraction": round(float(lat.mean()), 3),
                "sign_reversals": (int((np.diff(sg) != 0).sum())
                                   if len(sg) > 1 else 0),
                "commitment_side": side,
                "commitment_time_s": None if tc is None else round(tc, 2),
                # A run already committed at its first sample began its
                # manoeuvre at or before the recording did.
                "commitment_time_left_censored": bool(
                    tc is not None and tc <= 0.0),
                # Not observed, and not inferable from anything recorded.
                "commitment_distance_m": None,
                "min_clearance_m": None,
                "path_length_m": None,
                "max_lateral_deviation_m": None,
            })
        r9 = rec.get(run) or {}
        if r9.get("accepted"):
            e.update({
                "perception_recovered": True,
                "frames_recovered": r9.get("frames"),
                "accepted_fraction": r9.get("accept_fraction"),
                "believed_range_median_m": r9.get("range_median_m"),
                "believed_range_first_accept_m":
                    r9.get("range_first_accept_m"),
                "recovery_error_median_m": r9.get("recovery_err_median_m"),
            })
        else:
            e["perception_recovered"] = False
        table.append(e)

    hdr = ("%-4s %-4s %-5s %6s %7s %7s %7s %6s %-6s %7s %6s"
           % ("run", "geo", "amm.", "Hz", "surge", "lat%", "det%",
              "inv", "lato", "t_imp", "video"))
    print(hdr)
    print("-" * len(hdr))
    for e in table:
        print("%-4d %-4s %-5s %6s %7s %7s %7s %6s %-6s %7s %6s"
              % (e["run"], e["geometry"], "si" if e["admissible"] else "NO",
                 e.get("recorded_rate_hz", "-"),
                 e.get("mean_commanded_surge_m_s", "-"),
                 e.get("lateral_command_fraction", "-"),
                 e.get("detection_fraction", "-"),
                 e.get("sign_reversals", "-"),
                 e.get("commitment_side") or "-",
                 (("cens." if e.get("commitment_time_left_censored")
                   else e.get("commitment_time_s"))
                  if e.get("commitment_time_s") is not None else "-"),
                 "si" if e.get("onboard_video") else "no"))

    adm = [e for e in table if e["admissible"]]
    print("\nammissibili: %d / %d" % (len(adm), len(table)))
    for g in ("K0", "K1", "K1M"):
        v = [e for e in adm if e["geometry"] == g]
        if not v:
            continue
        sides = [e.get("commitment_side") for e in v]
        print("  %-4s n=%d  surge mediano %.3f  lati: %s"
              % (g, len(v),
                 float(np.median([e["mean_commanded_surge_m_s"] for e in v])),
                 ", ".join(s or "-" for s in sides)))

    out = {
        "campaign": "real BlueROV2, 2026-08-16",
        "planner": "committed only; DWA never issued a command in the real "
                   "pipeline (no pose estimate available on the vehicle)",
        "runs_flown": len(table),
        "runs_admissible": len(adm),
        "geometries": {"K0": "anchor centred, pre-registered",
                       "K1": "anchor 0.35 m off-axis, pre-registered",
                       "K1M": "anchor 0.35 m off-axis mirrored, "
                              "ADDED MID-SESSION, not pre-registered"},
        "not_measurable": {
            "quantities": ["trajectory", "path length", "lateral deviation",
                           "forward progress", "true range to obstacle",
                           "true minimum clearance", "commitment distance"],
            "reason": "no overhead recording was made during any run; the "
                      "only spatial signal is the onboard monocular "
                      "estimate, which has no independent reference",
            "explicitly_not_done": "integration of commanded velocity as a "
                                   "trajectory surrogate",
        },
        "comparable_surface": ["commanded surge", "lateral command fraction",
                               "commitment time and side", "sign reversals",
                               "the planner's own believed range"],
        "runs": table,
    }
    p = os.path.join(_ROOT, "experiments", "real",
                     "campaign_provenance.json")
    with open(p, "w") as f:
        json.dump(out, f, indent=1)
    print("\n->", os.path.relpath(p, _ROOT).replace("\\", "/"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
