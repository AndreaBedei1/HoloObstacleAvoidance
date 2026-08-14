"""OFFLINE validation harness for the ground-truth / abort machinery.

WHY: the abort predicate is the only thing standing between a real run
and a pool wall, and on 2026-08-14 it was validated by flying it - three
wet runs were spent discovering that the wall margin had to be relative
to the release point. This harness exercises the SAME predicate
(gt_guard.PoseGuard) against synthetic pose sequences and against the
RECORDED pilot trajectories, with no camera, no vehicle and no water, so
a threshold change can be qualified before anyone gets wet.

Two halves, both required to pass:

  * SYNTHETIC - one sequence per failure class (wall margin, missing
    pose, stale pose, impossible jump, low-confidence blob) plus nominal
    cases; each asserts the outcome the guard must produce.
  * REPLAY - every recorded mission trajectory in experiments/real is
    pushed through the guard. These runs completed without a wall abort,
    so an abort here means the proposed thresholds are too tight and
    would have destroyed a good run. The harness also reports how much
    headroom each run had against each threshold.

The pilot runs of 2026-08-14 are CALIBRATION only: they are used here as
a regression corpus for the guard, never as experimental results.

Usage:
    python scripts/real/gt_validate.py
    python scripts/real/gt_validate.py --json out.json --verbose
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gt_guard import (  # noqa: E402
    IMPOSSIBLE_JUMP,
    LEVEL_DEGRADED,
    LOW_CONFIDENCE,
    NO_POSE,
    STALE_POSE,
    WALL_MARGIN,
    GuardConfig,
    Observation,
    PoseGuard,
    run_sequence,
)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
REPLAY_GLOBS = (
    os.path.join(ROOT, "experiments", "real", "missions", "*", "log.json"),
    os.path.join(ROOT, "experiments", "real", "avoid_runs", "*", "log.json"),
)

DT = 0.27           # measured overhead cycle time, 2026-08-14
AREA = 15000.0      # measured ROV blob area at 1080p, mid-range

# Expected replay outcome per recorded session, with the reason. The
# DEFAULT for any session not listed here is "must not abort", which is
# the regression that matters for new runs. The four entries below are
# runs the guard SHOULD stop, established by inspecting the recordings:
# they are why this table exists rather than a blanket "never abort".
REPLAY_EXPECTATIONS = {
    "20260814_185340": (
        NO_POSE,
        "overhead GT locked on only 7 of 86 cycles (8 %); the run flew "
        "essentially without external position - aborting is correct"),
    "20260814_185622": (
        STALE_POSE,
        "overhead GT locked on 14 of 75 cycles (19 %) with a 1.32 s "
        "hole; the wall margin was uncertifiable for that long"),
    "20260814_190516": (
        IMPOSSIBLE_JUMP,
        "GT teleports 123 px (1.02 m/s) in one cycle at t=7.8 s against "
        "a 16 px median / 28 px p90 step - a detector re-lock, and it "
        "lands at the closest approach the run reports"),
    "20260814_191040": (
        IMPOSSIBLE_JUMP,
        "GT teleports 127 px (1.17 m/s) in one cycle at t=13.5 s "
        "against a 15 px median / 21 px p90 step, again at the closest "
        "approach; see docs/GT_VALIDATION.md"),
}


# ---------------------------------------------------------------------
# synthetic sequences
# ---------------------------------------------------------------------

def _obs(t, fx, fy, area=AREA, cfg=None):
    cfg = cfg or GuardConfig()
    return Observation(t=t, found=True, frac_x=fx, frac_y=fy,
                       area_px=area,
                       pixel=(fx * cfg.frame_w, fy * cfg.frame_h))


def _track(n, fx0, fy0, dfx=0.0, dfy=0.0, t0=0.0, dt=DT, area=AREA):
    return [_obs(t0 + i * dt, fx0 + i * dfx, fy0 + i * dfy, area)
            for i in range(n)]


def _miss(n, t0, dt=DT):
    return [Observation(t=t0 + i * dt, found=False) for i in range(n)]


def synthetic_cases() -> list[dict]:
    """Each case: the sequence, and the outcome the guard MUST produce.

    `expect_abort=None` means the run must complete; a string means the
    first abort must carry that reason."""
    cases = [
        {"name": "nominal_centre_run",
         "why": "straight left-to-right pass through the middle of the "
                "frame at the measured cycle rate",
         "seq": _track(40, 0.20, 0.55, dfx=0.004),
         "expect_abort": None},

        {"name": "nominal_edge_release",
         "why": "operator lowers the vehicle in at the pool edge "
                "(frac_x 0.05); the RELATIVE box must tolerate it - "
                "this is the 19:00/19:02/19:09 failure mode",
         "seq": _track(30, 0.052, 0.63, dfx=0.006),
         "expect_abort": None},

        {"name": "nominal_with_dropouts",
         "why": "measured detection rate was 72-92 % of cycles; "
                "isolated misses must degrade, not abort",
         "seq": [o if i % 4 else Observation(t=o.t, found=False)
                 for i, o in enumerate(_track(40, 0.20, 0.55, dfx=0.004))],
         "expect_abort": None},

        {"name": "nominal_slow_acquisition",
         "why": "worst measured acquisition was 7.4 s (mission 19:38)",
         "seq": _miss(25, 0.0, dt=0.3) + _track(20, 0.30, 0.55, t0=7.5,
                                                dfx=0.004),
         "expect_abort": None},

        {"name": "wall_margin_drift_to_near_rim",
         "why": "vehicle walks sideways into the pool wall - the "
                "primary physical hazard",
         "seq": _track(40, 0.45, 0.60, dfy=0.012),
         "expect_abort": WALL_MARGIN},

        {"name": "wall_margin_worsening_edge_release",
         "why": "released near the edge and then pushed FURTHER into "
                "it: relative must not mean permissive",
         "seq": _track(30, 0.08, 0.55, dfx=-0.01),
         "expect_abort": WALL_MARGIN},

        {"name": "missing_pose_never_acquired",
         "why": "overhead detector never locks (sun glare, vehicle out "
                "of frame) - the run must not start blind",
         "seq": _miss(60, 0.0, dt=0.3),
         "expect_abort": NO_POSE},

        {"name": "stale_pose_detector_dropout",
         "why": "lock lost mid-run; after the timeout the guard can no "
                "longer certify the wall margin",
         "seq": _track(15, 0.30, 0.55, dfx=0.004) + _miss(8, 4.2, dt=0.3),
         "expect_abort": STALE_POSE},

        {"name": "impossible_jump_relock_on_rim_shadow",
         "why": "the shaded band along the pool edge is dark and large; "
                "a re-lock teleports the centroid several hundred px",
         "seq": _track(10, 0.30, 0.55, dfx=0.004)
                + [_obs(10 * DT, 0.72, 0.55)],
         "expect_abort": IMPOSSIBLE_JUMP},

        {"name": "impossible_jump_lands_inside_the_box",
         "why": "a bad measurement must be caught on physics even when "
                "the reported position looks perfectly safe",
         "seq": _track(10, 0.30, 0.55, dfx=0.004)
                + [_obs(10 * DT, 0.55, 0.50)],
         "expect_abort": IMPOSSIBLE_JUMP},

        {"name": "low_confidence_small_blob_transient",
         "why": "one ripple/glare fragment: rejected as a fix, run "
                "continues",
         "seq": _track(10, 0.30, 0.55, dfx=0.004)
                + [_obs(10 * DT, 0.30, 0.55, area=900.0)]
                + _track(10, 0.35, 0.55, dfx=0.004, t0=11 * DT),
         "expect_abort": None,
         "expect_degraded": LOW_CONFIDENCE},

        {"name": "low_confidence_sustained",
         "why": "the detector keeps returning a too-small blob: the "
                "pose is effectively gone and must escalate",
         "seq": _track(10, 0.30, 0.55, dfx=0.004)
                + [_obs(2.7 + 0.3 * k, 0.30, 0.55, area=900.0)
                   for k in range(1, 8)],
         "expect_abort": STALE_POSE,
         "expect_degraded": LOW_CONFIDENCE},

        {"name": "low_confidence_merged_blob",
         "why": "the ROV blob merging with the rim shadow exceeds the "
                "hull area; its centroid is meaningless",
         "seq": _track(10, 0.30, 0.55, dfx=0.004)
                + [_obs(10 * DT, 0.30, 0.55, area=95000.0)],
         "expect_abort": None,
         "expect_degraded": LOW_CONFIDENCE},
    ]
    return cases


def check_case(case: dict, cfg: GuardConfig) -> dict:
    res = run_sequence(case["seq"], cfg=cfg)
    got = res["aborted_reason"]
    want = case.get("expect_abort")
    problems = []
    if got != want:
        problems.append(f"expected abort {want!r}, got {got!r}")
    want_deg = case.get("expect_degraded")
    if want_deg is not None:
        seen = {v.reason for v in res["verdicts"]
                if v.level == LEVEL_DEGRADED}
        if want_deg not in seen:
            problems.append(f"expected a DEGRADED {want_deg!r}; "
                            f"saw {sorted(x for x in seen if x)}")
    return {"name": case["name"], "why": case["why"],
            "expected": want, "got": got, "n": res["n"],
            "min_box_headroom": res["min_box_headroom"],
            "passed": not problems, "problems": problems}


# ---------------------------------------------------------------------
# replay of the recorded pilot trajectories
# ---------------------------------------------------------------------

def recorded_margin(path: str, default: float = 0.10) -> float:
    """The wall margin the run was actually flown with.

    The pilot sessions used 0.03, 0.08 and 0.10 as the operator hunted
    for a workable value, so replaying them all at one margin compares
    the guard against a box the run never had."""
    try:
        with open(path) as f:
            params = json.load(f)["summary"].get("params") or {}
        return float(params.get("margin", default))
    except (OSError, ValueError, KeyError, TypeError):
        return default


def load_recorded(path: str, cfg: GuardConfig) -> list[Observation]:
    """Rebuild a timestamped observation sequence from a mission log.

    avoid_mission.py wrote the cycle log and the trajectory separately:
    a row carries `gt_dist_px` exactly when the overhead detector had a
    fix that cycle, and `traj` holds those fixes in order. Zipping them
    recovers (t, pixel, found) per cycle. area_px was not logged before
    today, so replayed observations leave it None - the guard accepts an
    unknown area and flags it rather than failing the size gate."""
    with open(path) as f:
        data = json.load(f)
    rows = data.get("log") or []
    traj = data.get("traj") or []
    out, i = [], 0
    for r in rows:
        t = float(r.get("t", 0.0))
        if r.get("gt_dist_px") is not None and i < len(traj):
            px = traj[i]
            i += 1
            out.append(Observation(
                t=t, found=True, pixel=(float(px[0]), float(px[1])),
                frac_x=float(px[0]) / cfg.frame_w,
                frac_y=float(px[1]) / cfg.frame_h))
        else:
            out.append(Observation(t=t, found=False))
    return out


def replay_cases(cfg: GuardConfig, patterns=REPLAY_GLOBS) -> list[dict]:
    """Push every recorded trajectory through the guard.

    Each session is replayed with ITS OWN wall margin. The expected
    outcome is `None` (must complete) unless REPLAY_EXPECTATIONS says
    otherwise, so a new run that trips the guard is a failure and the
    four known-bad pilot runs are not."""
    results = []
    seen = set()
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            tag = os.path.basename(os.path.dirname(path))
            if tag in seen:
                continue
            seen.add(tag)
            run_cfg = replace(cfg, margin=recorded_margin(path, cfg.margin))
            obs = load_recorded(path, run_cfg)
            fixes = sum(1 for o in obs if o.found)
            if fixes == 0:
                # Either an avoid_run.py session (predates overhead
                # logging) or a run the old guard stopped before the
                # first log row was written.
                results.append({"session": tag, "path": path,
                                "skipped": "no overhead fixes recorded",
                                "passed": True})
                continue
            res = run_sequence(obs, cfg=run_cfg)
            want, why = REPLAY_EXPECTATIONS.get(tag, (None, ""))
            got = res["aborted_reason"]
            problems = []
            if got != want:
                problems.append(
                    f"expected {want!r}, got {got!r}"
                    + (f" at t={res['abort']['t']}" if res["abort"] else ""))
            results.append({
                "session": tag, "path": path, "n_cycles": len(obs),
                "n_fixes": fixes, "margin": run_cfg.margin,
                "detect_rate": round(fixes / len(obs), 3),
                "expected": want, "expected_why": why,
                "aborted_reason": got,
                "min_box_headroom": res["min_box_headroom"],
                "max_step_ratio": res["max_step_ratio"],
                "max_fix_gap_s": res["max_fix_gap_s"],
                "box": res["guard"]["box"],
                "passed": not problems, "problems": problems})
    return results


# ---------------------------------------------------------------------
# report
# ---------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Offline validation of the GT abort machinery "
                    "(no hardware).")
    ap.add_argument("--json", default=None, help="write the report here")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--margin", type=float, default=None)
    ap.add_argument("--max-speed", type=float, default=None,
                    help="max plausible vehicle speed, m/s")
    ap.add_argument("--stale-s", type=float, default=None)
    ap.add_argument("--min-area", type=float, default=None)
    ap.add_argument("--px-per-m", type=float, default=None)
    ap.add_argument("--replay-glob", action="append", default=None,
                    help="extra log.json glob to replay; repeatable. "
                         "Use it to point at sessions that are not in "
                         "the working tree (see docs/GT_VALIDATION.md).")
    args = ap.parse_args()

    over = {}
    if args.margin is not None:
        over["margin"] = args.margin
    if args.max_speed is not None:
        over["max_speed_m_s"] = args.max_speed
    if args.stale_s is not None:
        over["stale_abort_s"] = args.stale_s
    if args.min_area is not None:
        over["min_area_px"] = args.min_area
    if args.px_per_m is not None:
        over["px_per_m"] = args.px_per_m
    cfg = GuardConfig(**over)

    syn = [check_case(c, cfg) for c in synthetic_cases()]
    rep = replay_cases(cfg, patterns=tuple(args.replay_glob or ())
                       + REPLAY_GLOBS)

    print("=" * 70)
    print("GT ABORT VALIDATION - offline, no hardware")
    print("=" * 70)
    print("thresholds:", json.dumps(cfg.as_dict()))
    print()
    print("SYNTHETIC FAILURE CLASSES")
    print(f"  {'case':38s} {'expected':16s} {'got':16s} ok")
    for r in syn:
        print(f"  {r['name']:38s} {str(r['expected']):16s} "
              f"{str(r['got']):16s} {'PASS' if r['passed'] else 'FAIL'}")
        if args.verbose:
            print(f"      why: {r['why']}")
        for p in r["problems"]:
            print(f"      !! {p}")

    print()
    print("REPLAY OF RECORDED PILOT TRAJECTORIES "
          "(calibration data, never results)")
    if not rep:
        print("  (no recorded sessions found)")
    for r in rep:
        if r.get("skipped"):
            print(f"  {r['session']:20s} skipped: {r['skipped']}")
            continue
        head = ("  n/a" if r["min_box_headroom"] is None
                else f"{r['min_box_headroom']:.3f}")
        print(f"  {r['session']:20s} m={r['margin']:.2f} fixes "
              f"{r['n_fixes']:3d}/{r['n_cycles']:<3d} "
              f"({r['detect_rate']:.0%})  "
              f"want={str(r['expected']):16s} "
              f"got={str(r['aborted_reason']):16s} "
              f"head {head}  "
              f"step {r['max_step_ratio']:.2f}x  "
              f"gap {r['max_fix_gap_s']:.2f}s")
        if args.verbose and r["expected_why"]:
            print(f"      why: {r['expected_why']}")
        for p in r["problems"]:
            print(f"      !! {p}")

    n_fail = sum(1 for r in syn + rep if not r["passed"])
    print()
    print(f"{len(syn)} synthetic cases, "
          f"{sum(1 for r in rep if not r.get('skipped'))} replayed runs, "
          f"{n_fail} failure(s)")
    # Headroom summary over the runs that are supposed to COMPLETE: the
    # known-bad runs would only report how far past a threshold their
    # faults sit, which is not what the margin question asks.
    live = [r for r in rep
            if not r.get("skipped") and r["expected"] is None
            and r["min_box_headroom"] is not None]
    if live:
        head = min(r["min_box_headroom"] for r in live)
        step = max(r["max_step_ratio"] for r in live)
        gap = max(r["max_fix_gap_s"] for r in live)
        print("worst observed on real data:  "
              f"box headroom {head:.3f} (abort at 0.000) | "
              f"jump {step:.2f}x (abort at 1.00x) | "
              f"fix gap {gap:.2f}s "
              f"(abort at {cfg.stale_abort_s:.2f}s)")
    print("VERDICT:", "PASS" if n_fail == 0 else "FAIL")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"config": cfg.as_dict(), "synthetic": syn,
                       "replay": rep, "failures": n_fail}, f, indent=2)
        print("report ->", args.json)
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
