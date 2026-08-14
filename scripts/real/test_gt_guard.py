"""Unit tests for the ground-truth abort predicate (gt_guard.PoseGuard).

These are the tests that were impossible while the predicate lived inline
in avoid_mission.py: every abort class is exercised here without a
camera, a vehicle or water. Run with

    python -m pytest scripts/real/test_gt_guard.py -q
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gt_guard import (  # noqa: E402
    ACQUIRING,
    IMPOSSIBLE_JUMP,
    LEVEL_ABORT,
    LEVEL_DEGRADED,
    LEVEL_OK,
    LOW_CONFIDENCE,
    NO_DETECTION,
    NO_POSE,
    STALE_POSE,
    WALL_MARGIN,
    GuardConfig,
    Observation,
    PoseGuard,
    box_headroom,
    evaluate,
    run_sequence,
    safe_box,
)

CFG = GuardConfig()
DT = 0.27          # measured overhead cycle, 2026-08-14 missions
AREA = 15000.0     # measured ROV blob area, mid-range


def obs(t, fx, fy, area=AREA, found=True):
    return Observation(t=t, found=found, frac_x=fx, frac_y=fy,
                       area_px=area,
                       pixel=(fx * CFG.frame_w, fy * CFG.frame_h))


def straight_track(n=40, fx0=0.20, fy=0.55, dfx=0.004, t0=0.0, dt=DT,
                   area=AREA):
    """Nominal run: left to right across the middle of the frame."""
    return [obs(t0 + i * dt, fx0 + i * dfx, fy, area) for i in range(n)]


# ---------------------------------------------------------------------
# safe box construction
# ---------------------------------------------------------------------

def test_safe_box_mid_frame_start_is_the_absolute_margin() -> None:
    box = safe_box((0.5, 0.5), CFG)
    assert box == (0.10, 0.10, 0.90, 0.90)


def test_safe_box_relaxes_for_an_edge_release() -> None:
    """The regression that cost three wet runs on 2026-08-14: released
    at the pool edge, an absolute 0.10 margin aborts at t=0."""
    box = safe_box((0.06, 0.55), CFG)
    assert box[0] < 0.06                      # start is inside
    assert box[0] == 0.03                     # 0.06 - release_tol


def test_safe_box_never_below_the_hard_limit() -> None:
    box = safe_box((0.025, 0.5), CFG)
    assert box[0] == CFG.hard_limit


def test_box_headroom_sign() -> None:
    box = (0.1, 0.1, 0.9, 0.9)
    assert box_headroom((0.5, 0.5), box) > 0
    assert box_headroom((0.05, 0.5), box) < 0


# ---------------------------------------------------------------------
# nominal
# ---------------------------------------------------------------------

def test_nominal_track_never_aborts() -> None:
    res = run_sequence(straight_track())
    assert res["abort"] is None
    assert res["guard"]["n_fix"] == 40
    assert res["min_box_headroom"] > 0


def test_first_fix_is_ok_and_sets_the_box() -> None:
    g = PoseGuard()
    v = g.update(obs(0.0, 0.06, 0.55))
    assert v.level == LEVEL_OK
    assert g.has_fix
    assert g.snapshot()["box"][0] == 0.03


def test_occasional_dropouts_are_degraded_not_aborts() -> None:
    """Measured detection rate was 72-92 % of cycles; a single missed
    cycle must not stop a run."""
    seq = []
    for i in range(30):
        o = obs(i * DT, 0.2 + 0.004 * i, 0.55)
        seq.append(o if i % 4 else Observation(t=o.t, found=False))
    res = run_sequence(seq)
    assert res["abort"] is None


# ---------------------------------------------------------------------
# 1. wall-margin violation
# ---------------------------------------------------------------------

def test_wall_margin_abort_on_drift_to_the_edge() -> None:
    seq = straight_track(n=30, fx0=0.30, dfx=-0.01)   # walks left
    res = run_sequence(seq)
    assert res["aborted_reason"] == WALL_MARGIN
    assert res["abort"]["detail"]["frac"][0] <= 0.10


def test_wall_margin_abort_on_the_far_side_too() -> None:
    seq = straight_track(n=40, fx0=0.60, dfx=0.01)
    res = run_sequence(seq)
    assert res["aborted_reason"] == WALL_MARGIN


def test_wall_margin_abort_in_y() -> None:
    seq = [obs(i * DT, 0.5, 0.55 + 0.02 * i) for i in range(30)]
    res = run_sequence(seq)
    assert res["aborted_reason"] == WALL_MARGIN


def test_edge_release_does_not_abort_at_t0() -> None:
    """The 19:00 / 19:02 / 19:09 failure mode, as a regression test."""
    seq = straight_track(n=20, fx0=0.052, fy=0.63, dfx=0.006)
    res = run_sequence(seq)
    assert res["abort"] is None


def test_edge_release_still_aborts_if_it_gets_worse() -> None:
    """Relative does not mean permissive: moving further into the
    corner than the release point must still stop the run."""
    seq = straight_track(n=20, fx0=0.08, fy=0.55, dfx=-0.01)
    res = run_sequence(seq)
    assert res["aborted_reason"] == WALL_MARGIN


# ---------------------------------------------------------------------
# 2. missing pose (never acquired)
# ---------------------------------------------------------------------

def test_missing_pose_is_degraded_before_the_acquire_timeout() -> None:
    g = PoseGuard(t0=0.0)
    v = g.update(Observation(t=5.0, found=False))
    assert v.level == LEVEL_DEGRADED
    assert v.reason == ACQUIRING
    assert not g.has_fix


def test_missing_pose_aborts_after_the_acquire_timeout() -> None:
    seq = [Observation(t=i * 0.3, found=False) for i in range(60)]
    res = run_sequence(seq, t0=0.0)
    assert res["aborted_reason"] == NO_POSE
    assert res["abort"]["t"] > CFG.acquire_timeout_s


def test_slow_acquisition_is_tolerated() -> None:
    """Worst measured acquisition was 7.4 s (mission 19:38): the
    timeout must sit above it."""
    seq = [Observation(t=i * 0.3, found=False) for i in range(25)]
    seq += straight_track(n=10, t0=7.5)
    res = run_sequence(seq, t0=0.0)
    assert res["abort"] is None
    assert res["guard"]["n_fix"] == 10


# ---------------------------------------------------------------------
# 3. stale pose
# ---------------------------------------------------------------------

def test_stale_pose_aborts_after_the_timeout() -> None:
    seq = straight_track(n=10)
    t = seq[-1].t
    seq += [Observation(t=t + 0.3 * k, found=False) for k in range(1, 8)]
    res = run_sequence(seq)
    assert res["aborted_reason"] == STALE_POSE
    assert res["abort"]["detail"]["age_s"] > CFG.stale_abort_s


def test_stale_pose_warns_before_it_aborts() -> None:
    g = PoseGuard()
    g.update(obs(0.0, 0.5, 0.5))
    v = g.update(Observation(t=0.6, found=False))
    assert v.level == LEVEL_DEGRADED
    assert v.detail.get("warn")
    assert not g.aborted


def test_measured_worst_gap_does_not_trip_staleness() -> None:
    """Worst inter-fix gap over the three recorded missions was 0.31 s
    (160 fixes); the timeout must clear it with margin."""
    g = PoseGuard()
    g.update(obs(0.0, 0.5, 0.5))
    v = g.update(obs(0.31, 0.5, 0.5))
    assert v.level == LEVEL_OK


# ---------------------------------------------------------------------
# 4. impossible jump (teleport beyond a physical speed limit)
# ---------------------------------------------------------------------

def test_impossible_jump_aborts() -> None:
    """A detector re-lock onto the shaded pool-edge band moves the
    reported centroid by several hundred pixels in one cycle."""
    seq = straight_track(n=6)
    t = seq[-1].t
    seq.append(obs(t + DT, 0.20 + 0.40, 0.55))     # ~770 px in 0.27 s
    res = run_sequence(seq)
    assert res["aborted_reason"] == IMPOSSIBLE_JUMP
    assert res["abort"]["detail"]["implied_speed_m_s"] > CFG.max_speed_m_s


def test_measured_peak_speed_does_not_trip_the_jump_test() -> None:
    """Fastest step measured on the real vehicle was 37 px in 0.28 s
    (~0.33 m/s at 402 px/m)."""
    g = PoseGuard()
    g.update(obs(0.0, 0.30, 0.55))
    v = g.update(obs(0.28, 0.30 + 37.0 / CFG.frame_w, 0.55))
    assert v.level == LEVEL_OK
    assert v.detail["step_px"] < v.detail["allowed_px"]


def test_jump_tolerance_covers_centroid_wander_at_zero_dt() -> None:
    """Two detections with the same timestamp may still differ by the
    blob-centroid noise; that must not be read as infinite speed."""
    g = PoseGuard()
    g.update(obs(1.0, 0.30, 0.55))
    v = g.update(obs(1.0, 0.30 + 10.0 / CFG.frame_w, 0.55))
    assert v.level == LEVEL_OK


def test_jump_check_precedes_the_wall_check() -> None:
    """A teleport that lands outside the box must be reported as the
    measurement fault it is, not as a vehicle excursion."""
    g = PoseGuard()
    g.update(obs(0.0, 0.50, 0.50))
    v = g.update(obs(DT, 0.01, 0.50))
    assert v.reason == IMPOSSIBLE_JUMP


# ---------------------------------------------------------------------
# 5. low-confidence / small-blob observation
# ---------------------------------------------------------------------

def test_small_blob_is_rejected_not_used_as_a_fix() -> None:
    g = PoseGuard()
    g.update(obs(0.0, 0.50, 0.50))
    v = g.update(obs(DT, 0.50, 0.50, area=900.0))
    assert v.level == LEVEL_DEGRADED
    assert v.reason == LOW_CONFIDENCE
    assert g.state.n_fix == 1


def test_oversized_blob_is_rejected() -> None:
    """A blob far larger than the hull means the ROV merged with the
    rim shadow - the centroid is then meaningless."""
    g = PoseGuard()
    g.update(obs(0.0, 0.50, 0.50))
    v = g.update(obs(DT, 0.50, 0.50, area=90000.0))
    assert v.reason == LOW_CONFIDENCE


def test_sustained_low_confidence_escalates_to_stale_abort() -> None:
    seq = straight_track(n=6)
    t = seq[-1].t
    seq += [obs(t + 0.3 * k, 0.5, 0.5, area=500.0) for k in range(1, 8)]
    res = run_sequence(seq)
    assert res["aborted_reason"] == STALE_POSE
    assert res["guard"]["last_reject"] == LOW_CONFIDENCE


def test_low_confidence_at_acquisition_never_sets_the_box() -> None:
    g = PoseGuard(t0=0.0)
    g.update(obs(0.5, 0.05, 0.9, area=100.0))
    assert g.snapshot()["box"] is None
    assert not g.has_fix


def test_area_absent_is_accepted_and_flagged() -> None:
    """Replayed pilot logs predate area logging: an unknown area must
    not be read as a zero-area rejection."""
    g = PoseGuard()
    v = g.update(Observation(t=0.0, found=True, frac_x=0.5, frac_y=0.5))
    assert v.level == LEVEL_OK
    assert v.detail["area_px"] is None


def test_area_at_the_gate_boundaries_is_accepted() -> None:
    g = PoseGuard()
    assert g.update(obs(0.0, 0.5, 0.5, area=CFG.min_area_px)).ok
    g2 = PoseGuard()
    assert g2.update(obs(0.0, 0.5, 0.5, area=CFG.max_area_px)).ok


# ---------------------------------------------------------------------
# latching, adapters and configuration
# ---------------------------------------------------------------------

def test_abort_latches() -> None:
    g = PoseGuard()
    g.update(obs(0.0, 0.5, 0.5))
    g.update(obs(DT, 0.01, 0.5))                 # impossible jump
    v = g.update(obs(2 * DT, 0.5, 0.5))          # back to a good fix
    assert v.abort
    assert v.detail["latched"]
    assert g.aborted == IMPOSSIBLE_JUMP


def test_reset_clears_the_abort() -> None:
    g = PoseGuard()
    g.update(obs(0.0, 0.5, 0.5))
    g.update(obs(DT, 0.01, 0.5))
    g.reset(t0=0.0)
    assert g.aborted is None
    assert g.update(obs(0.0, 0.5, 0.5)).ok


def test_from_detection_adapts_the_overhead_tracker_dict() -> None:
    det = {"found": True, "t": 12.5, "pixel": [960.0, 540.0],
           "area_px": 15000.0, "frac_x": 0.5, "frac_y": 0.5}
    o = Observation.from_detection(det)
    assert o.found and o.t == 12.5 and o.area_px == 15000.0
    assert PoseGuard().update(det).ok


def test_from_detection_handles_not_found_and_none() -> None:
    assert not Observation.from_detection(None, t=1.0).found
    assert not Observation.from_detection({"found": False, "t": 1.0}).found


def test_from_detection_derives_fractions_from_the_pixel() -> None:
    det = {"found": True, "t": 0.0, "pixel": [192.0, 108.0],
           "area_px": AREA}
    o = Observation.from_detection(det)
    assert abs(o.frac_x - 0.1) < 1e-9 and abs(o.frac_y - 0.1) < 1e-9


def test_config_overrides_reach_the_predicate() -> None:
    g = PoseGuard(max_speed_m_s=0.05)
    g.update(obs(0.0, 0.30, 0.55))
    assert g.update(obs(DT, 0.30 + 37.0 / CFG.frame_w, 0.55)).abort


def test_evaluate_is_pure() -> None:
    """The state handed in must not be modified - that property is what
    lets the missions and the harness share the predicate safely."""
    g = PoseGuard()
    g.update(obs(0.0, 0.5, 0.5))
    before = g.state
    evaluate(before, obs(DT, 0.01, 0.5), CFG)
    evaluate(before, obs(DT, 0.5, 0.5), CFG)
    assert before == g.state
    assert before.aborted is None


def test_run_sequence_reports_headroom() -> None:
    res = run_sequence(straight_track(n=20, fx0=0.30, fy=0.50))
    assert 0.0 < res["min_box_headroom"] <= 0.4
    assert 0.0 < res["max_step_ratio"] < 1.0
    assert abs(res["max_fix_gap_s"] - DT) < 1e-6
