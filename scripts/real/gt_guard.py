"""Ground-truth pose guard: the single abort predicate for real runs.

WHY this module exists. During the 2026-08-14 pilot the wall-margin abort
lived inline inside avoid_mission.py, so the only way to exercise it was
to put the vehicle in the water: three runs (19:00, 19:02, 19:09) aborted
at t=0 because the margin was ABSOLUTE instead of relative to the release
point, and the defect was found by burning wet runs. An abort predicate
that cannot be tested dry is a liability. The predicate therefore lives
here as a PURE function over an explicit state, and the missions, the
offline harness (gt_validate.py) and the unit tests (test_gt_guard.py)
all execute exactly the same code.

Input is the OVERHEAD RealSense detection only. That camera is external
ground truth and safety supervision; it is never an input to the
obstacle-avoidance planner (scientific protocol).

Five failure classes are covered, all observed or plausible on the real
rig: wall-margin violation, pose never acquired, pose gone stale,
physically impossible jump (detector re-locking onto the rim shadow) and
a low-confidence / wrong-size blob.

Thresholds and their justification: docs/GT_VALIDATION.md.
Standard library only - no numpy, no cv2, no pyrealsense2 - so the guard
imports and runs on any machine, including a laptop with no camera.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

# Verdict levels. DEGRADED means "keep flying but the guard cannot
# certify the position this cycle"; only ABORT stops the run.
LEVEL_OK = "OK"
LEVEL_DEGRADED = "DEGRADED"
LEVEL_ABORT = "ABORT"

# Reasons. The first five are the abort classes; the last two are the
# transient states that precede stale_pose.
WALL_MARGIN = "wall_margin"
NO_POSE = "no_pose"
STALE_POSE = "stale_pose"
IMPOSSIBLE_JUMP = "impossible_jump"
LOW_CONFIDENCE = "low_confidence"
NO_DETECTION = "no_detection"
ACQUIRING = "acquiring"

ABORT_REASONS = (WALL_MARGIN, NO_POSE, STALE_POSE, IMPOSSIBLE_JUMP)


@dataclass(frozen=True)
class GuardConfig:
    """Acceptance thresholds. Every default is justified in
    docs/GT_VALIDATION.md against the 2026-08-14 recordings; do not
    change one here without changing the rationale there."""

    # Wall margin, in frame fractions. The box is RELATIVE to the release
    # point because the operator hands the vehicle in from the pool edge.
    margin: float = 0.10
    release_tol: float = 0.03
    hard_limit: float = 0.02

    # Physically impossible jump.
    max_speed_m_s: float = 0.60
    jump_tol_px: float = 25.0

    # Pose continuity.
    stale_abort_s: float = 1.0
    stale_warn_s: float = 0.45
    acquire_timeout_s: float = 12.0

    # Blob plausibility (overhead_track.py reports area_px at 1080p).
    min_area_px: float = 4000.0
    max_area_px: float = 60000.0

    # Scene scale. Anisotropic in truth (oblique camera): this is the
    # scale at the anchor plane from the metric survey, and it is
    # superseded by px_per_m_anchor_plane in the pool_frame_<date>.json
    # written by pool_remap.py.
    px_per_m: float = 402.0
    frame_w: float = 1920.0
    frame_h: float = 1080.0

    @property
    def max_speed_px_s(self) -> float:
        return self.max_speed_m_s * self.px_per_m

    def as_dict(self) -> dict:
        return {"margin": self.margin, "release_tol": self.release_tol,
                "hard_limit": self.hard_limit,
                "max_speed_m_s": self.max_speed_m_s,
                "jump_tol_px": self.jump_tol_px,
                "stale_abort_s": self.stale_abort_s,
                "stale_warn_s": self.stale_warn_s,
                "acquire_timeout_s": self.acquire_timeout_s,
                "min_area_px": self.min_area_px,
                "max_area_px": self.max_area_px,
                "px_per_m": self.px_per_m}


@dataclass(frozen=True)
class Observation:
    """One overhead cycle. `found=False` covers both "the detector
    returned nothing" and "the tracker raised" - the guard treats a
    missing observation and a rejected observation identically except
    for the reason it reports."""

    t: float
    found: bool = False
    frac_x: float | None = None
    frac_y: float | None = None
    area_px: float | None = None
    pixel: tuple[float, float] | None = None

    @classmethod
    def from_detection(cls, det: dict | None, t: float | None = None,
                       cfg: "GuardConfig | None" = None) -> "Observation":
        """Adapt the dict returned by OverheadTracker.detect()."""
        cfg = cfg or GuardConfig()
        if not det:
            return cls(t=t if t is not None else 0.0, found=False)
        ts = t if t is not None else float(det.get("t", 0.0))
        if not det.get("found"):
            return cls(t=ts, found=False)
        px = det.get("pixel")
        px = (float(px[0]), float(px[1])) if px else None
        fx, fy = det.get("frac_x"), det.get("frac_y")
        if fx is None and px is not None:
            fx, fy = px[0] / cfg.frame_w, px[1] / cfg.frame_h
        area = det.get("area_px")
        return cls(t=ts, found=True, pixel=px,
                   frac_x=None if fx is None else float(fx),
                   frac_y=None if fy is None else float(fy),
                   area_px=None if area is None else float(area))


@dataclass(frozen=True)
class Verdict:
    level: str
    reason: str | None = None
    detail: dict = field(default_factory=dict)

    @property
    def abort(self) -> bool:
        return self.level == LEVEL_ABORT

    @property
    def ok(self) -> bool:
        return self.level == LEVEL_OK

    def as_dict(self) -> dict:
        return {"level": self.level, "reason": self.reason,
                **self.detail}


@dataclass(frozen=True)
class GuardState:
    """All the memory the predicate needs. Immutable: `evaluate` returns
    a new state, which is what makes it testable by construction."""

    t_start: float = 0.0
    box: tuple[float, float, float, float] | None = None
    start_frac: tuple[float, float] | None = None
    last_fix_t: float | None = None
    last_fix_px: tuple[float, float] | None = None
    n_fix: int = 0
    n_reject: int = 0
    last_reject: str | None = None
    aborted: str | None = None


def safe_box(start_frac: tuple[float, float],
             cfg: GuardConfig) -> tuple[float, float, float, float]:
    """Frame-fraction box (lo_x, lo_y, hi_x, hi_y) allowed for the run.

    RELATIVE to the release point: the operator lowers the vehicle in
    from the pool edge, so an absolute margin is violated before the
    first command is sent. The rule is "never get closer to a border than
    where you were released (minus a small tolerance), and never cross
    the hard limit" - i.e. the run may keep the start standoff but may
    never worsen it."""
    sx, sy = start_frac
    lo_x = min(cfg.margin, max(cfg.hard_limit, sx - cfg.release_tol))
    lo_y = min(cfg.margin, max(cfg.hard_limit, sy - cfg.release_tol))
    hi_x = max(1.0 - cfg.margin,
               min(1.0 - cfg.hard_limit, sx + cfg.release_tol))
    hi_y = max(1.0 - cfg.margin,
               min(1.0 - cfg.hard_limit, sy + cfg.release_tol))
    return (lo_x, lo_y, hi_x, hi_y)


def box_headroom(frac: tuple[float, float],
                 box: tuple[float, float, float, float]) -> float:
    """Smallest signed distance (frame fractions) to a box edge.
    Negative means outside. Reported so a run can be scored on how close
    it came to the guard, not only on whether it tripped it."""
    fx, fy = frac
    lo_x, lo_y, hi_x, hi_y = box
    return min(fx - lo_x, hi_x - fx, fy - lo_y, hi_y - fy)


def _pixel(obs: Observation, cfg: GuardConfig) -> tuple[float, float]:
    if obs.pixel is not None:
        return obs.pixel
    return (obs.frac_x * cfg.frame_w, obs.frac_y * cfg.frame_h)


def _reject_reason(obs: Observation, cfg: GuardConfig) -> str | None:
    """None when the observation may be used as a position fix."""
    if not obs.found:
        return NO_DETECTION
    if obs.frac_x is None or obs.frac_y is None:
        if obs.pixel is None:
            return NO_DETECTION
    # area_px absent means "not reported" (replayed pilot logs predate
    # area logging), not "zero": it cannot be judged, so it is accepted
    # and flagged instead of silently failing the size gate.
    if obs.area_px is not None and not (
            cfg.min_area_px <= obs.area_px <= cfg.max_area_px):
        return LOW_CONFIDENCE
    return None


def _frac(obs: Observation, cfg: GuardConfig) -> tuple[float, float]:
    if obs.frac_x is not None and obs.frac_y is not None:
        return (obs.frac_x, obs.frac_y)
    px = obs.pixel
    return (px[0] / cfg.frame_w, px[1] / cfg.frame_h)


def evaluate(state: GuardState, obs: Observation,
             cfg: GuardConfig) -> tuple[Verdict, GuardState]:
    """THE abort predicate. Pure: no I/O, no clock, no mutation.

    Returns the verdict for this observation and the state to carry into
    the next one. Aborts LATCH - once the guard has stopped a run it
    keeps reporting the same reason, so a caller that misses one cycle
    cannot accidentally resume."""
    if state.aborted:
        return (Verdict(LEVEL_ABORT, state.aborted, {"latched": True}),
                state)

    reject = _reject_reason(obs, cfg)
    if reject is None:
        return _on_fix(state, obs, cfg)
    return _on_reject(state, obs, cfg, reject)


def _on_fix(state: GuardState, obs: Observation,
            cfg: GuardConfig) -> tuple[Verdict, GuardState]:
    px = _pixel(obs, cfg)
    frac = _frac(obs, cfg)
    detail = {"frac": [round(frac[0], 4), round(frac[1], 4)],
              "area_px": obs.area_px}

    # 1. Physical plausibility FIRST: after an impossible jump the
    # position is not trustworthy, so testing it against the wall box
    # would only launder a bad measurement.
    if state.last_fix_px is not None and state.last_fix_t is not None:
        dist = math.hypot(px[0] - state.last_fix_px[0],
                          px[1] - state.last_fix_px[1])
        dt = max(0.0, obs.t - state.last_fix_t)
        allow = cfg.jump_tol_px + cfg.max_speed_px_s * dt
        detail["step_px"] = round(dist, 1)
        detail["allowed_px"] = round(allow, 1)
        if dist > allow:
            detail["implied_speed_m_s"] = (
                round(dist / cfg.px_per_m / dt, 2) if dt > 0 else None)
            return (Verdict(LEVEL_ABORT, IMPOSSIBLE_JUMP, detail),
                    replace(state, aborted=IMPOSSIBLE_JUMP))

    # 2. Establish the safe box on the first accepted fix.
    box, start_frac = state.box, state.start_frac
    if box is None:
        start_frac = frac
        box = safe_box(frac, cfg)
    detail["box"] = [round(v, 3) for v in box]
    detail["headroom"] = round(box_headroom(frac, box), 4)

    new = replace(state, box=box, start_frac=start_frac,
                  last_fix_t=obs.t, last_fix_px=px,
                  n_fix=state.n_fix + 1)

    # 3. Wall margin.
    lo_x, lo_y, hi_x, hi_y = box
    if not (lo_x < frac[0] < hi_x and lo_y < frac[1] < hi_y):
        return (Verdict(LEVEL_ABORT, WALL_MARGIN, detail),
                replace(new, aborted=WALL_MARGIN))
    return Verdict(LEVEL_OK, None, detail), new


def _on_reject(state: GuardState, obs: Observation, cfg: GuardConfig,
               reject: str) -> tuple[Verdict, GuardState]:
    new = replace(state, n_reject=state.n_reject + 1, last_reject=reject)
    detail = {"reject": reject, "area_px": obs.area_px}

    if state.last_fix_t is None:
        # Never acquired. The mission must not leave its stationary
        # phase in this state (avoid_mission holds PREROLL until the
        # guard reports a fix).
        age = obs.t - state.t_start
        detail["since_start_s"] = round(age, 2)
        if age > cfg.acquire_timeout_s:
            return (Verdict(LEVEL_ABORT, NO_POSE, detail),
                    replace(new, aborted=NO_POSE))
        return Verdict(LEVEL_DEGRADED, ACQUIRING, detail), new

    age = obs.t - state.last_fix_t
    detail["age_s"] = round(age, 2)
    if age > cfg.stale_abort_s:
        return (Verdict(LEVEL_ABORT, STALE_POSE, detail),
                replace(new, aborted=STALE_POSE))
    if age > cfg.stale_warn_s:
        detail["warn"] = "pose ageing"
    return Verdict(LEVEL_DEGRADED, reject, detail), new


class PoseGuard:
    """Stateful wrapper around `evaluate` for the mission loop.

    Usage in a mission:

        guard = PoseGuard(margin=args.margin, t0=t0)
        ...
        v = guard.update(tracker.detect(near=near), t=time.time())
        if v.abort:
            break            # -> active hold, never a bare disarm
    """

    def __init__(self, cfg: GuardConfig | None = None,
                 t0: float | None = None, **overrides):
        if cfg is None:
            cfg = GuardConfig(**overrides)
        elif overrides:
            cfg = replace(cfg, **overrides)
        self.cfg = cfg
        self.state = GuardState(t_start=0.0 if t0 is None else float(t0))
        self.last: Verdict = Verdict(LEVEL_DEGRADED, ACQUIRING, {})

    def reset(self, t0: float) -> None:
        self.state = GuardState(t_start=float(t0))
        self.last = Verdict(LEVEL_DEGRADED, ACQUIRING, {})

    def update(self, det, t: float | None = None) -> Verdict:
        """Accepts either an Observation or an OverheadTracker dict."""
        obs = (det if isinstance(det, Observation)
               else Observation.from_detection(det, t=t, cfg=self.cfg))
        verdict, self.state = evaluate(self.state, obs, self.cfg)
        self.last = verdict
        return verdict

    @property
    def has_fix(self) -> bool:
        return self.state.n_fix > 0

    @property
    def aborted(self) -> str | None:
        return self.state.aborted

    def snapshot(self) -> dict:
        """Compact dict for the mission log / the run summary."""
        return {"n_fix": self.state.n_fix,
                "n_reject": self.state.n_reject,
                "last_reject": self.state.last_reject,
                "aborted": self.state.aborted,
                "box": (None if self.state.box is None
                        else [round(v, 3) for v in self.state.box]),
                "start_frac": (None if self.state.start_frac is None
                               else [round(v, 4)
                                     for v in self.state.start_frac])}


def run_sequence(observations, cfg: GuardConfig | None = None,
                 t0: float | None = None) -> dict:
    """Feed a whole sequence through the guard (offline harness, tests).

    Returns the first abort (if any) plus the headroom statistics that
    say how close a run came to each threshold."""
    cfg = cfg or GuardConfig()
    obs = list(observations)
    if t0 is None:
        t0 = obs[0].t if obs else 0.0
    guard = PoseGuard(cfg=cfg, t0=t0)
    verdicts = []
    abort_at = None
    min_headroom = None
    max_step_ratio = 0.0
    max_gap_s = 0.0
    prev_fix_t = None
    for i, o in enumerate(obs):
        v = guard.update(o)
        verdicts.append(v)
        if "headroom" in v.detail:
            h = v.detail["headroom"]
            min_headroom = h if min_headroom is None else min(min_headroom, h)
        if v.detail.get("allowed_px"):
            max_step_ratio = max(
                max_step_ratio,
                v.detail["step_px"] / v.detail["allowed_px"])
        if guard.state.last_fix_t is not None:
            if prev_fix_t is not None and guard.state.last_fix_t != prev_fix_t:
                max_gap_s = max(max_gap_s,
                                guard.state.last_fix_t - prev_fix_t)
            prev_fix_t = guard.state.last_fix_t
        if v.abort and abort_at is None:
            abort_at = {"index": i, "t": o.t, "reason": v.reason,
                        "detail": v.detail}
            break
    return {"n": len(verdicts), "abort": abort_at,
            "aborted_reason": None if abort_at is None else abort_at["reason"],
            "min_box_headroom": min_headroom,
            "max_step_ratio": round(max_step_ratio, 3),
            "max_fix_gap_s": round(max_gap_s, 3),
            "guard": guard.snapshot(), "verdicts": verdicts}
