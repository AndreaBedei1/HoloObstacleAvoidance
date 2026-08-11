"""Common perception qualification: startup warm-up + track confirmation.

Phase-7B layer closing the two integration weaknesses found in Phase 7:

  W1. startup-transient detections reached the planner before perception was
      trustworthy (early-commit family);
  W2. a giant outlier could corrupt a YOUNG track before the chi-square
      innovation gate became discriminative (E4 collisions).

Architecture (chosen and justified in docs/TEMPORAL_OBSTACLE_ESTIMATION.md):

    raw detector
      -> COMMON qualification (this module: monotonicity, degeneracy,
         physics-bounded coherence, source warm-up)
      -> T0/T1/T2/T3 temporal method (updates internally as usual)
      -> COMMON planner-valid confirmation policy (this module)
      -> planner

The layer is IDENTICAL for all methods so the T0-T3 comparison remains about
temporal estimation, not different startup safety logic.

Semantics preserved:
  - a raw message whose detections are ALL rejected is presented to the
    estimator as SILENCE (an outlier is not evidence of absence);
  - a genuinely empty raw message stays fresh-empty (absence evidence);
  - while not planner-valid, the node publishes EMPTY arrays (planner sees
    "perception alive, nothing hazardous") — the estimator still updates its
    tentative state internally.

Coherence bounds are PHYSICS-DERIVED (no magic bbox limits), from the
vehicle/scenario motion envelope (max relative speed ~1.5 m/s, min working
range ~2 m, HFOV 90 deg, 30 Hz):
  - image-center rate <= ~0.9 units/s worst case  -> bound 1.2 units/s
    (+ absolute per-message floor for frame jitter);
  - between consecutive frames range cannot change more than ~2.5%
    -> |delta log size| ~0.025 -> bound 0.5 (20x margin).
The E4-style corruption (jump 0.3 units + 2.5x size in one frame) violates
both bounds by construction of physics, not by tuning to the test.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .temporal_core import Detection, DetectionEvent


@dataclass
class QualificationConfig:
    # --- source warm-up -----------------------------------------------------
    warmup_min_updates: int = 20          # accepted coherent detections...
    warmup_min_span_s: float = 1.0        # ...spanning at least this long
    # Silence longer than this = source restart => re-warm. MUST exceed the
    # estimators' bridgeable dropout horizon (2.5 s): a shorter threshold
    # would trigger a mid-maneuver re-warm-up right after a bridged dropout
    # ends, creating a NEW planner-blindness failure mode. Beyond 4 s any
    # track has expired anyway, so requalification costs nothing extra.
    source_restart_silence_s: float = 4.0
    # --- per-measurement coherence (physics-derived, see module docstring) --
    max_center_rate_per_s: float = 1.2
    max_center_jump_abs: float = 0.25
    max_logsize_jump: float = 0.5
    # --- coherence-reference lifecycle --------------------------------------
    # The previous accepted measurement is only a meaningful coherence
    # reference for a bounded time; and if it keeps rejecting everything, the
    # REFERENCE itself is probably the garbage (majority evidence) — drop it.
    reference_max_age_s: float = 1.0
    reference_reset_rejections: int = 5
    # --- planner-valid confirmation ----------------------------------------
    confirm_min_updates: int = 3
    confirm_window_s: float = 1.5         # max gap between accepted updates
    empty_misses_to_reset: int = 3        # fresh-empty streak clears confidence


@dataclass
class QualifiedResult:
    """Outcome of feeding one raw message through qualification."""
    deliver_to_estimator: bool            # False => present as SILENCE
    event: Optional[DetectionEvent]       # filtered event when delivering
    planner_valid: bool                   # confirmation state AFTER this msg
    rejections: List[Dict[str, Any]] = field(default_factory=list)


class PerceptionQualifier:
    """Warm-up + coherence + confirmation state machine (single-object phase,
    multi-object-ready: per-class bookkeeping can be added without changing
    the interface)."""

    def __init__(self, config: Optional[QualificationConfig] = None) -> None:
        self.cfg = config or QualificationConfig()
        self.reset("init")

    # -- lifecycle -----------------------------------------------------------
    def reset(self, reason: str) -> None:
        self.state = "WARMING"
        self.last_msg_t: Optional[float] = None
        self.last_accepted: Optional[Detection] = None
        self.last_accepted_t: Optional[float] = None
        self.warmup_started_t: Optional[float] = None
        self.warmup_accepted = 0
        self.confirm_count = 0
        self.confirmed_at: Optional[float] = None
        self.empty_streak = 0
        self.updates_seen = 0
        self.total_rejections = 0
        self.rejection_reasons: Dict[str, int] = {}
        self.last_reset_reason = reason
        self.warmup_duration_s: Optional[float] = None
        self.last_confirm_t: Optional[float] = None
        self.consecutive_rejections = 0
        self.reference_resets = 0

    # -- helpers -------------------------------------------------------------
    def _reject(self, out: List[Dict[str, Any]], reason: str,
                detail: str) -> None:
        self.total_rejections += 1
        self.rejection_reasons[reason] = \
            self.rejection_reasons.get(reason, 0) + 1
        out.append({"reason": reason, "detail": detail})

    def _coherent(self, det: Detection, t: float,
                  rejections: List[Dict[str, Any]]) -> bool:
        # Degenerate bbox.
        if not (0.0 < det.w < 1.0 and 0.0 < det.h < 1.0):
            self._reject(rejections, "degenerate_bbox",
                         f"w={det.w:.3f} h={det.h:.3f}")
            return False
        if not (0.0 <= det.cx <= 1.0 and 0.0 <= det.cy <= 1.0):
            self._reject(rejections, "center_out_of_image",
                         f"cx={det.cx:.3f} cy={det.cy:.3f}")
            return False
        if self.last_accepted is None or self.last_accepted_t is None:
            return True
        # Stale reference: across a long gap the bounds are meaningless —
        # accept as a fresh reference (confirmation still requires M
        # consistent follow-ups before anything becomes planner-valid).
        if t - self.last_accepted_t > self.cfg.reference_max_age_s:
            return True
        dt = max(1e-3, t - self.last_accepted_t)
        jump = math.hypot(det.cx - self.last_accepted.cx,
                          det.cy - self.last_accepted.cy)
        bound = max(self.cfg.max_center_jump_abs,
                    self.cfg.max_center_rate_per_s * dt)
        if jump > bound:
            self._reject(rejections, "center_jump",
                         f"jump={jump:.3f} bound={bound:.3f} dt={dt:.3f}")
            return False
        dlw = abs(math.log(max(det.w, 1e-4))
                  - math.log(max(self.last_accepted.w, 1e-4)))
        dlh = abs(math.log(max(det.h, 1e-4))
                  - math.log(max(self.last_accepted.h, 1e-4)))
        if max(dlw, dlh) > self.cfg.max_logsize_jump:
            self._reject(rejections, "size_jump",
                         f"dlogw={dlw:.3f} dlogh={dlh:.3f} "
                         f"bound={self.cfg.max_logsize_jump}")
            return False
        return True

    def _penalize_confirmation(self) -> None:
        self.confirm_count = max(0, self.confirm_count - 1)
        if self.confirm_count < self.cfg.confirm_min_updates:
            self.confirmed_at = None

    # -- main entry ----------------------------------------------------------
    def feed(self, event: DetectionEvent) -> QualifiedResult:
        rejections: List[Dict[str, Any]] = []
        t = event.t
        self.updates_seen += 1

        # Timestamp monotonicity.
        if self.last_msg_t is not None and t < self.last_msg_t - 1e-6:
            self._reject(rejections, "non_monotonic_timestamp",
                         f"t={t:.3f} < last={self.last_msg_t:.3f}")
            return QualifiedResult(False, None, self.planner_valid(),
                                   rejections)

        # Source-restart detection: long silence re-warms the source.
        if self.last_msg_t is not None \
                and t - self.last_msg_t > self.cfg.source_restart_silence_s:
            self.reset("source_restart_after_silence")
        self.last_msg_t = t

        if event.is_empty:
            # Genuine absence evidence: forward as fresh-empty; a streak
            # clears the confirmation state (object gone). Empty messages DO
            # count toward SOURCE warm-up (a healthy stream of "nothing seen"
            # is stream-health evidence) so that a hazard appearing after a
            # quiet period confirms in ~M frames, not M + warm-up (Y8).
            if self.state == "WARMING":
                if self.warmup_started_t is None:
                    self.warmup_started_t = t
                self.warmup_accepted += 1
                if (self.warmup_accepted >= self.cfg.warmup_min_updates
                        and t - self.warmup_started_t
                        >= self.cfg.warmup_min_span_s):
                    self.state = "READY"
                    self.warmup_duration_s = t - self.warmup_started_t
            self.empty_streak += 1
            if self.empty_streak >= self.cfg.empty_misses_to_reset:
                self.confirm_count = 0
                self.confirmed_at = None
            return QualifiedResult(True, event, self.planner_valid(),
                                   rejections)
        self.empty_streak = 0

        best = max(event.detections, key=lambda d: d.confidence)
        if not self._coherent(best, t, rejections):
            # Outlier/garbage: penalize confirmation, present as SILENCE
            # (an outlier is NOT evidence of absence), and during warm-up
            # restart the coherence streak.
            if self.state == "WARMING":
                self.warmup_accepted = 0
                self.warmup_started_t = None
            self._penalize_confirmation()
            self.consecutive_rejections += 1
            if self.consecutive_rejections \
                    >= self.cfg.reference_reset_rejections:
                # Majority evidence says the REFERENCE is the garbage:
                # drop it so the next detection starts a fresh reference
                # (self-heals the garbage-reference deadlock, S0).
                self.last_accepted = None
                self.last_accepted_t = None
                self.consecutive_rejections = 0
                self.reference_resets += 1
            return QualifiedResult(False, None, self.planner_valid(),
                                   rejections)

        # Accepted coherent detection.
        self.consecutive_rejections = 0
        self.last_accepted = best
        self.last_accepted_t = t

        if self.state == "WARMING":
            if self.warmup_started_t is None:
                self.warmup_started_t = t
            self.warmup_accepted += 1
            if (self.warmup_accepted >= self.cfg.warmup_min_updates
                    and t - self.warmup_started_t
                    >= self.cfg.warmup_min_span_s):
                self.state = "READY"
                self.warmup_duration_s = t - self.warmup_started_t

        # Confirmation counting (gap-bounded).
        if self.confirmed_at is None and self.confirm_count > 0 \
                and self.last_confirm_t is not None \
                and t - self.last_confirm_t > self.cfg.confirm_window_s:
            self.confirm_count = 0
        self.confirm_count += 1
        self.last_confirm_t = t
        if self.state == "READY" \
                and self.confirm_count >= self.cfg.confirm_min_updates \
                and self.confirmed_at is None:
            self.confirmed_at = t

        filtered = DetectionEvent(t=t, detections=[best])
        return QualifiedResult(True, filtered, self.planner_valid(),
                               rejections)

    def planner_valid(self) -> bool:
        return self.state == "READY" and self.confirmed_at is not None

    def debug(self) -> Dict[str, Any]:
        return {
            "state": self.state,
            "planner_valid": self.planner_valid(),
            "warmup_accepted": self.warmup_accepted,
            "warmup_duration_s": self.warmup_duration_s,
            "confirm_count": self.confirm_count,
            "confirmed_at": self.confirmed_at,
            "updates_seen": self.updates_seen,
            "total_rejections": self.total_rejections,
            "rejection_reasons": dict(self.rejection_reasons),
            "reference_resets": self.reference_resets,
            "last_reset_reason": self.last_reset_reason,
        }
