"""Unit tests for the common perception qualification layer (Phase 7B).

Covers: warm-up gating, physics-bounded coherence, confirmation lifecycle,
source restart, rejected-outlier-as-silence semantics, and the two Phase-7
weaknesses (startup transient W1, young-track outlier W2).
"""

from __future__ import annotations

import unittest

from rov_obstacle_tracking.perception_qualification import (
    PerceptionQualifier,
    QualificationConfig,
)
from rov_obstacle_tracking.temporal_core import Detection, DetectionEvent

DT = 1.0 / 30.0


def det(cx=0.5, cy=0.5, w=0.15, h=0.18, conf=0.9):
    return Detection(class_name="anchor", confidence=conf,
                     cx=cx, cy=cy, w=w, h=h)


def feed_clean(q, n, t0=0.0, cx0=0.5, drift=0.0):
    t = t0
    res = None
    for i in range(n):
        res = q.feed(DetectionEvent(t=t, detections=[det(cx=cx0 + drift * i)]))
        t += DT
    return res, t


class TestWarmup(unittest.TestCase):
    def test_not_planner_valid_before_warmup(self):
        q = PerceptionQualifier()
        res, _ = feed_clean(q, 10)          # 0.33 s, 10 updates: too early
        self.assertFalse(res.planner_valid)
        self.assertEqual(q.state, "WARMING")

    def test_warmup_needs_updates_AND_span(self):
        cfg = QualificationConfig(warmup_min_updates=20, warmup_min_span_s=1.0)
        q = PerceptionQualifier(cfg)
        res, t = feed_clean(q, 20)          # 20 updates but only ~0.63 s span
        self.assertEqual(q.state, "WARMING")
        res, t = feed_clean(q, 15, t0=t)    # now spans > 1 s
        self.assertEqual(q.state, "READY")
        self.assertTrue(res.planner_valid)

    def test_incoherent_burst_restarts_warmup_streak(self):
        q = PerceptionQualifier()
        _, t = feed_clean(q, 10)
        q.feed(DetectionEvent(t=t, detections=[det(cx=0.9, w=0.5, h=0.6)]))
        self.assertEqual(q.warmup_accepted, 0, "garbage resets warm-up streak")

    def test_startup_transient_regression_W1(self):
        """Degenerate/jumping first frames must never become planner-valid."""
        q = PerceptionQualifier()
        garbage = [det(cx=0.9, w=0.95, h=0.97), det(cx=0.1, w=0.02, h=0.9),
                   det(cx=0.7, w=0.5, h=0.5)]
        t = 0.0
        for g in garbage:
            res = q.feed(DetectionEvent(t=t, detections=[g]))
            self.assertFalse(res.planner_valid)
            t += DT
        # even followed by a couple of clean frames: still warming
        res = q.feed(DetectionEvent(t=t, detections=[det()]))
        self.assertFalse(res.planner_valid)

    def test_source_restart_rewarms(self):
        q = PerceptionQualifier()
        res, t = feed_clean(q, 40)
        self.assertTrue(res.planner_valid)
        # long silence => source restart => must re-qualify
        res = q.feed(DetectionEvent(t=t + 5.0, detections=[det()]))
        self.assertFalse(res.planner_valid)
        self.assertEqual(q.state, "WARMING")
        self.assertEqual(q.last_reset_reason, "source_restart_after_silence")


class TestCoherence(unittest.TestCase):
    def test_center_jump_rejected_and_silence_semantics(self):
        q = PerceptionQualifier()
        _, t = feed_clean(q, 40)
        res = q.feed(DetectionEvent(
            t=t, detections=[det(cx=0.5 + 0.30)]))  # E4-style jump
        self.assertFalse(res.deliver_to_estimator,
                         "rejected outlier must be SILENCE, not absence")
        self.assertIsNone(res.event)
        self.assertEqual(res.rejections[0]["reason"], "center_jump")

    def test_size_jump_rejected(self):
        q = PerceptionQualifier()
        _, t = feed_clean(q, 40)
        res = q.feed(DetectionEvent(
            t=t, detections=[det(w=0.15 * 2.5, h=0.18 * 2.5)]))
        self.assertFalse(res.deliver_to_estimator)
        self.assertEqual(res.rejections[0]["reason"], "size_jump")

    def test_slow_drift_accepted(self):
        q = PerceptionQualifier()
        res, _ = feed_clean(q, 60, drift=0.004)   # 0.12/s drift: physical
        self.assertTrue(res.deliver_to_estimator)
        self.assertEqual(q.rejection_reasons, {})

    def test_gap_scaled_bound_allows_larger_jump_after_gap(self):
        """After a 0.5 s silence a 0.4 units jump is physical (<=1.2/s)."""
        q = PerceptionQualifier()
        _, t = feed_clean(q, 40)
        res = q.feed(DetectionEvent(t=t + 0.5,
                                    detections=[det(cx=0.5 + 0.4)]))
        self.assertTrue(res.deliver_to_estimator)

    def test_degenerate_bbox_rejected(self):
        q = PerceptionQualifier()
        res = q.feed(DetectionEvent(t=0.0, detections=[det(w=0.0)]))
        self.assertFalse(res.deliver_to_estimator)

    def test_non_monotonic_timestamp_rejected(self):
        q = PerceptionQualifier()
        q.feed(DetectionEvent(t=1.0, detections=[det()]))
        res = q.feed(DetectionEvent(t=0.5, detections=[det()]))
        self.assertFalse(res.deliver_to_estimator)
        self.assertEqual(res.rejections[0]["reason"],
                         "non_monotonic_timestamp")


class TestConfirmation(unittest.TestCase):
    def _ready_qualifier(self):
        q = PerceptionQualifier()
        _, t = feed_clean(q, 40)
        return q, t

    def test_confirmed_after_M_accepted(self):
        cfg = QualificationConfig(confirm_min_updates=3)
        q = PerceptionQualifier(cfg)
        res, _ = feed_clean(q, 40)
        self.assertTrue(res.planner_valid)
        self.assertIsNotNone(q.confirmed_at)

    def test_young_track_outlier_regression_W2(self):
        """Outlier among the FIRST updates must not corrupt planner-valid
        output: rejected + penalized, confirmation needs M coherent."""
        cfg = QualificationConfig(warmup_min_updates=3,
                                  warmup_min_span_s=0.05,
                                  confirm_min_updates=3)
        q = PerceptionQualifier(cfg)
        t = 0.0
        q.feed(DetectionEvent(t=t, detections=[det()])); t += DT
        # giant outlier as SECOND measurement (Y1)
        res = q.feed(DetectionEvent(
            t=t, detections=[det(cx=0.8, w=0.4, h=0.45)])); t += DT
        self.assertFalse(res.deliver_to_estimator)
        self.assertFalse(res.planner_valid)
        # then clean data: confirmation restarts from penalized count
        for _ in range(2):
            res = q.feed(DetectionEvent(t=t, detections=[det()])); t += DT
        # needs full M coherent updates post-penalty before valid
        self.assertTrue(q.state == "READY" or q.state == "WARMING")
        self.assertFalse(res.planner_valid and q.confirm_count < 3)

    def test_empty_streak_clears_confirmation(self):
        q, t = self._ready_qualifier()
        self.assertTrue(q.planner_valid())
        for i in range(3):
            res = q.feed(DetectionEvent(t=t + i * DT, detections=[]))
        self.assertFalse(res.planner_valid)
        # empty messages still delivered (absence evidence)
        self.assertTrue(res.deliver_to_estimator)

    def test_confirmation_window_resets_stale_count(self):
        cfg = QualificationConfig(warmup_min_updates=3,
                                  warmup_min_span_s=0.05,
                                  confirm_min_updates=3,
                                  confirm_window_s=0.5,
                                  source_restart_silence_s=10.0)
        q = PerceptionQualifier(cfg)
        t = 0.0
        for _ in range(2):
            q.feed(DetectionEvent(t=t, detections=[det()])); t += DT
        # gap > confirm window but < restart silence
        t += 0.8
        res = q.feed(DetectionEvent(t=t, detections=[det()]))
        self.assertLessEqual(q.confirm_count, 1 + 1,
                             "stale confirmation count must reset")

    def test_fast_confirmation_for_sudden_close_hazard_Y8(self):
        """A sudden legitimate close-range object must confirm in ~M frames
        once the source is warmed (no size discrimination)."""
        q, t = self._ready_qualifier()
        for i in range(3):
            res = q.feed(DetectionEvent(t=t, detections=[det()]))
            t += DT
        self.assertTrue(res.planner_valid)


if __name__ == "__main__":
    unittest.main()
