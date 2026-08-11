"""Tests for the replay framework, D-case generator and metrics layer."""

from __future__ import annotations

import os
import tempfile
import unittest

from rov_obstacle_tracking.metrics import evaluate
from rov_obstacle_tracking.replay import (
    ReplayRecord,
    load_dataset,
    run_replay,
    save_dataset,
)
from rov_obstacle_tracking.temporal_core import make_estimator
from rov_obstacle_tracking.test_cases import (
    ALL_CASES,
    T_ENGAGE,
    base_stream,
    generate_case,
)


class TestDatasetRoundtrip(unittest.TestCase):
    def test_save_load_identical(self):
        recs = generate_case("D3")
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "d3.jsonl")
            save_dataset(recs, p)
            back = load_dataset(p)
        self.assertEqual(len(back), len(recs))
        self.assertEqual(back[50].to_json(), recs[50].to_json())

    def test_determinism(self):
        a = generate_case("D10", seed=7)
        b = generate_case("D10", seed=7)
        self.assertEqual([r.to_json() for r in a], [r.to_json() for r in b])

    def test_all_cases_generate(self):
        for c in ALL_CASES:
            recs = generate_case(c)
            self.assertGreater(len(recs), 50, c)


class TestCaseSemantics(unittest.TestCase):
    def test_silence_vs_empty_distinction(self):
        d5 = generate_case("D5")   # silence window
        d8 = generate_case("D8")   # SAME window but fresh-empty
        w5 = [r for r in d5 if T_ENGAGE + 1.0 <= r.t < T_ENGAGE + 3.0]
        w8 = [r for r in d8 if T_ENGAGE + 1.0 <= r.t < T_ENGAGE + 3.0]
        self.assertTrue(all(not r.message_present for r in w5))
        self.assertTrue(all(r.message_present and not r.detection_present
                            for r in w8))

    def test_gt_survives_dropout_window(self):
        """GT must stay present during silence (for prediction-error eval)."""
        d5 = generate_case("D5")
        w = [r for r in d5 if T_ENGAGE + 1.0 <= r.t < T_ENGAGE + 3.0]
        self.assertTrue(all(r.gt and r.gt.get("present") for r in w))

    def test_d14_ends_with_fresh_empty_and_absent_gt(self):
        d14 = generate_case("D14")
        tail = d14[-30:]
        self.assertTrue(all(r.message_present and not r.detection_present
                            for r in tail))
        self.assertTrue(all(not (r.gt or {}).get("present") for r in tail))


class TestReplayRunner(unittest.TestCase):
    def test_t0_silent_during_silence(self):
        results = run_replay(make_estimator("t0"), generate_case("D5"))
        window = [r for r in results
                  if T_ENGAGE + 1.1 <= r.t < T_ENGAGE + 2.9]
        self.assertTrue(all(not r.output.publish for r in window),
                        "T0 must publish NOTHING during upstream silence")

    def test_t1_holds_during_silence(self):
        results = run_replay(make_estimator("t1"), generate_case("D2"))
        window = [r for r in results if 5.05 <= r.t < 5.45]
        held = [r for r in window
                if r.output.obstacles and r.output.obstacles[0].is_predicted]
        self.assertGreater(len(held), 5)

    def test_t2_predicts_during_silence(self):
        results = run_replay(make_estimator("t2"), generate_case("D3"))
        window = [r for r in results if 5.1 <= r.t < 5.9]
        pred = [r for r in window
                if r.output.obstacles and r.output.obstacles[0].is_predicted]
        self.assertGreater(len(pred), 10)

    def test_metrics_availability_ordering(self):
        """During a 2 s dropout, availability must be T2/T1 > T0."""
        avail = {}
        for m in ("t0", "t1", "t2"):
            res = run_replay(make_estimator(m), generate_case("D4"))
            avail[m] = evaluate(res, method=m).availability
        self.assertGreater(avail["t2"], avail["t0"])
        self.assertGreater(avail["t1"], avail["t0"])

    def test_ghost_metric_flags_persistent_false_positive(self):
        """D13: brief FP. Estimators must not maintain long ghosts; the
        metric must measure whatever ghost time exists."""
        for m in ("t1", "t2"):
            res = run_replay(make_estimator(m), generate_case("D13"))
            met = evaluate(res, method=m)
            self.assertLess(met.ghost_track_time_s, 2.5,
                            f"{m} ghost too long: {met.ghost_track_time_s}")

    def test_d14_no_indefinite_ghost_after_pass(self):
        for m in ("t1", "t2", "t3"):
            res = run_replay(make_estimator(m), generate_case("D14"))
            met = evaluate(res, method=m)
            self.assertLess(met.ghost_track_time_s, 1.0,
                            f"{m} kept a ghost after legitimate disappearance")


if __name__ == "__main__":
    unittest.main()
