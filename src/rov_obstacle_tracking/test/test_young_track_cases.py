"""Phase-7B regression tests: young-track outliers (Y-cases) and startup
transient (S0) through the FULL qualification + estimator pipeline exactly
as the node runs it."""

from __future__ import annotations

import unittest

from rov_obstacle_tracking.metrics import evaluate
from rov_obstacle_tracking.perception_qualification import (
    PerceptionQualifier,
    QualificationConfig,
)
from rov_obstacle_tracking.replay import run_replay
from rov_obstacle_tracking.temporal_core import make_estimator
from rov_obstacle_tracking.test_cases import Y_CASES, generate_y_case

DT = 1.0 / 30.0


def run(case: str, method: str = "t2"):
    return run_replay(make_estimator(method), generate_y_case(case),
                      qualifier=PerceptionQualifier())


def first_valid_t(results):
    for r in results:
        if r.output.publish and r.output.obstacles:
            return r.t
    return None


class TestYoungTrackOutliers(unittest.TestCase):
    def test_Y0_clean_confirms_promptly(self):
        res = run("Y0")
        t = first_valid_t(res)
        self.assertIsNotNone(t)
        # warm-up (1 s) + confirmation (already satisfied during warm-up)
        self.assertLess(t, 1.4)

    def test_Y1_to_Y5_outlier_never_planner_valid(self):
        """A giant outlier among the first updates must never surface in
        planner-valid output for ANY method."""
        for case in ("Y1", "Y2", "Y3", "Y4", "Y5"):
            for method in ("t0", "t1", "t2", "t3"):
                res = run(case, method)
                for r in res:
                    for ob in r.output.obstacles:
                        self.assertLess(
                            abs(ob.cx - 0.5), 0.2,
                            f"{case}/{method}: outlier-contaminated output "
                            f"cx={ob.cx:.2f} at t={r.t:.2f}")

    def test_Y1_confirmation_not_blocked_forever(self):
        res = run("Y1")
        self.assertIsNotNone(first_valid_t(res))

    def test_Y6_alternating_never_confirms(self):
        res = run("Y6")
        self.assertIsNone(first_valid_t(res),
                          "wildly alternating detections must not confirm")

    def test_Y7_short_false_positive_ghost_bounded(self):
        """A 4-frame temporally-consistent FP is indistinguishable from a
        real object by temporal evidence alone (M=3 confirmation accepts it
        briefly). The pre-registered requirement is a BOUNDED ghost: the
        empty-streak rule must clear it within a fraction of a second."""
        for method in ("t0", "t1", "t2"):
            res = run("Y7", method)
            ghost = sum(DT for r in res
                        if r.output.obstacles
                        and not (r.gt or {}).get("present"))
            self.assertLess(ghost, 0.4,
                            f"{method}: FP ghost {ghost:.2f}s exceeds bound")

    def test_Y8_sudden_close_hazard_fast_confirmation(self):
        """After a healthy quiet stream, a sudden hazard must be planner-valid
        within a few frames (no full re-warm-up penalty)."""
        res = run("Y8")
        t = first_valid_t(res)
        self.assertIsNotNone(t)
        appear_t = 60 * DT
        self.assertLess(t - appear_t, 0.25,
                        f"confirmation latency {t - appear_t:.2f}s too slow "
                        "for a sudden close hazard")

    def test_Y9_small_far_object_not_discriminated(self):
        res = run("Y9")
        self.assertIsNotNone(first_valid_t(res),
                             "small/far legitimate object must confirm")


class TestStartupTransient(unittest.TestCase):
    def test_S0_no_planner_valid_during_garbage(self):
        for method in ("t0", "t1", "t2", "t3"):
            res = run("S0", method)
            garbage_end = 4 * DT
            for r in res:
                if r.t <= garbage_end + 1e-6:
                    self.assertEqual(
                        r.output.obstacles, [],
                        f"{method}: planner-valid during startup garbage")
            # And eventually the clean stream qualifies.
            self.assertIsNotNone(first_valid_t(res), method)

    def test_S0_qualification_time_reasonable(self):
        res = run("S0")
        t = first_valid_t(res)
        # garbage 4 frames + warm-up ~1 s (+confirm inside) => < 1.8 s total
        self.assertLess(t, 1.8)


if __name__ == "__main__":
    unittest.main()
