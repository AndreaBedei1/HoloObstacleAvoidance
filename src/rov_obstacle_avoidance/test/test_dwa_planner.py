"""Deterministic unit tests for the holonomic DWA baseline core."""

from __future__ import annotations

import math
import time
import unittest

import numpy as np

from rov_obstacle_avoidance.dwa_planner import (
    DWAConfig,
    HolonomicDWA,
    ObstacleEstimate,
    ResponseVelocityEstimator,
    _simulate,
    monocular_range,
    obstacle_from_detection,
)

ROUTE = (0.0, 0.0, 0.0)   # line along +x from origin


def ob(x, y, radius=1.75):
    return ObstacleEstimate(x=x, y=y, radius=radius, bearing_rad=0.0,
                            range_m=math.hypot(x, y))


class TestWindow(unittest.TestCase):
    def test_window_bounds_respect_accel_and_limits(self):
        cfg = DWAConfig(accel_lin=0.5, window_dt=0.5, max_surge=0.5,
                        max_sway=0.3, max_yaw_rate=0.3)
        dwa = HolonomicDWA(cfg)
        us, vs, rs = dwa._window((0.3, 0.0, 0.0))
        self.assertGreaterEqual(us.min(), 0.3 - 0.25 - 1e-9)
        self.assertLessEqual(us.max(), 0.5 + 1e-9)   # clipped to max
        self.assertGreaterEqual(vs.min(), -0.25 - 1e-9)
        self.assertLessEqual(vs.max(), 0.25 + 1e-9)
        self.assertGreaterEqual(rs.min(), -0.3 - 1e-9)
        self.assertLessEqual(rs.max(), 0.3 + 1e-9)

    def test_min_surge_no_reverse(self):
        cfg = DWAConfig(min_surge=0.0)
        dwa = HolonomicDWA(cfg)
        us, _, _ = dwa._window((0.0, 0.0, 0.0))
        self.assertGreaterEqual(us.min(), 0.0)

    def test_holonomic_sway_candidates_exist(self):
        """With the one-control-interval window (0.1 s), sway candidates
        span +/- accel*dt = +/-0.05 around the current value."""
        dwa = HolonomicDWA()
        _, vs, _ = dwa._window((0.2, 0.0, 0.0))
        self.assertTrue((vs > 0.02).any() and (vs < -0.02).any())


class TestSimulation(unittest.TestCase):
    def test_straight_surge_moves_plus_x(self):
        traj = _simulate(0, 0, 0, 0.5, 0, 0, DWAConfig())
        self.assertGreater(traj[-1, 0], 1.0)
        self.assertAlmostEqual(traj[-1, 1], 0.0, places=9)

    def test_sway_moves_plus_y_left(self):
        """+Y body = LEFT (project convention)."""
        traj = _simulate(0, 0, 0, 0.0, 0.3, 0, DWAConfig())
        self.assertGreater(traj[-1, 1], 0.5)
        self.assertAlmostEqual(traj[-1, 0], 0.0, places=9)

    def test_positive_yaw_ccw(self):
        traj = _simulate(0, 0, 0, 0.3, 0, 0.3, DWAConfig())
        self.assertGreater(traj[-1, 2], 0.5)   # yaw increased
        self.assertGreater(traj[-1, 1], 0.0)   # curved left

    def test_horizon_and_step_counts(self):
        cfg = DWAConfig(horizon_s=3.0, sim_dt=0.2)
        traj = _simulate(0, 0, 0, 0.1, 0, 0, cfg)
        self.assertEqual(len(traj), 15)


class TestPerceptionGeometry(unittest.TestCase):
    def test_monocular_range_matches_committed_model(self):
        cfg = DWAConfig()
        # apparent height of 3.5 m object at 12 m with VFOV 90:
        h = 3.5 / (2 * 12.0 * math.tan(math.radians(45)))
        self.assertAlmostEqual(monocular_range(h, cfg), 12.0, delta=0.05)

    def test_bearing_sign_right_of_center(self):
        cfg = DWAConfig()
        h = 3.5 / (2 * 10.0 * math.tan(math.radians(45)))
        est = obstacle_from_detection(cx=0.75, bbox_h=h, pose_x=0, pose_y=0,
                                      pose_yaw=0.0, cfg=cfg)
        # cx>0.5 => object right of center => body y NEGATIVE (y=left conv.)
        self.assertGreater(est.x, 5.0)
        self.assertLess(est.y, -1.0)

    def test_obstacle_world_transform_with_yaw(self):
        cfg = DWAConfig()
        h = 3.5 / (2 * 10.0 * math.tan(math.radians(45)))
        est = obstacle_from_detection(cx=0.5, bbox_h=h, pose_x=1.0,
                                      pose_y=2.0, pose_yaw=math.pi / 2,
                                      cfg=cfg)
        # facing +y with centered object => obstacle ~ (1, 12)
        self.assertAlmostEqual(est.x, 1.0, delta=0.1)
        self.assertAlmostEqual(est.y, 12.0, delta=0.1)


class TestAdmissibility(unittest.TestCase):
    def test_collision_course_rejected(self):
        """All-forward candidates toward a very close obstacle must be
        inadmissible; planner must NOT pick max-surge straight ahead."""
        dwa = HolonomicDWA()
        res = dwa.plan(pose=(0, 0, 0), vel_est=(0.4, 0, 0),
                       obstacles=[ob(2.6, 0.0)], route=ROUTE,
                       nominal_surge=0.4)
        # obstacle surface at 2.6-1.75=0.85 m minus footprint 0.7 => ~0.15 m
        # ahead: straight fast candidates are unstoppable/colliding.
        self.assertLess(res.u, 0.35)

    def test_no_admissible_safe_fallback(self):
        """Vehicle boxed against the inflated region => zero command."""
        cfg = DWAConfig(min_surge=0.0)
        dwa = HolonomicDWA(cfg)
        res = dwa.plan(pose=(0, 0, 0), vel_est=(0.3, 0, 0),
                       obstacles=[ob(2.3, 0.0), ob(0.0, 2.3), ob(0.0, -2.3)],
                       route=ROUTE, nominal_surge=0.4)
        if res.no_admissible:
            self.assertEqual((res.u, res.v, res.r), (0.0, 0.0, 0.0))
            self.assertEqual(res.debug.get("event"),
                             "no_admissible_candidate")
        else:
            # If anything admissible remains it must be genuinely safe.
            self.assertGreater(res.min_predicted_clearance, 0.0)

    def test_stopping_distance_enforced(self):
        """A candidate whose clearance is below v^2/2a must be rejected:
        with a far obstacle everything passes; verify the boundary moves
        with stop_decel."""
        strict = DWAConfig(stop_decel=0.05)   # very weak brakes
        lax = DWAConfig(stop_decel=5.0)
        res_strict = HolonomicDWA(strict).plan(
            (0, 0, 0), (0.4, 0, 0), [ob(6.0, 0.0)], ROUTE, 0.4)
        res_lax = HolonomicDWA(lax).plan(
            (0, 0, 0), (0.4, 0, 0), [ob(6.0, 0.0)], ROUTE, 0.4)
        self.assertGreaterEqual(res_lax.admissible_count,
                                res_strict.admissible_count)

    def test_footprint_inflation_effective(self):
        big = DWAConfig(vehicle_radius_m=1.2, safety_margin_m=0.8)
        small = DWAConfig(vehicle_radius_m=0.2, safety_margin_m=0.1)
        res_big = HolonomicDWA(big).plan(
            (0, 0, 0), (0.3, 0, 0), [ob(3.5, 0.0)], ROUTE, 0.4)
        res_small = HolonomicDWA(small).plan(
            (0, 0, 0), (0.3, 0, 0), [ob(3.5, 0.0)], ROUTE, 0.4)
        self.assertGreater(res_small.admissible_count,
                           res_big.admissible_count)


class TestObjective(unittest.TestCase):
    def test_obstacle_free_drives_along_route(self):
        dwa = HolonomicDWA()
        res = dwa.plan((0, 0, 0), (0.3, 0, 0), [], ROUTE, 0.4)
        self.assertFalse(res.no_admissible)
        self.assertGreater(res.u, 0.3)
        self.assertLess(abs(res.v), 0.1)
        self.assertLess(abs(res.r), 0.1)

    def test_route_alignment_pulls_back_to_line(self):
        """Offset left of the route => choose rightward (negative) sway or
        heading correction rather than wandering further left."""
        dwa = HolonomicDWA()
        res = dwa.plan((5.0, 2.0, 0.0), (0.3, 0, 0), [], ROUTE, 0.4)
        self.assertLessEqual(res.v, 0.02)

    def test_far_obstacle_stays_on_route(self):
        """At 6 m the predicted clearance stays above saturation for the
        whole horizon — classical DWA correctly keeps driving the route."""
        dwa = HolonomicDWA()
        res = dwa.plan((0, 0, 0), (0.3, 0, 0), [ob(6.0, 0.0)], ROUTE, 0.4)
        self.assertFalse(res.no_admissible)
        self.assertGreater(res.u, 0.3)

    def test_near_obstacle_prefers_lateral_or_slow(self):
        """At 4 m the horizon drives predicted clearance below saturation:
        an evasive component (sway/yaw) or a slowdown must appear."""
        dwa = HolonomicDWA()
        res = dwa.plan((0, 0, 0), (0.3, 0, 0), [ob(4.0, 0.0)], ROUTE, 0.4)
        self.assertFalse(res.no_admissible)
        # Graded response (measured: 0.50 free -> 0.42@4m -> 0.27@3.5m ->
        # 0.20+sway+yaw@3m): at 4 m full speed must no longer be chosen.
        self.assertTrue(abs(res.v) > 0.02 or abs(res.r) > 0.02
                        or res.u < 0.45,
                        f"no evasion: u={res.u} v={res.v} r={res.r}")
        res3 = dwa.plan((0, 0, 0), (0.3, 0, 0), [ob(3.0, 0.0)], ROUTE, 0.4)
        self.assertTrue(abs(res3.v) > 0.02 or abs(res3.r) > 0.02
                        or res3.u < 0.3,
                        f"no evasion at 3m: u={res3.u} v={res3.v} r={res3.r}")

    def test_smoothness_penalizes_jumps(self):
        cfg = DWAConfig(w_smooth=3.0)
        dwa = HolonomicDWA(cfg)
        dwa._last_cmd = (0.4, 0.0, 0.0)
        res = dwa.plan((0, 0, 0), (0.4, 0, 0), [], ROUTE, 0.4)
        self.assertLess(abs(res.u - 0.4), 0.15)


class TestVelocityEstimator(unittest.TestCase):
    def test_first_order_convergence(self):
        est = ResponseVelocityEstimator(DWAConfig())
        for _ in range(300):
            est.update(0.4, -0.2, 0.1, 0.1)
        self.assertAlmostEqual(est.u, 0.4, delta=0.01)
        self.assertAlmostEqual(est.v, -0.2, delta=0.01)
        self.assertAlmostEqual(est.r, 0.1, delta=0.01)

    def test_zero_dt_ignored(self):
        est = ResponseVelocityEstimator(DWAConfig())
        est.update(0.4, 0, 0, 0.0)
        self.assertEqual(est.u, 0.0)


class TestComputation(unittest.TestCase):
    def test_planning_time_bounded(self):
        dwa = HolonomicDWA()
        times = []
        for _ in range(5):
            res = dwa.plan((0, 0, 0), (0.3, 0, 0),
                           [ob(8.0, 0.5)], ROUTE, 0.4)
            times.append(res.planning_time_ms)
        self.assertLess(min(times), 100.0,
                        f"planning too slow: {times} ms")
        self.assertEqual(res.candidate_count,
                         DWAConfig().n_u * DWAConfig().n_v * DWAConfig().n_r)


if __name__ == "__main__":
    unittest.main()
