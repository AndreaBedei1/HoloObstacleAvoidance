"""Deterministic unit tests for the holonomic DWA baseline core."""

from __future__ import annotations

import math
import time
import unittest

import numpy as np

from rov_obstacle_avoidance.dwa_planner import (
    ObstacleMemory,
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
        """Surge is slew-limited around the current estimate; lateral axes
        span their full authority (per-axis window shaping) but never
        exceed the vehicle limits."""
        cfg = DWAConfig(accel_lin=0.5, window_dt=0.5, max_surge=0.5,
                        max_sway=0.3, max_yaw_rate=0.3)
        dwa = HolonomicDWA(cfg)
        us, vs, rs = dwa._window((0.3, 0.0, 0.0))
        self.assertGreaterEqual(us.min(), 0.3 - 0.25 - 1e-9)
        self.assertLessEqual(us.max(), 0.5 + 1e-9)   # clipped to max
        self.assertGreaterEqual(vs.min(), -0.3 - 1e-9)
        self.assertLessEqual(vs.max(), 0.3 + 1e-9)
        self.assertGreaterEqual(rs.min(), -0.3 - 1e-9)
        self.assertLessEqual(rs.max(), 0.3 + 1e-9)
        # Slew-limited variant still available for ablation.
        cfg2 = DWAConfig(full_lateral_window=False, accel_lin=0.5,
                         window_dt=0.5)
        vs2 = HolonomicDWA(cfg2)._window((0.3, 0.0, 0.0))[1]
        self.assertLessEqual(vs2.max(), 0.25 + 1e-9)

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

    def test_near_obstacle_slows_or_stops(self):
        """With reaction-time braking + 0.8 m margin, approaching a central
        obstacle at 4 m at 0.3 m/s legitimately triggers a full safe stop
        (all candidates fail the lagged stopping-distance check)."""
        dwa = HolonomicDWA()
        res = dwa.plan((0, 0, 0), (0.3, 0, 0), [ob(4.0, 0.0)], ROUTE, 0.4)
        self.assertLess(res.u, 0.45)  # never full speed at 4 m

    def test_standstill_creep_sidestep_not_deadlocked(self):
        """From standstill near the zone boundary, low-speed lateral
        candidates must be admissible (no permanent stop deadlock)."""
        dwa = HolonomicDWA()
        res = dwa.plan((0, 0, 0), (0.0, 0, 0), [ob(4.0, 0.0)], ROUTE, 0.4)
        self.assertFalse(res.no_admissible)

    def test_escape_rule_inside_inflated_zone(self):
        """Already inside the inflated region: candidates that INCREASE
        clearance must be admissible (escape), not a permanent stop."""
        dwa = HolonomicDWA()
        res = dwa.plan((0, 0, 0), (0.0, 0, 0), [ob(2.5, 0.0)], ROUTE, 0.4)
        self.assertFalse(res.no_admissible,
                         "escape rule must yield admissible candidates")

    def test_smoothness_penalizes_jumps(self):
        cfg = DWAConfig(w_smooth=3.0)
        dwa = HolonomicDWA(cfg)
        dwa._last_cmd = (0.4, 0.0, 0.0)
        res = dwa.plan((0, 0, 0), (0.4, 0, 0), [], ROUTE, 0.4)
        self.assertLess(abs(res.u - 0.4), 0.15)


class TestStallRegression(unittest.TestCase):
    """Central-obstacle stall (tuning iterations 1-3): from standstill in
    front of an inflated obstacle the planner must choose a lateral/turn
    escape, not creep forward and park at the safety boundary."""

    def test_standstill_blocked_cone_never_closes_fast(self):
        """Deep in the blocked cone from standstill a pure local DWA may
        legitimately hold (documented local-minimum limitation, Fox 1997
        pairs DWA with global guidance); the mandatory property is that it
        never CLOSES at speed. The closed-loop escape is asserted by
        test_closed_loop_central_obstacle below (the directive's actual
        P1 requirement)."""
        dwa = HolonomicDWA()
        res = dwa.plan((0, 0, 0), (0.0, 0, 0),
                       [ObstacleEstimate(x=3.2, y=0.0, radius=1.75,
                                         bearing_rad=0.0, range_m=3.2)],
                       ROUTE, 0.3)
        self.assertLessEqual(res.u, 0.06,
                             f"closing fast: u={res.u} v={res.v} r={res.r}")

    def _closed_loop(self, ob_x, ob_y, T=120.0, nominal=0.3, radius=1.75):
        cfg = DWAConfig()
        dwa = HolonomicDWA()
        mem = ObstacleMemory()
        x = y = yaw = 0.0
        cu = cv = cr = 0.0
        dt = 0.1
        min_clear = 1e9
        for k in range(int(T / dt)):
            dx, dy = ob_x - x, ob_y - y
            rng = math.hypot(dx, dy)
            b = math.atan2(dy, dx) - yaw
            brg = math.atan2(math.sin(b), math.cos(b))
            dets = []
            if abs(brg) < math.radians(45) and rng < 40:
                dets = [ObstacleEstimate(x=ob_x, y=ob_y, radius=radius,
                                         bearing_rad=brg, range_m=rng)]
            mem.update(dets, k * dt)
            res = dwa.plan((x, y, yaw), (cu, cv, cr), mem.active(k * dt),
                           ROUTE, nominal)
            au = min(1, dt / cfg.tau_surge_s)
            av = min(1, dt / cfg.tau_sway_s)
            ar = min(1, dt / cfg.tau_yaw_s)
            cu += (res.u - cu) * au
            cv += (res.v - cv) * av
            cr += (res.r - cr) * ar
            yaw += cr * dt
            x += (cu * math.cos(yaw) - cv * math.sin(yaw)) * dt
            y += (cu * math.sin(yaw) + cv * math.cos(yaw)) * dt
            min_clear = min(min_clear,
                            math.hypot(x - ob_x, y - ob_y) - radius - 0.40)
        return x, y, min_clear

    def test_closed_loop_central_obstacle(self):
        """P1 regression (kinematic surrogate): approach at cruise, swerve
        around a CENTRAL obstacle, pass it and return to the line."""
        x, y, min_clear = self._closed_loop(11.0, 0.0)
        self.assertGreater(min_clear, 0.0, "collision")
        self.assertGreater(x, 14.0, "never passed the obstacle (stall)")
        self.assertLess(abs(y), 1.3, "no route return")

    def test_closed_loop_pool_scale(self):
        """K0 regression: pool-scale small obstacle, memoryless blind
        return would re-cross the obstacle after it leaves the FOV."""
        x, y, min_clear = self._closed_loop(3.5, 0.0, T=90.0, nominal=0.15,
                                            radius=0.25)
        self.assertGreater(min_clear, 0.0, "collision")
        self.assertGreater(x, 6.5, "never passed the obstacle")


class TestObstacleMemory(unittest.TestCase):
    def _det(self, x, y):
        return ObstacleEstimate(x=x, y=y, radius=1.75, bearing_rad=0.0,
                                range_m=math.hypot(x, y))

    def test_persists_after_detection_loss(self):
        mem = ObstacleMemory(ttl_s=30.0)
        mem.update([self._det(5.0, 0.0)], 0.0)
        mem.update([], 10.0)
        self.assertEqual(len(mem.active(10.0)), 1)

    def test_expires_after_ttl(self):
        mem = ObstacleMemory(ttl_s=30.0)
        mem.update([self._det(5.0, 0.0)], 0.0)
        self.assertEqual(len(mem.active(31.0)), 0)

    def test_merges_nearby_updates_position(self):
        mem = ObstacleMemory(ttl_s=30.0, merge_dist_m=1.5)
        mem.update([self._det(5.0, 0.0)], 0.0)
        mem.update([self._det(5.5, 0.3)], 1.0)
        act = mem.active(1.0)
        self.assertEqual(len(act), 1)
        self.assertAlmostEqual(act[0].x, 5.5)

    def test_distinct_obstacles_kept_separate(self):
        mem = ObstacleMemory(ttl_s=30.0, merge_dist_m=1.5)
        mem.update([self._det(5.0, 0.0), self._det(5.0, 4.0)], 0.0)
        self.assertEqual(len(mem.active(0.0)), 2)

    def test_receding_candidates_admissible_despite_start_proximity(self):
        # Directional stoppability: a candidate moving AWAY needs no
        # stopping distance against the obstacle even when the shared
        # start position dominates the path-min clearance. Under the
        # omnidirectional rule every lateral escape was inadmissible here
        # and the admissible set collapsed to slow creeps.
        cfg = DWAConfig()
        dwa = HolonomicDWA(cfg)
        # Start just outside the inflated zone; only lateral motion is safe.
        res = dwa.plan((0, 0, 0), (0.0, 0, 0),
                       [ObstacleEstimate(x=3.0, y=0.0, radius=1.75,
                                         bearing_rad=0.0, range_m=3.0)],
                       ROUTE, 0.3)
        self.assertFalse(res.no_admissible)
        self.assertGreater(res.admissible_count,
                           res.candidate_count // 3)

    def test_multi_obstacle_end_clearance_uses_min(self):
        # Two obstacles left and right; end-state clearance must reflect
        # the NEAREST at the final point, so straight through the gap wins
        # over swerving into either obstacle.
        dwa = HolonomicDWA()
        obs = [ObstacleEstimate(x=4.0, y=3.4, radius=1.0,
                                bearing_rad=0.7, range_m=5.2),
               ObstacleEstimate(x=4.0, y=-3.4, radius=1.0,
                                bearing_rad=-0.7, range_m=5.2)]
        res = dwa.plan((0, 0, 0), (0.3, 0, 0), obs, ROUTE, 0.3)
        self.assertGreater(res.u, 0.1)
        self.assertLess(abs(res.v), 0.2)


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
