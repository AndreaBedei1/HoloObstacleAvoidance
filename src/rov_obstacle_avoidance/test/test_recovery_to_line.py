"""Closed-loop tests for the committed circumnavigation planner.

Drives the deterministic ``LocalAvoidancePlanner`` through a holonomic kinematic
simulation that mirrors the HoloOcean sim server motion model, with a monocular
anchor "detector" that produces realistic image-space detections (bbox size from
range, bearing from geometry, and loses the target when it leaves the FOV or
goes behind).

The tests assert the behaviour the user requires:
  * a SINGLE committed maneuver (no continuous left/right oscillation),
  * no collision -- the vehicle keeps clear of the anchor even if the detector
    drops out mid-maneuver (the pass decision is odometry-gated, not
    detection-gated),
  * the vehicle passes the anchor and returns to the original straight line.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import unittest

from rov_obstacle_avoidance.planner import (
    LocalAvoidancePlanner,
    PlannerConfig,
    VelocityCommand,
    estimate_range,
    ObstacleObservation,
)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def _step(pose: Pose, cmd: VelocityCommand, dt: float) -> Pose:
    yaw = pose.yaw + cmd.yaw_rate * dt
    dx = cmd.surge * dt * math.cos(yaw) - cmd.sway * dt * math.sin(yaw)
    dy = cmd.surge * dt * math.sin(yaw) + cmd.sway * dt * math.cos(yaw)
    return Pose(pose.x + dx, pose.y + dy, yaw)


class MonocularAnchor:
    """Realistic monocular detector for a box-ish anchor at a world point."""

    def __init__(
        self,
        x: float,
        y: float,
        height_m: float = 3.5,
        half_width_m: float = 0.6,
        hfov_deg: float = 90.0,
        vfov_deg: float = 90.0,
        max_range_m: float = 30.0,
        drop_when_offset_gt: float | None = None,
    ) -> None:
        self.x = x
        self.y = y
        self.height_m = height_m
        self.half_width_m = half_width_m
        self.hfov = math.radians(hfov_deg)
        self.vfov = math.radians(vfov_deg)
        self.max_range_m = max_range_m
        # Simulate a detector dropout: stop reporting once the vehicle has
        # strafed more than this lateral offset (to prove the pass is not
        # detection-gated).
        self.drop_when_offset_gt = drop_when_offset_gt

    def detect(self, pose: Pose) -> list[ObstacleObservation]:
        if self.drop_when_offset_gt is not None and abs(pose.y) > self.drop_when_offset_gt:
            return []
        dxw = self.x - pose.x
        dyw = self.y - pose.y
        rx = math.cos(pose.yaw) * dxw + math.sin(pose.yaw) * dyw  # forward
        ry = -math.sin(pose.yaw) * dxw + math.cos(pose.yaw) * dyw  # left
        rng = math.hypot(rx, ry)
        if rx <= 0.2 or rng > self.max_range_m:
            return []  # behind the camera or out of range
        bearing_geo = math.atan2(ry, rx)  # >0 == left of the camera axis
        if abs(bearing_geo) >= self.hfov / 2.0:
            return []  # outside the horizontal FOV
        # Codebase convention: object on the LEFT -> center_x < 0.5.
        center_x = 0.5 - bearing_geo / self.hfov
        height_frac = _clamp(2.0 * math.atan((self.height_m * 0.5) / rng) / self.vfov, 0.0, 1.0)
        width_frac = _clamp(2.0 * math.atan(self.half_width_m / rng) / self.hfov, 0.0, 1.0)
        area = height_frac * width_frac
        centrality = 1.0 - 2.0 * abs(center_x - 0.5)
        area_term = _clamp(area * 4.0, 0.0, 1.0)
        risk = _clamp(0.9 * (0.55 * centrality + 0.45 * area_term), 0.0, 1.0)
        bearing_rad = (center_x - 0.5) * self.hfov
        return [
            ObstacleObservation(
                class_name="anchor",
                confidence=0.9,
                center_x=center_x,
                center_y=0.5,
                width=width_frac,
                height=height_frac,
                bearing_rad=bearing_rad,
                apparent_area=area,
                risk=risk,
                is_tracking_valid=True,
            )
        ]


def _clamp(v: float, lo: float, hi: float) -> float:
    return min(max(v, lo), hi)


PROD_CONFIG = PlannerConfig()  # ships with the committed-circumnavigation defaults


def run_closed_loop(
    config: PlannerConfig,
    anchor: MonocularAnchor,
    *,
    nominal: VelocityCommand = VelocityCommand(surge=0.4),
    duration_s: float = 90.0,
    dt: float = 0.05,
) -> dict:
    planner = LocalAvoidancePlanner(config)
    pose = Pose(0.0, 0.0, 0.0)
    states: list[str] = []
    avoiding_entries = 0
    recovering_entries = 0
    prev_state = None
    min_dist_to_anchor = float("inf")
    max_forward = 0.0
    for k in range(int(duration_s / dt)):
        now = k * dt
        planner.update_nominal_command(nominal, now)
        planner.update_obstacles(anchor.detect(pose), now)
        planner.update_pose(pose.x, pose.y, pose.yaw, now)
        out = planner.compute(now)

        s = out.state.value
        if not states or states[-1] != s:
            states.append(s)
        if s in ("AVOIDING_LEFT", "AVOIDING_RIGHT") and prev_state not in (
            "AVOIDING_LEFT",
            "AVOIDING_RIGHT",
        ):
            avoiding_entries += 1
        if s == "RECOVERING" and prev_state != "RECOVERING":
            recovering_entries += 1
        prev_state = s

        min_dist_to_anchor = min(min_dist_to_anchor, math.hypot(pose.x - anchor.x, pose.y - anchor.y))
        max_forward = max(max_forward, pose.x)
        pose = _step(pose, out.command, dt)

    lateral = abs(pose.y)
    yaw_err_deg = math.degrees(math.atan2(math.sin(pose.yaw), math.cos(pose.yaw)))
    return {
        "states": states,
        "avoiding_entries": avoiding_entries,
        "recovering_entries": recovering_entries,
        "min_dist_to_anchor_m": min_dist_to_anchor,
        "passed_anchor": max_forward > anchor.x + 1.0,
        "final_lateral_error_m": lateral,
        "final_yaw_error_deg": yaw_err_deg,
        "final_pose": pose,
        "returned_to_original_line": lateral < 0.5 and abs(yaw_err_deg) < 10.0,
    }


class RangeEstimateTest(unittest.TestCase):
    def test_larger_bbox_is_closer(self):
        vfov = math.radians(90.0)
        near = ObstacleObservation(height=0.4)
        far = ObstacleObservation(height=0.15)
        self.assertLess(
            estimate_range(near, vfov, 3.5, 40.0),
            estimate_range(far, vfov, 3.5, 40.0),
        )

    def test_range_matches_pinhole(self):
        vfov = math.radians(90.0)
        # An object of height 3.5 m filling ~19% of a 90-deg VFOV is ~12 m away.
        obs = ObstacleObservation(height=0.19)
        rng = estimate_range(obs, vfov, 3.5, 40.0)
        self.assertGreater(rng, 9.0)
        self.assertLess(rng, 15.0)

    def test_zero_height_returns_max(self):
        self.assertEqual(estimate_range(ObstacleObservation(height=0.0), 1.5, 3.5, 40.0), 40.0)


class CommittedCircumnavigationTest(unittest.TestCase):
    def test_single_maneuver_passes_and_returns(self):
        result = run_closed_loop(PROD_CONFIG, MonocularAnchor(12.0, 0.0))
        states = result["states"]
        self.assertIn("APPROACH_OBSTACLE", states)
        self.assertTrue(any(s.startswith("AVOIDING_") for s in states))
        self.assertIn("RECOVERING", states)
        self.assertEqual(states[-1], "NORMAL")
        # Committed, not oscillating: one avoidance and one recovery.
        self.assertEqual(result["avoiding_entries"], 1)
        self.assertEqual(result["recovering_entries"], 1)
        # Actually went around and past the anchor.
        self.assertTrue(result["passed_anchor"])
        # And came back to the original straight line.
        self.assertLess(result["final_lateral_error_m"], 0.5)
        self.assertLess(abs(result["final_yaw_error_deg"]), 10.0)
        self.assertTrue(result["returned_to_original_line"])

    def test_no_collision(self):
        result = run_closed_loop(PROD_CONFIG, MonocularAnchor(12.0, 0.0))
        # Kept well clear of the anchor throughout.
        self.assertGreater(result["min_dist_to_anchor_m"], 1.5)

    def test_no_collision_on_detection_dropout(self):
        # The detector stops reporting once the vehicle has strafed 1.2 m -- the
        # old logic would then return to the line and drive into the anchor.
        result = run_closed_loop(
            PROD_CONFIG, MonocularAnchor(12.0, 0.0, drop_when_offset_gt=1.2)
        )
        self.assertGreater(result["min_dist_to_anchor_m"], 1.5)
        self.assertTrue(result["passed_anchor"])
        self.assertTrue(result["returned_to_original_line"])
        # Still a single committed maneuver despite the dropout.
        self.assertLessEqual(result["avoiding_entries"], 1)

    def test_offset_anchor_picks_a_side_and_returns(self):
        # Anchor slightly to the right -> vehicle should go left and still return.
        result = run_closed_loop(PROD_CONFIG, MonocularAnchor(12.0, 0.6))
        self.assertTrue(result["passed_anchor"])
        self.assertTrue(result["returned_to_original_line"])
        self.assertLessEqual(result["avoiding_entries"], 2)


if __name__ == "__main__":
    unittest.main()
