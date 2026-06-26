"""Tests for holoocean_pose_bridge helper functions (pure Python, no HoloOcean)."""

import math
import unittest

from rov_obstacle_sim_bridge.holoocean_pose_bridge_node import (
    fake_pose_at_time,
    pose_from_holoocean_state,
    quaternion_from_yaw,
)


class TestQuaternionFromYaw(unittest.TestCase):
    """quaternion_from_yaw produces correct yaw-only quaternions."""

    def test_zero_yaw_is_identity(self):
        q = quaternion_from_yaw(0.0)
        self.assertAlmostEqual(q.w, 1.0, places=6)
        self.assertAlmostEqual(q.x, 0.0, places=6)
        self.assertAlmostEqual(q.y, 0.0, places=6)
        self.assertAlmostEqual(q.z, 0.0, places=6)

    def test_ninety_deg_yaw(self):
        q = quaternion_from_yaw(math.pi / 2)
        self.assertAlmostEqual(q.w, math.cos(math.pi / 4), places=6)
        self.assertAlmostEqual(q.z, math.sin(math.pi / 4), places=6)

    def test_negative_yaw(self):
        q = quaternion_from_yaw(-math.pi / 4)
        self.assertAlmostEqual(q.w, math.cos(-math.pi / 8), places=6)
        self.assertAlmostEqual(q.z, math.sin(-math.pi / 8), places=6)

    def test_unit_norm(self):
        for angle in [0.0, 0.5, math.pi / 2, math.pi, -math.pi]:
            q = quaternion_from_yaw(angle)
            norm = math.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2)
            self.assertAlmostEqual(norm, 1.0, places=6, msg=f"norm failed at {angle}")


class TestPoseFromHolooceanState(unittest.TestCase):
    """pose_from_holoocean_state handles multiple state dict layouts."""

    # --- Layout 1: flat 'pose' list [x, y, z, qx, qy, qz, qw] ---
    def test_flat_pose_layout(self):
        state = {
            "agents": {
                "auv0": {
                    "pose": [1.0, 2.0, -3.0, 0.0, 0.0, 0.382683, 0.92388]
                }
            }
        }
        x, y, z, yaw = pose_from_holoocean_state(state, "auv0")
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.0)
        self.assertAlmostEqual(z, -3.0)
        # qw=cos(pi/8), qz=sin(pi/8) -> yaw = pi/4
        self.assertAlmostEqual(yaw, math.pi / 4, places=5)

    def test_flat_pose_layout_no_agents_key(self):
        """State dict with agent directly at top level (no 'agents' wrapper)."""
        state = {
            "auv0": {
                "pose": [5.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
            }
        }
        x, y, z, yaw = pose_from_holoocean_state(state, "auv0")
        self.assertAlmostEqual(x, 5.0)
        self.assertAlmostEqual(yaw, 0.0, places=6)

    # --- Layout 2: separate 'position' and 'orientation' dicts ---
    def test_position_orientation_layout(self):
        state = {
            "agents": {
                "auv0": {
                    "position": {"x": 3.0, "y": -1.0, "z": -2.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071, "w": 0.7071},
                }
            }
        }
        x, y, z, yaw = pose_from_holoocean_state(state, "auv0")
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, -1.0)
        self.assertAlmostEqual(z, -2.0)
        self.assertAlmostEqual(yaw, math.pi / 2, places=4)

    # --- Error handling ---
    def test_missing_agent_raises_key_error(self):
        state = {"agents": {"other_agent": {"pose": [0, 0, 0, 0, 0, 0, 1]}}}
        with self.assertRaises(KeyError):
            pose_from_holoocean_state(state, "missing_agent")

    def test_unrecognised_layout_raises_key_error(self):
        state = {"agents": {"auv0": {"some_other_key": True}}}
        with self.assertRaises(KeyError):
            pose_from_holoocean_state(state, "auv0")


class TestFakePoseAtTime(unittest.TestCase):
    """fake_pose_at_time produces deterministic linear drift."""

    def test_zero_velocity_stays_at_origin(self):
        for t in [0.0, 1.0, 10.0, 60.0]:
            x, y, z, yaw = fake_pose_at_time(t)
            self.assertAlmostEqual(x, 0.0)
            self.assertAlmostEqual(y, 0.0)
            self.assertAlmostEqual(z, 0.0)
            self.assertAlmostEqual(yaw, 0.0)

    def test_linear_drift_along_x(self):
        x, y, z, yaw = fake_pose_at_time(5.0, velocity_x=0.2)
        self.assertAlmostEqual(x, 1.0)  # 0.2 * 5
        self.assertAlmostEqual(y, 0.0)

    def test_custom_start_position(self):
        x, y, z, yaw = fake_pose_at_time(
            3.0, start_x=10.0, start_y=-2.0, start_z=-5.0
        )
        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, -2.0)
        self.assertAlmostEqual(z, -5.0)

    def test_yaw_is_constant(self):
        yaw_expected = math.radians(45.0)
        for t in [0.0, 1.0, 10.0]:
            _, _, _, yaw = fake_pose_at_time(t, yaw_rad=yaw_expected)
            self.assertAlmostEqual(yaw, yaw_expected)


if __name__ == "__main__":
    unittest.main()
