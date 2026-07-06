"""Tests for the pure-Python oracle geometry module."""

import math
import os
import tempfile
import unittest

from rov_obstacle_sim_bridge.oracle_geometry import (
    CameraConfig,
    ObstacleConfig,
    ProjectedObstacle,
    RoverPose2D,
    class_risk_weight,
    clamp,
    load_obstacle_config_yaml,
    project_obstacle,
    project_obstacles,
    world_to_camera,
)


class TestClamp(unittest.TestCase):
    def test_clamp_within_range(self):
        self.assertEqual(clamp(0.5, 0.0, 1.0), 0.5)

    def test_clamp_below(self):
        self.assertEqual(clamp(-2.0, 0.0, 1.0), 0.0)

    def test_clamp_above(self):
        self.assertEqual(clamp(3.0, 0.0, 1.0), 1.0)


class TestWorldToCamera(unittest.TestCase):
    def test_forward_obstacle_zero_yaw(self):
        rover = RoverPose2D(0.0, 0.0, 0.0, 0.0)
        cam = world_to_camera((5.0, 0.0, 0.0), rover)
        self.assertAlmostEqual(cam[0], 5.0)
        self.assertAlmostEqual(cam[1], 0.0)
        self.assertAlmostEqual(cam[2], 0.0)

    def test_left_obstacle_zero_yaw(self):
        """y_cam < 0 means left of the rover."""
        rover = RoverPose2D(0.0, 0.0, 0.0, 0.0)
        cam = world_to_camera((5.0, -1.5, 0.0), rover)
        self.assertAlmostEqual(cam[0], 5.0)
        self.assertAlmostEqual(cam[1], -1.5)

    def test_right_obstacle_zero_yaw(self):
        """y_cam > 0 means right of the rover."""
        rover = RoverPose2D(0.0, 0.0, 0.0, 0.0)
        cam = world_to_camera((5.0, 1.5, 0.0), rover)
        self.assertAlmostEqual(cam[0], 5.0)
        self.assertAlmostEqual(cam[1], 1.5)

    def test_yaw_rotation(self):
        """90-degree yaw rotates the forward axis to world +y."""
        rover = RoverPose2D(0.0, 0.0, 0.0, math.pi / 2)
        cam = world_to_camera((0.0, 5.0, 0.0), rover)
        self.assertAlmostEqual(cam[0], 5.0, places=5)
        self.assertAlmostEqual(cam[1], 0.0, places=5)


class TestProjection(unittest.TestCase):
    @staticmethod
    def _rover() -> RoverPose2D:
        return RoverPose2D(0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def _camera() -> CameraConfig:
        return CameraConfig()

    def test_1_central_obstacle_centers_at_0_5(self):
        obs = ObstacleConfig("central", "rock", (4.0, 0.0, 0.0), 0.5)
        proj = project_obstacle(
            obs,
            self._rover(),
            CameraConfig(max_detection_range_m=20.0),
        )
        self.assertIsNotNone(proj)
        self.assertAlmostEqual(proj.center_x, 0.5, places=4)

    def test_2_central_obstacle_bearing_near_zero(self):
        obs = ObstacleConfig("central", "rock", (4.0, 0.0, 0.0), 0.5)
        proj = project_obstacle(
            obs,
            self._rover(),
            CameraConfig(max_detection_range_m=20.0),
        )
        self.assertIsNotNone(proj)
        self.assertAlmostEqual(proj.bearing_rad, 0.0, places=4)

    def test_3_left_obstacle_center_x_less_half_bearing_negative(self):
        obs = ObstacleConfig("left", "rock", (5.0, -1.5, 0.0), 0.4)
        proj = project_obstacle(obs, self._rover(), self._camera())
        self.assertIsNotNone(proj)
        self.assertLess(proj.center_x, 0.5)
        self.assertLess(proj.bearing_rad, 0.0)

    def test_4_right_obstacle_center_x_greater_half_bearing_positive(self):
        obs = ObstacleConfig("right", "rock", (5.0, 1.5, 0.0), 0.4)
        proj = project_obstacle(obs, self._rover(), self._camera())
        self.assertIsNotNone(proj)
        self.assertGreater(proj.center_x, 0.5)
        self.assertGreater(proj.bearing_rad, 0.0)

    def test_5_behind_rover_rejected(self):
        obs = ObstacleConfig("behind", "rock", (-3.0, 0.0, 0.0), 0.5)
        proj = project_obstacle(obs, self._rover(), self._camera())
        self.assertIsNone(proj)

    def test_6_outside_horizontal_fov_rejected(self):
        """An obstacle at 90 degrees bearing is outside a 90-degree FOV (half-FOV = 45 deg)."""
        obs = ObstacleConfig("wide", "rock", (1.0, 10.0, 0.0), 0.3)
        proj = project_obstacle(obs, self._rover(), self._camera())
        self.assertIsNone(proj)

    def test_7_farther_than_max_range_rejected(self):
        obs = ObstacleConfig("far", "rock", (12.0, 0.0, 0.0), 0.5)
        proj = project_obstacle(obs, self._rover(), self._camera())
        self.assertIsNone(proj)

    def test_8_close_obstacle_larger_than_far(self):
        close_obs = ObstacleConfig("close", "rock", (2.0, 0.0, 0.0), 0.5)
        far_obs = ObstacleConfig("far", "rock", (8.0, 0.0, 0.0), 0.5)
        close_proj = project_obstacle(close_obs, self._rover(), self._camera())
        far_proj = project_obstacle(far_obs, self._rover(), self._camera())
        self.assertIsNotNone(close_proj)
        self.assertIsNotNone(far_proj)
        self.assertGreater(close_proj.width, far_proj.width)
        self.assertGreater(close_proj.height, far_proj.height)

    def test_9_close_central_higher_risk_than_far(self):
        close_obs = ObstacleConfig("close", "rock", (2.0, 0.0, 0.0), 0.5)
        far_obs = ObstacleConfig("far", "rock", (8.0, 0.0, 0.0), 0.5)
        close_proj = project_obstacle(close_obs, self._rover(), self._camera())
        far_proj = project_obstacle(far_obs, self._rover(), self._camera())
        self.assertIsNotNone(close_proj)
        self.assertIsNotNone(far_proj)
        self.assertGreater(close_proj.risk, far_proj.risk)

    def test_10_anchor_bounds_project_as_single_detection(self):
        obs = ObstacleConfig(
            "anchor_center",
            "anchor",
            (6.0, 0.0, 0.0),
            2.0,
            bounds=((5.8, -1.7, -1.8), (6.2, 1.7, 1.8)),
        )
        proj = project_obstacle(obs, self._rover(), self._camera())

        self.assertIsNotNone(proj)
        self.assertEqual(proj.class_name, "anchor")
        self.assertAlmostEqual(proj.center_x, 0.5, places=2)
        self.assertGreater(proj.width, 0.0)
        self.assertGreater(proj.height, 0.0)
        self.assertTrue(proj.confidence > 0.0)

    def test_11_partially_visible_anchor_bounds_are_clipped(self):
        obs = ObstacleConfig(
            "anchor_edge",
            "anchor",
            (9.0, -7.6, 0.0),
            2.5,
            bounds=((8.8, -9.6, -1.5), (9.2, -5.9, 1.5)),
        )
        proj = project_obstacle(
            obs,
            self._rover(),
            CameraConfig(max_detection_range_m=20.0),
        )

        self.assertIsNotNone(proj)
        self.assertLess(proj.center_x, 0.5)
        self.assertGreater(proj.width, 0.0)
        self.assertLessEqual(proj.width, 1.0)

    def test_12_anchor_class_weight_increases_oracle_risk(self):
        anchor = ObstacleConfig("anchor", "anchor", (5.0, 0.0, 0.0), 0.7)
        sphere = ObstacleConfig("sphere", "sphere", (5.0, 0.0, 0.0), 0.7)
        anchor_proj = project_obstacle(anchor, self._rover(), self._camera())
        sphere_proj = project_obstacle(sphere, self._rover(), self._camera())

        self.assertIsNotNone(anchor_proj)
        self.assertIsNotNone(sphere_proj)
        self.assertGreater(class_risk_weight("anchor"), class_risk_weight("sphere"))
        self.assertGreater(anchor_proj.risk, sphere_proj.risk)


class TestProjectObstacles(unittest.TestCase):
    def test_filters_rejected(self):
        obstacles = [
            ObstacleConfig("visible", "rock", (4.0, 0.0, 0.0), 0.5),
            ObstacleConfig("behind", "rock", (-3.0, 0.0, 0.0), 0.5),
            ObstacleConfig("too_far", "rock", (20.0, 0.0, 0.0), 0.5),
        ]
        rover = RoverPose2D(0.0, 0.0, 0.0, 0.0)
        camera = CameraConfig()
        result = project_obstacles(obstacles, rover, camera)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].name, "visible")


class TestYamlLoader(unittest.TestCase):
    def test_10_loads_config_correctly(self):
        config_path = os.path.join(
            os.path.dirname(__file__), "..", "config", "obstacles_simple.yaml"
        )
        obstacles = load_obstacle_config_yaml(config_path)
        self.assertEqual(len(obstacles), 4)
        self.assertEqual(obstacles[0].name, "anchor_01")
        self.assertEqual(obstacles[0].class_name, "anchor")
        self.assertEqual(obstacles[0].position, (4.0, 0.0, 0.0))
        self.assertAlmostEqual(obstacles[0].radius_m, 0.5)

    def test_11_missing_obstacles_key_returns_empty(self):
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        ) as fh:
            fh.write("other_key: value\n")
            fh.flush()
            obstacles = load_obstacle_config_yaml(fh.name)
        self.assertEqual(obstacles, [])

    def test_12_invalid_entry_raises_value_error(self):
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        ) as fh:
            fh.write("obstacles:\n")
            fh.write("  - name: bad\n")
            # missing class_name, position, radius_m
            fh.flush()
            with self.assertRaises(ValueError):
                load_obstacle_config_yaml(fh.name)


class TestRiskBounds(unittest.TestCase):
    def test_risk_in_zero_one(self):
        obs = ObstacleConfig("central", "rock", (4.0, 0.0, 0.0), 0.5)
        rover = RoverPose2D(0.0, 0.0, 0.0, 0.0)
        camera = CameraConfig()
        proj = project_obstacle(obs, rover, camera)
        self.assertIsNotNone(proj)
        self.assertGreaterEqual(proj.risk, 0.0)
        self.assertLessEqual(proj.risk, 1.0)

    def test_normalized_coords_clamped(self):
        obs = ObstacleConfig("edge", "rock", (3.0, 2.5, 0.0), 0.5)
        rover = RoverPose2D(0.0, 0.0, 0.0, 0.0)
        camera = CameraConfig()
        proj = project_obstacle(obs, rover, camera)
        if proj is not None:
            self.assertGreaterEqual(proj.center_x, 0.0)
            self.assertLessEqual(proj.center_x, 1.0)
            self.assertGreaterEqual(proj.center_y, 0.0)
            self.assertLessEqual(proj.center_y, 1.0)


if __name__ == "__main__":
    unittest.main()
