"""Tests for the realistic dead-reckoning odometry estimator (pure core)."""

import math
import random
import unittest

from rov_obstacle_sim_bridge.odometry_estimator import (
    OdometryEstimator,
    OdometryNoiseConfig,
)


class NoiseFreeExactnessTest(unittest.TestCase):
    def test_straight_forward_is_exact(self):
        est = OdometryEstimator()  # zero noise, no rng
        for _ in range(100):
            est.update(0.4, 0.0, 0.0, 0.0, 0.05)
        state = est.state()
        self.assertAlmostEqual(state.x, 0.4 * 100 * 0.05, places=6)  # 2.0 m
        self.assertAlmostEqual(state.y, 0.0, places=9)
        self.assertAlmostEqual(state.yaw, 0.0, places=9)

    def test_body_forward_at_90deg_maps_to_world_plus_y(self):
        # Yaw to +90 deg, then drive body-forward: world motion should be +y.
        est = OdometryEstimator(yaw0=math.pi / 2.0)
        est.update(1.0, 0.0, 0.0, 0.0, 1.0)
        state = est.state()
        self.assertAlmostEqual(state.x, 0.0, places=6)
        self.assertAlmostEqual(state.y, 1.0, places=6)

    def test_pure_yaw_rate_does_not_move_position(self):
        est = OdometryEstimator()
        for _ in range(50):
            est.update(0.0, 0.0, 0.0, 0.3, 0.05)
        state = est.state()
        self.assertAlmostEqual(state.x, 0.0, places=9)
        self.assertAlmostEqual(state.y, 0.0, places=9)
        self.assertAlmostEqual(state.yaw, 0.3 * 50 * 0.05, places=6)

    def test_scale_error_scales_displacement(self):
        cfg = OdometryNoiseConfig(dvl_scale_error=0.1)
        est = OdometryEstimator(cfg)
        est.update(1.0, 0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(est.state().x, 1.1, places=6)


class DriftTest(unittest.TestCase):
    def test_gyro_bias_causes_linear_yaw_drift(self):
        cfg = OdometryNoiseConfig(gyro_bias_rad_s=0.01)
        est = OdometryEstimator(cfg)
        for _ in range(200):
            est.update(0.0, 0.0, 0.0, 0.0, 0.05)  # no true motion
        # yaw drift = bias * total_time = 0.01 * 10 s = 0.1 rad
        self.assertAlmostEqual(est.state().yaw, 0.1, places=6)
        self.assertAlmostEqual(est.state().x, 0.0, places=9)

    def test_dvl_bias_causes_position_drift_while_stationary(self):
        cfg = OdometryNoiseConfig(dvl_bias_x_ms=0.02)
        est = OdometryEstimator(cfg)
        for _ in range(200):
            est.update(0.0, 0.0, 0.0, 0.0, 0.05)  # truly stationary
        # position drift = bias * total_time = 0.02 * 10 s = 0.2 m along body-x
        self.assertAlmostEqual(est.state().x, 0.2, places=6)
        self.assertAlmostEqual(est.state().y, 0.0, places=9)

    def test_estimate_diverges_from_truth_under_noise(self):
        cfg = OdometryNoiseConfig(
            gyro_bias_rad_s=0.002, dvl_bias_x_ms=0.01, dvl_noise_std_ms=0.02
        )
        rng = random.Random(7)
        est = OdometryEstimator(cfg, rng=lambda: rng.gauss(0.0, 1.0))
        true_x = 0.0
        for _ in range(400):
            est.update(0.4, 0.0, 0.0, 0.0, 0.05)
            true_x += 0.4 * 0.05
        # The estimate should have drifted measurably from the true displacement.
        self.assertGreater(abs(est.state().x - true_x) + abs(est.state().y), 0.05)


class DeterminismTest(unittest.TestCase):
    def test_same_seed_same_output(self):
        cfg = OdometryNoiseConfig(dvl_noise_std_ms=0.05, gyro_noise_std_rad_s=0.02)
        r1 = random.Random(123)
        r2 = random.Random(123)
        e1 = OdometryEstimator(cfg, rng=lambda: r1.gauss(0.0, 1.0))
        e2 = OdometryEstimator(cfg, rng=lambda: r2.gauss(0.0, 1.0))
        for _ in range(100):
            e1.update(0.3, 0.05, 0.0, 0.1, 0.05)
            e2.update(0.3, 0.05, 0.0, 0.1, 0.05)
        self.assertEqual(e1.state(), e2.state())

    def test_rng_none_means_no_noise(self):
        cfg = OdometryNoiseConfig(dvl_noise_std_ms=0.05, gyro_noise_std_rad_s=0.02)
        noisy_cfg_no_rng = OdometryEstimator(cfg, rng=None)
        clean = OdometryEstimator(OdometryNoiseConfig())
        for _ in range(100):
            noisy_cfg_no_rng.update(0.3, 0.0, 0.0, 0.0, 0.05)
            clean.update(0.3, 0.0, 0.0, 0.0, 0.05)
        self.assertEqual(noisy_cfg_no_rng.state(), clean.state())

    def test_non_positive_dt_is_ignored(self):
        est = OdometryEstimator()
        est.update(1.0, 0.0, 0.0, 0.0, 0.0)
        est.update(1.0, 0.0, 0.0, 0.0, -0.1)
        self.assertEqual(est.state().x, 0.0)


if __name__ == "__main__":
    unittest.main()
