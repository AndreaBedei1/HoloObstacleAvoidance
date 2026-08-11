"""Unit tests for the T0-T3 temporal estimator core (pure Python)."""

from __future__ import annotations

import math
import unittest

import numpy as np

from rov_obstacle_tracking.temporal_core import (
    AdaptiveNoiseModel,
    Detection,
    DetectionEvent,
    KFConfig,
    T0Raw,
    T1Config,
    T1HoldEMA,
    T2FixedKalman,
    T3AdaptiveKalman,
    cv_process_noise,
    cv_transition,
    detection_features,
    make_estimator,
)

DT = 1.0 / 30.0


def det(cx=0.5, cy=0.5, w=0.2, h=0.25, conf=0.9, cls="anchor"):
    return Detection(class_name=cls, confidence=conf, cx=cx, cy=cy, w=w, h=h)


def feed(est, t0, seconds, gen, rate=30.0):
    """Feed messages at `rate` and tick at the same cadence; return outputs."""
    outs = []
    t = t0
    n = int(seconds * rate)
    for i in range(n):
        d = gen(t, i)
        if d == "silence":
            pass
        elif d is None:
            est.on_message(DetectionEvent(t=t, detections=[]))
        else:
            est.on_message(DetectionEvent(t=t, detections=[d]))
        outs.append((t, est.tick(t)))
        t += 1.0 / rate
    return outs


class TestT0(unittest.TestCase):
    def test_passthrough_and_silence(self):
        t0 = T0Raw()
        t0.on_message(DetectionEvent(t=0.0, detections=[det()]))
        out = t0.tick(0.0)
        self.assertTrue(out.publish)
        self.assertEqual(len(out.obstacles), 1)
        self.assertFalse(out.obstacles[0].is_predicted)
        # SILENCE: T0 must publish nothing at all (no hold, no empty array).
        out2 = t0.tick(DT)
        self.assertFalse(out2.publish)

    def test_fresh_empty_is_published_as_empty(self):
        t0 = T0Raw()
        t0.on_message(DetectionEvent(t=0.0, detections=[]))
        out = t0.tick(0.0)
        self.assertTrue(out.publish)
        self.assertEqual(out.obstacles, [])


class TestT1(unittest.TestCase):
    def test_hold_through_silence_then_expiry(self):
        cfg = T1Config(hold_duration_s=0.5, ema_alpha=1.0)
        t1 = T1HoldEMA(cfg)
        t1.on_message(DetectionEvent(t=0.0, detections=[det(cx=0.4)]))
        out = t1.tick(0.0)
        self.assertEqual(len(out.obstacles), 1)
        self.assertFalse(out.obstacles[0].is_predicted)
        # silence 0.3 s -> held, marked predicted, age grows
        out = t1.tick(0.3)
        self.assertEqual(len(out.obstacles), 1)
        self.assertTrue(out.obstacles[0].is_predicted)
        self.assertAlmostEqual(out.obstacles[0].time_since_meas_s, 0.3)
        self.assertAlmostEqual(out.obstacles[0].cx, 0.4)
        # past hold duration -> publishes EMPTY (not silent, not ghost)
        out = t1.tick(0.6)
        self.assertTrue(out.publish)
        self.assertEqual(out.obstacles, [])

    def test_fresh_empty_streak_drops_before_hold_expiry(self):
        cfg = T1Config(hold_duration_s=5.0, empty_misses_to_drop=3)
        t1 = T1HoldEMA(cfg)
        t1.on_message(DetectionEvent(t=0.0, detections=[det()]))
        for i in range(3):
            t1.on_message(DetectionEvent(t=0.1 + i * 0.1, detections=[]))
        out = t1.tick(0.5)
        self.assertEqual(out.obstacles, [],
                         "3 fresh-empty msgs must drop the track even with a "
                         "long hold window (absence evidence != silence)")

    def test_ema_smoothing(self):
        cfg = T1Config(ema_alpha=0.5)
        t1 = T1HoldEMA(cfg)
        t1.on_message(DetectionEvent(t=0.0, detections=[det(cx=0.4)]))
        t1.on_message(DetectionEvent(t=DT, detections=[det(cx=0.6)]))
        out = t1.tick(DT)
        self.assertAlmostEqual(out.obstacles[0].cx, 0.5)


class TestKFModel(unittest.TestCase):
    def test_transition_matrix(self):
        F = cv_transition(0.1)
        x = np.zeros(8)
        x[0] = 0.5
        x[4] = 1.0  # dcx = 1/s
        x2 = F @ x
        self.assertAlmostEqual(x2[0], 0.6)

    def test_process_noise_psd_scaling(self):
        Q1 = cv_process_noise(0.1, 1.0, 1.0)
        Q2 = cv_process_noise(0.2, 1.0, 1.0)
        self.assertAlmostEqual(Q2[4, 4] / Q1[4, 4], 2.0)

    def test_constant_velocity_convergence_and_prediction(self):
        """Feed a constant-velocity target; KF velocity must converge and
        dropout prediction must extrapolate correctly (analytic check)."""
        cfg = KFConfig(max_prediction_horizon_s=2.0)
        kf = T2FixedKalman(cfg)
        v = 0.05  # cx units/s
        for i in range(90):  # 3 s of measurements
            t = i * DT
            kf.on_message(DetectionEvent(
                t=t, detections=[det(cx=0.2 + v * t)]))
            kf.tick(t)
        tr = kf.track
        self.assertIsNotNone(tr)
        self.assertAlmostEqual(tr.x[4], v, delta=0.01)
        # SILENCE for 1 s: prediction should advance cx by ~v * 1.0.
        t_meas_end = 89 * DT
        out = kf.tick(t_meas_end + 1.0)
        self.assertEqual(len(out.obstacles), 1)
        ob = out.obstacles[0]
        self.assertTrue(ob.is_predicted)
        expected = 0.2 + v * t_meas_end + v * 1.0
        self.assertAlmostEqual(ob.cx, expected, delta=0.02)

    def test_prediction_horizon_expiry(self):
        cfg = KFConfig(max_prediction_horizon_s=0.5)
        kf = T2FixedKalman(cfg)
        kf.on_message(DetectionEvent(t=0.0, detections=[det()]))
        out = kf.tick(0.4)
        self.assertEqual(len(out.obstacles), 1)
        out = kf.tick(0.6)
        self.assertEqual(out.obstacles, [])
        self.assertEqual(kf.last_drop_reason, "prediction_horizon_exceeded")

    def test_outlier_gated_not_absorbed(self):
        kf = T2FixedKalman()
        for i in range(60):
            t = i * DT
            kf.on_message(DetectionEvent(t=t, detections=[det(cx=0.5)]))
            kf.tick(t)
        cx_before = kf.track.x[0]
        gated_before = kf.track.gated_out_count
        # One wild outlier
        kf.on_message(DetectionEvent(
            t=60 * DT, detections=[det(cx=0.9, cy=0.9, w=0.5, h=0.6)]))
        self.assertEqual(kf.track.gated_out_count, gated_before + 1)
        self.assertAlmostEqual(kf.track.x[0], cx_before, delta=0.02)

    def test_fresh_empty_streak_drops_track(self):
        cfg = KFConfig(empty_misses_to_drop=3,
                       max_prediction_horizon_s=10.0)
        kf = T2FixedKalman(cfg)
        kf.on_message(DetectionEvent(t=0.0, detections=[det()]))
        for i in range(3):
            kf.on_message(DetectionEvent(t=0.1 + i * 0.1, detections=[]))
        out = kf.tick(0.5)
        self.assertEqual(out.obstacles, [])
        self.assertEqual(kf.last_drop_reason, "fresh_empty_streak")

    def test_nis_reported_and_reasonable(self):
        kf = T2FixedKalman()
        rng = np.random.default_rng(0)
        nis_vals = []
        for i in range(200):
            t = i * DT
            kf.on_message(DetectionEvent(t=t, detections=[det(
                cx=0.5 + rng.normal(0, kf.cfg.r_center),
                cy=0.5 + rng.normal(0, kf.cfg.r_center))]))
            out = kf.tick(t)
            if out.obstacles and out.obstacles[0].debug.get("nis") is not None:
                nis_vals.append(out.obstacles[0].debug["nis"])
        mean_nis = sum(nis_vals[20:]) / len(nis_vals[20:])
        # 4-dof chi-square mean = 4; allow slack for transient + size axes.
        self.assertLess(mean_nis, 8.0)
        self.assertGreater(mean_nis, 0.5)


class TestT3(unittest.TestCase):
    def test_default_uncalibrated_equals_t2(self):
        """theta=0 must make T3 numerically identical to T2."""
        t2 = T2FixedKalman()
        t3 = T3AdaptiveKalman()
        self.assertFalse(t3.noise_model.calibrated)
        for i in range(90):
            t = i * DT
            ev1 = DetectionEvent(t=t, detections=[det(cx=0.3 + 0.002 * i)])
            ev2 = DetectionEvent(t=t, detections=[det(cx=0.3 + 0.002 * i)])
            t2.on_message(ev1)
            t3.on_message(ev2)
            o2 = t2.tick(t)
            o3 = t3.tick(t)
        self.assertAlmostEqual(o2.obstacles[0].cx, o3.obstacles[0].cx,
                               places=12)
        self.assertAlmostEqual(o2.obstacles[0].w, o3.obstacles[0].w,
                               places=12)

    def test_adaptive_scaling_direction(self):
        """A positive theta on (1-conf) must inflate R for low confidence."""
        nm = AdaptiveNoiseModel(theta_center=[2.0, 0.0, 0.0, 0.0],
                                theta_logsize=[0.0] * 4, calibrated=True,
                                source="unit-test")
        t3 = T3AdaptiveKalman(noise_model=nm)
        ev_hi = DetectionEvent(t=0.0, detections=[det(conf=0.95)])
        R_hi = t3.measurement_noise(ev_hi.detections[0], ev_hi)
        ev_lo = DetectionEvent(t=0.0, detections=[det(conf=0.30)])
        R_lo = t3.measurement_noise(ev_lo.detections[0], ev_lo)
        self.assertGreater(R_lo[0, 0], R_hi[0, 0])

    def test_features_extracted(self):
        f = detection_features(det(conf=0.8, w=0.1, h=0.1),
                               DetectionEvent(t=1.0, detections=[]), 0.9)
        self.assertAlmostEqual(f["one_minus_conf"], 0.2)
        self.assertAlmostEqual(f["frame_interval"], 0.1)
        self.assertGreater(f["inv_sqrt_area"], 1.0)


class TestFactory(unittest.TestCase):
    def test_all_methods_constructible(self):
        for name, cls in (("t0", T0Raw), ("t1", T1HoldEMA),
                          ("t2", T2FixedKalman), ("t3", T3AdaptiveKalman)):
            self.assertIsInstance(make_estimator(name), cls)

    def test_unknown_method_rejected(self):
        with self.assertRaises(ValueError):
            make_estimator("deepsort")


if __name__ == "__main__":
    unittest.main()
