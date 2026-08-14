"""Exhaustive fail-closed and sign tests for the real command mapping.

These tests encode the two failure modes that would be invisible at
runtime and dangerous in water:

  * a non-finite Twist component turning into a full-scale command
    (Python's min(limit, nan) returns limit, so a naive clamp yields
    FULL FORWARD on the real vehicle and FULL REVERSE in the simulator);
  * a sign convention mismatch between the ROS body frame (+y port,
    +yaw CCW) and this vehicle's MANUAL_CONTROL frame (+y starboard,
    +r CW), which would send the vehicle the wrong way around an
    obstacle.
"""

import math
import unittest

from rov_real_bridge.command_mapping import (
    MC_MAX,
    MC_MIN,
    Z_NEUTRAL,
    AxisCalibration,
    CommandMapping,
    twist_to_manual_control,
)


def calibrated(k=1.0e-3, db=0.0, min_cmd=0.0, max_cmd=1.0):
    return AxisCalibration(k_pos=k, k_neg=k, db_pos=db, db_neg=db,
                           min_command=min_cmd, max_command=max_cmd,
                           calibrated=True, source="test")


def mapping(**kw):
    return CommandMapping(surge=calibrated(**kw), sway=calibrated(**kw),
                          yaw=calibrated(**kw), calibration_id="test")


NON_FINITE = (float("nan"), float("inf"), float("-inf"))


class TestFailClosed(unittest.TestCase):
    """Every axis, every non-finite value, must yield the neutral command."""

    def test_every_axis_every_non_finite_value(self):
        m = mapping()
        for bad in NON_FINITE:
            for axis in range(3):
                args = [0.1, 0.1, 0.1]
                args[axis] = bad
                out = twist_to_manual_control(*args, mapping=m)
                self.assertFalse(out["accepted"], f"{bad} on axis {axis}")
                self.assertEqual((out["x"], out["y"], out["r"]), (0, 0, 0))
                self.assertEqual(out["z"], Z_NEUTRAL)
                self.assertIn("non-finite", out["reason"])

    def test_all_axes_non_finite_at_once(self):
        m = mapping()
        out = twist_to_manual_control(float("nan"), float("inf"),
                                      float("-inf"), mapping=m)
        self.assertFalse(out["accepted"])
        self.assertEqual((out["x"], out["y"], out["r"]), (0, 0, 0))

    def test_naive_clamp_would_have_passed_nan(self):
        """Documents the bug this guard exists for."""
        limit = 0.3
        naive = max(-limit, min(limit, float("nan")))
        self.assertEqual(naive, limit)          # full FORWARD on hardware
        sim_style = max(-limit, min(float("nan"), limit))
        self.assertTrue(math.isnan(sim_style) or sim_style == -limit)

    def test_non_numeric_rejected(self):
        m = mapping()
        out = twist_to_manual_control("0.2", 0.0, 0.0, mapping=m)
        self.assertFalse(out["accepted"])
        self.assertEqual(out["x"], 0)


class TestSignConversion(unittest.TestCase):
    """ROS frame -> this vehicle's MANUAL_CONTROL frame."""

    def test_positive_surge_is_forward(self):
        out = twist_to_manual_control(0.2, 0.0, 0.0, mapping=mapping())
        self.assertGreater(out["x"], 0)

    def test_negative_surge_is_backward(self):
        out = twist_to_manual_control(-0.2, 0.0, 0.0, mapping=mapping())
        self.assertLess(out["x"], 0)

    def test_ros_positive_sway_is_port_so_mc_y_negative(self):
        # ROS +y = LEFT/PORT; MANUAL_CONTROL +y = RIGHT/STARBOARD.
        out = twist_to_manual_control(0.0, 0.2, 0.0, mapping=mapping())
        self.assertLess(out["y"], 0, "ROS +sway (port) must give MC y < 0")

    def test_ros_negative_sway_is_starboard_so_mc_y_positive(self):
        out = twist_to_manual_control(0.0, -0.2, 0.0, mapping=mapping())
        self.assertGreater(out["y"], 0)

    def test_ros_positive_yaw_is_ccw_so_mc_r_negative(self):
        # ROS +yaw = CCW (to port); MANUAL_CONTROL +r = CW.
        out = twist_to_manual_control(0.0, 0.0, 0.2, mapping=mapping())
        self.assertLess(out["r"], 0, "ROS +yaw (CCW) must give MC r < 0")

    def test_ros_negative_yaw_is_cw_so_mc_r_positive(self):
        out = twist_to_manual_control(0.0, 0.0, -0.2, mapping=mapping())
        self.assertGreater(out["r"], 0)

    def test_heave_is_never_commanded(self):
        for v in (-0.5, 0.0, 0.5):
            out = twist_to_manual_control(v, v, v, mapping=mapping())
            self.assertEqual(out["z"], Z_NEUTRAL)


class TestDeadbandAndSaturation(unittest.TestCase):
    def test_below_min_command_is_exactly_zero(self):
        m = mapping(min_cmd=0.10)
        out = twist_to_manual_control(0.05, 0.0, 0.0, mapping=m)
        self.assertEqual(out["x"], 0)
        self.assertIn("surge", out["deadband"])

    def test_at_min_command_is_nonzero(self):
        m = mapping(min_cmd=0.10)
        out = twist_to_manual_control(0.10, 0.0, 0.0, mapping=m)
        self.assertNotEqual(out["x"], 0)
        self.assertNotIn("surge", out["deadband"])

    def test_deadband_offset_is_added(self):
        m = mapping(k=1.0e-3, db=150.0, min_cmd=0.0)
        out = twist_to_manual_control(0.05, 0.0, 0.0, mapping=m)
        self.assertEqual(out["x"], 200)      # 150 + 0.05/1e-3

    def test_counts_saturate_at_the_interface_limits(self):
        m = mapping(k=1.0e-4, max_cmd=10.0)
        out = twist_to_manual_control(5.0, -5.0, 5.0, mapping=m)
        self.assertEqual(out["x"], MC_MAX)
        self.assertEqual(out["y"], MC_MAX)     # -5 ROS sway -> +MC
        self.assertEqual(out["r"], MC_MIN)     # +5 ROS yaw  -> -MC
        self.assertEqual(set(out["saturated"]), {"surge", "sway", "yaw"})

    def test_max_command_clamps_before_conversion(self):
        m = CommandMapping(surge=calibrated(max_cmd=0.3),
                           sway=calibrated(), yaw=calibrated(),
                           calibration_id="test")
        a = twist_to_manual_control(0.3, 0, 0, mapping=m)["x"]
        b = twist_to_manual_control(9.9, 0, 0, mapping=m)["x"]
        self.assertEqual(a, b)


class TestSignedAsymmetry(unittest.TestCase):
    """The measured yaw response is ~2x stronger one way than the other,
    so the mapping must support different gains per sign."""

    def test_asymmetric_gains_give_asymmetric_counts(self):
        yaw = AxisCalibration(k_pos=1.045e-3, k_neg=2.236e-3,
                              calibrated=True, source="measured 2026-08-14")
        m = CommandMapping(surge=calibrated(), sway=calibrated(), yaw=yaw,
                           calibration_id="test")
        cw = twist_to_manual_control(0, 0, -0.2, mapping=m)["r"]
        ccw = twist_to_manual_control(0, 0, 0.2, mapping=m)["r"]
        self.assertNotEqual(abs(cw), abs(ccw))


class TestCalibrationGate(unittest.TestCase):
    def test_default_mapping_is_not_calibrated(self):
        m = CommandMapping()
        self.assertFalse(m.fully_calibrated)
        self.assertEqual(set(m.uncalibrated_axes()),
                         {"surge", "sway", "yaw"})

    def test_partial_calibration_is_reported(self):
        m = CommandMapping(surge=calibrated(), sway=AxisCalibration(),
                           yaw=calibrated())
        self.assertFalse(m.fully_calibrated)
        self.assertEqual(m.uncalibrated_axes(), ["sway"])


if __name__ == "__main__":
    unittest.main()
