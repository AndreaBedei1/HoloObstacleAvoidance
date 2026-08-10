"""Unit tests for the fail-closed real-actuation interlock.

Pure Python (no rclpy). These tests ARE the safety contract: if any of them
fails, real actuation semantics changed and must be re-reviewed.
"""

from __future__ import annotations

import itertools
import unittest

from rov_real_bridge.safety_interlock import (
    LIVE,
    SHADOW,
    SafetyInterlock,
    forbid_ground_truth_topics,
)


class TestDefaultsAreSafe(unittest.TestCase):
    def test_default_construction_blocks(self):
        lock = SafetyInterlock()
        self.assertFalse(lock.evaluate().may_actuate)

    def test_default_params_block(self):
        lock = SafetyInterlock.from_params({})
        self.assertFalse(lock.evaluate().may_actuate)
        self.assertEqual(lock.real_control_mode, SHADOW)

    def test_normal_launch_cannot_actuate(self):
        """Simulates 'someone just launches the stack': nothing set."""
        lock = SafetyInterlock.from_params(
            {"some_unrelated_param": True})
        decision = lock.gate("surge=0.4")
        self.assertFalse(decision.may_actuate)
        self.assertEqual(lock.blocked_count, 1)
        self.assertEqual(lock.permitted_count, 0)


class TestAllFlagCombinations(unittest.TestCase):
    def test_only_full_combination_permits(self):
        for in_water, allow, mode in itertools.product(
                [False, True], [False, True], [SHADOW, LIVE, "weird", ""]):
            lock = SafetyInterlock.from_params({
                "vehicle_in_water": in_water,
                "allow_real_actuation": allow,
                "real_control_mode": mode,
            })
            expected = in_water and allow and mode == LIVE
            self.assertEqual(
                lock.evaluate().may_actuate, expected,
                f"in_water={in_water} allow={allow} mode={mode!r}")


class TestFailClosedParsing(unittest.TestCase):
    def test_string_flags_must_be_exactly_true(self):
        for bad in ("True ", "yes", "1", "TRUEISH", 1, 1.0, [True], None):
            lock = SafetyInterlock.from_params({
                "vehicle_in_water": bad,
                "allow_real_actuation": bad,
                "real_control_mode": LIVE,
            })
            if isinstance(bad, str) and bad.strip().lower() == "true":
                continue
            self.assertFalse(lock.evaluate().may_actuate, f"value={bad!r}")

    def test_string_true_accepted_case_insensitive(self):
        lock = SafetyInterlock.from_params({
            "vehicle_in_water": "TRUE",
            "allow_real_actuation": "true",
            "real_control_mode": "LIVE",
        })
        self.assertTrue(lock.evaluate().may_actuate)

    def test_unknown_mode_degrades_to_shadow(self):
        lock = SafetyInterlock.from_params({
            "vehicle_in_water": True,
            "allow_real_actuation": True,
            "real_control_mode": "live_now_please",
        })
        self.assertEqual(lock.real_control_mode, SHADOW)
        self.assertFalse(lock.evaluate().may_actuate)


class TestShadowSemantics(unittest.TestCase):
    def test_shadow_blocks_even_with_both_flags(self):
        lock = SafetyInterlock(vehicle_in_water=True,
                               allow_real_actuation=True,
                               real_control_mode=SHADOW)
        self.assertFalse(lock.evaluate().may_actuate)

    def test_gate_audits_both_outcomes(self):
        lock = SafetyInterlock(vehicle_in_water=True,
                               allow_real_actuation=True,
                               real_control_mode=LIVE)
        self.assertTrue(lock.gate("cmd A").may_actuate)
        lock.real_control_mode = SHADOW
        self.assertFalse(lock.gate("cmd B").may_actuate)
        tail = lock.audit_tail
        self.assertTrue(tail[0].startswith("PERMIT"))
        self.assertTrue(tail[1].startswith("BLOCK"))


class TestGroundTruthGuard(unittest.TestCase):
    def test_forbids_ground_truth(self):
        err = forbid_ground_truth_topics(
            ["/planner/cmd_vel_safe", "/ground_truth/external/pose"])
        self.assertIsNotNone(err)

    def test_allows_normal_topics(self):
        self.assertIsNone(forbid_ground_truth_topics(
            ["/planner/cmd_vel_safe", "/perception/obstacles"]))


class TestAdapterHasNoTransmitImplementation(unittest.TestCase):
    def test_transmit_layer_is_absent(self):
        """The node source must raise NotImplementedError in _transmit and
        must not import any MAVLink library."""
        import pathlib
        src = (pathlib.Path(__file__).resolve().parents[1]
               / "rov_real_bridge" / "real_control_adapter_node.py").read_text()
        self.assertIn("NotImplementedError", src)
        self.assertNotIn("pymavlink", src.lower())
        self.assertNotIn("mavutil", src.lower())


if __name__ == "__main__":
    unittest.main()
