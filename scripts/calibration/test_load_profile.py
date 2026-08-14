"""Unit tests for the S0-S3 calibration profile loader.

Two groups, for two different failure risks:

  * REAL-FILE tests pin the properties the committed ladder must keep
    (S0 immutable and fully sourced, the measured values that ARE known,
    the ones that are deliberately null).  They fail if someone edits a
    profile in a way that breaks the ladder.
  * SYNTHETIC tests build deliberately broken chains in a temp directory to
    prove the validator actually rejects them - a validator that never says
    no is worse than none, because it looks like a guarantee.

Run::

    python scripts/calibration/test_load_profile.py
    python -m pytest scripts/calibration/test_load_profile.py
"""

from __future__ import annotations

import os
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from load_profile import (  # noqa: E402
    DEFAULT_CONFIG_DIR,
    ProfileError,
    _validate_no_double_override,
    deep_merge,
    flatten,
    load_profile,
    resolve,
)


# ---------------------------------------------------------------------------
# synthetic chain builder
# ---------------------------------------------------------------------------

def _s0(values=None, provenance=None):
    return {
        "profile": {"id": "S0", "name": "s0_historical", "parent": None,
                    "status": "IMMUTABLE", "base_level": True,
                    "owns": ["simulator"]},
        "values": values if values is not None else {
            "simulator": {"ticks_per_sec": 30},
            "camera": {"horizontal_fov_deg": 90.0},
            "observation": {"detection_probability": 1.0},
            "timing": {"observation_rate_hz": 30.0},
            "vehicle": {"command_latency_s": 0.0},
        },
        "provenance": provenance if provenance is not None else {
            "simulator": "test", "camera": "test", "observation": "test",
            "timing": "test", "vehicle": "test",
        },
        "todo": {},
    }


def _child(level_id, parent, owns, overrides, provenance=None, todo=None):
    return {
        "profile": {"id": level_id, "name": level_id.lower(),
                    "parent": parent, "status": "TEST", "owns": owns},
        "overrides": overrides,
        "provenance": provenance or {},
        "todo": todo or {},
    }


def _write(directory, docs):
    names = ["s0_historical.yaml", "s1_observation.yaml", "s2_timing.yaml",
             "s3_vehicle.yaml"]
    for name, doc in zip(names, docs):
        with open(os.path.join(directory, name), "w",
                  encoding="utf-8") as handle:
            yaml.safe_dump(doc, handle, sort_keys=False)


class SyntheticChainTest(unittest.TestCase):
    """The validator must reject every way of breaking the ladder."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def test_valid_chain_merges(self):
        _write(self.dir, [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera", "observation"],
                   {"camera": {"horizontal_fov_deg": 74.0}},
                   provenance={"camera.horizontal_fov_deg": "measured"}),
            _child("S2", "s1_observation.yaml", ["timing"],
                   {"timing": {"observation_rate_hz": None}},
                   todo={"timing.observation_rate_hz": "measure it"}),
            _child("S3", "s2_timing.yaml", ["vehicle"],
                   {"vehicle": {"command_latency_s": 0.07}},
                   provenance={"vehicle.command_latency_s": "measured"}),
        ])
        prof = load_profile("S3", self.dir)
        self.assertEqual(prof.values["camera"]["horizontal_fov_deg"], 74.0)
        self.assertEqual(prof.values["vehicle"]["command_latency_s"], 0.07)
        # S0 values survive where nothing overrode them.
        self.assertEqual(prof.values["simulator"]["ticks_per_sec"], 30)
        self.assertEqual(prof.unmeasured, ["timing.observation_rate_hz"])

    def test_same_key_at_two_levels_is_rejected(self):
        # S2 and S3 both claim `timing` -> caught as an ownership clash.
        _write(self.dir, [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera", "observation"], {}),
            _child("S2", "s1_observation.yaml", ["timing"],
                   {"timing": {"observation_rate_hz": 10.0}},
                   provenance={"timing.observation_rate_hz": "m"}),
            _child("S3", "s2_timing.yaml", ["vehicle", "timing"],
                   {"timing": {"observation_rate_hz": 12.0}},
                   provenance={"timing.observation_rate_hz": "m"}),
        ])
        with self.assertRaises(ProfileError) as ctx:
            load_profile("S3", self.dir)
        self.assertIn("owned by both", str(ctx.exception))

    def test_override_outside_owned_section_is_rejected(self):
        _write(self.dir, [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera", "observation"],
                   {"vehicle": {"command_latency_s": 0.07}},
                   provenance={"vehicle.command_latency_s": "m"}),
            _child("S2", "s1_observation.yaml", ["timing"], {}),
            _child("S3", "s2_timing.yaml", ["vehicle"], {}),
        ])
        with self.assertRaises(ProfileError) as ctx:
            load_profile("S3", self.dir)
        self.assertIn("owned by S3", str(ctx.exception))

    def test_double_override_of_one_leaf_is_rejected(self):
        """The per-leaf check, exercised directly.

        With today's section-level ownership the ownership clash fires
        first, so the leaf check is unreachable through a file chain. It is
        kept as defence in depth for any future finer-grained ownership,
        and therefore tested on its own.
        """
        levels = [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera", "timing"],
                   {"timing": {"shared": 2.0}}),
            _child("S2", "s1_observation.yaml", ["observation", "timing"],
                   {"timing": {"shared": 3.0}}),
        ]
        with self.assertRaises(ProfileError) as ctx:
            _validate_no_double_override(levels)
        self.assertIn("exactly one level", str(ctx.exception))

    def test_null_without_todo_is_rejected(self):
        _write(self.dir, [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera", "observation"],
                   {"camera": {"horizontal_fov_deg": None}}),
            _child("S2", "s1_observation.yaml", ["timing"], {}),
            _child("S3", "s2_timing.yaml", ["vehicle"], {}),
        ])
        with self.assertRaises(ProfileError) as ctx:
            load_profile("S1", self.dir)
        self.assertIn("no todo", str(ctx.exception))

    def test_value_without_provenance_is_rejected(self):
        _write(self.dir, [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera", "observation"],
                   {"camera": {"horizontal_fov_deg": 74.0}}),
            _child("S2", "s1_observation.yaml", ["timing"], {}),
            _child("S3", "s2_timing.yaml", ["vehicle"], {}),
        ])
        with self.assertRaises(ProfileError) as ctx:
            load_profile("S1", self.dir)
        self.assertIn("no provenance", str(ctx.exception))

    def test_null_in_s0_is_rejected(self):
        base = _s0()
        base["values"]["simulator"]["ticks_per_sec"] = None
        _write(self.dir, [base,
                          _child("S1", "s0_historical.yaml", ["camera"], {}),
                          _child("S2", "s1_observation.yaml", ["timing"], {}),
                          _child("S3", "s2_timing.yaml", ["vehicle"], {})])
        with self.assertRaises(ProfileError) as ctx:
            load_profile("S0", self.dir)
        self.assertIn("cannot contain unmeasured", str(ctx.exception))

    def test_s0_leaf_without_provenance_is_rejected(self):
        base = _s0(provenance={"simulator": "test"})
        _write(self.dir, [base,
                          _child("S1", "s0_historical.yaml", ["camera"], {}),
                          _child("S2", "s1_observation.yaml", ["timing"], {}),
                          _child("S3", "s2_timing.yaml", ["vehicle"], {})])
        with self.assertRaises(ProfileError) as ctx:
            load_profile("S0", self.dir)
        self.assertIn("no provenance", str(ctx.exception))

    def test_broken_parent_link_is_rejected(self):
        _write(self.dir, [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera"], {}),
            _child("S2", "s0_historical.yaml", ["timing"], {}),
            _child("S3", "s2_timing.yaml", ["vehicle"], {}),
        ])
        with self.assertRaises(ProfileError) as ctx:
            load_profile("S2", self.dir)
        self.assertIn("parent", str(ctx.exception))

    def test_require_measured_rejects_nulls(self):
        _write(self.dir, [
            _s0(),
            _child("S1", "s0_historical.yaml", ["camera", "observation"],
                   {"camera": {"horizontal_fov_deg": None}},
                   todo={"camera.horizontal_fov_deg": "measure"}),
            _child("S2", "s1_observation.yaml", ["timing"], {}),
            _child("S3", "s2_timing.yaml", ["vehicle"], {}),
        ])
        load_profile("S1", self.dir)          # legal without the flag
        with self.assertRaises(ProfileError):
            load_profile("S1", self.dir, require_measured=True)


class HelperTest(unittest.TestCase):

    def test_flatten_treats_lists_as_leaves(self):
        flat = flatten({"a": {"b": [1, 2]}, "c": None})
        self.assertEqual(flat, {"a.b": [1, 2], "c": None})

    def test_deep_merge_null_replaces_subtree(self):
        merged = deep_merge({"a": {"b": 1, "c": 2}}, {"a": None})
        self.assertIsNone(merged["a"])

    def test_resolve_uses_longest_prefix(self):
        mapping = {"a": "broad", "a.b": "narrow"}
        self.assertEqual(resolve(mapping, "a.b.c"), "narrow")
        self.assertEqual(resolve(mapping, "a.x.y"), "broad")
        self.assertIsNone(resolve(mapping, "z"))


class CommittedLadderTest(unittest.TestCase):
    """Properties of the real config/calibration/ files."""

    def test_s0_is_complete_and_sourced(self):
        prof = load_profile("S0", DEFAULT_CONFIG_DIR, require_measured=True)
        self.assertEqual(prof.unmeasured, [])
        self.assertEqual(prof.values["simulator"]["ticks_per_sec"], 30)

    def test_full_chain_validates(self):
        prof = load_profile("S3", DEFAULT_CONFIG_DIR)
        self.assertEqual(prof.chain[0], "s0_historical.yaml")
        self.assertEqual(len(prof.chain), 4)

    def test_ownership_is_disjoint_and_covers_the_ladder(self):
        prof = load_profile("S3", DEFAULT_CONFIG_DIR)
        self.assertEqual(prof.ownership["S1"], ["camera", "observation"])
        self.assertEqual(prof.ownership["S2"], ["timing"])
        self.assertEqual(prof.ownership["S3"], ["vehicle"])
        owned = [s for sections in prof.ownership.values() for s in sections]
        self.assertEqual(len(owned), len(set(owned)))

    def test_measured_values_are_present(self):
        prof = load_profile("S3", DEFAULT_CONFIG_DIR)
        self.assertAlmostEqual(prof.values["camera"]["horizontal_fov_deg"],
                               74.0)
        self.assertAlmostEqual(
            prof.values["vehicle"]["gain_asymmetry"]["yaw_neg_over_pos"],
            2.14)
        self.assertAlmostEqual(
            prof.values["vehicle"]["response"]["surge_tau_rise_s"], 1.44)
        self.assertAlmostEqual(
            prof.values["vehicle"]["response"]["yaw_tau_rise_s"], 1.04)

    def test_unmeasured_values_stay_null_with_a_todo(self):
        prof = load_profile("S3", DEFAULT_CONFIG_DIR)
        for key in ("vehicle.response.sway_tau_rise_s",
                    "vehicle.gain_asymmetry.sway_right_over_left",
                    "vehicle.gain_asymmetry.surge_fwd_over_rev",
                    "vehicle.deadband.surge_frac",
                    "observation.detection_probability",
                    "timing.observation_latency_s"):
            self.assertIn(key, prof.unmeasured, f"{key} must stay null")
            self.assertIsNotNone(resolve(prof.todo, key),
                                 f"{key} must name its measurement")

    def test_no_level_can_be_run_as_a_frozen_condition_yet(self):
        for level in ("S1", "S2", "S3"):
            with self.assertRaises(ProfileError):
                load_profile(level, DEFAULT_CONFIG_DIR,
                             require_measured=True)

    def test_s0_sim_tick_contract_is_not_touched_above_s0(self):
        """D-010: 30 ticks/s must survive the whole ladder."""
        for level in ("S1", "S2", "S3"):
            prof = load_profile(level, DEFAULT_CONFIG_DIR)
            self.assertEqual(prof.values["simulator"]["ticks_per_sec"], 30)


if __name__ == "__main__":
    unittest.main(verbosity=2)
