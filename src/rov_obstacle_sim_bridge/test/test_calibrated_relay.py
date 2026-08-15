"""The S2 burst process must not change HOW MANY observations survive.

S1 owns the marginal detection probability; S2 owns only the correlation
between misses. If the two-state gate had a marginal of its own, the
stream would be thinned twice and every S2 and S3 result would carry a
perception penalty twice as large as anything that was measured -- and
it would look like a finding, not a bug. These tests simulate the gate
and check the realised marginal against the requested one.
"""

import os
import random
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..",
    "rov_obstacle_sim_bridge"))

# import the pure math without pulling in rclpy
import importlib.util

_spec = importlib.util.spec_from_file_location(
    "_relay_math", os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..",
        "rov_obstacle_sim_bridge",
        "calibrated_observation_relay_node.py"))


def _burst_params():
    """Load burst_params without executing the ROS imports."""
    src = open(_spec.origin).read()
    start = src.index("def burst_params")
    end = src.index("class CalibratedObservationRelay")
    ns = {}
    exec(compile(src[start:end], "burst_params", "exec"), ns)
    return ns["burst_params"]


burst_params = _burst_params()

# measured on the real vehicle (config/calibration/s2_timing_fit.json)
MEAS_VISIBLE = 3.29
MEAS_BLIND = 9.16


def realised_marginal(p_target, n=200000, seed=7):
    mv, mb, _ = burst_params(MEAS_VISIBLE, MEAS_BLIND, p_target)
    rng = random.Random(seed)
    blind = False
    seen = 0
    for _ in range(n):
        if blind:
            if rng.random() < 1.0 / mb:
                blind = False
        else:
            if rng.random() < 1.0 / mv:
                blind = True
        if not blind:
            seen += 1
    return seen / float(n)


def test_marginal_is_preserved():
    """The gate reproduces the requested marginal, so S2 cannot thin the
    stream a second time on top of S1."""
    for p in (0.15, 0.3, 0.5, 0.67, 0.94):
        got = realised_marginal(p)
        assert abs(got - p) < 0.03, (p, got)


def test_runs_are_longer_than_independent():
    """S2 must actually change WHEN the misses fall: at the same
    marginal, the blind runs have to be longer than independent dropout,
    otherwise the level adds nothing and the simpler model is honest."""
    p = 0.5
    mv, mb, burstiness = burst_params(MEAS_VISIBLE, MEAS_BLIND, p)
    independent_blind_run = 1.0 / p
    assert mb > 1.5 * independent_blind_run, (mb, independent_blind_run)
    assert burstiness > 1.0


def test_identity_at_full_visibility():
    """With p close to 1 the gate must not blind the stream."""
    assert realised_marginal(0.99) > 0.95


def test_measured_values_round_trip():
    """Feeding the gate its OWN measured marginal must give back the
    measured run lengths, or the parameterisation is inconsistent with
    the fit it came from."""
    p_meas = MEAS_VISIBLE / (MEAS_VISIBLE + MEAS_BLIND)
    mv, mb, _ = burst_params(MEAS_VISIBLE, MEAS_BLIND, p_meas)
    assert abs(mv - MEAS_VISIBLE) < 0.05, (mv, MEAS_VISIBLE)
    assert abs(mb - MEAS_BLIND) < 0.05, (mb, MEAS_BLIND)
