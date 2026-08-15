"""The commitment metric must behave the same on any command trace.

It is applied offline to simulated and to real runs by the same code, so
its edge cases decide what "the vehicle committed to a manoeuvre" means
in BOTH domains. The case that motivated the persistence requirement is
the third test: a planner that always trims laterally would otherwise
register a commitment on its first cycle, at the start distance, in
every run -- which is exactly what DWA did at a bare 0.02 m/s threshold.
"""

import importlib.util
import os

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                    "..", "scripts", "analysis", "sim_real_compare.py")
_spec = importlib.util.spec_from_file_location("src_cmp", _SRC)
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
commitment = _m.commitment
THR = _m.COMMIT_LATERAL_M_S
HOLD = _m.COMMIT_HOLD_S


def trace(samples):
    return [{"t": t, "x": 0.1, "y": y, "r": 0.0, "d": d}
            for t, y, d in samples]


def test_no_lateral_command_is_not_a_commitment():
    """Never manoeuvring is an outcome, not a zero."""
    out = commitment(trace([(i * 0.1, 0.0, 4.0 - i * 0.01)
                            for i in range(50)]))
    assert out["commit_distance_m"] is None
    assert out["maneuver_s"] is None


def test_sustained_lateral_command_commits_at_its_distance():
    s = [(i * 0.1, 0.0, 4.0 - i * 0.02) for i in range(20)]
    s += [(2.0 + i * 0.1, THR * 2, 3.6 - i * 0.02) for i in range(40)]
    out = commitment(trace(s))
    assert out["commit_distance_m"] is not None
    assert abs(out["commit_distance_m"] - 3.6) < 0.05
    assert out["maneuver_s"] >= HOLD


def test_brief_lateral_trim_is_not_a_commitment():
    """A blip shorter than the hold time must not count: without this a
    planner that trims laterally from its first cycle commits at the
    start distance in every run and the metric carries no information."""
    s = [(i * 0.1, 0.0, 4.0) for i in range(5)]
    s += [(0.5 + i * 0.1, THR * 2, 3.9) for i in range(3)]   # 0.3 s only
    s += [(0.8 + i * 0.1, 0.0, 3.8) for i in range(40)]
    out = commitment(trace(s))
    assert out["commit_distance_m"] is None


def test_peak_is_recorded_even_without_commitment():
    """The peak is evidence about what the vehicle attempted, so it
    survives a non-commitment."""
    s = [(i * 0.1, THR * 3, 4.0) for i in range(3)]
    s += [(0.3 + i * 0.1, 0.0, 3.9) for i in range(40)]
    out = commitment(trace(s))
    assert out["commit_distance_m"] is None
    assert out["lateral_peak_m_s"] > THR


def test_sign_does_not_matter():
    """Avoiding to port and to starboard are the same manoeuvre."""
    a = commitment(trace([(i * 0.1, THR * 2, 3.0) for i in range(40)]))
    b = commitment(trace([(i * 0.1, -THR * 2, 3.0) for i in range(40)]))
    assert a["maneuver_s"] == b["maneuver_s"]


def test_empty_trace_is_empty_not_a_crash():
    assert commitment([]) == {}


# --- left censoring and the common observable window -------------------

clip_to_window = _m.clip_to_window
CENSOR = _m.CENSOR_LATERAL_M_S
WINDOW = _m.COMMON_WINDOW_M


def test_already_manoeuvring_at_first_sample_is_censored():
    """The manoeuvre began at or before the window opened, so its
    distance is a lower bound and must be flagged as one. Reporting it
    as a measurement would say the vehicle committed exactly where the
    recording happened to start."""
    out = commitment(trace([(i * 0.1, THR * 2, 1.86 - i * 0.01)
                            for i in range(40)]))
    assert out["commit_censored"] is True
    assert out["maneuver_censored_start"] is True
    assert out["commit_distance_m"] is not None


def test_manoeuvre_starting_inside_the_window_is_not_censored():
    s = [(i * 0.1, 0.0, 3.5 - i * 0.02) for i in range(20)]
    s += [(2.0 + i * 0.1, THR * 2, 3.1 - i * 0.02) for i in range(40)]
    out = commitment(trace(s))
    assert out["commit_censored"] is False
    assert out["maneuver_censored_start"] is False


def test_censoring_threshold_is_lower_than_the_commit_threshold():
    """A lower bar for 'already moving' flags more runs as censored,
    which is the conservative direction."""
    assert CENSOR < THR


def test_window_drops_the_unobservable_approach_and_rebases_time():
    s = [(i * 0.1, 0.0, 3.5 - i * 0.05) for i in range(40)]
    clipped = clip_to_window(trace(s))
    assert clipped, "the run does enter the window"
    assert all(x["d"] <= WINDOW for x in clipped)
    assert clipped[0]["t"] == 0.0, "time re-based to the window opening"


def test_window_is_empty_when_the_run_never_gets_that_close():
    """At S0 and S1 the DWA planner keeps 2.1-2.5 m of clearance, so the
    simulated vehicle never comes as close as the real one STARTS. That
    is a fact about the trajectory, not missing data, and the analysis
    must be able to say so."""
    s = [(i * 0.1, 0.1, 2.4) for i in range(40)]
    assert clip_to_window(trace(s)) == []


def test_window_view_censors_a_manoeuvre_already_under_way():
    """Clipping starts the trace mid-manoeuvre, which is exactly the
    case censoring exists for."""
    s = [(i * 0.1, 0.0, 3.5 - i * 0.05) for i in range(20)]     # 3.5->2.55
    s += [(2.0 + i * 0.1, THR * 2, 2.5 - i * 0.05) for i in range(30)]
    out = commitment(clip_to_window(trace(s)))
    assert out["commit_censored"] is True


def test_samples_without_ground_truth_are_dropped():
    """A sample with no distance carries no information about which side
    of the window it falls on."""
    s = [{"t": i * 0.1, "x": 0.1, "y": 0.0, "r": 0.0, "d": None}
         for i in range(5)]
    s += [{"t": 0.5 + i * 0.1, "x": 0.1, "y": 0.0, "r": 0.0, "d": 1.5}
          for i in range(10)]
    clipped = clip_to_window(s)
    assert len(clipped) == 10
    assert all(x["d"] is not None for x in clipped)
