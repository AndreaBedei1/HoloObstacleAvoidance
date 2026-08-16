"""The offline recovery must reproduce the planner's own range, exactly.

scripts/analysis/reconstruct_real_perception.py reimplements the
monocular range estimator rather than importing it, so that the recovery
runs without a built ROS workspace. That convenience is only safe if the
two implementations agree, and they must agree on the constants as well
as the formula: the recovery reports the range the planner BELIEVED, and
a recovery that used a different field of view would describe a system
that did not run.
"""

import importlib.util
import math
import os

import pytest

from rov_obstacle_avoidance.planner import estimate_range

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "..")
_RECON = os.path.join(_ROOT, "scripts", "analysis",
                      "reconstruct_real_perception.py")


def _load():
    spec = importlib.util.spec_from_file_location("recon", _RECON)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Obs:
    def __init__(self, height):
        self.height = height


@pytest.mark.skipif(not os.path.exists(_RECON), reason="recovery absent")
def test_recovered_range_matches_the_planner():
    recon = _load()
    vfov = math.radians(recon.VFOV_DEG)
    for h in (0.02, 0.05, 0.1, 0.15, 0.3, 0.5, 0.75, 0.9, 1.0):
        mine = recon.estimate_range_m(h)
        theirs = estimate_range(_Obs(h), vfov, recon.TARGET_HEIGHT_M,
                                recon.MAX_RANGE_M)
        assert mine == pytest.approx(theirs, abs=1e-9), h


@pytest.mark.skipif(not os.path.exists(_RECON), reason="recovery absent")
def test_uses_the_field_of_view_the_planner_actually_ran():
    """90 degrees, the planner's default, and not the 60 the benchmark
    file records but no node reads."""
    recon = _load()
    assert recon.VFOV_DEG == 90.0
    assert recon.TARGET_HEIGHT_M == 0.5


@pytest.mark.skipif(not os.path.exists(_RECON), reason="recovery absent")
def test_detector_size_gates_bound_the_observable_range():
    """The window the paper reports, recomputed rather than quoted.

    scripts/real/anchor_detect.py accepts a blob only when its height is
    between 0.15 and 0.90 of the frame, so the detector cannot report a
    range outside the interval those gates imply -- which is the finding
    the deployment section rests on.
    """
    recon = _load()
    near = recon.estimate_range_m(0.90)
    far = recon.estimate_range_m(0.15)
    assert near == pytest.approx(0.293, abs=0.005)
    assert far == pytest.approx(2.112, abs=0.005)
    # The real vehicle started at 1.86 m: inside the window, and with the
    # 1.8 m engagement range only 6 cm away from it.
    assert near < 1.86 < far
