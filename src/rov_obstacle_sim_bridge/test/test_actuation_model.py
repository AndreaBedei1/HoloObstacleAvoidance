"""The S3 plant must reproduce the measured vehicle and nothing else.

The property that matters most is the one an earlier version of S3 got
wrong: the shortfall between what the planner asks for and what the
vehicle delivers has to appear HERE, downstream of the frozen boundary,
never by quietly lowering the planner's own limits.
"""

import importlib.util
import math
import os

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                    "rov_obstacle_sim_bridge", "actuation_model_node.py")


def _load():
    """Load Axis without importing rclpy."""
    src = open(_SRC).read()
    start = src.index("FULL_SCALE")
    end = src.index("def axes_from_profile")
    ns = {"math": math}
    exec(compile(src[start:end], "axis", "exec"), ns)
    return ns["Axis"]


Axis = _load()

# measured (config/calibration/s3_vehicle.json)
K_SURGE = 1.45e-4        # m/s per count, symmetric
K_SWAY = 1.23e-4
K_YAW_P = math.radians(0.0654)
K_YAW_N = math.radians(0.0537)
DB_YAW_P, DB_YAW_N = 383.0, 220.0


def steady(axis, v_req, seconds=30.0, dt=0.05):
    out = 0.0
    for _ in range(int(seconds / dt)):
        out = axis.step(v_req, dt)
    return out


def test_saturation_is_the_measured_maximum():
    """A request beyond the vehicle's authority is delivered at the
    measured maximum, not at the request: 0.145 m/s surge, 0.123 sway."""
    surge = Axis(K_SURGE, K_SURGE, 0, 0, 0.5, 0.5)
    sway = Axis(K_SWAY, K_SWAY, 0, 0, 0.5, 0.5)
    assert abs(steady(surge, 0.5) - 0.145) < 0.002
    assert abs(steady(sway, 0.30) - 0.123) < 0.002


def test_request_within_authority_is_delivered():
    """Below saturation the plant must not distort the command, or S3
    would penalise the vehicle everywhere instead of at its limits."""
    sway = Axis(K_SWAY, K_SWAY, 0, 0, 0.3, 0.3)
    assert abs(steady(sway, 0.06) - 0.06) < 1e-3


def test_yaw_asymmetry_is_preserved():
    """Yaw is the one axis whose asymmetry was measured on two levels
    per direction from the IMU; the plant must carry it."""
    yaw = Axis(K_YAW_P, K_YAW_N, DB_YAW_P, DB_YAW_N, 0.5, 0.5)
    pos = steady(yaw, 5.0)
    neg = steady(yaw, -5.0)
    assert abs(pos - math.radians(40.4)) < math.radians(1.0)
    assert abs(neg + math.radians(41.9)) < math.radians(1.0)
    assert abs(pos) != abs(neg)


def test_zero_stays_zero():
    surge = Axis(K_SURGE, K_SURGE, 0, 0, 0.5, 0.5)
    assert steady(surge, 0.0) == 0.0


def test_first_order_lag_is_not_instant():
    """The vehicle reaches roughly 63 percent of its steady speed after
    one time constant; an instant response would erase the dynamics the
    rung exists to add."""
    tau = 1.0
    surge = Axis(K_SURGE, K_SURGE, 0, 0, tau, tau)
    dt, out = 0.01, 0.0
    for _ in range(int(tau / dt)):
        out = surge.step(0.10, dt)
    assert 0.55 * 0.10 < out < 0.70 * 0.10


def test_decay_uses_the_decay_constant():
    """Releasing the command must coast, not stop dead: stopping the
    thrusters does not stop the vehicle, which is why every 'stop' in
    this project is an active hold."""
    surge = Axis(K_SURGE, K_SURGE, 0, 0, 0.2, 2.0)
    steady(surge, 0.10, seconds=5.0)
    before = surge.state
    for _ in range(20):
        surge.step(0.0, 0.05)
    assert surge.state > 0.5 * before
