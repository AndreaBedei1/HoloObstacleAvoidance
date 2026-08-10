"""Unit tests for the BlueROV2 dynamics module (allocation + controller).

Pure Python/numpy: no holoocean, no ROS 2, no engine required.
"""

from __future__ import annotations

import importlib.util
import math
import sys
import unittest
from pathlib import Path

import numpy as np

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SERVER_DIR = PACKAGE_DIR / "holoocean_server"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


dyn = _load_module("bluerov2_dynamics_test", SERVER_DIR / "bluerov2_dynamics.py")


class TestAllocationMatrix(unittest.TestCase):
    def test_full_rank_for_controlled_dofs(self):
        """The 8-thruster layout must span Fx, Fy, Fz, Mx, My, Mz."""
        B = dyn.build_allocation_matrix()
        self.assertEqual(B.shape, (6, 8))
        self.assertEqual(np.linalg.matrix_rank(B, tol=1e-9), 6)

    def test_wrench_reproduction(self):
        """pinv allocation must reproduce any feasible wrench exactly."""
        alloc = dyn.ThrusterAllocator(max_thrust=1e6)  # no saturation
        rng = np.random.default_rng(42)
        for _ in range(20):
            wrench = rng.uniform(-20, 20, size=6)
            forces = alloc.allocate(wrench)
            achieved = alloc.B @ forces
            np.testing.assert_allclose(achieved, wrench, atol=1e-9)

    def test_pure_surge_uses_equal_angled_thrusters(self):
        alloc = dyn.ThrusterAllocator(max_thrust=1e6)
        forces = alloc.allocate(np.array([10.0, 0, 0, 0, 0, 0]))
        # Engine geometry: the angled pods are at z=0 relative to the COM, so
        # pure surge is exactly equal positive force on all four angled
        # thrusters and zero on the verticals.
        np.testing.assert_allclose(forces[:4], 0.0, atol=1e-9)
        np.testing.assert_allclose(forces[4:], forces[4], atol=1e-9)
        self.assertGreater(forces[4], 0.0)
        np.testing.assert_allclose(
            alloc.B @ forces, [10.0, 0, 0, 0, 0, 0], atol=1e-9)

    def test_sway_left_sign_convention(self):
        """+Fy (leftward, ROS convention) loads t4/t6 positive, t5/t7 negative."""
        alloc = dyn.ThrusterAllocator(max_thrust=1e6)
        forces = alloc.allocate(np.array([0, 10.0, 0, 0, 0, 0]))
        self.assertGreater(forces[4], 0.0)
        self.assertLess(forces[5], 0.0)
        self.assertGreater(forces[6], 0.0)
        self.assertLess(forces[7], 0.0)

    def test_yaw_ccw_sign_convention(self):
        """+Mz (CCW) loads t4/t7 positive, t5/t6 negative (verified vs engine)."""
        alloc = dyn.ThrusterAllocator(max_thrust=1e6)
        forces = alloc.allocate(np.array([0, 0, 0, 0, 0, 2.0]))
        self.assertGreater(forces[4], 0.0)
        self.assertLess(forces[5], 0.0)
        self.assertLess(forces[6], 0.0)
        self.assertGreater(forces[7], 0.0)

    def test_pure_heave_uses_only_vertical_thrusters(self):
        alloc = dyn.ThrusterAllocator(max_thrust=1e6)
        forces = alloc.allocate(np.array([0, 0, 10.0, 0, 0, 0]))
        np.testing.assert_allclose(forces[4:], 0.0, atol=1e-9)
        np.testing.assert_allclose(forces[:4], 2.5, atol=1e-9)

    def test_saturation_preserves_direction(self):
        alloc = dyn.ThrusterAllocator(max_thrust=5.0)
        wrench = np.array([1000.0, 0, 0, 0, 0, 0])
        forces = alloc.allocate(wrench)
        self.assertLessEqual(np.max(np.abs(forces)), 5.0 + 1e-9)
        achieved = alloc.B @ forces
        # Direction preserved: achieved wrench parallel to requested.
        self.assertGreater(achieved[0], 0.0)
        np.testing.assert_allclose(achieved[1:], 0.0, atol=1e-9)


class TestLatencyBuffer(unittest.TestCase):
    def test_zero_delay_passthrough(self):
        buf = dyn.CommandLatencyBuffer(0.0, dt=1 / 30)
        out = buf.push({"surge": 0.4})
        self.assertAlmostEqual(out["surge"], 0.4)

    def test_delay_steps(self):
        dt = 1 / 30
        buf = dyn.CommandLatencyBuffer(3 * dt, dt=dt)
        outs = [buf.push({"surge": 1.0})["surge"] for _ in range(5)]
        # First 3 pushes yield zeros (nothing old enough), then the value.
        self.assertEqual(outs[:3], [0.0, 0.0, 0.0])
        self.assertEqual(outs[3:], [1.0, 1.0])


class _FirstOrderPlant:
    """Toy 4-DOF plant: m*dv = F - d*v (per axis), I*dw = M - dr*w."""

    def __init__(self, dt):
        self.dt = dt
        self.v = np.zeros(3)
        self.w = 0.0
        self.mass = 11.5
        self.drag = 8.0
        self.inertia = 0.3
        self.ang_drag = 0.6

    def apply(self, wrench):
        F = wrench[:3]
        Mz = wrench[5]
        self.v += (F - self.drag * self.v) / self.mass * self.dt
        self.w += (Mz - self.ang_drag * self.w) / self.inertia * self.dt


class TestBodyVelocityController(unittest.TestCase):
    def _run(self, target, seconds=8.0):
        dt = 1 / 30
        cfg = dyn.DynamicsConfig()
        ctl = dyn.BodyVelocityController(cfg, dt)
        plant = _FirstOrderPlant(dt)
        for _ in range(int(seconds / dt)):
            forces = ctl.update(target, plant.v, np.array([0, 0, plant.w]),
                                roll=0.0, pitch=0.0)
            wrench = ctl.allocator.B @ forces
            plant.apply(wrench)
        return plant

    def test_surge_step_converges(self):
        plant = self._run(dict(surge=0.4, sway=0.0, heave=0.0, yaw_rate=0.0))
        self.assertAlmostEqual(plant.v[0], 0.4, delta=0.02)
        self.assertAlmostEqual(plant.v[1], 0.0, delta=0.01)

    def test_sway_step_converges(self):
        plant = self._run(dict(surge=0.0, sway=0.25, heave=0.0, yaw_rate=0.0))
        self.assertAlmostEqual(plant.v[1], 0.25, delta=0.02)

    def test_yaw_rate_step_converges(self):
        plant = self._run(dict(surge=0.0, sway=0.0, heave=0.0, yaw_rate=0.3))
        self.assertAlmostEqual(plant.w, 0.3, delta=0.02)

    def test_setpoint_rate_limited(self):
        dt = 1 / 30
        cfg = dyn.DynamicsConfig(max_lin_accel=0.5)
        ctl = dyn.BodyVelocityController(cfg, dt)
        ctl.update(dict(surge=1.0, sway=0.0, heave=0.0, yaw_rate=0.0),
                   np.zeros(3), np.zeros(3), 0.0, 0.0)
        # After one tick the internal setpoint moved at most max_lin_accel*dt.
        self.assertLessEqual(ctl.last_setpoint["surge"], 0.5 * dt + 1e-9)

    def test_attitude_stabilization_sign(self):
        dt = 1 / 30
        ctl = dyn.BodyVelocityController(dyn.DynamicsConfig(), dt)
        target = dict(surge=0.0, sway=0.0, heave=0.0, yaw_rate=0.0)
        ctl.update(target, np.zeros(3), np.zeros(3), roll=0.2, pitch=0.0)
        # Positive roll must produce negative Mx (restoring torque).
        self.assertLess(ctl.last_wrench[3], 0.0)
        ctl2 = dyn.BodyVelocityController(dyn.DynamicsConfig(), dt)
        ctl2.update(target, np.zeros(3), np.zeros(3), roll=0.0, pitch=0.15)
        self.assertLess(ctl2.last_wrench[4], 0.0)


class TestConfigLoading(unittest.TestCase):
    def test_defaults(self):
        cfg = dyn.load_dynamics_config(None)
        self.assertEqual(cfg.command_latency_s, 0.0)

    def test_override(self):
        cfg = dyn.load_dynamics_config(
            {"command_latency_s": 0.1, "kp_lin": [10, 11, 12]}
        )
        self.assertAlmostEqual(cfg.command_latency_s, 0.1)
        self.assertEqual(cfg.kp_lin, (10.0, 11.0, 12.0))

    def test_unknown_key_rejected(self):
        with self.assertRaises(ValueError):
            dyn.load_dynamics_config({"tipo": 1})


class TestFrameHelpers(unittest.TestCase):
    def test_rotation_to_roll_pitch_identity(self):
        roll, pitch = dyn.rotation_to_roll_pitch(np.eye(3))
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)

    def test_world_to_body_yaw_90(self):
        yaw = math.pi / 2
        R = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ])
        v_body = dyn.world_to_body(np.array([0.0, 1.0, 0.0]), R)
        np.testing.assert_allclose(v_body, [1.0, 0.0, 0.0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
