"""BlueROV2 Heavy body-velocity control and thruster allocation.

Pure numpy module (no holoocean import) shared by the sim server and unit
tests. It converts commanded body velocities (surge, sway, heave, yaw_rate)
into the 8 thruster forces expected by the HoloOcean ``BlueROV2`` agent in
control scheme 0 (thruster forces).

Pipeline per tick:

    cmd (body vel) -> latency buffer -> rate limiter -> PI velocity controller
        + roll/pitch PD stabilization -> body wrench -> thruster allocation
        -> per-thruster saturation -> 8 forces

The thruster geometry matches the HoloOcean BlueROV2 (= BlueROV2 Heavy)
constants, which in turn match the real vehicle inventoried with
FRAME_CONFIG=2 (vectored 6-DOF, 8 thrusters).

Thruster order (HoloOcean scheme 0):
    [Vert Front Stbd, Vert Front Port, Vert Back Port, Vert Back Stbd,
     Ang Front Stbd,  Ang Front Port,  Ang Back Port,  Ang Back Stbd]
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# Geometry from the HoloOcean ENGINE source (BlueROV2.h / BlueROV2.cpp,
# ApplyThrusters), converted to the client/ROS body frame (x fwd, y LEFT,
# z up, right-handed; UE y is flipped by ConvertLinearVector).  The Python
# agent docstring's matrices are WRONG (misordered/sign-flipped); using them
# produced zero net surge in open-loop probes.  This geometry reproduces all
# observed probe responses (experiments/simulation/frame_probe/).
#
# Engine facts: i<4 vertical, LocalForce=(0,0,f); i>=4 even: (f,+f,0)/sqrt2;
# i>=4 odd: (f,-f,0)/sqrt2 (client frame).  Locations (UE cm, mesh frame)
# minus CenterMass=(0,0,-1) [Perfect=true], y negated for the client frame.
_S2 = math.sqrt(2.0) / 2.0
BLUEROV2_THRUSTER_DIRS = np.array([
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 1.0],
    [_S2, _S2, 0.0],    # t4 front-right pod, pushes forward-left
    [_S2, -_S2, 0.0],   # t5 front-left pod, pushes forward-right
    [_S2, _S2, 0.0],    # t6 back-left pod, pushes forward-left
    [_S2, -_S2, 0.0],   # t7 back-right pod, pushes forward-right
])

# Positions of the thrusters in the client body frame [m], relative to COM.
BLUEROV2_THRUSTER_POS = np.array([
    [0.12, -0.2181, 0.0809],
    [0.12, 0.2181, 0.0809],
    [-0.12, 0.2181, 0.0809],
    [-0.12, -0.2181, 0.0809],
    [0.1562, -0.0988, 0.0],
    [0.1562, 0.0988, 0.0],
    [-0.1562, 0.0988, 0.0],
    [-0.1562, -0.0988, 0.0],
])

# Engine-side per-thruster limit (HoveringAUV.h: MAX_LIN_ACCEL*mass/4).
BLUEROV2_MAX_THRUST_N = 10.0 * 11.5 / 4.0  # 28.75


def build_allocation_matrix(
    dirs: np.ndarray = BLUEROV2_THRUSTER_DIRS,
    pos: np.ndarray = BLUEROV2_THRUSTER_POS,
) -> np.ndarray:
    """6x8 matrix B with wrench = B @ forces (Fx,Fy,Fz,Mx,My,Mz)."""
    n = dirs.shape[0]
    B = np.zeros((6, n))
    for i in range(n):
        B[0:3, i] = dirs[i]
        B[3:6, i] = np.cross(pos[i], dirs[i])
    return B


class ThrusterAllocator:
    """Least-squares wrench-to-forces allocation with direction-preserving
    saturation."""

    def __init__(self, max_thrust: float = BLUEROV2_MAX_THRUST_N) -> None:
        self.B = build_allocation_matrix()
        self.B_pinv = np.linalg.pinv(self.B)
        self.max_thrust = float(max_thrust)

    def allocate(self, wrench: np.ndarray) -> np.ndarray:
        forces = self.B_pinv @ np.asarray(wrench, dtype=float).reshape(6)
        peak = float(np.max(np.abs(forces))) if forces.size else 0.0
        if peak > self.max_thrust > 0.0:
            # Scale the whole vector so the commanded wrench direction is
            # preserved instead of clipping thrusters independently.
            forces = forces * (self.max_thrust / peak)
        return forces


@dataclass
class DynamicsConfig:
    """Configuration for the body-velocity controller (YAML `dynamics:`)."""

    # PI gains on body linear velocity error [N per m/s, N per m].
    kp_lin: Tuple[float, float, float] = (40.0, 40.0, 45.0)
    ki_lin: Tuple[float, float, float] = (12.0, 12.0, 15.0)
    # PI gains on yaw rate error [N*m per rad/s, N*m per rad].
    kp_yaw_rate: float = 8.0
    ki_yaw_rate: float = 2.0
    # PD stabilization of roll/pitch toward zero [N*m/rad, N*m/(rad/s)].
    kp_att: float = 12.0
    kd_att: float = 4.0
    # Integrator clamps (anti-windup).
    int_limit_lin: float = 20.0
    int_limit_yaw: float = 6.0
    # Setpoint rate limits (accel of the commanded velocity target).
    max_lin_accel: float = 1.0     # m/s^2 applied to the setpoint
    max_yaw_accel: float = 1.5     # rad/s^2 applied to the setpoint
    # Command latency emulation (seconds; 0 = none).
    command_latency_s: float = 0.0
    # Per-thruster force limit.
    max_thrust: float = BLUEROV2_MAX_THRUST_N


def load_dynamics_config(data: Optional[Dict[str, Any]]) -> DynamicsConfig:
    """Build a DynamicsConfig from a YAML dict (unknown keys rejected)."""
    cfg = DynamicsConfig()
    if not data:
        return cfg
    valid = set(cfg.__dataclass_fields__)
    for key, value in data.items():
        if key not in valid:
            raise ValueError(f"unknown dynamics config key: {key!r}")
        current = getattr(cfg, key)
        if isinstance(current, tuple):
            setattr(cfg, key, tuple(float(v) for v in value))
        else:
            setattr(cfg, key, float(value))
    return cfg


class CommandLatencyBuffer:
    """Fixed-delay line emulating command-path latency."""

    def __init__(self, delay_s: float, dt: float) -> None:
        self.steps = max(0, int(round(float(delay_s) / dt))) if dt > 0 else 0
        self._buf: deque = deque(maxlen=self.steps + 1)

    def push(self, cmd: Dict[str, float]) -> Dict[str, float]:
        self._buf.append(dict(cmd))
        if len(self._buf) <= self.steps:
            return {k: 0.0 for k in cmd}
        return self._buf[0]


class BodyVelocityController:
    """PI body-velocity + yaw-rate controller with roll/pitch stabilization.

    Inputs per tick:
      target: dict(surge, sway, heave, yaw_rate)  [m/s, rad/s]
      meas_lin_body: measured body-frame linear velocity (3,)
      meas_ang_body: measured body-frame angular rates (3,) [rad/s]
      roll, pitch: measured attitude [rad]
    Output: 8 thruster forces (numpy array).
    """

    def __init__(self, cfg: DynamicsConfig, dt: float) -> None:
        self.cfg = cfg
        self.dt = float(dt)
        self.allocator = ThrusterAllocator(cfg.max_thrust)
        self.latency = CommandLatencyBuffer(cfg.command_latency_s, self.dt)
        self._int_lin = np.zeros(3)
        self._int_yaw = 0.0
        self._sp_lin = np.zeros(3)
        self._sp_yaw_rate = 0.0
        self.last_wrench = np.zeros(6)
        self.last_forces = np.zeros(8)
        self.last_setpoint = dict(surge=0.0, sway=0.0, heave=0.0, yaw_rate=0.0)

    def reset(self) -> None:
        self._int_lin[:] = 0.0
        self._int_yaw = 0.0
        self._sp_lin[:] = 0.0
        self._sp_yaw_rate = 0.0

    def _rate_limit(self, current: float, target: float, max_rate: float) -> float:
        step = max_rate * self.dt
        return current + max(-step, min(step, target - current))

    def update(
        self,
        target: Dict[str, float],
        meas_lin_body: np.ndarray,
        meas_ang_body: np.ndarray,
        roll: float,
        pitch: float,
    ) -> np.ndarray:
        cfg = self.cfg
        delayed = self.latency.push(target)

        # Rate-limit the setpoint so step commands become bounded-accel ramps.
        raw = np.array([
            float(delayed.get("surge", 0.0)),
            float(delayed.get("sway", 0.0)),
            float(delayed.get("heave", 0.0)),
        ])
        for i in range(3):
            self._sp_lin[i] = self._rate_limit(self._sp_lin[i], raw[i], cfg.max_lin_accel)
        self._sp_yaw_rate = self._rate_limit(
            self._sp_yaw_rate, float(delayed.get("yaw_rate", 0.0)), cfg.max_yaw_accel
        )
        self.last_setpoint = dict(
            surge=self._sp_lin[0], sway=self._sp_lin[1],
            heave=self._sp_lin[2], yaw_rate=self._sp_yaw_rate,
        )

        # Linear velocity PI (body frame).
        err_lin = self._sp_lin - np.asarray(meas_lin_body, dtype=float).reshape(3)
        self._int_lin = np.clip(
            self._int_lin + err_lin * self.dt,
            -cfg.int_limit_lin, cfg.int_limit_lin,
        )
        force = (np.array(cfg.kp_lin) * err_lin
                 + np.array(cfg.ki_lin) * self._int_lin)

        # Yaw-rate PI.
        yaw_rate_meas = float(np.asarray(meas_ang_body, dtype=float).reshape(3)[2])
        err_yaw = self._sp_yaw_rate - yaw_rate_meas
        self._int_yaw = float(np.clip(
            self._int_yaw + err_yaw * self.dt,
            -cfg.int_limit_yaw, cfg.int_limit_yaw,
        ))
        mz = cfg.kp_yaw_rate * err_yaw + cfg.ki_yaw_rate * self._int_yaw

        # Roll/pitch PD stabilization to zero.
        ang = np.asarray(meas_ang_body, dtype=float).reshape(3)
        mx = -cfg.kp_att * float(roll) - cfg.kd_att * float(ang[0])
        my = -cfg.kp_att * float(pitch) - cfg.kd_att * float(ang[1])

        wrench = np.array([force[0], force[1], force[2], mx, my, mz])
        self.last_wrench = wrench
        self.last_forces = self.allocator.allocate(wrench)
        return self.last_forces


def rotation_to_roll_pitch(R: np.ndarray) -> Tuple[float, float]:
    """Extract roll and pitch [rad] from a 3x3 rotation matrix (ZYX yaw-pitch-roll)."""
    R = np.asarray(R, dtype=float)
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    return roll, pitch


def world_to_body(v_world: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Rotate a world-frame vector into the body frame given body-to-world R."""
    return np.asarray(R, dtype=float).T @ np.asarray(v_world, dtype=float).reshape(3)
