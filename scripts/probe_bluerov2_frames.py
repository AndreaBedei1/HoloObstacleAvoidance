"""Empirically identify BlueROV2 thruster/sensor sign conventions in HoloOcean.

Open-loop probes: apply canonical wrenches through our allocation matrix for a
few seconds each, observe PoseSensor/VelocitySensor/IMU responses, and print
the measured mapping. No controller in the loop, no ROS.

Run in the holoocean env:
    python scripts/probe_bluerov2_frames.py
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SERVER_DIR = os.path.join(REPO, "src", "rov_obstacle_sim_bridge", "holoocean_server")
sys.path.insert(0, SERVER_DIR)

from bluerov2_dynamics import ThrusterAllocator, rotation_to_roll_pitch  # noqa: E402

import holoocean  # noqa: E402

SCENARIO = {
    "name": "frame_probe",
    "world": "SimpleUnderwater",
    "package_name": "Ocean",
    "main_agent": "rov0",
    "ticks_per_sec": 30,
    "frames_per_sec": False,
    "agents": [{
        "agent_name": "rov0",
        "agent_type": "BlueROV2",
        "sensors": [
            {"sensor_type": "PoseSensor"},
            {"sensor_type": "VelocitySensor"},
            {"sensor_type": "IMUSensor"},
            {"sensor_type": "DepthSensor"},
        ],
        "control_scheme": 0,
        "location": [0, 0, -10],
        "rotation": [0, 0, 0],
    }],
}


def settle(env, ticks=60):
    state = None
    for _ in range(ticks):
        state = env.step(np.zeros(8))
    return state


def pose_of(state):
    P = np.array(state["PoseSensor"])
    return P[:3, 3].copy(), P[:3, :3].copy()


def probe(env, forces, seconds, label):
    state = settle(env, 90)
    p0, R0 = pose_of(state)
    yaw0 = math.atan2(R0[1, 0], R0[0, 0])
    n = int(seconds * 30)
    vels, gyros = [], []
    for _ in range(n):
        state = env.step(np.asarray(forces, dtype=float))
        vels.append(np.array(state["VelocitySensor"]).reshape(3))
        imu = np.array(state["IMUSensor"])
        gyros.append(imu[1].reshape(3) if imu.shape[0] >= 2 else np.zeros(3))
    p1, R1 = pose_of(state)
    yaw1 = math.atan2(R1[1, 0], R1[0, 0])
    dp = p1 - p0
    dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    v_mean = np.mean(vels[len(vels) // 2:], axis=0)
    g_mean = np.mean(gyros[len(gyros) // 2:], axis=0)
    roll, pitch = rotation_to_roll_pitch(R1)
    out = {
        "label": label,
        "forces": list(map(float, forces)),
        "delta_pos_world": [round(float(v), 3) for v in dp],
        "delta_yaw_deg": round(math.degrees(dyaw), 2),
        "mean_velocity_sensor": [round(float(v), 3) for v in v_mean],
        "mean_imu_gyro": [round(float(v), 3) for v in g_mean],
        "final_roll_deg": round(math.degrees(roll), 2),
        "final_pitch_deg": round(math.degrees(pitch), 2),
        "yaw_rate_from_pose_deg_s": round(math.degrees(dyaw) / seconds, 2),
    }
    print(json.dumps(out))
    return out


def main():
    alloc = ThrusterAllocator(max_thrust=1e6)
    with holoocean.make(scenario_cfg=SCENARIO, show_viewport=True) as env:
        results = []
        # Canonical wrenches through OUR allocation matrix (magnitudes small).
        results.append(probe(env, alloc.allocate([8, 0, 0, 0, 0, 0]), 4, "wrench +Fx"))
        results.append(probe(env, alloc.allocate([0, 8, 0, 0, 0, 0]), 4, "wrench +Fy"))
        results.append(probe(env, alloc.allocate([0, 0, 8, 0, 0, 0]), 4, "wrench +Fz"))
        results.append(probe(env, alloc.allocate([0, 0, 0, 0, 0, 2]), 4, "wrench +Mz"))
        # Single-thruster probes to pin the physical order (indices 4 and 0).
        f = np.zeros(8); f[4] = 8.0
        results.append(probe(env, f, 3, "thruster[4] only +8N"))
        f = np.zeros(8); f[0] = 8.0
        results.append(probe(env, f, 3, "thruster[0] only +8N"))

        out_path = os.path.join(REPO, "experiments", "simulation",
                                "frame_probe", "probe_results.json")
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w") as fh:
            json.dump(results, fh, indent=2)
        print("wrote", out_path)


if __name__ == "__main__":
    main()
