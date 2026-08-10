"""BlueROV2 dynamics step-response identification in HoloOcean.

Runs the sim server class DIRECTLY (no ROS, no TCP) in the holoocean Python
environment, commands step profiles on each body axis, logs the response per
tick, and writes machine-readable results + a summary.

These simulated responses are the S-level baseline that will later be compared
against measured real BlueROV2 step responses (WP: sim-to-real calibration).

Usage (holoocean env):
    python scripts/run_bluerov2_step_response.py ^
        --scenario src/rov_obstacle_sim_bridge/config/holoocean_scenarios/bluerov2_dynamics_smoke.yaml ^
        --out experiments/simulation/step_response_S0

Axes: surge, sway, heave, yaw_rate. Profile per axis:
    2 s zero -> hold_s at amplitude -> 4 s zero (settle)
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SERVER_DIR = os.path.join(
    REPO, "src", "rov_obstacle_sim_bridge", "holoocean_server")
sys.path.insert(0, SERVER_DIR)


def git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
    except Exception:
        return "unknown"


def run_axis(server, axis: str, amplitude: float, hold_s: float,
             ticks_per_sec: int) -> list:
    rows = []
    phases = [(2.0, 0.0), (hold_s, amplitude), (4.0, 0.0)]
    t_axis = 0.0
    for phase_s, value in phases:
        for _ in range(int(phase_s * ticks_per_sec)):
            cmd = {"surge": 0.0, "sway": 0.0, "heave": 0.0,
                   "roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0}
            cmd[axis] = value
            server.apply_command(cmd)
            header, _ = server.step()
            dyn = header.get("dynamics", {})
            rows.append({
                "t": t_axis,
                "cmd": value,
                "setpoint": dyn.get("setpoint", {}).get(
                    axis if axis != "yaw_rate" else "yaw_rate"),
                "achieved_body_velocity": dyn.get("achieved_body_velocity"),
                "angular_rates_body": dyn.get("angular_rates_body"),
                "roll": dyn.get("roll"),
                "pitch": dyn.get("pitch"),
                "thruster_forces": dyn.get("thruster_forces"),
                "pose": header.get("pose"),
                "depth": header.get("depth"),
            })
            t_axis += 1.0 / ticks_per_sec
    return rows


AXIS_MEAS_INDEX = {"surge": 0, "sway": 1, "heave": 2}


def measured_value(row: dict, axis: str) -> float:
    if axis == "yaw_rate":
        rates = row.get("angular_rates_body") or [0, 0, 0]
        return float(rates[2])
    vel = row.get("achieved_body_velocity") or [0, 0, 0]
    return float(vel[AXIS_MEAS_INDEX[axis]])


def summarize(rows: list, axis: str, amplitude: float,
              ticks_per_sec: int, hold_s: float) -> dict:
    """Rise time (10-90%), steady-state mean/error, overshoot."""
    step_start = 2.0
    step_end = 2.0 + hold_s
    during = [r for r in rows if step_start <= r["t"] < step_end]
    meas = [measured_value(r, axis) for r in during]
    if not meas:
        return {"axis": axis, "error": "no samples"}
    sign = 1.0 if amplitude >= 0 else -1.0
    m = [sign * v for v in meas]
    a = abs(amplitude)
    t10 = next((r["t"] - step_start for r, v in zip(during, m) if v >= 0.1 * a), None)
    t90 = next((r["t"] - step_start for r, v in zip(during, m) if v >= 0.9 * a), None)
    tail = m[int(len(m) * 0.6):]
    ss = sum(tail) / len(tail)
    return {
        "axis": axis,
        "amplitude": amplitude,
        "rise_time_10_90_s": (None if t10 is None or t90 is None else t90 - t10),
        "time_to_90pct_s": t90,
        "steady_state_mean": sign * ss,
        "steady_state_error": a - ss,
        "steady_state_error_pct": 100.0 * (a - ss) / a if a > 0 else None,
        "overshoot_pct": 100.0 * (max(m) - a) / a if a > 0 else None,
        "final_residual_after_release": measured_value(rows[-1], axis),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default=os.path.join(
        REPO, "src", "rov_obstacle_sim_bridge", "config",
        "holoocean_scenarios", "bluerov2_dynamics_smoke.yaml"))
    parser.add_argument("--out", default=os.path.join(
        REPO, "experiments", "simulation", "step_response_S0"))
    parser.add_argument("--hold-s", type=float, default=8.0)
    parser.add_argument("--amplitudes", default="surge=0.4,sway=0.25,heave=-0.2,yaw_rate=0.3")
    args = parser.parse_args()

    from holoocean_sim_server import HolooceanSimServer, load_config

    amplitudes = {}
    for part in args.amplitudes.split(","):
        k, v = part.split("=")
        amplitudes[k.strip()] = float(v)

    cfg = load_config(args.scenario)
    if cfg.motion_model != "dynamics":
        print("FATAL: step response requires motion_model: dynamics")
        return 2

    os.makedirs(args.out, exist_ok=True)
    server = HolooceanSimServer(cfg, verbose=True)
    t0 = time.time()
    server.start()

    results = {}
    summaries = []
    try:
        for axis, amplitude in amplitudes.items():
            print(f"[step-response] axis={axis} amplitude={amplitude}")
            rows = run_axis(server, axis, amplitude, args.hold_s,
                            cfg.ticks_per_sec)
            results[axis] = rows
            s = summarize(rows, axis, amplitude, cfg.ticks_per_sec, args.hold_s)
            summaries.append(s)
            print(f"[step-response] {axis}: {json.dumps(s)}")
    finally:
        server.close()

    manifest = {
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "commit_sha": git_sha(),
        "scenario": os.path.relpath(args.scenario, REPO),
        "ticks_per_sec": cfg.ticks_per_sec,
        "hold_s": args.hold_s,
        "amplitudes": amplitudes,
        "dynamics_config": cfg.dynamics,
        "wall_time_s": round(time.time() - t0, 1),
        "summaries": summaries,
    }
    with open(os.path.join(args.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    with open(os.path.join(args.out, "step_response_raw.json"), "w") as f:
        json.dump(results, f)
    print(f"[step-response] wrote {args.out}")

    failures = [s for s in summaries
                if s.get("time_to_90pct_s") is None
                or (s.get("steady_state_error_pct") or 0) > 15]
    if failures:
        print(f"[step-response] WARNING: {len(failures)} axes out of spec: "
              f"{[f['axis'] for f in failures]}")
        return 1
    print("[step-response] all axes within spec (reach 90% and <15% ss error)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
