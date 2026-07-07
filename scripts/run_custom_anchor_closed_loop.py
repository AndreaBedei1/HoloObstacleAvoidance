#!/usr/bin/env python
"""One-command VISIBLE closed-loop run with the REAL custom anchor.

Orchestrates the full system (all simulation-only, all visible):

  1. HoloOcean sim server (conda ``ocean``) -- it launches the EXTERNAL
     modified engine in a visible window, attaches, spawns the real
     ``/Game/ancora.ancora`` mesh via SpawnAsset, then serves TCP 47654.
  2. Zenoh router + ROS 2 nodes via one of the simulation-only launches:
     oracle relay baseline or YOLO detector pipeline.
  3. Validator that collects metrics, saves camera frames (PNG) into
     ``visualizations/`` and writes a JSON report into ``logs/``.

Must be started from a shell where the ROS 2 environment is sourced
(``scripts\\run_custom_anchor_closed_loop.bat`` does this for you).

No MAVLink, no thrusters, no headless mode.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SCENARIO = (
    REPO_ROOT / "src" / "rov_obstacle_sim_bridge" / "config"
    / "holoocean_scenarios" / "custom_anchor_visible.yaml"
)
DEFAULT_YOLO_WEIGHTS = (
    REPO_ROOT / "training" / "yolo_custom_objects" / "runs"
    / "yolov8n_custom_underwater" / "weights" / "best.pt"
)
SIM_SERVER = (
    REPO_ROOT / "src" / "rov_obstacle_sim_bridge" / "holoocean_server"
    / "holoocean_sim_server.py"
)


def ocean_python_cmd(conda_env: str) -> list:
    """Command prefix for the conda env's Python.

    Prefers the env's python.exe directly (works even when ``conda`` is not
    on PATH — proven sufficient for holoocean); falls back to ``conda run``.
    Override with the OCEAN_PYTHON environment variable.
    """
    override = os.environ.get("OCEAN_PYTHON", "").strip()
    if override and Path(override).is_file():
        return [override]
    direct = Path.home() / ".conda" / "envs" / conda_env / "python.exe"
    if direct.is_file():
        return [str(direct)]
    return ["conda", "run", "--no-capture-output", "-n", conda_env, "python"]


def port_open(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def start_logged(cmd: list, log_path: Path, name: str) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, "w", encoding="utf-8", errors="replace")
    print(f"[closed-loop] starting {name}: {subprocess.list2cmdline(cmd)}")
    print(f"[closed-loop]   log -> {log_path}")
    return subprocess.Popen(
        cmd,
        stdout=log,
        stderr=subprocess.STDOUT,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
    )


def stop(proc: subprocess.Popen | None, name: str) -> None:
    if proc is None or proc.poll() is not None:
        return
    print(f"[closed-loop] stopping {name} (pid {proc.pid})")
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/PID", str(proc.pid), "/T", "/F"],
            capture_output=True,
        )
    else:
        proc.terminate()
    try:
        proc.wait(timeout=10)
    except Exception:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default=str(DEFAULT_SCENARIO))
    parser.add_argument("--duration-s", type=float, default=45.0)
    parser.add_argument("--conda-env", default="ocean")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=47654)
    parser.add_argument("--sim-ready-timeout-s", type=float, default=600.0)
    parser.add_argument("--engine-running", action="store_true",
                        help="reuse an engine window that is already open")
    parser.add_argument("--keep-engine", action="store_true",
                        help="leave the engine window open at the end")
    parser.add_argument("--detector", choices=("oracle", "yolo"), default="oracle",
                        help="planner input source: oracle relay baseline or YOLO")
    parser.add_argument("--model-path", default=str(DEFAULT_YOLO_WEIGHTS),
                        help="YOLO weights path when --detector yolo")
    parser.add_argument("--confidence-threshold", type=float, default=0.25,
                        help="YOLO confidence threshold when --detector yolo")
    parser.add_argument("--inference-stride", type=int, default=1,
                        help="YOLO inference stride when --detector yolo")
    parser.add_argument("--run-label", default="custom_anchor")
    args = parser.parse_args()

    scenario = Path(args.scenario)
    if not scenario.is_file():
        print(f"[closed-loop] scenario not found: {scenario}")
        return 2
    if not os.environ.get("ROS_DISTRO"):
        print("[closed-loop] ROS 2 environment not sourced "
              "(use scripts\\run_custom_anchor_closed_loop.bat)")
        return 2

    logs = REPO_ROOT / "logs"
    vis = REPO_ROOT / "visualizations"
    label = args.run_label
    if args.detector == "yolo" and label == "custom_anchor":
        label = "custom_anchor_yolo"
    report_json = logs / f"{label}_validation.json"

    sim_cmd = ocean_python_cmd(args.conda_env) + [
        str(SIM_SERVER), "--config", str(scenario), "--serve",
    ]
    if args.engine_running:
        sim_cmd.append("--engine-running")
    if args.keep_engine:
        sim_cmd.append("--keep-engine")

    zenoh_proc = None
    launch_proc = None
    sim_proc = None
    try:
        sim_proc = start_logged(sim_cmd, logs / f"{label}_sim_server.log", "sim server")

        print(f"[closed-loop] waiting for sim server on {args.host}:{args.port} "
              f"(engine load can take ~1-2 min)")
        deadline = time.time() + args.sim_ready_timeout_s
        while time.time() < deadline:
            if sim_proc.poll() is not None:
                print(f"[closed-loop] sim server exited early "
                      f"(code {sim_proc.returncode}); see its log")
                return 1
            if port_open(args.host, args.port):
                break
            time.sleep(2.0)
        else:
            print("[closed-loop] sim server never opened the port")
            return 1
        print("[closed-loop] sim server is ready (visible engine window up)")

        zenoh_proc = start_logged(
            ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
            logs / f"{label}_zenoh.log", "zenoh router",
        )
        time.sleep(3.0)

        # The validator subscribes BEFORE the nodes start so it observes the
        # whole episode (including the initial APPROACH_OBSTACLE phase).
        startup_grace_s = 15.0
        validate_cmd = [
            sys.executable, str(REPO_ROOT / "scripts" / "validate_holoocean_closed_loop.py"),
            "--duration-s", str(args.duration_s + startup_grace_s),
            "--save-images-dir", str(vis),
            "--image-prefix", label,
            "--report-json", str(report_json),
        ]
        print(f"[closed-loop] starting validator "
              f"({args.duration_s:.0f}s + {startup_grace_s:.0f}s startup grace)")
        validator = subprocess.Popen(validate_cmd)
        time.sleep(4.0)

        launch_file = (
            "holoocean_yolo_avoidance.launch.py"
            if args.detector == "yolo"
            else "holoocean_oracle_avoidance.launch.py"
        )
        launch_cmd = [
            "ros2", "launch", "rov_obstacle_sim_bridge", launch_file,
            f"host:={args.host}", f"port:={args.port}",
        ]
        if args.detector == "yolo":
            launch_cmd.extend(
                [
                    f"model_path:={args.model_path}",
                    f"confidence_threshold:={args.confidence_threshold}",
                    f"inference_stride:={max(1, args.inference_stride)}",
                ]
            )
        launch_proc = start_logged(
            launch_cmd,
            logs / f"{label}_ros2_launch.log", "ROS 2 nodes",
        )

        validator.wait()
        result = validator

        print(f"[closed-loop] validator exit code: {result.returncode}")
        if report_json.is_file():
            print(report_json.read_text(encoding="utf-8"))
        return int(result.returncode)
    finally:
        stop(launch_proc, "ROS 2 nodes")
        stop(zenoh_proc, "zenoh router")
        stop(sim_proc, "sim server")
        print(f"[closed-loop] evidence: {vis}\\{label}_frame*.png, {report_json}")


if __name__ == "__main__":
    raise SystemExit(main())
