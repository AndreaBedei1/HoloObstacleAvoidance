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


def start_stable_logged(
    cmd: list,
    log_path: Path,
    name: str,
    *,
    attempts: int = 3,
    settle_s: float = 3.0,
) -> subprocess.Popen:
    """Start a process and retry if it exits during the first few seconds."""
    last_proc: subprocess.Popen | None = None
    for attempt in range(1, max(1, attempts) + 1):
        actual_log = log_path
        if attempt > 1:
            actual_log = log_path.with_name(
                f"{log_path.stem}_attempt{attempt}{log_path.suffix}"
            )
        proc = start_logged(cmd, actual_log, f"{name} attempt {attempt}")
        last_proc = proc
        time.sleep(max(0.0, settle_s))
        if proc.poll() is None:
            return proc
        print(
            f"[closed-loop] {name} exited early with code {proc.returncode}; "
            "retrying"
        )
    assert last_proc is not None
    return last_proc


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


def bool_arg(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"expected true/false, got {value!r}")


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
    parser.add_argument("--ros-start-mode", choices=("launch", "direct"),
                        default="launch",
                        help="start ROS nodes via ros2 launch or direct Python modules")
    parser.add_argument("--model-path", default=str(DEFAULT_YOLO_WEIGHTS),
                        help="YOLO weights path when --detector yolo")
    parser.add_argument("--confidence-threshold", type=float, default=0.25,
                        help="YOLO confidence threshold when --detector yolo")
    parser.add_argument("--inference-stride", type=int, default=1,
                        help="YOLO inference stride when --detector yolo")
    parser.add_argument("--nominal-publisher-enabled", type=bool_arg, default=True,
                        help="start the launch-file nominal publisher")
    parser.add_argument("--manual-nominal-command", action="store_true",
                        help="publish /cmd_vel_nominal with ros2 topic pub")
    parser.add_argument("--manual-nominal-x", type=float, default=0.4)
    parser.add_argument("--manual-nominal-y", type=float, default=0.0)
    parser.add_argument("--manual-nominal-z", type=float, default=0.0)
    parser.add_argument("--manual-nominal-yaw-rate", type=float, default=0.0)
    parser.add_argument("--manual-nominal-rate-hz", type=float, default=10.0)
    parser.add_argument("--manual-nominal-start-delay-s", type=float, default=8.0)
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
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")
    os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
    print(
        "[closed-loop] ROS middleware: "
        f"{os.environ.get('RMW_IMPLEMENTATION')}"
    )

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
    manual_nominal_proc = None
    direct_node_procs: list[tuple[subprocess.Popen, str]] = []
    validator = None
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

        validate_cmd = [
            sys.executable, str(REPO_ROOT / "scripts" / "validate_holoocean_closed_loop.py"),
            "--duration-s", str(args.duration_s),
            "--save-images-dir", str(vis),
            "--image-prefix", label,
            "--report-json", str(report_json),
        ]

        if args.ros_start_mode == "direct":
            bridge_cmd = [
                sys.executable,
                "-c",
                "from rov_obstacle_sim_bridge.holoocean_bridge_node import main; main()",
                "--ros-args",
                "-p", f"host:={args.host}",
                "-p", f"port:={args.port}",
            ]
            bridge_proc = start_stable_logged(
                bridge_cmd, logs / f"{label}_bridge_node.log", "bridge node"
            )
            direct_node_procs.append((bridge_proc, "bridge node"))

            planner_cmd = [
                sys.executable,
                "-c",
                "from rov_obstacle_avoidance.local_avoidance_planner_node import main; main()",
            ]
            planner_proc = start_stable_logged(
                planner_cmd, logs / f"{label}_planner_node.log", "planner node"
            )
            direct_node_procs.append((planner_proc, "planner node"))

            if args.detector == "yolo":
                yolo_cmd = [
                    sys.executable,
                    "-c",
                    "from rov_obstacle_perception.yolo_obstacle_detector_node import main; main()",
                    "--ros-args",
                    "-p", f"model_path:={args.model_path}",
                    "-p", f"confidence_threshold:={args.confidence_threshold}",
                    "-p", f"inference_stride:={max(1, args.inference_stride)}",
                ]
                yolo_proc = start_stable_logged(
                    yolo_cmd,
                    logs / f"{label}_yolo_detector_node.log",
                    "YOLO detector node",
                )
                direct_node_procs.append((yolo_proc, "YOLO detector node"))
        else:
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
                        (
                            "nominal_publisher_enabled:="
                            f"{str(args.nominal_publisher_enabled).lower()}"
                        ),
                    ]
                )
            launch_proc = start_logged(
                launch_cmd,
                logs / f"{label}_ros2_launch.log", "ROS 2 nodes",
            )

        print(f"[closed-loop] starting validator ({args.duration_s:.0f}s)")
        validator = subprocess.Popen(validate_cmd)
        time.sleep(2.0)

        if args.manual_nominal_command:
            time.sleep(max(0.0, args.manual_nominal_start_delay_s))
            twist_yaml = (
                "{linear: {"
                f"x: {args.manual_nominal_x}, "
                f"y: {args.manual_nominal_y}, "
                f"z: {args.manual_nominal_z}"
                "}, angular: {"
                f"z: {args.manual_nominal_yaw_rate}"
                "}}"
            )
            manual_nominal_proc = start_logged(
                [
                    "ros2", "topic", "pub", "/cmd_vel_nominal",
                    "geometry_msgs/msg/Twist", twist_yaml,
                    "-r", str(args.manual_nominal_rate_hz),
                    "-w", "1",
                    "--max-wait-time-secs", "20",
                ],
                logs / f"{label}_manual_nominal_pub.log",
                "manual nominal command publisher",
            )

        validator.wait()
        result = validator

        print(f"[closed-loop] validator exit code: {result.returncode}")
        if report_json.is_file():
            report = json.loads(report_json.read_text(encoding="utf-8"))
            report["detector"] = args.detector
            report["ros_start_mode"] = args.ros_start_mode
            report["oracle_relay_enabled"] = args.detector != "yolo"
            report["yolo_weights"] = str(args.model_path) if args.detector == "yolo" else ""
            report["launch_nominal_publisher_enabled"] = bool(
                args.nominal_publisher_enabled
            )
            report["nominal_command_source"] = (
                "manual_ros_topic_pub"
                if args.manual_nominal_command
                else (
                    "launch_nominal_publisher"
                    if args.nominal_publisher_enabled
                    else "none"
                )
            )
            report["manual_nominal_command"] = {
                "topic": "/cmd_vel_nominal",
                "type": "geometry_msgs/msg/Twist",
                "rate_hz": float(args.manual_nominal_rate_hz),
                "start_delay_s": float(args.manual_nominal_start_delay_s),
                "linear": {
                    "x": float(args.manual_nominal_x),
                    "y": float(args.manual_nominal_y),
                    "z": float(args.manual_nominal_z),
                },
                "angular": {"z": float(args.manual_nominal_yaw_rate)},
            } if args.manual_nominal_command else None
            report["direct_node_statuses"] = [
                {
                    "name": name,
                    "pid": proc.pid,
                    "returncode": proc.poll(),
                    "running": proc.poll() is None,
                }
                for proc, name in direct_node_procs
            ]
            report["launch_returncode"] = (
                launch_proc.poll() if launch_proc is not None else None
            )
            report["manual_nominal_returncode"] = (
                manual_nominal_proc.poll()
                if manual_nominal_proc is not None
                else None
            )
            report_json.write_text(
                json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
            )
            print(report_json.read_text(encoding="utf-8"))
        return int(result.returncode)
    finally:
        stop(manual_nominal_proc, "manual nominal command publisher")
        stop(validator, "validator")
        for proc, name in reversed(direct_node_procs):
            stop(proc, name)
        stop(launch_proc, "ROS 2 nodes")
        stop(zenoh_proc, "zenoh router")
        stop(sim_proc, "sim server")
        print(f"[closed-loop] evidence: {vis}\\{label}_frame*.png, {report_json}")


if __name__ == "__main__":
    raise SystemExit(main())
