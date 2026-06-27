#!/usr/bin/env python3
"""Cross-platform ROS 2 preflight checks for this workspace."""

from __future__ import annotations

import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys


EXPECTED_PACKAGES = {
    "rov_obstacle_msgs",
    "rov_obstacle_perception",
    "rov_obstacle_avoidance",
    "rov_obstacle_bringup",
}


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    expected_distro = os.environ.get("ROS_DISTRO_EXPECTED") or os.environ.get("ROS_DISTRO", "")
    failures: list[str] = []

    print("HoloObstacleAvoidance ROS 2 preflight")
    print(f"Workspace: {workspace}")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Expected ROS_DISTRO: {expected_distro or '(any sourced distro)'}")
    print()

    _check_command("ros2", ["ros2", "--help"], failures)
    _check_command("colcon", ["colcon", "--help"], failures)
    _check_import("rclpy", failures)
    _check_ros_distro(expected_distro, failures)
    _check_workspace_packages(workspace, failures)

    print()
    if failures:
        print("[RESULT] Preflight failed.")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("[RESULT] Preflight passed.")
    return 0


def _check_command(name: str, args: list[str], failures: list[str]) -> None:
    command_path = shutil.which(name)
    if command_path is None:
        _fail(f"{name} command", "not found on PATH", failures)
        return
    _ok(f"{name} command", command_path)
    result = _run(args)
    if result.returncode == 0:
        _ok(" ".join(args))
    else:
        _fail(" ".join(args), (result.stderr or result.stdout).strip(), failures)


def _check_import(module_name: str, failures: list[str]) -> None:
    try:
        importlib.import_module(module_name)
    except Exception as exc:  # noqa: BLE001
        _fail(f"import {module_name}", str(exc), failures)
        return
    _ok(f"import {module_name}", "imported")


def _check_ros_distro(expected_distro: str, failures: list[str]) -> None:
    active = os.environ.get("ROS_DISTRO", "")
    if not active:
        _fail("ROS_DISTRO", "not set; source the ROS 2 setup file first", failures)
        return
    if expected_distro and active.lower() != expected_distro.lower():
        _fail("ROS_DISTRO", f"{active}, expected {expected_distro}", failures)
        return
    _ok("ROS_DISTRO", active)


def _check_workspace_packages(workspace: Path, failures: list[str]) -> None:
    src = workspace / "src"
    if not src.exists():
        _fail("workspace src", f"missing: {src}", failures)
        return
    result = _run(["colcon", "list", "--base-paths", str(src), "--names-only"])
    if result.returncode != 0:
        _fail("workspace package discovery", (result.stderr or result.stdout).strip(), failures)
        return
    discovered = {line.strip() for line in result.stdout.splitlines() if line.strip()}
    missing = sorted(EXPECTED_PACKAGES - discovered)
    if missing:
        _fail("workspace package discovery", f"missing {missing}", failures)
        return
    _ok("workspace package discovery", ", ".join(sorted(discovered)))


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(args, check=False, capture_output=True, text=True, timeout=60)


def _ok(label: str, detail: str = "") -> None:
    suffix = f" - {detail}" if detail else ""
    print(f"[OK]   {label}{suffix}")


def _fail(label: str, detail: str, failures: list[str]) -> None:
    failures.append(f"{label}: {detail}")
    print(f"[FAIL] {label} - {detail}")


if __name__ == "__main__":
    raise SystemExit(main())
