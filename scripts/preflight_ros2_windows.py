#!/usr/bin/env python
"""Preflight checks for the HoloObstacleAvoidance ROS 2 Windows workspace."""

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


class Reporter:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.warnings: list[str] = []

    def ok(self, label: str, detail: str = "") -> None:
        suffix = f" - {detail}" if detail else ""
        print(f"[OK]   {label}{suffix}")

    def warn(self, label: str, detail: str) -> None:
        self.warnings.append(f"{label}: {detail}")
        print(f"[WARN] {label} - {detail}")

    def fail(self, label: str, detail: str) -> None:
        self.failures.append(f"{label}: {detail}")
        print(f"[FAIL] {label} - {detail}")


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    reporter = Reporter()
    expected_python = get_expected_python()
    expected_distro = get_expected_distro()

    print("HoloObstacleAvoidance ROS 2 Windows preflight")
    print(f"Workspace: {workspace}")
    print(f"Expected Python: {format_version(expected_python)}")
    print(f"Expected ROS_DISTRO: {expected_distro}")
    print()

    check_python(reporter, expected_python)
    check_command(reporter, "ros2", ["ros2", "--help"])
    check_command(reporter, "colcon", ["colcon", "--help"])
    check_import(reporter, "rclpy")
    check_ros_distro(reporter, expected_distro)
    check_workspace_discovery(workspace, reporter)

    print()
    if reporter.failures:
        print("[RESULT] Preflight failed.")
        for failure in reporter.failures:
            print(f"  - {failure}")
        return 1

    print("[RESULT] Preflight passed.")
    if reporter.warnings:
        print("[RESULT] Warnings:")
        for warning in reporter.warnings:
            print(f"  - {warning}")
    return 0


def get_expected_distro() -> str:
    return os.environ.get("ROS_DISTRO_EXPECTED", os.environ.get("ROS_DISTRO", "lyrical")).lower()


def get_expected_python() -> tuple[int, int, int]:
    configured = os.environ.get("EXPECTED_PYTHON_VERSION") or os.environ.get("PYTHON_VERSION")
    if configured:
        try:
            return parse_version(configured)
        except ValueError:
            print(f"[WARN] Invalid EXPECTED_PYTHON_VERSION={configured!r}; using Lyrical default.")
    return (3, 12, 3)


def parse_version(text: str) -> tuple[int, int, int]:
    parts = text.strip().split(".")
    if len(parts) != 3:
        raise ValueError(text)
    return int(parts[0]), int(parts[1]), int(parts[2])


def format_version(version: tuple[int, int, int]) -> str:
    return ".".join(str(part) for part in version)


def check_python(reporter: Reporter, expected: tuple[int, int, int]) -> None:
    current = sys.version_info[:3]
    if current == expected:
        reporter.ok("Python version", format_version(current))
    else:
        reporter.fail("Python version", f"{format_version(current)}, expected {format_version(expected)}")


def check_command(reporter: Reporter, name: str, args: list[str]) -> None:
    command_path = shutil.which(name)
    if command_path is None:
        reporter.fail(f"{name} command", "not found on PATH")
        return
    reporter.ok(f"{name} command", command_path)

    result = run(args)
    if result.returncode == 0:
        reporter.ok(" ".join(args))
    else:
        reporter.fail(" ".join(args), result.stderr_or_stdout())


def check_import(reporter: Reporter, module_name: str) -> None:
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:  # noqa: BLE001
        reporter.fail(f"import {module_name}", str(exc))
        return
    version = getattr(module, "__version__", "")
    reporter.ok(f"import {module_name}", f"version {version}" if version else "imported")


def check_ros_distro(reporter: Reporter, expected: str) -> None:
    active = os.environ.get("ROS_DISTRO", "").lower()
    if active == expected:
        reporter.ok("ROS_DISTRO", active)
    elif active:
        reporter.fail("ROS_DISTRO", f"{active}, expected {expected}")
    else:
        reporter.warn("ROS_DISTRO", "not set; confirm the ROS setup script was called")


def check_workspace_discovery(workspace: Path, reporter: Reporter) -> None:
    src = workspace / "src"
    if not src.exists():
        reporter.fail("workspace src", f"missing: {src}")
        return

    result = run(["colcon", "list", "--base-paths", str(src), "--names-only"])
    if result.returncode != 0:
        reporter.fail("colcon workspace discovery", result.stderr_or_stdout())
        return

    discovered = {line.strip() for line in result.stdout.splitlines() if line.strip()}
    missing = sorted(EXPECTED_PACKAGES - discovered)
    if missing:
        reporter.fail("workspace package discovery", f"missing {missing}")
    else:
        reporter.ok("workspace package discovery", ", ".join(sorted(discovered)))


class CommandResult:
    def __init__(self, completed: subprocess.CompletedProcess[str] | None, exc: Exception | None = None) -> None:
        self.completed = completed
        self.exc = exc
        self.returncode = completed.returncode if completed else 1
        self.stdout = completed.stdout if completed else ""
        self.stderr = completed.stderr if completed else ""

    def stderr_or_stdout(self) -> str:
        if self.exc is not None:
            return str(self.exc)
        return (self.stderr or self.stdout or f"exit code {self.returncode}").strip()


def run(args: list[str]) -> CommandResult:
    try:
        return CommandResult(
            subprocess.run(args, check=False, capture_output=True, text=True, timeout=60)
        )
    except Exception as exc:  # noqa: BLE001
        return CommandResult(None, exc)


if __name__ == "__main__":
    raise SystemExit(main())

