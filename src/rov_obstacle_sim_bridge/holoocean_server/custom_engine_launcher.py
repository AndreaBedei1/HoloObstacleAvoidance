"""Helpers to launch the EXTERNAL modified HoloOcean engine (read-only usage).

The modified HoloOcean engine is an Unreal Engine 5.3 *editor project*
(``Holodeck.uproject``) living in an external folder that this repository
treats as strictly read-only.  It is started in visible ``-game`` mode via
the stock Unreal Editor binary and then a HoloOcean Python client attaches
to it through shared memory (``holoocean.make(scenario_cfg=..., start_world=False)``).

This module is dependency-light (stdlib + PyYAML) and works on both Python
3.9 (conda ``ocean``) and Python 3.12 (pixi ROS 2 env), so the same code can
be used by the sim server, by capture scripts and by unit tests.

Nothing here ever writes into the external engine folder: the engine log is
redirected into this repository with ``-abslog`` and all spawning happens at
runtime in engine memory.
"""

from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path
from typing import Any, Optional

import yaml

#: Environment variable that overrides the default engine config location.
ENGINE_CONFIG_ENV_VAR = "HOLO_CUSTOM_ENGINE_CONFIG"

#: Repo root == three levels above this file (src/<pkg>/holoocean_server/).
_REPO_ROOT = Path(__file__).resolve().parents[3]

#: Default engine config shipped with the repository.
DEFAULT_ENGINE_CONFIG_PATH = _REPO_ROOT / "config" / "custom_holoocean_engine.yaml"


def repo_root() -> Path:
    """Absolute path of the HoloObstacleAvoidance repository root."""
    return _REPO_ROOT


def resolve_engine_config_path(explicit: Optional[str] = None) -> Path:
    """Resolution order: explicit arg > env var > repo default."""
    if explicit:
        return Path(explicit).expanduser().resolve()
    env_value = os.environ.get(ENGINE_CONFIG_ENV_VAR, "").strip()
    if env_value:
        return Path(env_value).expanduser().resolve()
    return DEFAULT_ENGINE_CONFIG_PATH


def load_engine_config(path: Optional[str] = None) -> dict:
    """Load the engine YAML into a plain dict (raises on missing file)."""
    cfg_path = resolve_engine_config_path(path)
    if not cfg_path.is_file():
        raise FileNotFoundError(
            f"Custom engine config not found: {cfg_path} "
            f"(set {ENGINE_CONFIG_ENV_VAR} or pass an explicit path)"
        )
    with open(cfg_path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Engine config must be a YAML mapping: {cfg_path}")
    data.setdefault("__config_path__", str(cfg_path))
    return data


def validate_engine_config(cfg: dict, check_paths: bool = True) -> list:
    """Return a list of human-readable problems (empty list == valid).

    ``check_paths=False`` validates only the structure, so unit tests can run
    on machines without the external engine folder.
    """
    problems: list = []
    engine = cfg.get("external_engine")
    if not isinstance(engine, dict):
        return ["missing 'external_engine' section"]

    for key in ("ue_editor_exe", "uproject", "default_map"):
        if not str(engine.get(key, "") or "").strip():
            problems.append(f"external_engine.{key} is empty or missing")

    launch = cfg.get("launch", {})
    if not isinstance(launch, dict):
        problems.append("'launch' section must be a mapping")
        launch = {}
    for key in ("res_x", "res_y", "ticks_per_sec"):
        value = launch.get(key)
        if value is not None:
            try:
                if int(value) <= 0:
                    problems.append(f"launch.{key} must be positive")
            except (TypeError, ValueError):
                problems.append(f"launch.{key} must be an integer")

    if check_paths and not problems:
        exe = Path(str(engine["ue_editor_exe"]))
        project = Path(str(engine["uproject"]))
        if not exe.is_file():
            problems.append(f"UE editor not found: {exe}")
        if not project.is_file():
            problems.append(f"Holodeck.uproject not found: {project}")
    return problems


def external_engine_available(cfg: Optional[dict] = None) -> bool:
    """True when the configured external engine paths exist on this machine."""
    try:
        data = cfg if cfg is not None else load_engine_config()
    except (FileNotFoundError, ValueError):
        return False
    return not validate_engine_config(data, check_paths=True)


def engine_log_path(cfg: dict) -> Path:
    """Absolute engine log path (inside this repo unless configured absolute)."""
    raw = str(cfg.get("launch", {}).get("log_file", "logs/ue_custom_engine.log"))
    path = Path(raw)
    if not path.is_absolute():
        path = _REPO_ROOT / path
    return path


def build_engine_command(cfg: dict, map_name: Optional[str] = None) -> list:
    """Build the exact UE command line for a VISIBLE ``-game`` run.

    The command intentionally has no offscreen/nullrhi flags: these launchers
    only support visible rendering.
    """
    engine = cfg["external_engine"]
    launch = cfg.get("launch", {}) or {}
    world = map_name or str(engine.get("default_map", "ExampleLevel"))

    cmd = [
        str(engine["ue_editor_exe"]),
        str(engine["uproject"]),
        f"/Game/{world}",
        "-game",
        # Live Coding is an editor hot-reload feature; in -game runs it adds
        # a startup thread that intermittently crashes with a data race.
        "-NoLiveCoding",
    ]
    if bool(launch.get("windowed", True)):
        cmd.append("-windowed")
    cmd.append(f"-ResX={int(launch.get('res_x', 1280))}")
    cmd.append(f"-ResY={int(launch.get('res_y', 720))}")
    cmd.append(f"-TicksPerSec={int(launch.get('ticks_per_sec', 30))}")
    frames = launch.get("frames_per_sec", 30)
    if frames:
        cmd.append(f"-FramesPerSec={int(frames)}")
    log_path = engine_log_path(cfg)
    cmd.append(f"-abslog={log_path}")
    for extra in launch.get("extra_args", []) or []:
        cmd.append(str(extra))
    return cmd


def launch_engine(
    cfg: dict,
    map_name: Optional[str] = None,
    verbose: bool = True,
) -> "subprocess.Popen[Any]":
    """Start the external engine in a new visible window and return the process."""
    problems = validate_engine_config(cfg, check_paths=True)
    if problems:
        raise RuntimeError(
            "Cannot launch external engine:\n  - " + "\n  - ".join(problems)
        )
    log_path = engine_log_path(cfg)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = build_engine_command(cfg, map_name=map_name)
    if verbose:
        print("[custom-engine] launching (visible):", flush=True)
        print("  " + subprocess.list2cmdline(cmd), flush=True)

    creation_flags = 0
    if os.name == "nt":  # detach into its own console/window group
        creation_flags = subprocess.CREATE_NEW_PROCESS_GROUP
    return subprocess.Popen(cmd, creationflags=creation_flags)


def stop_engine(process: Optional["subprocess.Popen[Any]"], timeout_s: float = 10.0) -> None:
    """Terminate an engine process previously returned by :func:`launch_engine`."""
    if process is None or process.poll() is not None:
        return
    try:
        process.terminate()
        process.wait(timeout=timeout_s)
    except Exception:
        try:
            process.kill()
        except Exception:
            pass


def engine_ready_signal_pending(uuid: str = "") -> Optional[bool]:
    """Probe the engine's one-time "server ready" semaphore release (Windows).

    HoloOcean's attach protocol (``start_world=False``) begins with the client
    acquiring the CLIENT semaphore with an INFINITE timeout.  The engine
    releases it exactly once when the map starts, so:

    * ``True``  -> release pending: a FRESH engine window nobody attached to
      (safe to call ``holoocean.make``; the probe restores the count),
    * ``False`` -> no release pending: either the window was already used by
      a previous client (attaching would DEADLOCK) or the map is mid-start,
    * ``None``  -> semaphores don't exist yet: engine still loading.
    """
    if os.name != "nt":  # pragma: no cover - Windows-only integration
        return True
    import win32event

    semaphore_all_access = 0x1F0003
    try:
        sem = win32event.OpenSemaphore(
            semaphore_all_access, False, "Global\\HOLODECK_SEMAPHORE_CLIENT" + uuid
        )
    except Exception:
        return None
    if not sem:
        return None
    rc = win32event.WaitForSingleObject(sem, 0)
    if rc == win32event.WAIT_OBJECT_0:
        win32event.ReleaseSemaphore(sem, 1)  # restore the ready signal
        return True
    return False


def attach_holoocean(
    scenario_cfg: dict,
    cfg: dict,
    engine_process: Optional["subprocess.Popen[Any]"] = None,
    verbose: bool = True,
):
    """Attach the HoloOcean Python client to the already-running engine.

    Waits until the engine's shared-memory server is up (its one-time ready
    signal is pending) before calling
    ``holoocean.make(scenario_cfg=..., start_world=False)`` — calling it
    against a window that a previous client already used would deadlock
    forever inside an INFINITE semaphore wait.
    Must run in the conda ``ocean`` environment (needs ``holoocean``).
    """
    import holoocean  # deferred: only available in the ocean env

    attach = cfg.get("attach", {}) or {}
    timeout_s = float(attach.get("timeout_s", 900.0))
    poll_s = max(1.0, float(attach.get("poll_interval_s", 5.0)))
    deadline = time.time() + timeout_s
    last_error: Optional[BaseException] = None
    saw_stale_window = False

    while time.time() < deadline:
        if engine_process is not None and engine_process.poll() is not None:
            raise RuntimeError(
                f"Engine process exited early with code {engine_process.returncode}; "
                f"see {engine_log_path(cfg)}"
            )
        pending = engine_ready_signal_pending()
        if pending is not True:
            saw_stale_window = pending is False
            if verbose:
                state = "still loading" if pending is None else (
                    "no ready signal (window already used by a previous client?)"
                )
                print(f"[custom-engine] engine {state}; retrying in {poll_s:.0f}s",
                      flush=True)
            time.sleep(poll_s)
            continue
        try:
            env = holoocean.make(scenario_cfg=scenario_cfg, start_world=False)
            if verbose:
                print("[custom-engine] HoloOcean client attached", flush=True)
            return env
        except BaseException as exc:  # TimeoutError, pywin32 errors, ...
            last_error = exc
            if verbose:
                print(
                    f"[custom-engine] attach failed ({type(exc).__name__}); "
                    f"retrying in {poll_s:.0f}s",
                    flush=True,
                )
            time.sleep(poll_s)
    hint = (
        " The engine window seems to have been used by a previous client "
        "session — HoloOcean supports only one attach per engine start. "
        "Close the engine window and launch a fresh one."
        if saw_stale_window
        else ""
    )
    raise TimeoutError(
        f"Could not attach to the engine within {timeout_s:.0f}s "
        f"(last error: {last_error!r}).{hint}"
    )
