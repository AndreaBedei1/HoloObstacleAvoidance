#!/usr/bin/env python
"""Visible verification of the EXTERNAL modified HoloOcean engine.

Launches the modified engine (UE 5.3 ``Holodeck.uproject``) in a VISIBLE
window, attaches the HoloOcean Python client, spawns the *real* custom
anchor mesh (``/Game/ancora.ancora``) via the engine's ``SpawnAsset`` world
command, and saves RGB camera frames showing the anchor into
``visualizations/``.

Run inside the conda ``ocean`` environment (Python 3.9 + holoocean)::

    conda run --no-capture-output -n ocean python scripts/capture_custom_anchor_frame.py

Useful flags::

    --no-launch      attach to an engine window that is already running
    --keep-engine    leave the engine window open when the script exits
    --engine-config  alternate custom_holoocean_engine.yaml

The script never modifies the external engine folder.  Simulation-only:
no MAVLink, no thrusters, no real rover.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _SCRIPT_DIR.parent
_SERVER_DIR = _REPO_ROOT / "src" / "rov_obstacle_sim_bridge" / "holoocean_server"
sys.path.insert(0, str(_SERVER_DIR))

from custom_engine_launcher import (  # noqa: E402
    attach_holoocean,
    build_engine_command,
    engine_log_path,
    launch_engine,
    load_engine_config,
    stop_engine,
    validate_engine_config,
)
from custom_asset_commands import (  # noqa: E402
    enqueue_clear_spawned,
    enqueue_spawn_asset,
)

import subprocess  # noqa: E402


def ue_to_client(loc_ue):
    """UE centimetres (left-handed, +Y right) -> client metres (+Y left)."""
    return (loc_ue[0] / 100.0, -loc_ue[1] / 100.0, loc_ue[2] / 100.0)


def pick_anchor_site(population_json: Path, index: int = 0) -> dict:
    """Pick a realistic anchor site from the external population file.

    Prefers entries inside the core octree area (real seabed underneath) and
    reproduces the transforms the previous manual workflow (main.py) used:
    +1.2 m Z offset and a 2/3 scale multiplier.
    """
    data = json.loads(population_json.read_text(encoding="utf-8"))
    anchors = [s for s in data.get("spawns", []) if s.get("category") == "ancora"]
    core = [
        a for a in anchors
        if abs(a["location"][0]) < 12000 and abs(a["location"][1]) < 10000
    ]
    pool = core or anchors
    if not pool:
        raise ValueError(f"No 'ancora' entries found in {population_json}")
    entry = pool[index % len(pool)]
    loc = list(entry["location"])
    loc[2] += 120.0  # main.py POPULATION_Z_OFFSET
    scale = [v * 0.6666667 for v in entry.get("scale", [12.0, 12.0, 12.0])]
    return {
        "label": entry.get("label", "ancora"),
        "client_position": ue_to_client(loc),
        "rotation": list(entry.get("rotation", [0.0, 0.0, 0.0])),
        "scale": scale,
    }


def build_probe_scenario(agent_location, agent_yaw_deg, width, height, tps):
    return {
        "name": "custom_anchor_probe",
        "world": "ExampleLevel",
        "main_agent": "auv0",
        "ticks_per_sec": int(tps),
        # Must be present: holoocean.make() falls back to an interactive
        # input() prompt when this key is missing (hangs non-interactive runs).
        "frames_per_sec": 30,
        "agents": [
            {
                "agent_name": "auv0",
                "agent_type": "HoveringAUV",
                "sensors": [
                    {"sensor_type": "PoseSensor", "socket": "IMUSocket"},
                    {"sensor_type": "VelocitySensor", "socket": "IMUSocket"},
                    {"sensor_type": "DepthSensor", "socket": "DepthSocket"},
                    {
                        "sensor_type": "RGBCamera",
                        "sensor_name": "FrontCamera",
                        "socket": "CameraLeftSocket",
                        "configuration": {
                            "CaptureWidth": int(width),
                            "CaptureHeight": int(height),
                        },
                    },
                ],
                "control_scheme": 0,
                "location": [float(v) for v in agent_location],
                "rotation": [0.0, 0.0, float(agent_yaw_deg)],
            }
        ],
    }


def save_rgb(frame: np.ndarray, path: Path) -> None:
    # HoloOcean camera buffers are BGRA (UE FColor memory layout), despite
    # the client docstring saying RGBA: channels 2,1,0 are the true RGB.
    rgb = np.ascontiguousarray(np.array(frame)[:, :, 2::-1].astype(np.uint8))
    try:
        import cv2

        cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    except ImportError:
        from PIL import Image

        Image.fromarray(rgb).save(str(path))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine-config", default=None)
    parser.add_argument("--no-launch", action="store_true",
                        help="attach to an already-running engine window")
    parser.add_argument("--keep-engine", action="store_true",
                        help="leave the engine window running on exit")
    parser.add_argument("--anchor-site-index", type=int, default=0)
    parser.add_argument("--anchor-distance-m", type=float, default=8.0)
    parser.add_argument("--camera-width", type=int, default=512)
    parser.add_argument("--camera-height", type=int, default=512)
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--settle-ticks", type=int, default=30)
    parser.add_argument("--out-dir", default=str(_REPO_ROOT / "visualizations"))
    parser.add_argument("--prefix", default="custom_anchor_probe")
    args = parser.parse_args()

    cfg = load_engine_config(args.engine_config)
    problems = validate_engine_config(cfg, check_paths=True)
    if problems:
        print("[probe] invalid engine config:")
        for p in problems:
            print("  -", p)
        return 2

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    population = Path(str(cfg["external_engine"].get("world_population_json", "")))
    site = pick_anchor_site(population, args.anchor_site_index)
    ax, ay, az = site["client_position"]
    dist = float(args.anchor_distance_m)
    # Agent sits `dist` metres behind the anchor along -X, facing +X (yaw 0).
    agent_loc = (ax - dist, ay, az + 0.6)
    anchor_mesh = str(cfg.get("assets", {}).get("anchor_mesh", "/Game/ancora.ancora"))
    tps = int(cfg.get("launch", {}).get("ticks_per_sec", 30))

    print(f"[probe] anchor site {site['label']}: client=({ax:.2f},{ay:.2f},{az:.2f}) "
          f"scale={site['scale'][0]:.2f}")
    print(f"[probe] agent at ({agent_loc[0]:.2f},{agent_loc[1]:.2f},{agent_loc[2]:.2f}) yaw=0")

    engine_cmd = build_engine_command(cfg)
    engine_proc = None
    if not args.no_launch:
        engine_proc = launch_engine(cfg)
    else:
        print("[probe] --no-launch: expecting a running engine window")

    scenario = build_probe_scenario(
        agent_loc, 0.0, args.camera_width, args.camera_height, tps
    )

    env = None
    t0 = time.time()
    try:
        env = attach_holoocean(scenario, cfg, engine_process=engine_proc)
        print(f"[probe] attached after {time.time() - t0:.1f}s")

        # Remove any runtime actors left over from a previous client session.
        # NOTE: direct commands (CommandFactory), NOT send_world_command —
        # the blueprint path fatals on unknown names in this engine.
        enqueue_clear_spawned(env)
        env.tick()

        enqueue_spawn_asset(
            env,
            position=[ax, ay, az],
            rotation=site["rotation"],
            scale=site["scale"],
            mesh_asset=anchor_mesh,
            label="probe_anchor",
            units="meters",
        )
        print(f"[probe] SpawnAsset '{anchor_mesh}' @ client=({ax:.2f},{ay:.2f},{az:.2f}) "
              f"scale={site['scale'][0]:.2f} (units=meters)")

        saved = []
        state = None
        for i in range(max(1, args.settle_ticks)):
            state = env.tick()
        for k in range(max(1, args.frames)):
            for _ in range(5):
                state = env.tick()
            if isinstance(state, dict) and "FrontCamera" in state:
                path = out_dir / f"{args.prefix}_{k:02d}.png"
                save_rgb(state["FrontCamera"], path)
                saved.append(str(path))
                print(f"[probe] saved {path}")
            else:
                keys = list(state.keys()) if isinstance(state, dict) else type(state)
                print(f"[probe] no FrontCamera in state (keys={keys})")

        pose_info = None
        if isinstance(state, dict) and "PoseSensor" in state:
            P = np.array(state["PoseSensor"])
            pose_info = {
                "x": float(P[0, 3]), "y": float(P[1, 3]), "z": float(P[2, 3]),
                "yaw_deg": math.degrees(math.atan2(float(P[1, 0]), float(P[0, 0]))),
            }
            print(f"[probe] agent pose: {pose_info}")

        info = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "engine_command": subprocess.list2cmdline(engine_cmd),
            "engine_log": str(engine_log_path(cfg)),
            "attach_seconds": round(time.time() - t0, 1),
            "anchor_site": site,
            "anchor_mesh": anchor_mesh,
            "agent_location_client_m": list(agent_loc),
            "frames_saved": saved,
            "agent_pose": pose_info,
            "visible_window": True,
            "headless": False,
        }
        info_path = out_dir / f"{args.prefix}_info.json"
        info_path.write_text(json.dumps(info, indent=2), encoding="utf-8")
        print(f"[probe] wrote {info_path}")

        ok = bool(saved)
        print("[probe] RESULT:", "PASS" if ok else "FAIL (no camera frames)")
        return 0 if ok else 1
    finally:
        if env is not None:
            try:
                env.__exit__(None, None, None)
            except Exception:
                pass
        if engine_proc is not None and not args.keep_engine:
            print("[probe] stopping engine window")
            stop_engine(engine_proc)
        elif engine_proc is not None:
            print("[probe] leaving engine window running (--keep-engine)")


if __name__ == "__main__":
    raise SystemExit(main())
