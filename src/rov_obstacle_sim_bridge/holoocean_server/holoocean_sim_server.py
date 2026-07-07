"""HoloOcean simulation server (runs in the conda ``ocean`` Python 3.9 env).

This is the *simulator side* of the two-process bridge.  It owns the HoloOcean
environment and streams sensor data to a ROS 2 bridge node (which runs in the
separate pixi ROS 2 / Python 3.12 environment) over a localhost TCP socket.

Responsibilities:
  * load a scenario YAML (HoloOcean scenario + obstacle layout),
  * launch HoloOcean and spawn obstacle props (spheres, boxes, ...),
  * step the simulation and read camera / pose / velocity / depth,
  * apply incoming abstract velocity commands by kinematic teleport, and
  * publish state + ground-truth obstacle positions back over the socket.

It deliberately performs NO ROS 2 imports.  The only non-stdlib dependencies
are ``holoocean``, ``numpy`` and ``pyyaml`` (all present in the ``ocean`` env).

This server NEVER talks to a real ROV, thrusters or MAVLink.  Vehicle motion is
a kinematic teleport convenience for simulation only.

Usage::

    conda run -n ocean python holoocean_sim_server.py --config <scenario.yaml> --serve
    conda run -n ocean python holoocean_sim_server.py --config <scenario.yaml> --selftest
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

try:
    import yaml
except ImportError as exc:  # pragma: no cover - ocean env always has pyyaml
    raise SystemExit("PyYAML is required: pip install pyyaml") from exc

# Make the sibling pure-Python package importable so we can reuse the wire
# protocol without duplicating it.  The protocol module imports only stdlib.
_PKG_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

from rov_obstacle_sim_bridge.sim_bridge_protocol import (  # noqa: E402
    DEFAULT_HOST,
    DEFAULT_PORT,
    MSG_CMD_VEL,
    MSG_STATE,
    FrameStream,
    coerce_float,
)

# ---------------------------------------------------------------------------
# Coordinate convention (calibrated against HoloOcean OpenWater + teleport).
#
# HoloOcean uses a right-handed, ROS REP-103 world frame:
#     +X = forward, +Y = LEFT, +Z = up.
# Verified empirically: facing +X (yaw=0), a sphere at world +Y renders on the
# LEFT side of the RGB image, and teleport yaw=+90 deg turns the camera toward
# +Y. yaw is extracted as atan2(R[1,0], R[0,0]) from the 4x4 PoseSensor matrix.
#
# We maintain the kinematic rover pose directly in this frame, using the
# standard 2D rotation for body->world (lateral is +left):
#   body +X (surge)  -> world [cos(yaw),  sin(yaw), 0]
#   body +Y (lateral)-> world [-sin(yaw), cos(yaw), 0]   (+lateral = +left)
#   body +Z (heave)  -> world +Z
#
# Scenario obstacle ``relative_position`` is [forward_m, left_m, up_m].
# The ROS 2 bridge negates y/yaw when projecting through oracle_geometry, which
# uses the opposite (+y = right) convention.
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

@dataclass
class ObstacleSpec:
    name: str
    class_name: str
    prop_type: str            # box | sphere | cylinder | cone
    relative_position: tuple[float, float, float]  # forward, left, up (m)
    radius_m: float
    scale: Any = 1.0          # float or [x, y, z]
    material: str = "gold"
    sim_physics: bool = False
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)  # roll, pitch, yaw deg


@dataclass
class PrimitivePartSpec:
    name: str
    prop_type: str            # box | sphere | cylinder | cone
    relative_position: tuple[float, float, float]  # forward, left, up (m)
    radius_m: float
    scale: Any = 1.0          # float or [x, y, z]
    material: str = "gold"
    sim_physics: bool = False
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)  # roll, pitch, yaw deg
    class_name: Optional[str] = None


@dataclass
class SemanticObjectSpec:
    name: str
    class_name: str
    relative_position: tuple[float, float, float]  # forward, left, up (m)
    parts: list[PrimitivePartSpec]
    radius_m: Optional[float] = None
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)  # roll, pitch, yaw deg


@dataclass
class SpawnedPrimitive:
    name: str
    class_name: str
    prop_type: str
    position: tuple[float, float, float]
    radius_m: float
    scale: Any = 1.0
    material: str = "gold"
    sim_physics: bool = False
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    semantic_parent: Optional[str] = None
    bounds: Optional[dict[str, list[float]]] = None


@dataclass
class CustomAssetSpec:
    """A real Unreal mesh asset from the EXTERNAL modified engine.

    Spawned at runtime through the engine's custom ``SpawnAsset`` world
    command (or assumed to already exist in the world when
    ``spawned_at_runtime`` is false).  The oracle never tries to visually
    detect the mesh: detections come from the configured position/bounds.
    """

    name: str
    class_name: str
    mesh_asset: str                     # e.g. /Game/ancora.ancora
    relative_position: Optional[tuple[float, float, float]] = None  # fwd,left,up (m)
    absolute_position: Optional[tuple[float, float, float]] = None  # world client m
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)  # roll, pitch, yaw deg
    scale: Any = 1.0                    # float or [x, y, z] (Unreal actor scale)
    radius_m: float = 1.5               # oracle bounding radius
    half_extents_m: Optional[tuple[float, float, float]] = None  # oracle box half sizes
    spawned_at_runtime: bool = True


@dataclass
class CustomEngineSpec:
    """Settings for attaching to the EXTERNAL modified HoloOcean engine."""

    enabled: bool = False
    engine_config: str = ""             # path to custom_holoocean_engine.yaml
    world: str = "ExampleLevel"
    auto_launch: bool = True            # launch the visible engine window ourselves
    stop_engine_on_exit: bool = True
    agent_type: str = "HoveringAUV"
    agent_location: tuple[float, float, float] = (0.0, 0.0, -20.0)  # client m
    agent_yaw_deg: float = 0.0
    camera_socket: str = "CameraLeftSocket"
    clear_spawned_on_start: bool = True


@dataclass
class SimConfig:
    scenario: str = "OpenWater-HoveringCamera"
    agent_name: str = "auv0"
    camera_sensor: str = "LeftCamera"
    ticks_per_sec: int = 30
    frames_per_sec: Any = False
    show_viewport: bool = False
    motion_model: str = "teleport"   # teleport | hold
    start_offset: tuple[float, float, float] = (0.0, 0.0, 5.0)  # fwd, right, up
    camera_width: int = 512
    camera_height: int = 512
    horizontal_fov_deg: float = 90.0
    vertical_fov_deg: float = 60.0
    max_surge: float = 1.5
    max_sway: float = 1.5
    max_heave: float = 1.0
    max_yaw_rate: float = 0.8
    obstacles: list[ObstacleSpec] = field(default_factory=list)
    semantic_objects: list[SemanticObjectSpec] = field(default_factory=list)
    oracle_debug_primitive_detections: bool = False
    custom_engine: Optional[CustomEngineSpec] = None
    custom_assets: list[CustomAssetSpec] = field(default_factory=list)
    config_dir: str = ""             # directory of the scenario YAML (for relpaths)


def load_config(path: str) -> SimConfig:
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}

    ho = data.get("holoocean", {})
    sim = data.get("sim", {})
    cam = data.get("camera", {})
    limits = data.get("limits", {})
    oracle = data.get("oracle", {})

    obstacles: list[ObstacleSpec] = []
    for entry in data.get("obstacles", []) or []:
        obstacles.append(
            ObstacleSpec(
                name=str(entry["name"]),
                class_name=str(entry.get("class_name", "unknown_obstacle")),
                prop_type=str(entry.get("prop_type", "sphere")),
                relative_position=tuple(float(v) for v in entry["relative_position"]),
                radius_m=float(entry.get("radius_m", 1.0)),
                scale=entry.get("scale", 1.0),
                material=str(entry.get("material", "gold")),
                sim_physics=bool(entry.get("sim_physics", False)),
                rotation=tuple(float(v) for v in entry.get("rotation", [0.0, 0.0, 0.0])),
            )
        )

    semantic_objects: list[SemanticObjectSpec] = []
    for entry in data.get("semantic_objects", []) or []:
        name = str(entry["name"])
        class_name = str(entry.get("class_name", "unknown_obstacle"))
        raw_parts = entry.get("parts", []) or []
        if not raw_parts:
            raise ValueError(f"semantic object '{name}' must define at least one part")
        parts = [
            _load_part_spec(
                raw_part,
                default_name=f"part_{idx}",
                default_class_name=f"{class_name}_part",
            )
            for idx, raw_part in enumerate(raw_parts)
        ]
        semantic_objects.append(
            SemanticObjectSpec(
                name=name,
                class_name=class_name,
                relative_position=_float3(
                    entry.get("relative_position", [0.0, 0.0, 0.0]),
                    f"semantic_objects.{name}.relative_position",
                ),
                radius_m=(
                    float(entry["radius_m"])
                    if entry.get("radius_m") is not None
                    else None
                ),
                rotation=_float3(
                    entry.get("rotation", [0.0, 0.0, 0.0]),
                    f"semantic_objects.{name}.rotation",
                ),
                parts=parts,
            )
        )

    custom_engine = _load_custom_engine_spec(data.get("custom_engine"))
    custom_assets = _load_custom_asset_specs(data.get("custom_assets"))
    if custom_assets and (custom_engine is None or not custom_engine.enabled):
        raise ValueError(
            "custom_assets require a 'custom_engine' section with enabled: true "
            "(the SpawnAsset world command only exists in the modified engine)"
        )

    return SimConfig(
        scenario=str(ho.get("scenario", "OpenWater-HoveringCamera")),
        agent_name=str(ho.get("agent_name", "auv0")),
        camera_sensor=str(ho.get("camera_sensor", "LeftCamera")),
        ticks_per_sec=int(ho.get("ticks_per_sec", 30)),
        frames_per_sec=ho.get("frames_per_sec", False),
        show_viewport=bool(ho.get("show_viewport", False)),
        motion_model=str(sim.get("motion_model", "teleport")),
        start_offset=tuple(float(v) for v in sim.get("start_offset", [0.0, 0.0, 5.0])),
        camera_width=int(cam.get("width", 512)),
        camera_height=int(cam.get("height", 512)),
        horizontal_fov_deg=float(cam.get("horizontal_fov_deg", 90.0)),
        vertical_fov_deg=float(cam.get("vertical_fov_deg", 60.0)),
        max_surge=float(limits.get("max_surge", 1.5)),
        max_sway=float(limits.get("max_sway", 1.5)),
        max_heave=float(limits.get("max_heave", 1.0)),
        max_yaw_rate=float(limits.get("max_yaw_rate", 0.8)),
        obstacles=obstacles,
        semantic_objects=semantic_objects,
        oracle_debug_primitive_detections=bool(
            oracle.get("debug_primitive_detections", False)
        ),
        custom_engine=custom_engine,
        custom_assets=custom_assets,
        config_dir=os.path.dirname(os.path.abspath(path)),
    )


def _load_custom_engine_spec(raw: Any) -> Optional[CustomEngineSpec]:
    if not raw:
        return None
    if not isinstance(raw, dict):
        raise ValueError("'custom_engine' must be a mapping")
    return CustomEngineSpec(
        enabled=bool(raw.get("enabled", False)),
        engine_config=str(raw.get("engine_config", "") or ""),
        world=str(raw.get("world", "ExampleLevel")),
        auto_launch=bool(raw.get("auto_launch", True)),
        stop_engine_on_exit=bool(raw.get("stop_engine_on_exit", True)),
        agent_type=str(raw.get("agent_type", "HoveringAUV")),
        agent_location=_float3(
            raw.get("agent_location", [0.0, 0.0, -20.0]),
            "custom_engine.agent_location",
        ),
        agent_yaw_deg=float(raw.get("agent_yaw_deg", 0.0)),
        camera_socket=str(raw.get("camera_socket", "CameraLeftSocket")),
        clear_spawned_on_start=bool(raw.get("clear_spawned_on_start", True)),
    )


def _load_custom_asset_specs(raw: Any) -> list[CustomAssetSpec]:
    specs: list[CustomAssetSpec] = []
    for entry in raw or []:
        name = str(entry["name"])
        mesh_asset = str(entry.get("mesh_asset", "") or "")
        if not mesh_asset.startswith("/"):
            raise ValueError(
                f"custom_assets.{name}.mesh_asset must be an Unreal path "
                f"like /Game/ancora.ancora (got {mesh_asset!r})"
            )
        relative = entry.get("relative_position")
        absolute = entry.get("absolute_position")
        if relative is None and absolute is None:
            raise ValueError(
                f"custom_assets.{name} needs relative_position or absolute_position"
            )
        spawned = bool(entry.get("spawned_at_runtime", True))
        if not spawned and absolute is None:
            raise ValueError(
                f"custom_assets.{name}: static world assets "
                "(spawned_at_runtime: false) require absolute_position"
            )
        half = entry.get("half_extents_m")
        specs.append(
            CustomAssetSpec(
                name=name,
                class_name=str(entry.get("class_name", "unknown_obstacle")),
                mesh_asset=mesh_asset,
                relative_position=(
                    _float3(relative, f"custom_assets.{name}.relative_position")
                    if relative is not None
                    else None
                ),
                absolute_position=(
                    _float3(absolute, f"custom_assets.{name}.absolute_position")
                    if absolute is not None
                    else None
                ),
                rotation=_float3(
                    entry.get("rotation", [0.0, 0.0, 0.0]),
                    f"custom_assets.{name}.rotation",
                ),
                scale=entry.get("scale", 1.0),
                radius_m=float(entry.get("radius_m", 1.5)),
                half_extents_m=(
                    _float3(half, f"custom_assets.{name}.half_extents_m")
                    if half is not None
                    else None
                ),
                spawned_at_runtime=spawned,
            )
        )
    return specs


def _load_part_spec(
    entry: dict,
    *,
    default_name: str,
    default_class_name: str,
) -> PrimitivePartSpec:
    return PrimitivePartSpec(
        name=str(entry.get("name", default_name)),
        class_name=str(entry.get("class_name", default_class_name)),
        prop_type=str(entry.get("prop_type", "box")),
        relative_position=_float3(
            entry.get("relative_position", [0.0, 0.0, 0.0]),
            f"parts.{entry.get('name', default_name)}.relative_position",
        ),
        radius_m=float(entry.get("radius_m", 0.5)),
        scale=entry.get("scale", 1.0),
        material=str(entry.get("material", "gold")),
        sim_physics=bool(entry.get("sim_physics", False)),
        rotation=_float3(
            entry.get("rotation", [0.0, 0.0, 0.0]),
            f"parts.{entry.get('name', default_name)}.rotation",
        ),
    )


def _float3(values: Any, field_name: str) -> tuple[float, float, float]:
    try:
        if len(values) != 3:
            raise ValueError("expected exactly three values")
        return (float(values[0]), float(values[1]), float(values[2]))
    except Exception as exc:
        raise ValueError(f"{field_name} must be [forward_m, left_m, up_m]") from exc


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def yaw_from_pose_matrix(P: np.ndarray) -> float:
    return math.atan2(float(P[1, 0]), float(P[0, 0]))


def body_to_world(forward: float, left: float, up: float,
                  yaw_rad: float) -> tuple[float, float, float]:
    """Map a body-frame offset (forward, left, up) into a world delta.

    Standard 2D rotation about +Z; ``left`` is +Y in the REP-103 world frame.
    """
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    dx = forward * cos_y - left * sin_y
    dy = forward * sin_y + left * cos_y
    dz = up
    return dx, dy, dz


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(v, hi))


def _add3(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _matvec3(
    m: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def _matmul3(
    a: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    b: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )


def _euler_matrix_deg(
    rotation_deg: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    roll, pitch, yaw = (math.radians(v) for v in rotation_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return _matmul3(rz, _matmul3(ry, rx))


def _combined_rotation_deg(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _spawn_rotation_deg(
    local_rotation: tuple[float, float, float],
    rover_yaw_rad: float,
) -> tuple[float, float, float]:
    return (
        local_rotation[0],
        local_rotation[1],
        local_rotation[2] + math.degrees(rover_yaw_rad),
    )


def _scale_vector(scale: Any) -> tuple[float, float, float]:
    if isinstance(scale, (list, tuple)):
        if len(scale) == 3:
            return (abs(float(scale[0])), abs(float(scale[1])), abs(float(scale[2])))
        if len(scale) == 1:
            value = abs(float(scale[0]))
            return (value, value, value)
    value = abs(float(scale))
    return (value, value, value)


def _half_extents(part: PrimitivePartSpec) -> tuple[float, float, float]:
    if part.prop_type.strip().lower() == "sphere":
        radius = max(0.01, float(part.radius_m))
        return (radius, radius, radius)

    sx, sy, sz = _scale_vector(part.scale)
    half = (max(0.01, sx * 0.5), max(0.01, sy * 0.5), max(0.01, sz * 0.5))
    if half == (0.01, 0.01, 0.01):
        radius = max(0.01, float(part.radius_m))
        return (radius, radius, radius)
    return half


def _relative_to_world(
    origin: tuple[float, float, float],
    yaw_rad: float,
    relative: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx, dy, dz = body_to_world(relative[0], relative[1], relative[2], yaw_rad)
    return (origin[0] + dx, origin[1] + dy, origin[2] + dz)


def _bounds_from_points(points: list[tuple[float, float, float]]) -> dict[str, list[float]]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return {
        "min": [min(xs), min(ys), min(zs)],
        "max": [max(xs), max(ys), max(zs)],
    }


def _center_from_bounds(bounds: dict[str, list[float]]) -> tuple[float, float, float]:
    mn = bounds["min"]
    mx = bounds["max"]
    return (
        (mn[0] + mx[0]) * 0.5,
        (mn[1] + mx[1]) * 0.5,
        (mn[2] + mx[2]) * 0.5,
    )


def _radius_from_bounds(bounds: dict[str, list[float]]) -> float:
    mn = bounds["min"]
    mx = bounds["max"]
    return 0.5 * math.sqrt(
        (mx[0] - mn[0]) ** 2
        + (mx[1] - mn[1]) ** 2
        + (mx[2] - mn[2]) ** 2
    )


def _part_body_corners(
    object_spec: SemanticObjectSpec,
    part: PrimitivePartSpec,
) -> tuple[tuple[float, float, float], list[tuple[float, float, float]]]:
    object_rotation = _euler_matrix_deg(object_spec.rotation)
    part_offset = _matvec3(object_rotation, part.relative_position)
    part_center = _add3(object_spec.relative_position, part_offset)
    part_rotation = _euler_matrix_deg(
        _combined_rotation_deg(object_spec.rotation, part.rotation)
    )
    hx, hy, hz = _half_extents(part)
    corners: list[tuple[float, float, float]] = []
    for x in (-hx, hx):
        for y in (-hy, hy):
            for z in (-hz, hz):
                rotated_corner = _matvec3(part_rotation, (x, y, z))
                corners.append(_add3(part_center, rotated_corner))
    return part_center, corners


def _oracle_entry(
    *,
    name: str,
    class_name: str,
    position: tuple[float, float, float],
    radius_m: float,
    bounds: Optional[dict[str, list[float]]] = None,
    part_count: Optional[int] = None,
) -> dict[str, Any]:
    entry: dict[str, Any] = {
        "name": name,
        "class_name": class_name,
        "position": [position[0], position[1], position[2]],
        "radius_m": float(radius_m),
    }
    if bounds is not None:
        entry["bounds"] = bounds
    if part_count is not None:
        entry["part_count"] = int(part_count)
    return entry


def build_spawn_plan(
    config: SimConfig,
    sx: float,
    sy: float,
    sz: float,
    syaw: float,
) -> tuple[list[SpawnedPrimitive], list[dict[str, Any]]]:
    """Return primitive HoloOcean spawns and semantic oracle objects.

    The helper is dependency-free except for stdlib math, so unit tests can
    validate YAML semantics without importing HoloOcean.
    """
    origin = (sx, sy, sz)
    spawns: list[SpawnedPrimitive] = []
    oracle_obstacles: list[dict[str, Any]] = []

    for spec in config.obstacles:
        position = _relative_to_world(origin, syaw, spec.relative_position)
        spawns.append(
            SpawnedPrimitive(
                name=spec.name,
                class_name=spec.class_name,
                prop_type=spec.prop_type,
                position=position,
                radius_m=spec.radius_m,
                scale=spec.scale,
                material=spec.material,
                sim_physics=spec.sim_physics,
                rotation=spec.rotation,
            )
        )
        oracle_obstacles.append(
            _oracle_entry(
                name=spec.name,
                class_name=spec.class_name,
                position=position,
                radius_m=spec.radius_m,
            )
        )

    for semantic in config.semantic_objects:
        semantic_spawns: list[SpawnedPrimitive] = []
        all_corners_world: list[tuple[float, float, float]] = []

        for part in semantic.parts:
            part_center_body, part_corners_body = _part_body_corners(semantic, part)
            part_position = _relative_to_world(origin, syaw, part_center_body)
            part_corners_world = [
                _relative_to_world(origin, syaw, corner)
                for corner in part_corners_body
            ]
            bounds = _bounds_from_points(part_corners_world)
            spawn_name = (
                part.name
                if part.name.startswith(f"{semantic.name}_")
                else f"{semantic.name}_{part.name}"
            )
            local_rotation = _combined_rotation_deg(semantic.rotation, part.rotation)
            spawn = SpawnedPrimitive(
                name=spawn_name,
                class_name=part.class_name or f"{semantic.class_name}_part",
                prop_type=part.prop_type,
                position=part_position,
                radius_m=part.radius_m,
                scale=part.scale,
                material=part.material,
                sim_physics=part.sim_physics,
                rotation=_spawn_rotation_deg(local_rotation, syaw),
                semantic_parent=semantic.name,
                bounds=bounds,
            )
            spawns.append(spawn)
            semantic_spawns.append(spawn)
            all_corners_world.extend(part_corners_world)

        aggregate_bounds = _bounds_from_points(all_corners_world)
        aggregate_center = _center_from_bounds(aggregate_bounds)
        aggregate_radius = (
            semantic.radius_m
            if semantic.radius_m is not None
            else _radius_from_bounds(aggregate_bounds)
        )
        oracle_obstacles.append(
            _oracle_entry(
                name=semantic.name,
                class_name=semantic.class_name,
                position=aggregate_center,
                radius_m=aggregate_radius,
                bounds=aggregate_bounds,
                part_count=len(semantic_spawns),
            )
        )

        if config.oracle_debug_primitive_detections:
            for spawn in semantic_spawns:
                oracle_obstacles.append(
                    _oracle_entry(
                        name=spawn.name,
                        class_name=spawn.class_name,
                        position=spawn.position,
                        radius_m=spawn.radius_m,
                        bounds=spawn.bounds,
                    )
                )

    return spawns, oracle_obstacles


@dataclass
class CustomAssetSpawn:
    """A resolved SpawnAsset call (client metres, client RPY degrees)."""

    name: str
    class_name: str
    mesh_asset: str
    position: tuple[float, float, float]
    rotation: tuple[float, float, float]
    scale: tuple[float, float, float]
    spawned_at_runtime: bool


def build_custom_asset_plan(
    config: SimConfig,
    sx: float,
    sy: float,
    sz: float,
    syaw: float,
) -> tuple[list[CustomAssetSpawn], list[dict[str, Any]]]:
    """Resolve custom mesh assets to world poses and oracle entries.

    Mirrors :func:`build_spawn_plan` for real Unreal assets from the external
    modified engine.  ``relative_position`` entries are placed relative to the
    rover's working pose (like primitives); ``absolute_position`` entries stay
    fixed in the world (e.g. anchors that conceptually belong to the map).
    The oracle entry uses the configured ``radius_m``/``half_extents_m`` —
    ground truth comes from configuration, never from rendering.
    """
    origin = (sx, sy, sz)
    plan: list[CustomAssetSpawn] = []
    oracle_entries: list[dict[str, Any]] = []

    for spec in config.custom_assets:
        if spec.absolute_position is not None:
            position = spec.absolute_position
            rotation = spec.rotation
        else:
            position = _relative_to_world(origin, syaw, spec.relative_position)
            rotation = _spawn_rotation_deg(spec.rotation, syaw)

        bounds: Optional[dict[str, list[float]]] = None
        if spec.half_extents_m is not None:
            rot_matrix = _euler_matrix_deg(rotation)
            hx, hy, hz = (
                max(0.01, abs(spec.half_extents_m[0])),
                max(0.01, abs(spec.half_extents_m[1])),
                max(0.01, abs(spec.half_extents_m[2])),
            )
            corners = []
            for cx in (-hx, hx):
                for cy in (-hy, hy):
                    for cz in (-hz, hz):
                        corners.append(_add3(position, _matvec3(rot_matrix, (cx, cy, cz))))
            bounds = _bounds_from_points(corners)

        plan.append(
            CustomAssetSpawn(
                name=spec.name,
                class_name=spec.class_name,
                mesh_asset=spec.mesh_asset,
                position=position,
                rotation=rotation,
                scale=_scale_vector(spec.scale),
                spawned_at_runtime=spec.spawned_at_runtime,
            )
        )
        oracle_entries.append(
            _oracle_entry(
                name=spec.name,
                class_name=spec.class_name,
                position=position,
                radius_m=spec.radius_m,
                bounds=bounds,
            )
        )

    return plan, oracle_entries


def build_custom_scenario_cfg(config: SimConfig) -> dict[str, Any]:
    """HoloOcean ``scenario_cfg`` dict for attaching to the modified engine.

    Pure helper (no holoocean import) so unit tests can validate it.  The
    camera sensor is named after ``config.camera_sensor`` so the rest of the
    server (``step()``) works unchanged in custom-engine mode.
    """
    spec = config.custom_engine
    if spec is None or not spec.enabled:
        raise ValueError("build_custom_scenario_cfg requires an enabled custom_engine")
    return {
        "name": "rov_obstacle_custom_engine",
        "world": spec.world,
        "main_agent": config.agent_name,
        "ticks_per_sec": int(config.ticks_per_sec),
        # Explicit value required: holoocean.make() falls back to an
        # interactive input() prompt when the key is missing.
        "frames_per_sec": (
            int(config.frames_per_sec) if config.frames_per_sec else False
        ),
        "agents": [
            {
                "agent_name": config.agent_name,
                "agent_type": spec.agent_type,
                "sensors": [
                    {"sensor_type": "PoseSensor", "socket": "IMUSocket"},
                    {"sensor_type": "VelocitySensor", "socket": "IMUSocket"},
                    {"sensor_type": "DepthSensor", "socket": "DepthSocket"},
                    {
                        "sensor_type": "RGBCamera",
                        "sensor_name": config.camera_sensor,
                        "socket": spec.camera_socket,
                        "configuration": {
                            "CaptureWidth": int(config.camera_width),
                            "CaptureHeight": int(config.camera_height),
                        },
                    },
                ],
                "control_scheme": 0,
                "location": [float(v) for v in spec.agent_location],
                "rotation": [0.0, 0.0, float(spec.agent_yaw_deg)],
            }
        ],
    }


# ---------------------------------------------------------------------------
# Sim server
# ---------------------------------------------------------------------------

class HolooceanSimServer:
    def __init__(self, config: SimConfig, verbose: bool = True) -> None:
        self.cfg = config
        self.verbose = verbose
        self.env: Any = None
        self.agent: Any = None
        self._engine_proc: Any = None       # external engine process (if we launched it)
        self._engine_launcher: Any = None   # lazily imported custom_engine_launcher

        # Kinematic pose state (world frame).
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # Latest commanded body velocity.
        self.cmd = dict(surge=0.0, sway=0.0, heave=0.0,
                        roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0)

        # World positions of spawned obstacles (filled after spawn).
        self.obstacle_world: list[dict] = []
        self._seq = 0
        self._dt = 1.0 / max(1, self.cfg.ticks_per_sec)

    def log(self, *args: Any) -> None:
        if self.verbose:
            print("[sim-server]", *args, flush=True)

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        t0 = time.time()
        if self.cfg.custom_engine is not None and self.cfg.custom_engine.enabled:
            self._start_custom_engine()
        else:
            import holoocean
            self.log(f"make({self.cfg.scenario}) ...")
            self.env = holoocean.make(
                self.cfg.scenario,
                show_viewport=self.cfg.show_viewport,
                ticks_per_sec=self.cfg.ticks_per_sec,
                frames_per_sec=self.cfg.frames_per_sec,
            )
        self.agent = self.env.agents[self.cfg.agent_name]
        self.log(f"env ready in {time.time() - t0:.1f}s")

        # Read spawn pose -> kinematic origin.
        state = self.env.tick()
        P = np.array(state["PoseSensor"])
        spawn_x, spawn_y, spawn_z = float(P[0, 3]), float(P[1, 3]), float(P[2, 3])
        spawn_yaw = yaw_from_pose_matrix(P)
        self.log(f"spawn world=({spawn_x:.1f},{spawn_y:.1f},{spawn_z:.1f}) "
                 f"yaw={math.degrees(spawn_yaw):.1f} deg")

        # Apply configured start offset (in body frame of the spawn yaw).
        off = self.cfg.start_offset
        ox, oy, oz = body_to_world(off[0], off[1], off[2], spawn_yaw)
        self.x, self.y, self.z = spawn_x + ox, spawn_y + oy, spawn_z + oz
        self.yaw = spawn_yaw
        if self.cfg.motion_model != "hold":
            self.agent.teleport(
                location=np.array([self.x, self.y, self.z]),
                rotation=np.array([0.0, 0.0, math.degrees(self.yaw)]),
            )
            for _ in range(5):
                self.env.tick()

        # Obstacles are placed relative to the rover's WORKING pose (after the
        # start offset), so they share the rover's height and sit in front of
        # the camera regardless of the raw spawn location.
        self._spawn_obstacles(self.x, self.y, self.z, self.yaw)

    def _start_custom_engine(self) -> None:
        """Attach to the EXTERNAL modified engine (launching it if needed).

        The engine is an UE editor project run in visible ``-game`` mode; the
        HoloOcean client attaches through shared memory instead of launching a
        packaged binary.  The external folder is used strictly read-only.
        """
        spec = self.cfg.custom_engine
        here = os.path.dirname(os.path.abspath(__file__))
        if here not in sys.path:
            sys.path.insert(0, here)
        import custom_engine_launcher as cel
        self._engine_launcher = cel

        engine_config_path = spec.engine_config or None
        if engine_config_path and not os.path.isabs(engine_config_path):
            engine_config_path = os.path.normpath(
                os.path.join(self.cfg.config_dir, engine_config_path)
            )
        engine_cfg = cel.load_engine_config(engine_config_path)
        problems = cel.validate_engine_config(engine_cfg, check_paths=True)
        if problems:
            raise RuntimeError(
                "External engine not usable:\n  - " + "\n  - ".join(problems)
            )

        if spec.auto_launch:
            self.log(f"launching external engine (world={spec.world}, visible)")
            self._engine_proc = cel.launch_engine(engine_cfg, map_name=spec.world)
        else:
            self.log("expecting an already-running external engine window")

        scenario_cfg = build_custom_scenario_cfg(self.cfg)
        self.log(f"attaching to '{spec.world}' as agent "
                 f"'{self.cfg.agent_name}' ({spec.agent_type})")
        self.env = cel.attach_holoocean(
            scenario_cfg,
            engine_cfg,
            engine_process=self._engine_proc,
            verbose=self.verbose,
        )
        if spec.clear_spawned_on_start:
            # Direct command via CommandFactory; send_world_command would
            # fatal the engine (unknown blueprint custom command).
            import custom_asset_commands as cac
            cac.enqueue_clear_spawned(self.env)
            self.env.tick()

    def _spawn_obstacles(self, sx: float, sy: float, sz: float,
                         syaw: float) -> None:
        spawn_plan, oracle_obstacles = build_spawn_plan(self.cfg, sx, sy, sz, syaw)
        for primitive in spawn_plan:
            wx, wy, wz = primitive.position
            try:
                self.env.spawn_prop(
                    prop_type=primitive.prop_type,
                    location=[wx, wy, wz],
                    rotation=list(primitive.rotation),
                    scale=primitive.scale,
                    sim_physics=primitive.sim_physics,
                    material=primitive.material,
                    tag=primitive.name,
                )
                parent = (
                    f" parent={primitive.semantic_parent}"
                    if primitive.semantic_parent
                    else ""
                )
                self.log(f"spawned {primitive.prop_type} '{primitive.name}' "
                         f"({primitive.class_name}) at world=({wx:.1f},{wy:.1f},{wz:.1f})"
                         f"{parent}")
            except Exception as exc:  # pragma: no cover - depends on engine
                self.log(f"WARNING: failed to spawn '{primitive.name}': {exc!r}")

        asset_plan, asset_oracle = build_custom_asset_plan(self.cfg, sx, sy, sz, syaw)
        if asset_plan:
            here = os.path.dirname(os.path.abspath(__file__))
            if here not in sys.path:
                sys.path.insert(0, here)
        spawned_any_asset = False
        for asset in asset_plan:
            if not asset.spawned_at_runtime:
                self.log(f"custom asset '{asset.name}' assumed static in world "
                         f"at ({asset.position[0]:.1f},{asset.position[1]:.1f},"
                         f"{asset.position[2]:.1f})")
                continue
            wx, wy, wz = asset.position
            try:
                import custom_asset_commands as cac
                cac.enqueue_spawn_asset(
                    self.env,
                    position=[wx, wy, wz],
                    rotation=list(asset.rotation),
                    scale=list(asset.scale),
                    mesh_asset=asset.mesh_asset,
                    label=asset.name,
                    units="meters",
                )
                spawned_any_asset = True
                self.log(f"spawned custom asset '{asset.name}' "
                         f"({asset.class_name}, {asset.mesh_asset}) at "
                         f"world=({wx:.1f},{wy:.1f},{wz:.1f}) "
                         f"scale={asset.scale[0]:.2f}")
            except Exception as exc:  # pragma: no cover - depends on engine
                self.log(f"WARNING: failed to spawn custom asset "
                         f"'{asset.name}': {exc!r}")
        if spawned_any_asset:
            # Commands execute on the next engine tick.
            self.env.tick()

        self.obstacle_world = oracle_obstacles + asset_oracle
        self.log(f"oracle semantic obstacles: {len(self.obstacle_world)}")

    def close(self) -> None:
        if self.env is not None:
            try:
                self.env.__exit__(None, None, None)
            except Exception:
                pass
            self.env = None
        if self._engine_proc is not None:
            spec = self.cfg.custom_engine
            if spec is not None and spec.stop_engine_on_exit:
                self.log("stopping external engine window")
                try:
                    self._engine_launcher.stop_engine(self._engine_proc)
                except Exception:
                    pass
            self._engine_proc = None

    # -- per-tick update -----------------------------------------------------
    def apply_command(self, header: dict) -> None:
        c = self.cfg
        self.cmd = dict(
            surge=clamp(coerce_float(header.get("surge")), -c.max_surge, c.max_surge),
            sway=clamp(coerce_float(header.get("sway")), -c.max_sway, c.max_sway),
            heave=clamp(coerce_float(header.get("heave")), -c.max_heave, c.max_heave),
            roll_rate=coerce_float(header.get("roll_rate")),
            pitch_rate=coerce_float(header.get("pitch_rate")),
            yaw_rate=clamp(coerce_float(header.get("yaw_rate")),
                           -c.max_yaw_rate, c.max_yaw_rate),
        )

    def _integrate_and_teleport(self) -> None:
        if self.cfg.motion_model == "hold":
            return
        self.yaw += self.cmd["yaw_rate"] * self._dt
        dx, dy, dz = body_to_world(
            self.cmd["surge"] * self._dt,
            self.cmd["sway"] * self._dt,
            self.cmd["heave"] * self._dt,
            self.yaw,
        )
        self.x += dx
        self.y += dy
        self.z += dz
        self.agent.teleport(
            location=np.array([self.x, self.y, self.z]),
            rotation=np.array([0.0, 0.0, math.degrees(self.yaw)]),
        )

    def step(self) -> tuple[dict, bytes]:
        """Advance one tick and return a (state_header, image_blob) pair."""
        self._integrate_and_teleport()
        state = self.env.tick()

        # Pose / velocity / depth from sensors when present, else kinematic.
        depth = None
        velocity = [0.0, 0.0, 0.0]
        if isinstance(state, dict):
            if "DepthSensor" in state:
                depth = float(np.array(state["DepthSensor"]).reshape(-1)[0])
            if "VelocitySensor" in state:
                v = np.array(state["VelocitySensor"]).reshape(-1)
                velocity = [float(v[0]), float(v[1]), float(v[2])]

        image_blob = b""
        image_meta = None
        cam_key = self.cfg.camera_sensor
        if isinstance(state, dict) and cam_key in state:
            frame = np.array(state[cam_key])
            # HoloOcean camera buffers are BGRA in memory (UE FColor layout;
            # the client docstring says "RGBA" but the engine fills FColor*),
            # so channels 2,1,0 give true RGB for the advertised rgb8 encoding.
            rgb = np.ascontiguousarray(frame[:, :, 2::-1].astype(np.uint8))
            image_blob = rgb.tobytes()
            image_meta = {
                "present": True,
                "height": int(rgb.shape[0]),
                "width": int(rgb.shape[1]),
                "encoding": "rgb8",
                "step": int(rgb.shape[1] * 3),
            }

        self._seq += 1
        header = {
            "type": MSG_STATE,
            "seq": self._seq,
            "t": float(state.get("t", time.time())) if isinstance(state, dict) else time.time(),
            "pose": {"x": self.x, "y": self.y, "z": self.z, "yaw": self.yaw},
            "velocity": {"x": velocity[0], "y": velocity[1], "z": velocity[2]},
            "depth": depth if depth is not None else self.z,
            "camera": {
                "horizontal_fov_deg": self.cfg.horizontal_fov_deg,
                "vertical_fov_deg": self.cfg.vertical_fov_deg,
            },
            "image": image_meta,
            "obstacles": self.obstacle_world,
        }
        return header, image_blob

    # -- run modes -----------------------------------------------------------
    def serve(self, host: str, port: int) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(1)
        srv.settimeout(1.0)
        self.log(f"listening on {host}:{port} (waiting for ROS 2 bridge)")
        try:
            while True:
                stream = self._accept(srv)
                if stream is None:
                    continue
                self.log("bridge connected")
                self._serve_client(stream)
                self.log("bridge disconnected; waiting for reconnect")
        except KeyboardInterrupt:
            self.log("interrupted")
        finally:
            srv.close()

    def _accept(self, srv: socket.socket) -> Optional[FrameStream]:
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            # Keep the engine alive while waiting for a client.
            try:
                self.env.tick()
            except Exception:
                pass
            return None
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return FrameStream(conn)

    def _serve_client(self, stream: FrameStream) -> None:
        period = self._dt if not self.cfg.frames_per_sec else 0.0
        while not stream.closed:
            loop_start = time.time()
            # Drain commands, keep only the latest.
            try:
                latest = stream.read_latest()
            except ConnectionError:
                break
            if latest is not None:
                header, _ = latest
                if header.get("type") == MSG_CMD_VEL:
                    self.apply_command(header)

            header, blob = self.step()
            try:
                stream.send(header, blob)
            except ConnectionError:
                break

            if period > 0.0:
                elapsed = time.time() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)
        stream.close()

    def selftest(self, seconds: float = 6.0, save_frames: bool = True) -> int:
        """Run a scripted forward motion without a socket; validate state."""
        self.log(f"selftest for {seconds:.1f}s (scripted surge)")
        self.apply_command({"surge": 0.6, "yaw_rate": 0.0})
        n_ticks = int(seconds * self.cfg.ticks_per_sec)
        frames = 0
        last_header = None
        for i in range(n_ticks):
            header, blob = self.step()
            last_header = header
            if header.get("image") and blob:
                frames += 1
                if save_frames and frames == 1:
                    try:
                        from PIL import Image
                        h = header["image"]["height"]
                        w = header["image"]["width"]
                        arr = np.frombuffer(blob, dtype=np.uint8).reshape(h, w, 3)
                        Image.fromarray(arr).save("selftest_first_frame.png")
                        self.log("saved selftest_first_frame.png")
                    except Exception as exc:
                        self.log(f"frame save skipped: {exc!r}")
        self.log(f"ticked {n_ticks}, camera frames={frames}")
        if last_header is not None:
            p = last_header["pose"]
            self.log(f"final pose x={p['x']:.2f} y={p['y']:.2f} z={p['z']:.2f} "
                     f"yaw={math.degrees(p['yaw']):.1f}")
            self.log(f"obstacles tracked: {len(last_header['obstacles'])}")
        ok = (
            frames > 0
            and last_header is not None
            and len(last_header["obstacles"]) == len(self.obstacle_world)
        )
        self.log("SELFTEST PASS" if ok else "SELFTEST FAIL")
        return 0 if ok else 1


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="HoloOcean sim server")
    parser.add_argument("--config", required=True, help="scenario YAML path")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--serve", action="store_true", help="run TCP server")
    parser.add_argument("--selftest", action="store_true",
                        help="run scripted self-test (no socket)")
    parser.add_argument("--selftest-seconds", type=float, default=6.0)
    parser.add_argument("--engine-running", action="store_true",
                        help="attach to an already-running external engine "
                             "window instead of launching one")
    parser.add_argument("--keep-engine", action="store_true",
                        help="leave the external engine window running on exit")
    parser.add_argument("--engine-config", default=None,
                        help="override path to custom_holoocean_engine.yaml")
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    if cfg.custom_engine is not None:
        if args.engine_running:
            cfg.custom_engine.auto_launch = False
        if args.keep_engine:
            cfg.custom_engine.stop_engine_on_exit = False
        if args.engine_config:
            cfg.custom_engine.engine_config = os.path.abspath(args.engine_config)
    server = HolooceanSimServer(cfg)
    server.start()
    try:
        if args.selftest:
            return server.selftest(seconds=args.selftest_seconds)
        if args.serve:
            server.serve(args.host, args.port)
            return 0
        print("Nothing to do: pass --serve or --selftest", file=sys.stderr)
        return 2
    finally:
        server.close()


if __name__ == "__main__":
    raise SystemExit(main())
