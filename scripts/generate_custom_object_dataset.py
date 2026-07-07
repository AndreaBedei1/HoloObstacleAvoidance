#!/usr/bin/env python
"""Generate a YOLO dataset from the visible custom HoloOcean engine.

This script runs in the conda ``ocean`` environment. It launches (or attaches
to) the visible modified Unreal/HoloOcean engine, spawns the custom Unreal
meshes documented in ``config/custom_holoocean_engine.yaml``, captures RGB
camera frames, and writes YOLO labels from the same oracle projection logic
used by the ROS bridge.

No manual annotation, no real rover, no MAVLink, no thrusters.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import shutil
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SERVER_DIR = REPO_ROOT / "src" / "rov_obstacle_sim_bridge" / "holoocean_server"
PKG_PARENT = REPO_ROOT / "src" / "rov_obstacle_sim_bridge"
for path in (SERVER_DIR, PKG_PARENT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from custom_asset_commands import enqueue_clear_spawned, enqueue_spawn_asset  # noqa: E402
from custom_engine_launcher import (  # noqa: E402
    attach_holoocean,
    launch_engine,
    load_engine_config,
    stop_engine,
    validate_engine_config,
)
from holoocean_sim_server import (  # noqa: E402
    _bounds_from_points,
    _euler_matrix_deg,
    _matvec3,
    body_to_world,
)
from rov_obstacle_sim_bridge.oracle_geometry import (  # noqa: E402
    CameraConfig,
    ObstacleConfig,
    RoverPose2D,
    project_obstacles,
)


CLASSES = ("anchor", "mine", "torpedo")
CLASS_IDS = {name: idx for idx, name in enumerate(CLASSES)}


@dataclass(frozen=True)
class ObjectClassSpec:
    class_id: int
    name: str
    mesh_asset: str
    scale_range: tuple[float, float]
    half_extents_per_scale: tuple[float, float, float]
    non_uniform_range: tuple[float, float]
    bounds_offset_per_scale: tuple[float, float, float]
    label_padding_xy: tuple[float, float]


CLASS_SPECS = {
    "anchor": ObjectClassSpec(
        class_id=0,
        name="anchor",
        mesh_asset="/Game/ancora.ancora",
        scale_range=(0.9, 4.2),
        # Existing validated scenario: scale 4.0 -> [0.6, 1.5, 1.8] m.
        half_extents_per_scale=(0.20, 0.46, 0.56),
        non_uniform_range=(0.85, 1.18),
        bounds_offset_per_scale=(0.0, 0.0, 0.0),
        label_padding_xy=(1.18, 1.18),
    ),
    "mine": ObjectClassSpec(
        class_id=1,
        name="mine",
        mesh_asset="/Game/mina.mina",
        # The external population uses large actor scales for this small mesh.
        scale_range=(20.0, 95.0),
        # The rendered mine sits above the actor origin and has visible spikes;
        # use conservative visual bounds instead of the tight sphere core.
        half_extents_per_scale=(0.018, 0.018, 0.020),
        non_uniform_range=(0.92, 1.10),
        bounds_offset_per_scale=(0.0, 0.0, 0.010),
        label_padding_xy=(1.30, 1.30),
    ),
    "torpedo": ObjectClassSpec(
        class_id=2,
        name="torpedo",
        mesh_asset="/Game/siluro.siluro",
        scale_range=(3.0, 12.0),
        # The siluro mesh renders longer than the initial oracle box.
        half_extents_per_scale=(0.34, 0.075, 0.075),
        non_uniform_range=(0.85, 1.20),
        bounds_offset_per_scale=(0.0, 0.0, 0.0),
        label_padding_xy=(1.35, 1.25),
    ),
}


@dataclass
class PlannedObject:
    name: str
    class_name: str
    mesh_asset: str
    position: tuple[float, float, float]
    rotation: tuple[float, float, float]
    scale: tuple[float, float, float]
    half_extents_m: tuple[float, float, float]
    radius_m: float
    bounds: dict[str, list[float]]
    height_bucket: str
    camera_position: tuple[float, float, float]


@dataclass
class BackgroundSite:
    label: str
    category: str
    position: tuple[float, float, float]
    seabed_z: float


def ue_to_client(location_ue_cm: list[float]) -> tuple[float, float, float]:
    return (
        float(location_ue_cm[0]) / 100.0,
        -float(location_ue_cm[1]) / 100.0,
        float(location_ue_cm[2]) / 100.0,
    )


def load_background_sites(population_json: Path, min_site_depth_m: float) -> list[BackgroundSite]:
    data = json.loads(population_json.read_text(encoding="utf-8"))
    sites: list[BackgroundSite] = []
    for entry in data.get("spawns", []):
        loc = entry.get("location")
        if not loc or len(loc) != 3:
            continue
        x, y, z = ue_to_client(loc)
        # Prefer the documented core octree region with real seabed data.
        if not (-190.0 <= x <= 250.0 and -170.0 <= y <= 180.0):
            continue
        if z > -abs(float(min_site_depth_m)):
            continue
        sites.append(
            BackgroundSite(
                label=str(entry.get("label", "site")),
                category=str(entry.get("category", "unknown")),
                position=(x, y, z),
                seabed_z=z,
            )
        )
    if not sites:
        raise RuntimeError(
            f"No usable sites found in {population_json} "
            f"with min_site_depth_m={min_site_depth_m}"
        )
    return sites


def build_scenario_cfg(
    *,
    world: str,
    agent_name: str,
    agent_location: tuple[float, float, float],
    agent_yaw_deg: float,
    camera_width: int,
    camera_height: int,
    ticks_per_sec: int,
    frames_per_sec: int,
    camera_sensor: str,
    camera_socket: str,
    agent_type: str,
) -> dict[str, Any]:
    return {
        "name": "custom_underwater_object_dataset",
        "world": world,
        "main_agent": agent_name,
        "ticks_per_sec": int(ticks_per_sec),
        "frames_per_sec": int(frames_per_sec),
        "agents": [
            {
                "agent_name": agent_name,
                "agent_type": agent_type,
                "sensors": [
                    {"sensor_type": "PoseSensor", "socket": "IMUSocket"},
                    {"sensor_type": "VelocitySensor", "socket": "IMUSocket"},
                    {"sensor_type": "DepthSensor", "socket": "DepthSocket"},
                    {
                        "sensor_type": "RGBCamera",
                        "sensor_name": camera_sensor,
                        "socket": camera_socket,
                        "configuration": {
                            "CaptureWidth": int(camera_width),
                            "CaptureHeight": int(camera_height),
                        },
                    },
                ],
                "control_scheme": 0,
                "location": [float(v) for v in agent_location],
                "rotation": [0.0, 0.0, float(agent_yaw_deg)],
            }
        ],
    }


def choose_height_bucket(
    rng: random.Random,
    hard: bool,
    placement_mode: str,
) -> tuple[str, float]:
    if placement_mode == "suspended_only":
        return "suspended", rng.uniform(14.0, 26.0)

    if hard:
        bucket = rng.choice(["suspended", "near_seabed", "touching", "hard"])
    else:
        r = rng.random()
        if r < 0.30:
            bucket = "suspended"
        elif r < 0.70:
            bucket = "near_seabed"
        elif r < 0.90:
            bucket = "touching"
        else:
            bucket = "hard"

    if bucket == "suspended":
        return bucket, rng.uniform(2.5, 6.0)
    if bucket == "near_seabed":
        return bucket, rng.uniform(0.5, 2.0)
    if bucket == "touching":
        return bucket, rng.uniform(0.0, 0.35)
    return bucket, rng.choice([rng.uniform(0.0, 0.2), rng.uniform(4.0, 7.0)])


def choose_target_center(rng: random.Random, hard: bool) -> tuple[float, float, str]:
    if not hard:
        return rng.uniform(0.22, 0.78), rng.uniform(0.18, 0.40), "normal"

    mode = rng.choice(["edge", "close", "far", "cutoff", "occluded"])
    if mode == "cutoff":
        side = rng.choice(["left", "right", "top", "bottom"])
        if side == "left":
            return rng.uniform(-0.05, 0.08), rng.uniform(0.15, 0.82), mode
        if side == "right":
            return rng.uniform(0.92, 1.05), rng.uniform(0.15, 0.82), mode
        if side == "top":
            return rng.uniform(0.14, 0.86), rng.uniform(-0.04, 0.08), mode
        return rng.uniform(0.14, 0.86), rng.uniform(0.90, 1.04), mode
    return rng.uniform(0.06, 0.94), rng.uniform(0.08, 0.88), mode


def choose_distance_m(rng: random.Random, hard_reason: str) -> float:
    if hard_reason == "close":
        return rng.uniform(6.0, 9.0)
    if hard_reason == "far":
        return rng.uniform(18.0, 30.0)
    return rng.uniform(10.0, 28.0)


def object_scale(spec: ObjectClassSpec, rng: random.Random) -> tuple[float, float, float]:
    base = rng.uniform(spec.scale_range[0], spec.scale_range[1])
    lo, hi = spec.non_uniform_range
    return (
        base * rng.uniform(lo, hi),
        base * rng.uniform(lo, hi),
        base * rng.uniform(lo, hi),
    )


def object_rotation(class_name: str, rng: random.Random) -> tuple[float, float, float]:
    if class_name == "mine":
        return (
            rng.uniform(-20.0, 20.0),
            rng.uniform(-20.0, 20.0),
            rng.uniform(0.0, 360.0),
        )
    if class_name == "torpedo":
        return (
            rng.uniform(-35.0, 35.0),
            rng.uniform(-35.0, 35.0),
            rng.uniform(0.0, 360.0),
        )
    return (
        rng.uniform(-28.0, 28.0),
        rng.uniform(-28.0, 28.0),
        rng.uniform(0.0, 360.0),
    )


def placement_mode_is_suspended(height_bucket: str) -> bool:
    return str(height_bucket).strip().lower() == "suspended"


def bounds_for_object(
    position: tuple[float, float, float],
    rotation: tuple[float, float, float],
    half_extents: tuple[float, float, float],
    bounds_offset: tuple[float, float, float],
) -> dict[str, list[float]]:
    rot = _euler_matrix_deg(rotation)
    off_x, off_y, off_z = _matvec3(rot, bounds_offset)
    visual_center = (position[0] + off_x, position[1] + off_y, position[2] + off_z)
    corners: list[tuple[float, float, float]] = []
    for x in (-half_extents[0], half_extents[0]):
        for y in (-half_extents[1], half_extents[1]):
            for z in (-half_extents[2], half_extents[2]):
                dx, dy, dz = _matvec3(rot, (x, y, z))
                corners.append((visual_center[0] + dx, visual_center[1] + dy, visual_center[2] + dz))
    return _bounds_from_points(corners)


def plan_object(
    *,
    sample_index: int,
    object_index: int,
    class_name: str,
    site: BackgroundSite,
    camera_position: tuple[float, float, float],
    camera_yaw_rad: float,
    target_center: tuple[float, float],
    distance_m: float,
    height_bucket: str,
    clearance_m: float,
    rng: random.Random,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    camera_clearance_min_m: float,
    camera_clearance_max_m: float,
    lock_camera_z: bool = False,
) -> PlannedObject:
    spec = CLASS_SPECS[class_name]
    scale = object_scale(spec, rng)
    half_extents = (
        spec.half_extents_per_scale[0] * scale[0],
        spec.half_extents_per_scale[1] * scale[1],
        spec.half_extents_per_scale[2] * scale[2],
    )
    bounds_offset = (
        spec.bounds_offset_per_scale[0] * scale[0],
        spec.bounds_offset_per_scale[1] * scale[1],
        spec.bounds_offset_per_scale[2] * scale[2],
    )
    center_x, center_y = target_center
    bearing_right = (center_x - 0.5) * math.radians(horizontal_fov_deg)
    elevation = (0.5 - center_y) * math.radians(vertical_fov_deg)
    left_m = -math.tan(bearing_right) * distance_m

    min_camera_z = site.seabed_z + 4.0
    max_camera_z = site.seabed_z + 34.0
    if lock_camera_z:
        camera_z = max(min_camera_z, min(max_camera_z, camera_position[2]))
        rel_up = math.tan(elevation) * distance_m
        obj_z = camera_z + rel_up
        clearance_m = obj_z - half_extents[2] - site.seabed_z
        if clearance_m > 2.5:
            height_bucket = "suspended"
        elif clearance_m > 0.5:
            height_bucket = "near_seabed"
        elif clearance_m >= -0.2:
            height_bucket = "touching"
        else:
            height_bucket = "hard"
    else:
        if placement_mode_is_suspended(height_bucket):
            camera_z = max(
                min_camera_z,
                min(
                    max_camera_z,
                    site.seabed_z + rng.uniform(
                        camera_clearance_min_m,
                        camera_clearance_max_m,
                    ),
                ),
            )
            rel_up = math.tan(elevation) * distance_m
            obj_z = camera_z + rel_up
            clearance_m = obj_z - half_extents[2] - site.seabed_z
        else:
            obj_z = site.seabed_z + half_extents[2] + clearance_m
            camera_z = obj_z - math.tan(elevation) * distance_m
            camera_z = max(min_camera_z, min(max_camera_z, camera_z))
            rel_up = obj_z - camera_z

    dx, dy, dz = body_to_world(distance_m, left_m, rel_up, camera_yaw_rad)
    position = (
        camera_position[0] + dx,
        camera_position[1] + dy,
        camera_z + dz,
    )
    rotation = object_rotation(class_name, rng)
    bounds = bounds_for_object(position, rotation, half_extents, bounds_offset)
    radius = math.sqrt(sum(v * v for v in half_extents))
    return PlannedObject(
        name=f"{class_name}_{sample_index:06d}_{object_index}",
        class_name=class_name,
        mesh_asset=spec.mesh_asset,
        position=position,
        rotation=rotation,
        scale=scale,
        half_extents_m=half_extents,
        radius_m=radius,
        bounds=bounds,
        height_bucket=height_bucket,
        camera_position=(camera_position[0], camera_position[1], camera_z),
    )


def convert_bounds_to_oracle(
    raw_bounds: dict[str, list[float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    mn = raw_bounds["min"]
    mx = raw_bounds["max"]
    corners = [
        (x, -y, z)
        for x in (float(mn[0]), float(mx[0]))
        for y in (float(mn[1]), float(mx[1]))
        for z in (float(mn[2]), float(mx[2]))
    ]
    return (
        (
            min(p[0] for p in corners),
            min(p[1] for p in corners),
            min(p[2] for p in corners),
        ),
        (
            max(p[0] for p in corners),
            max(p[1] for p in corners),
            max(p[2] for p in corners),
        ),
    )


def project_labels(
    objects: list[PlannedObject],
    *,
    camera_position: tuple[float, float, float],
    camera_yaw_rad: float,
    horizontal_fov_deg: float,
    vertical_fov_deg: float,
    max_range_m: float,
    min_box_size: float,
) -> list[dict[str, Any]]:
    rover = RoverPose2D(
        x=camera_position[0],
        y=-camera_position[1],
        z=camera_position[2],
        yaw_rad=-camera_yaw_rad,
    )
    camera = CameraConfig(
        horizontal_fov_deg=horizontal_fov_deg,
        vertical_fov_deg=vertical_fov_deg,
        min_detection_range_m=0.2,
        max_detection_range_m=max_range_m,
        confidence=1.0,
        risk_area_gain=4.0,
    )
    obstacles = [
        ObstacleConfig(
            name=obj.name,
            class_name=obj.class_name,
            position=(obj.position[0], -obj.position[1], obj.position[2]),
            radius_m=obj.radius_m,
            bounds=convert_bounds_to_oracle(obj.bounds),
        )
        for obj in objects
    ]
    labels = []
    for projected in project_obstacles(obstacles, rover, camera):
        pad_x, pad_y = CLASS_SPECS[projected.class_name].label_padding_xy
        width = projected.width * pad_x
        height = projected.height * pad_y
        center_x = projected.center_x
        center_y = projected.center_y
        if width < min_box_size or height < min_box_size:
            continue
        labels.append(
            {
                "class_id": CLASS_IDS[projected.class_name],
                "class_name": projected.class_name,
                "center_x": float(center_x),
                "center_y": float(center_y),
                "width": float(width),
                "height": float(height),
                "range_m": float(projected.range_m),
                "risk": float(projected.risk),
            }
        )
    return labels


def write_rgb(path: Path, rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import cv2

        cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    except ImportError:
        from PIL import Image

        Image.fromarray(rgb).save(str(path))


def capture_rgb(state: dict[str, Any], camera_sensor: str) -> np.ndarray:
    if camera_sensor not in state:
        raise RuntimeError(f"No {camera_sensor!r} frame in HoloOcean state")
    frame = np.array(state[camera_sensor])
    if frame.ndim != 3 or frame.shape[2] < 3:
        raise RuntimeError(f"Unexpected camera frame shape: {frame.shape}")
    # UE FColor memory layout is BGRA; channels 2,1,0 are true RGB.
    return np.ascontiguousarray(frame[:, :, 2::-1].astype(np.uint8))


def apply_color_variation(rgb: np.ndarray, rng: random.Random) -> tuple[np.ndarray, dict[str, Any]]:
    contrast = rng.uniform(0.82, 1.18)
    brightness = rng.uniform(-18.0, 18.0)
    gains = np.array(
        [rng.uniform(0.88, 1.12), rng.uniform(0.90, 1.15), rng.uniform(0.88, 1.12)],
        dtype=np.float32,
    )
    haze_alpha = rng.uniform(0.0, 0.45)
    haze = np.array([90.0, 135.0, 145.0], dtype=np.float32)
    arr = rgb.astype(np.float32)
    arr = (arr - 127.5) * contrast + 127.5 + brightness
    arr = arr * gains
    arr = arr * (1.0 - haze_alpha) + haze * haze_alpha
    arr = np.clip(arr, 0, 255).astype(np.uint8)
    return arr, {
        "contrast": round(contrast, 4),
        "brightness": round(brightness, 4),
        "rgb_gains": [round(float(v), 4) for v in gains],
        "haze_alpha": round(haze_alpha, 4),
    }


def rendered_bbox_label(
    background_rgb: np.ndarray,
    object_rgb: np.ndarray,
    expected_label: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any] | None:
    """YOLO label from rendered object pixels.

    The generator captures the same camera pose once with no spawned object and
    once with the object.  The difference mask gives a conservative box for the
    visible mesh, so labels follow the real render instead of approximate
    configured half-extents.
    """
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - ocean env has cv2
        raise RuntimeError("OpenCV is required for rendered bbox extraction") from exc

    if background_rgb.shape != object_rgb.shape:
        raise ValueError(
            f"background/object frame shape mismatch: "
            f"{background_rgb.shape} != {object_rgb.shape}"
        )

    img_h, img_w = object_rgb.shape[:2]
    diff = np.max(
        np.abs(object_rgb.astype(np.int16) - background_rgb.astype(np.int16)),
        axis=2,
    ).astype(np.uint8)

    cx = float(expected_label["center_x"])
    cy = float(expected_label["center_y"])
    bw = float(expected_label["width"])
    bh = float(expected_label["height"])
    half_w = max(args.diff_roi_min_half_width, bw * args.diff_roi_scale * 0.5)
    half_h = max(args.diff_roi_min_half_height, bh * args.diff_roi_scale * 0.5)
    rx0 = max(0, int((cx - half_w) * img_w))
    rx1 = min(img_w, int((cx + half_w) * img_w))
    ry0 = max(0, int((cy - half_h) * img_h))
    ry1 = min(img_h, int((cy + half_h) * img_h))
    if rx1 <= rx0 or ry1 <= ry0:
        return None

    roi_mask = np.zeros((img_h, img_w), dtype=np.uint8)
    roi_mask[ry0:ry1, rx0:rx1] = 255
    _, mask = cv2.threshold(diff, int(args.diff_threshold), 255, cv2.THRESH_BINARY)
    mask = cv2.bitwise_and(mask, roi_mask)

    k = max(1, int(args.diff_morph_kernel))
    kernel = np.ones((k, k), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    count, components, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
    best = None
    best_score = -1.0
    for comp_id in range(1, count):
        x = int(stats[comp_id, cv2.CC_STAT_LEFT])
        y = int(stats[comp_id, cv2.CC_STAT_TOP])
        w = int(stats[comp_id, cv2.CC_STAT_WIDTH])
        h = int(stats[comp_id, cv2.CC_STAT_HEIGHT])
        area = int(stats[comp_id, cv2.CC_STAT_AREA])
        if area < args.diff_min_component_area_px:
            continue
        if w < args.diff_min_component_width_px or h < args.diff_min_component_height_px:
            continue
        comp_cx = (x + w * 0.5) / img_w
        comp_cy = (y + h * 0.5) / img_h
        distance = abs(comp_cx - cx) + abs(comp_cy - cy)
        score = float(area) / (0.05 + distance)
        if score > best_score:
            best_score = score
            best = (x, y, w, h, area)

    if best is None:
        return None

    x, y, w, h, area = best
    pad_x = max(args.rendered_bbox_padding_px, int(w * args.rendered_bbox_padding_fraction))
    pad_y = max(args.rendered_bbox_padding_px, int(h * args.rendered_bbox_padding_fraction))
    x0 = max(0, x - pad_x)
    y0 = max(0, y - pad_y)
    x1 = min(img_w, x + w + pad_x)
    y1 = min(img_h, y + h + pad_y)
    norm_w = (x1 - x0) / img_w
    norm_h = (y1 - y0) / img_h
    return {
        "class_id": int(expected_label["class_id"]),
        "class_name": str(expected_label["class_name"]),
        "center_x": (x0 + x1) * 0.5 / img_w,
        "center_y": (y0 + y1) * 0.5 / img_h,
        "width": norm_w,
        "height": norm_h,
        "range_m": float(expected_label.get("range_m", 0.0)),
        "risk": float(expected_label.get("risk", 0.0)),
        "rendered_component_area_px": int(area),
    }


def yolo_line(label: dict[str, Any]) -> str:
    return (
        f"{int(label['class_id'])} "
        f"{label['center_x']:.6f} {label['center_y']:.6f} "
        f"{label['width']:.6f} {label['height']:.6f}"
    )


def labels_are_clean(labels: list[dict[str, Any]], args: argparse.Namespace) -> bool:
    if len(labels) != 1:
        return False
    label = labels[0]
    width = float(label["width"])
    height = float(label["height"])
    area = width * height
    if width < args.min_box_width or height < args.min_box_height:
        return False
    if width > args.max_box_width or height > args.max_box_height:
        return False
    if area > args.max_box_area:
        return False
    margin = float(args.full_visibility_margin)
    cx = float(label["center_x"])
    cy = float(label["center_y"])
    x0 = cx - width / 2.0
    x1 = cx + width / 2.0
    y0 = cy - height / 2.0
    y1 = cy + height / 2.0
    return x0 >= margin and y0 >= margin and x1 <= 1.0 - margin and y1 <= 1.0 - margin


def objects_are_above_seabed(
    objects: list[PlannedObject],
    site: BackgroundSite,
    min_bottom_clearance_m: float,
) -> bool:
    for obj in objects:
        bottom_z = float(obj.bounds["min"][2])
        if bottom_z < site.seabed_z + min_bottom_clearance_m:
            return False
    return True


def prepare_output(root: Path, overwrite: bool) -> None:
    if root.exists() and overwrite:
        shutil.rmtree(root)
    for split in ("train", "val", "test"):
        (root / "images" / split).mkdir(parents=True, exist_ok=True)
        (root / "labels" / split).mkdir(parents=True, exist_ok=True)
    (root / "metadata").mkdir(parents=True, exist_ok=True)


def write_dataset_yaml(root: Path) -> None:
    data = {
        "path": str(root.resolve()),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": {idx: name for idx, name in enumerate(CLASSES)},
    }
    (root / "dataset.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def yaml_safe(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, tuple):
        return [yaml_safe(v) for v in value]
    if isinstance(value, list):
        return [yaml_safe(v) for v in value]
    if isinstance(value, dict):
        return {str(k): yaml_safe(v) for k, v in value.items()}
    return value


def write_generation_config(root: Path, args: argparse.Namespace, engine_cfg: dict[str, Any]) -> None:
    payload = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "generator": "scripts/generate_custom_object_dataset.py",
        "args": yaml_safe(vars(args)),
        "classes": yaml_safe({name: asdict(spec) for name, spec in CLASS_SPECS.items()}),
        "engine": {
            "config": str(args.engine_config),
            "map": str(engine_cfg.get("external_engine", {}).get("default_map", "ExampleLevel")),
            "assets": engine_cfg.get("assets", {}),
            "visible": True,
        },
        "label_source": "rendered foreground/background image difference, with oracle projection only used as a candidate ROI",
        "simulation_only": True,
    }
    (root / "metadata" / "generation_config.yaml").write_text(
        yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
    )


def plan_scene(
    *,
    global_index: int,
    primary_class: str,
    site: BackgroundSite,
    camera_position_xy: tuple[float, float],
    camera_yaw_rad: float,
    rng: random.Random,
    args: argparse.Namespace,
) -> tuple[tuple[float, float, float], list[PlannedObject], list[dict[str, Any]], dict[str, Any]]:
    hard = rng.random() < args.hard_case_fraction
    center_x, center_y, hard_reason = choose_target_center(rng, hard)
    distance = choose_distance_m(rng, hard_reason)
    bucket, clearance = choose_height_bucket(rng, hard, args.placement_mode)

    # Camera Z is resolved by plan_object. Seed with a valid value and update
    # it after the primary object computes the actual camera height.
    camera_position = (camera_position_xy[0], camera_position_xy[1], site.seabed_z + 3.0)
    primary = plan_object(
        sample_index=global_index,
        object_index=0,
        class_name=primary_class,
        site=site,
        camera_position=camera_position,
        camera_yaw_rad=camera_yaw_rad,
        target_center=(center_x, center_y),
        distance_m=distance,
        height_bucket=bucket,
        clearance_m=clearance,
        rng=rng,
        horizontal_fov_deg=args.horizontal_fov_deg,
        vertical_fov_deg=args.vertical_fov_deg,
        camera_clearance_min_m=args.camera_clearance_min_m,
        camera_clearance_max_m=args.camera_clearance_max_m,
    )
    camera_position = primary.camera_position

    objects = [primary]
    scene_notes = {
        "hard_case": hard,
        "hard_reason": hard_reason,
        "primary_target_center": [round(center_x, 4), round(center_y, 4)],
        "primary_distance_m": round(distance, 4),
    }

    if args.multi_object_fraction > 0.0 and rng.random() < args.multi_object_fraction:
        extra_count = 1 + (1 if rng.random() < 0.25 else 0)
        for extra_idx in range(extra_count):
            cls = rng.choice(list(CLASSES))
            if hard_reason == "occluded" and extra_idx == 0:
                cx = max(0.05, min(0.95, center_x + rng.uniform(-0.08, 0.08)))
                cy = max(0.05, min(0.92, center_y + rng.uniform(-0.06, 0.06)))
                dist = max(2.0, distance - rng.uniform(0.8, 2.0))
            else:
                cx = rng.uniform(0.08, 0.92)
                cy = rng.uniform(0.10, 0.86)
                dist = rng.uniform(4.0, 16.0)
            bkt, clr = choose_height_bucket(rng, hard, args.placement_mode)
            objects.append(
                plan_object(
                    sample_index=global_index,
                    object_index=extra_idx + 1,
                    class_name=cls,
                    site=site,
                    camera_position=camera_position,
                    camera_yaw_rad=camera_yaw_rad,
                    target_center=(cx, cy),
                    distance_m=dist,
                    height_bucket=bkt,
                    clearance_m=clr,
                    rng=rng,
                    horizontal_fov_deg=args.horizontal_fov_deg,
                    vertical_fov_deg=args.vertical_fov_deg,
                    camera_clearance_min_m=args.camera_clearance_min_m,
                    camera_clearance_max_m=args.camera_clearance_max_m,
                    lock_camera_z=True,
                )
            )

    labels = project_labels(
        objects,
        camera_position=camera_position,
        camera_yaw_rad=camera_yaw_rad,
        horizontal_fov_deg=args.horizontal_fov_deg,
        vertical_fov_deg=args.vertical_fov_deg,
        max_range_m=args.max_label_range_m,
        min_box_size=args.min_box_size,
    )
    return camera_position, objects, labels, scene_notes


def generate_sample(
    *,
    env: Any,
    split: str,
    split_index: int,
    global_index: int,
    sites: list[BackgroundSite],
    rng: random.Random,
    args: argparse.Namespace,
) -> dict[str, Any] | None:
    agent = env.agents[args.agent_name]
    accepted = None
    for attempt in range(args.max_attempts_per_sample):
        site = rng.choice(sites)
        camera_yaw_rad = rng.uniform(-math.pi, math.pi)
        primary_class = CLASSES[global_index % len(CLASSES)]
        jitter_fwd = rng.uniform(-4.0, 4.0)
        jitter_left = rng.uniform(-4.0, 4.0)
        dx, dy, _ = body_to_world(jitter_fwd, jitter_left, 0.0, camera_yaw_rad)
        camera_xy = (site.position[0] + dx, site.position[1] + dy)

        camera_position, objects, labels, scene_notes = plan_scene(
            global_index=global_index,
            primary_class=primary_class,
            site=site,
            camera_position_xy=camera_xy,
            camera_yaw_rad=camera_yaw_rad,
            rng=rng,
            args=args,
        )
        if (
            labels_are_clean(labels, args)
            and objects_are_above_seabed(objects, site, args.min_bottom_clearance_m)
        ):
            enqueue_clear_spawned(env)
            env.tick()
            agent.teleport(
                location=np.array(camera_position),
                rotation=np.array([0.0, 0.0, math.degrees(camera_yaw_rad)]),
            )
            state = None
            for _ in range(max(1, args.background_settle_ticks)):
                state = env.tick()
            background_rgb = capture_rgb(state, args.camera_sensor)

            for obj in objects:
                enqueue_spawn_asset(
                    env,
                    position=list(obj.position),
                    rotation=list(obj.rotation),
                    scale=list(obj.scale),
                    mesh_asset=obj.mesh_asset,
                    label=obj.name,
                    units="meters",
                )
            for _ in range(max(1, args.settle_ticks)):
                state = env.tick()
            object_rgb = capture_rgb(state, args.camera_sensor)
            rendered_label = rendered_bbox_label(
                background_rgb,
                object_rgb,
                labels[0],
                args,
            )
            if rendered_label is None:
                continue
            rendered_labels = [rendered_label]
            if not labels_are_clean(rendered_labels, args):
                continue
            labels = rendered_labels
            rgb = object_rgb
            accepted = (
                site,
                camera_yaw_rad,
                camera_position,
                objects,
                labels,
                scene_notes,
                rgb,
            )
            break
    else:
        return None

    if accepted is None:
        return None
    site, camera_yaw_rad, camera_position, objects, labels, scene_notes, rgb = accepted

    color_variation = {}
    if not args.disable_color_variation:
        rgb, color_variation = apply_color_variation(rgb, rng)

    stem = f"{split}_{split_index:06d}"
    image_path = args.output_dir / "images" / split / f"{stem}.png"
    label_path = args.output_dir / "labels" / split / f"{stem}.txt"
    write_rgb(image_path, rgb)
    label_path.write_text("\n".join(yolo_line(label) for label in labels) + "\n", encoding="utf-8")

    return {
        "split": split,
        "image": str(image_path.relative_to(args.output_dir)),
        "label": str(label_path.relative_to(args.output_dir)),
        "camera": {
            "position": [round(float(v), 4) for v in camera_position],
            "yaw_deg": round(math.degrees(camera_yaw_rad), 4),
            "width": int(args.camera_width),
            "height": int(args.camera_height),
            "horizontal_fov_deg": float(args.horizontal_fov_deg),
            "vertical_fov_deg": float(args.vertical_fov_deg),
        },
        "background_site": asdict(site),
        "scene": scene_notes,
        "bbox_source": "rendered_background_difference",
        "objects": [
            {
                "name": obj.name,
                "class_name": obj.class_name,
                "position": [round(float(v), 4) for v in obj.position],
                "rotation": [round(float(v), 4) for v in obj.rotation],
                "scale": [round(float(v), 4) for v in obj.scale],
                "half_extents_m": [round(float(v), 4) for v in obj.half_extents_m],
                "height_bucket": obj.height_bucket,
            }
            for obj in objects
        ],
        "labels": [
            {
                key: (round(float(value), 6) if isinstance(value, float) else value)
                for key, value in label.items()
            }
            for label in labels
        ],
        "color_variation": color_variation,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "datasets" / "custom_underwater_objects")
    parser.add_argument("--engine-config", type=Path, default=REPO_ROOT / "config" / "custom_holoocean_engine.yaml")
    parser.add_argument("--train-count", type=int, default=1500)
    parser.add_argument("--val-count", type=int, default=300)
    parser.add_argument("--test-count", type=int, default=300)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--camera-width", type=int, default=512)
    parser.add_argument("--camera-height", type=int, default=512)
    parser.add_argument("--horizontal-fov-deg", type=float, default=90.0)
    parser.add_argument("--vertical-fov-deg", type=float, default=90.0)
    parser.add_argument("--ticks-per-sec", type=int, default=30)
    parser.add_argument("--frames-per-sec", type=int, default=30)
    parser.add_argument("--background-settle-ticks", type=int, default=3)
    parser.add_argument("--settle-ticks", type=int, default=3)
    parser.add_argument("--agent-name", default="auv0")
    parser.add_argument("--agent-type", default="HoveringAUV")
    parser.add_argument("--camera-sensor", default="FrontCamera")
    parser.add_argument("--camera-socket", default="CameraLeftSocket")
    parser.add_argument("--world", default="ExampleLevel")
    parser.add_argument("--engine-running", action="store_true", help="attach to a freshly opened visible engine")
    parser.add_argument("--keep-engine", action="store_true", help="leave the launched engine window open")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--disable-color-variation", action="store_true")
    parser.add_argument("--placement-mode", choices=["suspended_only", "mixed"], default="suspended_only")
    parser.add_argument("--multi-object-fraction", type=float, default=0.0)
    parser.add_argument("--hard-case-fraction", type=float, default=0.0)
    parser.add_argument("--min-site-depth-m", type=float, default=65.0)
    parser.add_argument("--camera-clearance-min-m", type=float, default=20.0)
    parser.add_argument("--camera-clearance-max-m", type=float, default=30.0)
    parser.add_argument("--max-label-range-m", type=float, default=36.0)
    parser.add_argument("--min-box-size", type=float, default=0.008)
    parser.add_argument("--min-box-width", type=float, default=0.035)
    parser.add_argument("--min-box-height", type=float, default=0.035)
    parser.add_argument("--max-box-width", type=float, default=0.70)
    parser.add_argument("--max-box-height", type=float, default=0.70)
    parser.add_argument("--max-box-area", type=float, default=0.32)
    parser.add_argument("--full-visibility-margin", type=float, default=0.055)
    parser.add_argument("--min-bottom-clearance-m", type=float, default=12.0)
    parser.add_argument("--diff-threshold", type=int, default=24)
    parser.add_argument("--diff-morph-kernel", type=int, default=3)
    parser.add_argument("--diff-min-component-area-px", type=int, default=120)
    parser.add_argument("--diff-min-component-width-px", type=int, default=5)
    parser.add_argument("--diff-min-component-height-px", type=int, default=5)
    parser.add_argument("--diff-roi-scale", type=float, default=4.5)
    parser.add_argument("--diff-roi-min-half-width", type=float, default=0.30)
    parser.add_argument("--diff-roi-min-half-height", type=float, default=0.24)
    parser.add_argument("--rendered-bbox-padding-px", type=int, default=8)
    parser.add_argument("--rendered-bbox-padding-fraction", type=float, default=0.20)
    parser.add_argument("--max-attempts-per-sample", type=int, default=180)
    parser.add_argument("--progress-every", type=int, default=50)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir = args.output_dir.resolve()
    rng = random.Random(args.seed)

    engine_cfg = load_engine_config(str(args.engine_config))
    problems = validate_engine_config(engine_cfg, check_paths=True)
    if problems:
        print("[dataset] external engine is not usable:")
        for problem in problems:
            print(f"  - {problem}")
        return 2

    population = Path(str(engine_cfg["external_engine"].get("world_population_json", "")))
    sites = load_background_sites(population, args.min_site_depth_m)
    print(f"[dataset] background sites: {len(sites)} from {population}")
    prepare_output(args.output_dir, overwrite=args.overwrite)
    write_dataset_yaml(args.output_dir)
    write_generation_config(args.output_dir, args, engine_cfg)

    launch_cfg = engine_cfg.get("launch", {}) or {}
    start_position = sites[0].position
    start_location = (start_position[0], start_position[1], start_position[2] + 3.0)
    scenario_cfg = build_scenario_cfg(
        world=args.world,
        agent_name=args.agent_name,
        agent_location=start_location,
        agent_yaw_deg=0.0,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        ticks_per_sec=args.ticks_per_sec or int(launch_cfg.get("ticks_per_sec", 30)),
        frames_per_sec=args.frames_per_sec or int(launch_cfg.get("frames_per_sec", 30)),
        camera_sensor=args.camera_sensor,
        camera_socket=args.camera_socket,
        agent_type=args.agent_type,
    )

    engine_proc = None
    env = None
    t0 = time.time()
    try:
        if not args.engine_running:
            print("[dataset] launching visible external engine")
            engine_proc = launch_engine(engine_cfg, map_name=args.world)
        else:
            print("[dataset] attaching to an already-open fresh visible engine")
        env = attach_holoocean(scenario_cfg, engine_cfg, engine_process=engine_proc)
        print(f"[dataset] attached after {time.time() - t0:.1f}s")
        enqueue_clear_spawned(env)
        env.tick()

        samples_path = args.output_dir / "metadata" / "samples.jsonl"
        total = args.train_count + args.val_count + args.test_count
        written = 0
        failed = 0
        splits = [
            ("train", args.train_count),
            ("val", args.val_count),
            ("test", args.test_count),
        ]
        with samples_path.open("w", encoding="utf-8") as fh:
            global_index = 0
            for split, count in splits:
                for split_index in range(count):
                    sample = generate_sample(
                        env=env,
                        split=split,
                        split_index=split_index,
                        global_index=global_index,
                        sites=sites,
                        rng=rng,
                        args=args,
                    )
                    global_index += 1
                    if sample is None:
                        failed += 1
                        continue
                    fh.write(json.dumps(sample, sort_keys=True) + "\n")
                    written += 1
                    if written == 1 or written % max(1, args.progress_every) == 0:
                        print(f"[dataset] {written}/{total} samples written")

        print(f"[dataset] finished: written={written}, failed={failed}, root={args.output_dir}")
        return 0 if failed == 0 else 1
    finally:
        if env is not None:
            try:
                enqueue_clear_spawned(env)
                env.tick()
                env.__exit__(None, None, None)
            except Exception:
                pass
        if engine_proc is not None and not args.keep_engine:
            stop_engine(engine_proc)


if __name__ == "__main__":
    raise SystemExit(main())
