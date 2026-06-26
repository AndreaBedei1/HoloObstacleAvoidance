"""Pure-Python simulation oracle geometry.

Convert known obstacle world positions and a simulated rover pose into
camera-space detections compatible with the Obstacle2DArray interface.

No ROS 2 nodes, no HoloOcean dependency, no neural network.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class ObstacleConfig:
    """Static obstacle definition loaded from YAML."""

    name: str
    class_name: str
    position: tuple[float, float, float]
    radius_m: float


@dataclass
class RoverPose2D:
    """Rover pose in the world (ENU-like) frame."""

    x: float
    y: float
    z: float
    yaw_rad: float


@dataclass
class CameraConfig:
    """Camera / detection parameters."""

    horizontal_fov_deg: float = 90.0
    vertical_fov_deg: float = 60.0
    min_detection_range_m: float = 0.2
    max_detection_range_m: float = 10.0
    confidence: float = 1.0
    risk_area_gain: float = 4.0


@dataclass
class ProjectedObstacle:
    """Camera-space projection of a single obstacle."""

    name: str
    class_name: str
    center_x: float
    center_y: float
    width: float
    height: float
    bearing_rad: float
    elevation_rad: float
    range_m: float
    apparent_area: float
    risk: float
    confidence: float


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp *value* to [lower, upper]."""
    return max(lower, min(value, upper))


def world_to_camera(
    obstacle_position: tuple[float, float, float],
    rover_pose: RoverPose2D,
) -> tuple[float, float, float]:
    r"""Transform a world position into the rover / camera frame.

    Convention:
        +x forward, +y right, +z up.
        yaw_rad = 0 means rover faces world +x.
    """
    dx = obstacle_position[0] - rover_pose.x
    dy = obstacle_position[1] - rover_pose.y
    dz = obstacle_position[2] - rover_pose.z

    cos_y = math.cos(rover_pose.yaw_rad)
    sin_y = math.sin(rover_pose.yaw_rad)

    x_cam = cos_y * dx + sin_y * dy
    y_cam = -sin_y * dx + cos_y * dy
    z_cam = dz

    return (x_cam, y_cam, z_cam)


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------

def _compute_risk(
    center_x: float,
    range_m: float,
    apparent_area: float,
    camera: CameraConfig,
) -> float:
    """Oracle risk score in [0, 1]."""
    centrality = 1.0 - min(abs(center_x - 0.5) * 2.0, 1.0)

    distance_factor = 1.0 - clamp(
        (range_m - camera.min_detection_range_m)
        / (camera.max_detection_range_m - camera.min_detection_range_m),
        0.0,
        1.0,
    )

    area_factor = clamp(apparent_area * camera.risk_area_gain, 0.0, 1.0)

    risk = camera.confidence * clamp(
        0.45 * centrality + 0.35 * distance_factor + 0.20 * area_factor,
        0.0,
        1.0,
    )
    return risk


def project_obstacle(
    obstacle: ObstacleConfig,
    rover_pose: RoverPose2D,
    camera: CameraConfig,
) -> Optional[ProjectedObstacle]:
    """Project a single obstacle into the camera frame.

    Returns ``None`` when the obstacle is outside the detection volume.
    """
    x_cam, y_cam, z_cam = world_to_camera(obstacle.position, rover_pose)

    # Behind the camera
    if x_cam <= 0:
        return None

    bearing_rad = math.atan2(y_cam, x_cam)
    elevation_rad = math.atan2(z_cam, x_cam)
    range_m = math.sqrt(x_cam ** 2 + y_cam ** 2 + z_cam ** 2)

    # Range bounds
    if range_m < camera.min_detection_range_m:
        return None
    if range_m > camera.max_detection_range_m:
        return None

    h_fov_rad = math.radians(camera.horizontal_fov_deg)
    v_fov_rad = math.radians(camera.vertical_fov_deg)

    # FOV bounds
    if abs(bearing_rad) > h_fov_rad / 2:
        return None
    if abs(elevation_rad) > v_fov_rad / 2:
        return None

    # Normalised image coordinates
    center_x = 0.5 + bearing_rad / h_fov_rad
    center_y = 0.5 - elevation_rad / v_fov_rad

    # Apparent size
    angular_radius = math.atan2(obstacle.radius_m, range_m)
    width = 2.0 * angular_radius / h_fov_rad
    height = 2.0 * angular_radius / v_fov_rad

    apparent_area = width * height

    risk = _compute_risk(center_x, range_m, apparent_area, camera)

    return ProjectedObstacle(
        name=obstacle.name,
        class_name=obstacle.class_name,
        center_x=clamp(center_x, 0.0, 1.0),
        center_y=clamp(center_y, 0.0, 1.0),
        width=clamp(width, 0.0, 1.0),
        height=clamp(height, 0.0, 1.0),
        bearing_rad=bearing_rad,
        elevation_rad=elevation_rad,
        range_m=range_m,
        apparent_area=apparent_area,
        risk=clamp(risk, 0.0, 1.0),
        confidence=clamp(camera.confidence, 0.0, 1.0),
    )


def project_obstacles(
    obstacles: list[ObstacleConfig],
    rover_pose: RoverPose2D,
    camera: CameraConfig,
) -> list[ProjectedObstacle]:
    """Project a list of obstacles, filtering out those outside the FOV."""
    results: list[ProjectedObstacle] = []
    for obs in obstacles:
        projected = project_obstacle(obs, rover_pose, camera)
        if projected is not None:
            results.append(projected)
    return results


# ---------------------------------------------------------------------------
# YAML loader
# ---------------------------------------------------------------------------

def load_obstacle_config_yaml(path: str | Path) -> list[ObstacleConfig]:
    """Load obstacle definitions from a YAML file.

    Expected top-level key: ``obstacles`` (list of mappings).
    Returns an empty list when the key is missing.
    Raises ``ValueError`` on malformed obstacle entries.
    """
    if yaml is None:
        raise ImportError("PyYAML is required to load obstacle config files.")

    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    if not isinstance(data, dict):
        return []

    raw_list = data.get("obstacles")
    if raw_list is None:
        return []

    obstacles: list[ObstacleConfig] = []
    for idx, entry in enumerate(raw_list):
        try:
            name = str(entry["name"])
            class_name = str(entry["class_name"])
            pos = entry["position"]
            radius_m = float(entry["radius_m"])
            p = (float(pos[0]), float(pos[1]), float(pos[2]))
            obstacles.append(ObstacleConfig(name, class_name, p, radius_m))
        except Exception as exc:
            raise ValueError(
                f"Invalid obstacle entry at index {idx}: {exc}"
            ) from exc

    return obstacles
