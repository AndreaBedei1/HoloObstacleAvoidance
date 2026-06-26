"""Tests for oracle_demo_recorder_node helper functions and CSV setup."""

import csv
import tempfile
from pathlib import Path

from rov_obstacle_sim_bridge.oracle_demo_recorder_node import (
    CSV_HEADER,
    build_csv_row,
    summarize_obstacles,
    twist_to_command_fields,
)


class FakeObstacle:
    """Minimal stand-in for Obstacle2D (avoids importing ROS messages)."""

    def __init__(self, risk: float, center_x: float, bearing_rad: float) -> None:
        self.risk = risk
        self.center_x = center_x
        self.bearing_rad = bearing_rad


class FakeTwist:
    """Minimal stand-in for Twist."""

    def __init__(
        self, linear_x: float = 0.0, linear_y: float = 0.0, angular_z: float = 0.0
    ) -> None:
        class Linear:
            x: float
            y: float

            def __init__(self, x: float, y: float) -> None:
                self.x = x
                self.y = y

        class Angular:
            z: float

            def __init__(self, z: float) -> None:
                self.z = z

        self.linear = Linear(linear_x, linear_y)
        self.angular = Angular(angular_z)


# ---------------------------------------------------------------------------
# summarize_obstacles
# ---------------------------------------------------------------------------

def test_summarize_obstacles_empty() -> None:
    result = summarize_obstacles([])
    assert result["obstacle_count"] == 0
    assert result["max_obstacle_risk"] == 0.0
    assert result["most_dangerous_center_x"] == 0.0
    assert result["most_dangerous_bearing_rad"] == 0.0


def test_summarize_obstacles_single() -> None:
    obstacles = [FakeObstacle(risk=0.8, center_x=0.3, bearing_rad=-0.5)]
    result = summarize_obstacles(obstacles)
    assert result["obstacle_count"] == 1
    assert result["max_obstacle_risk"] == 0.8
    assert result["most_dangerous_center_x"] == 0.3
    assert result["most_dangerous_bearing_rad"] == -0.5


def test_summarize_obstacles_picks_worst() -> None:
    obstacles = [
        FakeObstacle(risk=0.4, center_x=0.2, bearing_rad=-1.0),
        FakeObstacle(risk=0.9, center_x=0.7, bearing_rad=0.6),
        FakeObstacle(risk=0.6, center_x=0.5, bearing_rad=0.0),
    ]
    result = summarize_obstacles(obstacles)
    assert result["obstacle_count"] == 3
    assert result["max_obstacle_risk"] == 0.9
    assert result["most_dangerous_center_x"] == 0.7
    assert result["most_dangerous_bearing_rad"] == 0.6


# ---------------------------------------------------------------------------
# twist_to_command_fields
# ---------------------------------------------------------------------------

def test_twist_to_command_fields_none() -> None:
    result = twist_to_command_fields(None)
    assert result == {"surge": 0.0, "sway": 0.0, "yaw_rate": 0.0}


def test_twist_to_command_fields_values() -> None:
    twist = FakeTwist(linear_x=1.0, linear_y=-0.5, angular_z=0.3)
    result = twist_to_command_fields(twist)
    assert result["surge"] == 1.0
    assert result["sway"] == -0.5
    assert result["yaw_rate"] == 0.3


# ---------------------------------------------------------------------------
# build_csv_row
# ---------------------------------------------------------------------------

def test_build_csv_row_length() -> None:
    row = build_csv_row(
        timestamp_s=1.0,
        rov_x=0.0,
        rov_y=0.0,
        rov_z=0.0,
        obstacle_summary={"obstacle_count": 0, "max_obstacle_risk": 0.0, "most_dangerous_center_x": 0.0, "most_dangerous_bearing_rad": 0.0},
        nominal_fields={"surge": 0.0, "sway": 0.0, "yaw_rate": 0.0},
        safe_fields={"surge": 0.0, "sway": 0.0, "yaw_rate": 0.0},
        planner_state=None,
        selected_side=None,
        debug_risk=None,
    )
    assert len(row) == len(CSV_HEADER)


def test_build_csv_row_fills_none_values() -> None:
    row = build_csv_row(
        timestamp_s=0.5,
        rov_x=None,
        rov_y=None,
        rov_z=None,
        obstacle_summary={"obstacle_count": 0, "max_obstacle_risk": 0.0, "most_dangerous_center_x": 0.0, "most_dangerous_bearing_rad": 0.0},
        nominal_fields={"surge": 0.0, "sway": 0.0, "yaw_rate": 0.0},
        safe_fields={"surge": 0.0, "sway": 0.0, "yaw_rate": 0.0},
        planner_state=None,
        selected_side=None,
        debug_risk=None,
    )
    assert row[1] == 0.0  # rov_x fallback
    assert row[2] == 0.0  # rov_y fallback
    assert row[3] == 0.0  # rov_z fallback
    assert row[15] == ""  # planner_state fallback
    assert row[16] == 0.0  # debug_risk fallback


def test_build_csv_row_populated() -> None:
    obstacle_summary = {
        "obstacle_count": 2,
        "max_obstacle_risk": 0.75,
        "most_dangerous_center_x": 0.4,
        "most_dangerous_bearing_rad": -0.3,
    }
    row = build_csv_row(
        timestamp_s=10.0,
        rov_x=1.0,
        rov_y=2.0,
        rov_z=-3.0,
        obstacle_summary=obstacle_summary,
        nominal_fields={"surge": 0.5, "sway": 0.0, "yaw_rate": 0.1},
        safe_fields={"surge": 0.3, "sway": -0.2, "yaw_rate": -0.15},
        planner_state="AVOIDING",
        selected_side="RIGHT",
        debug_risk=0.75,
    )
    assert row[0] == 10.0
    assert row[1] == 1.0
    assert row[4] == 2
    assert row[5] == 0.75
    assert row[8] == 0.5
    assert row[12] == -0.2
    assert row[14] == "AVOIDING"
    assert row[15] == "RIGHT"


# ---------------------------------------------------------------------------
# CSV header
# ---------------------------------------------------------------------------

def test_csv_header_has_17_columns() -> None:
    assert len(CSV_HEADER) == 17


def test_csv_header_contains_expected_columns() -> None:
    expected = {"timestamp_s", "rov_x", "nominal_surge", "safe_sway", "planner_state", "debug_risk"}
    assert expected.issubset(set(CSV_HEADER))


# ---------------------------------------------------------------------------
# CSV file creation (directory auto-creation)
# ---------------------------------------------------------------------------

def test_csv_directory_auto_creation() -> None:
    """The recorder creates parent directories for the output CSV path."""
    with tempfile.TemporaryDirectory() as tmp:
        nested = Path(tmp) / "deep" / "nested" / "test.csv"
        nested.parent.mkdir(parents=True, exist_ok=True)
        with nested.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
        assert nested.exists()
        assert nested.stat().st_size > 0
