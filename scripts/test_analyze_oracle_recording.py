"""Tests for scripts/analyze_oracle_recording.py helper functions and analysis logic."""

import csv
import tempfile
from pathlib import Path

# Import the standalone analysis module directly (no ROS dependency).
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

from analyze_oracle_recording import (  # noqa: E402
    CMD_DIFF_THRESHOLD,
    COL,
    _cmd_differs,
    _float,
    analyze,
    read_csv,
)


# ---------------------------------------------------------------------------
# _float
# ---------------------------------------------------------------------------

def test_float_normal() -> None:
    assert _float(["1.5", "2"], 0) == 1.5


def test_float_zero() -> None:
    assert _float(["0.0"], 0) == 0.0


def test_float_negative() -> None:
    assert _float(["-3.14"], 0) == -3.14


def test_float_index_error_returns_zero() -> None:
    assert _float(["1.0"], 5) == 0.0


def test_float_value_error_returns_zero() -> None:
    assert _float(["abc"], 0) == 0.0


# ---------------------------------------------------------------------------
# _cmd_differs
# ---------------------------------------------------------------------------

def _make_row(
    nominal_surge: float, nominal_sway: float, nominal_yaw: float,
    safe_surge: float, safe_sway: float, safe_yaw: float
) -> list[str]:
    """Build a minimal 17-column row with the given command values."""
    row = [""] * 17
    row[COL["nominal_surge"]] = str(nominal_surge)
    row[COL["nominal_sway"]] = str(nominal_sway)
    row[COL["nominal_yaw_rate"]] = str(nominal_yaw)
    row[COL["safe_surge"]] = str(safe_surge)
    row[COL["safe_sway"]] = str(safe_sway)
    row[COL["safe_yaw_rate"]] = str(safe_yaw)
    return row


def test_cmd_differs_identical() -> None:
    row = _make_row(1.0, 0.5, 0.2, 1.0, 0.5, 0.2)
    assert not _cmd_differs(row)


def test_cmd_differs_surge_above_threshold() -> None:
    row = _make_row(1.0, 0.0, 0.0, 1.0 + CMD_DIFF_THRESHOLD + 0.01, 0.0, 0.0)
    assert _cmd_differs(row)


def test_cmd_differs_sway() -> None:
    row = _make_row(0.0, 0.0, 0.0, 0.0, -0.5, 0.0)
    assert _cmd_differs(row)


def test_cmd_differs_yaw() -> None:
    row = _make_row(0.0, 0.0, 0.0, 0.0, 0.0, 0.3)
    assert _cmd_differs(row)


def test_cmd_differs_below_threshold() -> None:
    """Difference strictly below threshold should NOT trigger."""
    row = _make_row(1.0, 0.0, 0.0, 1.0 + CMD_DIFF_THRESHOLD - 0.001, 0.0, 0.0)
    assert not _cmd_differs(row)


# ---------------------------------------------------------------------------
# read_csv
# ---------------------------------------------------------------------------

def test_read_csv_normal() -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["a", "b", "c"])
        writer.writerow([1, 2, 3])
        writer.writerow([4, 5, 6])
        path = f.name

    try:
        header, rows = read_csv(path)
        assert header == ["a", "b", "c"]
        assert len(rows) == 2
        assert rows[0] == ["1", "2", "3"]
    finally:
        Path(path).unlink()


def test_read_csv_header_only_system_exits() -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False, newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x"])
        path = f.name

    try:
        read_csv(path)  # should call sys.exit(1)
        assert False, "expected SystemExit"  # pragma: no cover
    except SystemExit:
        pass
    finally:
        Path(path).unlink()


def test_read_csv_missing_file_system_exits() -> None:
    try:
        read_csv("/nonexistent/path/file.csv")
        assert False, "expected SystemExit"  # pragma: no cover
    except SystemExit:
        pass


# ---------------------------------------------------------------------------
# analyze
# ---------------------------------------------------------------------------

def _make_full_header() -> list[str]:
    return [
        "timestamp_s", "rov_x", "rov_y", "rov_z", "obstacle_count",
        "max_obstacle_risk", "most_dangerous_center_x", "most_dangerous_bearing_rad",
        "nominal_surge", "nominal_sway", "nominal_yaw_rate",
        "safe_surge", "safe_sway", "safe_yaw_rate",
        "planner_state", "selected_side", "debug_risk",
    ]


def _make_full_row(
    ts: float, risk: float, count: int, state: str,
    nominal_surge: float = 1.0, safe_surge: float = 1.0,
    nominal_sway: float = 0.0, safe_sway: float = 0.0,
    nominal_yaw: float = 0.0, safe_yaw: float = 0.0,
) -> list[str]:
    row = [""] * 17
    row[COL["timestamp_s"]] = str(ts)
    row[COL["max_obstacle_risk"]] = str(risk)
    row[COL["obstacle_count"]] = str(count)
    row[COL["nominal_surge"]] = str(nominal_surge)
    row[COL["nominal_sway"]] = str(nominal_sway)
    row[COL["nominal_yaw_rate"]] = str(nominal_yaw)
    row[COL["safe_surge"]] = str(safe_surge)
    row[COL["safe_sway"]] = str(safe_sway)
    row[COL["safe_yaw_rate"]] = str(safe_yaw)
    row[COL["planner_state"]] = state
    return row


def test_analyze_empty_rows() -> None:
    stats = analyze(_make_full_header(), [])
    assert stats["num_samples"] == 0
    assert stats["duration_s"] == 0.0
    assert stats["max_obstacle_risk"] == 0.0


def test_analyze_all_normal() -> None:
    rows = [_make_full_row(ts=i, risk=0.1, count=0, state="NORMAL") for i in range(20)]
    stats = analyze(_make_full_header(), rows)
    assert stats["num_samples"] == 20
    assert stats["max_obstacle_risk"] == 0.1
    assert stats["obstacle_samples"] == 0
    assert stats["non_normal_samples"] == 0
    assert stats["cmd_diff_samples"] == 0
    assert stats["first_high_risk_ts"] is None


def test_analyze_detects_avoidance() -> None:
    rows = []
    for i in range(10):
        rows.append(_make_full_row(ts=i, risk=0.2, count=0, state="NORMAL"))
    for i in range(10, 20):
        rows.append(_make_full_row(
            ts=i, risk=0.8, count=2, state="AVOIDING",
            nominal_sway=0.0, safe_sway=-0.6,
            nominal_yaw=0.0, safe_yaw=-0.4,
        ))
    stats = analyze(_make_full_header(), rows)
    assert stats["num_samples"] == 20
    assert stats["max_obstacle_risk"] == 0.8
    assert stats["obstacle_samples"] == 10
    assert stats["non_normal_samples"] == 10
    assert stats["cmd_diff_samples"] == 10
    assert stats["first_high_risk_ts"] == 10.0
    assert stats["first_non_normal_ts"] == 10.0
    assert stats["first_cmd_diff_ts"] == 10.0
    assert stats["max_safe_sway"] == 0.6
    assert stats["max_safe_yaw_rate"] == 0.4


def test_analyze_collects_plot_data() -> None:
    rows = [_make_full_row(ts=i, risk=float(i) / 10, count=1, state="NORMAL") for i in range(15)]
    stats = analyze(_make_full_header(), rows)
    assert len(stats["timestamps"]) == 15
    assert len(stats["risks"]) == 15
    assert len(stats["safe_sways"]) == 15
    assert len(stats["safe_yaws"]) == 15


# ---------------------------------------------------------------------------
# Column index constants
# ---------------------------------------------------------------------------

def test_col_indices_cover_expected_keys() -> None:
    expected_keys = {
        "timestamp_s", "rov_x", "rov_y", "rov_z", "obstacle_count",
        "max_obstacle_risk", "most_dangerous_center_x", "most_dangerous_bearing_rad",
        "nominal_surge", "nominal_sway", "nominal_yaw_rate",
        "safe_surge", "safe_sway", "safe_yaw_rate",
        "planner_state", "selected_side", "debug_risk",
    }
    assert set(COL.keys()) == expected_keys


def test_col_indices_within_17_columns() -> None:
    for idx in COL.values():
        assert 0 <= idx < 17
