#!/usr/bin/env python
"""Analyze a CSV recording produced by oracle_demo_recorder_node.

Reads the CSV and prints a short validation report with pass/fail checks.
Optionally saves a plot if matplotlib is available.

Usage:
    python scripts/analyze_oracle_recording.py [csv_path]

Defaults to logs/oracle_demo_record.csv when no argument is given.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

DEFAULT_CSV = "logs/oracle_demo_record.csv"

# Column indices matching CSV_HEADER in oracle_demo_recorder_node.py
COL = {
    "timestamp_s": 0,
    "rov_x": 1,
    "rov_y": 2,
    "rov_z": 3,
    "obstacle_count": 4,
    "max_obstacle_risk": 5,
    "most_dangerous_center_x": 6,
    "most_dangerous_bearing_rad": 7,
    "nominal_surge": 8,
    "nominal_sway": 9,
    "nominal_yaw_rate": 10,
    "safe_surge": 11,
    "safe_sway": 12,
    "safe_yaw_rate": 13,
    "planner_state": 14,
    "selected_side": 15,
    "debug_risk": 16,
}

CMD_DIFF_THRESHOLD = 0.01


def read_csv(path: str) -> tuple[list[str], list[list[str]]]:
    """Return (header, rows) from the CSV file."""
    csv_path = Path(path)
    if not csv_path.exists():
        print(f"FAIL: CSV file not found: {csv_path}")
        sys.exit(1)

    with csv_path.open("r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            print("FAIL: CSV file is empty.")
            sys.exit(1)
        rows = list(reader)

    if len(rows) == 0:
        print("FAIL: CSV contains header but no data rows.")
        sys.exit(1)

    return header, rows


def _float(row: list[str], idx: int) -> float:
    """Safely convert a CSV cell to float."""
    try:
        return float(row[idx])
    except (ValueError, IndexError):
        return 0.0


def _cmd_differs(row: list[str]) -> bool:
    """Return True if safe command differs from nominal by more than threshold."""
    for n_idx, s_idx in [
        (COL["nominal_surge"], COL["safe_surge"]),
        (COL["nominal_sway"], COL["safe_sway"]),
        (COL["nominal_yaw_rate"], COL["safe_yaw_rate"]),
    ]:
        if abs(_float(row, n_idx) - _float(row, s_idx)) > CMD_DIFF_THRESHOLD:
            return True
    return False


def analyze(header: list[str], rows: list[list[str]]) -> dict:
    """Compute all report metrics from parsed CSV rows."""
    num_samples = len(rows)
    duration = _float(rows[-1], COL["timestamp_s"]) if rows else 0.0

    max_risk = 0.0
    obstacle_samples = 0
    non_normal_samples = 0
    cmd_diff_samples = 0
    max_safe_sway = 0.0
    max_safe_yaw = 0.0

    first_high_risk_ts: float | None = None
    first_non_normal_ts: float | None = None
    first_cmd_diff_ts: float | None = None

    timestamps: list[float] = []
    risks: list[float] = []
    safe_sways: list[float] = []
    safe_yaws: list[float] = []

    for row in rows:
        ts = _float(row, COL["timestamp_s"])
        risk = _float(row, COL["max_obstacle_risk"])
        count = _float(row, COL["obstacle_count"])
        state = row[COL["planner_state"]] if len(row) > COL["planner_state"] else ""

        timestamps.append(ts)
        risks.append(risk)
        safe_sways.append(_float(row, COL["safe_sway"]))
        safe_yaws.append(_float(row, COL["safe_yaw_rate"]))

        if risk > max_risk:
            max_risk = risk

        if count > 0:
            obstacle_samples += 1

        if state != "NORMAL":
            non_normal_samples += 1
            if first_non_normal_ts is None:
                first_non_normal_ts = ts

        if _cmd_differs(row):
            cmd_diff_samples += 1
            if first_cmd_diff_ts is None:
                first_cmd_diff_ts = ts

        if risk > 0.55 and first_high_risk_ts is None:
            first_high_risk_ts = ts

        sway_abs = abs(_float(row, COL["safe_sway"]))
        if sway_abs > max_safe_sway:
            max_safe_sway = sway_abs

        yaw_abs = abs(_float(row, COL["safe_yaw_rate"]))
        if yaw_abs > max_safe_yaw:
            max_safe_yaw = yaw_abs

    return {
        "num_samples": num_samples,
        "duration_s": duration,
        "max_obstacle_risk": max_risk,
        "obstacle_samples": obstacle_samples,
        "non_normal_samples": non_normal_samples,
        "cmd_diff_samples": cmd_diff_samples,
        "max_safe_sway": max_safe_sway,
        "max_safe_yaw_rate": max_safe_yaw,
        "first_high_risk_ts": first_high_risk_ts,
        "first_non_normal_ts": first_non_normal_ts,
        "first_cmd_diff_ts": first_cmd_diff_ts,
        "timestamps": timestamps,
        "risks": risks,
        "safe_sways": safe_sways,
        "safe_yaws": safe_yaws,
    }


def print_report(stats: dict) -> None:
    """Print the validation report to stdout."""
    sep = "=" * 60

    print(sep)
    print("ORACLE RECORDING ANALYSIS REPORT")
    print(sep)
    print(f"  Samples              : {stats['num_samples']}")
    print(f"  Recording duration   : {stats['duration_s']:.2f} s")
    print(f"  Max obstacle risk    : {stats['max_obstacle_risk']:.4f}")
    print(f"  Samples w/ obstacles : {stats['obstacle_samples']}")
    print(f"  Non-NORMAL samples   : {stats['non_normal_samples']}")
    print(f"  Safe != nominal      : {stats['cmd_diff_samples']}")
    print(f"  Max |safe_sway|      : {stats['max_safe_sway']:.4f}")
    print(f"  Max |safe_yaw_rate|  : {stats['max_safe_yaw_rate']:.4f}")

    def _ts_label(v: float | None) -> str:
        return f"{v:.3f} s" if v is not None else "N/A"

    print(f"  First risk > 0.55    : {_ts_label(stats['first_high_risk_ts'])}")
    print(f"  First non-NORMAL     : {_ts_label(stats['first_non_normal_ts'])}")
    print(f"  First safe!=nominal  : {_ts_label(stats['first_cmd_diff_ts'])}")

    # --- Pass / Fail checks ---
    print()
    checks = [
        (stats["num_samples"] >= 10, f"At least 10 samples (got {stats['num_samples']})"),
        (
            stats["obstacle_samples"] > 0,
            f"At least one obstacle detected (got {stats['obstacle_samples']} samples)",
        ),
        (
            stats["max_obstacle_risk"] > 0.55,
            f"Max risk exceeds 0.55 (got {stats['max_obstacle_risk']:.4f})",
        ),
        (
            stats["non_normal_samples"] > 0,
            f"Planner leaves NORMAL at least once (got {stats['non_normal_samples']} samples)",
        ),
        (
            stats["cmd_diff_samples"] > 0,
            f"/planner/cmd_vel_safe differs from /cmd_vel_nominal at least once "
            f"(got {stats['cmd_diff_samples']} samples)",
        ),
    ]

    all_pass = True
    for passed, reason in checks:
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_pass = False
        print(f"  [{status}] {reason}")

    print()
    if all_pass:
        print("OVERALL: PASS — All validation checks succeeded.")
    else:
        print("OVERALL: FAIL — One or more validation checks failed.")
    print(sep)


def save_plot(stats: dict, output_path: str = "logs/oracle_demo_record_plot.png") -> bool:
    """Attempt to save a plot using matplotlib. Return True on success."""
    try:
        import matplotlib.pyplot as plt
        import matplotlib  # noqa: F811
    except ImportError:
        return False

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    ts = stats["timestamps"]

    axes[0].plot(ts, stats["risks"], color="red", linewidth=1)
    axes[0].axhline(y=0.55, color="orange", linestyle="--", label="Enter threshold")
    axes[0].set_ylabel("Max obstacle risk")
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(ts, stats["safe_sways"], color="green", linewidth=1)
    axes[1].set_ylabel("safe_sway")
    axes[1].grid(True)

    axes[2].plot(ts, stats["safe_yaws"], color="blue", linewidth=1)
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("safe_yaw_rate")
    axes[2].grid(True)

    fig.suptitle("Oracle Recording Analysis")
    fig.tight_layout()

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)

    print(f"Plot saved to {out}")
    return True


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CSV
    header, rows = read_csv(csv_path)
    stats = analyze(header, rows)
    print_report(stats)
    save_plot(stats)


if __name__ == "__main__":
    main()
