"""Generate trajectory and time-series plots for the Scientific Baseline 0
campaign.

Reads `experiments/simulation/scientific_baseline_0/` (or --campaign path)
and writes PNGs next to the manifest. Uses only the validator JSONs — every
figure is regenerable from committed artifacts.

Run with any python that has matplotlib (ros2_lyrical env).
"""

from __future__ import annotations

import argparse
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_runs(root):
    runs = []
    runs_dir = os.path.join(root, "runs")
    for name in sorted(os.listdir(runs_dir)):
        p = os.path.join(runs_dir, name, "validation.json")
        if os.path.isfile(p):
            with open(p) as f:
                runs.append((name, json.load(f)))
    return runs


def plot_trajectories(runs, out_path):
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), sharey=True)
    by_scenario = {"A": [], "B": [], "C": []}
    for name, m in runs:
        by_scenario.setdefault(name.split("_")[0], []).append((name, m))
    for ax, sc in zip(axes, ("A", "B", "C")):
        ax.set_title(f"Scenario {sc}")
        for name, m in by_scenario.get(sc, []):
            xs = [r["gt_dx"] for r in m.get("odo_series", [])]
            ys = [r["gt_dy"] for r in m.get("odo_series", [])]
            ox = [r["odo_dx"] for r in m.get("odo_series", [])]
            oy = [r["odo_dy"] for r in m.get("odo_series", [])]
            ax.plot(xs, ys, "-", linewidth=1.6, label=f"{name} GT")
            ax.plot(ox, oy, "--", linewidth=1.0, label=f"{name} odo")
            for ob in m.get("obstacle_geometry", []):
                # Obstacle position relative to the run's start pose:
                # obstacles were spawned at [12, 0] body-relative and runs
                # start at yaw~0, so plot in start-relative coordinates.
                circ = plt.Circle((12.0, 0.0), ob["radius_m"],
                                  color="crimson", alpha=0.25)
                ax.add_patch(circ)
            mcp = m.get("min_clearance_point")
            if mcp and m.get("obstacle_geometry"):
                pass  # world coords; start-relative marker below is enough
        ax.axhline(0.0, color="gray", linewidth=0.6, linestyle=":")
        ax.set_xlabel("forward [m]")
        ax.grid(alpha=0.3)
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(fontsize=6)
    axes[0].set_ylabel("lateral (left+) [m]")
    fig.suptitle("Scientific Baseline 0 — GT vs transferable odometry "
                 "(start-relative), obstacle in red")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_timeseries(runs, out_path):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for name, m in runs:
        if not name.startswith(("B", "C")):
            continue
        dyn = m.get("dyn_series", [])
        t = [r["t"] for r in dyn]
        axes[0].plot(t, [r["target_surge"] for r in dyn], alpha=0.5,
                     linewidth=0.8)
        axes[0].plot(t, [r["ach_surge"] for r in dyn], linewidth=1.2,
                     label=name)
        axes[1].plot(t, [r["ach_sway"] for r in dyn], linewidth=1.0,
                     label=name)
        axes[2].plot(t, [r["peak_force"] for r in dyn], linewidth=1.0,
                     label=name)
    axes[0].set_ylabel("surge [m/s]\n(thin=target)")
    axes[1].set_ylabel("sway [m/s]")
    axes[2].set_ylabel("peak |thrust| [N]")
    axes[2].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=6, ncol=3)
    fig.suptitle("Scientific Baseline 0 — command tracking and thruster "
                 "utilization (scenarios B/C)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def plot_odo_error(runs, out_path):
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for name, m in runs:
        srs = m.get("odo_series", [])
        ax.plot([r["t"] for r in srs], [r["err"] for r in srs],
                linewidth=1.1, label=name)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|GT - odo| planar error [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=3)
    ax.set_title("Transferable no-DVL dead-reckoning error vs ground truth")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "experiments", "simulation", "scientific_baseline_0"))
    args = parser.parse_args()

    runs = load_runs(args.campaign)
    if not runs:
        print("no runs found"); return 1
    plot_trajectories(runs, os.path.join(args.campaign, "fig_trajectories.png"))
    plot_timeseries(runs, os.path.join(args.campaign, "fig_timeseries.png"))
    plot_odo_error(runs, os.path.join(args.campaign, "fig_odo_error.png"))
    print("wrote 3 figures to", args.campaign)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
