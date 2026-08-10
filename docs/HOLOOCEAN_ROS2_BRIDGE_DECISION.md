# HoloOcean ROS 2 Bridge — Official vs Custom, Decision

Date: 2026-08-10. Status: **retain the custom two-process TCP bridge**;
align conventions where cheap. Revisit only if we migrate the stack to Linux.

## What exists officially

1. **Migeran `holoocean_ros`** (github.com/migeran/holoocean_ros, 2023,
   presented at the OSRF Maritime WG): rclpy-based bridge using the HoloOcean
   Python API in-process; thruster-control messages; HoveringAUV URDF;
   ImagingSonar→PointCloud via a PyBind C++ helper on a background thread.
2. **HoloOcean ≥2.x in-tree ROS 2 support** (documented in the HoloOcean 2.0
   preview, arXiv:2510.06160, and used by BYU FRoStLab for HIL — Meyers et al.,
   OCEANS 2025): example rclpy nodes that publish sensors and accept commands
   from the same Python process that owns the HoloOcean env.

Both designs assume **rclpy and holoocean import into the same interpreter**.

## Comparison against our bridge

| Criterion | Official (either) | Ours (two-process TCP) |
|---|---|---|
| HoloOcean version | 2.x in-tree; Migeran pinned to older 1.x API | 2.3.0 (installed) — works |
| ROS 2 distro | Linux-first examples (Humble-era; 2.x docs) | ROS 2 lyrical on Windows — works, tested |
| **Windows + split interpreters** | **Blocking problem**: our HoloOcean env is conda Python 3.9; ROS 2 lyrical binaries pin Python 3.12.3 — one interpreter cannot host both | Explicitly designed for this: sim server (py3.9) ↔ ROS node (py3.12) over localhost TCP |
| Custom Unreal world + SpawnAsset | Not supported (stock API only) | Supported (custom_engine_launcher + direct commands) |
| BlueROV2 agent | Supported (stock API) | Supported (this repo, verified) |
| Sensor publishing | Sensors incl. sonar→PointCloud (we must NOT use sonar anyway) | Camera/pose/velocity/depth + oracle; extensible |
| /clock, TF, image-stamp propagation | Examples minimal; no /clock authority either | Missing too — tracked as our own work item |
| Latency control | rclpy in sim loop; sonar conversion threaded | `read_latest()` drop-stale policy both directions; measured |
| Lifecycle/reproducibility | External dependency, examples not versioned for our stack | In-repo, unit-tested (protocol + node tests), CI-able |
| Maintenance cost | Adopting = porting our custom-engine + oracle + scenario layer into it | Keeping = we own ~1 file of protocol code already covered by tests |

## Decision

Retain the custom bridge. Rationale:

1. The official designs presuppose a shared interpreter; on this project's
   Windows machines the HoloOcean and ROS 2 environments cannot share one.
   Adopting the official bridge would require moving the whole experimental
   stack to Linux — an infrastructure rewrite with zero scientific payoff.
2. The bridge is NOT part of the scientific claims (see D-008); switching
   would consume effort exactly where the novelty audit says not to invest.
3. Our custom-engine/SpawnAsset layer (anchor/torpedo/mine world) has no
   official equivalent.

## Convention alignment (adopted as work items)

- Propagate source-image stamps end-to-end (bridge → detector output) instead
  of publish-time stamps — required for latency-aware science regardless.
- Name topics/frames per REP-103/REP-105 where they already match (client
  body frame is x-fwd/y-left/z-up = REP-103 — no conversion needed).
- Consider publishing `/clock` from sim ticks in a later S-level if
  sim-time-driven evaluation proves necessary.

Sources: [Migeran holoocean_ros](https://github.com/migeran/holoocean_ros) ·
[Migeran blog](https://migeran.com/blog/holoocean-simulator-ros2-bridge) ·
[OSRF Discourse announcement](https://discourse.openrobotics.org/t/holoocean-underwater-robotics-simulator-ros2-bridge/32705) ·
[HoloOcean 2.0 preview](https://arxiv.org/abs/2510.06160) ·
Meyers et al., OCEANS 2025 (HoloOcean HIL), doi:10.23919/OCEANS59106.2025.11245188.
