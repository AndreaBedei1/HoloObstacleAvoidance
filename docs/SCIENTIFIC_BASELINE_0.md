# Scientific Baseline 0 — Dynamics Closed Loop

**Label: `simulation dynamics integration baseline` — NOT a visual obstacle
avoidance result.** Obstacle perception is the simulator oracle (relay) on a
primitive stand-in obstacle; the point of this milestone is the physics,
control, navigation, and safety plumbing, validated end-to-end.

## Architecture (what runs where)

```
/cmd_vel_nominal (10 Hz, surge 0.3)
  -> local_avoidance_planner (CURRENT committed baseline, UNCHANGED)
  -> /planner/cmd_vel_safe
  -> holoocean_bridge (TCP) -> sim server (holoocean env)
       command watchdog (1.0 s) -> latency buffer -> setpoint rate limiter
       -> PI body-velocity + PD attitude controller
       -> 8-thruster allocation (engine-verified geometry, ±28.75 N)
       -> agent.act -> UE rigid-body physics (BlueROV2 Heavy)
  <- PoseSensor/VelocitySensor/IMU/Depth (30 Hz, real-time paced)
```

### Runtime information (transferable — real counterparts exist)

| Topic | Content | Real counterpart |
|---|---|---|
| `/rov/attitude_measured` | roll, pitch, yaw | ArduSub ATTITUDE (EKF/compass) |
| `/rov/depth` | pressure depth | SCALED_PRESSURE2 |
| `/planner/cmd_vel_safe` | commanded body velocity | same topic on the real stack |
| `/perception/obstacles` | image-space obstacles (oracle here; YOLO later) | YOLO on onboard camera |
| `/rov/odom_estimated` | **commanded-motion dead reckoning** (below) | same estimator, same inputs |

### Validator-only information (never enters the control path)

`/rov/pose_ground_truth`, `/sim/obstacles_world`, `/sim/dynamics_debug`,
`/sim/dropout_debug`, `/perception/obstacles_oracle` (relay input side).
Enforcement: `assert_topics_allowed` guard in the estimator (unit-tested),
planner subscription set unchanged (3 topics), validator is the only GT
consumer in the launch.

## No-DVL transferable navigation (D-006 resolution)

The real BlueROV2 has NO DVL and no velocity sensor. Runtime odometry is
`commanded_odometry_node`: commanded body velocity passed through a
first-order vehicle-response model (τ surge/sway/heave = 1.2/1.15/0.8 s from
the step-response identification), integrated with the MEASURED yaw; z is the
depth MEASUREMENT. Command staleness (>1 s) decays the model target to zero,
mirroring the vehicle-side watchdog. The sim `VelocitySensor` is used ONLY
inside the simulated low-level PI thruster controller (part of the simulated
plant, mirroring the real thruster ESC/vehicle response); it never reaches
the estimator (forbidden-topic guard + tests).

## Command watchdog

Server-side: no fresh `cmd_vel` frame within `cmd_timeout_s` (default 1.0 s,
`<=0` disables) → commanded body velocity zeroed until a fresh command
arrives (smooth stop through the setpoint rate limiter). Trips are logged,
exposed in the state header (`watchdog_active`) and counted by the validator.
8 unit tests (`test_command_watchdog.py`).

## Physics-time consistency (D-010 — critical infrastructure finding)

Two silent clock traps were found and fixed while building this baseline:

1. Windows sleep quantization (~15.6 ms) capped the serve loop below the tick
   rate → sim time slower than wall time. Fixed with `timeBeginPeriod(1)` +
   hybrid sleep/spin pacing.
2. **UE physics advances at most 1/30 s per frame**: at `ticks_per_sec: 20`
   the vehicle moved at 0.646× its reported velocity (declared sim time ≠
   physics time). Scientific scenarios MUST use `ticks_per_sec: 30`.

With both fixes the sim runs 30 Hz real-time (tick cost ≈22 ms, 256² camera,
no viewport) and wall-clock == sim-time == physics-time, which is the
precondition for wall-clock ROS estimators and for latency calibration.
Diagnostic evidence: `experiments/simulation/baseline0_diag*`.

## Campaign

`scripts/run_baseline0_campaign.py` — scenarios:

- **A** straight, no obstacle (45 s): controller stability, watchdog,
  odometry drift.
- **B** one deterministic obstacle 12 m ahead (120 s): full committed
  circumnavigation on the CURRENT planner.
- **C** = B + deterministic detector dropout (trigger: planner enters
  AVOIDING, +1.0 s, duration 2.0 s; relay goes silent like a YOLO miss).

Every run: fresh sim-server/engine process, fresh zenoh router, fresh ROS 2
launch. Pre-registered acceptance criteria in the orchestrator
(`acceptance()`); per-run validator JSON with full metric set + decimated
diagnostic time series. Obstacle clearance is computed from **explicit known
obstacle geometry** (`/sim/obstacles_world`: center + radius 1.75 m) because
the primitive test world has no mesh-collision report — documented
limitation, physical contact would appear as trajectory perturbation.

## Results (campaign of 2026-08-11, commit 2b2dd51, 9/9 accepted)

All runs from fresh engine/router/launch processes. Full data:
`experiments/simulation/scientific_baseline_0/` (manifest + per-run JSON +
figures). The first attempt (6/9) is preserved unmodified in
`scientific_baseline_0_attempt1/` — see "Failures" below.

| Metric | A (straight, n=3) | B (avoidance, n=3) | C (avoidance+dropout, n=3) |
|---|---|---|---|
| Accepted | 3/3 | 3/3 | 3/3 |
| Collision | n/a | 0/3 | 0/3 |
| Min clearance to obstacle surface [m] | n/a | 0.672–0.712 | 0.688–0.715 |
| Forward progress [m] | 13.4–14.0 | 34.9–35.0 | 34.4–35.0 |
| Max lateral deviation [m] | 0.000 | 2.43–2.48 | 2.45–2.47 |
| Final lateral error [m] | 0.000 | 0.019–0.058 | 0.016–0.045 |
| Final yaw error [deg] | 0.0 | 0.0 | 0.0 |
| Returned to original line | — | 3/3 | 3/3 |
| Avoidance entries / side switches | 0 / 0 | 1 / 0 | 1 / 0 |
| Maneuver time [s] | — | 70.6–70.8 | 70.7–70.8 |
| Odometry error max (whole run) [m] | 0.106–0.117 | 0.174–0.193 | 0.172–0.198 |
| Thruster peak [N] (of 28.75) | 2.99 | 3.84 | 3.84 |
| Saturation / watchdog trips | 0 / 0 | 0 / 0 | 0 / 0 |

State sequence in every B/C run: `NORMAL → APPROACH_OBSTACLE → AVOIDING_LEFT
→ RECOVERING → NORMAL`, exactly one maneuver, no side reversal. In every C
run the deterministic dropout (2 s of relay silence starting 3 s after
AVOIDING) was injected and survived: no premature recovery, no side switch,
no collision, normal recovery.

Repeatability: not bit-identical (distributed wall-clock scheduling), but
tight — clearance spread 4 cm, maneuver-time spread 0.3 s, final lateral
spread 4 cm across fresh processes.

Figures: `fig_trajectories.png` (GT vs transferable odometry, obstacle,
committed go-around shape), `fig_timeseries.png` (command tracking, thruster
utilization), `fig_odo_error.png` (dead-reckoning error).

## Failures (attempt 1 — preserved, analyzed, fixed)

The first campaign attempt scored 6/9. All three failures were diagnosed:

- **A_2 (infrastructure):** the bridge's zenoh session started isolated (one
  state message then silence) — vehicle never moved. Fix: orchestrator
  graph-liveness gate (validator must report bridge traffic within 30 s) with
  one launch retry; `ZENOH_ROUTER_CHECK_ATTEMPTS` set for all nodes.
- **B_2 (startup race):** the planner engaged 0.07 s after start, before the
  odometry node was publishing → pose-less fallback maneuver diverged 17 m.
  Fix: the perception relay now gates on nav liveness (no obstacles reach the
  planner until `/rov/odom_estimated` and `/rov/attitude_measured` are alive)
  — an integration-order guarantee the real stack will need too.
- **C_3 (genuine current-planner weakness, kept on record):** with dropout
  injected only ~1 s after commitment, 1 s of relay SILENCE (stale
  detections, unlike the fresh-empty-array dropout of the unit tests) made
  the planner abort AVOIDING → re-engage on reacquisition → recover to a
  re-captured reference line 6.5 m off the original route. This is exactly
  the failure class the Phase 7 temporal estimators (hold/KF prediction
  through dropouts) are meant to fix; scenario C now injects at 3 s
  (mid-maneuver, per the design intent), and the early-dropout weakness is
  a documented motivating finding, not a hidden one.

## Acceptance

**Milestone ACCEPTED** against the pre-registered criteria (9/9 runs, all
exit-criterion items met). The `simulation dynamics integration baseline` is
established; next step is Phase 7 (temporal estimators T0–T3) on top of it.

## Limitations

- Oracle perception (not YOLO), primitive sphere obstacle (custom assets on
  the lab machine — blockers B1/B2).
- Stock `SimpleUnderwater` world, not the pool digital twin.
- No collision mesh contact signal; clearance from known geometry.
- Attitude/depth measurements are currently noise-free sim values; noise
  models are an S-level calibration task.
- Single obstacle; single approach geometry.

## Acceptance

*(set after the 3×3 campaign — see roadmap Phase 6 checklist)*
