# Scientific Sim-to-Real Roadmap — Master Checklist

Persistent execution checklist for turning HoloObstacleAvoidance into a rigorous,
reproducible, publishable sim-to-real underwater obstacle avoidance study.

**Baseline commit:** `6692f07` (origin/main, "update") — contains the final YOLO
committed-circumnavigation demo with estimated odometry.
**Working branch:** `feature/scientific-sim-to-real-obstacle-avoidance`
**Rules:** update this file at every milestone; never rely on conversation history.
Methodological decisions go to `docs/SCIENTIFIC_DECISIONS.md`.

Legend: `[x]` done+tested · `[~]` in progress · `[ ]` open · `[B]` blocked (see Blockers)

---

## Phase 1 — Freeze / audit

- [x] Locate the real repository state (repo was NOT checked out locally; fresh clone
      from `AndreaBedei1/HoloObstacleAvoidance`; `main` is ahead of
      `feature/holoocean-anchor-primitives` and is the true latest state)
- [x] Create branch `feature/scientific-sim-to-real-obstacle-avoidance` from `6692f07`
- [x] Full-architecture audit (7 parallel subsystem deep-reads; results summarized in
      `docs/SCIENTIFIC_DECISIONS.md` D-002)
- [x] Verify weakness 1: custom scenarios use `HoveringAUV`
      (`holoocean_sim_server.py:166`, all `config/holoocean_scenarios/*.yaml`)
- [x] Verify weakness 2: scientific motion uses kinematic `teleport`
      (`holoocean_sim_server.py:1063-1079`, `agent.teleport()` every tick)
- [x] Verify ground-truth separation: planner subscribes only
      `/perception/obstacles`, `/cmd_vel_nominal`, `/rov/odom_estimated`
      (`local_avoidance_planner_node.py:42-44`); enforcement is convention + text
      tests only — runtime guard still to add (see Phase 9)
- [x] Create this roadmap and `docs/SCIENTIFIC_DECISIONS.md`
- [~] Run existing unit test suite on this machine (ROS 2 lyrical + colcon)
- [B] Re-run current YOLO committed-circumnavigation closed loop 3× from fresh engine
      processes — **blocked on this machine**: custom Unreal world (cooked
      `ancora/mina/siluro` assets) and trained `best.pt` weights exist only on the
      lab machine (`andrea.bedei3` paths). See Blockers.

## Phase 2 — Literature / novelty

- [~] Structured literature sweep (8 topic clusters, primary sources, 2022–2026)
- [ ] `docs/LITERATURE_AND_NOVELTY.md` with full comparison table
- [ ] `paper/references.bib`
- [ ] Five closest papers identified
- [ ] Novelty threat analysis (per-contribution verdict)
- [ ] Classical baseline planner selected and justified
      (`docs/CLASSICAL_PLANNER_BASELINE.md`)

## Phase 3 — Safe read-only real hardware inventory (NO actuation)

- [x] BlueROV2 discovered and inventoried read-only (BlueOS 1.3.1, ArduSub 4.1.2,
      Navigator, FRAME_CONFIG=2 → Heavy 8-thruster)
- [x] Camera stream identified (H.264 USB, 1920×1080@30, UDP 5600)
- [x] DVL presence check → **NO DVL** (no DVL extension, no velocity/odometry
      MAVLink messages) — see `docs/SIM_REAL_SENSOR_EQUIVALENCE.md`
- [x] Ping1D presence confirmed (config only, `/dev/ttyAMA3`) — NOT used, dry
- [x] Imaging sonar identified (Cerulean SonarView + STM device at its default IP)
      — **prohibited, never used**
- [x] Position-locator injection path identified (NMEA→UDP 27000→GPS_INPUT
      component 220; topside receiver not currently connected; no fix while dry)
- [x] RealSense inventoried: D435, FW/profiles/intrinsics/extrinsics captured,
      1080p RGB measured 29.8 fps (`scripts/inspect_realsense.py`,
      `visualizations/realsense_inventory/`)
- [x] `docs/REAL_BLUEROV2_HARDWARE_INVENTORY.md`
- [x] `config/real_bluerov2_detected.yaml` (sanitized)
- [x] `docs/SIM_REAL_SENSOR_EQUIVALENCE.md` (critical DVL mismatch documented)
- [x] `docs/REALSENSE_GROUND_TRUTH_PLAN.md`
- [ ] Camera-to-body transform measured (needs physical access / calibration)
- [ ] Onboard camera intrinsics + underwater FOV calibration (needs wet access)

## Phase 4 — Official HoloOcean ROS 2 bridge audit

- [ ] Compare official bridge vs custom two-process TCP bridge
- [ ] `docs/HOLOOCEAN_ROS2_BRIDGE_DECISION.md`

## Phase 5 — BlueROV2 vehicle + real dynamics in simulation

- [x] Verify holoocean 2.3.0 ships a `BlueROV2` agent = **Heavy, 8 thrusters,
      vectored-6DOF layout matching real FRAME_CONFIG=2**; control schemes:
      0=thruster forces, 1=PID, 2=accelerations (+ Fossen custom dynamics)
- [x] Smoke test on THIS machine: BlueROV2 + RGBCamera/IMU/DVL/Depth/Pose/Velocity
      sensors spawn and tick in local prebuilt Ocean world (SimpleUnderwater),
      8-thruster command accepted
- [ ] Scientific scenarios switched from HoveringAUV to BlueROV2
- [ ] Dynamics-based motion mode (body-velocity controller → thruster allocation),
      saturation + rate limiting + configurable latency
- [ ] Teleport demoted to `motion_model:=legacy_teleport` (regression tests only)
- [ ] Step-response experiments: surge, sway, yaw (+heave) logged
- [ ] `docs/BLUEROV2_HOLOOCEAN_INTEGRATION.md`, `docs/BLUEROV2_SIMULATION_DYNAMICS.md`
- [B] Custom anchor/torpedo/mine assets in a BlueROV2 scenario — cooked custom
      world lives on the lab machine; needs asset transfer or re-cook (see Blockers)

## Phase 6 — Reproduce committed avoidance under real dynamics

- [ ] Current planner (Baseline A, `planner_mode:=committed_baseline`) closes the
      loop with BlueROV2 dynamics — 3 consecutive fresh-process successes

## Phase 7 — Temporal obstacle estimation

- [ ] T0 raw (existing), T1 hold/EMA, T2 fixed KF, T3 calibrated adaptive estimator
- [ ] Deterministic replay harness + synthetic-sequence unit tests
- [ ] Residual-statistics calibration vs sim oracle (offline only)
- [ ] `docs/OBSTACLE_UNCERTAINTY_CALIBRATION.md` + machine-readable calibration

## Phase 8 — Classical planner baseline

- [ ] Implement literature-selected baseline (candidate: DWA) on same perception input

## Phase 9 — Proposed uncertainty-aware method + safety interlocks

- [ ] Real-control adapter with `vehicle_in_water:=false` AND
      `allow_real_actuation:=false` defaults; `real_control_mode:=shadow`;
      interlock unit tests; no launch path can actuate by default
- [ ] Runtime guard: planner nodes must not subscribe `/ground_truth/*`
- [ ] Uncertainty-aware committed circumnavigation (formulation frozen only after
      novelty audit)

## Phase 10 — Simulation experiment campaign

- [ ] Scenario matrix (anchor/torpedo/mine × 3 approaches × variations), fixed seeds
- [ ] ≥20 valid runs per main condition; failures recorded
- [ ] `experiments/simulation/<campaign_id>/` structure + per-run JSON + stats scripts

## Phase 11 — Real digital twin / ground truth preparation

- [ ] `config/pool_digital_twin.yaml` (placeholders `TODO_MEASURE_REAL_POOL`)
- [ ] `docs/POOL_MEASUREMENT_PROTOCOL.md`
- [ ] `rov_ground_truth` package (`/ground_truth/external/*`, isolated from control)
- [ ] Multi-depth refraction-aware calibration tooling (desk-testable now)
- [ ] `docs/REAL_POOL_EXPERIMENT_PROTOCOL.md` (validity criteria pre-registered)
- [ ] `docs/REAL_WET_TEST_SAFETY_CHECKLIST.md`

## Paper

- [ ] `paper/outline.md`, `paper/contributions.md`, `paper/references.bib`,
      `paper/figures/README.md`
- [ ] `docs/PUBLICATION_POSITIONING.md`

---

## Blockers (require external action)

| # | Blocker | Needed from |
|---|---------|-------------|
| B1 | Custom Unreal world with cooked `ancora/mina/siluro` assets exists only at `C:/Users/andrea.bedei3/Desktop/Holo/MondoTest/HoloOcean/engine` on the lab machine; no UE 5.3 editor installed here | Andrea: copy the cooked engine/world folder (or its packaged build) to this machine, or provide UE 5.3 + project to re-cook |
| B2 | Trained YOLO weights `training/yolo_custom_objects/runs/.../best.pt` are gitignored and absent here | Andrea: copy `best.pt` (and ideally the dataset) from the lab machine |
| B3 | Pool measurements, RealSense mounting, obstacle dimensions | pool access |
| B4 | Any wet/actuation test | Andrea's explicit "ROV in water" authorization |

## Standing safety rules (do not remove)

- Real vehicle: READ-ONLY until explicit in-water authorization. No arming, no
  motor/RC/velocity/actuator commands, no parameter writes, no reboots.
- Imaging/side-scan sonar (Cerulean Omniscan-class): NEVER used, in or out of water.
- Ping1D: untouched until explicit in-water authorization; not a dependency of the
  first paper.
- External RealSense ground truth must never feed YOLO, the tracker, the planner,
  or command generation.
