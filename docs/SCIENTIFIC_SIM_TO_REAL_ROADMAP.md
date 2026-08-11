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

- [x] Structured literature sweep (8 topic clusters, primary sources, 2022–2026;
      63 papers: 35 deep-read + 28 catalogued; `literature/literature_review.json`)
- [x] `docs/LITERATURE_AND_NOVELTY.md` with full comparison table
- [x] `paper/references.bib` (63 entries; unverified ones flagged)
- [x] Five closest papers identified (Mari 2026, Han 2026, DUViN 2025,
      Bergantin 2024, Kadian 2020)
- [x] Novelty threat analysis: C1 ladder study clearly novel (headline);
      C2 calibrated estimator potentially novel (ablations mandatory);
      planner claim demoted; protocol/system dropped as claims (D-008)
- [x] Classical baseline planner selected: DWA per Eriksen 2016
      (`docs/CLASSICAL_PLANNER_BASELINE.md`, D-009)
- [ ] Full-text verification of 2 aggregator-verified threat papers
      (Li AOR 2026; Li JMSE 2024) before claim freeze

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

- [x] Compare official bridge vs custom two-process TCP bridge — **retain
      custom** (official designs need holoocean+rclpy in ONE interpreter;
      impossible on our Windows py3.9/py3.12 split; bridge is not a claim)
- [x] `docs/HOLOOCEAN_ROS2_BRIDGE_DECISION.md` (+ convention alignment items:
      image-stamp propagation, REP-103 naming)

## Phase 5 — BlueROV2 vehicle + real dynamics in simulation

- [x] Verify holoocean 2.3.0 ships a `BlueROV2` agent = **Heavy, 8 thrusters,
      vectored-6DOF layout matching real FRAME_CONFIG=2**; control schemes:
      0=thruster forces, 1=PID, 2=accelerations (+ Fossen custom dynamics)
- [x] Smoke test on THIS machine: BlueROV2 + RGBCamera/IMU/DVL/Depth/Pose/Velocity
      sensors spawn and tick in local prebuilt Ocean world (SimpleUnderwater),
      8-thruster command accepted
- [x] Scientific scenario switched to BlueROV2
      (`bluerov2_dynamics_smoke.yaml`; stock-world path added; custom-engine
      path accepts `agent_type: BlueROV2` for when the world is transferred)
- [x] Dynamics-based motion mode: PI body-velocity + PD attitude controller →
      pseudo-inverse 8-thruster allocation with engine-verified geometry
      (Python docstring geometry proven WRONG via open-loop probes + C++ source);
      saturation + setpoint rate limiting + configurable command latency
- [x] Teleport excluded from dynamics mode (initial placement only);
      legacy scenarios keep it for regression; `load_config` validates
- [x] Step-response experiments surge/sway/heave/yaw: all in spec
      (ss err ≤4.8%, no overshoot), **3/3 consecutive fresh-process runs,
      bit-identical summaries (deterministic)** —
      `experiments/simulation/step_response_S0*`
- [x] `docs/BLUEROV2_HOLOOCEAN_INTEGRATION.md`, `docs/BLUEROV2_SIMULATION_DYNAMICS.md`
- [ ] Command watchdog in the sim server (zero setpoint on cmd timeout)
- [B] Custom anchor/torpedo/mine assets in a BlueROV2 scenario — cooked custom
      world lives on the lab machine; needs asset transfer or re-cook (see Blockers)

## Phase 6 — Reproduce committed avoidance under real dynamics (Scientific Baseline 0)

- [x] Transferable no-DVL runtime odometry (`commanded_odometry_node`):
      commanded-motion dead reckoning + measured yaw + measured depth;
      forbidden-input guard; synthetic-DVL estimator NOT in the loop
- [x] Sim-server command watchdog (`cmd_timeout_s`, default 1 s) + 8 tests
- [x] Bridge: `/rov/attitude_measured` (runtime), `/sim/dynamics_debug`,
      `/sim/obstacles_world` (validator-only)
- [x] Oracle relay with deterministic state-triggered detector dropout
- [x] Baseline0 validator: physical metrics incl. clearance from known
      geometry, thruster utilization, odometry error, diagnostic series
- [x] Physics-time consistency established (D-010): tps=30 mandatory,
      real-time hybrid pacing; straight-run odo error 4.08 m → 0.117 m
- [x] Campaign A/B/C × 3 fresh-process runs: **9/9 accepted** (2026-08-11)
      — no collision, clearance 0.67–0.72 m, single committed maneuver, no
      side switch, return to line ≤6 cm, dropout survived in all C runs,
      odometry error ≤0.20 m; attempt-1 failures preserved and analyzed
      (`docs/SCIENTIFIC_BASELINE_0.md`)
- [x] Milestone Scientific Baseline 0 ACCEPTED — cleared to start Phase 7

## Phase 7 — Temporal obstacle estimation

- [x] Common interface: `/perception/obstacles_raw` → estimator (t0–t3) →
      `/perception/obstacles`; planner UNCHANGED; T0 inside the same framework
      (`rov_obstacle_tracking`, 29 unit tests)
- [x] Three upstream conditions explicit (fresh / fresh-empty / silence) —
      the C_3 lesson, unit- and case-tested
- [x] T0 raw, T1 hold/EMA (2.5 s), T2 fixed KF (CV image-space, χ² gating,
      2.5 s horizon), T3 adaptive framework (θ=0 default ≡ T2,
      calibrated=false — no fake coefficients; pending B1/B2)
- [x] Deterministic replay harness + controlled cases D0–D14 + metrics
      (NIS, coverage, ghost time, availability, reacquisition)
- [x] Calibration pipeline scripts (fit + consistency eval) — framework
      validated by smoke test; real residual data pending B1/B2
- [x] Offline D-case evaluation
      (`experiments/simulation/temporal_estimators/replay_eval/`)
- [~] Closed-loop campaign E0–E4 × T0–T3 × 3 fresh-process reps — running
- [ ] `docs/TEMPORAL_ESTIMATOR_RESULTS.md` + decision gate before DWA
- [ ] `docs/OBSTACLE_UNCERTAINTY_CALIBRATION.md` (real coefficients — after
      B1/B2 transfer)

## Phase 8 — Classical planner baseline

- [ ] Implement literature-selected baseline (candidate: DWA) on same perception input

## Phase 9 — Proposed uncertainty-aware method + safety interlocks

- [x] Real-control adapter package `rov_real_bridge`: fail-closed
      `SafetyInterlock` (defaults false/shadow; strict bool parsing; unknown
      mode → shadow; audit trail), shadow mode publishes `/real/shadow_cmd`,
      transmit layer INTENTIONALLY UNIMPLEMENTED (NotImplementedError +
      no-mavlink-import test); 12 unit tests green
- [x] Ground-truth guard helper `forbid_ground_truth_topics` used by the
      adapter (extend to planner nodes when rov_ground_truth lands)
- [ ] Extend guard to all control-path nodes at launch level
- [ ] Uncertainty-aware committed circumnavigation (formulation frozen only after
      novelty audit)

## Phase 10 — Simulation experiment campaign

- [ ] Scenario matrix (anchor/torpedo/mine × 3 approaches × variations), fixed seeds
- [ ] ≥20 valid runs per main condition; failures recorded
- [ ] `experiments/simulation/<campaign_id>/` structure + per-run JSON + stats scripts

## Phase 11 — Real digital twin / ground truth preparation

- [x] `config/pool_digital_twin.yaml` (placeholders `TODO_MEASURE_REAL_POOL`;
      no invented measurements)
- [x] `docs/POOL_MEASUREMENT_PROTOCOL.md`
- [ ] `rov_ground_truth` package (`/ground_truth/external/*`, isolated from control)
- [ ] Multi-depth refraction-aware calibration tooling (desk-testable now)
- [x] `docs/REAL_POOL_EXPERIMENT_PROTOCOL.md` (validity criteria pre-registered)
- [x] `docs/REAL_WET_TEST_SAFETY_CHECKLIST.md`

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
