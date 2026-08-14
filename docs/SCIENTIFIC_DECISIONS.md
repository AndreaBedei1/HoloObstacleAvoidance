# Scientific Decisions Log

Every important methodological decision: question, alternatives, evidence, choice,
reason, implications, commit. Newest last. Referenced from
`docs/SCIENTIFIC_SIM_TO_REAL_ROADMAP.md`.

---

## D-001 — Which commit is the scientific baseline?

- **Question:** the project brief pointed at branch `feature/holoocean-anchor-primitives`;
  which state is actually the latest stable baseline?
- **Alternatives:** (a) feature branch tip `85bc041`; (b) `origin/main` `6692f07`.
- **Evidence:** `git log` shows `main` *contains* `85bc041` plus
  `1f249f9` (planner navigates on estimated odometry), `5da1a2d` (committed
  circumnavigation, no oscillation), `778a260` (README for final YOLO demo),
  `6692f07`. The estimated-odometry and committed-circumnavigation work described
  as "recent remote work" lives on `main`, not on the feature branch.
- **Chosen:** baseline = `origin/main` @ `6692f07`; scientific branch
  `feature/scientific-sim-to-real-obstacle-avoidance` created from it.
- **Implication:** the feature branch is historical; do not develop on it.
- **Commit:** branch point `6692f07`.

## D-002 — Repository was not present on this machine

- **Question:** where is the working checkout?
- **Evidence:** no local clone existed anywhere on this machine (searched Desktop,
  `source`, `IdeaProjects`, git configs). The machine's `RovTest` repo is a
  separate real-vehicle MAVLink/ROS 2 project. `HoloObstacleAvoidance` was found on
  GitHub (`AndreaBedei1/HoloObstacleAvoidance`, last push 2026-07-07).
- **Chosen:** fresh clone at `C:/Users/Andrea/Desktop/HoloObstacleAvoidance`.
- **Implication:** all "uncommitted local work" concerns are void — the remote was
  the only source of truth. Prior sim experiments ran on a different lab machine
  (`andrea.bedei3` user paths inside tracked configs), which explains missing
  externals here (see D-004).

## D-003 — Audit conclusions that gate the scientific upgrade

- **Question:** does the baseline really do what the README claims?
- **Evidence (7-subsystem parallel audit, file:line):**
  - Vehicle is `HoveringAUV` everywhere (`holoocean_sim_server.py:166,308`;
    all custom scenario YAMLs). No BlueROV2 usage.
  - Motion is kinematic teleport at 30 Hz (`holoocean_sim_server.py:1063-1079`),
    clamps surge/sway 1.5, heave 1.0, yaw_rate 0.8; roll/pitch never integrated;
    **no collision detection** (vehicle can pass through geometry).
  - Planner runtime inputs are only `/perception/obstacles`, `/cmd_vel_nominal`,
    `/rov/odom_estimated` (`local_avoidance_planner_node.py:42-44`) — ground truth
    separation holds, but only by convention/text-tests.
  - The "DVL+gyro" odometry input is synthesized: bridge finite-differences
    ground-truth pose with wall-clock dt (`holoocean_bridge_node.py:223-254`),
    then `odometry_estimator` adds deterministic bias/noise (seed 12345).
  - YOLO node stamps detections at publish time, discarding image stamps —
    perception latency is unmeasurable downstream.
  - Headline YOLO result is n=1; the committed oracle-baseline JSON is actually a
    crashed run (0xC0000005, empty state sequence).
  - Planner correctness debt: `risk_exit_threshold`, `recovery_time_s`,
    `recovery_max_time_s` declared but unused (no hysteresis, no recovery timeout);
    `_steer` clamps body sway with `max_surge` not `max_sway`.
- **Implications:** (1) HoveringAUV→BlueROV2 and teleport→dynamics are confirmed
  mandatory; (2) timestamping must move to image-stamp propagation before any
  latency-aware science; (3) the oracle-vs-YOLO A/B must be re-run to have any
  valid baseline statistics; (4) estimator redesign must not assume DVL (see D-006).

## D-004 — Baseline YOLO closed-loop is not reproducible on this machine (yet)

- **Question:** can we re-run the final YOLO demo 3× here as required?
- **Evidence:** requires (a) UE 5.3 custom engine world with cooked
  `/Game/ancora|mina|siluro` assets — only on lab machine; (b) `best.pt` — gitignored,
  absent; (c) `ocean` conda env — absent (but `holoocean_joystick` env has
  holoocean 2.3.0); (d) pixi ROS env at `C:/dev/lyrical` — absent (but a ROS 2
  "lyrical" binary install exists at `C:/dev/ros2_lyrical` with conda env
  `ros2_lyrical`, Python 3.12.3).
- **Chosen:** do NOT fake the reproduction. Recorded as blockers B1/B2. Proceed
  with what this machine can do: unit tests, stock-world BlueROV2 smoke tests, and
  the scientific migration (which must abandon HoveringAUV anyway). Ask Andrea to
  transfer the cooked custom world + weights for the paired baseline rerun.
- **Implication:** "current baseline reproduced 3×" stays open; the scientific
  claim of baseline reproduction will be made on the lab machine or after asset
  transfer.

## D-005 — Simulation vehicle: HoloOcean native BlueROV2 agent

- **Question:** how to replace HoveringAUV scientifically?
- **Alternatives:** (a) keep HoveringAUV + relabel (rejected — vehicle mismatch is
  a core sim-to-real threat); (b) custom Fossen model bolted onto HoveringAUV;
  (c) HoloOcean 2.3.0 native `BlueROV2` agent.
- **Evidence:** holoocean 2.3.0 (installed, env `holoocean_joystick`) ships
  `BlueROV2` = **BlueROV2 Heavy**: 8 thrusters, vectored-6DOF geometry
  (`thruster_d/p` match Heavy layout), control schemes 0=8×thruster forces,
  1=PID, 2=accelerations, plus documented custom/Fossen dynamics hooks.
  The real vehicle inventory (D-006) confirms FRAME_CONFIG=2 = Vectored-6DOF
  Heavy → **the sim agent's thruster topology matches the real vehicle**.
  Smoke test on this machine: BlueROV2 + RGBCamera(512²)/IMU/DVL/Depth/Pose/
  Velocity spawn and tick in the prebuilt `SimpleUnderwater` Ocean world with
  scheme-0 commands (60 ticks, all sensor outputs well-formed).
- **Chosen:** (c) native BlueROV2 agent, control scheme 0 (thruster forces) driven
  by our own body-velocity controller + allocation (Phase 5), so the same command
  interface can later map to real thruster/velocity commands.
- **Implications:** dynamics become engine-side (real hydrodynamic response);
  teleport demoted to regression-only; step-response identification becomes
  meaningful and comparable with the real vehicle.

## D-006 — Real vehicle has NO DVL: estimator must not assume one

- **Question:** does the real platform provide the velocity measurement the sim
  estimator consumes?
- **Evidence (read-only inventory, 2026-08-10):** BlueOS 1.3.1 extension list has
  no DVL driver; MAVLink stream (31 message types) contains no
  `VISION_POSITION_DELTA`/`ODOMETRY`/`LOCAL_POSITION_NED`-style aiding, no
  RANGEFINDER/DISTANCE_SENSOR; EKF runs on IMU+compass+baro only. Full details in
  `docs/REAL_BLUEROV2_HARDWARE_INVENTORY.md` and
  `docs/SIM_REAL_SENSOR_EQUIVALENCE.md`.
- **Chosen:** the runtime navigation design must be rebuilt around sensors that
  exist: ATTITUDE/yaw (10 Hz), pressure depth, IMU, commanded-motion dead
  reckoning, and possibly the position locator (pending identification of its
  runtime data path and in-water authorization). The sim `DVLSensor` may be used
  ONLY in a variant explicitly labeled non-transferable, or with noise inflated to
  emulate dead-reckoning quality; the paired sim-real experiments must use the
  matched sensor suite.
- **Implication:** Phase 7 estimator state and the planner's pass-detection logic
  must be re-derived for a velocity-sensor-less platform. This is now a tracked
  scientific risk, not a footnote.

## D-007 — RealSense D435 as external ground-truth camera

- **Evidence:** device inventory (`scripts/inspect_realsense.py`): Intel RealSense
  D435 (RGB module), 1920×1080 RGB measured 29.81 fps sustained (mean frame
  interval 33.5 ms, σ 2.8 ms, p95 36.1 ms), timestamp domain = system time
  (host-clock comparable), depth-to-color extrinsics recorded. Depth stream 848×480.
- **Chosen:** RGB-primary ground truth as planned; depth/IR recorded but untrusted
  through the air-water interface until validated (see
  `docs/REALSENSE_GROUND_TRUTH_PLAN.md`).
- **Implication:** 30 fps host-stamped RGB is sufficient for x/y/yaw ground truth;
  timestamp alignment with ROS host clock is straightforward (same machine).

## D-008 — Contribution reordering after the novelty audit

- **Question:** which planned contributions survive the literature?
- **Evidence:** 63 papers catalogued (35 deep-read), five closest identified
  (Mari 2026 Sensors; Han 2026 belief-CBF; DUViN 2025; Bergantin 2024;
  Kadian 2020). Full verdicts in `docs/LITERATURE_AND_NOVELTY.md`.
- **Chosen:** headline = S0–S3 measured-calibration predictiveness/ranking
  study (clearly novel); second = calibrated + NIS/coverage-validated adaptive
  estimator (potentially novel, requires calibrated-vs-heuristic ablations
  NSA-Kalman & UTrack-style); committed-circumnavigation uncertainty modulation
  demoted to component (cite Zhang 2022, de Groot 2025, Han 2026); pool
  protocol and integrated system dropped as claims; open code/data release
  added as differentiator. SRCC (Kadian) adopted as ranking metric; Truong
  2022 null-result framing pre-registered.
- **Implication:** `paper/contributions.md` restructured; experiment design
  must include the estimator ablation baselines from day one.
- **Commit:** (this commit).

## D-009 — Classical baseline = underwater-adapted DWA

- **Evidence:** Mari 2026 uses DWA as THE BlueROV2 avoidance baseline; EROAS
  2024, VADWA 2024, 2025 marine reviews confirm DWA mainstream; Eriksen 2016
  provides the canonical AUV adaptation. Alternatives assessed in
  `docs/CLASSICAL_PLANNER_BASELINE.md` (APF optional secondary; VFH/VO/MPC
  rejected with reasons).
- **Chosen:** DWA per Eriksen 2016, fed the same estimator mean (no oracle),
  honestly tuned with a documented procedure.
- **Commit:** (this commit).

## D-010 — Sim must run at 30 ticks/s AND in real time (physics-time consistency)

- **Question:** why did the transferable dead-reckoning odometry show ~4 m
  drift on a perfectly tracked straight run?
- **Evidence (diagnostic series, `experiments/simulation/baseline0_diag*`):**
  (a) With the serve loop paced by plain `time.sleep`, Windows' ~15.6 ms sleep
  quantization capped the loop at ~16-21 Hz → sim time ran slower than wall
  time while ROS-side wall-clock estimators kept integrating — apparent
  "drift" was a clock mismatch. (b) At `ticks_per_sec: 20` the vehicle moved
  at 0.646× its sensor-reported velocity: **Unreal physics advances at most
  1/30 s per frame** (Max Physics Delta Time), so dt=50 ms frames silently
  advance physics only 33 ms — declared sim time and physics time diverge.
- **Chosen:** `ticks_per_sec: 30` is MANDATORY for scientific scenarios
  (dt = the UE physics cap ⇒ consistent), and the server paces the loop in
  real time with a high-resolution timer (`timeBeginPeriod(1)` + sleep/spin
  hybrid). Render load reduced (no viewport, 256² camera) so 30 Hz holds
  (tick cost ≈ 22 ms < 33.3 ms).
- **Result:** straight-run transferable-odometry error dropped from 4.08 m to
  **0.117 m max (2.2 cm mean)** over 12.2 m — the estimator was correct all
  along; the sim clock was lying.
- **Implications:** any future scenario must keep tps=30 and verify
  real-time pacing (dynamics_debug rate == 30 Hz) before trusting wall-clock
  estimators; this is also a warning for the S0–S3 latency calibration.
- **Commit:** (baseline0 commit).

## D-012 — Phase-7B parameter freeze (perception qualification)

- **Question:** which qualification parameters run the n=5 campaign?
- **Development evidence (never the final campaign):** unit tests (70),
  Y0–Y9/S0 replay cases, D-series, and two closed-loop shakeouts
  (`experiments/simulation/temporal_estimators_phase7b/shakeout*`).
- **Frozen values:** warm-up = 20 stream-healthy updates AND ≥1.0 s span
  (empty messages count as stream health; rejected garbage resets the
  streak); coherence bounds physics-derived: center rate ≤1.2 units/s
  (abs floor 0.25/msg), |Δlog size| ≤0.5/msg; reference lifecycle: max age
  1.0 s, reset after 5 consecutive rejections (majority evidence);
  confirmation M=3 accepted coherent updates, window 1.5 s, empty-streak
  3 clears; source-restart silence 4.0 s (> the 2.5 s estimator horizon so a
  bridged dropout can NEVER trigger a mid-maneuver re-warm-up); estimator
  horizons unchanged from Phase 7 (hold/prediction 2.5 s).
- **Shakeout acceptance measured:** E0×t0 clean (commit only after
  qualification; distance at first planner-valid 11.97 m vs ~12.0 m visible
  — negligible engagement-distance loss; confirmation delay 0.63 s in
  validator clock, warm-up 1.02 s in estimator clock); E4×t2 clean (young-
  track outlier no longer reaches the planner; no collision).
- **Rule:** these parameters are FROZEN for the entire n=5 campaign; any
  change would require a full rerun and a new decision entry.
- **Commit:** (Phase-7B commit).

## D-013 — Phase 8 DWA baseline parameter freeze

- **Question:** which holonomic-DWA configuration runs the final C-vs-D
  comparison campaign?
- **Development evidence (P-series dev scenarios ONLY, archived under
  `experiments/simulation/planner_dwa/tuning/`):** stage-1 grid r6
  (w_clearance {0.5, 1.0, 2.0} x margin {0.5, 0.8} on P1/P2) and stage-2
  single-axis probes (w_route {0.25, 1.0}, w_speed 0.1, w_progress 0.5),
  plus the kinematic closed-loop regression battery in the unit suite.
  Lexicographic objective, measured results: zero collisions in all 20
  tuning runs; w_clearance 2.0 REJECTED (parks 5 m before a central
  obstacle: path 5.5-5.9 m, the clearance-dominant local minimum);
  w_progress 0.5 REJECTED (parks at 7.45 m, anti-parking pressure too
  weak); w_route 0.25 looser returns (|final lat| 0.60-0.72 vs
  0.26-0.48); w_route 1.0 residual yaw -10.7 deg; w_speed 0.1 residual
  yaw -7.8 deg with no efficiency gain. Margin 0.8 preferred over 0.5 at
  criterion 3 (P1 GT clearance 1.65 vs 1.06 m against the close-range
  monocular over-estimation stack).
- **Frozen values:** w_clearance 0.5, w_progress 1.0, w_speed 0.3,
  w_route 0.5, w_smooth 0.1; safety_margin_m 0.80 (+ vehicle radius
  0.40); horizon 6.0 s, rollout step 0.2 s, window = one control
  interval (0.1 s, surge slew-limited; sway/yaw full-authority per-axis
  window); sampling 7 x 7 x 9; limits u<=0.5, |v|<=0.3, |r|<=0.3;
  goal_lookahead 4.0 m; clearance saturation 2.0 m; stoppability
  v*1.2 s + v^2/(2*0.5) applied to closing candidates only + escape
  rule; response taus 1.2/1.15/0.3 s; mission-paced progress + cruise
  tracking with light sway penalty (0.15/max_sway); obstacle memory TTL
  30 s, merge 1.5 m; obstacle class constants per scenario set (3.5 m /
  1.75 m long-range; 0.5 m / 0.25 m pool) applied to BOTH planners.
- **Formulation lineage:** 10 documented iterations
  (`docs/DWA_IMPLEMENTATION_AND_PROTOCOL.md`), each fixed BEFORE this
  freeze; two candidate variants rejected with recorded numeric evidence
  (course-alignment heading; relative clearance normalization).
- **Rule:** FROZEN for the entire Phase 8 campaign. Any change
  invalidates the campaign and requires a new decision entry.
- **Commit:** (Phase 8 freeze commit).

## D-015 — Real monocular range model (one-point calibration)

- **Question:** how is the distance to the real anchor obtained from the
  onboard camera, with no trained detector and no calibration rig?
- **Method:** the anchor arm span (0.80 m, operator measurement) spanned
  399 px at 2.56 m of overhead ground-truth distance -> f_px = 1277,
  i.e. HFOV ~74 deg underwater, consistent with the BlueROV2 low-light
  camera behind a dome in water.
- **Measured accuracy (47 paired samples, 1.29-2.64 m):** bias -0.26 m,
  MAE 0.44 m, RMS 0.58 m. Roughly 25% of range: adequate to TRIGGER a
  committed maneuver, NOT adequate for continuous servoing.
- **Rule:** the model is frozen for the Phase-9 campaign; refining it
  requires a new decision entry and a re-run.

## D-016 — Phase 9 real-campaign freeze

- **Question:** which configuration runs the 10-repetition real
  avoidance campaign?
- **Development evidence (2026-08-14 afternoon, declared as development,
  never as results):** 8 missions; the behaviour was reshaped three
  times on operator observation — active heading hold instead of
  disarming (the vehicle coasts and keeps rotating), lateral clearing
  ONLY while the anchor is visible followed by a straight leg (pure
  sideways travel walks into the pool walls), and a wall guard relative
  to the release point (the operator releases from the pool edge, so an
  absolute margin aborted every run). Authority had to be raised to 35%
  surge: below ~30% the thrusters spin without moving the vehicle
  against tether drag.
- **Frozen values:** as tabulated in
  `docs/PHASE9_REAL_CAMPAIGN_PROTOCOL.md` section 4.
- **Rule:** FROZEN for the entire campaign. Pilot runs are declared as
  pilots and excluded from the results.
- **Commit:** (Phase 9 freeze commit).

---

*(Add new decisions below with incrementing IDs.)*
