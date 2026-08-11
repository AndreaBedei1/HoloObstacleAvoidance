# DWA Implementation and Protocol (Phase 8)

Classical literature baseline (Planner D). NOT a contribution — a
recognizable, honestly tuned Dynamic Window Approach adapted only where
fairness demands it.

## Literature grounding

| Source | Velocity space | Motion model | Dynamic constraints | Obstacle repr. | Admissibility | Stop criterion | Cost function | Tuning | Vehicle |
|---|---|---|---|---|---|---|---|---|---|
| Fox, Burgard, Thrun 1997 (the DWA) | (v, ω) translational+rotational | circular arcs | reachable window from accel limits over Δt | occupancy/distance | trajectory clear AND stoppable before contact | v ≤ √(2·dist·a_brake) | α·heading + β·dist + γ·velocity (normalized) | hand weights | synchro-drive indoor robot |
| Eriksen, Breivik, Pettersen 2016 (AUV DWA) | (u, r) surge+yaw-rate | AUV surge/steering dynamics | 2nd-order actuator/dynamic constraints | circular obstacles | modified admissibility for 2nd-order dynamics | braking-aware | weighted multi-objective incl. path convergence | documented weights | AUV (non-holonomic, cruise) |
| Mari et al. 2026 (BlueROV2 DWA baseline, Sensors) | (v, ω)-style | kinematic | velocity window | virtual occupancy grid from twin | clearance-based | n/a (harbor scale) | goal attraction + clearance + progress | reported baseline tuning | BlueROV2 (digital-twin sensing) |
| EROAS 2024 (IEEE JOE, DWA as baseline) | (v, ω) | kinematic | window | sonar-derived | clearance | classical | classical terms | baseline | UUV |
| **Ours (Planner D)** | **[u, v, r] body surge/sway/yaw-rate (holonomic)** | response-aware holonomic rollout (first-order τ toward candidate, Eriksen-style) | per-axis window: surge slew-limited over ONE control interval (a_lin 0.5 m/s², Δt 0.1 s); sway/yaw span their full authority (holonomic per-axis shaping) | circle: monocular estimate + class radius, inflated by footprint 0.40 m + margin | path clear > 0 AND directional stoppability: braking distance v·t_react + v²/(2·a_brake) required only of candidates still CLOSING at horizon end | fallback zero cmd on no-admissible; escape rule inside inflated zone | w_clear·clear_end_sat + w_prog·goal-progress + w_speed·u + w_route·goal-heading − w_smooth·Δcmd (5 weights) | pre-registered ranges, dev-scenario grid (below), frozen in D-013 before campaign | BlueROV2 Heavy (holonomic horizontal) |

**Why holonomic:** the BlueROV2 Heavy natively produces surge+sway+yaw; the
committed planner uses sway heavily. A (v, ω) DWA would be an unfair
handicap. This is "classical DWA principle adapted to the native horizontal
control space of the BlueROV2 Heavy" — an adaptation in the spirit of
Eriksen's, and NOT claimed as novel.

## Formulation (implemented in `rov_obstacle_avoidance/dwa_planner.py`)

- **Window (per-axis, Eriksen-style shaping):** surge is slew-limited over
  ONE control interval `u ∈ [max(0, u₀−a·Δt), min(u_max, u₀+a·Δt)]` with
  Δt = 0.1 s (the replanning period — the classical "commands reachable
  within the next control interval"); sway and yaw-rate span their FULL
  authority ±v_max/±r_max because the response-aware rollout (below)
  already enforces how fast they are physically approached. u₀,v₀,r₀ =
  TRANSFERABLE estimate from the planner's own commanded-response model
  (first-order, same τ as the no-DVL odometry). No DVL, no VelocitySensor,
  no reverse surge in the baseline.
- **Sampling:** 7×7×5 = 245 candidates/cycle at 10 Hz (vectorized;
  measured planning time ≈ 0.3 ms; recorded per run).
- **Rollout (response-aware, after Eriksen 2016):** velocities relax
  first-order (τ_u 1.2 s, τ_v 1.15 s, τ_r 0.3 s) toward the candidate
  command during integration — constant-command rollouts over-predicted
  clearance by 0.3–0.5 m. Horizon 6.0 s, step 0.2 s (30 steps), inside the
  planner (never Unreal).
- **Obstacle:** T2(+qualification) detection → bearing = (cx−0.5)·HFOV;
  monocular range = H_ref/(2·h_bbox·tan(VFOV/2)) with H_ref = 3.5 m —
  IDENTICAL assumptions to the committed planner; obstacle = circle of
  class-reference radius 1.75 m at the estimated odom-frame position.
- **Footprint:** vehicle circle 0.40 m (circumscribed BlueROV2 Heavy
  0.576×0.457 m, declared conservative) + explicit safety margin
  (tuning parameter). No GT-derived inflation anywhere.
- **Admissibility (directional stoppability):** path-min clearance > 0
  AND, for candidates still CLOSING at the horizon end (min clearance at
  the final point), end clearance ≥ v·t_react + v²/(2·a_brake) with
  t_react = 1.2 s (dominant response lag) — the classical stoppability
  criterion applied along the closing direction. A candidate whose
  clearance is receding (min occurred mid-path, or it moves away) has
  already escaped/passed and needs no braking distance: requiring stopping
  distance against the shared start proximity made every lateral escape
  inadmissible and stalled the planner at the safety boundary (iteration
  log below). ESCAPE rule: when already inside the inflated zone, the
  admissible set is the candidates that increase clearance.
- **Fallback:** no admissible candidate ⇒ publish zero Twist, log
  `no_admissible_candidate` (first-class event, counted).
- **Objective (goal-directed, after Fox 1997):** 5 normalized terms —
  clearance at the rollout END state, saturated at 2 m (path-min is
  degenerate across candidates because response-aware rollouts share a
  long common prefix); progress toward a receding route goal 15 m ahead
  (d₀ − d_end, LOS-style guidance goal as paired with marine DWA in
  Eriksen); forward speed; heading-to-goal alignment at the end state
  (replaces cross-track distance-to-line, which penalizes the avoidance
  detour itself and creates a stall equilibrium in front of the obstacle);
  command-change smoothness.

## Interface parity

Output `geometry_msgs/Twist` on `/planner/cmd_vel_safe` → identical TCP
bridge, watchdog, latency buffer, rate limiter, PI controller, thruster
allocation, HoloOcean dynamics. DWA never touches thrusters. Inputs and
staleness semantics identical to the committed planner
(`docs/PLANNER_INPUT_EQUIVALENCE.md`).

## Tuning protocol (anti-straw-man; ranges pre-declared)

Development scenarios only (P-series / dev seeds; final F-campaign
untouched). Ranges declared BEFORE tuning:

| Parameter | Range |
|---|---|
| w_clearance | {0.5, 1.0, 2.0} |
| w_progress | {0.5, 1.0} |
| w_speed | {0.1, 0.3} |
| w_route | {0.25, 0.5, 1.0} |
| w_smooth | {0.1} (fixed) |
| safety_margin_m | {0.2, 0.3, 0.5} — amended pre-freeze: {0.5, 0.8}, see below |
| horizon_s | {2.0, 3.0} — amended pre-freeze: 6.0, see below |
| clearance_saturation_m | {1.5, 2.0} (paired with margin) |

**Pre-freeze amendments (declared honestly, decided on development
scenarios only, before D-013 and before any F-run):** (a) safety_margin_m
range moved up to {0.5, 0.8} — with the monocular range model's
over-estimation bias near bbox edge-clipping, margins below 0.5 m produced
grazing passes (min GT clearance ≈ 0.3 m) on development scenarios; the
committed planner operates with a comparable effective standoff, so this
is parity, not handicap. (b) horizon_s 2–3 s was myopic for lateral
escapes at |v| ≤ 0.3 m/s (a full lateral displacement past a 1.75 m
obstacle takes ≈ 5–6 s); 6.0 s makes the escape visible to the objective.
Both amendments were required to make the baseline FUNCTION, not to make
it lose: every amendment strictly improves the DWA's own safety/success on
development scenarios.

## Formulation iteration log (development scenarios only)

Chronological record of the four formulation defects found while bringing
the classical formulation to the underwater setting, all diagnosed and
fixed BEFORE parameter freeze; kept for the paper's honesty section:

1. **Window slew vs replanning rate:** an 0.5 s window Δt replanned at
   10 Hz allowed ±0.25 m/s command jumps per 0.1 s cycle (effective
   2.5 m/s² — 5× the declared limit): thruster saturation and 10 m drift.
   Fix: Δt = one control interval (classical definition).
2. **Constant-command rollouts** over-predicted clearance 0.3–0.5 m
   (commands are not states under 1.2 s response lag): grazing passes at
   0.296–0.299 m GT clearance. Fix: response-aware rollouts (Eriksen).
3. **Cross-track route term** penalized the avoidance detour itself:
   stable stall equilibrium at the safety boundary in front of a central
   obstacle (creep-forward wins over any lateral candidate). Fix:
   goal-directed objective (receding 15 m goal: progress + heading terms,
   Fox-style) and END-state clearance scoring (path-min is degenerate
   across candidates sharing the response-limited prefix).
4. **Omnidirectional stoppability** demanded braking distance from
   candidates moving AWAY from the obstacle (min clearance dominated by
   the shared start position): every lateral escape inadmissible from
   standstill near the boundary → planner locked to slow creep even with
   the corrected objective. Fix: directional stoppability (braking
   requirement only for candidates still closing at horizon end).
   Regression-tested (standstill in front of inflated obstacle must
   produce a lateral/turn escape).
5. **Raw-speed objective raced at max_surge:** Fox's velocity term
   rewards raw forward speed, so the DWA cruised at 0.5 m/s while the
   committed planner holds the 0.3 m/s mission speed — an unfair speed
   mismatch that also tripled dead-reckoning drift (8–20 m) and drove the
   vehicle off the usable world. Fix: mission-speed tracking
   (1 − |u − u_nominal|/u_max), the marine-DWA practice of pairing the
   window search with LOS guidance at cruise speed.
6. **Infra (runner): graph-liveness retry reused the running engine**, so
   attempt 2 inherited the pose/yaw attempt 1 left behind and both the
   planner and the validator captured a poisoned (diagonal) route — one
   tuning run showed a −22.6° route from residual yaw. Fix: full
   environment restart per attempt, sim server included.
7. **Infra (validator): sleep-guard tickles masked a boundary jam.** A
   40 s jam against the world edge (commanded 0.5 m/s, world speed ~0,
   thruster force 8–12 N) never triggered the infra-freeze rule because
   each same-pose wake teleport (~every 2 s) blipped the consecutive
   150-tick counter. Fix (pre-campaign, pre-registered): the rule also
   fires on the engine's monotonic `frozen_tick_total ≥ 150`.
8. **Two candidate objective variants tried and REJECTED** while fixing
   the goal terms: (a) COURSE-to-goal alignment (end-state velocity
   direction) — any detour necessarily points its velocity away from a
   goal behind the obstacle, so it rewards parking in front of the
   obstacle and punishes every escape (the classical local minimum,
   amplified); (b) RELATIVE min-max clearance normalization over the
   candidate set — extreme flee-backward candidates stretch the range so
   far that useful escape candidates compress toward zero. Final form:
   Fox's own yaw-based heading + ABSOLUTE saturated clearance, with the
   route-convergence gradient carried by MISSION-PACED progress toward a
   receding goal at maneuver scale (lookahead 4 m): progress peaks at
   exactly the nominal pace (anti-racing) and parking scores zero
   progress (anti-parking); w_clearance default 0.5 per Fox's own weight
   ratio (safety lives in the admissibility constraint, not the
   objective).
9. **Yaw-grid quantization held a skewed equilibrium:** with n_r = 5
   (step 0.15 rad/s) the smallest nonzero turn over-rotates ~46° by the
   horizon end, so small corrective turns were inexpressible and every
   pass ended with ~30° residual yaw and lateral drift. Fix: n_r = 9
   (step 0.075). A related exploit — crab-walking (sway + counter-yaw)
   to make up mission pace when surge is window-capped — is closed by a
   LIGHT sway penalty in the cruise term (Eriksen cruise condition
   u = U_d, v ≈ 0); the penalty must stay far below the clearance gain
   of a legitimate swerve (margin analysis: crab gains < 0.01 cost
   units, a swerve gains > 0.1).
10. **Memoryless DWA re-crossed the obstacle after the pass took it out
    of the camera FOV** (pool-scale surrogate: swerve, obstacle leaves
    the ±45° FOV, blind early return THROUGH the obstacle position).
    Fix: rolling odom-frame obstacle memory in the node (TTL 30 s,
    merge 1.5 m) — the classical deployment practice (ROS move_base
    pairs DWA with a persistent rolling costmap) and information parity
    with the committed planner's commitment-state memory; only past
    perception, no ground truth. Regression-tested closed-loop (P1 and
    pool-scale K0 kinematic surrogates in the unit suite).

Objective (lexicographic): 1) zero collisions mandatory; 2) maximize
success; 3) maximize min clearance above threshold; 4) minimize
path/time/effort. Structured sequential search (safety weights first, then
efficiency), all runs archived in
`experiments/simulation/planner_dwa/tuning/`. Selected values frozen in
`docs/SCIENTIFIC_DECISIONS.md` (D-013) BEFORE the final campaign.

## Committed planner (Planner C) treatment

Parameters remain at their long-established values (unchanged since the
original baseline). They were NOT tuned on the future F-scenarios; they were
developed on the original custom-anchor scenario (documented lineage). Any
residual bias favors DWA's freshly tuned configuration, not ours.
