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
| **Ours (Planner D)** | **[u, v, r] body surge/sway/yaw-rate (holonomic)** | constant-command holonomic rollout | window = current ESTIMATED vel ± accel·Δt, clipped to vehicle limits (a_lin 0.5 m/s², a_yaw 0.8 rad/s², Δt 0.5 s) | circle: monocular estimate + class radius, inflated by footprint 0.40 m + margin | predicted clearance > 0 AND clearance ≥ v²/(2·a_brake), a_brake 0.5 | fallback zero cmd on no-admissible | w_clear·clear_sat + w_prog·route-progress + w_speed·u − w_route·cross-track − w_smooth·Δcmd (5 weights) | pre-registered ranges, dev-scenario grid (below), frozen in D-013 before campaign | BlueROV2 Heavy (holonomic horizontal) |

**Why holonomic:** the BlueROV2 Heavy natively produces surge+sway+yaw; the
committed planner uses sway heavily. A (v, ω) DWA would be an unfair
handicap. This is "classical DWA principle adapted to the native horizontal
control space of the BlueROV2 Heavy" — an adaptation in the spirit of
Eriksen's, and NOT claimed as novel.

## Formulation (implemented in `rov_obstacle_avoidance/dwa_planner.py`)

- **Window:** `u ∈ [max(0, u₀−a·Δt), min(u_max, u₀+a·Δt)]`, same for v
  (±v_max) and r (±r_max); u₀,v₀,r₀ = TRANSFERABLE estimate from the
  planner's own commanded-response model (first-order, same τ as the no-DVL
  odometry). No DVL, no VelocitySensor, no reverse surge in the baseline.
- **Sampling:** 7×7×5 = 245 candidates/cycle at 10 Hz (measured planning
  time < 100 ms; recorded per run).
- **Rollout:** constant-command holonomic integration, horizon 3.0 s,
  step 0.2 s (15 steps), inside the planner (never Unreal).
- **Obstacle:** T2(+qualification) detection → bearing = (cx−0.5)·HFOV;
  monocular range = H_ref/(2·h_bbox·tan(VFOV/2)) with H_ref = 3.5 m —
  IDENTICAL assumptions to the committed planner; obstacle = circle of
  class-reference radius 1.75 m at the estimated odom-frame position.
- **Footprint:** vehicle circle 0.40 m (circumscribed BlueROV2 Heavy
  0.576×0.457 m, declared conservative) + explicit safety margin
  (tuning parameter). No GT-derived inflation anywhere.
- **Admissibility:** every trajectory step outside the inflated circle AND
  min-clearance ≥ v²/(2·a_brake) (classical stoppability).
- **Fallback:** no admissible candidate ⇒ publish zero Twist, log
  `no_admissible_candidate` (first-class event, counted).
- **Objective:** 5 normalized terms (clearance saturated at 2 m; route
  progress over horizon; forward speed; cross-track+heading alignment
  penalty — needed because a holonomic vehicle can wander sideways;
  command-change smoothness).

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
| safety_margin_m | {0.2, 0.3, 0.5} |
| horizon_s | {2.0, 3.0} |
| clearance_saturation_m | {1.5, 2.0} (paired with margin) |

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
