# Phase 8 — Pre-registered Planner Comparison Protocol (C vs D)

STATUS: FROZEN (D-013 recorded in `docs/SCIENTIFIC_DECISIONS.md`,
2026-08-11). No F-scenario run happened before this freeze. Runs
executed before the freeze exist only under
`experiments/simulation/planner_dwa/{smoke*,tuning}/` and are development
artifacts, never comparison evidence.

## 1. Question

On identical transferable inputs (T2 temporal estimation + Phase-7B
qualification, monocular range model, no-DVL commanded odometry), how does
the committed-circumnavigation planner (C) compare against an honestly
tuned classical holonomic DWA baseline (D) on safety, success, efficiency,
and robustness — at long range and at pool scale?

## 2. Planner versions

| | Planner C (committed) | Planner D (DWA baseline) |
|---|---|---|
| Node | `local_avoidance_planner_node` | `dwa_planner_node` |
| Config | `config/local_avoidance_planner.yaml` (unchanged since Baseline 0; lineage documented) | frozen in D-013 (TBD) |
| Inputs | `/perception/obstacles`, `/cmd_vel_nominal`, `/rov/odom_estimated` | identical (`docs/PLANNER_INPUT_EQUIVALENCE.md`) |
| Output | `/planner/cmd_vel_safe` (Twist) | identical |
| Formulation | committed circumnavigation state machine | `docs/DWA_IMPLEMENTATION_AND_PROTOCOL.md` |

Common stack (fixed for every run): T2 fixed Kalman + Phase-7B
qualification, oracle perception relay, sensor-clocked commanded odometry,
dynamics motion model, tps=30, watchdog, zenoh isolation, graph-liveness
gate with FULL-environment retry (sim server included).

## 3. Scenarios

Long-range set (route ~35 m, nominal 0.3 m/s unless stated): F0 central,
F1 +1.5 m left, F2 −1.5 m right, F5 smaller extent, F6 reduced left
clearance (second body), F7 mid-maneuver 2 s dropout, F8 young-track
outlier, F9 +15° initial heading error, F10 approach 0.4 m/s.
Pool-scale set (route ~6 m, 0.15 m/s, 0.5 m obstacle class, shared class
constants scaled for BOTH planners): K0 central, K1 0.75 m left.
Geometry YAMLs are committed under `config/holoocean_scenarios/` before
the freeze. Development scenarios (P-series) never enter the campaign.

## 4. Design

Primary comparison: n = 10 repetitions × 2 planners × 9 long-range
scenarios = 180 runs. Pool-scale feasibility profile: n = 5 × 2 planners
× 2 K-scenarios = 20 runs (feasibility characterization, not the primary
hypothesis test — reported descriptively). Total 200 runs, paired by
scenario, fixed run duration 120 s (long-range) / 90 s (pool),
alternating planner order inside each scenario block. Technical-invalid
runs are excluded and re-run; algorithm failures are never re-run.

## 5. Outcome metrics (validator, GT-based, control path never sees GT)

Primary: collision (GT min center distance < obstacle radius + vehicle
radius 0.40 m); success = no collision AND forward progress past the
obstacle plane AND GT return to the nominal line (|lateral| < 1.0 m)
before run end. Secondary: min clearance, max lateral deviation, path
length, maneuver time, side switches, avoidance entries, thruster
peak/mean force, saturation fraction, command smoothness, odometry error
(max / max-during-maneuver), no-admissible events (D), commit distance
(C), qualification delays. Pool-scale addendum: lateral envelope and
return distance within the 6 m route window (from `odo_series` GT trace).

## 6. Technical-invalid rules (objective, machine-detectable, frozen here)

A run is technical-invalid iff any of:
1. `infra_freeze_detected` — consecutive rule (150 ticks cmd>0.1,
   world speed <0.01, max thruster force >2 N) OR cumulative engine
   `frozen_tick_total ≥ 150` (sleep-guard tickle interaction, defect #7
   in the implementation doc);
2. graph liveness failed twice (launch never became live);
3. validator output missing or corrupt (crash before write);
4. sim-server port never opened;
5. `cmd_path_dead_detected` — the ROS side publishes a nonzero safe
   command but the sim-side controller setpoint stays ~0 with world speed
   ~0 for 150 consecutive dynamics ticks (bridge->server TCP command-path
   outage; observed as a 60 s dead start with bridge reconnect flaps that
   the graph-liveness gate cannot see).
No other exclusion is permitted; everything else is an algorithm result.
The campaign runner re-runs a technical-invalid run exactly once and
records the replacement (`replaced_technical_invalid`).

## 7. Analysis plan

Per scenario: collision counts (exact), success proportions with 95%
Wilson CIs, paired comparison of continuous metrics (median + IQR;
Mann-Whitney U at α = 0.05 with Holm correction across scenarios,
reported as descriptive given n = 10). No metric selection after
unblinding: every metric in §5 is reported for every scenario.
Aggregation script: `scripts/aggregate_planner_campaign.py` (committed
before the campaign).

## 8. Frozen DWA parameters (D-013, frozen 2026-08-11)

| Parameter | Value |
|---|---|
| w_clearance / w_progress / w_speed / w_route / w_smooth | 0.5 / 1.0 / 0.3 / 0.5 / 0.1 |
| safety_margin_m | 0.80 (+ vehicle radius 0.40) |
| horizon_s / sim_dt / window_dt | 6.0 / 0.2 / 0.1 (surge slew-limited; sway/yaw full-authority) |
| n_u × n_v × n_r | 7 × 7 × 9 |
| max_surge / max_sway / max_yaw_rate | 0.5 / 0.3 / 0.3 |
| goal_lookahead_m / clearance_saturation_m | 4.0 / 2.0 |
| reaction_time_s / stop_decel | 1.2 / 0.5 (closing candidates only + escape rule) |
| taus (u, v, r) | 1.2 / 1.15 / 0.3 |
| obstacle memory | TTL 30 s, merge 1.5 m |
| sway cruise penalty | 0.15 (normalized on max_sway) |

STATUS: FROZEN. Tuning evidence and rejections in D-013
(`docs/SCIENTIFIC_DECISIONS.md`).

## 9. Honesty commitments

The DWA is a baseline, not a contribution; it is not modified after the
freeze. Tuning used only P-series development scenarios; every tuning run
is archived. Pre-declared range amendments are documented in the
implementation doc with reasons. Neither planner is tuned on F/K
scenarios. Scientific honesty outranks making either planner win.
