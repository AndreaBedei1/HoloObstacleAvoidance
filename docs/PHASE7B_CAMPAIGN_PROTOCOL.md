# Phase 7B Campaign Protocol (pre-registered BEFORE the n=5 campaign)

Campaign: `experiments/simulation/temporal_estimators_phase7b/closedloop/`
Cells: {T0, T1, T2, T3} × {E0, E1, E2, E3, E4} × 5 fresh-process reps
= 100 target valid runs. Parameters frozen per D-012; NOT changed during the
campaign. T2b is exploratory-only and NOT part of the registered benchmark.
Label: `simulation dynamics integration baseline` (oracle perception).

## TECHNICAL-INVALID criteria (rerun, not counted as experimental failures)

Written before seeing results; applied mechanically:

- graph liveness gate fails twice (validator reports no bridge traffic);
- estimator process dies (estimator_debug_msgs 0/absent);
- sim server/engine never starts or port never opens;
- validator output missing or corrupt;
- zero simulation progress (no dynamics_debug messages);
- raw perception stream absent for the entire run
  (raw_nonempty_detection_msgs = 0 in an obstacle scenario).

## ALGORITHM-FAILURE criteria (counted, never reclassified)

- collision (min clearance ≤ vehicle radius from known geometry);
- min clearance < 0.3 m without collision (near-collision);
- maneuver abort (AVOIDING → NORMAL without RECOVERING);
- re-engagement (avoidance_entries > 1);
- side switch;
- failure to return to the original line (|final lateral| > 0.5 m or
  |final yaw| > 10° at run end with final state NORMAL expected);
- planner-valid obstacle before qualification (architecturally prevented;
  if observed = failure);
- ghost-driven unnecessary avoidance (avoidance entry while no GT obstacle
  within engage range).

## Success definition (per run)

No collision AND clearance > 0.3 m AND exactly one avoidance entry AND no
side switch AND no abort AND returned to line. (Same as Phase 7 aggregate,
now pre-registered.)

## Statistics plan

Per cell: success counts reported directly (n=5, no significance testing);
continuous metrics (clearance, final lateral, maneuver time, odometry error,
confirmation delay): median, mean, sd, range; bootstrap 95% CI only where
n≥5 and the metric is defined for ≥4 runs. Objective: establish the noise
floor and whether T1/T2 differences exceed it — NOT to declare a winner at
n=5.

## Execution

Runner: `scripts/run_temporal_closedloop_campaign.py --runs 5
--out .../temporal_estimators_phase7b/closedloop`. Incremental manifest;
technical-invalid runs preserved in place and rerun into
`closedloop_patch_*` directories, merged by the analyzer with the same
pre-registered criteria.
