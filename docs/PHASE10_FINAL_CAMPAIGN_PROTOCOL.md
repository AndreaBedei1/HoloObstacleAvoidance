# Phase 10 — Matched sim-to-real campaign (pre-registration, DRAFT)

STATUS: DRAFT. Becomes FROZEN when the freeze record of section 9 is
filled and committed. No final-validation run happens before that, and
the simulation predictions are produced and stored BEFORE the real runs.

## 1. Question

How accurately does the simulator predict the safety and performance of
a camera-based obstacle-avoidance stack, and how does progressive
calibration of the visual observation process, of timing and of vehicle
dynamics affect sim-to-real agreement?

## 2. The shared boundary

Per `docs/EXPERIMENTAL_BOUNDARY.md`: identical source code from
`/perception/obstacles_raw` to `/planner/cmd_vel_safe` (Phase-7B
qualification, T2, Planner C / Planner D). Reality obtains raw
observations from the onboard visual detector; the simulator from
progressively calibrated observation models estimated on an INDEPENDENT
real calibration dataset. Only the layers outside that span differ.

## 3. Design — deliberately small

| Factor | Levels |
|---|---|
| Obstacle | the suspended anchor, ONE object (no torpedo) |
| Geometry | 2 repeatable configurations (section 4) |
| Planner | Planner C (committed), Planner D (DWA) |
| Repetitions | 5 per planner per configuration |

**20 real runs total.** These 20 are the FINAL VALIDATION DATASET: no
parameter is tuned from them, at any point, for any reason.

The same 2 x 2 x 5 design is instantiated in simulation under S0, S1, S2
and S3, giving 80 simulated runs, executed and stored BEFORE the real
campaign.

## 4. Scenario geometry: start REGIONS, not start points

The operator cannot place a tethered vehicle at a centimetre-accurate
pose, and requiring it would make the experiment unrepeatable in
practice. Instead each configuration defines a NOMINAL start pose and a
pre-registered TOLERANCE:

| Configuration | Nominal start (pool frame) | Tolerance |
|---|---|---|
| K-CENTRE: anchor ahead | TBD after remap | +/- 0.30 m lateral, +/- 0.40 m along, +/- 12 deg heading |
| K-OFFSET: anchor offset to one side | TBD after remap | same |

Procedure per run:
1. the operator places the vehicle roughly in the start region;
2. BEFORE releasing, the overhead ground truth measures the ACTUAL pose;
3. inside tolerance -> the run proceeds, and the measured pose is
   recorded in the manifest;
4. outside tolerance -> the operator nudges the vehicle and it is
   measured again; no run is scored from a pose outside tolerance;
5. results are NEVER corrected post hoc using the observed pose. The
   pose is a recorded covariate, not an adjustment.

This keeps the starts physically repeatable without manual metric
measurement, and it quantifies the real initial-pose variability instead
of pretending it is zero.

## 5. Order of operations (mandatory)

1. RealSense pool remap and ground-truth validation.
2. Paired observation dataset (four rough positions) -> S1.
3. Minimal adaptive actuator identification -> S3.
4. Pool-scale planner configuration derived from measured geometry and
   dynamics; identical high-level parameters in sim and real.
5. Full shadow test of the real pipeline.
6. FREEZE (section 9).
7. Simulation predictions S0/S1/S2/S3, stored.
8. The 20 real runs.
9. Analysis.

## 6. Outcome definitions (identical in sim and real)

* SUCCESS: the vehicle passes the anchor plane without collision and
  without a safety abort, having triggered from its own perception.
* COLLISION: ground-truth distance below the physical contact radius
  (vehicle half-width + anchor half-span), TBD from the remap.
* MINIMUM CLEARANCE: minimum ground-truth distance during the run.
* Secondary: maximum lateral excursion, path length, maneuver duration,
  recovery error, trajectory shape, control effort where comparable.
* TECHNICAL INVALID (excluded and re-run once): arming failure, camera
  stream loss, ground-truth loss > 5 s, start pose outside tolerance at
  release, or an abort in the first 2 s.

## 7. Sim-to-real analysis (pre-specified)

Per metric and per calibration level: absolute sim-real error and, where
the scale makes it meaningful, normalized error. Whether calibration
improves prediction monotonically (S0 -> S3). Whether the C-vs-D
ranking and the direction of the trade-offs observed in simulation are
preserved in reality.

With two planners a rank correlation is not meaningful and will not be
reported. The defensible statistics are pairwise ranking agreement,
direction of effect, and absolute prediction error.

## 8. What is deliberately NOT done

* No torpedo, no extra obstacle classes.
* No third planner invented to make a correlation computable.
* No temporal-estimator ablation inside the main comparison: both
  planners use the SAME T2 stack, so the planner difference is not
  confounded with a perception difference. A T0/T1/T2 ablation, if the
  data support one, is a separate secondary experiment.

## 9. Freeze record — TO BE COMPLETED BEFORE ANY FINAL RUN

git SHA; detector source hash and all detector parameters; Phase-7B
parameters; T2 parameters; Planner C pool configuration; Planner D pool
configuration; adapter configuration and calibration id; S0/S1/S2/S3
definitions and parameter values; RealSense calibration version and pool
frame file; pool geometry; ROV and obstacle dimensions; scenario
geometry and start regions with tolerances; target depth; repetitions;
success, collision and clearance definitions; technical-invalid rules;
the planned statistical analysis.
