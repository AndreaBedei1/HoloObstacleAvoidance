# Phase 10 — freeze record

STATUS: **FROZEN 2026-08-15.** The 80 simulated predictions exist, are
verified and are hashed. The real campaign may begin. Nothing below may
change without re-running all 80 predictions first.

The freeze exists so the 20 real runs test a prediction rather than
illustrate one. Everything a real run could otherwise be tuned against
is fixed here, before the real runs exist, with hashes where a hash is
possible.

## 1. Code

| Item | Value |
|---|---|
| repository | `HoloObstacleAvoidance` |
| branch | `feature/scientific-sim-to-real-obstacle-avoidance` |
| code state that produced the predictions | `338ac09` |
| ROS 2 | lyrical, `C:/dev/ros2_lyrical`, env `ros2_lyrical` |
| simulator | HoloOcean 2.3.0, prebuilt Ocean worlds |
| tick rate | 30 ticks/s (D-010: below this the vehicle moves slower than the reported velocity) |

## 2. The frozen boundary

Identical source code, both domains:

```
/perception/obstacles_raw -> Phase-7B qualification -> T2 estimator
                          -> Planner C / Planner D -> /planner/cmd_vel_safe
```

What differs by construction: how raw observations are produced (real
detector versus calibrated observation model) and what consumes the
command (real vehicle versus plant model). Nothing between.

## 3. The pool benchmark — identical at every level and in reality

Single source: `config/pool_benchmark_FROZEN.yaml`, read by BOTH the
simulated campaign and the real pipeline.

| Parameter | Value |
|---|---|
| nominal surge | 0.15 m/s |
| engagement distance | 1.5 m |
| target obstacle height (shared monocular constant) | 0.5 m |
| DWA obstacle radius | 0.25 m |
| DWA goal lookahead | 4.0 m |
| camera horizontal FOV | 74 deg |
| K0 geometry | obstacle at [3.5, 0.0, 0.0] m |
| K1 geometry | obstacle at [3.5, 0.35, 0.0] m |

This is NOT a calibration rung. It defines the matched experiment and
must never differ between S0 and S3;
`scripts/analysis/audit_rung_ownership.py` verifies that from the stored
launch invocations of every run.

## 4. Calibration levels — what each one owns

| Level | Adds | Source |
|---|---|---|
| S0 | nothing; the historical simulator | — |
| S1 | static observation model: detection probability vs true range, monocular range bias, bearing scatter | `config/calibration/s1_observation_fit.json` |
| S2 | timing: rate, jitter, BURST structure of misses | `config/calibration/s2_timing_fit.json` |
| S3 | plant downstream of `/planner/cmd_vel_safe`: saturation, deadband, per-sign asymmetry, first-order lag | `config/calibration/s3_vehicle.json` |

S3 does NOT change the planners. Measured values:

| Quantity | Value |
|---|---|
| detection probability | 0.67 below 0.8 m, 0.94 at 1.6-2.0 m, 0.15 beyond 2.0 m |
| shared estimator vs truth | 0.469 x truth, median absolute error 1.01 m, n=24 |
| bearing scatter (offset removed) | 0.32 deg |
| observation acceptance | 0.259 of cycles at 3.5 Hz |
| mean blind run | 9.2 cycles = 2.6 s (3.5 cycles if independent) |
| longest blind run | 57 cycles = 16 s |
| surge / sway at full command | 0.145 / 0.123 m/s |
| yaw | 0.0654 deg/s per count with 383-count deadband (CW), 0.0537 with 220 (CCW) |
| rise / decay | surge 2.0 s / 1.0 s, sway 0.30 s / 0.97 s, both weakly identified |
| command authority when measured | 0.20 |

**Command authority must be re-measured before every real session.** A
reboot resets ArduSub's joystick gain to `JS_GAIN_DEFAULT` (0.5), which
would invalidate the whole S3 profile with no outward sign.

## 5. Predictions

| Item | Value |
|---|---|
| design | 4 levels x 2 planners x 2 geometries x 5 repetitions = 80 |
| location | `experiments/simulation/phase10_<level>/` |
| aggregate | `experiments/simulation/phase10_predictions/predictions.json` |
| sha256 | `2f51abbdc57b04f250cac7246479525b35427e91b13c46a708963d09780d9b9e` |
| verification | every run's relay and plant sentinel matches its level; no technical invalid |

## 6. Analysis, fixed before the real runs

| Choice | Value |
|---|---|
| statistic | median over the 5 repetitions |
| commitment | lateral command > 0.05 m/s SUSTAINED 1.0 s, applied offline to the recorded trace, identically in both domains |
| metrics | minimum clearance, commitment distance, maximum lateral excursion, path length, manoeuvre duration, lateral peak |
| primary question | absolute error \|sim - real\| per level, and whether it shrinks S0 -> S3 |
| planner comparison | ranking agreement and direction of effect; NO rank correlation, which is not meaningful with two planners |
| never manoeuvred | recorded as None, never as zero |
| manoeuvre duration | SPAN from first commitment to last lateral command, plus the active time separately. Ending the manoeuvre at the first gap longer than the hold time chopped every S2 manoeuvre systematically, because at S2 the vehicle is blind for 2.6 s at a stretch: the gap is a property of the perception being modelled, not the end of the manoeuvre |
| commanded lateral peak | CONTROL variable, not an outcome. Measured on the frozen boundary, so it is the command and not what the vehicle achieves; its constancy at 0.300 m/s across all four levels is evidence the planner configuration really was identical |

### What the predictions say, before any real run

| Prediction | Value |
|---|---|
| minimum clearance, committed, K0 | 1.48 m at S0 falling to 0.58 m at S3 |
| minimum clearance, DWA, K0 | 2.14 m at S0 falling to 0.86 m at S3 |
| commitment distance, committed | 1.87 m at S0, 3.01 m at S1, 1.16 m at S3 |
| commitment distance, DWA | 3.50 m at every level: it strafes from the first cycle regardless of calibration |
| manoeuvre span, committed | shortens, 64 s to 50 s |
| manoeuvre span, DWA | lengthens, 60 s to 92 s |

The two planners are predicted to respond to calibration in OPPOSITE
directions on manoeuvre duration. That is the sharpest testable claim in
the set and it does not depend on absolute agreement.

## 7. Real campaign

| Item | Value |
|---|---|
| runs | 2 geometries x 2 planners x 5 repetitions = 20 |
| start regions | nominal pose with pre-registered tolerance: +/- 0.30 m lateral, +/- 0.40 m along, +/- 12 deg heading |
| start pose | MEASURED before release and recorded; never corrected post hoc |
| recording | both raw streams continuously, timestamps in sidecar indices |
| technical invalid | arming failure, camera loss, ground-truth loss > 5 s, start pose outside tolerance, abort in the first 2 s; re-run once |
| rehearsal | one pass on the day, NOT counted, checking frozen predictions and calibration hashes, pool benchmark, adapter interlock, dual recording, timestamp indices and their shared clock, the next run the frozen order selects, overhead ground truth, onboard camera, the start pose, and the command authority. If it passes, no tuning: the 20 runs start |
| execution order | FROZEN in `config/run_order_FROZEN.yaml` and selected automatically by the runner |

### Execution order — frozen

Five blocks of four. Every block contains all four conditions exactly
once, so any drift slower than one block is shared equally; the order
within blocks rotates, so each condition takes each of the four
positions at least once; and planners alternate inside every block,
which is the comparison the paper rests on.

Running planner by planner instead would confound the planner with
battery charge, water temperature and the pool recirculation that was
measured GROWING across the actuator session.

| # | geometry | planner | rep | | # | geometry | planner | rep |
|---|---|---|---|---|---|---|---|---|
| 1 | K0 | committed | 1 | | 11 | K0 | committed | 3 |
| 2 | K0 | dwa | 1 | | 12 | K0 | dwa | 3 |
| 3 | K1 | committed | 1 | | 13 | K1 | dwa | 4 |
| 4 | K1 | dwa | 1 | | 14 | K0 | committed | 4 |
| 5 | K0 | dwa | 2 | | 15 | K0 | dwa | 4 |
| 6 | K1 | committed | 2 | | 16 | K1 | committed | 4 |
| 7 | K1 | dwa | 2 | | 17 | K1 | dwa | 5 |
| 8 | K0 | committed | 2 | | 18 | K1 | committed | 5 |
| 9 | K1 | committed | 3 | | 19 | K0 | dwa | 5 |
| 10 | K1 | dwa | 3 | | 20 | K0 | committed | 5 |

`final_campaign.py` with no arguments runs the next pending entry and
refuses one out of order unless `--force-out-of-order` is given, which
exists only for repeating a technically invalid run.

### Approach distance: shorter in reality than in the predictions

The simulated geometries start the vehicle 3.5 m from the anchor. The
overhead camera covers 4.24 x 2.38 m with the anchor at x = 1044 px, so
the furthest the vehicle can start and still be WHOLLY visible is about
1.86 m, and a start outside the frame cannot have its pose measured
before release — which is the requirement that makes the pre-registered
start regions mean anything.

The nominal real start is therefore 1.86 m, not 3.5 m. The engagement
distance is 1.5 m, so the vehicle still has run-up before it engages,
but the approach is shorter in reality than in the predictions. This is
a property of the comparison, not a free parameter, and it must be
stated wherever the sim-real numbers are reported.

Two ANALYSIS-ONLY consequences, handled offline from the recorded
traces. Neither changes the protocol, the planners, the calibrations or
the frozen predictions, whose hash is unchanged.

**Left censoring.** If the vehicle is already commanding laterally at
the first recorded sample, the manoeuvre began at or before the window
opened: its commitment distance is a LOWER BOUND, reported as `>=`, and
the manoeuvre duration inherits the same bound. Without this, DWA's
simulated 3.50 m and a real 1.86 m would both be "the start" while
looking like a 1.6 m sim-to-real error. The censoring threshold is
0.02 m/s, deliberately lower than the 0.05 m/s commitment threshold: a
lower bar for "already moving" flags more runs as censored, which is the
conservative direction. Cells report the count when only some
repetitions are censored, because averaging bounds together with
measurements is how an artefact becomes a finding.

**Common observable window.** The same simulated traces are also
reported with everything beyond 1.86 m discarded, so both domains cover
the same stretch. The frozen predictions are unchanged and reported
alongside.

That view produces a result of its own, visible before any real run: at
S1 no condition enters the window at all. The simulated vehicle keeps
2.2-2.5 m of clearance and never comes as close as the real run STARTS,
and the same is true of DWA at S0. Those cells are reported as "never in
window" rather than as missing data — the two domains cannot be compared
there, and saying so is the result.

## 8. Excluded, explicitly

The imaging/side-scan sonar is physically connected to the vehicle and
was never activated, initialised, configured, queried in any way that
could cause transmission, or recorded, at any point in this work.

## 9. Known limitations, recorded before the results

* The pool disturbance is an uncontrolled input present in the real runs
  and absent from the simulation: 0.037-0.053 m/s of thruster-induced
  recirculation, non-stationary enough that successive measurements
  pointed in opposite directions. It is not modelled — fitting a
  disturbance model to four inconsistent measurements would be
  invention.
* The paired observation dataset has no coverage between 0.8 and 1.6 m.
* Command latency was never measured; the plant exposes it and leaves it
  at zero rather than injecting a plausible value.
* Rise time constants have spreads as large as their values.
* `sway+` rests on a single valid trial; the translation axes are
  reported as symmetric because their directions are indistinguishable
  at this sample size, not because symmetry was verified.
* Two actuator trials ended in contact (pool wall, anchor) and are
  excluded from the fit and kept in the record with their reason.
