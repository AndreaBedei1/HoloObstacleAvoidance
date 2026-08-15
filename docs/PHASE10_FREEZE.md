# Phase 10 — freeze record

STATUS: filled as the simulated side closes. No real validation run
happens until every line below is complete and this file is committed.

The freeze exists so the 20 real runs test a prediction rather than
illustrate one. Everything a real run could otherwise be tuned against
is fixed here, before the real runs exist, with hashes where a hash is
possible.

## 1. Code

| Item | Value |
|---|---|
| repository | `HoloObstacleAvoidance` |
| branch | `feature/scientific-sim-to-real-obstacle-avoidance` |
| freeze commit | TO BE FILLED at freeze |
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
| sha256 | TO BE FILLED at freeze |
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

## 7. Real campaign

| Item | Value |
|---|---|
| runs | 2 geometries x 2 planners x 5 repetitions = 20 |
| start regions | nominal pose with pre-registered tolerance: +/- 0.30 m lateral, +/- 0.40 m along, +/- 12 deg heading |
| start pose | MEASURED before release and recorded; never corrected post hoc |
| recording | both raw streams continuously, timestamps in sidecar indices |
| technical invalid | arming failure, camera loss, ground-truth loss > 5 s, start pose outside tolerance, abort in the first 2 s; re-run once |
| rehearsal | one shadow end-to-end pass on the day, NOT counted, to check camera, ground truth, synchronisation, interlock and logging. If it passes, no tuning: the 20 runs start |

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
