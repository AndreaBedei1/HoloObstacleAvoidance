# Calibration Profiles S0-S3

The headline claim (C1, `paper/contributions.md`) is that *progressively
calibrating the simulator with measured models changes how well it predicts
real closed-loop avoidance outcomes and how well it preserves the ranking of
estimator/planner stacks*. That claim is only readable if each rung of the
ladder differs from the previous one by exactly **one class of effect**. If
two rungs both touch the frame rate, or if a plausible-but-unmeasured number
fills a slot, a difference in predictivity cannot be attributed to anything.

These files are the machine-readable form of that discipline:

| File | Level | Adds |
|---|---|---|
| `config/calibration/s0_historical.yaml` | S0 | nothing — the current simulation, captured as-is (IMMUTABLE) |
| `config/calibration/s1_observation.yaml` | S1 | measured **static observation model** |
| `config/calibration/s2_timing.yaml` | S2 | measured **timing / transport** of observations |
| `config/calibration/s3_vehicle.yaml` | S3 | measured **vehicle / actuation** response |

Loader and validator: `scripts/calibration/load_profile.py`
(tests: `scripts/calibration/test_load_profile.py`, 20 cases, passing).

The scientific boundary is unchanged: everything a level calibrates is
upstream of `/perception/obstacles_raw` (S1, S2) or downstream of
`/planner/cmd_vel_safe` (S3). No level touches the planner or the estimator —
those are the systems under test, frozen separately by D-012 and D-013.

## 1. The nesting rule

    S0 = the historical configuration, exactly as it runs today
    Sn = S(n-1) + one new class of MEASURED effect,   n = 1, 2, 3

Three consequences, all enforced by the loader:

1. **Inheritance is real.** A level lists only what it overrides; everything
   else is inherited verbatim from its parent. `load_profile("S2")` returns
   S0 merged with S1 merged with S2.
2. **One knob, one level.** A key overridden at S1 may not be overridden at
   S2 or S3. Sections are owned exclusively (table below) and the loader
   rejects a chain that violates it.
3. **Measured, or null.** A value is written down only if a measurement or a
   documented derivation supports it. Everything else is `null` plus a TODO
   naming the measurement that would fill it. `--require-measured` refuses to
   emit a profile that still contains a null, so an unmeasured value cannot
   become an experimental condition by accident.

S0 additionally carries **provenance for every leaf** — the file each value
came from — because the control condition of the whole study must be
auditable, not remembered.

## 2. Ownership table (effect -> owning level)

| Effect | Owner | Key(s) |
|---|---|---|
| Engine, world, agent, control scheme | S0 | `simulator.*` |
| Tick rate / real-time pacing (D-010) | S0, immutable | `simulator.ticks_per_sec`, `simulator.real_time_pacing` |
| Command timeout, start pose, no-collision, no-current | S0 | `simulator.*` |
| Camera FOV, intrinsics, distortion, refraction | S1 | `camera.*` |
| Render size / bbox quantization | S1 | `camera.width`, `camera.height` |
| Detection probability (marginal, per frame) | S1 | `observation.detection_probability` |
| Bearing and size bias/scatter | S1 | `observation.center_*`, `observation.log_size_*` |
| False positives, class confusion, occlusion, edge clip | S1 | `observation.*` |
| Detector acceptance gate | S1 | `observation.detector_gate.*` |
| Range gates of the observation | S1 | `observation.min/max_detection_range_m` |
| Observation rate and its source | S2 | `timing.observation_rate_hz`, `timing.observation_rate_source` |
| Inter-arrival jitter, arrival process | S2 | `timing.observation_interval_jitter_std_s`, `timing.arrival_process` |
| Observation (perception) latency and its jitter | S2 | `timing.observation_latency_s`, `timing.observation_latency_jitter_std_s` |
| Burst dropout / availability process | S2 | `timing.burst_dropout.*` |
| Timestamp source | S2 | `timing.stamp_source` |
| Signed axis mapping sim<->real | S3 | `vehicle.axis_sign_convention.*` |
| Per-axis authority limits | S3 | `vehicle.limits.*` |
| Deadband | S3 | `vehicle.deadband.*` |
| Directional gain asymmetry | S3 | `vehicle.gain_asymmetry.*` |
| Rise / decay time constants, coasting | S3 | `vehicle.response.*`, `vehicle.coasting_model` |
| Command (actuation) latency | S3 | `vehicle.command_latency_s` |
| Setpoint rate limits, controller gains | S3 | `vehicle.setpoint_rate_limits.*`, `vehicle.controller.*` |

### 2.1 The three splits that would otherwise be double counted

**Detection probability vs burst structure.** S1 owns the *marginal*
probability `p` that a frame yields an accepted detection. S2 owns the fact
that the misses arrive in bursts. A two-state visible/blind process has a
marginal of its own, so the S2 fit is **constrained** by

    mean_visible_run_s / (mean_visible_run_s + mean_blind_run_s) == p

Fitting both freely would thin the observation stream twice and make S2
harsher than the real vehicle while looking like a fidelity improvement.

**Observation latency vs command latency.** `timing.observation_latency_s`
(S2) is measured on the camera path; `vehicle.command_latency_s` (S3) is
measured from MANUAL_CONTROL to thruster PWM. They are separate measurements
on separate signals and must never be merged into one "loop delay".

**Render size vs the tick contract.** Raising `camera.width/height` (S1) also
raises the per-tick render cost, and D-010 makes 30 ticks/s mandatory —
`simulator.ticks_per_sec` is S0 and immutable. S1 must therefore either find
a render size that holds 30 Hz or apply the observation model in normalized
coordinates with an explicit quantization term. The conflict is recorded in
the S1 TODO rather than resolved by silently slowing the simulator.

### 2.2 Effects owned by NO level (declared out of scope)

Optical medium (scattering, turbidity, illumination), tether dynamics, water
currents (the real pool cannot produce them —
`config/real_pool/pool_mission_profile.yaml`), and engine-side collision
physics (absent by construction, D-003). These are threats to validity to be
discussed in the paper, not silent gaps: a reader must be able to see that
they were left out on purpose.

### 2.3 Divergence from the draft paper table

`paper/tables/tab_calibration_levels.tex` currently places *frame rate* at S1.
Here it is at S2, with camera geometry alone at S1. The reason is the
ownership rule: frame rate is an arrival-process property and is inseparable
from jitter, latency and dropout, which are S2. **The LaTeX table must be
updated to match before submission** — it is the same ladder, described
inconsistently.

## 3. Freeze rule

1. **S0 is IMMUTABLE.** It is a record of what the simulator did on
   2026-08-14. If the simulator changes, do not edit S0: add a decision entry
   in `docs/SCIENTIFIC_DECISIONS.md`, create `s0b_*.yaml`, and re-run.
   Editing S0 invalidates every campaign that referenced it.
2. **A level is frozen when it is first used to produce campaign data.**
   From that moment its values may not change; a change requires a new
   decision entry, a new file, and a full re-run of every campaign at that
   level and above (same rule as D-012, D-013, D-016).
3. **A level may only be run when it has no nulls.** Enforced by
   `load_profile.py --require-measured`. A campaign script must call the
   loader in that mode and record the resolved profile with the run.
4. **Nulls are filled by measurement, never by tuning against outcomes.**
   No level is tuned against avoidance results; that is what licenses
   attributing a change in predictivity to the rung that changed.
5. **Pilot data is admissible as calibration input only.** The 2026-08-14 wet
   runs (`experiments/real/`, see its README and `PILOT_MANIFEST.json`) may be
   used to FIT these profiles. None of their numbers may appear as results.
6. **Every change to a profile is logged** in `docs/SCIENTIFIC_DECISIONS.md`
   with the measurement that justified it.

## 4. Using the loader

```
python scripts/calibration/load_profile.py S3            # report + ownership
python scripts/calibration/load_profile.py S2 --json     # merged values
python scripts/calibration/load_profile.py S3 --todo     # what is missing
python scripts/calibration/load_profile.py S1 --require-measured   # gate
python scripts/calibration/test_load_profile.py          # 20 tests
```

The loader rejects: a broken parent chain, a non-immutable S0, an S0 with a
null or an unsourced leaf, a level overriding a section it does not own, two
levels touching one key, a null without a TODO, a value without provenance,
and a stale TODO/provenance entry that matches no override.

## 5. State on 2026-08-14

`S3` currently merges to 104 leaves, **43 of them null**. What is measured:

| Level | Filled | Null |
|---|---|---|
| S0 | 98 / 98 | — |
| S1 | HFOV 74 deg (D-015); single-class scene; detector gate (D-016) | 18: resolution, VFOV, distortion, refraction, detection probability, confidence, range gates, bearing/size bias and scatter, false positives, occlusion, edge clip |
| S2 | rate is detector-limited; the stream IS bursty | 9: rate, jitter, latency and its jitter, arrival process, stamp source, burst model and its two run lengths |
| S3 | sim<->real sign map; yaw asymmetry 2.14x; surge t63 1.44 s; yaw t63 1.04 s; yaw decay 0.21 s | 16: all limits, command latency, all deadbands, surge and sway asymmetry, sway response entirely, surge decay, setpoint rate limits, coasting |

Three honest caveats a reviewer will ask about:

* **Sway is entirely uncharacterized** — no lateral displacement was ever
  ground-truthed — and the adopted real behaviour is a *lateral* clearing
  maneuver. This is the most consequential gap in the ladder.
* **The yaw asymmetry (2.14x) rests on one pulse per direction**, and the
  separate yaw-authority sweep is non-monotonic in commanded level, so it
  cannot corroborate it. The value is present but carries a replication TODO;
  the loader reports it under "valued but flagged".
* **A ~1.8x surge forward/reverse asymmetry has been quoted but no committed
  artifact supports it**: the paired pulses give 1.05x on drift-contaminated
  EKF speed, and the overhead runs that would settle it are aborted with
  implausible tracks. It stays null.

## 6. Implementation gap (structure exists, injection does not)

These files declare *what* must be calibrated and *by what*. The simulator
does **not** yet have the machinery to apply most of it:

* `observation.*` — no per-detection noise, miss, or false-positive injection
  exists between the oracle and `/perception/obstacles_raw`. The dropout
  relay can script a silence window and one outlier, nothing else. An
  observation-model relay node is needed.
* `timing.*` — no delay line and no rate limiter exist on the perception
  path; observations are published inside the state callback at tick rate.
* `vehicle.*` — the dynamics config has `command_latency_s` and setpoint rate
  limits, but no deadband and no per-direction gain terms.
* `experiments/real/observation_model/` — the artifact the S1/S2 TODOs point
  at does not exist yet (parallel task).

Building that machinery is deliberately out of scope here: a profile that
promises effects the simulator cannot apply is a lie the loader cannot catch,
so it is recorded as a gap instead. Until it is built, `--require-measured`
failing is the correct behaviour, not a blocker to work around.
