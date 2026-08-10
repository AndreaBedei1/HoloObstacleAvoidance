# BlueROV2 Simulation Dynamics — Motion Mode `dynamics`

Replaces teleport for all scientific experiments. The vehicle is moved by the
engine's rigid-body physics (buoyancy, drag, damping, collisions) driven by
our controller through control scheme 0 (8 thruster forces).

## Architecture

```
/planner/cmd_vel_safe (Twist, body velocities)
  -> TCP cmd_vel frame (surge, sway, heave, yaw_rate)   [unchanged interface]
  -> sim server apply_command (existing limit clamps)
  -> CommandLatencyBuffer (configurable command_latency_s; S2 calibration)
  -> setpoint rate limiter (max_lin_accel, max_yaw_accel)
  -> PI body-velocity controller (surge/sway/heave) + PI yaw-rate
     + PD roll/pitch stabilization to zero
  -> body wrench [Fx Fy Fz Mx My Mz]
  -> pseudo-inverse thruster allocation (engine-verified geometry)
  -> direction-preserving per-thruster saturation (28.75 N)
  -> agent.act(8 forces) -> engine physics -> PoseSensor/VelocitySensor/IMU
```

Implementation: `holoocean_server/bluerov2_dynamics.py` (pure numpy, shared
with unit tests), wired in `holoocean_sim_server.py` (`motion_model:
dynamics`). Feedback per tick: world velocity from VelocitySensor rotated to
body via PoseSensor rotation; angular rates from IMUSensor; roll/pitch from
the pose rotation. In dynamics mode the server's published pose comes from
PoseSensor (engine truth), not the kinematic integrator, and collisions are
physically possible again (they were impossible under teleport).

Per-tick logging (in the TCP state header, `dynamics` key): target,
rate-limited setpoint, achieved body velocity, angular rates, roll/pitch,
8 thruster forces, wrench. The ROS bridge can ignore or republish it.

## Verified step responses (this machine, fresh engine process)

`scripts/run_bluerov2_step_response.py` → `experiments/simulation/step_response_S0/`
(manifest with commit SHA, config, summaries; raw per-tick JSON).

| Axis | Amplitude | Rise 10–90% | Time to 90% | SS error | Overshoot |
|---|---|---|---|---|---|
| surge | 0.40 m/s | 3.03 s | 3.20 s | 4.8 % | none |
| sway | 0.25 m/s | 2.90 s | 3.03 s | 4.5 % | none |
| heave | −0.20 m/s | 2.03 s | 2.13 s | 3.2 % | none |
| yaw rate | 0.30 rad/s | 0.20 s | 0.23 s | 0.6 % | none |

Note: rise times include the setpoint rate limiter (max_lin_accel = 1.0 m/s²
→ the setpoint itself takes 0.4 s to reach 0.4 m/s) plus vehicle drag
dynamics. These are the S0 (uncalibrated) responses; S3 calibration will tune
gains/limits/latency to match the *measured real* BlueROV2 responses, and the
same script re-runs the comparison.

## Debugging history (kept for the record)

The first controller attempt used the thruster geometry from the Python agent
docstring and produced inverted surge, dead heave, and yaw spin-up. Open-loop
probes (`scripts/probe_bluerov2_frames.py`) + the engine C++ source proved the
docstring matrices wrong; the corrected client-frame geometry reproduces all
probe observations. Details in `docs/BLUEROV2_HOLOOCEAN_INTEGRATION.md`.
Failure artifacts preserved in git history (first step_response_S0 run).

## Unit tests

`src/rov_obstacle_sim_bridge/test/test_bluerov2_dynamics.py` (19 tests):
allocation-matrix rank and wrench reproduction; engine-verified sign
conventions for surge/sway-left/yaw-CCW; direction-preserving saturation;
latency buffer timing; PI convergence on a first-order plant for
surge/sway/yaw; setpoint rate limiting; attitude-stabilization signs; config
parsing (unknown keys rejected); frame helpers.

## Open items

- [x] Command watchdog in the server: `sim.cmd_timeout_s` (default 1.0 s,
      `<=0` disables) zeroes the commanded velocity when no fresh cmd_vel
      frame arrives; smooth stop through the setpoint rate limiter; state in
      the TCP header (`watchdog_active`); 8 unit tests.
- [x] Real-time pacing + physics-time consistency: see D-010 in
      `docs/SCIENTIFIC_DECISIONS.md` — `ticks_per_sec: 30` is mandatory;
      serve loop paces with a 1 ms-resolution hybrid sleep/spin.
- [x] Step-response repeatability: 3 consecutive fresh-process runs completed
      2026-08-10 (17:07, 17:10, 17:11 UTC — separate engine processes) with
      bit-identical per-axis summaries (`experiments/simulation/
      step_response_S0{,_run2,_run3}/manifest.json`). Must be repeated after
      any gain change.
- [ ] S2: set `command_latency_s` from measured real command-path latency.
- [ ] S3: gain/drag matching against real step responses (pool phase).
