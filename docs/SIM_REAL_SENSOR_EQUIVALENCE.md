# Sim ↔ Real Sensor Equivalence

Critical check mandated before designing the final estimator/planner: the runtime
state estimation must be based on sensors that actually exist on the real
platform. Real side from the 2026-08-10 read-only inventory
(`docs/REAL_BLUEROV2_HARDWARE_INVENTORY.md`); sim side from the repo audit
(`docs/SCIENTIFIC_DECISIONS.md` D-003) and the HoloOcean BlueROV2 agent.

| Quantity | Simulation source (current baseline) | Real source | Equivalent? | Action |
|----------|--------------------------------------|-------------|-------------|--------|
| RGB image | HoloOcean RGBCamera 512×512 RGBA, socket CameraSocket | H.264 USB cam 1920×1080@30 via UDP 5600 | **No** (resolution, FOV unverified, codec latency) | S1 calibration: set sim camera to measured real resolution/FOV/fps; measure real stream latency; propagate image stamps end-to-end |
| Pressure depth | DepthSensor (clean scalar) | SCALED_PRESSURE2 (Bar30-class, 2 Hz default) | Partial | Add noise/rate model matched to real sensor; raise real stream rate during experiments (documented parameter change, wet phase only) |
| Roll/pitch/yaw | PoseSensor ground truth (planner uses yaw via estimated odometry) | ATTITUDE 10 Hz (EKF, IMU+compass) | Partial | Use attitude as a *measured* input in both domains; add heading noise/bias model in sim |
| Angular velocity | Synthesized from GT finite-diff + noise ("gyro") | ATTITUDE rates / RAW_IMU 10 Hz | Partial | Replace synthesized gyro with sim IMUSensor; match rates/noise to real IMU |
| Translational velocity (body) | Bridge finite-diffs GT pose → "DVL" + noise | **NONE — NO DVL on the real vehicle** | **NO — CRITICAL MISMATCH** | Redesign runtime navigation without velocity sensing (see below). Sim DVLSensor allowed only in explicitly non-transferable variants |
| x/y position | Estimated odometry integrating synthetic DVL+gyro | **No runtime x/y sensor** (locator inactive/unvalidated) | **NO — CRITICAL MISMATCH** | Same redesign; optional: validate position locator (GPS_INPUT path) as a slow aiding source, wet phase only |
| Obstacle measurement | YOLO on sim camera; oracle for validation only | YOLO on onboard camera | Yes (by design) | Keep identical code path; calibrate visual noise model per domain |
| Command feedback | Commanded twist applied by teleport (perfect) | SERVO_OUTPUT_RAW PWM telemetry; no achieved-velocity sensor | **No** | Move sim to thruster-force dynamics; log commanded vs achieved in sim; on real vehicle, only commanded + attitude/depth response observable |
| Collision/contact | None (teleport passes through geometry) | Physical contact | No | Dynamics mode restores collisions in sim; real collisions detected via external ground-truth video review |
| External ground truth | GT pose topic (sim-only) | Overhead RealSense D435 x/y/yaw | Validation-only in both | Enforce `/ground_truth/*` isolation at runtime (Phase 9 guard) |

## Consequence: navigation redesign for a velocity-sensor-less vehicle

The current estimator (integrate DVL+gyro) has **no real counterpart**. The
realistic runtime navigation options, to be developed and compared in Phase 7:

1. **Attitude + commanded-motion dead reckoning (primary candidate):** propagate
   x/y with commanded body velocity (or thrust-model-predicted velocity) rotated
   by measured yaw; measured pressure depth for z. Honest, transfers 1:1, drifts —
   which the committed-circumnavigation planner was already designed to tolerate
   over short horizons.
2. **+ visual obstacle-relative correction:** during avoidance the tracked
   obstacle itself provides a relative reference (bearing/size), reducing the
   effect of dead-reckoning drift exactly where it matters.
3. **+ position locator aiding (optional, wet-validated only):** slow global x/y
   via the GPS_INPUT path if the locator proves usable in the pool.

The sim odometry chain must be rebuilt to emulate option 1 exactly (same inputs,
same noise character measured from real trials), replacing the synthetic
finite-difference "DVL".
