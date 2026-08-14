# Real BlueROV2 — Definitive Technical Reference

Vehicle: BlueROV2 **Heavy** (8 thrusters), tether to topside PC.
Everything here was VERIFIED on the actual vehicle on 2026-08-14 unless
marked otherwise. Machine-readable data in `data/`.

## Network / access map

| What | Address | Verified |
|---|---|---|
| Vehicle (BlueOS on Raspberry Pi) | 192.168.2.2 | ✔ ping 4-13 ms |
| Topside PC | 192.168.2.1 (Ethernet 2) | ✔ |
| MAVLink to PC | BlueOS "GCS Client Link" **udpout → 192.168.2.1:14550** (enabled) | ✔ used by all scripts |
| mavlink2rest (REST telemetry) | http://192.168.2.2:6040/v1/mavlink/... | ✔ |
| BlueOS web / APIs | http://192.168.2.2/ (version-chooser, ardupilot-manager, ping service :9110, camera manager :6020) | ✔ |
| Onboard camera | H.264 RTP **udp://192.168.2.1:5600** ("UDP Stream 0", /dev/video2 USB) + redirect :5601 ("Prova1") | ✔ frames captured |
| Ping1D bridge | UDP 192.168.2.2:9090 (BlueOS ping service) + DISTANCE_SENSOR on MAVLink component 194 | ✔ message seen |
| **Imaging sonar (Cerulean)** | **192.168.2.86 — NEVER USE / NEVER ACTIVATE.** Present on the network, driver disabled. Total project exclusion. | presence only |

## Software (verified 2026-08-14)

- BlueOS **1.3.1** (arm, sha d47b4b32…)
- ArduSub **4.1.2** — MAV_TYPE_SUBMARINE, autopilot ArduPilotMega
- mavlink-camera-manager 0.2.4 (no RTSP for this config: use UDP 5600)
- FRAME_CONFIG = 2 → **VECTORED_6DOF** (Heavy); STATUSTEXT at boot
  confirms "Frame: VECTORED_6DOF"

## Key parameters (data/mavlink_inventory.json for the full dump)

| Param | Value | Meaning |
|---|---|---|
| FS_PILOT_INPUT / FS_PILOT_TIMEOUT | 2 / 3.0 | **disarm 3 s after pilot stream stops** (our hardware backstop) |
| FS_GCS_ENABLE | 2 | GCS-loss failsafe (CRITICAL state without our 1 Hz heartbeat — benign when disarmed) |
| FS_LEAK_ENABLE | 1 | leak → warn |
| SERVO1..8_FUNCTION | 33..40 | Motor1..Motor8 (8 thrusters) |
| SERVO13/14_FUNCTION | 59/60 | RCIN9/RCIN10 = **Lights1/Lights2** |
| SERVO16_FUNCTION | 7 | mount tilt (camera servo) |
| MOT_PWM_MIN/MAX | 1100/1900 | thruster PWM range, 1500 neutral |
| RNGFND1_TYPE | 0 | Ping1D NOT an ArduPilot rangefinder — read it via MAVLink comp 194 DISTANCE_SENSOR or BlueOS :9110/UDP 9090 |
| BATT_MONITOR | 4 | V+I analog |

Measured stream rates (defaults): ATTITUDE/VFR_HUD/SCALED_PRESSURE2/
SERVO_OUTPUT_RAW ≈ 12 Hz (raisable via MAV_CMD_SET_MESSAGE_INTERVAL,
verified at 15 Hz), SYS_STATUS ≈ 2.6 Hz.

## ArduSub modes (custom_mode ids)

0 STABILIZE · 1 ACRO · 2 ALT_HOLD · 3 AUTO · 4 GUIDED · 7 CIRCLE ·
9 SURFACE · 16 POSHOLD · 19 MANUAL · 20 MOTOR_DETECT.
No GPS → POSHOLD/AUTO unusable; project modes: MANUAL (open loop),
STABILIZE (attitude hold), ALT_HOLD (attitude + depth hold).
Note: after disarm the FCU sometimes reverts to STABILIZE on its own —
ALWAYS set the mode explicitly before arming.

## Command reference (all tested — see scripts/real/)

Connection (pymavlink 2.4.49, env `ros2_lyrical`):
```python
from scripts.real.rovlink import RovLink
with RovLink() as rov:                  # udpin:0.0.0.0:14550
    print(rov.heartbeat())              # {'mode','armed','system_status'}
```
`RovLink` streams a GCS heartbeat at 1 Hz (clears FS_GCS) and on exit
ALWAYS sends neutral + disarm (context-manager safety contract).

| Action | Call (verified) |
|---|---|
| read attitude | `rov.recv_match("ATTITUDE")` → roll/pitch/yaw rad |
| read depth | `rov.recv_match("VFR_HUD").alt` = −depth m (or SCALED_PRESSURE2 mbar) |
| read battery | `rov.recv_match("SYS_STATUS")` (15.9 V measured) |
| set mode | `rov.set_mode("MANUAL")` (or STABILIZE/ALT_HOLD) |
| arm / disarm | `rov.arm()` / `rov.disarm()` |
| surge fwd/back | `rov.pulse(x=+150, duration_s=1.2)` / `x=-150` |
| sway right/left | `rov.pulse(y=+150, ...)` / `y=-150` |
| yaw CW/CCW | `rov.pulse(r=+150, ...)` / `r=-150` |
| heave down/up | `rov.pulse(z=350, ...)` / `z=650` (500 = neutral) |
| stop | `rov.neutral()` (MANUAL_CONTROL zeros; keep streaming) |
| lights | `rov.lights(0.5)` → RC9 override 1100–1900 µs |
| camera frame/video | `python scripts/real/camera_grab.py --frames 3` / `--video 10` |
| Ping1D range | GET :6040 `.../components/194/messages/DISTANCE_SENSOR` (cm) |
| safe shutdown | leave the `with` block (neutral + disarm) or kill the process (FS_PILOT disarms in 3 s) |

MANUAL_CONTROL scaling measured: |x|=150/1000 → ±30 µs on the mixed
thrusters (~7.5% of half-range); gentle and ideal for first tests.

## Verified thruster mixing (2026-08-14, 15% pulses, in water)

| Command | Thr 1-4 PWM deltas | Thr 5-8 | Yaw response |
|---|---|---|---|
| surge+ | (+,+,+,+) | 0 | −0.5° |
| surge− | (−,−,−,−) | 0 | +3.3° (slight asymmetry) |
| sway+ (right) | (−,+,+,−) | 0 | +3.8° |
| sway− (left) | (+,−,−,+) | 0 | +0.2° |
| yaw+ (CW) | (−,+,−,+) | 0 | **+6.4°** ✓ |
| yaw− (CCW) | (+,−,+,−) | 0 | **−13.7°** ✓ |

Static arm test: 141 SERVO_OUTPUT_RAW samples, all eight = 1500 while
armed at neutral ✓. Depth held 0.45 m throughout all horizontal tests
(verticals untouched by the mixer) ✓.

## Measured dynamic response (2026-08-14, 20% steps, 3 s)

| Quantity | Real measured | HoloOcean model | Note |
|---|---|---|---|
| cmd->PWM latency | 53-89 ms | 0 (D-model) | one telemetry frame |
| first motion (surge) | 0.36 s | - | spin-up + inertia |
| surge v @20%, 3 s | 0.102 m/s | (0.5 max_surge) | ~linear extrap ~0.5 m/s full |
| surge rise t63 | ~1.44 s | tau_surge 1.2 s | GOOD agreement |
| surge coast to 37% | ~1.2 s | - | IMU-drift caveat |
| first gyro response (yaw) | 0.38 s | - | |
| yaw rate @20% | 0.192 rad/s | max 0.3 rad/s cmd | |
| yaw rise t63 | ~1.04 s | tau_yaw 0.3 s | **3x slower than sim** |
| yaw decay after release | 0.21 s | - | asymmetric (drag-dominated) |
| yaw asymmetry | yaw- stronger than yaw+ | none | to quantify |

IMU: RAW_IMU is the live stream (mg, zacc -998 at rest);
SCALED_IMU2 is all zeros (no secondary IMU) - use RAW_IMU.

## Control modes - tested findings

- **ALT_HOLD = the science mode**: depth held rock-solid (0.45 m const)
  with verticals actively trimming (1421-1576 PWM) while horizontal axes
  respond to MANUAL_CONTROL. Mirrors the HoloOcean architecture (planner
  horizontal + control layer holds depth).
- STABILIZE: attitude hold, depth free. MANUAL: fully open loop.
- ArduSub BOUNCES the mode (often to STABILIZE) around GCS-failsafe
  transitions (our heartbeat stops between scripts). Recipe that works:
  set mode -> arm (ACK + retry) -> RE-ASSERT mode after arming
  (implemented in rovlink.arm(mode=...) + rov_motion_test post-arm
  assert). Occasional first-arm refusals: retry succeeds.

## Sensors

- IMU/AHRS: ATTITUDE ~12 Hz; compass heading in VFR_HUD.
- Depth: SCALED_PRESSURE2 (external barometer, mbar) → ArduSub fuses to
  VFR_HUD.alt (= −depth). In air 1015 mbar; in water reads 0.45 m at
  float trim.
- Ping1D: /dev/ttyAMA3, DISTANCE_SENSOR id 1 comp 194, min 20 cm,
  "max 120 m" configured; first reading 75.29 m = NO LOCK (orientation
  and in-water behavior to be characterized). NOT the primary perception
  sensor — supplementary only.
- Camera: USB H.264 1920×1080 ("H264 USB Camera", /dev/video2), RTP to
  UDP 5600. Capture recipe (Windows, no GStreamer): SDP file + OpenCV
  FFMPEG with `OPENCV_FFMPEG_CAPTURE_OPTIONS=protocol_whitelist;file,rtp,udp`
  set BEFORE cv2 loads (scripts/real/camera_grab.py self-reexecs).
- Leak/temp/voltage via SYS_STATUS + BlueOS.

## Safety invariants (Phase 9)

1. The imaging/side-scan sonar is permanently excluded from everything.
2. Motion scripts: bounded pulses (≤40% power, ≤4 s), context-manager
   neutral+disarm, ArduSub FS_PILOT 3 s as backstop.
3. Every actuation session appends to docs/REAL_POOL_EXPERIMENT_LOG.md.
