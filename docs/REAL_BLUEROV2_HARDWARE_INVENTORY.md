# Real BlueROV2 — Read-Only Hardware Inventory

Acquired 2026-08-10 from the live, connected, **dry** vehicle. Every interaction
was read-only (HTTP GETs on BlueOS APIs, MAVLink telemetry reads, parameter
reads via `PARAM_REQUEST_READ`). No arming, no actuation, no configuration
writes, no sonar interaction, no reboots.

Machine-readable version: `config/real_bluerov2_detected.yaml`.
Sanitization: no credentials, tokens, or MAC addresses are recorded here.

## Vehicle and software

| Item | Detected value |
|------|----------------|
| Companion computer | Raspberry Pi 4 (ARM Cortex-A72 ×4 @ 900 MHz-governed) |
| BlueOS | core 1.3.1 (bluerobotics/blueos-core, arm, image 2024-08-26) |
| Autopilot board | Blue Robotics **Navigator** (RPi HAT) |
| Firmware | **ArduSub 4.1.2 STABLE** |
| MAV type | MAV_TYPE_SUBMARINE, ArduPilotMega, system id 1 |
| Frame | **FRAME_CONFIG = 2 → SUB_FRAME_VECTORED_6DOF = BlueROV2 Heavy, 8 thrusters** |
| Thruster outputs | SERVO1..8_FUNCTION = 33..40 (Motor1..Motor8); MOT_PWM 1100–1900 µs |
| Aux outputs | servo13/14 = lights channels (observed 1100 = off), servo16 = camera tilt (1500 = centered) |
| State at inventory | Disarmed (base_mode 81), mode MANUAL (custom_mode 19), MAV_STATE_CRITICAL (failsafe while no pilot/GCS input; FS_PILOT_INPUT=2) |
| BlueOS extensions | Cockpit v1.0.2; **Cerulean SonarView 1.14.4** (imaging sonar viewer — prohibited device, untouched); major_tom (cloud agent) |

## Network topology

| Node | Address | Notes |
|------|---------|-------|
| Topside PC (this machine) | 192.168.2.1/24 ("Ethernet 2") | experiment computer |
| BlueOS / vehicle | 192.168.2.2 (eth0); also WiFi AP 192.168.42.1, USB 192.168.3.1 | ping RTT ≈ 4 ms |
| Third device | 192.168.2.86, STMicroelectronics OUI, TCP `62312` | **Cerulean Surveyor 240-16**, used only through the dry-mode locked/replay viewer; no acoustic transmission while out of water |

## MAVLink telemetry actually streaming (31 message types)

Rates observed via mavlink2rest:

- 10 Hz: `ATTITUDE` (roll/pitch/yaw + rates), `AHRS2`, `VFR_HUD` (incl. depth as alt, groundspeed, heading)
- 3 Hz: `AHRS`, `BATTERY_STATUS`, `EKF_STATUS_REPORT`, `GLOBAL_POSITION_INT`, `HWSTATUS`, `MOUNT_STATUS`, `VIBRATION`, `SYSTEM_TIME`
- 2 Hz: `GPS_RAW_INT` (**NO_FIX, 0 sats** — see position locator), `RAW_IMU`, `SCALED_IMU2`, `SCALED_PRESSURE` (internal baro), `SCALED_PRESSURE2` (**external water pressure = depth sensor**, Bar30-class), `SERVO_OUTPUT_RAW`, `RC_CHANNELS`, `SYS_STATUS`, `POWER_STATUS`, `MEMINFO`, `MISSION_CURRENT`, `NAV_CONTROLLER_OUTPUT`
- 16 Hz: `NAMED_VALUE_FLOAT` (ArduSub pilot values, e.g. RollPitch)
- 1 Hz: `HEARTBEAT`; sporadic: `STATUSTEXT`, `TIMESYNC`, `SENSOR_OFFSETS`, `PARAM_VALUE`, `COMMAND_ACK`, `AUTOPILOT_VERSION`

Observed sample values (dry, on bench): external pressure 1012.1 hPa (≈ ambient →
confirms out of water), internal 1040.6 hPa, VFR_HUD alt −0.45 m, attitude valid,
IMU live.

### Navigation-relevant conclusions

- **NO DVL**: no DVL extension, no `VISION_POSITION_DELTA` / `ODOMETRY` /
  `LOCAL_POSITION_NED` messages, no velocity aiding of any kind.
- **NO rangefinder feed**: no `RANGEFINDER` / `DISTANCE_SENSOR` messages (the
  Ping1D driver is enabled in BlueOS but produced nothing while dry).
- Available real navigation signals: attitude+rates (10 Hz), heading, external
  pressure depth (2 Hz), raw IMU (2 Hz effective on this stream config),
  EKF status. Telemetry rates are requestable higher via `SR0_*`/message
  intervals — **not changed during this inventory** (would be a parameter write).

## Sonar devices (RESTRICTED / DRY MODE)

- **Multibeam imaging sonar**: Cerulean Surveyor 240-16 at `192.168.2.86:62312`
  + SonarView extension. **TX locked while dry**; only passive/replay decoding
  is permitted until explicit in-water authorization.
- **Ping1D 1-D echosounder**: Ping service reports a `Ping1D` device_id 1,
  firmware 1.0.0 on `/dev/ttyAMA3`, UDP driver port 9090, `mavlink_driver_enabled:
  true`. Configuration inspected only; no ping/range request issued. Not a
  dependency of the first camera-based paper; may be considered only after
  explicit in-water authorization.

## Position locator

- BlueOS **NMEA Injector** has one configured UDP sock: **port 27000 →
  MAVLink component 220** (GPS_INPUT path). This is the standard injection route
  for an external acoustic positioning topside (e.g. Water Linked UGPS or Cerulean
  ROV Locator feeding NMEA).
- `GPS_RAW_INT` currently NO_FIX / 0 satellites — no locator data while dry, and
  no locator topside receiver is currently attached to the experiment PC (no
  matching USB serial device found).
- **Open question for Andrea:** exact locator model + where its topside software
  runs. Runtime use for navigation state is *possible* via GPS_INPUT→EKF, but
  must be validated (update rate, latency, pool multipath) before the planner may
  rely on it. Acoustic devices stay off until in-water authorization.

## Onboard camera

| Item | Detected value |
|------|----------------|
| Source | "H264 USB Camera: USB Camera" at `/dev/video2` (USB) — Blue Robotics H.264 low-light class; exact module to be confirmed physically |
| Stream | H.264, **1920×1080 @ 30 fps**, RTP/UDP to `192.168.2.1:5600` (running) |
| Second stream | "Prova1": redirect endpoint at `udp://192.168.2.1:5601` (running) |
| Tilt | Mount/servo16 present, currently centered (1500 µs); MOUNT_STATUS streaming |
| Intrinsics/FOV | NOT measured yet — requires calibration (dry + underwater); do not trust datasheet |
| Latency | NOT measured yet — needs receive-side measurement (glass-to-glass) |

## Explicitly NOT determined (needs physical access or wet access)

- Exact BlueROV2 revision/serials (readable on hardware labels; keep out of git)
- Camera module exact model + lens; camera-to-body transform
- Underwater intrinsics/FOV; stream latency distribution
- Locator model/protocol details; Ping1D mounting orientation
- Battery/thruster serial numbers

## Safety statement

All discovery was passive/read-only. The single MAVLink message type sent was
`PARAM_REQUEST_READ` (parameter download, explicitly allowed). No ARM, no RC
override, no motor test, no mode change, no parameter write, no camera tilt
command, no lights, no sonar traffic, no reboot.
