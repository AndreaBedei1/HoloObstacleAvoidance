# Real Pool Experiment Log

Structured, append-only. Every actuation session gets an entry:
date/time (topside PC clock), git SHA, hardware state, mode, commands,
measured response, anomalies, changes.

---

## 2026-08-14 — Session 1: Phase 9 bring-up (first in-water actuation)

- **Git SHA at session start:** ba6c7d6 (+ Phase-9 scripts uncommitted)
- **Hardware:** ROV in pool (Andrea), battery 15.9 V, leak OK,
  depth sensor reading 0.45 m at float trim, water temp ~31 °C zone.
  Overhead RealSense mounted (ROTATED since 2026-08-13 survey — old
  camera→pool transform INVALID, remap pending).
- **Vehicle software:** BlueOS 1.3.1, ArduSub 4.1.2, FRAME 2 (Heavy).
- **Pool:** ~6 m, anchor on the transverse rod (from yesterday's survey;
  remap pending). Anchor VISIBLE from the onboard camera at session
  start (frame_20260814_155428_2.png — thin shank, wide flukes ✓ the
  taper Andrea described).

### Tests executed (all MANUAL mode, 15% power, 1.2 s pulses)

| # | Test | Result |
|---|---|---|
| 1 | Lights1 flash 0→50%→0 (RC9 override) | command path OK (log lights_155628) |
| 2 | Static arm 4 s neutral | PASS — 8×PWM=1500 for 141 samples, clean disarm |
| 3 | surge+ | PASS — thr1-4 +30 µs, yaw drift −0.5°, depth const |
| 4 | surge− | PASS — thr1-4 −30 µs, +3.3° yaw asymmetry noted |
| 5 | sway− | PASS — (+,−,−,+) pattern |
| 6 | sway+ | PASS on retry (first attempt: arming refused, transient; retried OK) |
| 7 | yaw+ | PASS — +6.4° |
| 8 | yaw− | PASS — −13.7° (stronger than yaw+, asymmetry to characterize) |

Logs: `experiments/real/motion_tests/*_20260814_*.jsonl` (ATTITUDE,
VFR_HUD, SCALED_PRESSURE2, SERVO_OUTPUT_RAW, DISTANCE_SENSOR,
HEARTBEAT + command records, 15 Hz).

### Anomalies / notes

- FCU auto-reverts mode to STABILIZE after some disarms → mode is now
  always set explicitly pre-arm.
- One transient arming refusal (test 6 first attempt) — retry succeeded;
  watch for recurrence.
- Ping1D DISTANCE_SENSOR reads 75.29 m (no lock) pre-dive; orientation
  unknown — characterize submerged.
- Disarm state sampling at 0.5 s was too early (showed stale armed=True);
  settle raised to 1.5 s; actual disarm verified via mavlink2rest each
  time.
- MAV_STATE_CRITICAL appears whenever no GCS heartbeat is streaming
  (FS_GCS_ENABLE=2); clears to STANDBY/ACTIVE during sessions. Benign
  when disarmed.
