# Real Pool Experiment Log — Phase 9

Vehicle: BlueROV2 Heavy, ArduSub 4.1.2, BlueOS 1.3.1. Pool ~6 m,
suspended anchor on a transverse rod. Overhead Intel RealSense D435 =
external ground truth and safety supervisor (NEVER a planner input).
Imaging/side-scan sonar: present, PERMANENTLY EXCLUDED, never activated.

## Session 2026-08-14 — first real camera-based obstacle avoidance

Software: branch `feature/scientific-sim-to-real-obstacle-avoidance`.
Perception: `scripts/real/anchor_detect.py` (classical, NO trained
weights — none exist for the real setup). Control: `scripts/real/
avoid_mission.py`. Vehicle link: `scripts/real/rovlink.py`.

### Vehicle characterization (measured)

| Quantity | Value | Note |
|---|---|---|
| Thruster directions | all 6 DOF correct | vectored mixing verified on SERVO_OUTPUT_RAW |
| Surge response t63 | ~1.44 s | sim used 1.2 s |
| Yaw response t63 | ~1.04 s | sim used 0.3 s — **3x discrepancy** |
| Command latency | 53-89 ms | MANUAL_CONTROL to thruster PWM |
| Yaw/translation deadband | commands <= 15-22% do not move the vehicle | tether drag; 35% needed for reliable surge |
| Coasting | keeps rotating/translating after thrusters stop | every stop must be an ACTIVE hold |
| Heading hold (P+D on compass) | max error 1.9-4.3 deg, mean 1.3-1.8 deg | `rovlink.hold_heading()` |
| Depth sensor reference | keel (reads 0.45 m when floating with the top at the surface) | operator-confirmed |
| Camera | 1920x1080 H.264 RTP on UDP 5600, ~25 fps | persistent reader required (per-frame open costs 3-5 s) |
| Lights | RC9 override, verified | |
| Ping1D | present, down-facing (PITCH_270), UDP 9090 | not used as obstacle sensor |

### Perception (weight-free detector)

Shape prior: long thin vertical dark shank + arm span, with three
discriminators developed against the real failure modes:
tall-thin morphological opening (kills dome bubbles), straight-line test
(row width + centroid wander), left/right isolation (kills the shaded
pool-edge band). On clean frames it returns a SINGLE candidate on the
anchor.

Measured monocular range error vs overhead ground truth (n = 47 paired
samples, 1.29-2.64 m): **bias -0.26 m, MAE 0.44 m, RMS 0.58 m**
(one-point calibration D-015: f_px = 1277, anchor arm span 0.80 m).

Detection acceptance rate is dominated by dome cleanliness:
8-25% with bubbles, **46% right after wiping the dome**.

### Missions (8 runs)

| Session | Result | Trigger range | Bearing | Min GT distance | Detect rate |
|---|---|---|---|---|---|
| 18:44 (avoid_run) | avoided, range grew 1.24 -> 2.46 m | 1.24 m | -2.1° | — | 16% |
| 18:51 | PASSED (window) | 1.26 m | +12.2° | 1.33 m | 18% |
| 18:53 | PASSED (window) | 1.29 m | +2.1° | 2.55 m | 21% |
| 18:56 | PASSED (window) | 1.21 m | -16.9° | 1.85 m | 25% |
| 19:00, 19:02, 19:09 | ABORTED by wall guard at t=0 | — | — | — | 0% |
| 19:05 | **PASSED (GT anchor plane)** | 1.21 m | -10.4° | 0.87 m | 8% |
| **19:10** | **PASSED (GT anchor plane)** | **1.42 m** | **-2.9°** | **0.80 m** | **46%** |

Zero collisions, zero uncommanded excursions, every run ended in an
active heading hold.

Additional runs after the D-016 freeze: pilot 1 (19:38) drove into the
anchor (closest approach 0.32 m) because the configuration forced
"always clear right" while the anchor was itself to the right; the rule
was corrected to clear-away-from-bearing before the campaign. Pilot 2
(19:41) and pilot 3 (19:50) passed at 0.91 m and 0.85 m. One further run
(19:45) was TERMINATED ABNORMALLY (process killed mid-run): the trial
never completed, its outcome is undefined, and it is excluded from every
results artifact — recorded here so the total run count reconciles.

### Behaviour finally adopted (operator-driven)

APPROACH straight -> TRIGGER on monocular range -> LATERAL clearing to
the right ONLY while the anchor is still seen -> STRAIGHT ahead once it
is cleared (sideways travel walks into the pool walls) -> active hold.
Wall guard is RELATIVE to the release point: the operator releases the
vehicle from the pool edge, so an absolute margin aborted every run.

### Sim-to-real discrepancies measured

1. **Yaw time constant 1.04 s vs 0.3 s in HoloOcean** — the largest
   dynamic discrepancy; the sim over-estimates yaw agility ~3x.
2. **Command deadband**: the real vehicle needs >= 30-35% authority to
   move at all (tether drag); the sim has no deadband.
3. **Post-command coasting** with no damping: the sim vehicle stops far
   more readily than the real one.
4. **Monocular range noise**: MAE 0.44 m at 1.3-2.6 m, i.e. ~25% of the
   range — far noisier than the simulated oracle-derived estimate.
5. **Perception availability**: 8-46% of frames vs ~100% in simulation.

### Open items

* Trained detector: no weights exist for the real anchor; the classical
  detector is the current solution and its acceptance rate is the
  limiting factor.
* The overhead GT distance is measured to the anchor's suspension pixel
  with an oblique camera; a proper homography would tighten it.
* Multi-repetition campaign for statistics (only 5 successful runs so
  far, in varying start geometry).
