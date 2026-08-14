# Real BlueROV2 — Camera-Based Obstacle Avoidance (2026-08-14)

Presentation package. Pool trials with a tethered BlueROV2 Heavy avoiding
a suspended anchor using ONLY its onboard camera. An overhead RealSense
provides independent ground truth and a safety supervisor; it never
feeds the avoidance logic.

## Headline

* **8 autonomous avoidance runs succeeded** (obstacle detected from the
  camera, committed lateral clearing, straight pass, no operator input).
* Closest approach on the successful runs: **0.80-0.91 m** from the
  anchor (vehicle half-width 0.28 m, anchor arm span 0.80 m).
* **No trained detector exists for this setup**: the anchor is found by
  a classical shape-based detector written for this project.
* The most informative run is 19:50 — it completed the maneuver with
  **only 4% of frames yielding an accepted detection**, which is exactly
  the property the simulation campaign attributed to committed
  avoidance: the maneuver does not need continuous perception.

## Run table

| Session | Result | Trigger range | Anchor bearing | Closest approach (GT) | Detection rate |
|---|---|---|---|---|---|
| 18:51 | passed | 1.26 m | +12.2° | 1.33 m | 18% |
| 18:53 | passed | 1.29 m | +2.1° | 2.55 m | 21% |
| 18:56 | passed | 1.21 m | −16.9° | 1.85 m | 25% |
| 19:00, 19:02, 19:09 | aborted at t=0 by the wall guard (release position too close to the pool edge) | — | — | — | — |
| 19:05 | passed | 1.21 m | −10.4° | 0.87 m | 8% |
| 19:10 | passed | 1.42 m | −2.9° | 0.80 m | 46% |
| 19:38 (pilot 1) | **contact** | 1.21 m | +20.8° | 0.32 m | 44% |
| 19:41 (pilot 2) | passed | 1.42 m | +1.5° | 0.91 m | 37% |
| 19:50 (pilot 3) | passed | 1.21 m | −19.4° | 0.85 m | **4%** |

Runs before 19:38 are development runs: the behaviour was still being
reshaped. Runs 19:38 onward use the frozen configuration (D-016).

## The failure that mattered

Pilot 1 (19:38) drove into the anchor. The configuration at that point
forced "always clear to the right", per an operator preference. The
anchor was itself 20.8° to the RIGHT, so the vehicle steered into it.
The rule was corrected to *clear away from the obstacle bearing* and the
freeze re-issued before the campaign.

This failure mode **cannot occur in the simulation**, where the obstacle
is always centred on the approach line. It is a direct argument for the
sim-to-real part of the study.

## Files

| What | Where |
|---|---|
| Onboard camera video with detector overlay | `video_<session>.mp4` |
| Overhead trajectory with anchor and closest approach | `traj_<session>.png` |
| Distance vs time (camera range + ground truth, phases shaded) | `range_<session>.png` |
| Detector contact sheet (approach → trigger → clearing → passed) | `sheet_<session>.png` |
| Per-run numbers | `missions_summary.json` |
| Monocular range accuracy vs ground truth | `range_error.json` |

Raw session recordings (full video, per-frame captures) are NOT in the
repository; they live under `experiments/real/missions/<session>/` and
`experiments/real/anchor_dataset/` on the experiment PC.

## Measured sim-to-real discrepancies

| Quantity | Simulation | Real vehicle |
|---|---|---|
| Yaw time constant | 0.3 s | **1.04 s** (3× slower) |
| Surge time constant | 1.2 s | 1.44 s |
| Command deadband | none | **~30-35%** of authority (tether drag) |
| Perception availability | ~100% | **4-46%** of frames |
| Range estimate | oracle-derived | monocular, MAE **0.44 m** over 1.3-2.6 m |
| Stopping | settles quickly | coasts and keeps rotating; needs an active hold |

## Next step

Ten repetitions with this frozen configuration, in stable morning light,
per `docs/PHASE9_REAL_CAMPAIGN_PROTOCOL.md`.
