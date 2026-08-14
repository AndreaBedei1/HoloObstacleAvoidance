# Phase 9 — Pre-registered REAL-VEHICLE Avoidance Campaign

STATUS: FROZEN 2026-08-14, before any campaign run. Configuration is
recorded in D-016 (`docs/SCIENTIFIC_DECISIONS.md`). Nothing in the
perception or control code may change between this freeze and the end of
the campaign; any change invalidates the campaign and requires a new
protocol and a full re-run.

## 1. Question

Does the camera-driven committed-avoidance behaviour characterized in
simulation (Phase 8) transfer to the real BlueROV2 in the pool, using
onboard perception only, with a frozen configuration and repeated
trials?

## 2. Setup

* Vehicle: BlueROV2 Heavy, ArduSub 4.1.2, STABILIZE mode, tethered.
* Obstacle: the suspended anchor (~0.80 m arm span) on the transverse
  rod, overhead pixel (1069, 635).
* Perception: `scripts/real/anchor_detect.py` — classical detector, NO
  trained weights (none exist for this setup).
* Controller/mission: `scripts/real/avoid_mission.py`.
* Ground truth / safety: overhead Intel RealSense D435
  (`scripts/real/overhead_track.py`). NEVER an input to the avoidance
  logic. Its absolute metric error is NOT claimed; it is used for
  (a) the along-track pass decision, (b) the clearance proxy, and
  (c) the wall guard. Each run records the detected ROV pixel so the
  operator can confirm the lock visually.
* Imaging/side-scan sonar: present, PERMANENTLY EXCLUDED.

## 3. Run procedure (identical for every repetition)

1. Operator wipes the camera dome (dome bubbles are the dominant
   perception failure: acceptance 8% dirty vs 46% clean).
2. Operator holds the vehicle in the CENTRAL area of the overhead view,
   bow pointing at the anchor, 2.0-2.5 m away, and releases it when the
   run starts.
3. The script: visual centring (<= 6 s) -> APPROACH -> TRIGGER on
   monocular range -> LATERAL clearing to the side AWAY from the
   obstacle bearing (RIGHT when it is dead ahead, |bearing| < 5°) while
   the anchor is still seen -> STRAIGHT ahead once cleared -> active
   heading hold.

   AMENDMENT (pilot 1, 2026-08-14, BEFORE the campaign): the frozen
   configuration initially forced "always clear to the right". Pilot 1
   triggered with the anchor at +20.8° (to the right) and the vehicle
   swerved INTO it — closest approach 0.32 m, i.e. contact by the
   collision definition below. The side-selection rule was corrected to
   the away-from-bearing rule above and the freeze re-issued before any
   campaign run. Pilot 1 is reported as a pilot failure, not excluded
   from the record.
4. No operator intervention between release and the end of the hold.

   AMENDMENT 2 (pilot analysis, 2026-08-14 evening, BEFORE the
   campaign): the trigger fired on the FIRST frame that accepted a
   detection, so the vehicle curved immediately instead of running
   straight first (run 19:41 triggered at t = 1.06 s with only 0.8 s of
   approach). Cause: a single monocular sample (MAE 0.44 m) can read
   1.4 m while the vehicle is at 2 m. Fix: the trigger now needs
   **3 consecutive confirmations** on a **median-filtered range**
   (window 5) — the same confirmation principle as the Phase-7B
   qualification used in simulation, so this brings the real pipeline
   CLOSER to the simulated one. Trigger range lowered to 1.20 m to keep
   it separated from the release distance (now 2.5-3.0 m).

   AMENDMENT 3 (presentation, no effect on behaviour): the run now
   records a stationary PREROLL (4 s), continues straight for an OUTRUN
   leg (7 s) after the anchor plane is passed, and records the final
   hold, so the full trajectory — before, during and after the
   avoidance — is visible in one overhead video.

## 4. Frozen parameters (D-016)

| Parameter | Value |
|---|---|
| surge / sway | 0.35 / 0.45 of full authority |
| trigger range | 1.20 m (amendment 2) |
| trigger confirmation | 3 consecutive samples, median range over 5 (amendment 2) |
| preroll / outrun | 4 s stationary / 7 s straight departure (amendment 3) |
| release distance | 2.5-3.0 m from the anchor |
| lateral clearing max | 7 s; ends after 5 consecutive non-detections or bearing > 35° |
| straight leg | 7 s (or until the anchor plane is passed) |
| centring window | 6 s |
| wall guard | relative to the release point, hard limit 2% of frame |
| hold after run | 10 s closed-loop heading hold, then disarm |
| detector gate | score >= 0.55, height >= 0.12, width >= 0.02 |
| range model | f_px = 1277, anchor arm span 0.80 m (D-015) |
| ArduSub mode | STABILIZE |

## 5. Outcome definitions (declared BEFORE the runs)

* **SUCCESS**: a trigger occurred from a perception detection AND the
  lateral clearing executed AND the vehicle passed the anchor plane
  (overhead along-track projection) AND no collision AND no wall-guard
  abort.
* **COLLISION**: overhead ROV-to-anchor distance < 0.50 m at any time.
* **NO-TRIGGER**: the run timed out without any accepted detection —
  counted as an ALGORITHM/PERCEPTION FAILURE, not excluded.
* **TECHNICAL INVALID** (excluded, re-run once): arming failure, camera
  stream loss, overhead tracker losing the vehicle for > 5 s, or a
  wall-guard abort within the first 2 s (bad release position).

## 6. Metrics recorded per run

Trigger range and bearing; detection acceptance rate; minimum overhead
distance; time from release to trigger; time from trigger to pass; total
run duration; final state; heading drift during the approach; whether
the overhead lock matched the vehicle.

## 7. Design

n = 10 valid repetitions, identical configuration and procedure. Pilot
runs executed before the campaign are declared as pilots and are NOT
part of the results.

## 8. Analysis (pre-specified)

Success proportion with a 95% Wilson interval; median and IQR of trigger
range, minimum distance and detection rate; comparison of the observed
behaviour with the Phase-8 simulation (trigger-to-pass geometry,
committed-maneuver completion under intermittent perception). No metric
is added or removed after seeing the results.

## 9. Honesty commitments

The detector and the mission parameters are frozen before the campaign
and were tuned only on development runs (2026-08-14 afternoon), which
are archived and reported as development, not evidence. Failures are
reported, not re-run. Lighting conditions are recorded per session, and
runs from different sessions are never pooled without saying so.
