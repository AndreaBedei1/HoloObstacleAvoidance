# The paper's story, and what the evidence will and will not carry

Written after the final audit of 2026-08-16, with the real dataset closed.
Everything below is either verified against a file in this repository or
marked as not yet established. This document is the backbone the
manuscript is written from; if the two ever disagree, this one is wrong
and should be fixed first.

## 1. What actually happened, in order

1. A camera-driven obstacle-avoidance stack was developed against
   simulated perception in HoloOcean.
2. Its observation, timing and vehicle-response mismatches were MEASURED
   against the real BlueROV2 and the real pool, producing three
   calibration levels on top of the historical simulator (S1, S2, S3).
3. Eighty simulated runs were executed and hashed BEFORE any real run:
   4 levels x 2 planners x 2 geometries x 5 repetitions.
4. The stack was taken to the pool. It did not work. A morning was spent
   changing it until it did.
5. Nine real runs were flown, all with the committed planner. Eight are
   admissible.
6. The pool was drained. No further physical experiment is possible.
7. Post-deployment diagnostic simulations were run to ask which of the
   changes mattered.

The paper's spine is that chronology, and its main methodological claim is
that steps 3 and 7 must never be reported as the same kind of evidence.

## 2. The one-sentence story

A simulation study calibrated against measurements of the real system
still failed to transfer to it, and the reasons are specific, findable
and mostly not the ones the calibration ladder was built to capture.

## 3. What the evidence supports

### 3.1 Supported, strongly

**The deployment gap is real and enumerable.** Seven documented
differences separate the stack that produced the 80 predictions from the
stack that flew. Each has a file and a line on both sides
(`config/real_session_20260816.yaml`, verified against
`src/rov_real_bridge/launch/real_pipeline.launch.py` and
`src/rov_obstacle_avoidance/config/local_avoidance_planner.yaml`).

**The real experiment had no approach phase, and this was structural
rather than accidental.** The onboard detector's blob-size gates accept a
box between 0.15 and 0.90 of frame height
(`scripts/real/anchor_detect.py:203-205`), which through the shared
monocular estimator at the 90 degree vertical FOV the planner actually
used is an observable range window of **0.29 m to 2.11 m**. The vehicle
started 1.86 m from the anchor and engaged at 1.80 m. Six centimetres.
The simulated vehicle approaches from 3.5 m and engages at 1.5 m.

**The manoeuvre the planner asks for does not fit the pool.** The frozen
planner strafes to a 2.5 m lateral offset and runs 4 m past the obstacle
(`local_avoidance_planner.yaml`). The pool is 2.7-3.0 m wide with the
anchor 1.86 m ahead. Nobody changed these on the day; run 7 ended against
the wall.

**Side choice follows the obstacle rather than a fixed bias.** Real: four
of four right in K0, one each way in K1, two of two left in K1M, by the
pre-registered commitment definition. K1M was invented mid-session
precisely because K0 and K1 cannot separate those two explanations.

**The real campaign can validate the command domain and nothing else.**
No overhead recording was made during any run, so trajectory, path
length, lateral deviation and true clearance do not exist for any real
run and cannot be recovered. This is a limitation and also a finding
about what a camera-only deployment can prove.

**The perception stream is recoverable.** The detector drew its own
accepted and rejected boxes on every frame it processed, so the
observation stream that reached the planner can be read back exactly, for
the six runs that have video, including the range the planner believed.

### 3.2 Supported, with the sample size stated every time

**Real behaviour is consistent across eight runs.** Commanded surge
medians are 0.123 / 0.114 / 0.115 m/s in K0 / K1 / K1M -- propulsion does
not depend on geometry. Detection fractions 71-97 %. Never stalled.

Two runs per off-axis geometry. Every statement about K1 or K1M carries
n=2 in the same sentence.

### 3.3 NOT supported, and must not be claimed

**No causal S0 -> S3 validation of simulator predictivity.** The frozen
stack and the flown stack differ in seven documented ways, so a
level-to-level difference in reality cannot be attributed to a
calibration rung.

**No committed-versus-DWA ranking in reality.** DWA never issued a
command on the vehicle: it needs a pose estimate the BlueROV2 has no
sensor for. The C-vs-D comparison is simulated, and the paper says so
wherever it appears.

**No spatial sim-real agreement.** Reality has no spatial ground truth.

## 4. What the audit found in the frozen campaign

These are defects in our own pre-registered experiment. They are reported
because a frozen result that is quietly known to be flawed is worse than
one that is openly flawed.

1. **The relay and the planner disagree about the camera.** The
   calibrated observation relay converts apparent height to range at a
   60 degree vertical FOV (`calibrated_observation_relay_node.py:108`)
   while the oracle projects and the planner inverts at 90 degrees
   (`planner.py:106`; no launch file in either domain overrides it). The
   relay therefore believes the obstacle is about 1.5x further than the
   planner does, and applies the measured detection-probability curve to
   the wrong range band, in the 60 runs at S1 and above.

2. **The runs are not reproducible.** `random.Random(seed if seed else
   None)` with the campaign's `seed=0` seeds from system entropy, because
   0 is falsy.

3. **The command loop ran at half rate.** The frozen runs record ~10 Hz on
   the nominal command topic and ~20 Hz on the safe topic; a run of the
   same configuration on an idle machine today records ~20 and ~40 Hz.
   The perception gate is simulated-time based and unaffected, so the
   frozen campaign closed its loop half as often as intended.

4. **`camera_vfov_deg: 60.0` in the frozen pool benchmark is read by no
   node in either domain.** Both used the planner's 90 degree default.
   The two domains agree, so no sim-real mismatch follows -- but the
   benchmark documents a constant it does not enforce.

5. Four DWA runs lose their commitment distance to an instrumentation
   race; one run is the campaign's only collision; two S2 runs never
   delivered a planner-valid detection.

Consequence for the design of everything after: the diagnostic ablation
is **self-contained**. Its own frozen-configuration rung is its baseline,
run in the same session on the same machine, and no rung is ever compared
against the 80 frozen numbers.

## 5. Research questions

- **RQ1 (simulation sensitivity).** How do progressively measured
  observation, timing and vehicle-response mismatches change closed-loop
  camera-driven avoidance in simulation?
- **RQ2 (deployment gap).** Which mismatches prevented the
  simulation-developed stack from being deployed unchanged on the
  physical BlueROV2, and what had to change?
- **RQ3 (physical behaviour).** What closed-loop behaviour does the
  deployed controller show across the real geometries available?
- **RQ4 (diagnostic agreement).** With the deployed configuration
  reproduced in simulation, which observable behaviours agree with the
  physical runs and which remain apart?

RQ4 is deliberately about agreement, not validation: the diagnostic
simulations were run knowing the real outcome.

## 6. Contributions, as strong as the evidence allows

1. A measurement-grounded calibration ladder for a camera-driven
   underwater avoidance loop, with every rung fitted to paired
   observations of the real vehicle rather than assumed.
2. A pre-registered, hashed simulated campaign executed before any real
   run, and an honest audit of its own defects.
3. A documented deployment gap: seven specific changes, each with what
   forced it, and the finding that the dominant ones were not the
   calibrated quantities but the geometry of the basin and the range
   window of the detector.
4. A reproducible offline recovery of the real perception stream from
   annotated video, and an explicit account of which real quantities can
   and cannot be measured without external ground truth.
5. A post-deployment ablation that separates the effect of each change,
   which the physical campaign could not do.

No "first" claim anywhere.

## 7. Title

Working: *Camera-Driven Obstacle Avoidance from HoloOcean to a BlueROV2:
Measurement-Grounded Simulation, a Documented Deployment Gap, and What a
Camera-Only Field Campaign Can Prove*

Shorter alternative: *When Calibration Is Not Enough: A Documented
Deployment Gap for Camera-Driven Underwater Obstacle Avoidance*

## 8. Venue

ROBOVIS 2027, 27-28 February 2027. Regular paper deadline 15 September
2026. 25 pages, 10,000-50,000 characters. Double-blind, no preprints
during review. Springer LNCS template (the Templates page links the LNCS
conference proceedings guidelines). AI-generated text must be disclosed
in the acknowledgements and cited.
