# S3 vehicle profile — what was identified, and what was not

Source: `experiments/real/actuator_id/*/trials.json`, fitted by
`scripts/analysis/fit_actuator_model.py` into
`config/calibration/s3_vehicle.json`. Session of 2026-08-15, BlueROV2
Heavy in the ~6 m pool, overhead RealSense as ground truth.

This document exists so the paper can state the dynamics calibration
level honestly. S3 is a PARTIAL calibration and must be described as
one.

## 1. Result — all six signed axes carry a slope

| Signed axis | Slope | Deadband | Valid trials | Basis |
|---|---|---|---|---|
| yaw+ (CW) | 0.0654 deg/s per count | 383 counts, measured | 2 levels | IMU |
| yaw- (CCW) | 0.0537 deg/s per count | 220 counts, measured | 2 levels | IMU |
| surge+ | 0.00015 m/s per count | 0, NOT resolved | 5 | overhead |
| surge- | 0.00046 m/s per count | 463 counts, measured | 2 | overhead |
| sway+ | 0.00009 m/s per count | 0, ASSUMED | 1 | overhead |
| sway- | 0.00014 m/s per count | 0, NOT resolved | 3 | overhead |

The yaw asymmetry is substantial and real: the counter-clockwise
direction starts moving at roughly half the command the clockwise
direction needs, and its slope is 18 % lower. Imposing symmetry would
misrepresent the vehicle. This is the asymmetry the per-sign deadband
model in `rov_real_bridge/command_mapping.py` exists to carry.

Yaw is the best-identified axis because it is measured by the IMU, which
the water disturbance does not affect.

## 2. Confidence is NOT uniform, and the paper must say so

* **Strong**: yaw both signs, surge+ (five trials across two levels,
  mutually consistent: 0.087-0.108 m/s at 600 counts and 0.116-0.172 at
  1000).
* **Weak — one trial**: `sway+`. Its slope comes from a single level
  with the deadband assumed zero.
* **Weak — implausible asymmetry**: `surge-` fits to 0.00046 m/s per
  count, which would make reverse about 65 % FASTER than forward at full
  command. A symmetric vectored frame should not do that. The fit rests
  on two points and is dominated by one 1.2 m trial, so the asymmetry is
  much more likely residual drift contamination than a property of the
  vehicle. It is reported, not trusted, and no conclusion in the paper
  depends on it.

Three of the four translation axes could not resolve a deadband: their
free fits drove the intercept to zero or below, because both command
levels sit well above any threshold. Where that happened the slope is
refitted through the origin, which imposes the physical constraint
`deadband >= 0` instead of keeping a slope fitted alongside an
impossible intercept.

## 3. What it cost to get there, and what still limits it

Sway was unmeasurable for most of the session. Three physical facts
combine, and they still bound the accuracy of every translation number
above:

* **The disturbance is comparable to the signal.** With thrusters idle
  the vehicle drifts at 0.037-0.053 m/s. A full sway command produces
  roughly 0.10 m/s. The disturbance is thruster-induced recirculation in
  a small pool and it GREW over the session as the water was stirred.
* **The disturbance is not stationary.** Successive measurements gave
  (-9, +14), (+15, -6), (+24, -0.4) and (-22, +1) px/s — different
  directions minutes apart. A single global drift estimate is therefore
  useless and subtracting one makes trials worse, so the drift is
  re-measured immediately before every pulse.
* **The working volume is small.** The overhead camera covers
  4.24 x 2.38 m. A 4 s pulse travels 0.4 m, and the vehicle leaves the
  frame after two or three trials no matter how carefully it is
  recentred.

Trials with drift above 60 % of the signal are excluded. The evidence
that the rule is right is in the raw data: in the contaminated sessions
sway came out SLOWER at 1000 counts than at 600, which no thruster does,
and which is the signature of a measurement dominated by the
disturbance. In the final session the same axis gave 0.106 and 0.158 m/s
at 600 and 1000 counts with the disturbance down to 9-38 % of the
signal.

What eventually made sway measurable was procedural, not physical:
measuring the drift immediately BEFORE each pulse instead of once per
session; subtracting it from the axis-direction probes as well (without
that, surge and sway came out 30 degrees apart instead of orthogonal,
because both probes were mostly measuring the drift); lengthening the
pulses to 4 s; and having the operator keep the vehicle near the centre
of the frame during the run.

## 4. Excluded trials, and why they are kept

Every attempted trial is stored. Exclusions are recorded per trial:

| Reason | Meaning |
|---|---|
| `wall contact (operator observed)` | 20260815_105604 surge +1000: the operator saw the vehicle touch the pool wall, so the displacement is not free motion |
| `pulse too short` | cut off at the frame edge; dividing a full displacement by a fraction of a second once reported 5.4 m/s |
| `drift comparable to signal` | disturbance above 60 % of the measured motion |

A trial that did not move the vehicle is evidence about the deadband, and
a trial that was aborted is evidence about what the vehicle can do in
this pool. Deleting either would bias the profile toward success.

## 5. A non-physical deadband is reported as such

The `surge+` fit extrapolates to a deadband of -178 counts. A negative
deadband would mean motion at zero command. With two command levels and
this scatter the intercept is simply not resolved, so it is clamped to
zero and flagged `deadband_resolved: false` rather than handed to the
simulator, where it would make the simulated vehicle creep with idle
thrusters.

## 6. Command authority is part of this calibration

ArduSub scales `MANUAL_CONTROL` by a runtime joystick gain. During this
session that gain was **0.20**: a full command reached only 20 % of
thruster authority, which is why the vehicle appeared not to respond
while the mixer patterns were perfectly correct. The operator judged
that motion appropriate for a 6 m pool, so it was kept and folded into
the fit — the numbers above map COMMAND COUNTS to speed at that gain.

**A reboot resets the gain to `JS_GAIN_DEFAULT` (0.5) and silently
invalidates every number in this document.** `actuator_id_run.py`
measures the authority at the start of every session and records it in
the output; the same check must pass before any validation run.

## 7. Consequence for the campaign

S3 uses the measured slope on all six signed axes and the measured
command authority. Two deadbands are measured (yaw both signs, surge-);
the rest are set to zero and flagged unresolved.

The sim-to-real analysis must weight the axes by the confidence in
section 2: any S2 -> S3 change attributed to `sway+` rests on one trial,
and any conclusion that depends on the surge forward/reverse asymmetry
is not supported.

The pool disturbance itself is an uncontrolled input present in the real
runs and absent from the simulation. It is reported as a limitation, not
modelled: fitting a disturbance model to four inconsistent measurements
would be invention.
