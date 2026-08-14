# Minimal adaptive actuator identification (Phase 10, S3)

Purpose: estimate, for each signed degree of freedom the planners
actually use, the smallest set of parameters the command mapping needs:

* the deadband (below which the vehicle does not move),
* the minimum reliably commandable value,
* the signed steady response (SI units per count above the deadband),
* the rise time constant and the decay/coasting behaviour,
* the directional asymmetry, where the data support it,
* repeatability.

This is NOT a 6 x 5 x 6 s grid. A fixed grid would spend most of its
runs on levels that carry no information, and in a pool this small a
30 % translation step for 6 s consumes more room than is available.

## Principle

Sequential, informative trials. Each trial is chosen from what the
previous trials showed, and the experiment stops when the parameters are
identifiable with adequate repeatability — not when a matrix is full.

## Procedure per signed DOF (surge+, surge-, sway+, sway-, yaw+, yaw-)

1. **Bracket the deadband.** Start at a level known not to move the
   vehicle (15 %) and step up (15 -> 22 -> 30 -> 38 %) with SHORT pulses
   until the ground truth shows motion above the noise floor. Stop the
   ladder at the first level that moves.
2. **Confirm the bracket.** Two repetitions just BELOW and two just
   ABOVE the transition. This is what actually estimates the deadband
   and its repeatability; further levels below it add nothing.
3. **Operating points.** Two levels above the deadband, chosen to span
   the range the planners will actually command (not the full authority
   range). Three repetitions each: three points estimate the slope AND
   its variability.
4. **Dynamics.** The rise constant comes from the same steps (fit the
   first-order response to the ground-truth speed); the decay comes from
   the post-pulse coast already recorded in every trial. No dedicated
   runs.

**8-12 trials per axis is an UPPER BOUND, not a target.** Every existing
valid pilot trial is reused; only the trials that are still needed for
identifiability are collected. `scripts/analysis/actuator_data_
sufficiency.py` computes that gap from the recorded data and prints the
shortest remaining experiment.

What it found (2026-08-14):

| Signed axis | Existing | Still needed |
|---|---|---|
| surge+ | one 20 % step, attitude only | overhead speed |
| surge- | one 15 % pulse | overhead speed + a second level |
| sway+ / sway- | short cross-coupling pulses only, no speed | the real gap: speed at two levels each |
| yaw+ | 15 % and 20 % steps with gyro rate | a second moving level + one repetition |
| yaw- | one 15 % pulse (2.5 deg, below the movement floor) | a moving level, a second one, one repetition |

CONTAMINATED DATA, excluded: the 18:28 yaw authority sweep was run while
the vehicle was grounded on the shallow bottom, and its response is
non-monotone (140 deg at 15 %, 2.5 deg at 30 %, 2.8 deg at 45 %). It
measures the grounding, not the vehicle.

Realistic remaining cost: about 3 short trials per signed axis, roughly
15-18 trials in total, all with overhead ground truth.

## Termination and safety: geometric, not temporal

A translation trial ends at the FIRST of:

| Limit | Value |
|---|---|
| displacement from the start | 0.6 m |
| distance to the nearest pool boundary | wall margin from the remap |
| ground-truth track valid | lost for > 1.0 s -> stop |
| duration | 6 s hard cap |
| depth deviation | > 0.25 m from the hold setpoint |
| roll or pitch | > 15 deg |
| vehicle telemetry stale | > 1.0 s |
| operator | emergency stop at any moment |

Yaw trials may run longer while the vehicle stays centred, under the
same guards.

Short pulses first; a pulse is lengthened only when the previous one
left a parameter unidentifiable (for example, a rise constant that the
data cannot separate from the dead time).

## Recording

EVERY attempted trial is recorded, including aborted ones and ones where
the vehicle did not move: a non-moving trial is the primary evidence for
the deadband, and discarding aborts would bias the estimate of what the
vehicle can actually do.

Per trial: commanded axis and level, pulse duration, the reason the
trial ended, the full telemetry stream, the overhead ground-truth track,
and the derived steady speed, rise constant and coast.

## What is fitted

Per signed DOF, the affine deadband model used by
`rov_real_bridge/command_mapping.py`:

    counts = sign(v) * (db + |v| / k)      for |v| >= min_command
    counts = 0                             otherwise

Symmetry is NOT imposed: the pilot data already show a yaw asymmetry of
about a factor two. Nor are parameters added that repeated measurements
do not support — an axis whose two directions agree within their spread
is reported as symmetric, with the spread.

An axis that cannot be identified is left `calibrated: false`, and the
adapter refuses live actuation while any axis is uncalibrated.
