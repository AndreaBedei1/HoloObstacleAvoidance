# The Experimental Boundary: what is shared between simulation and reality

This document defines, formally and once, what the sim-to-real study of
this project does and does not claim. Every other document, every commit
message and eventually the paper must use these terms.

## The four domains

```
                    SIMULATION                         REALITY
                    ----------                         -------
PIXEL DOMAIN        HoloOcean rendered frames          BlueROV2 camera frames
(platform-          (not used in the Phase-8           1920x1080 H.264 over
 specific)           campaign at all)                  UDP 5600

                            |                                  |
                    observation model                  visual detector
                    (geometric projection of           (classical shape-prior
                     the simulated obstacle,            detector, no trained
                     progressively calibrated           weights exist for this
                     S1/S2 from real data)              setup)
                            |                                  |
                            v                                  v
OBSERVATION         ============ /perception/obstacles_raw ============
DOMAIN                     rov_obstacle_msgs/Obstacle2DArray
(the shared                 image-space bbox + class + confidence
 interface)

                            |                                  |
SHARED              ============ IDENTICAL SOURCE CODE ================
NAVIGATION            Phase-7B qualification  (perception_qualification.py)
STACK                 T2 temporal estimator   (temporal_core.py)
                      Planner C or Planner D  (planner.py / dwa_planner.py)
                            |                                  |
                    ========== /planner/cmd_vel_safe ==================
                          geometry_msgs/Twist, SI units, ROS frame

                            |                                  |
ACTUATION           simulation vehicle adapter         real vehicle adapter
DOMAIN              (holoocean_bridge_node +           (real_control_live_node
(platform-           sim server controller,             -> MANUAL_CONTROL ->
 specific)           thruster allocation)               ArduSub ALT_HOLD)
                            |                                  |
                    HoloOcean BlueROV2                 physical BlueROV2 Heavy
```

## What we claim, in exact words

> The physical and simulated systems share the same observation
> conditioning, temporal estimation and planning implementation from
> `/perception/obstacles_raw` to `/planner/cmd_vel_safe`. The real system
> obtains raw observations from an onboard visual detector, whereas the
> simulator uses progressively calibrated observation models derived from
> an independent real calibration dataset.

## What we do NOT claim

* NOT "the same end-to-end perception and planning pipeline".
* NOT "the same detector in simulation and reality".
* NOT identical pixel-level perception.
* NOT that the same binary runs in Unreal and on ArduSub.

The reason is factual, not rhetorical: in the Phase-8 campaign
`/perception/obstacles_raw` is produced by `oracle_dropout_relay_node`,
which republishes a geometric projection of the simulator's ground-truth
obstacle. **There is no detector in the simulation loop.** Claiming a
shared detector would be false. Rendering an anchor in HoloOcean and
running the real detector on simulated frames is a legitimate FUTURE
ablation, not something to retrofit in order to justify a phrase.

## Why this boundary is the right one

1. **It is where the code is genuinely identical.** The qualifier, the
   temporal estimator and both planners import nothing platform-specific;
   every scale-dependent value is a ROS parameter.
2. **The planner cannot tell where an observation came from.** It
   consumes `Obstacle2DArray` and has no channel through which the
   provenance could leak. This is what makes the comparison meaningful.
3. **It makes the observation process an explicit object of study**
   rather than an unexamined assumption. The difference between the
   simulated and the real observation stream is exactly what the S1/S2
   calibration levels model, and exactly what the paper measures.

## Consequences for the experimental design

* The **detector is characterized, not transferred.** Its error model
  (residual distribution vs observables, availability, dropout structure,
  false positives) is estimated from the PILOT/CALIBRATION dataset and
  injected into the simulator as the S1/S2 observation model.
* **The calibration dataset and the evaluation dataset are disjoint.**
  The wet runs of 2026-08-14 are pilot/bring-up/calibration only. The
  final matched campaign is a separate, frozen, prospective experiment.
* **Simulation predictions are produced and stored BEFORE the final real
  runs**, so the real campaign is a prospective validation of the
  simulator rather than a post-hoc fit.
* **Only the actuation domain may differ by implementation.** The
  simulator's controller and the real adapter are different code, and
  they must be, because they drive different actuators. What they share
  is the input contract: a Twist in SI units in the ROS body frame.

## The Twist contract at the boundary

`/planner/cmd_vel_safe`, `geometry_msgs/Twist`, ROS body frame (REP-103):

| Field | Meaning | Units | Sign |
|---|---|---|---|
| `linear.x` | surge | m/s | + forward |
| `linear.y` | sway | m/s | + to PORT (left) |
| `linear.z` | heave | m/s | + up (ignored on the real vehicle: ALT_HOLD owns depth; the omission is logged) |
| `angular.z` | yaw rate | rad/s | + counter-clockwise seen from above |
| `angular.x/y` | roll/pitch rate | rad/s | not commandable on either platform |

Any consumer of this topic must reject non-finite components before
clamping (see `command_mapping.py`): the naive clamp
`max(-limit, min(limit, v))` maps NaN to `+limit` (full forward on the
vehicle) while the simulator's `max(lo, min(v, hi))` maps it to `lo`
(full reverse) — a silent, opposite-sign divergence between the domains.

## Where each artifact lives

| Domain | Simulation | Reality |
|---|---|---|
| Pixel | HoloOcean camera (unused in Phase 8) | `scripts/real/camera_stream.py`, `anchor_detect.py` |
| Observation | `oracle_dropout_relay_node` + S1/S2 observation model | real detector -> `/perception/obstacles_raw` |
| Shared stack | `rov_obstacle_tracking`, `rov_obstacle_avoidance` | the same packages, same parameters |
| Actuation | `holoocean_bridge_node`, sim server | `rov_real_bridge` adapter -> MANUAL_CONTROL |
