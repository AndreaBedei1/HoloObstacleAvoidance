# Planner Input Equivalence (Phase 8 fairness audit)

Verified against the actual subscriptions and parameter usage of both nodes
(`local_avoidance_planner_node.py`, `dwa_planner_node.py`).

| Input | Committed planner (C) | DWA (D) | Same information? |
|---|---|---|---|
| Obstacle stream | `/perception/obstacles` (T2 + Phase-7B qualification) | identical topic | ✔ identical |
| Bearing | `bearing_rad` field ((cx−0.5)·HFOV, computed by the estimator node) | recomputed from the same `center_x` with the same HFOV | ✔ same formula |
| Range estimate | monocular: H_ref 3.5 m / (2·h_bbox·tan(VFOV/2)), VFOV 90° | identical formula, identical constants | ✔ same model |
| Obstacle extent | implicit via clearance_offset (2.5 m from estimated center) | explicit circle radius 1.75 m (= H_ref/2) + footprint + margin | ✔ same class-dimension assumption, representation differs by design |
| Vehicle pose | `/rov/odom_estimated` (no-DVL commanded dead reckoning) | identical topic | ✔ identical |
| Measured yaw | via odom quaternion | identical | ✔ |
| Depth | not used horizontally (depth held by control layer) | not used | ✔ equal non-use |
| Body-velocity state | none (stateless w.r.t. velocity; commands from state machine) | internal commanded-response estimate (same first-order τ as odometry; from the planner's OWN outputs) | ✔ derived only from its own commands — no extra sensor |
| Nominal route | captured from first nonzero `/cmd_vel_nominal` + pose reference | identical convention | ✔ |
| Nominal command | `/cmd_vel_nominal` (staleness 1 s → stop) | identical topic + identical staleness rule | ✔ |
| Speed limits | max_surge 0.5, sway caps, yaw 0.3 (config) | same limits in DWAConfig | ✔ aligned |
| Accel limits | implicit (controller rate limiter downstream) | explicit planning bound 0.5 m/s² ≤ controller limit | ✔ DWA is not given extra authority |
| Ground truth / oracle / `/sim/obstacles_world` | never subscribed | never subscribed + forbidden-topic guard | ✔ both excluded |
| Simulator VelocitySensor / DVL | not used | not used | ✔ |
| Output | Twist on `/planner/cmd_vel_safe` | identical | ✔ same downstream stack |

**Conclusion:** no planner receives runtime information unavailable to the
other; the only differences are internal representations (state machine vs
sampled rollouts), which is precisely the variable under study.

Perception stack fixed for the primary comparison: **T2 fixed Kalman +
Phase-7B qualification for BOTH planners** (T2 was 25/25 in Phase 7B;
using one stack isolates the planner variable). T0/T1 remain available for
secondary ablations only. T3 excluded (uncalibrated ≡ T2).
