# Oracle vs YOLO Closed Loop

Status: YOLO closed-loop run succeeds with a **committed circumnavigation**: the
vehicle estimates the range to the anchor, commits to one side, runs parallel
past it (odometry-gated), and returns to the original straight line **once** — no
oscillation, no collision. The planner navigates on estimated odometry, never
the simulator ground truth. Oracle baseline numbers below are kept for reference.

## Commands

Oracle baseline command:

```bat
scripts\run_custom_anchor_closed_loop.bat --detector oracle --duration-s 45
```

YOLO manual ROS-commanded command (oracle relay disabled, YOLO active, built-in
nominal publisher disabled, manual straight `/cmd_vel_nominal`):

```bat
scripts\run_custom_anchor_closed_loop.bat --detector yolo ^
  --nominal-publisher-enabled false --manual-nominal-command --duration-s 90
```

The manual command is `geometry_msgs/msg/Twist {linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {z: 0.0}}` at 10 Hz.

## Evidence

```text
logs/custom_anchor_yolo_validation.json
logs/custom_anchor_yolo_ros2_launch.log
logs/custom_anchor_yolo_sim_server.log
logs/custom_anchor_yolo_manual_nominal_pub.log
visualizations/custom_anchor_yolo_frame0001.png
visualizations/custom_anchor_yolo_frame0200.png
```

## Return-to-path metrics (YOLO, manual straight command)

The planner now navigates on **estimated odometry** (`/rov/odom_estimated`,
dead-reckoned DVL+gyro with realistic drift), never the simulator ground truth.
Ground truth (`/rov/pose_ground_truth`) is used only by the validator.

| Metric | Broken (body-frame) | Reactive line-keep (oscillated) | Committed circumnavigation (current) |
|---|---:|---:|---:|
| final lateral error, GT (m) | ~18.588 | 0.246 | **0.257** |
| final yaw error, GT (deg) | ~61 | -0.69 | **-0.72** |
| max lateral deviation (m) | 18.588 | 2.817 | 2.433 |
| max forward progress (m) | 10.755 | 12.923 | 21.283 |
| avoid/recover cycles | 1 (then stuck) | 8 (oscillation) | **1** |
| yaw used during maneuver | large (runaway) | small | **~0 (0.001 rad)** |
| odometry drift vs GT (m) | n/a (used GT) | 0.556 | **0.659** |
| returned to original line | false | true | **true** |

The current column is a single committed go-around: the planner estimates the
range monocularly, commits to one side, strafes to a ~2.4 m offset, runs
parallel PAST the anchor (advancing 21.3 m — well past the 12 m anchor), then
returns once. It navigates on a DVL+gyro odometry estimate that drifts 0.659 m
from ground truth, yet still returns to within **0.257 m / 0.72 deg** of the true
line. The vehicle held the ~2.4 m offset while passing the anchor (which is on
the line), so it stayed clear — no collision.

## State sequence (current)

```text
NORMAL -> APPROACH_OBSTACLE -> AVOIDING_LEFT -> RECOVERING -> NORMAL
```

A single committed maneuver. The pass from AVOIDING to RECOVERING is decided by
odometry forward-progress past the estimated obstacle position (plus a margin),
not by loss of detection — so a detector dropout mid-maneuver cannot make the
vehicle turn back into the anchor.

## Interpretation

The oracle relay was disabled for the YOLO run, so `/perception/obstacles` was
produced only by the visual detector (~814 planner-input detections). YOLO drove
the planner through a single approach → avoid → recover cycle, and the committed
circumnavigation carried the vehicle around and past the anchor before returning
to the original straight route and heading.

The planner's pose input was the estimated odometry (`/rov/odom_estimated`),
never the simulator ground truth. The estimate drifted 0.659 m from ground
truth over the run, which is realistic DVL+gyro dead-reckoning error; the return
still succeeded because the cross-track component of that drift stayed small and
the along-track component does not affect the lateral error.

`/planner/cmd_vel_safe` differed from `/cmd_vel_nominal` in 586 of 1726 safe
messages, confirming the planner actively overrode the straight command during
the maneuver while leaving it untouched once back on the line.

## Reproducing

```bat
scripts\run_custom_anchor_closed_loop.bat --detector yolo ^
  --nominal-publisher-enabled false --manual-nominal-command --duration-s 90
```

Then inspect `logs/custom_anchor_yolo_validation.json` for
`final_lateral_error_m`, `final_yaw_error_deg`, `returned_to_original_line`, and
the odometry-drift fields (`odom_final_position_error_m`).
