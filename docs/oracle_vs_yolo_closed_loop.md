# Oracle vs YOLO Closed Loop

Status: YOLO closed-loop run succeeds **and now returns to the original path**.
The planner is pose-aware: it avoids laterally with limited yaw drift and then
steers back onto the original straight line and heading instead of keeping the
detour route. Oracle baseline numbers below are the clean baseline captured in
an earlier session (kept for reference).

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

| Metric | Before fix | After fix |
|---|---:|---:|
| initial yaw (rad) | 0.000 | 0.000 |
| final yaw (rad) | 1.071 (~61 deg) | 0.000 |
| final lateral error (m) | ~18.588 | 0.000 |
| final yaw error (deg) | ~61 | 0.00 |
| max lateral deviation (m) | 18.588 | 2.847 |
| max forward progress (m) | 10.755 | 13.084 |
| returned to original line | false | **true** |

Before the fix the vehicle avoided left and kept the new leftward route (61 deg
of permanent heading drift, ~18.6 m off the line). After the fix it strafes up
to ~2.85 m to clear the anchor and comes back to the original line: the final
pose matches the start line to within 0.0 m and 0.0 deg.

## State sequence (after fix)

```text
NORMAL
  -> (APPROACH_OBSTACLE -> AVOIDING_LEFT -> RECOVERING) x8
  -> NORMAL
```

The repeated avoid/recover cycles are the expected "crab past" behavior: the
anchor sits on the original line, so each time recovery steers back toward the
line while the anchor is still ahead it is re-detected and the vehicle strafes
again, advancing each cycle until it is past the anchor. Once past, recovery
completes and `NORMAL` line-keeping holds the original path.

## Interpretation

The oracle relay was disabled for the YOLO run, so `/perception/obstacles` was
produced only by the visual detector (1531 planner-input detections). YOLO drove
the planner through approach, avoidance, and recovery, and the pose-aware
recovery/line-keeping returned the vehicle to the original straight route and
heading.

`/planner/cmd_vel_safe` differed from `/cmd_vel_nominal` in 1382 of 1723 safe
messages, confirming the planner actively overrode the straight command during
the maneuver while leaving it untouched once back on the line.

## Reproducing

```bat
scripts\run_custom_anchor_closed_loop.bat --detector yolo ^
  --nominal-publisher-enabled false --manual-nominal-command --duration-s 90
```

Then inspect `logs/custom_anchor_yolo_validation.json` for
`final_lateral_error_m`, `final_yaw_error_deg`, and `returned_to_original_line`.
```
