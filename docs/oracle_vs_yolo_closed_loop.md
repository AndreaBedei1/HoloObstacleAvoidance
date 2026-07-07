# Oracle vs YOLO Closed Loop

Status: YOLO closed-loop run succeeded. Oracle baseline is preserved; the
comparison below uses the clean oracle baseline already captured in this
session because the later oracle rerun was blocked by simulator/startup
instability.

## Commands

Oracle baseline command:

```bat
scripts\run_custom_anchor_closed_loop.bat --detector oracle --duration-s 45
```

YOLO command:

```bat
scripts\run_custom_anchor_closed_loop.bat --detector yolo --duration-s 45 --confidence-threshold 0.25 --inference-stride 1
```

## Evidence

Oracle baseline:

```text
logs/custom_anchor_validation.json
visualizations/custom_anchor_frame0001.png
visualizations/custom_anchor_frame0200.png
```

YOLO run:

```text
logs/custom_anchor_yolo_validation.json
logs/custom_anchor_yolo_ros2_launch.log
logs/custom_anchor_yolo_sim_server.log
visualizations/custom_anchor_yolo_frame0001.png
visualizations/custom_anchor_yolo_frame0200.png
```

## Metrics

| Metric | Oracle baseline | YOLO |
|---|---:|---:|
| camera frames | 1286 | 1314 |
| planner input detections | 61 | 61 |
| safe cmd messages | 1085 | 1081 |
| avoidance cmd messages | 90 | 92 |
| max risk | 0.9756 | 0.8418 |
| max lateral deviation m | 11.323 | 0.490 |
| recovered after avoidance | true | true |

State sequence:

```text
oracle: NORMAL -> AVOIDING_LEFT -> RECOVERING -> NORMAL
YOLO:   NORMAL -> APPROACH_OBSTACLE -> AVOIDING_LEFT -> RECOVERING -> NORMAL
```

## Interpretation

YOLO detected the object early enough in this run to drive the planner through
approach, avoidance, recovery, and back to normal. The oracle relay was disabled
for the YOLO run, so `/perception/obstacles` was produced by the visual detector.

Detection stability is adequate for this first pass but not continuous. The
validator saw 61 YOLO planner-input detections over 896 planner-input messages.
That was enough to trigger avoidance, but temporal smoothing or a short
detection hold would make the behavior less dependent on single-frame hits.

The YOLO run passed the validator, but its ROS launch log still recorded a
Windows access-violation exit from `nominal_cmd_publisher_node` after startup.
The planner continued publishing `/planner/cmd_vel_safe` and recovered, but
this should be rerun from a fresh Unreal process after the new `TimerAction`
staging change.

The oracle baseline remains useful as the simulation ground truth path. The
large lateral deviation in the baseline run is not directly comparable as a
strict controller-quality metric because the rerun intended for a same-session
pair failed before producing a clean oracle episode.

## Failed Rerun Notes

A later oracle comparison rerun had two instability modes:

- first attempt: `local_avoidance_planner_node` exited with Windows access
  violation code `3221225477` during startup;
- second attempt after launch staging: the visible Unreal/HoloOcean simulator
  stayed in engine loading and the sim server never opened TCP port `47654`
  within 600 seconds.

The launch files now stage node startup with `TimerAction` to reduce the ROS 2
Windows startup crash. The simulator loading issue is external runtime state,
not a code-path change in the repository.

## Next Step

Add a short detection-hold or tracker layer after YOLO so the planner receives a
stable obstacle for a few frames after each confident detection, then rerun the
same oracle/YOLO comparison from a fresh Unreal process to confirm the staged
launch removes the startup access-violation behavior.
