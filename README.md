# HoloObstacleAvoidance

ROS 2 simulation stack for camera-based underwater obstacle avoidance with a custom HoloOcean/Unreal scene.

The current demo uses a front RGB camera, a YOLO detector trained on simulated underwater objects, estimated odometry, and a local planner that performs a committed go-around maneuver around an anchor. The planner publishes bounded abstract velocity commands only. It does not command MAVLink, thrusters, or a real vehicle.

## What the system does

The supported closed-loop pipeline is:

```text
custom Unreal/HoloOcean world
  -> /camera/front/image_raw
  -> YOLO detector
  -> /perception/obstacles
  -> local avoidance planner
  -> /planner/cmd_vel_safe
  -> HoloOcean sim bridge
```

For path recovery, the planner does not use simulator ground-truth pose. Runtime navigation uses estimated odometry:

```text
/rov/velocity  (DVL/gyro-like simulated measurement)
  -> odometry_estimator_node
  -> /rov/odom_estimated
  -> local avoidance planner
```

Ground truth is separated:

```text
/rov/pose_ground_truth       simulation-only, validation/debug only
/perception/obstacles_oracle simulation-only, labels/debug/baseline only
```

In the final YOLO run, the oracle relay is disabled. The obstacle input comes from YOLO; ground truth is used only by the validator to compute metrics.

## Main result

The current final demo command sends a straight manual ROS command:

```text
/cmd_vel_nominal = {linear.x: 0.4, linear.y: 0.0, angular.z: 0.0}
```

The vehicle goes straight, detects the anchor with YOLO, commits to a lateral go-around, passes the anchor using estimated odometry, then returns to the original line and heading.

Latest tracked validation result:

```text
state sequence: NORMAL -> APPROACH_OBSTACLE -> AVOIDING_LEFT -> RECOVERING -> NORMAL
final_lateral_error_m: 0.257
final_yaw_error_deg: -0.72
returned_to_original_line: true
max_lateral_deviation_m: 2.433
max_forward_progress_m: 21.283
planner_uses_estimated_odometry: true
oracle_relay_enabled: false
detector: yolo
```

Detailed evidence is in `docs/oracle_vs_yolo_closed_loop.md` and `logs/custom_anchor_yolo_validation.json`.

## Repository layout

```text
src/rov_obstacle_msgs/        ROS 2 obstacle/debug messages
src/rov_obstacle_perception/  YOLO detector node and config
src/rov_obstacle_avoidance/   local planner and nominal-command tools
src/rov_obstacle_sim_bridge/  HoloOcean TCP bridge, sim runner, odometry estimator
scripts/                      run, validation and dataset utilities
config/                       external custom engine configuration
training/yolo_custom_objects/ YOLO training config and notes
docs/                         experiment reports and technical notes
logs/                         tracked evidence logs for key runs
visualizations/               tracked example frames from key runs
```

## Requirements

This project uses two Python environments because HoloOcean and ROS 2 cannot run in the same interpreter here.

### ROS 2 side

Tested on Windows with ROS 2 Lyrical/Python 3.12. The helper script assumes the local ROS 2 installation is available under `C:\dev\lyrical`.

Typical commands:

```bat
call scripts\source_ros2_windows.bat
colcon build --merge-install
call install\setup.bat
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

The YOLO node uses Ultralytics when available in the ROS environment. Install it in that environment if needed:

```bat
pip install ultralytics
```

### HoloOcean / Unreal side

The simulator side runs in a separate conda environment named `ocean` with Python 3.9 and HoloOcean 2.3.0.

The custom scene uses an external modified Unreal/HoloOcean project containing:

```text
/Game/ancora.ancora   anchor mesh
/Game/mina.mina       mine mesh
/Game/siluro.siluro   torpedo mesh
/Game/ExampleLevel    custom underwater world
```

The external engine folder is not modified by this repository. Configure local paths in:

```text
config/custom_holoocean_engine.yaml
```

This file points to Unreal Editor 5.3, the external `Holodeck.uproject`, the default custom map, and launch/attach settings.

## Build and test

From the repository root:

```bat
call scripts\source_ros2_windows.bat
colcon build --merge-install
call install\setup.bat
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected current status: all package tests pass. The latest reported run had zero failures.

## Run the final YOLO closed-loop demo

Run from the repository root:

```bat
scripts\run_custom_anchor_closed_loop.bat --detector yolo ^
  --nominal-publisher-enabled false --manual-nominal-command --duration-s 90
```

This starts:

```text
visible Unreal/HoloOcean custom world
HoloOcean sim server in conda ocean
ROS 2 bridge
odometry estimator
YOLO detector
local avoidance planner
validator
manual straight /cmd_vel_nominal publisher
```

The command uses the trained YOLO weights at:

```text
training/yolo_custom_objects/runs/yolov8n_custom_underwater/weights/best.pt
```

Model weights are treated as generated artifacts and may not be committed. If the file is missing, recreate it using the notes in `training/yolo_custom_objects/README.md`.

## Run the oracle baseline

The oracle baseline is simulation-only and is useful for comparison or debugging:

```bat
scripts\run_custom_anchor_closed_loop.bat --detector oracle --duration-s 45
```

Do not treat the oracle as a real onboard sensor. It projects known simulated object bounds and must stay out of the real runtime pipeline.

## View the run live

The Unreal window opens visibly during the run. To view ROS topics, open another ROS 2 terminal:

```bat
call scripts\source_ros2_windows.bat
call install\setup.bat
```

Camera stream:

```bat
ros2 run rqt_image_view rqt_image_view /camera/front/image_raw
```

YOLO detections:

```bat
ros2 topic echo /perception/obstacles
```

Planner state:

```bat
ros2 topic echo /avoidance/debug
```

Safe command sent to the simulator:

```bat
ros2 topic echo /planner/cmd_vel_safe
```

Estimated odometry:

```bat
ros2 topic echo /rov/odom_estimated
```

## Important ROS topics

| Topic | Meaning |
|---|---|
| `/camera/front/image_raw` | RGB camera stream from HoloOcean |
| `/perception/obstacles` | Runtime obstacle detections, produced by YOLO in the final demo |
| `/perception/obstacles_oracle` | Simulation-only ground-truth projection for labels/debug/baseline |
| `/cmd_vel_nominal` | Desired command, e.g. straight motion from manual publisher |
| `/planner/cmd_vel_safe` | Planner output after obstacle avoidance correction |
| `/rov/velocity` | DVL/gyro-like simulated measurement used by the odometry estimator |
| `/rov/odom_estimated` | Runtime pose estimate consumed by the planner |
| `/rov/pose_ground_truth` | Simulation-only pose used by validator/debug only |
| `/avoidance/debug` | Planner state, selected side, risk and command diagnostics |

## Planner behavior

The current planner performs a committed circumnavigation:

1. `NORMAL`: follow the original straight line using estimated odometry.
2. `APPROACH_OBSTACLE`: YOLO detects an obstacle ahead; the planner estimates range monocularly from bounding-box height, known target height and camera vertical FOV.
3. `AVOIDING_LEFT` / `AVOIDING_RIGHT`: once close enough, the planner commits to one side, strafes to a clearance offset, then runs parallel past the obstacle.
4. `RECOVERING`: only after passing the estimated obstacle position plus a margin, the vehicle returns to the original line and heading.
5. `NORMAL`: line keeping resumes.

The pass decision is based on estimated odometry and the estimated obstacle position, not on losing the detection. This prevents YOLO dropouts from causing the rover to return into the obstacle.

Main planner parameters are in:

```text
src/rov_obstacle_avoidance/config/local_avoidance_planner.yaml
```

Relevant parameters include `engage_distance_m`, `clearance_offset_m`, `pass_margin_m`, `go_around_surge`, `target_obstacle_height_m`, and `camera_vertical_fov_deg`.

## YOLO detector

The current supervised detector was trained for:

```text
anchor
mine
torpedo
```

Training/evaluation notes are in:

```text
docs/custom_object_dataset.md
docs/open_vocab_detection_eval.md
docs/yolo_custom_objects_results.md
training/yolo_custom_objects/README.md
```

The final closed-loop demo currently uses the anchor scenario.

## Realism boundaries

Runtime inputs used by the final planner are intended to be realistic sensor-like inputs:

```text
camera -> YOLO detections
DVL/gyro-like velocity -> odometry estimate
manual/nominal velocity command
```

The simulator still generates sensor-like measurements internally from HoloOcean state, as simulators normally do. The key constraint is that the planner does not consume perfect simulator pose or oracle obstacle detections in the YOLO run.

The current simulator command interface is kinematic and abstract. It is for simulation only and does not represent real thruster allocation, MAVLink control, hydrodynamics or vehicle actuation limits.

## What is not included

This repository currently does not command a real ROV, does not connect to MAVLink, does not control thrusters, and does not deploy on BlueOS. The real-world version would need a real sensor bridge, real odometry source, actuator-safe control manager, and hardware safety checks.

## Quick command summary

```bat
:: Build and test
call scripts\source_ros2_windows.bat
colcon build --merge-install
call install\setup.bat
colcon test --event-handlers console_direct+
colcon test-result --verbose

:: Final YOLO demo
scripts\run_custom_anchor_closed_loop.bat --detector yolo ^
  --nominal-publisher-enabled false --manual-nominal-command --duration-s 90

:: Oracle baseline
scripts\run_custom_anchor_closed_loop.bat --detector oracle --duration-s 45

:: Live camera viewer
ros2 run rqt_image_view rqt_image_view /camera/front/image_raw
```
