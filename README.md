# HoloObstacleAvoidance

First ROS 2 prototype for underwater ROV obstacle avoidance.

This workspace is separate from the `RovTest` reference repository. The reference was used only for package layout, Windows build style, ROS 2 naming conventions, and the safety philosophy: autonomy should publish bounded abstract commands and must not directly command MAVLink thrusters.

## Goal

The first version builds a modular and interpretable avoidance stack:

```text
front camera or simulated detector
  -> obstacle perception
  -> obstacle tracking / risk estimation
  -> local avoidance planner
  -> safe velocity command output
```

The real rover currently has a front camera and no front-facing sonar, so the final runtime pipeline is camera-based. Sonar or simulator ground truth can be useful later as a simulation oracle or label source, but it is not the primary runtime input for the real rover.

The neural detector is intentionally not implemented yet. The fake detector gives deterministic obstacle messages so the planner, topics, launch files, and tests can stabilize first.

## Stabilization Patch

The ROS topic names are configurable through node parameters, while the default demo topics remain unchanged. The fake detector now computes `bearing_rad` from normalized image `center_x` and a configurable horizontal field of view, so left/right/crossing scenarios are geometrically consistent.

HoloOcean integration and the camera neural detector are still future work. This patch keeps `/cmd_vel_safe` as an abstract safe velocity command and does not add simulator, real ROV, MAVLink, thruster, or actuator control.

## Packages

```text
src/
  rov_obstacle_msgs/
  rov_obstacle_perception/
  rov_obstacle_avoidance/
  rov_obstacle_bringup/
  rov_obstacle_sim_bridge/
```

## Simulation Oracle Geometry

`rov_obstacle_sim_bridge` contains pure-Python geometry logic for converting known simulated obstacle world positions and a simulated rover pose into camera-space detections compatible with the existing `Obstacle2DArray` perception interface.

- Uses deterministic geometric projection (no neural network, no real camera images).
- Does not require HoloOcean to be installed.
- Provides reusable dataclasses (`ObstacleConfig`, `RoverPose2D`, `CameraConfig`, `ProjectedObstacle`) and helper functions for world-to-camera transforms, FOV clipping, apparent size estimation, and oracle risk scoring.

### Oracle ROS 2 Nodes

Three nodes wrap the oracle geometry so it can replace the fake detector in a full demo pipeline:

| Node | Package | Input | Output |
| --- | --- | --- | --- |
| `simulated_rover_pose_publisher_node` | `rov_obstacle_sim_bridge` | — | `/sim/rov_pose` (`PoseStamped`) |
| `holoocean_obstacle_oracle_node` | `rov_obstacle_sim_bridge` | `/sim/rov_pose` | `/perception/obstacles` (`Obstacle2DArray`) |
| `cmd_vel_safe_logger_node` | `rov_obstacle_sim_bridge` | `/cmd_vel_safe` | CSV log file (optional) |

The simulated pose publisher supports four motion modes: `static`, `forward`, `lateral`, and `yaw_scan`. All parameters are configurable via YAML or launch arguments.

### Run The Oracle Demo

```bat
cd /d C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance
call scripts\source_ros2_windows.bat
call install\setup.bat
ros2 launch rov_obstacle_sim_bridge holoocean_oracle_demo.launch.py
```

Choose a motion mode:

```bat
ros2 launch rov_obstacle_sim_bridge holoocean_oracle_demo.launch.py motion_mode:=static
ros2 launch rov_obstacle_sim_bridge holoocean_oracle_demo.launch.py motion_mode:=forward
ros2 launch rov_obstacle_sim_bridge holoocean_oracle_demo.launch.py motion_mode:=lateral
ros2 launch rov_obstacle_sim_bridge holoocean_oracle_demo.launch.py motion_mode:=yaw_scan
```

Enable CSV logging of `/cmd_vel_safe`:

```bat
ros2 launch rov_obstacle_sim_bridge holoocean_oracle_demo.launch.py ^
  log_file:=C:/Users/andrea.bedei3/Desktop/HoloObstacleAvoidance/logs/cmd_vel_safe.csv
```

## Oracle Demo Recording

The `oracle_demo_recorder` node passively records the full oracle demo pipeline to a CSV file for quantitative validation. It subscribes to all five topics and writes one row per sample interval without publishing any commands.

### CSV Columns

| Column | Description |
| --- | --- |
| `timestamp_s` | Elapsed seconds since recorder start |
| `rov_x`, `rov_y`, `rov_z` | Simulated rover position |
| `obstacle_count` | Number of detected obstacles |
| `max_obstacle_risk` | Highest risk among current obstacles |
| `most_dangerous_center_x` | Normalized image x of the highest-risk obstacle |
| `most_dangerous_bearing_rad` | Bearing in radians of the highest-risk obstacle |
| `nominal_surge`, `nominal_sway`, `nominal_yaw_rate` | Nominal command components |
| `safe_surge`, `safe_sway`, `safe_yaw_rate` | Safe (planner output) command components |
| `planner_state` | Current planner state string (`NORMAL`, `AVOIDING`, `RECOVERING`) |
| `selected_side` | Selected avoidance side (`LEFT`, `RIGHT`, or empty) |
| `debug_risk` | Current debug risk value |

### Run With Recording

```bat
cd /d C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance
call scripts\source_ros2_windows.bat
call install\setup.bat
ros2 launch rov_obstacle_sim_bridge oracle_recording_demo.launch.py
```

Configure recording parameters via launch arguments:

```bat
ros2 launch rov_obstacle_sim_bridge oracle_recording_demo.launch.py ^
  motion_mode:=forward ^
  output_csv:=logs/oracle_demo_record.csv ^
  duration_s:=30.0 ^
  auto_shutdown:=true
```

With `auto_shutdown:=true`, the recorder and all pipeline nodes shut down automatically after `duration_s` seconds. The CSV file is written to the specified path (default: `logs/oracle_demo_record.csv`).

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/sim/rov_pose` | `geometry_msgs/msg/PoseStamped` | Simulated rover pose (oracle demo only). |
| `/perception/obstacles` | `rov_obstacle_msgs/msg/Obstacle2DArray` | Fake detector output for now. |
| `/cmd_vel_nominal` | `geometry_msgs/msg/Twist` | Desired operator/autonomy velocity before avoidance. |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | Planner output only; no thrusters or MAVLink commands. |
| `/avoidance/debug` | `rov_obstacle_msgs/msg/AvoidanceDebug` | Current planner state, side, risk, and selected command. |

`geometry_msgs/Twist` mapping:

- `linear.x`: surge
- `linear.y`: sway
- `linear.z`: heave, preserved from nominal command
- `angular.z`: yaw rate
- `angular.x` and `angular.y` are preserved from nominal command

## Abstract Command Sign Convention

The current planner uses image-space obstacle position to select an abstract avoidance side:

- obstacle on the left side of the image -> avoid right
- obstacle on the right side of the image -> avoid left
- `AvoidanceSide.LEFT` produces positive `linear.y` sway and positive `angular.z` yaw rate
- `AvoidanceSide.RIGHT` produces negative `linear.y` sway and negative `angular.z` yaw rate

This sign convention must be verified against HoloOcean body-frame conventions and against the real BlueROV command convention before connecting `/cmd_vel_safe` to any simulator or real vehicle controller.

## Messages

`Obstacle2D` contains normalized image-space bounding box fields, bearing, apparent area, risk, and tracking validity.

`Obstacle2DArray` wraps a header and a list of obstacles.

`AvoidanceDebug` reports the planner state, selected side, risk, and desired surge/sway/yaw rate.

## Build On Windows

Open `cmd.exe`, then:

```bat
cd /d C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance
call scripts\source_ros2_windows.bat
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
python scripts\preflight_ros2_windows.py
colcon build --merge-install
call install\setup.bat
```

The verified local ROS 2 install is ROS 2 Lyrical under:

```text
C:\dev\lyrical
```

`--merge-install` is used because it is the most reliable Windows layout and matches the reference workspace style.

## Run The Fake Demo

```bat
cd /d C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance
call scripts\source_ros2_windows.bat
call install\setup.bat
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py
```

Choose a scenario:

```bat
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=left_static
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=right_static
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=crossing_left_to_right
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=approaching
```

Inspect outputs:

```bat
ros2 topic echo /perception/obstacles
ros2 topic echo /cmd_vel_nominal
ros2 topic echo /cmd_vel_safe
ros2 topic echo /avoidance/debug
```

## Planner Behavior

In `NORMAL`, `/cmd_vel_nominal` passes through. When risk crosses the enter threshold, the planner reduces surge and chooses a stable avoidance side. Left obstacles cause right avoidance; right obstacles cause left avoidance; central obstacles choose the side with more apparent free image space. After risk drops below the exit threshold, the planner blends back to nominal over the configured recovery time.

## Tests

```bat
call scripts\source_ros2_windows.bat
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## TODO

- Replace the fake detector with a camera neural detector that publishes `Obstacle2DArray`.
- Integrate `/cmd_vel_safe` into the real ROV command manager only after simulation validation and explicit safety review.
