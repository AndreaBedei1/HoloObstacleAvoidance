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
```

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
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
- Add a HoloOcean bridge that converts simulated camera detections into the same perception topic.
- Integrate `/cmd_vel_safe` into the real ROV command manager only after simulation validation and explicit safety review.
