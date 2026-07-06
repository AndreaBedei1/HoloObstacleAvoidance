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

## Runtime Architecture

```text
/camera/front/image_raw
  -> future RGB perception node
/perception/obstacles
  -> local avoidance planner
/cmd_vel_nominal
  -> local avoidance planner
/planner/cmd_vel_safe
  -> future control manager / future MAVLink bridge
```

The current fake detector skips `/camera/front/image_raw` and publishes `Obstacle2DArray` directly so the planner can be tested before the neural detector exists.

## Real HoloOcean Bridge (Two-Process)

HoloOcean and ROS 2 run in **two different Python interpreters that cannot share
a process**:

- HoloOcean 2.3.0 -> conda env `ocean` (Python 3.9).
- ROS 2 Lyrical -> pixi env at `C:\dev\lyrical` (Python 3.12).

So the integration is a **two-process bridge** connected by a localhost TCP
socket. This matches the design goal "HoloOcean is a simulator that publishes
sensors and vehicle state into ROS 2":

```text
[conda ocean / py3.9]                         [pixi ROS 2 / py3.12]
holoocean_sim_server.py  -- TCP 127.0.0.1 -->  holoocean_bridge_node
  make(scenario) + spawn primitive props       /camera/front/image_raw (Image rgb8)
  step sim, read camera/pose/vel/depth         /rov/pose /rov/velocity /rov/depth
  apply incoming cmd_vel (kinematic)  <-- TCP -- /perception/obstacles_oracle  (SIM-ONLY)
                                               forwards /planner/cmd_vel_safe -> server
       shared: rov_obstacle_sim_bridge/sim_bridge_protocol.py  (stdlib only, both envs)
```

The sim server (`src/rov_obstacle_sim_bridge/holoocean_server/holoocean_sim_server.py`)
spawns obstacles with HoloOcean's native `spawn_prop` (`sphere`, `box`,
`cylinder`, `cone`). Scenarios live in
`config/holoocean_scenarios/*.yaml`; obstacle `relative_position` is
`[forward_m, left_m, up_m]` in the rover's spawn body frame. Vehicle motion is a
**kinematic teleport** convenience for simulation only; it never touches
thrusters, MAVLink or a real ROV.

`/perception/obstacles_oracle` is a **simulation-only** ground-truth projection
(debugging / dataset labels / planner validation), never a real onboard sensor.

### Primitive-composed semantic objects

HoloOcean 2.3.0 is used here through `spawn_prop` primitives only. This setup
has no confirmed custom mesh import API, so complex obstacles are approximated
with grouped primitive parts instead of external meshes.

Scenario YAML supports both simple `obstacles` and grouped `semantic_objects`.
A simple obstacle spawns one primitive and publishes one oracle obstacle. A
semantic object spawns every primitive in its `parts` list, then publishes one
aggregated oracle obstacle by default:

```yaml
semantic_objects:
  - name: anchor_center
    class_name: anchor
    relative_position: [10.0, 0.0, 0.0]  # [forward_m, left_m, up_m]
    parts:
      - name: stem
        prop_type: box
        relative_position: [0.0, 0.0, 0.0]
        scale: [0.35, 0.35, 3.2]
      - name: upper_crossbar
        prop_type: box
        relative_position: [0.0, 0.0, 1.15]
        scale: [0.35, 3.2, 0.35]
```

The anchor scenarios build an approximate anchor from a central vertical bar,
an upper crossbar, two angled lower arms, lateral sphere tips, and optional
sphere details for the top ring. The oracle projects the aggregate bounds as a
single `class_name: anchor` detection with one bounding box, bearing, apparent
area, risk, confidence, and valid tracking flag. Primitive-level oracle
detections are disabled by default and are available only through
`oracle.debug_primitive_detections: true`.

### Coordinate convention (calibrated against real renders)

HoloOcean's world frame is right-handed REP-103: **+X forward, +Y left, +Z up**.
Verified empirically — facing +X, a sphere at world +Y renders on the image
LEFT, and teleport yaw=+90 deg turns the camera toward +Y. The bridge negates
y/yaw when projecting through `oracle_geometry` (which uses +y = right) so the
oracle `center_x` matches where the obstacle actually appears.

### Run real HoloOcean closed loop

Terminal 1: sim server in the conda `ocean` env. Use `sphere_front.yaml` for
the baseline sphere scenario:

```bat
conda run -n ocean python ^
  src\rov_obstacle_sim_bridge\holoocean_server\holoocean_sim_server.py ^
  --config src\rov_obstacle_sim_bridge\config\holoocean_scenarios\sphere_front.yaml ^
  --serve
```

Or run the centered primitive anchor:

```bat
conda run -n ocean python ^
  src\rov_obstacle_sim_bridge\holoocean_server\holoocean_sim_server.py ^
  --config src\rov_obstacle_sim_bridge\config\holoocean_scenarios\anchor_center_static.yaml ^
  --serve
```

Terminal 2: ROS 2 closed-loop bridge and generic planner in the ROS 2 env:

```bat
call scripts\source_ros2_windows.bat
call install\setup.bat
ros2 launch rov_obstacle_sim_bridge holoocean_oracle_avoidance.launch.py
```

For anchor work, the equivalent convenience launch is:

```bat
ros2 launch rov_obstacle_sim_bridge holoocean_anchor_avoidance.launch.py ^
  scenario_config:=src\rov_obstacle_sim_bridge\config\holoocean_scenarios\anchor_center_static.yaml
```

The `scenario_config` launch argument is informational; the HoloOcean server is
still started separately in Terminal 1 because it must run in the conda `ocean`
Python 3.9 process.

Available HoloOcean scenario YAMLs:

- `sphere_front.yaml`
- `sphere_left.yaml`
- `sphere_right.yaml`
- `multi_sphere.yaml`
- `anchor_center_static.yaml`
- `anchor_left_static.yaml`
- `anchor_right_static.yaml`
- `anchor_partially_visible.yaml`
- `anchor_with_spheres.yaml`

Smoke-test the sim server alone (no ROS 2, real HoloOcean) with a scripted
forward run:

```bat
conda run -n ocean python ^
  src\rov_obstacle_sim_bridge\holoocean_server\holoocean_sim_server.py ^
  --config src\rov_obstacle_sim_bridge\config\holoocean_scenarios\sphere_front.yaml ^
  --selftest
```

Anchor self-test:

```bat
conda run -n ocean python ^
  src\rov_obstacle_sim_bridge\holoocean_server\holoocean_sim_server.py ^
  --config src\rov_obstacle_sim_bridge\config\holoocean_scenarios\anchor_center_static.yaml ^
  --selftest
```

Verified end-to-end (sim server + bridge + planner) in real HoloOcean: the
planner detects the central sphere, runs the full
`NORMAL -> APPROACH_OBSTACLE -> AVOIDING_LEFT -> RECOVERING -> NORMAL` cycle, and
the rover deviates and recovers.

## Stabilization Patch

The ROS topic names are configurable through node parameters, while the default demo topics remain unchanged. The fake detector now computes `bearing_rad` from normalized image `center_x` and a configurable horizontal field of view, so left/right/crossing scenarios are geometrically consistent.

The HoloOcean bridge is simulation-only and keeps `/planner/cmd_vel_safe` as an abstract safe velocity command. It does not connect to a real ROV, MAVLink, thrusters, or actuator control. The camera neural detector remains future work.

## Packages

```text
src/
  rov_obstacle_msgs/
  rov_obstacle_perception/
  rov_obstacle_avoidance/
  rov_obstacle_bringup/
  rov_obstacle_sim_bridge/
```

## Configuration Files

- `rov_obstacle_perception/config/fake_detector.yaml`: fake detector topic, scenario, geometry, and risk defaults.
- `rov_obstacle_avoidance/config/local_avoidance_planner.yaml`: planner topics, risk thresholds, hold/recovery timing, and command limits.
- `rov_obstacle_bringup/config/demo.yaml`: simple nominal command publisher defaults for local demos.

The legacy YAML names are kept for compatibility, but new launches use the files above.

## Simulation Oracle Geometry

`rov_obstacle_sim_bridge` contains pure-Python geometry logic for converting known simulated obstacle world positions and a simulated rover pose into camera-space detections compatible with the existing `Obstacle2DArray` perception interface.

- Uses deterministic geometric projection (no neural network, no real camera images).
- Does not require HoloOcean to be installed.
- Provides reusable dataclasses (`ObstacleConfig`, `RoverPose2D`, `CameraConfig`, `ProjectedObstacle`) and helper functions for world-to-camera transforms, FOV clipping, apparent size estimation, grouped-object bounds projection, and oracle risk scoring.
- Risk scoring considers image centrality, apparent size, simulated range, class weight, confidence, and closing speed when a simulated velocity is available.

### Oracle ROS 2 Nodes

Four nodes wrap the oracle geometry so it can replace the fake detector in a full demo pipeline:

| Node | Package | Input | Output |
| --- | --- | --- | --- |
| `simulated_rover_pose_publisher_node` | `rov_obstacle_sim_bridge` | — | `/sim/rov_pose` (`PoseStamped`) |
| `holoocean_pose_bridge_node` | `rov_obstacle_sim_bridge` | HoloOcean env (optional) | `/sim/rov_pose` (`PoseStamped`) |
| `holoocean_obstacle_oracle_node` | `rov_obstacle_sim_bridge` | `/sim/rov_pose` | `/perception/obstacles` (`Obstacle2DArray`) |
| `cmd_vel_safe_logger_node` | `rov_obstacle_sim_bridge` | `/planner/cmd_vel_safe` | CSV log file (optional) |

The simulated pose publisher supports four motion modes: `static`, `forward`, `lateral`, and `yaw_scan`. All parameters are configurable via YAML or launch arguments.

The HoloOcean pose bridge attempts to import HoloOcean at startup; when unavailable it falls back to a deterministic fake pose so the entire pipeline still runs for smoke testing. It does not send `/planner/cmd_vel_safe`, does not control thrusters, and does not connect to MAVLink or the real ROV.

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

Enable CSV logging of `/planner/cmd_vel_safe`:

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

### Analyze a Recording

After a recording completes, run the standalone analysis script to validate the avoidance behavior:

```bat
python scripts\analyze_oracle_recording.py logs/oracle_demo_record.csv
```

The report prints:

- Total samples and recording duration
- Maximum obstacle risk observed
- Number of samples with obstacles, non-NORMAL planner states, and command differences
- Timestamps of the first high-risk event, first avoidance activation, and first command difference
- Peak safe sway and yaw rate magnitudes

If `matplotlib` is installed, the script also saves a plot to `logs/oracle_demo_record_plot.png` showing risk, safe sway, and safe yaw over time. Matplotlib is optional—analysis runs without it.

## HoloOcean Pose Smoke Bridge

The `holoocean_pose_bridge_node` reads or simulates ROV pose and publishes `/sim/rov_pose`. When HoloOcean is installed the node opens a scenario, steps each timer tick, reads agent pose and publishes a `PoseStamped`. When HoloOcean is unavailable (or `use_holoocean=False`) the node falls back to a deterministic fake pose so the rest of the pipeline can run.

This bridge does **not** send `/planner/cmd_vel_safe`, does **not** control thrusters, and does **not** connect to MAVLink or the real ROV. It only reads/simulates pose.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `output_topic` | `/sim/rov_pose` | Output topic name. |
| `publish_rate_hz` | `20.0` | Timer frequency. |
| `frame_id` | `world` | Header frame ID. |
| `scenario_name` | `OpenWater-Hovering` | HoloOcean scenario name. |
| `agent_name` | `auv0` | Agent key in the HoloOcean state dict. |
| `use_holoocean` | `true` | Attempt to use HoloOcean for pose reading. |
| `fallback_to_fake_pose` | `true` | Fall back to fake pose when HoloOcean fails. |
| `fake_velocity_x` | `0.2` | Fake pose drift velocity along X (m/s). |

### Run The HoloOcean Smoke Bridge

```bat
cd /d C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance
call scripts\source_ros2_windows.bat
call install\setup.bat
ros2 launch rov_obstacle_sim_bridge holoocean_pose_smoke.launch.py
```

Run with fake-pose fallback (no HoloOcean required):

```bat
ros2 launch rov_obstacle_sim_bridge holoocean_pose_smoke.launch.py ^
  use_holoocean:=false ^
  output_csv:=logs/holoocean_fake_smoke.csv ^
  duration_s:=10.0 ^
  auto_shutdown:=true
```

Analyze the resulting CSV:

```bat
python scripts\analyze_oracle_recording.py logs/holoocean_fake_smoke.csv
```

### Helper Functions

Three pure-Python helpers are testable without HoloOcean or ROS 2:

- `quaternion_from_yaw(yaw_rad)` — returns a yaw-only `Quaternion`.
- `pose_from_holoocean_state(state, agent_name)` — robustly extracts `(x, y, z, yaw_rad)` from multiple HoloOcean state dict layouts.
- `fake_pose_at_time(t, ...)` — returns a deterministic linear-drift pose for smoke testing.

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/sim/rov_pose` | `geometry_msgs/msg/PoseStamped` | Simulated rover pose (oracle demo only). |
| `/perception/obstacles` | `rov_obstacle_msgs/msg/Obstacle2DArray` | Planner input from the fake detector or the simulation-only HoloOcean oracle remap. |
| `/cmd_vel_nominal` | `geometry_msgs/msg/Twist` | Desired operator/autonomy velocity before avoidance. |
| `/planner/cmd_vel_safe` | `geometry_msgs/msg/Twist` | Planner output only; no thrusters or MAVLink commands. |
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

This sign convention must be verified against HoloOcean body-frame conventions and against the real BlueROV command convention before connecting `/planner/cmd_vel_safe` to any simulator or real vehicle controller.

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

## Build On Ubuntu

Use the ROS 2 distro matching the OS (`jazzy` on Ubuntu 24.04, `humble` on Ubuntu 22.04):

```bash
cd ~/HoloObstacleAvoidance
source /opt/ros/<distro>/setup.bash
colcon build --merge-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=central_static
ros2 topic echo /planner/cmd_vel_safe
```

## Run The Fake Demo

```bat
cd /d C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance
call scripts\source_ros2_windows.bat
call install\setup.bat
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py
```

Choose a scenario:

```bat
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=none
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=central_static
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=left_static
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=right_static
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=crossing_left_to_right
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=crossing_right_to_left
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=approaching
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=disappearing
ros2 launch rov_obstacle_bringup obstacle_avoidance_demo.launch.py scenario_mode:=intermittent
```

Expected fake scenario behavior:

- `none`: publishes an empty obstacle array.
- `central_static`: publishes a central high-risk obstacle.
- `left_static`: publishes an obstacle on the left; planner should avoid right.
- `right_static`: publishes an obstacle on the right; planner should avoid left.
- `crossing_left_to_right`: obstacle moves from left to right over time.
- `crossing_right_to_left`: obstacle moves from right to left over time.
- `approaching`: obstacle grows in apparent size and risk.
- `disappearing`: obstacle is removed after a few seconds.
- `intermittent`: obstacle appears and drops out deterministically.

Inspect outputs:

```bat
ros2 topic echo /perception/obstacles
ros2 topic echo /cmd_vel_nominal
ros2 topic echo /planner/cmd_vel_safe
ros2 topic echo /avoidance/debug
```

## Planner Behavior

In `NORMAL`, `/cmd_vel_nominal` passes through. When risk crosses the enter threshold, the planner reduces surge and chooses a stable avoidance side. Left obstacles cause right avoidance; right obstacles cause left avoidance; central obstacles choose the side with more apparent free image space. After risk drops below the exit threshold, the planner blends back to nominal over the configured recovery time.

Demo launch arguments:

- `scenario_mode`
- `risk_enter_threshold`
- `risk_exit_threshold`
- `avoidance_sway`
- `avoidance_yaw_rate`
- `min_avoidance_hold_s`

## HoloOcean Status And Limits

Current HoloOcean integration is a simulation-only, two-process closed loop:

```text
HoloOcean RGB camera -> /camera/front/image_raw
HoloOcean pose/velocity/depth -> /rov/pose, /rov/velocity, /rov/depth
HoloOcean ground truth -> /perception/obstacles_oracle or /perception/obstacles
nominal command publisher -> /cmd_vel_nominal
local avoidance planner -> /planner/cmd_vel_safe
bridge forwards abstract safe velocity -> sim server kinematic teleport
```

Known limitations:

- Custom mesh import is not confirmed in this HoloOcean 2.3.0 setup; complex objects are approximated from primitives.
- The oracle is ground truth for simulation, debugging, and validation only. It is not a real onboard sensor.
- Primitive aggregate bounds are approximate and conservative, especially for rotated parts.
- Vehicle motion in the sim server is kinematic teleport, not hydrodynamic thruster control.
- No neural detector, dataset export, MAVLink bridge, real thruster command path, or real rover integration is implemented here.

## Tests

```bat
call scripts\source_ros2_windows.bat
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## TODO

- Validate all anchor scenarios in real HoloOcean and record planner state transitions.
- Replace the fake detector with a camera neural detector that publishes `Obstacle2DArray`.
- Collect synthetic RGB images from HoloOcean and use oracle labels for obstacle training.
- Compare no avoidance, oracle/fake avoidance, and RGB neural perception avoidance.
- Integrate `/planner/cmd_vel_safe` into the real ROV command manager only after simulation validation and explicit safety review.
