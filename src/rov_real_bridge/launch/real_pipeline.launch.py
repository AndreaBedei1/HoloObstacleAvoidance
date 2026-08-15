"""REAL scientific pipeline: the same shared stack as simulation.

    real camera -> real detector -> /perception/obstacles_raw
        -> temporal_estimator (Phase-7B qualification + T2)
        -> /perception/obstacles
        -> Planner C or Planner D
        -> /planner/cmd_vel_safe
        -> real adapter (SHADOW by default) -> ArduSub

Everything between /perception/obstacles_raw and /planner/cmd_vel_safe is
the SAME source code that runs in simulation; only the nodes at the two
ends are platform-specific (docs/EXPERIMENTAL_BOUNDARY.md).

The adapter starts in SHADOW: it computes, logs and publishes what it
WOULD send, and transmits nothing. Real actuation requires all three of
real_control_mode:=live vehicle_in_water:=true allow_real_actuation:=true
AND a fully calibrated command mapping, or the node refuses and stays in
shadow.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _pool_benchmark():
    """The frozen pool benchmark, the SAME file the simulated campaign
    reads.

    Until 2026-08-15 this launch started the planner nodes with no
    parameters, so they used the defaults tuned for the 11 m simulated
    scenarios: a 9 m engagement distance in a 6 m pool, an assumed
    obstacle height of 3.5 m for an anchor of 0.5 m, and a DWA obstacle
    radius seven times too large. The simulation used the pool values.
    The sim-to-real comparison would have been between two differently
    configured planners, and nothing in either domain's output would
    have shown it.
    """
    import os
    import yaml
    rel = os.path.join("config", "pool_benchmark_FROZEN.yaml")
    # Search upward: this launch file runs from the source tree during
    # development and from install/share once built, so a fixed relative
    # path is right in one place and wrong in the other. An explicit
    # environment override comes first for out-of-tree installs.
    roots = []
    env = os.environ.get("HOLO_REPO_ROOT")
    if env:
        roots.append(env)
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        roots.append(here)
        here = os.path.dirname(here)
    for root in roots:
        path = os.path.join(root, rel)
        if os.path.isfile(path):
            with open(path) as f:
                return yaml.safe_load(f)["pool_benchmark"]
    # Never fall back to the library defaults: they are the 11 m
    # scenario values, and a real run started with them would look
    # normal while being a different experiment from the simulation.
    raise RuntimeError(
        "pool_benchmark_FROZEN.yaml not found; refusing to launch the "
        "real pipeline with the default planner configuration, which is "
        "tuned for the 11 m simulated scenarios and would silently make "
        "the real campaign incomparable to the predictions. Set "
        "HOLO_REPO_ROOT to the repository root.")


def generate_launch_description():
    pool = _pool_benchmark()
    planner = LaunchConfiguration("planner")
    return LaunchDescription([
        # OpenCV's FFMPEG backend reads this at DLL load time; setting it
        # inside a running interpreter is ignored.
        SetEnvironmentVariable(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "protocol_whitelist;file,rtp,udp|fflags;nobuffer|"
            "flags;low_delay"),
        DeclareLaunchArgument("planner", default_value="committed"),
        DeclareLaunchArgument("estimator_method", default_value="t2"),
        DeclareLaunchArgument("real_control_mode", default_value="shadow"),
        DeclareLaunchArgument("vehicle_in_water", default_value="false"),
        DeclareLaunchArgument("allow_real_actuation",
                              default_value="false"),
        DeclareLaunchArgument("calibration_id",
                              default_value="uncalibrated"),
        # Defaults come from the frozen pool benchmark, never from this
        # file: one source, both domains.
        DeclareLaunchArgument("nominal_surge",
                              default_value=str(pool["nominal_surge"])),
        DeclareLaunchArgument("camera_hfov_deg",
                              default_value=str(pool["camera_hfov_deg"])),

        Node(package="rov_real_bridge", executable="real_detector_node",
             name="real_detector", output="screen",
             parameters=[{
                 "camera_horizontal_fov_deg":
                     LaunchConfiguration("camera_hfov_deg"),
             }]),
        Node(package="rov_obstacle_tracking",
             executable="temporal_estimator_node",
             name="temporal_estimator", output="screen",
             parameters=[{
                 "method": LaunchConfiguration("estimator_method"),
             }]),
        Node(package="rov_obstacle_bringup",
             executable="nominal_cmd_publisher_node",
             name="nominal_cmd_publisher", output="screen",
             parameters=[{"surge": LaunchConfiguration("nominal_surge")}]),
        Node(package="rov_obstacle_avoidance",
             executable="local_avoidance_planner_node",
             name="local_avoidance_planner", output="screen",
             parameters=[{
                 "engage_distance_m": float(pool["engage_distance_m"]),
                 "target_obstacle_height_m":
                     float(pool["target_obstacle_height_m"]),
             }],
             condition=IfCondition(PythonExpression(
                 ["'", planner, "' == 'committed'"]))),
        Node(package="rov_obstacle_avoidance",
             executable="dwa_planner_node",
             name="dwa_planner", output="screen",
             parameters=[{
                 "target_obstacle_height_m":
                     float(pool["target_obstacle_height_m"]),
                 "obstacle_radius_m":
                     float(pool["dwa_obstacle_radius_m"]),
                 "goal_lookahead_m":
                     float(pool["dwa_goal_lookahead_m"]),
             }],
             condition=IfCondition(PythonExpression(
                 ["'", planner, "' == 'dwa'"]))),
        Node(package="rov_real_bridge",
             executable="real_control_live_node",
             name="real_control_live", output="screen",
             parameters=[{
                 "real_control_mode":
                     LaunchConfiguration("real_control_mode"),
                 "vehicle_in_water":
                     LaunchConfiguration("vehicle_in_water"),
                 "allow_real_actuation":
                     LaunchConfiguration("allow_real_actuation"),
                 "calibration_id":
                     LaunchConfiguration("calibration_id"),
             }]),
    ])
