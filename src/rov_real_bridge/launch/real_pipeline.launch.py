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


def generate_launch_description():
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
        DeclareLaunchArgument("nominal_surge", default_value="0.0"),
        DeclareLaunchArgument("camera_hfov_deg", default_value="74.0"),

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
             condition=IfCondition(PythonExpression(
                 ["'", planner, "' == 'committed'"]))),
        Node(package="rov_obstacle_avoidance",
             executable="dwa_planner_node",
             name="dwa_planner", output="screen",
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
