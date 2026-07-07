"""Closed-loop avoidance launch for the REAL custom anchor scenarios.

The HoloOcean sim server still runs separately in the conda ``ocean`` env
(it launches the EXTERNAL modified engine and spawns /Game/ancora.ancora).
This launch starts only the ROS 2 side: TCP bridge, nominal command
publisher, and the generic local avoidance planner.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_sim_bridge"), "config", "holoocean_bridge.yaml"]
    )
    planner_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_avoidance"), "config", "local_avoidance_planner.yaml"]
    )
    nominal_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_bringup"), "config", "demo.yaml"]
    )
    default_scenario = PathJoinSubstitution(
        [
            FindPackageShare("rov_obstacle_sim_bridge"),
            "config",
            "holoocean_scenarios",
            "custom_anchor_visible.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="47654"),
            DeclareLaunchArgument("scenario_config", default_value=default_scenario),
            LogInfo(
                msg=[
                    "Start holoocean_sim_server separately with scenario_config=",
                    LaunchConfiguration("scenario_config"),
                ]
            ),
            Node(
                package="rov_obstacle_sim_bridge",
                executable="holoocean_bridge_node",
                name="holoocean_bridge",
                output="screen",
                parameters=[
                    bridge_config,
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "relay_oracle_topic": "/perception/obstacles",
                    },
                ],
            ),
            Node(
                package="rov_obstacle_bringup",
                executable="nominal_cmd_publisher_node",
                name="nominal_cmd_publisher",
                output="screen",
                parameters=[nominal_config],
            ),
            Node(
                package="rov_obstacle_avoidance",
                executable="local_avoidance_planner_node",
                name="local_avoidance_planner",
                output="screen",
                parameters=[planner_config],
            ),
        ]
    )
