"""Closed-loop obstacle avoidance in real HoloOcean using the oracle detector.

Pipeline (the HoloOcean sim server runs separately in the conda ``ocean`` env)::

    holoocean_sim_server (ocean) --TCP--> holoocean_bridge_node (this launch)
        bridge -> /perception/obstacles (oracle projection, Mode 2 detector)
        bridge -> /rov/pose /rov/velocity /rov/depth /camera/front/image_raw
    nominal_cmd_publisher -> /cmd_vel_nominal
    local_avoidance_planner: /perception/obstacles + /cmd_vel_nominal
                              -> /planner/cmd_vel_safe -> bridge -> sim server

Start the sim server first (in another terminal, conda ``ocean`` env)::

    conda run -n ocean python \
      src/rov_obstacle_sim_bridge/holoocean_server/holoocean_sim_server.py \
      --config src/rov_obstacle_sim_bridge/config/holoocean_scenarios/sphere_front.yaml \
      --serve
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="47654"),
            # Bridge: oracle projection published directly on /perception/obstacles
            # so it acts as the "Mode 2" HoloOcean oracle detector for the planner.
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
                        "oracle_topic": "/perception/obstacles",
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
