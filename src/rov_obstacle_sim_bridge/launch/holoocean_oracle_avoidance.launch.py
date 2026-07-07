"""Closed-loop obstacle avoidance in real HoloOcean using the oracle detector.

Pipeline (the HoloOcean sim server runs separately in the conda ``ocean`` env)::

    holoocean_sim_server (ocean) --TCP--> holoocean_bridge_node (this launch)
        bridge -> /perception/obstacles_oracle (SIM-ONLY oracle projection)
        bridge relay -> /perception/obstacles (planner input)
        bridge -> /rov/pose_ground_truth (SIM-ONLY debug/validation)
        bridge -> /rov/velocity (DVL+gyro) /rov/depth /camera/front/image_raw
    odometry_estimator_node: /rov/velocity -> /rov/odom_estimated (planner input)
    nominal_cmd_publisher -> /cmd_vel_nominal
    local_avoidance_planner: /perception/obstacles + /cmd_vel_nominal
                              + /rov/odom_estimated
                              -> /planner/cmd_vel_safe -> bridge -> sim server

Start the sim server first (in another terminal, conda ``ocean`` env).  The
supported scenario is the REAL custom anchor on the external modified engine::

    conda run -n ocean python \
      src/rov_obstacle_sim_bridge/holoocean_server/holoocean_sim_server.py \
      --config src/rov_obstacle_sim_bridge/config/holoocean_scenarios/custom_anchor_visible.yaml \
      --serve
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
    estimator_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_sim_bridge"), "config", "odometry_estimator.yaml"]
    )

    bridge_node = Node(
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
    )
    nominal_node = Node(
        package="rov_obstacle_bringup",
        executable="nominal_cmd_publisher_node",
        name="nominal_cmd_publisher",
        output="screen",
        parameters=[nominal_config],
    )
    planner_node = Node(
        package="rov_obstacle_avoidance",
        executable="local_avoidance_planner_node",
        name="local_avoidance_planner",
        output="screen",
        parameters=[planner_config],
    )
    estimator_node = Node(
        package="rov_obstacle_sim_bridge",
        executable="odometry_estimator_node",
        name="odometry_estimator",
        output="screen",
        parameters=[estimator_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="47654"),
            planner_node,
            TimerAction(period=1.0, actions=[bridge_node]),
            TimerAction(period=1.5, actions=[estimator_node]),
            TimerAction(period=2.0, actions=[nominal_node]),
        ]
    )
