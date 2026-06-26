"""Oracle demo launch: simulated rover pose -> oracle obstacles -> avoidance planner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_mode = LaunchConfiguration("motion_mode")
    obstacle_config = LaunchConfiguration("obstacle_config")

    oracle_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_sim_bridge"), "config", "holoocean_oracle.yaml"]
    )
    obstacles_simple = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_sim_bridge"), "config", "obstacles_simple.yaml"]
    )
    planner_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_avoidance"), "config", "avoidance_planner.yaml"]
    )
    nominal_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_avoidance"), "config", "nominal_cmd_publisher.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "motion_mode",
                default_value="forward",
                description="Rover motion mode: static, forward, lateral, yaw_scan.",
            ),
            DeclareLaunchArgument(
                "obstacle_config",
                default_value=obstacles_simple,
                description="Path to the obstacle YAML configuration file.",
            ),
            # Simulated rover pose publisher
            Node(
                package="rov_obstacle_sim_bridge",
                executable="simulated_rover_pose_publisher_node",
                name="simulated_rover_pose_publisher",
                output="screen",
                parameters=[oracle_config, {"motion_mode": motion_mode}],
            ),
            # Oracle obstacle projector
            Node(
                package="rov_obstacle_sim_bridge",
                executable="holoocean_obstacle_oracle_node",
                name="holoocean_obstacle_oracle",
                output="screen",
                parameters=[oracle_config, {"obstacle_config_file": obstacle_config}],
            ),
            # Nominal command publisher (same as fake demo)
            Node(
                package="rov_obstacle_avoidance",
                executable="nominal_cmd_publisher_node",
                name="nominal_cmd_publisher",
                output="screen",
                parameters=[nominal_config],
            ),
            # Local avoidance planner (same as fake demo)
            Node(
                package="rov_obstacle_avoidance",
                executable="local_avoidance_planner_node",
                name="local_avoidance_planner",
                output="screen",
                parameters=[planner_config],
            ),
            # Command logger (disabled by default, enable via log_file parameter)
            Node(
                package="rov_obstacle_sim_bridge",
                executable="cmd_vel_safe_logger_node",
                name="cmd_vel_safe_logger",
                output="screen",
                parameters=[oracle_config],
            ),
        ]
    )
