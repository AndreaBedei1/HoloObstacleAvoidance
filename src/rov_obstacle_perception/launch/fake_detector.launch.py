from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario_mode = LaunchConfiguration("scenario_mode")
    config_file = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_perception"), "config", "fake_detector_scenarios.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_mode",
                default_value="central_static",
                description="Fake obstacle detector scenario mode.",
            ),
            Node(
                package="rov_obstacle_perception",
                executable="fake_obstacle_detector_node",
                name="fake_obstacle_detector",
                output="screen",
                parameters=[config_file, {"scenario_mode": scenario_mode}],
            ),
        ]
    )

