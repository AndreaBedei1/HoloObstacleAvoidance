from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario_mode = LaunchConfiguration("scenario_mode")

    fake_detector_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_perception"), "config", "fake_detector_scenarios.yaml"]
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
                "scenario_mode",
                default_value="central_static",
                description="Fake obstacle scenario mode for the demo.",
            ),
            Node(
                package="rov_obstacle_perception",
                executable="fake_obstacle_detector_node",
                name="fake_obstacle_detector",
                output="screen",
                parameters=[fake_detector_config, {"scenario_mode": scenario_mode}],
            ),
            Node(
                package="rov_obstacle_avoidance",
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

