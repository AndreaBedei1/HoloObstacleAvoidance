from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_avoidance"), "config", "avoidance_planner.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="rov_obstacle_avoidance",
                executable="local_avoidance_planner_node",
                name="local_avoidance_planner",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )

