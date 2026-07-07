"""YOLO visual detector (skeleton) — camera -> /perception/obstacles.

Runs only the detector node; combine with the HoloOcean bridge launch
(started with relay_oracle_topic:='') and the planner for a full visual
pipeline.  Simulation-only.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_perception"), "config", "yolo_detector.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="rov_obstacle_perception",
                executable="yolo_obstacle_detector_node",
                name="yolo_obstacle_detector",
                output="screen",
                parameters=[detector_config],
            ),
        ]
    )
