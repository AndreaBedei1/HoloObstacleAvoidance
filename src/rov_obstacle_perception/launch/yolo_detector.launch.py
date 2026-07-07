"""YOLO visual detector - camera -> /perception/obstacles.

Runs only the detector node; combine with the HoloOcean bridge launch
(started with relay_oracle_topic:='') and the planner for a full visual
pipeline. Simulation-only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_perception"), "config", "yolo_detector.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=(
                    "training/yolo_custom_objects/runs/"
                    "yolov8n_custom_underwater/weights/best.pt"
                ),
                description=(
                    "Ultralytics weights path. Use an absolute path if launching "
                    "outside the repository root."
                ),
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.25",
                description="Minimum detector confidence published to the planner.",
            ),
            DeclareLaunchArgument(
                "inference_stride",
                default_value="1",
                description="Run inference every Nth camera frame.",
            ),
            Node(
                package="rov_obstacle_perception",
                executable="yolo_obstacle_detector_node",
                name="yolo_obstacle_detector",
                output="screen",
                parameters=[
                    detector_config,
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("confidence_threshold"),
                            value_type=float,
                        ),
                        "inference_stride": ParameterValue(
                            LaunchConfiguration("inference_stride"),
                            value_type=int,
                        ),
                    },
                ],
            ),
        ]
    )
