"""Closed-loop HoloOcean avoidance using the YOLO visual detector.

Pipeline (the HoloOcean sim server runs separately in the conda ``ocean`` env)::

    holoocean_sim_server (ocean) --TCP--> holoocean_bridge_node
        bridge -> /perception/obstacles_oracle (SIM-ONLY debug/validation)
        bridge -> /rov/pose_ground_truth (SIM-ONLY debug/validation)
        bridge -> /rov/velocity (DVL+gyro) /rov/depth /camera/front/image_raw
    odometry_estimator_node: /rov/velocity -> /rov/odom_estimated (realistic,
                              drifting; the planner's only pose input)
    yolo_obstacle_detector_node: /camera/front/image_raw -> /perception/obstacles
    nominal_cmd_publisher -> /cmd_vel_nominal (optional)
    local_avoidance_planner: /perception/obstacles + /cmd_vel_nominal
                              + /rov/odom_estimated
                              -> /planner/cmd_vel_safe -> bridge -> sim server

The oracle relay is intentionally disabled so the planner receives only YOLO
detections. Ground-truth pose is published only for validation; the planner
navigates on the estimated odometry.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    detector_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_perception"), "config", "yolo_detector.yaml"]
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
                "relay_oracle_topic": "",
            },
        ],
    )
    detector_node = Node(
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
    )
    nominal_node = Node(
        package="rov_obstacle_bringup",
        executable="nominal_cmd_publisher_node",
        name="nominal_cmd_publisher",
        output="screen",
        parameters=[nominal_config],
        condition=IfCondition(LaunchConfiguration("nominal_publisher_enabled")),
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
            DeclareLaunchArgument(
                "model_path",
                default_value=(
                    "training/yolo_custom_objects/runs/"
                    "yolov8n_custom_underwater/weights/best.pt"
                ),
            ),
            DeclareLaunchArgument("confidence_threshold", default_value="0.25"),
            DeclareLaunchArgument("inference_stride", default_value="1"),
            DeclareLaunchArgument(
                "nominal_publisher_enabled",
                default_value="true",
                description=(
                    "Start the built-in nominal command publisher. Set false "
                    "when publishing /cmd_vel_nominal manually from ROS."
                ),
            ),
            bridge_node,
            TimerAction(period=1.5, actions=[estimator_node]),
            TimerAction(period=2.0, actions=[planner_node]),
            TimerAction(period=4.0, actions=[detector_node]),
            TimerAction(period=5.0, actions=[nominal_node]),
        ]
    )
