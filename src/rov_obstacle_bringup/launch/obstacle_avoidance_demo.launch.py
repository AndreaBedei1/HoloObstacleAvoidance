from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scenario_mode = LaunchConfiguration("scenario_mode")
    risk_enter_threshold = LaunchConfiguration("risk_enter_threshold")
    risk_exit_threshold = LaunchConfiguration("risk_exit_threshold")
    avoidance_sway = LaunchConfiguration("avoidance_sway")
    avoidance_yaw_rate = LaunchConfiguration("avoidance_yaw_rate")
    min_avoidance_hold_s = LaunchConfiguration("min_avoidance_hold_s")

    fake_detector_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_perception"), "config", "fake_detector.yaml"]
    )
    planner_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_avoidance"), "config", "local_avoidance_planner.yaml"]
    )
    demo_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_bringup"), "config", "demo.yaml"]
    )

    planner_node = Node(
        package="rov_obstacle_avoidance",
        executable="local_avoidance_planner_node",
        name="local_avoidance_planner",
        output="screen",
        parameters=[
            planner_config,
            {
                "risk_enter_threshold": ParameterValue(
                    risk_enter_threshold, value_type=float
                ),
                "risk_exit_threshold": ParameterValue(
                    risk_exit_threshold, value_type=float
                ),
                "avoidance_sway": ParameterValue(avoidance_sway, value_type=float),
                "avoidance_yaw_rate": ParameterValue(
                    avoidance_yaw_rate, value_type=float
                ),
                "min_avoidance_hold_s": ParameterValue(
                    min_avoidance_hold_s, value_type=float
                ),
            },
        ],
    )
    fake_detector_node = Node(
        package="rov_obstacle_perception",
        executable="fake_obstacle_detector_node",
        name="fake_obstacle_detector",
        output="screen",
        parameters=[fake_detector_config, {"scenario_mode": scenario_mode}],
    )
    nominal_command_node = Node(
        package="rov_obstacle_bringup",
        executable="nominal_cmd_publisher_node",
        name="nominal_cmd_publisher",
        output="screen",
        parameters=[demo_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_mode",
                default_value="central_static",
                description="Fake obstacle scenario mode for the demo.",
            ),
            DeclareLaunchArgument(
                "risk_enter_threshold",
                default_value="0.55",
                description="Risk threshold that enters avoidance.",
            ),
            DeclareLaunchArgument(
                "risk_exit_threshold",
                default_value="0.30",
                description="Risk threshold that starts recovery.",
            ),
            DeclareLaunchArgument(
                "avoidance_sway",
                default_value="0.20",
                description="Absolute sway command used while avoiding.",
            ),
            DeclareLaunchArgument(
                "avoidance_yaw_rate",
                default_value="0.35",
                description="Absolute yaw-rate command used while avoiding.",
            ),
            DeclareLaunchArgument(
                "min_avoidance_hold_s",
                default_value="1.0",
                description="Minimum seconds to hold the selected avoidance side.",
            ),
            planner_node,
            TimerAction(
                period=1.0,
                actions=[fake_detector_node, nominal_command_node],
            ),
        ]
    )
