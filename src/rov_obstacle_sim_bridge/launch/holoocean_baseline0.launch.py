"""Scientific Baseline 0 closed loop (simulation dynamics integration baseline).

Pipeline::

    holoocean_sim_server (ocean env, motion_model: dynamics, BlueROV2 Heavy)
        --TCP--> holoocean_bridge_node
        bridge -> /perception/obstacles_oracle  (SIM-ONLY oracle projection)
        bridge -> /rov/pose_ground_truth        (VALIDATOR-ONLY)
        bridge -> /rov/attitude_measured /rov/depth (runtime-allowed measurements)
        bridge -> /sim/dynamics_debug /sim/obstacles_world (VALIDATOR-ONLY)
    oracle_dropout_relay: obstacles_oracle -> /perception/obstacles
        (single perception publisher; optional deterministic dropout)
    commanded_odometry_node (NO-DVL transferable dead reckoning):
        /planner/cmd_vel_safe + /rov/attitude_measured + /rov/depth
        -> /rov/odom_estimated                  (planner's ONLY pose input)
    nominal_cmd_publisher -> /cmd_vel_nominal
    local_avoidance_planner -> /planner/cmd_vel_safe -> bridge -> sim server
    baseline0_validator (consumes GT + debug; control path never does)

NOTE: the legacy odometry_estimator_node (synthetic DVL integration) is NOT
launched here — the transferable estimator replaces it.

Launch args: host, port, dropout_enabled, dropout_delay_s,
dropout_duration_s, validator_output, label.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_sim_bridge"), "config",
         "holoocean_bridge.yaml"])
    planner_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_avoidance"), "config",
         "local_avoidance_planner.yaml"])
    nominal_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_bringup"), "config", "demo.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="47654"),
        DeclareLaunchArgument("dropout_enabled", default_value="false"),
        DeclareLaunchArgument("dropout_delay_s", default_value="3.0"),
        DeclareLaunchArgument("dropout_duration_s", default_value="2.0"),
        DeclareLaunchArgument("dropout_mode", default_value="single"),
        DeclareLaunchArgument("outlier_at_s", default_value="0.0"),
        # Temporal estimator between the relay and the planner (Phase 7).
        DeclareLaunchArgument("estimator_method", default_value="t0"),
        DeclareLaunchArgument("noise_model_path", default_value=""),
        DeclareLaunchArgument("validator_output",
                              default_value="logs/baseline0_validation.json"),
        DeclareLaunchArgument("label", default_value="baseline0"),

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
                    # Perception relay is handled by oracle_dropout_relay so
                    # /perception/obstacles has exactly one publisher.
                    "relay_oracle_topic": "",
                },
            ],
        ),
        Node(
            package="rov_obstacle_sim_bridge",
            executable="oracle_dropout_relay_node",
            name="oracle_dropout_relay",
            output="screen",
            parameters=[{
                "dropout_enabled": LaunchConfiguration("dropout_enabled"),
                "dropout_delay_s": LaunchConfiguration("dropout_delay_s"),
                "dropout_duration_s": LaunchConfiguration("dropout_duration_s"),
                "dropout_mode": LaunchConfiguration("dropout_mode"),
                "outlier_at_s": LaunchConfiguration("outlier_at_s"),
                # Relay feeds the temporal estimator, not the planner.
                "output_topic": "/perception/obstacles_raw",
            }],
        ),
        Node(
            package="rov_obstacle_tracking",
            executable="temporal_estimator_node",
            name="temporal_estimator",
            output="screen",
            parameters=[{
                "method": LaunchConfiguration("estimator_method"),
                "noise_model_path": LaunchConfiguration("noise_model_path"),
            }],
        ),
        Node(
            package="rov_obstacle_sim_bridge",
            executable="commanded_odometry_node",
            name="commanded_odometry",
            output="screen",
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
        Node(
            package="rov_obstacle_sim_bridge",
            executable="baseline0_validator_node",
            name="baseline0_validator",
            output="screen",
            parameters=[{
                "output_path": LaunchConfiguration("validator_output"),
                "label": LaunchConfiguration("label"),
            }],
        ),
    ])
