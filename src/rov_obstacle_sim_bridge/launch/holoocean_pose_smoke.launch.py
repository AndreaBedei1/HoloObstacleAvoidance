"""HoloOcean pose smoke bridge: full pipeline with HoloOcean (or fake) pose source.

Replaces the simulated_rover_pose_publisher with holoocean_pose_bridge.
When HoloOcean is unavailable the bridge falls back to a deterministic
fake pose so the entire pipeline still runs for smoke testing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_holoocean = LaunchConfiguration("use_holoocean")
    fallback_to_fake_pose = LaunchConfiguration("fallback_to_fake_pose")
    scenario_name = LaunchConfiguration("scenario_name")
    output_csv = LaunchConfiguration("output_csv")
    duration_s = LaunchConfiguration("duration_s")
    auto_shutdown = LaunchConfiguration("auto_shutdown")
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
            # --- HoloOcean-specific launch args ---
            DeclareLaunchArgument(
                "use_holoocean",
                default_value="true",
                description="Attempt to use HoloOcean for pose reading.",
            ),
            DeclareLaunchArgument(
                "fallback_to_fake_pose",
                default_value="true",
                description="Fall back to fake pose when HoloOcean fails.",
            ),
            DeclareLaunchArgument(
                "scenario_name",
                default_value="OpenWater-Hovering",
                description="HoloOcean scenario name.",
            ),
            # --- Shared launch args ---
            DeclareLaunchArgument(
                "obstacle_config",
                default_value=obstacles_simple,
                description="Path to the obstacle YAML configuration file.",
            ),
            DeclareLaunchArgument(
                "output_csv",
                default_value="logs/holoocean_smoke_record.csv",
                description="Output CSV path for the recorder node.",
            ),
            DeclareLaunchArgument(
                "duration_s",
                default_value="20.0",
                description="Recording duration in seconds (used with auto_shutdown).",
            ),
            DeclareLaunchArgument(
                "auto_shutdown",
                default_value="false",
                description="Automatically shut down after duration_s.",
            ),
            # --- Pose bridge (HoloOcean or fake fallback) ---
            Node(
                package="rov_obstacle_sim_bridge",
                executable="holoocean_pose_bridge_node",
                name="holoocean_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "use_holoocean": use_holoocean,
                        "fallback_to_fake_pose": fallback_to_fake_pose,
                        "scenario_name": scenario_name,
                    }
                ],
            ),
            # Oracle obstacle projector
            Node(
                package="rov_obstacle_sim_bridge",
                executable="holoocean_obstacle_oracle_node",
                name="holoocean_obstacle_oracle",
                output="screen",
                parameters=[oracle_config, {"obstacle_config_file": obstacle_config}],
            ),
            # Nominal command publisher
            Node(
                package="rov_obstacle_avoidance",
                executable="nominal_cmd_publisher_node",
                name="nominal_cmd_publisher",
                output="screen",
                parameters=[nominal_config],
            ),
            # Local avoidance planner
            Node(
                package="rov_obstacle_avoidance",
                executable="local_avoidance_planner_node",
                name="local_avoidance_planner",
                output="screen",
                parameters=[planner_config],
            ),
            # Passive CSV recorder
            Node(
                package="rov_obstacle_sim_bridge",
                executable="oracle_demo_recorder_node",
                name="oracle_demo_recorder",
                output="screen",
                parameters=[
                    {
                        "output_csv": output_csv,
                        "duration_s": duration_s,
                        "auto_shutdown": auto_shutdown,
                    }
                ],
            ),
        ]
    )
