"""Launch only the ROS 2 side of the HoloOcean bridge.

The HoloOcean sim server is a SEPARATE process that must be started in the conda
``ocean`` environment (Python 3.9), e.g.::

    conda run -n ocean python \
      src/rov_obstacle_sim_bridge/holoocean_server/holoocean_sim_server.py \
      --config <scenario.yaml> --serve

This launch file starts the ROS 2 bridge node, which connects to that server and
republishes the camera/pose/velocity/depth topics plus the simulation-only
oracle obstacle projection.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("rov_obstacle_sim_bridge"), "config", "holoocean_bridge.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1",
                                  description="Sim server host."),
            DeclareLaunchArgument("port", default_value="47654",
                                  description="Sim server TCP port."),
            DeclareLaunchArgument("oracle_topic",
                                  default_value="/perception/obstacles_oracle",
                                  description="Topic for SIM-ONLY oracle obstacles."),
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
                        "oracle_topic": LaunchConfiguration("oracle_topic"),
                    },
                ],
            ),
        ]
    )
