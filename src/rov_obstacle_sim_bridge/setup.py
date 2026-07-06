from glob import glob
import os
import subprocess
import sys

from setuptools import Command, find_packages, setup


package_name = "rov_obstacle_sim_bridge"


class PyTestCommand(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        raise SystemExit(subprocess.call([sys.executable, "-m", "pytest", "test"]))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "holoocean_scenarios"),
            glob("config/holoocean_scenarios/*.yaml"),
        ),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (
            os.path.join("share", package_name, "holoocean_server"),
            glob("holoocean_server/*.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrea",
    maintainer_email="andrea@example.com",
    description="Pure-Python simulation oracle geometry for converting world obstacle positions into camera-space detections.",
    license="MIT",
    cmdclass={"test": PyTestCommand},
    entry_points={
        "console_scripts": [
            "holoocean_bridge_node=rov_obstacle_sim_bridge.holoocean_bridge_node:main",
            "holoocean_obstacle_oracle_node=rov_obstacle_sim_bridge.holoocean_obstacle_oracle_node:main",
            "holoocean_pose_bridge_node=rov_obstacle_sim_bridge.holoocean_pose_bridge_node:main",
            "simulated_rover_pose_publisher_node=rov_obstacle_sim_bridge.simulated_rover_pose_publisher_node:main",
            "cmd_vel_safe_logger_node=rov_obstacle_sim_bridge.cmd_vel_safe_logger_node:main",
            "oracle_demo_recorder_node=rov_obstacle_sim_bridge.oracle_demo_recorder_node:main",
        ],
    },
)
