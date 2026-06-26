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
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
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
            "holoocean_obstacle_oracle_node=rov_obstacle_sim_bridge.holoocean_obstacle_oracle_node:main",
            "simulated_rover_pose_publisher_node=rov_obstacle_sim_bridge.simulated_rover_pose_publisher_node:main",
            "cmd_vel_safe_logger_node=rov_obstacle_sim_bridge.cmd_vel_safe_logger_node:main",
        ],
    },
)
