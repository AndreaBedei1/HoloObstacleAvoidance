from glob import glob
import os
import subprocess
import sys

from setuptools import Command, find_packages, setup


package_name = "rov_obstacle_bringup"


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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Andrea",
    maintainer_email="andrea@example.com",
    description="Launch files for the underwater ROV obstacle avoidance prototype.",
    license="MIT",
    cmdclass={"test": PyTestCommand},
)

