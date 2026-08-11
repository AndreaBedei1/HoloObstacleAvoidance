import subprocess
import sys

from setuptools import Command, find_packages, setup

package_name = "rov_obstacle_tracking"


class PyTestCommand(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        raise SystemExit(
            subprocess.call([sys.executable, "-m", "pytest", "test"]))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Temporal obstacle estimation T0-T3 (common interface, offline-replayable).",
    license="MIT",
    tests_require=["pytest"],
    cmdclass={"test": PyTestCommand},
    entry_points={
        "console_scripts": [
            "temporal_estimator_node = "
            "rov_obstacle_tracking.temporal_estimator_node:main",
        ],
    },
)
