import os
import subprocess
import sys
from glob import glob

from setuptools import Command, find_packages, setup

package_name = "rov_real_bridge"


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
        # Without these two entries a launch file or YAML placed in the
        # package is silently never installed (verified: the package had
        # no such entries).
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Real BlueROV2 adapter with fail-closed safety interlock (shadow-first).",
    license="MIT",
    tests_require=["pytest"],
    cmdclass={"test": PyTestCommand},
    entry_points={
        "console_scripts": [
            "real_control_adapter_node = "
            "rov_real_bridge.real_control_adapter_node:main",
            "real_control_live_node = "
            "rov_real_bridge.real_control_live_node:main",
            "real_camera_node = "
            "rov_real_bridge.real_camera_node:main",
            "real_detector_node = "
            "rov_real_bridge.real_detector_node:main",
        ],
    },
)
