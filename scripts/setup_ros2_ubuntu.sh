#!/usr/bin/env bash
set -euo pipefail

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
else
  echo "[FAIL] /etc/os-release not found."
  exit 1
fi

case "${VERSION_ID:-}" in
  "24.04")
    ROS_DISTRO="${ROS_DISTRO:-jazzy}"
    ;;
  "22.04")
    ROS_DISTRO="${ROS_DISTRO:-humble}"
    ;;
  *)
    echo "[FAIL] Unsupported Ubuntu version: ${VERSION_ID:-unknown}"
    exit 1
    ;;
esac

sudo apt update
sudo apt install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y "ros-${ROS_DISTRO}-ros-base" ros-dev-tools python3-colcon-common-extensions

echo "ROS 2 ${ROS_DISTRO} installed. Source it with:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"

