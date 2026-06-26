#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -r "${SETUP}" ]]; then
  echo "[FAIL] ROS 2 setup file not found: ${SETUP}"
  exit 1
fi

# shellcheck disable=SC1090
source "${SETUP}"

