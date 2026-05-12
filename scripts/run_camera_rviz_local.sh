#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_RVIZ="${START_RVIZ:-true}"
BUILD_ON_START="${BUILD_ON_START:-true}"

usage() {
  cat <<EOF
Usage:
  ./scripts/$(basename "$0")

Run this directly on the target machine, for example on Orin:
  cd /home/nv/projects/bi_camera_ws
  ./scripts/$(basename "$0")

Environment overrides:
  START_RVIZ=true|false
  BUILD_ON_START=true|false
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

cd "${WS_DIR}"

# ROS / ament setup scripts on this Orin image can read unset trace vars.
# Source them before enabling nounset to avoid "AMENT_TRACE_SETUP_FILES: unbound variable".
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash

if [[ "${BUILD_ON_START}" == "true" ]]; then
  colcon build --symlink-install \
    --packages-select mono_camera_cpp_ros2 bi_camera_cpp_ros2 camera_bringup_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_TESTING=OFF
fi

source install/setup.bash
set -u

exec ros2 launch camera_bringup_ros2 dual_usb_cameras_stable.launch.py \
  start_rviz:="${START_RVIZ}"
