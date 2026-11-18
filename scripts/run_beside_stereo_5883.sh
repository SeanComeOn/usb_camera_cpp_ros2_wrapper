#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONFIG_PATH="${CONFIG_PATH:-${WS_DIR}/src/camera_bringup_ros2/config/beside_stereo_5883.yaml}"
BUILD_ON_START="${BUILD_ON_START:-false}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "Camera config not found: ${CONFIG_PATH}" >&2
  exit 1
fi

cd "${WS_DIR}"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash

if [[ "${BUILD_ON_START}" == "true" || ! -f install/setup.bash ]]; then
  colcon build --symlink-install \
    --packages-select bi_camera_cpp_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_TESTING=OFF
fi

source install/setup.bash
set -u

exec ros2 run bi_camera_cpp_ros2 stereo_camera_node --ros-args \
  -p config_path:="${CONFIG_PATH}"
