#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

START_RVIZ="${START_RVIZ:-true}"
START_WATCHDOG="${START_WATCHDOG:-true}"
BUILD_ON_START="${BUILD_ON_START:-true}"
MONO_CONFIG="${MONO_CONFIG:-}"
STEREO_CONFIG="${STEREO_CONFIG:-}"
WRIST_STEREO_CONFIG="${WRIST_STEREO_CONFIG:-}"
WRIST_STEREO_ENABLED="${WRIST_STEREO_ENABLED:-false}"
WRIST_USB_IDENTITY="${WRIST_USB_IDENTITY:-}"
STEREO_USB_IDENTITY="${STEREO_USB_IDENTITY:-}"
WRIST_USB_DEVICE="${WRIST_USB_DEVICE:-}"
STEREO_USB_DEVICE="${STEREO_USB_DEVICE:-}"
WRIST_STEREO_USB_IDENTITY="${WRIST_STEREO_USB_IDENTITY:-}"
WRIST_STEREO_USB_DEVICE="${WRIST_STEREO_USB_DEVICE:-}"

usage() {
  cat <<EOF
Usage:
  ./scripts/$(basename "$0")

Run this directly on the target machine, for example on Orin:
  cd /home/nv/projects/bi_camera_ws
  ./scripts/$(basename "$0")

Environment overrides:
  START_RVIZ=true|false
  START_WATCHDOG=true|false
  BUILD_ON_START=true|false
  MONO_CONFIG=/absolute/path/to/mono.yaml
  STEREO_CONFIG=/absolute/path/to/stereo.yaml
  WRIST_STEREO_CONFIG=/absolute/path/to/wrist_stereo.yaml
  WRIST_STEREO_ENABLED=true|false
  WRIST_USB_IDENTITY=vendor:product
  STEREO_USB_IDENTITY=vendor:product
  WRIST_USB_DEVICE=/dev/v4l/by-path/...
  STEREO_USB_DEVICE=/dev/v4l/by-path/...
  WRIST_STEREO_USB_IDENTITY=vendor:product
  WRIST_STEREO_USB_DEVICE=/dev/v4l/by-path/...
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

launch_args=(
  start_rviz:="${START_RVIZ}"
  start_watchdog:="${START_WATCHDOG}"
  wrist_stereo_enabled:="${WRIST_STEREO_ENABLED}"
)
if [[ -n "${MONO_CONFIG}" ]]; then
  launch_args+=(mono_config:="${MONO_CONFIG}")
fi
if [[ -n "${STEREO_CONFIG}" ]]; then
  launch_args+=(stereo_config:="${STEREO_CONFIG}")
fi
if [[ -n "${WRIST_STEREO_CONFIG}" ]]; then
  launch_args+=(wrist_stereo_config:="${WRIST_STEREO_CONFIG}")
fi
if [[ -n "${WRIST_USB_IDENTITY}" ]]; then
  launch_args+=(wrist_usb_identity:="${WRIST_USB_IDENTITY}")
fi
if [[ -n "${STEREO_USB_IDENTITY}" ]]; then
  launch_args+=(stereo_usb_identity:="${STEREO_USB_IDENTITY}")
fi
if [[ -n "${WRIST_USB_DEVICE}" ]]; then
  launch_args+=(wrist_usb_device:="${WRIST_USB_DEVICE}")
fi
if [[ -n "${STEREO_USB_DEVICE}" ]]; then
  launch_args+=(stereo_usb_device:="${STEREO_USB_DEVICE}")
fi
if [[ -n "${WRIST_STEREO_USB_IDENTITY}" ]]; then
  launch_args+=(wrist_stereo_usb_identity:="${WRIST_STEREO_USB_IDENTITY}")
fi
if [[ -n "${WRIST_STEREO_USB_DEVICE}" ]]; then
  launch_args+=(wrist_stereo_usb_device:="${WRIST_STEREO_USB_DEVICE}")
fi

exec ros2 launch camera_bringup_ros2 dual_usb_cameras_stable.launch.py \
  "${launch_args[@]}"
