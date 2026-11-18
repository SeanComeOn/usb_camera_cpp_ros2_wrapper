#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_ON_START="${BUILD_ON_START:-}"
STEREO_LEFT_TOPIC="${STEREO_LEFT_TOPIC:-${THIRD_PERSON_TOPIC:-/stereo/left/image_raw}}"
STEREO_RIGHT_TOPIC="${STEREO_RIGHT_TOPIC:-/stereo/right/image_raw}"
WRIST_STEREO_LEFT_TOPIC="${WRIST_STEREO_LEFT_TOPIC:-/wrist/stereo/left/image_raw}"
WRIST_STEREO_RIGHT_TOPIC="${WRIST_STEREO_RIGHT_TOPIC:-/wrist/stereo/right/image_raw}"
WRIST_TOPIC="${WRIST_TOPIC:-/mono/image_raw}"
JOINT_STATES_TOPIC="${JOINT_STATES_TOPIC:-/joint_states}"
CONTROL_INPUT_TOPIC="${CONTROL_INPUT_TOPIC:-/fr3_web_teleop/input}"
BASE_FRAME="${BASE_FRAME:-fr3_link0}"
END_EFFECTOR_FRAME="${END_EFFECTOR_FRAME:-fr3_hand_tcp}"
VIDEO_OUTPUT_ROOT="${VIDEO_OUTPUT_ROOT:-${HOME}/data/camera_recordings}"
VIDEO_FPS="${VIDEO_FPS:-20}"
TRAJECTORY_HZ="${TRAJECTORY_HZ:-100}"
GRIPPER_MAX_WIDTH="${GRIPPER_MAX_WIDTH:-0.08}"
REQUIRE_TRAJECTORY="${REQUIRE_TRAJECTORY:-false}"
VIDEO_CODEC="${VIDEO_CODEC:-mp4v}"
IMAGE_QOS_RELIABILITY="${IMAGE_QOS_RELIABILITY:-best_effort}"
IMAGE_TRANSPORT="${IMAGE_TRANSPORT:-compressed}"
WRITE_COMBINED="${WRITE_COMBINED:-true}"
SESSION_PREFIX="${SESSION_PREFIX:-}"
AUTO_START="${AUTO_START:-false}"
RECORD_SECONDS="${RECORD_SECONDS:-0}"

usage() {
  cat <<EOF
Usage:
  ./scripts/$(basename "$0")

Run this directly on the target machine, for example on Orin:
  cd /home/nv/projects/bi_camera_ws
  ./scripts/$(basename "$0")

Keyboard controls:
  r  start recording
  p  pause or resume the current recording
  s  stop and save
  d  discard current recording
  q  quit

Outputs per session:
  stereo_left.mp4
  stereo_right.mp4
  wrist_stereo_left.mp4
  wrist_stereo_right.mp4
  wrist.mp4
  combined.mp4
  video_frames.csv
  trajectory.csv
  control_inputs.csv
  metadata.json

Environment overrides:
  BUILD_ON_START=true|false
  STEREO_LEFT_TOPIC=/stereo/left/image_raw
  STEREO_RIGHT_TOPIC=/stereo/right/image_raw
  WRIST_STEREO_LEFT_TOPIC=/wrist/stereo/left/image_raw
  WRIST_STEREO_RIGHT_TOPIC=/wrist/stereo/right/image_raw
  WRIST_TOPIC=/mono/image_raw
  JOINT_STATES_TOPIC=/joint_states
  CONTROL_INPUT_TOPIC=/fr3_web_teleop/input
  BASE_FRAME=fr3_link0
  END_EFFECTOR_FRAME=fr3_hand_tcp
  VIDEO_OUTPUT_ROOT=\$HOME/data/camera_recordings
  VIDEO_FPS=20
  TRAJECTORY_HZ=100
  GRIPPER_MAX_WIDTH=0.08
  REQUIRE_TRAJECTORY=true|false
  VIDEO_CODEC=mp4v
  IMAGE_QOS_RELIABILITY=best_effort|reliable
  IMAGE_TRANSPORT=compressed|raw
  WRITE_COMBINED=true|false
  SESSION_PREFIX=demo
  AUTO_START=true|false
  RECORD_SECONDS=0
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

cd "${WS_DIR}"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
source /opt/ros/humble/setup.bash

if [[ -z "${BUILD_ON_START}" ]]; then
  if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
    BUILD_ON_START=false
  else
    BUILD_ON_START=true
  fi
fi

if [[ "${BUILD_ON_START}" == "true" ]]; then
  colcon build --symlink-install \
    --packages-select mono_camera_cpp_ros2 bi_camera_cpp_ros2 camera_bringup_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_TESTING=OFF
fi

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  source "${WS_DIR}/install/setup.bash"
fi
set -u

args=(
  --stereo-left-topic "${STEREO_LEFT_TOPIC}"
  --stereo-right-topic "${STEREO_RIGHT_TOPIC}"
  --wrist-stereo-left-topic "${WRIST_STEREO_LEFT_TOPIC}"
  --wrist-stereo-right-topic "${WRIST_STEREO_RIGHT_TOPIC}"
  --wrist-topic "${WRIST_TOPIC}"
  --joint-states-topic "${JOINT_STATES_TOPIC}"
  --control-input-topic "${CONTROL_INPUT_TOPIC}"
  --base-frame "${BASE_FRAME}"
  --end-effector-frame "${END_EFFECTOR_FRAME}"
  --output-root "${VIDEO_OUTPUT_ROOT}"
  --fps "${VIDEO_FPS}"
  --trajectory-hz "${TRAJECTORY_HZ}"
  --gripper-max-width "${GRIPPER_MAX_WIDTH}"
  --codec "${VIDEO_CODEC}"
  --image-qos-reliability "${IMAGE_QOS_RELIABILITY}"
  --image-transport "${IMAGE_TRANSPORT}"
  --record-seconds "${RECORD_SECONDS}"
)

if [[ "${REQUIRE_TRAJECTORY}" == "true" ]]; then
  args+=(--require-trajectory)
else
  args+=(--no-require-trajectory)
fi

if [[ "${WRITE_COMBINED}" == "true" ]]; then
  args+=(--write-combined)
else
  args+=(--no-write-combined)
fi

if [[ -n "${SESSION_PREFIX}" ]]; then
  args+=(--session-prefix "${SESSION_PREFIX}")
fi

if [[ "${AUTO_START}" == "true" ]]; then
  args+=(--auto-start)
fi

exec python3 "${SCRIPT_DIR}/record_dual_camera_videos.py" "${args[@]}" "$@"
