#!/usr/bin/env bash
set -eo pipefail

TASK1_ROOT="${TASK1_ROOT:-/workspace/EBiM_Challenge/task1_isaacsim}"
OUTPUT_DIR="${1:-${TASK1_ROOT}/recordings/task1_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "${OUTPUT_DIR}")"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
fi

set -u

TOPICS=(
    /isaac/left_wrist_camera/image_raw
    /isaac/right_wrist_camera/image_raw
    /isaac/head_camera/image_raw
    /isaac/left_joint_states
    /isaac/right_joint_states
    /isaac/left_robotiq_joint_states
    /isaac/right_robotiq_joint_states
    /isaac/base_pose_relative
    /isaac/base_command
)

echo "Recording synchronized Task 1 data to: ${OUTPUT_DIR}"
echo "Stop cleanly with Ctrl+C."
printf '  %s\n' "${TOPICS[@]}"

exec ros2 bag record \
    --storage mcap \
    --storage-preset-profile zstd_fast \
    --max-cache-size 1073741824 \
    --output "${OUTPUT_DIR}" \
    --topics "${TOPICS[@]}"
