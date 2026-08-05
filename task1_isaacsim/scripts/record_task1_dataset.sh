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
    /isaac/left_wrist_camera/image_compressed
    /isaac/right_wrist_camera/image_compressed
    /isaac/head_camera/image_compressed
    /isaac/data_contract/base_state
    /isaac/data_contract/arm_state
    /isaac/data_contract/gripper_state
    /isaac/data_contract/action
    /isaac/data_contract/timestamp
    /isaac/data_contract/step_count
    /isaac/data_contract/metadata
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
