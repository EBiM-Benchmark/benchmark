#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

TELEOPERATION_ROOT=/workspace/teleoperation
INSTALL_ROOT=/tmp/task1_teleop_install
BUILD_ROOT=/tmp/task1_teleop_build
LOG_ROOT=/tmp/task1_teleop_log
GELLO_CONFIG_FILE="${GELLO_CONFIG_FILE:-franka_gello_duo.yaml}"

if [[ ! -f "${TELEOPERATION_ROOT}/src/franka_gello_state_publisher/package.xml" ]]; then
  echo "teleoperation repository is not mounted at ${TELEOPERATION_ROOT}" >&2
  echo "Set TELEOPERATION_ROOT in task1_isaacsim/.env to its host path." >&2
  exit 1
fi

cd "${TELEOPERATION_ROOT}"
colcon --log-base "${LOG_ROOT}" build \
  --build-base "${BUILD_ROOT}" \
  --install-base "${INSTALL_ROOT}" \
  --symlink-install \
  --packages-select franka_gello_state_publisher pedal_state_publisher
source "${INSTALL_ROOT}/setup.bash"

python3 /workspace/task1_isaacsim/scripts/adapters/gello_to_bridge.py &
gello_bridge_pid=$!

ros2 launch franka_gello_state_publisher main.launch.py config_file:="${GELLO_CONFIG_FILE}" &
gello_publisher_pid=$!

echo "GELLO teleoperation is running in task1_gello_pedal_teleop."
echo "Start the pedal publisher from an interactive terminal with:"
echo "  docker exec -it task1_gello_pedal_teleop bash -lc 'source /opt/ros/jazzy/setup.bash && source ${INSTALL_ROOT}/setup.bash && ros2 run pedal_state_publisher pedal_state_publisher'"

cleanup() {
  kill "${gello_bridge_pid}" "${gello_publisher_pid}" 2>/dev/null || true
  wait "${gello_bridge_pid}" "${gello_publisher_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n "${gello_bridge_pid}" "${gello_publisher_pid}"
