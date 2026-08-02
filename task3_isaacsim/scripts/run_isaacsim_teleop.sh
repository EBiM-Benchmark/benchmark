#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${TASK3_ROOT}/.." && pwd)"
TASK3_DIRNAME="$(basename "${TASK3_ROOT}")"
if [[ -f "${TASK3_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${TASK3_ROOT}/.env"
  set +a
fi

ISAACSIM_CONTAINER="${ISAACSIM_CONTAINER:-isaac-sim-5-1-0-workshop}"
CONTAINER_REPO="${CONTAINER_REPO:-/workspace/EBiM_Challenge}"
CONTAINER_TASK3="${CONTAINER_REPO}/${TASK3_DIRNAME}"
GRIPPER="${GRIPPER_PROFILE:-robotiq}"
USD_PATH=""
ROOM_USD_PATH="${ROOM_USD_PATH:-../assets/robot_room.usd}"
HEAD_PLACEMENT="random"
CONTROLLER_MODE="${CONTROLLER_MODE:-position}"
WITH_KEYBOARD_TELEOP=false
WITH_GELLO_TELEOP=false
WITH_BROWSER=true
WITH_REPUBLISHER=true
DYNAMIC_BEANS=true
HEADLESS=false
EXTRA_ARGS=()

usage() {
  cat <<'EOF'
Usage: task3_isaacsim/scripts/run_isaacsim_teleop.sh [options]

Options:
  --gripper robotiq|panda    Robot/gripper profile (default: robotiq)
  --usd-path PATH            Override the profile's robot USD
  --room-usd-path PATH       Override the shared robot-room USD
  --head-placement A-I       Fixed head position, or random
  --with-keyboard-teleop     Start keyboard-to-base ROS adapter
  --with-gello-teleop        Start GELLO arm/gripper ROS adapter
  --with-gello-pedal-teleop  Alias of --with-gello-teleop
  --controller-mode MODE     none|position (default: position)
  --no-browser               Do not start browser controller
  --no-republisher           Do not start ROS joint republisher
  --no-dynamic-beans         Keep Task 3 beans static
  --headless                 Run without the Isaac Sim GUI
  --                         Pass remaining options to scene_room.py

The Robotiq profile uses the competition robot asset and requires the Task 1
large-asset download. The Panda profile uses third_party/franka_description.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gripper)
      GRIPPER="$2"
      shift 2
      ;;
    --usd-path)
      USD_PATH="$2"
      shift 2
      ;;
    --room-usd-path)
      ROOM_USD_PATH="$2"
      shift 2
      ;;
    --head-placement)
      HEAD_PLACEMENT="$2"
      shift 2
      ;;
    --with-keyboard-teleop)
      WITH_KEYBOARD_TELEOP=true
      shift
      ;;
    --with-gello-teleop|--with-gello-pedal-teleop)
      WITH_GELLO_TELEOP=true
      shift
      ;;
    --controller-mode)
      CONTROLLER_MODE="$2"
      shift 2
      ;;
    --no-browser)
      WITH_BROWSER=false
      shift
      ;;
    --no-republisher)
      WITH_REPUBLISHER=false
      shift
      ;;
    --no-dynamic-beans)
      DYNAMIC_BEANS=false
      shift
      ;;
    --headless)
      HEADLESS=true
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${GRIPPER}" in
  robotiq)
    DEFAULT_USD="../task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd"
    ;;
  panda)
    DEFAULT_USD="../third_party/franka_description/urdfs/mobile_fr3_duo_v0_2_franka_hand.usd"
    ;;
  *)
    echo "--gripper must be 'robotiq' or 'panda'" >&2
    exit 2
    ;;
esac
USD_PATH="${USD_PATH:-${DEFAULT_USD}}"

resolve_host_path() {
  local value="$1"
  if [[ "${value}" = /* ]]; then
    echo "${value}"
  else
    echo "$(cd "${TASK3_ROOT}" && cd "$(dirname "${value}")" && pwd)/$(basename "${value}")"
  fi
}

HOST_USD="$(resolve_host_path "${USD_PATH}")"
HOST_ROOM_USD="$(resolve_host_path "${ROOM_USD_PATH}")"
if [[ ! -f "${HOST_USD}" ]]; then
  echo "Robot USD not found: ${HOST_USD}" >&2
  if [[ "${GRIPPER}" == "robotiq" ]]; then
    echo "Run task1_isaacsim/scripts/download_large_assets.sh first." >&2
  fi
  exit 1
fi
if [[ ! -f "${HOST_ROOM_USD}" ]]; then
  echo "Room USD not found: ${HOST_ROOM_USD}" >&2
  exit 1
fi
for host_path in "${HOST_USD}" "${HOST_ROOM_USD}"; do
  if [[ "${host_path}" != "${REPO_ROOT}/"* ]]; then
    echo "Asset must be inside ${REPO_ROOT}: ${host_path}" >&2
    exit 1
  fi
done

if ! docker ps --format '{{.Names}}' | grep -qx "${ISAACSIM_CONTAINER}"; then
  echo "Isaac Sim container '${ISAACSIM_CONTAINER}' is not running." >&2
  exit 1
fi
if ! docker exec "${ISAACSIM_CONTAINER}" test -d "${CONTAINER_REPO}"; then
  echo "Repository is not mounted at ${CONTAINER_REPO} in the container." >&2
  exit 1
fi

HELPER_ARGS=("--gripper" "${GRIPPER}" "--controller-mode" "${CONTROLLER_MODE}")
${WITH_KEYBOARD_TELEOP} && HELPER_ARGS+=("--with-keyboard-teleop")
${WITH_GELLO_TELEOP} && HELPER_ARGS+=("--with-gello-teleop")
${WITH_BROWSER} || HELPER_ARGS+=("--no-browser")
${WITH_REPUBLISHER} || HELPER_ARGS+=("--no-republisher")
"${SCRIPT_DIR}/run_helper_containers.sh" up "${HELPER_ARGS[@]}"

CONTAINER_USD="${CONTAINER_REPO}/${HOST_USD#"${REPO_ROOT}/"}"
CONTAINER_ROOM_USD="${CONTAINER_REPO}/${HOST_ROOM_USD#"${REPO_ROOT}/"}"
SCENE_ARGS=(
  "--gripper" "${GRIPPER}"
  "--robot-usd" "${CONTAINER_USD}"
  "--room-usd" "${CONTAINER_ROOM_USD}"
  "--head-placement" "${HEAD_PLACEMENT}"
  "--franka-root" "${CONTAINER_REPO}/task1_isaacsim"
)
${WITH_BROWSER} || SCENE_ARGS+=("--disable-browser-command-topics")
${DYNAMIC_BEANS} || SCENE_ARGS+=("--no-dynamic-beans")
${HEADLESS} && SCENE_ARGS+=("--headless")
SCENE_ARGS+=("${EXTRA_ARGS[@]}")

echo "Launching Task 3: gripper=${GRIPPER} robot=${HOST_USD}"
DOCKER_EXEC_ENV=(
  "-e" "QT_X11_NO_MITSHM=1"
  "-e" "ROS_DISTRO=jazzy"
  "-e" "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  "-e" "FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
  "-e" "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib"
  "-e" "ROS_HOME=/tmp/isaac_ros_home"
)
[[ -n "${DISPLAY:-}" ]] && DOCKER_EXEC_ENV+=("-e" "DISPLAY=${DISPLAY}")
[[ -n "${TERM:-}" ]] && DOCKER_EXEC_ENV+=("-e" "TERM=${TERM}")
docker exec -it "${DOCKER_EXEC_ENV[@]}" "${ISAACSIM_CONTAINER}" \
  /isaac-sim/python.sh "${CONTAINER_TASK3}/scripts/scene_room.py" "${SCENE_ARGS[@]}"
