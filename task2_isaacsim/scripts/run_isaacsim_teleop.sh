#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

# --------------------------------------------------------------------------
# Task 2 launcher: mobile FR3 Duo teleoperation on Isaac Sim 5.1.0 (PhysX).
#
# Layout assumptions (see task2_isaacsim/README.md):
#   * The Isaac Sim 5.1.0 container (${ISAACSIM_CONTAINER}) is already running
#     with this repo bind-mounted at ${CONTAINER_REPO}
#     (default /workspace/EBiM_Challenge).
#   * Lightweight ROS 2 helper services are defined in
#     task2_isaacsim/docker-compose.yml and reuse the Task 1 scripts.
# All paths below are derived from the repository location; nothing is
# hardcoded to a specific machine.
# --------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK2_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TASK2_DIRNAME="$(basename "${TASK2_ROOT}")"
REPO_ROOT="$(cd "${TASK2_ROOT}/.." && pwd)"
REPO_NAME="$(basename "${REPO_ROOT}")"

ISAACSIM_CONTAINER="${ISAACSIM_CONTAINER:-isaac-sim-5-1-0-workshop}"
# Path at which this repo is mounted inside the Isaac Sim container.
CONTAINER_REPO="${CONTAINER_REPO:-/workspace/EBiM_Challenge}"
CONTAINER_TASK2="${CONTAINER_REPO}/${TASK2_DIRNAME}"

USD_PATH="${USD_PATH:-../task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd}"
OBJECTS_USD_PATH="${OBJECTS_USD_PATH:-../assets/task2_objects/task2_objects.usda}"
ROOM_USD_PATH="${ROOM_USD_PATH:-../assets/robot_room.usd}"
SCENE="${SCENE:-room}"
EMBODIMENT="fr3duo_mobile"
CONTROLLER_MODE="${CONTROLLER_MODE:-position}"
WITH_KEYBOARD_TELEOP=false
WITH_GELLO_TELEOP=false
WITH_ARM_KEYBOARD_TELEOP=false
WITH_BROWSER=true
WITH_REPUBLISHER=true
HEADLESS=false
EXTRA_BRIDGE_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  task2_isaacsim/scripts/run_isaacsim_teleop.sh [options]

Options:
  --scene SCENE              room|barebone (default: room)
                             barebone: robot + task2 objects on a ground plane
                             room:     full robot room scene + eval camera
                             (scene_room.py, --task task2)
  --usd-path PATH            Robot USD path relative to task2_isaacsim/ or absolute
  --objects-usd-path PATH    Task 2 objects USD path (barebone scene only)
  --room-usd-path PATH       Room USD path (room scene only)
  --embodiment NAME          Embodiment config key (default: fr3duo_mobile)
  --controller-mode MODE     none|position (default: position)
  --with-keyboard-teleop     Start the keyboard->base teleop adapter (default input)
  --with-gello-teleop        Start the GELLO->bridge teleop adapter
  --with-gello-pedal-teleop  Alias of --with-gello-teleop (tested pedal+GELLO path)
  --with-arm-keyboard-teleop Drive both arm end effectors from the Isaac Sim
                             window keyboard via dual RMPflow. LEFT arm:
                             W/S A/D Q/E move, Z/X T/G C/V rotate, F gripper.
                             RIGHT arm: O/L K/; I/P move, N/M U/J ,/. rotate,
                             ' gripper. R resets both targets. While active,
                             ROS arm/gripper commands (browser/GELLO) are NOT
                             applied. (Equivalent to passing
                             '-- --arm-keyboard-teleop'.)
  --no-browser               Do not start browser_controller
  --no-republisher           Do not start ros_republisher
  --headless                 Run Isaac Sim without a visible Kit window
  --                         Pass remaining args to the scene script (scene_room.py | scene_barebone.py)

The teleop input *device* publishers (keyboard / GELLO / pedal) come from the
EBiM `teleoperation` repository running on the host; see task2_isaacsim/README.md.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --usd-path)
      USD_PATH="$2"
      shift 2
      ;;
    --objects-usd-path)
      OBJECTS_USD_PATH="$2"
      shift 2
      ;;
    --room-usd-path)
      ROOM_USD_PATH="$2"
      shift 2
      ;;
    --scene)
      SCENE="$2"
      shift 2
      ;;
    --embodiment)
      EMBODIMENT="$2"
      shift 2
      ;;
    --controller-mode)
      CONTROLLER_MODE="$2"
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
    --with-arm-keyboard-teleop)
      WITH_ARM_KEYBOARD_TELEOP=true
      shift
      ;;
    --no-browser)
      WITH_BROWSER=false
      shift
      ;;
    --no-republisher)
      WITH_REPUBLISHER=false
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
      EXTRA_BRIDGE_ARGS+=("$@")
      break
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

resolve_host_path() {
  local value="$1"
  if [[ "${value}" = /* ]]; then
    echo "${value}"
  else
    echo "$(cd "${TASK2_ROOT}" && cd "$(dirname "${value}")" && pwd)/$(basename "${value}")"
  fi
}

case "${SCENE}" in
  barebone|room) ;;
  *)
    echo "--scene must be 'barebone' or 'room'" >&2
    exit 2
    ;;
esac

HOST_USD="$(resolve_host_path "${USD_PATH}")"
HOST_OBJECTS_USD="$(resolve_host_path "${OBJECTS_USD_PATH}")"
HOST_ROOM_USD="$(resolve_host_path "${ROOM_USD_PATH}")"

if [[ ! -f "${HOST_USD}" ]]; then
  cat >&2 <<EOF
Robot USD file not found: ${HOST_USD}
Download it with task1_isaacsim/scripts/download_large_assets.sh.
EOF
  exit 1
fi

if [[ "${SCENE}" == "barebone" && ! -f "${HOST_OBJECTS_USD}" ]]; then
  echo "Objects USD file not found: ${HOST_OBJECTS_USD}" >&2
  exit 1
fi

if [[ "${SCENE}" == "room" && ! -f "${HOST_ROOM_USD}" ]]; then
  echo "Room USD file not found: ${HOST_ROOM_USD}" >&2
  exit 1
fi

case "${CONTROLLER_MODE}" in
  none|position) ;;
  *)
    echo "--controller-mode must be 'none' or 'position'" >&2
    exit 2
    ;;
esac

# Build the set of teleop adapters to launch inside the teleop_adapters service.
TELEOP_ADAPTERS=""
${WITH_KEYBOARD_TELEOP} && TELEOP_ADAPTERS="${TELEOP_ADAPTERS} keyboard"
${WITH_GELLO_TELEOP} && TELEOP_ADAPTERS="${TELEOP_ADAPTERS} gello"
TELEOP_ADAPTERS="$(echo "${TELEOP_ADAPTERS}" | xargs || true)"

echo "Isaac Sim container: ${ISAACSIM_CONTAINER}"
echo "Repo mount:          ${REPO_ROOT} -> ${CONTAINER_REPO}"
echo "Scene:               ${SCENE}"
echo "Robot USD:           ${HOST_USD}"
if [[ "${SCENE}" == "room" ]]; then
  echo "Room USD:            ${HOST_ROOM_USD}"
else
  echo "Objects USD:         ${HOST_OBJECTS_USD}"
fi
echo "Embodiment:          ${EMBODIMENT}"
echo "Controller mode:     ${CONTROLLER_MODE}"
echo "Teleop adapters:     ${TELEOP_ADAPTERS:-<none>}"
echo "Arm keyboard teleop: ${WITH_ARM_KEYBOARD_TELEOP}"

if ! docker ps --format '{{.Names}}' | grep -qx "${ISAACSIM_CONTAINER}"; then
  cat >&2 <<EOF
Isaac Sim container '${ISAACSIM_CONTAINER}' is not running.
Start it first (it must bind-mount this repo at ${CONTAINER_REPO}), or set
ISAACSIM_CONTAINER to the name of your running Isaac Sim 5.1.0 container.
EOF
  exit 1
fi

if ! docker exec "${ISAACSIM_CONTAINER}" test -d "${CONTAINER_REPO}"; then
  echo "The Isaac Sim container does not have this repository mounted at ${CONTAINER_REPO}." >&2
  exit 1
fi

HELPER_ARGS=("--controller-mode" "${CONTROLLER_MODE}")
${WITH_KEYBOARD_TELEOP} && HELPER_ARGS+=("--with-keyboard-teleop")
${WITH_GELLO_TELEOP} && HELPER_ARGS+=("--with-gello-teleop")
${WITH_BROWSER} || HELPER_ARGS+=("--no-browser")
${WITH_REPUBLISHER} || HELPER_ARGS+=("--no-republisher")
"${SCRIPT_DIR}/run_helper_containers.sh" up "${HELPER_ARGS[@]}"

if [[ "${HOST_USD}" != "${REPO_ROOT}/"* ]]; then
  echo "Robot USD must be inside ${REPO_ROOT} so the Isaac Sim container can see it." >&2
  exit 1
fi

CONTAINER_USD="${CONTAINER_REPO}/${HOST_USD#"${REPO_ROOT}/"}"
if [[ "${SCENE}" == "room" ]]; then
  if [[ "${HOST_ROOM_USD}" != "${REPO_ROOT}/"* ]]; then
    echo "Room USD must be inside ${REPO_ROOT} so the Isaac Sim container can see it." >&2
    exit 1
  fi
  CONTAINER_ROOM_USD="${CONTAINER_REPO}/${HOST_ROOM_USD#"${REPO_ROOT}/"}"
  BRIDGE_SCRIPT="scene_room.py"
  BRIDGE_ARGS=(
    "--robot-usd" "${CONTAINER_USD}"
    "--room-usd" "${CONTAINER_ROOM_USD}"
    "--task" "task2"
    "--embodiment" "${EMBODIMENT}"
    "--franka-root" "${CONTAINER_REPO}/task1_isaacsim"
  )
else
  if [[ "${HOST_OBJECTS_USD}" != "${REPO_ROOT}/"* ]]; then
    echo "Objects USD must be inside ${REPO_ROOT} so the Isaac Sim container can see it." >&2
    exit 1
  fi
  CONTAINER_OBJECTS_USD="${CONTAINER_REPO}/${HOST_OBJECTS_USD#"${REPO_ROOT}/"}"
  BRIDGE_SCRIPT="scene_barebone.py"
  BRIDGE_ARGS=(
    "--usd-path" "${CONTAINER_USD}"
    "--objects-usd-path" "${CONTAINER_OBJECTS_USD}"
    "--embodiment" "${EMBODIMENT}"
    "--franka-root" "${CONTAINER_REPO}/task1_isaacsim"
  )
fi

if ! ${WITH_BROWSER}; then
  BRIDGE_ARGS+=("--disable-browser-command-topics")
fi

if ${WITH_ARM_KEYBOARD_TELEOP}; then
  BRIDGE_ARGS+=("--arm-keyboard-teleop")
fi

if ${HEADLESS}; then
  BRIDGE_ARGS+=("--headless")
fi

BRIDGE_ARGS+=("${EXTRA_BRIDGE_ARGS[@]}")

echo "Launching Isaac Sim 5.1.0 teleop bridge..."
DOCKER_EXEC_ENV=()
if [[ -n "${DISPLAY:-}" ]]; then
  DOCKER_EXEC_ENV+=("-e" "DISPLAY=${DISPLAY}")
fi
if [[ -n "${TERM:-}" ]]; then
  DOCKER_EXEC_ENV+=("-e" "TERM=${TERM}")
fi
DOCKER_EXEC_ENV+=("-e" "QT_X11_NO_MITSHM=1")
# Use the ROS 2 jazzy libraries bundled with Isaac Sim's ros2 bridge extension
# (the container has no system ROS 2). LD_LIBRARY_PATH must be set before the
# process starts or rclpy node creation fails.
DOCKER_EXEC_ENV+=(
  "-e" "ROS_DISTRO=jazzy"
  "-e" "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  "-e" "FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
  "-e" "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib"
  "-e" "ROS_HOME=/tmp/isaac_ros_home"
)
docker exec -it "${DOCKER_EXEC_ENV[@]}" "${ISAACSIM_CONTAINER}" \
  /isaac-sim/python.sh "${CONTAINER_TASK2}/scripts/${BRIDGE_SCRIPT}" "${BRIDGE_ARGS[@]}"
