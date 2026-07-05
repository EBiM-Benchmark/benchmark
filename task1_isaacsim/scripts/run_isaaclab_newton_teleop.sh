#!/usr/bin/env bash
# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

# --------------------------------------------------------------------------
# Task 1 launcher: mobile FR3 Duo teleoperation on Isaac Lab + Newton/MJWarp.
#
# Layout assumptions (see task1_isaacsim/README.md):
#   * This repo (benchmark) is bind-mounted into the Isaac Lab container at
#     ${CONTAINER_REPO} (default /workspace/EBiM_Challenge), configured by the
#     ros2_jazzy docker overlay in task1_isaacsim/isaaclab_overlay/.
#   * A Newton-enabled Isaac Lab checkout lives at ${ISAACLAB_ROOT}.
#   * Lightweight ROS 2 helper services are defined in task1_isaacsim/docker-compose.yml.
# All paths below are derived from the repository location; nothing is hardcoded
# to a specific machine.
# --------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK1_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TASK1_DIRNAME="$(basename "${TASK1_ROOT}")"
REPO_ROOT="$(cd "${TASK1_ROOT}/.." && pwd)"
REPO_NAME="$(basename "${REPO_ROOT}")"

# Newton-enabled Isaac Lab checkout (see task1_isaacsim/isaaclab_overlay/README.md).
# Defaults to a sibling checkout next to this repository.
ISAACLAB_ROOT="${ISAACLAB_ROOT:-$(cd "${REPO_ROOT}/.." && pwd)/IsaacLab}"
ISAACLAB_SERVICE="${ISAACLAB_SERVICE:-ros2_jazzy}"
ISAACLAB_CONTAINER="${ISAACLAB_CONTAINER:-isaac-lab-ros2_jazzy}"
ISAACLAB_CONTAINER_WS="${ISAACLAB_CONTAINER_WS:-/workspace/isaaclab}"
# Path at which this repo is mounted inside the Isaac Lab container.
CONTAINER_REPO="${CONTAINER_REPO:-/workspace/${REPO_NAME}}"
CONTAINER_TASK1="${CONTAINER_REPO}/${TASK1_DIRNAME}"

EMBODIMENT="${EMBODIMENT:-fr3duo_mobile}"
USD_PATH="${USD_PATH:-assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd}"
CONTROLLER_MODE="${CONTROLLER_MODE:-position}"
WITH_KEYBOARD_TELEOP=false
WITH_GELLO_TELEOP=false
WITH_BROWSER=true
WITH_REPUBLISHER=true
WITH_CABLE=false
HEADLESS=false
CABLE_DEVICE="${CABLE_DEVICE:-cuda:0}"
CABLE_CONFIG_PATH="${CABLE_CONFIG_PATH:-${CONTAINER_TASK1}/cable_world/configs/table_board_fixture_cable.yaml}"
CABLE_GRIPPER_CONFIG_PATH="${CABLE_GRIPPER_CONFIG_PATH:-${CONTAINER_TASK1}/cable_world/configs/gripper.yaml}"
CABLE_LOG_PATH="${CABLE_LOG_PATH:-/tmp/task1_cable_vbd.log}"
EXTRA_BRIDGE_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh [options]

Options:
  --embodiment NAME          Embodiment config key (default: fr3duo_mobile)
  --usd-path PATH            USD path relative to task1_isaacsim/ or absolute
  --controller-mode MODE     none|position (default: position)
  --with-keyboard-teleop     Start the keyboard->base teleop adapter (default input)
  --with-gello-teleop        Start the GELLO->bridge teleop adapter
  --with-gello-pedal-teleop  Alias of --with-gello-teleop (tested pedal+GELLO path)
  --no-browser               Do not start browser_controller
  --no-republisher           Do not start ros_republisher
  --with-cable               Run the raw Newton VBD board-cable world
  --headless                 Run Isaac Lab without a visible Kit window
  --                         Pass remaining args to isaaclab_fr3duo_newton_bridge.py

The teleop input *device* publishers (keyboard / GELLO / pedal) come from the
EBiM `teleoperation` repository running on the host; see task1_isaacsim/README.md.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --embodiment)
      EMBODIMENT="$2"
      shift 2
      ;;
    --usd-path)
      USD_PATH="$2"
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
    --no-browser)
      WITH_BROWSER=false
      shift
      ;;
    --no-republisher)
      WITH_REPUBLISHER=false
      shift
      ;;
    --with-cable)
      WITH_CABLE=true
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

if [[ "${USD_PATH}" = /* ]]; then
  HOST_USD="${USD_PATH}"
else
  HOST_USD="${TASK1_ROOT}/${USD_PATH}"
fi

if [[ ! -f "${HOST_USD}" ]]; then
  echo "USD file not found: ${HOST_USD}" >&2
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

echo "Isaac Lab container: ${ISAACLAB_CONTAINER}"
echo "Isaac Lab checkout:  ${ISAACLAB_ROOT}"
echo "Repo mount:          ${REPO_ROOT} -> ${CONTAINER_REPO}"
echo "Embodiment:          ${EMBODIMENT}"
echo "USD:                 ${HOST_USD}"
echo "Controller mode:     ${CONTROLLER_MODE}"
echo "Teleop adapters:     ${TELEOP_ADAPTERS:-<none>}"
echo "Cable VBD:           ${WITH_CABLE}"

if [[ ! -d "${ISAACLAB_ROOT}" ]]; then
  cat >&2 <<EOF
Isaac Lab checkout not found at: ${ISAACLAB_ROOT}
Set ISAACLAB_ROOT to your Newton-enabled Isaac Lab checkout, or place it next to
this repository. See task1_isaacsim/isaaclab_overlay/README.md for setup.
EOF
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${ISAACLAB_CONTAINER}"; then
  echo "Starting Isaac Lab container via ${ISAACLAB_ROOT}/docker/container.py..."
  (cd "${ISAACLAB_ROOT}" && ./docker/container.py start "${ISAACLAB_SERVICE}")
fi

if ! docker exec "${ISAACLAB_CONTAINER}" test -d "${CONTAINER_REPO}"; then
  cat >&2 <<EOF
The Isaac Lab container does not have this repository mounted at ${CONTAINER_REPO}.

Apply the ros2_jazzy overlay so the compose file bind-mounts this repo:
  task1_isaacsim/isaaclab_overlay/apply_overlay.sh "${ISAACLAB_ROOT}" "${REPO_ROOT}"

Then recreate ${ISAACLAB_CONTAINER}.
EOF
  exit 1
fi

if ${WITH_CABLE}; then
  echo "Starting raw Newton cable VBD ROS process inside ${ISAACLAB_CONTAINER}..."
  docker exec "${ISAACLAB_CONTAINER}" bash -lc "pkill -f '[r]un_cable_vbd_ros_headless.py' || true"
  docker exec -d "${ISAACLAB_CONTAINER}" bash -lc "cd ${ISAACLAB_CONTAINER_WS} && source /opt/ros/jazzy/setup.bash && ./isaaclab.sh -p ${CONTAINER_TASK1}/scripts/run_cable_vbd_ros_headless.py --viewer null --device ${CABLE_DEVICE} --config-path ${CABLE_CONFIG_PATH} --gripper-config-path ${CABLE_GRIPPER_CONFIG_PATH} --gripper-pose-topic /isaac/left_gripper_pose --gripper-gap-topic /isaac/left_gripper_gap --cable-point-topic /cable/body_centers --num-frames 0 > ${CABLE_LOG_PATH} 2>&1"
  echo "Cable VBD log: docker exec ${ISAACLAB_CONTAINER} tail -f ${CABLE_LOG_PATH}"
fi

if ${WITH_REPUBLISHER}; then
  echo "Starting ros_republisher..."
  REPUBLISHER_ENV=()
  if ! ${WITH_BROWSER}; then
    REPUBLISHER_ENV+=("REPUBLISHER_DISABLE_BROWSER_COMMAND_TOPICS=true")
  fi
  (cd "${TASK1_ROOT}" && env "${REPUBLISHER_ENV[@]}" docker compose up -d --no-deps ros_republisher)
fi

if [[ "${CONTROLLER_MODE}" == "position" ]]; then
  echo "Starting position_controller..."
  (cd "${TASK1_ROOT}" && docker compose --profile position up -d --no-deps position_controller)
fi

if [[ -n "${TELEOP_ADAPTERS}" ]]; then
  echo "Starting teleop_adapters (${TELEOP_ADAPTERS})..."
  (cd "${TASK1_ROOT}" && env "TELEOP_ADAPTERS=${TELEOP_ADAPTERS}" docker compose --profile teleop up -d --no-deps teleop_adapters)
fi

if ${WITH_BROWSER}; then
  echo "Starting browser_controller..."
  (cd "${TASK1_ROOT}" && docker compose up -d --no-deps browser_controller)
  echo "Browser UI: http://localhost:8090"
fi

if [[ "${HOST_USD}" != "${TASK1_ROOT}/"* ]]; then
  echo "USD must be inside ${TASK1_ROOT} so the Isaac Lab container can see it." >&2
  exit 1
fi

CONTAINER_USD="${CONTAINER_TASK1}/${HOST_USD#"${TASK1_ROOT}/"}"
BRIDGE_ARGS=(
  "--usd-path" "${CONTAINER_USD}"
  "--embodiment" "${EMBODIMENT}"
  "--franka-root" "${CONTAINER_TASK1}"
)

if ${WITH_CABLE}; then
  BRIDGE_ARGS+=("--with-cable")
fi

if ! ${WITH_BROWSER}; then
  BRIDGE_ARGS+=("--disable-browser-command-topics")
fi

if ${HEADLESS}; then
  BRIDGE_ARGS+=("--headless")
else
  BRIDGE_ARGS+=("--visualizer" "kit")
fi

BRIDGE_ARGS+=("${EXTRA_BRIDGE_ARGS[@]}")

echo "Launching Isaac Lab Newton/MJWarp bridge..."
printf -v BRIDGE_ARGS_QUOTED " %q" "${BRIDGE_ARGS[@]}"
DOCKER_EXEC_ENV=()
if [[ -n "${DISPLAY:-}" ]]; then
  DOCKER_EXEC_ENV+=("-e" "DISPLAY=${DISPLAY}")
fi
if [[ -n "${TERM:-}" ]]; then
  DOCKER_EXEC_ENV+=("-e" "TERM=${TERM}")
fi
DOCKER_EXEC_ENV+=("-e" "QT_X11_NO_MITSHM=1")
docker exec -it "${DOCKER_EXEC_ENV[@]}" "${ISAACLAB_CONTAINER}" bash -lc \
  "cd ${ISAACLAB_CONTAINER_WS} && source /opt/ros/jazzy/setup.bash && ./isaaclab.sh -p ${CONTAINER_TASK1}/scripts/isaaclab_fr3duo_newton_bridge.py${BRIDGE_ARGS_QUOTED}"
