#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK1_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${TASK1_ROOT}/.." && pwd)"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-$(cd "${REPO_ROOT}/.." && pwd)/IsaacLab}"
ISAACLAB_SERVICE="${ISAACLAB_SERVICE:-ros2_jazzy}"
ISAACLAB_CONTAINER="${ISAACLAB_CONTAINER:-isaac-lab-ros2_jazzy}"
ISAACLAB_CONTAINER_WS="${ISAACLAB_CONTAINER_WS:-/workspace/isaaclab}"
CONTAINER_REPO="${CONTAINER_REPO:-/workspace/EBiM_Challenge}"
CONTAINER_TASK1="${CONTAINER_TASK1:-${CONTAINER_REPO}/task1_isaacsim}"
EMBODIMENT="${EMBODIMENT:-fr3duo_mobile}"
USD_PATH="${USD_PATH:-assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd}"
CONTROLLER_MODE="${CONTROLLER_MODE:-position}"
WITH_GELLO_PEDAL_TELEOP=false
WITH_KEYBOARD_TELEOP=false
WITH_BROWSER=true
WITH_REPUBLISHER=true
HEADLESS=false
CABLE_DEVICE="${CABLE_DEVICE:-cuda:0}"
CABLE_CONFIG_PATH="${CABLE_CONFIG_PATH:-${CONTAINER_TASK1}/cable_world/configs/table_board_fixture_cable.yaml}"
CABLE_GRIPPER_CONFIG_PATH="${CABLE_GRIPPER_CONFIG_PATH:-${CONTAINER_TASK1}/cable_world/configs/gripper.yaml}"
CABLE_LOG_PATH="${CABLE_LOG_PATH:-/tmp/task1_cable_vbd.log}"
EXTRA_BRIDGE_ARGS=()

usage() {
  cat <<'EOF'
Usage:
  scripts/run_isaaclab_newton_teleop.sh [options]

Options:
  --embodiment NAME          Embodiment config key (default: fr3duo_mobile)
  --usd-path PATH            USD path relative to repo root or absolute
  --controller-mode MODE     none|position (default: position)
  --with-gello-pedal-teleop  Start GELLO arm teleop and pedal base teleop together
  --with-keyboard-teleop     Control both arm TCPs/grippers from the Isaac Sim keyboard
  --no-browser               Do not start browser_controller
  --no-republisher           Do not start ros_republisher
  --headless                 Run IsaacLab without a visible Kit window
  --                         Pass remaining args to isaaclab_fr3duo_newton_bridge.py

This path uses the IsaacLab container for simulation. It deliberately avoids
starting the old docker-compose isaac-sim service.
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
    --with-gello-pedal-teleop)
      WITH_GELLO_PEDAL_TELEOP=true
      shift
      ;;
    --with-keyboard-teleop)
      WITH_KEYBOARD_TELEOP=true
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

echo "IsaacLab container: ${ISAACLAB_CONTAINER}"
echo "Task 1 mount: ${TASK1_ROOT} -> ${CONTAINER_TASK1}"
echo "Embodiment: ${EMBODIMENT}"
echo "USD: ${HOST_USD}"
echo "Controller mode: ${CONTROLLER_MODE}"
echo "GELLO + pedal teleop: ${WITH_GELLO_PEDAL_TELEOP}"
echo "Keyboard arm teleop: ${WITH_KEYBOARD_TELEOP}"
echo "Cable VBD: always enabled"

if ! docker ps --format '{{.Names}}' | grep -qx "${ISAACLAB_CONTAINER}"; then
  echo "Starting IsaacLab container via ${ISAACLAB_ROOT}/docker/container.py..."
  (cd "${ISAACLAB_ROOT}" && ./docker/container.py start "${ISAACLAB_SERVICE}")
fi

if ! docker exec "${ISAACLAB_CONTAINER}" test -d "${CONTAINER_TASK1}"; then
  cat >&2 <<EOF
The IsaacLab container does not have Task 1 mounted at ${CONTAINER_TASK1}.

Apply task1_isaacsim/isaaclab_overlay/apply_overlay.sh and recreate
${ISAACLAB_CONTAINER}, or set CONTAINER_REPO to the actual repository mount.
EOF
  exit 1
fi

echo "Starting raw Newton cable VBD ROS process inside ${ISAACLAB_CONTAINER}..."
docker exec "${ISAACLAB_CONTAINER}" bash -lc "pkill -f '[r]un_cable_vbd_ros_headless.py' || true"
docker exec -d "${ISAACLAB_CONTAINER}" bash -lc "cd ${ISAACLAB_CONTAINER_WS} && source /opt/ros/jazzy/setup.bash && ./isaaclab.sh -p ${CONTAINER_TASK1}/scripts/run_cable_vbd_ros_headless.py --viewer null --device ${CABLE_DEVICE} --config-path ${CABLE_CONFIG_PATH} --gripper-config-path ${CABLE_GRIPPER_CONFIG_PATH} --cable-point-topic /cable/body_centers --num-frames 0 > ${CABLE_LOG_PATH} 2>&1"
echo "Cable VBD log: docker exec ${ISAACLAB_CONTAINER} tail -f ${CABLE_LOG_PATH}"

if ${WITH_REPUBLISHER}; then
  echo "Starting ros_republisher without old isaac-sim dependency..."
  REPUBLISHER_ENV=()
  if ! ${WITH_BROWSER}; then
    REPUBLISHER_ENV+=("REPUBLISHER_DISABLE_BROWSER_COMMAND_TOPICS=true")
  fi
  (cd "${TASK1_ROOT}" && env "${REPUBLISHER_ENV[@]}" docker compose up -d --no-deps ros_republisher)
fi

if [[ "${CONTROLLER_MODE}" == "position" ]]; then
  echo "Starting position_controller without old isaac-sim dependency..."
  (cd "${TASK1_ROOT}" && docker compose --profile position up -d --no-deps position_controller)
fi

if ${WITH_GELLO_PEDAL_TELEOP}; then
  echo "Starting task1_gello_pedal_teleop without old isaac-sim dependency..."
  (cd "${TASK1_ROOT}" && docker compose --profile teleop up -d --no-deps gello_pedal_teleop)
  echo "Pedal publisher (run in a second terminal):"
  echo "  docker exec -it task1_gello_pedal_teleop bash -lc 'source /opt/ros/jazzy/setup.bash && source /tmp/task1_teleop_install/setup.bash && ros2 run pedal_state_publisher pedal_state_publisher'"
fi

if ${WITH_KEYBOARD_TELEOP} && ! ${WITH_GELLO_PEDAL_TELEOP}; then
  echo "Stopping any previously running GELLO teleop container for keyboard-only mode..."
  (cd "${TASK1_ROOT}" && docker compose --profile teleop stop gello_pedal_teleop)
fi

if ${WITH_BROWSER}; then
  echo "Starting browser_controller without old isaac-sim dependency..."
  (cd "${TASK1_ROOT}" && docker compose up -d --no-deps browser_controller)
  echo "Browser UI: http://localhost:8090"
fi

if [[ "${HOST_USD}" != "${TASK1_ROOT}/"* ]]; then
  echo "USD must be inside ${TASK1_ROOT} so the IsaacLab container can see it." >&2
  exit 1
fi

CONTAINER_USD="${CONTAINER_TASK1}/${HOST_USD#"${TASK1_ROOT}/"}"
BRIDGE_ARGS=(
  "--usd-path" "${CONTAINER_USD}"
  "--embodiment" "${EMBODIMENT}"
  "--franka-root" "${CONTAINER_TASK1}"
)


if ${WITH_KEYBOARD_TELEOP}; then
  BRIDGE_ARGS+=("--with-keyboard-teleop")
fi

if ! ${WITH_BROWSER}; then
  BRIDGE_ARGS+=("--no-browser")
fi

if ${HEADLESS}; then
  BRIDGE_ARGS+=("--headless")
else
  BRIDGE_ARGS+=("--visualizer" "kit")
fi

BRIDGE_ARGS+=("${EXTRA_BRIDGE_ARGS[@]}")

echo "Launching IsaacLab Newton/MJWarp bridge..."
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
