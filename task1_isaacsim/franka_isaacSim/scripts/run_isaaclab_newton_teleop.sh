#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-/homeL/qiguan/dataSSD/IsaacLab}"
ISAACLAB_SERVICE="${ISAACLAB_SERVICE:-ros2_jazzy}"
ISAACLAB_CONTAINER="${ISAACLAB_CONTAINER:-isaac-lab-ros2_jazzy}"
EMBODIMENT="${EMBODIMENT:-fr3duo_mobile}"
USD_PATH="${USD_PATH:-assets/digital_twin_fr3Duo_mobile.usd}"
CONTROLLER_MODE="${CONTROLLER_MODE:-position}"
WITH_GELLO_PEDAL_TELEOP=false
WITH_BROWSER=true
WITH_REPUBLISHER=true
WITH_CABLE=false
HEADLESS=false
CABLE_DEVICE="${CABLE_DEVICE:-cuda:0}"
CABLE_CONFIG_PATH="${CABLE_CONFIG_PATH:-/workspace/franka_isaacSim/cable_world/configs/table_board_fixture_cable.yaml}"
CABLE_GRIPPER_CONFIG_PATH="${CABLE_GRIPPER_CONFIG_PATH:-/workspace/franka_isaacSim/cable_world/configs/gripper.yaml}"
CABLE_LOG_PATH="${CABLE_LOG_PATH:-/tmp/franka_cable_vbd.log}"
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
  --no-browser               Do not start browser_controller
  --no-republisher           Do not start ros_republisher
  --with-cable               Run the raw Newton VBD board-cable world
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
  HOST_USD="${REPO_ROOT}/${USD_PATH}"
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
echo "Embodiment: ${EMBODIMENT}"
echo "USD: ${HOST_USD}"
echo "Controller mode: ${CONTROLLER_MODE}"
echo "GELLO + pedal teleop: ${WITH_GELLO_PEDAL_TELEOP}"
echo "Cable VBD: ${WITH_CABLE}"

if ! docker ps --format '{{.Names}}' | grep -qx "${ISAACLAB_CONTAINER}"; then
  echo "Starting IsaacLab container via ${ISAACLAB_ROOT}/docker/container.py..."
  (cd "${ISAACLAB_ROOT}" && ./docker/container.py start "${ISAACLAB_SERVICE}")
fi

if ! docker exec "${ISAACLAB_CONTAINER}" test -d /workspace/franka_isaacSim; then
  cat >&2 <<EOF
The IsaacLab container does not have /workspace/franka_isaacSim mounted.

Add this bind mount to IsaacLab's docker compose volumes:
  /homeL/qiguan/dataSSD/franka_isaacSim:/workspace/franka_isaacSim

Then recreate ${ISAACLAB_CONTAINER}.
EOF
  exit 1
fi

if ${WITH_CABLE}; then
  echo "Starting raw Newton cable VBD ROS process inside ${ISAACLAB_CONTAINER}..."
  docker exec "${ISAACLAB_CONTAINER}" bash -lc "pkill -f '[r]un_cable_vbd_ros_headless.py' || true"
  docker exec -d "${ISAACLAB_CONTAINER}" bash -lc "cd /workspace/isaaclab && source /opt/ros/jazzy/setup.bash && ./isaaclab.sh -p /workspace/franka_isaacSim/scripts/run_cable_vbd_ros_headless.py --viewer null --device ${CABLE_DEVICE} --config-path ${CABLE_CONFIG_PATH} --gripper-config-path ${CABLE_GRIPPER_CONFIG_PATH} --cable-point-topic /cable/body_centers --num-frames 0 > ${CABLE_LOG_PATH} 2>&1"
  echo "Cable VBD log: docker exec ${ISAACLAB_CONTAINER} tail -f ${CABLE_LOG_PATH}"
fi

if ${WITH_REPUBLISHER}; then
  echo "Starting ros_republisher without old isaac-sim dependency..."
  REPUBLISHER_ENV=()
  if ! ${WITH_BROWSER}; then
    REPUBLISHER_ENV+=("REPUBLISHER_DISABLE_BROWSER_COMMAND_TOPICS=true")
  fi
  (cd "${REPO_ROOT}" && env "${REPUBLISHER_ENV[@]}" docker compose up -d --no-deps ros_republisher)
fi

if [[ "${CONTROLLER_MODE}" == "position" ]]; then
  echo "Starting position_controller without old isaac-sim dependency..."
  (cd "${REPO_ROOT}" && docker compose --profile position up -d --no-deps position_controller)
fi

if ${WITH_GELLO_PEDAL_TELEOP}; then
  echo "Starting gello_pedal_teleop without old isaac-sim dependency..."
  (cd "${REPO_ROOT}" && docker compose --profile teleop up -d --no-deps gello_pedal_teleop)
fi

if ${WITH_BROWSER}; then
  echo "Starting browser_controller without old isaac-sim dependency..."
  (cd "${REPO_ROOT}" && docker compose up -d --no-deps browser_controller)
  echo "Browser UI: http://localhost:8090"
fi

if [[ "${HOST_USD}" != "${REPO_ROOT}/"* ]]; then
  echo "USD must be inside ${REPO_ROOT} so the IsaacLab container can see it." >&2
  exit 1
fi

CONTAINER_USD="/workspace/franka_isaacSim/${HOST_USD#"${REPO_ROOT}/"}"
BRIDGE_ARGS=(
  "--usd-path" "${CONTAINER_USD}"
  "--embodiment" "${EMBODIMENT}"
  "--franka-root" "/workspace/franka_isaacSim"
)

if ${WITH_CABLE}; then
  BRIDGE_ARGS+=("--with-cable")
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
  "cd /workspace/isaaclab && source /opt/ros/jazzy/setup.bash && ./isaaclab.sh -p /workspace/franka_isaacSim/scripts/isaaclab_fr3duo_newton_bridge_sam.py${BRIDGE_ARGS_QUOTED}"
