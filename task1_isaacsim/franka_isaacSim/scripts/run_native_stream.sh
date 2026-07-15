#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

# Load optional environment defaults (for example INIT_USD from .env)
ENV_FILE="${TWIN_ENV_FILE:-${REPO_ROOT}/.env}"
if [[ -f "${ENV_FILE}" ]]; then
  set -a
  source "${ENV_FILE}"
  set +a
fi

# Ensure mounted Isaac persistence paths are writable by the host user.
mkdir -p docker/isaac-sim/data/runtime_logs/Kit
chmod -R a+rwX docker/isaac-sim/data docker/isaac-sim/data/runtime_logs >/dev/null 2>&1 || true

# Load defaults from embodiment configuration
DEFAULT_EMBODIMENT="${EMBODIMENT:-fr3duo_m+v}"
if [[ -f "${REPO_ROOT}/scripts/stack_config.py" ]]; then
  config_exports="$(python3 "${REPO_ROOT}/scripts/stack_config.py" --embodiment "${DEFAULT_EMBODIMENT}" --format shell-runtime 2>/dev/null)" || true
  if [[ -n "${config_exports}" ]]; then
    eval "${config_exports}"
  fi
fi

# Set script defaults (may be overridden by embodiment config above or CLI args below)
FORCE_RECREATE="${FORCE_RECREATE:-true}"
START_REPUBLISHER="${START_REPUBLISHER:-true}"
START_BROWSER="${START_BROWSER:-true}"
START_GELLO_TELEOP="${START_GELLO_TELEOP:-false}"
START_CABLE_SIM="${START_CABLE_SIM:-false}"
ENABLE_STREAM="${ENABLE_STREAM:-true}"
PHYSICS_BACKEND="${PHYSICS_BACKEND:-${ISAAC_PHYSICS_BACKEND:-physx}}"
EMBEDDED_APP_MODE="${EMBEDDED_APP_MODE:-false}"
FOREGROUND_MODE=false
NATIVE_STAGE_PATH=""
ROS_PUBLISH_RATE="${ROS_PUBLISH_RATE:-60.0}"
PHYSICS_HZ="${PHYSICS_HZ:-240.0}"
RENDER_HZ="${RENDER_HZ:-60.0}"
PHYSICS_SUBSTEPS="${PHYSICS_SUBSTEPS:-2}"
COMMAND_SMOOTHING_ALPHA="${COMMAND_SMOOTHING_ALPHA:-0.35}"
MAX_POSITION_STEP_RAD="${MAX_POSITION_STEP_RAD:-0.04}"
ASSET_NAME="${ASSET_NAME:-}"
ASSET_INDEX="${ASSET_INDEX:-}"
USD_PATH="${USD_PATH:-}"
if [[ -z "${USD_PATH}" && -n "${INIT_USD:-}" ]]; then
  USD_PATH="${INIT_USD}"
fi
ROBOT_PRIM_PATH="${ROBOT_PRIM_PATH:-}"
LIST_ASSETS=false
PORTABLE_ROOT="${PORTABLE_ROOT:-${ISAAC_PORTABLE_ROOT:-/tmp/isaac_portable}}"
ISAAC_SIM_STREAM_LAYOUT_PATH="${ISAAC_SIM_STREAM_LAYOUT_PATH:-}"
# Controller mode: "effort" (default — impedance, gated on franka_controller)
#                  "position" — direct position passthrough via position_controller service
CONTROLLER_MODE="${CONTROLLER_MODE:-effort}"
EXTRA_ARGS=()

print_help() {
  cat <<'EOF'
Usage: bash scripts/run_native_stream.sh [options] [-- <extra isaac_joint_bridge.py args>]

Default behavior:
1) Start docker services: isaac-sim + ros_republisher + browser_controller
2) Launch the stock Isaac app inside isaac-sim
3) Run /workspace/scripts/isaac_joint_bridge_native.py inside that app via --exec
4) Keep Isaac running independently from the calling shell

Options:
  --force-recreate           Recreate Docker containers before starting (default: enabled)
  --no-force-recreate        Skip container recreation (faster restart when containers are healthy)
  --no-republisher           Do not start ros_republisher
  --no-browser               Do not start browser_controller
  --with-gello-pedal-teleop Start GELLO arm teleop and pedal base teleop together
  --with-cable-sim           Start newton_cable service (cable profile) for Newton cable simulation
  --physics-backend <name>   Physics backend inside Isaac: "physx" (default) or "newton"
  --no-stream                Run Isaac app without WebRTC stream
  --embedded-app             Use the legacy embedded SimulationApp bridge path
  --native-app               Explicitly use the native Isaac app path (default)
  --foreground               Keep the launch attached to the current terminal
  --ros-publish-rate <hz>    Forwarded to isaac_joint_bridge.py (default: 60.0)
  --physics-hz <hz>          Isaac physics loop rate (default: 240.0)
  --render-hz <hz>           Isaac render rate (default: 60.0)
  --physics-substeps <n>     Isaac requested physics substeps (default: 2)
  --command-smoothing-alpha <alpha>
                             Command low-pass alpha (default: 0.35, 1.0=off)
  --max-position-step-rad <rad>
                             Max joint command step per sim step (default: 0.04)
  --asset-name <name>        Forwarded to isaac_joint_bridge.py
  --asset-index <index>      Forwarded to isaac_joint_bridge.py
  --usd-path <path>          Forwarded to isaac_joint_bridge.py
  --robot-prim-path <path>   Force articulation root prim path
  --list-assets              Forwarded to isaac_joint_bridge.py and exits
  --portable-root <path>     Kit portable root (default: /tmp/isaac_portable)
  --stage-path <path>        Optional initial stage path for the native app launcher
  --controller-mode <mode>   "effort" (default) or "position":
                               effort   — impedance controller (franka_controller service,
                                          start it separately with docker compose up franka_controller)
                               position — direct position passthrough via position_controller
                                          service (started automatically)

  -h, --help                 Show this help
EOF
}

quote_args() {
  local quoted=()
  local arg
  for arg in "$@"; do
    quoted+=("$(printf '%q' "$arg")")
  done
  local joined
  printf -v joined '%s ' "${quoted[@]}"
  printf '%s' "${joined% }"
}

to_container_path() {
  local path="$1"
  if [[ -z "${path}" ]]; then
    printf '%s\n' "${path}"
    return 0
  fi
  if [[ "${path}" == /workspace/* || "${path}" == /tmp/* || "${path}" == /isaac-sim/* ]]; then
    printf '%s\n' "${path}"
    return 0
  fi
  if [[ "${path}" == /* ]]; then
    printf '%s\n' "${path}"
    return 0
  fi
  local normalized="${path#./}"
  if [[ -e "${REPO_ROOT}/${normalized}" ]]; then
    printf '/workspace/%s\n' "${normalized}"
    return 0
  fi
  printf '%s\n' "${path}"
}

host_path_from_workspace() {
  local path="$1"
  if [[ -z "${path}" ]]; then
    printf '%s\n' "${path}"
    return 0
  fi
  if [[ "${path}" == /workspace/* ]]; then
    printf '%s%s\n' "${REPO_ROOT}" "${path#/workspace}"
    return 0
  fi
  if [[ "${path}" != /* ]]; then
    printf '%s/%s\n' "${REPO_ROOT}" "${path#./}"
    return 0
  fi
  printf '%s\n' "${path}"
}

container_path_from_host() {
  local path="$1"
  if [[ "${path}" == /workspace/* ]]; then
    printf '%s\n' "${path}"
    return 0
  fi
  if [[ "${path}" == "${REPO_ROOT}"* ]]; then
    printf '/workspace%s\n' "${path#${REPO_ROOT}}"
    return 0
  fi
  return 1
}

normalize_extra_args() {
  local normalized=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config|--camera-config|--joint-drive-config|--usd-path)
        normalized+=("$1")
        if [[ $# -gt 1 ]]; then
          normalized+=("$(to_container_path "$2")")
          shift 2
        else
          shift
        fi
        ;;
      *)
        normalized+=("$1")
        shift
        ;;
    esac
  done
  printf '%s\n' "${normalized[@]}"
}

resolve_selected_usd_host() {
  local resolver_args=("--assets-dir" "${REPO_ROOT}/assets")
  local host_usd_path=""
  local resolver_output=""
  local selected_usd=""
  local arg=""

  if [[ -n "${USD_PATH}" ]]; then
    host_usd_path="$(host_path_from_workspace "${USD_PATH}")"
    resolver_args+=("--usd-path" "${host_usd_path}")
  fi
  if [[ -n "${ASSET_NAME}" ]]; then
    resolver_args+=("--asset-name" "${ASSET_NAME}")
  fi
  if [[ -n "${ASSET_INDEX}" ]]; then
    resolver_args+=("--asset-index" "${ASSET_INDEX}")
  fi
  for arg in "${EXTRA_ARGS[@]}"; do
    if [[ "${arg}" == "--select-asset" ]]; then
      resolver_args+=("--select-asset")
      break
    fi
  done

  if ! resolver_output="$(
    python3 "${REPO_ROOT}/scripts/resolve_selected_usd.py" "${resolver_args[@]}" | tee /dev/stderr
  )"; then
    return 1
  fi
  selected_usd="$(printf '%s\n' "${resolver_output}" | sed -n 's/^SELECTED_USD=//p' | tail -n 1)"
  if [[ -z "${selected_usd}" ]]; then
    return 1
  fi
  printf '%s\n' "${selected_usd}"
}

wait_for_kit() {
  local timeout_s="${1:-120}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if docker exec isaac-sim pgrep -x kit >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for Isaac Kit to start" >&2
  return 1
}

wait_for_kit_exit() {
  local timeout_s="${1:-60}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ! docker exec isaac-sim pgrep -x kit >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  echo "Warning: Kit process did not exit within ${timeout_s}s, proceeding anyway" >&2
  return 0
}

wait_for_joint_states() {
  local timeout_s="${1:-180}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if docker compose exec -T ros_republisher bash -lc \
      "source /opt/ros/jazzy/setup.bash && timeout 5 ros2 topic echo /isaac/left_joint_states --once >/dev/null 2>&1"; then
      return 0
    fi
    sleep 2
  done
  echo "Timed out waiting for /isaac/left_joint_states after ${timeout_s}s" >&2
  echo "Diagnostics:" >&2
  echo "  Kit process:" >&2
  docker exec isaac-sim ps -eo pid,args 2>/dev/null | grep '/isaac-sim/kit/kit' | grep -v grep >&2 || echo '    (none)' >&2
  echo "  Recent Isaac logs:" >&2
  docker exec isaac-sim bash -c \
    "latest=\$(find '${PORTABLE_ROOT}/logs' -type f -name 'kit_*.log' 2>/dev/null | sort | tail -1); if [[ -n \"\$latest\" ]]; then tail -30 \"\$latest\"; else echo '    (no log found)'; fi" >&2 || true
  return 1
}

wait_for_stream_port() {
  local port="${1:-8211}"
  local timeout_s="${2:-60}"
  python3 - "$port" "$timeout_s" <<'PY'
import socket
import sys
import time

port = int(sys.argv[1])
deadline = time.time() + float(sys.argv[2])
while time.time() < deadline:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=2.0):
            sys.exit(0)
    except OSError:
        time.sleep(1.0)
sys.exit(1)
PY
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --force-recreate)
      FORCE_RECREATE=true
      shift
      ;;
    --no-force-recreate)
      FORCE_RECREATE=false
      shift
      ;;
    --no-republisher)
      START_REPUBLISHER=false
      shift
      ;;
    --no-browser)
      START_BROWSER=false
      shift
      ;;
    --with-gello-pedal-teleop)
      START_GELLO_TELEOP=true
      shift
      ;;
    --with-cable-sim)
      START_CABLE_SIM=true
      shift
      ;;
    --physics-backend)
      PHYSICS_BACKEND="$2"
      if [[ "${PHYSICS_BACKEND}" != "physx" && "${PHYSICS_BACKEND}" != "newton" ]]; then
        echo "Error: --physics-backend must be 'physx' or 'newton'" >&2
        exit 1
      fi
      shift 2
      ;;
    --no-stream)
      ENABLE_STREAM=false
      shift
      ;;
    --native-app)
      EMBEDDED_APP_MODE=false
      shift
      ;;
    --embedded-app)
      EMBEDDED_APP_MODE=true
      shift
      ;;
    --foreground)
      FOREGROUND_MODE=true
      shift
      ;;
    --stage-path)
      NATIVE_STAGE_PATH="$2"
      shift 2
      ;;
    --ros-publish-rate)
      ROS_PUBLISH_RATE="$2"
      shift 2
      ;;
    --physics-hz)
      PHYSICS_HZ="$2"
      shift 2
      ;;
    --render-hz)
      RENDER_HZ="$2"
      shift 2
      ;;
    --physics-substeps)
      PHYSICS_SUBSTEPS="$2"
      shift 2
      ;;
    --command-smoothing-alpha)
      COMMAND_SMOOTHING_ALPHA="$2"
      shift 2
      ;;
    --max-position-step-rad)
      MAX_POSITION_STEP_RAD="$2"
      shift 2
      ;;
    --asset-name)
      ASSET_NAME="$2"
      shift 2
      ;;
    --asset-index)
      ASSET_INDEX="$2"
      shift 2
      ;;
    --usd-path)
      USD_PATH="$2"
      shift 2
      ;;
    --robot-prim-path)
      ROBOT_PRIM_PATH="$2"
      shift 2
      ;;
    --list-assets)
      LIST_ASSETS=true
      shift
      ;;
    --portable-root)
      PORTABLE_ROOT="$2"
      shift 2
      ;;
    --controller-mode)
      CONTROLLER_MODE="$2"
      if [[ "${CONTROLLER_MODE}" != "effort" && "${CONTROLLER_MODE}" != "position" ]]; then
        echo "Error: --controller-mode must be 'effort' or 'position'" >&2
        exit 1
      fi
      shift 2
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ -n "${USD_PATH}" ]]; then
  USD_PATH="$(to_container_path "${USD_PATH}")"
fi
if [[ -n "${NATIVE_STAGE_PATH}" ]]; then
  NATIVE_STAGE_PATH="$(to_container_path "${NATIVE_STAGE_PATH}")"
fi
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  mapfile -t EXTRA_ARGS < <(normalize_extra_args "${EXTRA_ARGS[@]}")
fi
if [[ "${LIST_ASSETS}" == "true" ]]; then
  python3 "${REPO_ROOT}/scripts/resolve_selected_usd.py" --assets-dir "${REPO_ROOT}/assets" --list-assets
  exit $?
fi

resolved_usd_host="$(resolve_selected_usd_host)" || exit 1
USD_PATH="$(container_path_from_host "${resolved_usd_host}")" || {
  echo "Error: Selected USD must be under ${REPO_ROOT} so the container can access it."
  exit 1
}

services=(isaac-sim)
if [[ "${START_REPUBLISHER}" == "true" ]]; then
  services+=(ros_republisher)
fi
if [[ "${START_BROWSER}" == "true" ]]; then
  services+=(browser_controller)
fi
if [[ "${START_GELLO_TELEOP}" == "true" ]]; then
  services+=(gello_pedal_teleop)
fi
if [[ "${START_CABLE_SIM}" == "true" ]]; then
  services+=(newton_cable)
fi
if [[ "${CONTROLLER_MODE}" == "position" ]]; then
  services+=(position_controller)
  echo "Controller mode: position (position_controller service, no primary-controller gating)"
else
  echo "Controller mode: effort (impedance — start franka_controller separately if needed)"
fi

compose_global_args=()
if [[ "${CONTROLLER_MODE}" == "position" ]]; then
  compose_global_args+=("--profile" "position")
fi
if [[ "${START_GELLO_TELEOP}" == "true" ]]; then
  compose_global_args+=("--profile" "teleop")
fi
if [[ "${START_CABLE_SIM}" == "true" ]]; then
  compose_global_args+=("--profile" "cable")
fi

compose_up_args=(up -d)
if [[ "${FORCE_RECREATE}" == "true" ]]; then
  compose_up_args+=(--force-recreate)
fi

echo "Starting services: ${services[*]}"
docker compose "${compose_global_args[@]}" "${compose_up_args[@]}" "${services[@]}"

echo "Ensuring Isaac portable root exists: ${PORTABLE_ROOT}"
docker exec isaac-sim mkdir -p "${PORTABLE_ROOT}" || true

bridge_args=(
  "--embodiment" "${DEFAULT_EMBODIMENT}"
  "--portable-root" "${PORTABLE_ROOT}"
  "--ros-publish-rate" "${ROS_PUBLISH_RATE}"
  "--physics-backend" "${PHYSICS_BACKEND}"
  "--physics-hz" "${PHYSICS_HZ}"
  "--render-hz" "${RENDER_HZ}"
  "--physics-substeps" "${PHYSICS_SUBSTEPS}"
  "--command-smoothing-alpha" "${COMMAND_SMOOTHING_ALPHA}"
  "--max-position-step-rad" "${MAX_POSITION_STEP_RAD}"
)
if [[ "${ENABLE_STREAM}" == "true" ]]; then
  bridge_args+=("--stream")
fi
if [[ -n "${ASSET_NAME}" ]]; then
  bridge_args+=("--asset-name" "${ASSET_NAME}")
fi
if [[ -n "${ASSET_INDEX}" ]]; then
  bridge_args+=("--asset-index" "${ASSET_INDEX}")
fi
if [[ -n "${USD_PATH}" ]]; then
  bridge_args+=("--usd-path" "${USD_PATH}")
fi
if [[ -n "${ROBOT_PRIM_PATH}" ]]; then
  bridge_args+=("--robot-prim-path" "${ROBOT_PRIM_PATH}")
fi
if [[ "${LIST_ASSETS}" == "true" ]]; then
  bridge_args+=("--list-assets")
fi
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  bridge_args+=("${EXTRA_ARGS[@]}")
fi

for arg in "${EXTRA_ARGS[@]}"; do
  if [[ "${arg}" == "--select-asset" ]]; then
    FOREGROUND_MODE=true
  fi
done
if [[ "${LIST_ASSETS}" == "true" ]]; then
  FOREGROUND_MODE=true
fi

echo "Stopping any stale Isaac Kit process..."
if docker exec isaac-sim pgrep -x kit >/dev/null 2>&1; then
  docker exec isaac-sim pkill -KILL -x kit || true
  wait_for_kit_exit 15
else
  echo "No stale Kit process found."
fi

echo "Launching Isaac ROS bridge simulation..."
echo "Loaded config values: RENDER_HZ=${RENDER_HZ} PHYSICS_HZ=${PHYSICS_HZ} ROS_PUBLISH_RATE=${ROS_PUBLISH_RATE}"
echo "Physics backend: ${PHYSICS_BACKEND}"
echo "Bridge args: ${bridge_args[*]}"
if [[ "${START_BROWSER}" == "true" ]]; then
  echo "Browser UI: http://localhost:8090"
fi

if [[ "${EMBEDDED_APP_MODE}" == "true" ]]; then
  if [[ "${ENABLE_STREAM}" == "true" ]]; then
    echo "Browser stream: http://localhost:8211"
  fi
  launch_cmd=(docker exec)
  if [[ "${FOREGROUND_MODE}" == "true" ]]; then
    launch_cmd+=(-it)
  else
    launch_cmd+=(-d)
  fi
  launch_cmd+=(
    isaac-sim
    /isaac-sim/python.sh
    /workspace/scripts/isaac_joint_bridge.py
    "${bridge_args[@]}"
  )
  "${launch_cmd[@]}"
else
  if [[ "${ENABLE_STREAM}" == "true" ]]; then
    echo "Isaac Sim WebRTC Streaming Client: connect to 127.0.0.1:49100"
  fi
  launcher_path="/isaac-sim/isaac-sim.streaming.sh"
  native_bridge_args=(
    "isaac_joint_bridge_native.py"
    "--embodiment" "${DEFAULT_EMBODIMENT}"
    "--ros-publish-rate" "${ROS_PUBLISH_RATE}"
    "--physics-backend" "${PHYSICS_BACKEND}"
    "--physics-hz" "${PHYSICS_HZ}"
    "--render-hz" "${RENDER_HZ}"
    "--physics-substeps" "${PHYSICS_SUBSTEPS}"
    "--command-smoothing-alpha" "${COMMAND_SMOOTHING_ALPHA}"
    "--max-position-step-rad" "${MAX_POSITION_STEP_RAD}"
  )
  if [[ -n "${ASSET_NAME}" ]]; then
    native_bridge_args+=("--asset-name" "${ASSET_NAME}")
  fi
  if [[ -n "${ASSET_INDEX}" ]]; then
    native_bridge_args+=("--asset-index" "${ASSET_INDEX}")
  fi
  if [[ -n "${USD_PATH}" ]]; then
    native_bridge_args+=("--usd-path" "${USD_PATH}")
  fi
  if [[ -n "${ROBOT_PRIM_PATH}" ]]; then
    native_bridge_args+=("--robot-prim-path" "${ROBOT_PRIM_PATH}")
  fi
  if [[ "${LIST_ASSETS}" == "true" ]]; then
    native_bridge_args+=("--list-assets")
  fi
  if [[ "${CONTROLLER_MODE}" == "position" ]]; then
    native_bridge_args+=("--primary-controller" "")
  fi
  if [[ "${ENABLE_STREAM}" != "true" ]]; then
    launcher_path="/isaac-sim/isaac-sim.sh"
    native_bridge_args+=("--headless")
  fi
  if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
    native_bridge_args+=("${EXTRA_ARGS[@]}")
  fi
  initial_stage_path="${NATIVE_STAGE_PATH}"
  if [[ -z "${initial_stage_path}" && -n "${USD_PATH}" ]]; then
    initial_stage_path="${USD_PATH}"
  fi
  native_launch_args=(docker exec)
  if [[ "${FOREGROUND_MODE}" == "true" ]]; then
    native_launch_args+=(-it)
  else
    native_launch_args+=(-d)
  fi
  native_launch_args+=(
    isaac-sim
    "${launcher_path}"
    "--portable-root" "${PORTABLE_ROOT}"
    "--/app/content/emptyStageOnStart=false"
    "--/app/python/scriptFolders/0=/workspace/scripts"
  )
  if [[ -n "${initial_stage_path}" ]]; then
    native_launch_args+=("--/app/content/stagePath=${initial_stage_path}")
  fi
  if [[ "${ENABLE_STREAM}" == "true" ]]; then
    layout_path="${ISAAC_SIM_STREAM_LAYOUT_PATH}"
    if [[ -z "${layout_path}" ]]; then
      for candidate_layout in \
        "/isaac-sim/kit/data/Kit/Isaac-Sim Streaming/6.0/isaacSim_demo_layout.json" \
        "/isaac-sim/kit/data/Kit/Isaac-Sim Streaming/5.1/isaacSim_demo_layout.json"; do
        if docker exec isaac-sim test -f "${candidate_layout}" >/dev/null 2>&1; then
          layout_path="${candidate_layout}"
          break
        fi
      done
    elif ! docker exec isaac-sim test -f "${layout_path}" >/dev/null 2>&1; then
      echo "Warning: requested Isaac streaming layout not found: ${layout_path}" >&2
      layout_path=""
    fi
    if [[ -n "${layout_path}" ]]; then
      native_launch_args+=(
        "--/app/layout/file=${layout_path}"
        "--/persistent/app/window/layout=${layout_path}"
      )
    else
      echo "No Isaac streaming layout override found; using app default layout."
    fi
  fi
  native_launch_args+=("--exec" "$(quote_args "${native_bridge_args[@]}")")
  "${native_launch_args[@]}"
fi

if [[ "${FOREGROUND_MODE}" == "true" ]]; then
  exit 0
fi

wait_for_kit 120
if [[ "${ENABLE_STREAM}" == "true" ]]; then
  if [[ "${EMBEDDED_APP_MODE}" == "true" ]]; then
    wait_for_stream_port 8211 60
  else
    wait_for_stream_port 49100 60
  fi
fi
if [[ "${START_REPUBLISHER}" == "true" && "${LIST_ASSETS}" != "true" ]]; then
  if ! wait_for_joint_states 360; then
    echo "" >&2
    echo "Stack may still be starting. Check with:" >&2
    echo "  docker logs isaac-sim --tail=50" >&2
    echo "  docker compose exec -T ros_republisher bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 topic list | grep isaac'" >&2
    echo "If Isaac did not start cleanly, re-run (containers are recreated by default)." >&2
    exit 1
  fi
fi

echo "Isaac stack is running in the background."
