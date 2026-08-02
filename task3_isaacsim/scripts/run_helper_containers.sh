#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ -f "${TASK3_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${TASK3_ROOT}/.env"
  set +a
fi

usage() {
  cat <<'EOF'
Usage: task3_isaacsim/scripts/run_helper_containers.sh COMMAND [options]

Commands:
  up [options]     Start selected helper containers
  down             Stop and remove all Task 3 helper containers
  status           Show Task 3 helper container status
  logs [SERVICE]   Follow helper logs

Options for up:
  --gripper robotiq|panda    Select gripper calibration (default: robotiq)
  --with-keyboard-teleop     Start keyboard-to-base adapter
  --with-gello-teleop        Start GELLO arm/gripper adapter
  --controller-mode MODE     none|position (default: position)
  --no-browser               Do not start browser controller
  --no-republisher           Do not start ROS joint republisher
EOF
}

profile_environment() {
  local gripper="$1"
  case "${gripper}" in
    robotiq)
      PROFILE_CLOSED_POSITION="0.8"
      PROFILE_INVERT="false"
      ;;
    panda)
      PROFILE_CLOSED_POSITION="0.04"
      PROFILE_INVERT="true"
      ;;
    *)
      echo "--gripper must be 'robotiq' or 'panda'" >&2
      exit 2
      ;;
  esac
  PROFILE_OPEN_POSITION="0.0"
}

compose_with_profile() {
  env \
    "REPUBLISHER_GRIPPER_OPEN_POSITION=${PROFILE_OPEN_POSITION}" \
    "REPUBLISHER_GRIPPER_CLOSED_POSITION=${PROFILE_CLOSED_POSITION}" \
    "REPUBLISHER_GRIPPER_INVERT=${PROFILE_INVERT}" \
    docker compose "$@"
}

cmd_up() {
  local gripper="${GRIPPER_PROFILE:-robotiq}"
  local controller_mode="${CONTROLLER_MODE:-position}"
  local with_keyboard=false
  local with_gello=false
  local with_browser=true
  local with_republisher=true

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --gripper)
        gripper="$2"
        shift 2
        ;;
      --with-keyboard-teleop)
        with_keyboard=true
        shift
        ;;
      --with-gello-teleop|--with-gello-pedal-teleop)
        with_gello=true
        shift
        ;;
      --controller-mode)
        controller_mode="$2"
        shift 2
        ;;
      --no-browser)
        with_browser=false
        shift
        ;;
      --no-republisher)
        with_republisher=false
        shift
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      *)
        echo "Unknown up option: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done

  case "${controller_mode}" in
    none|position) ;;
    *)
      echo "--controller-mode must be 'none' or 'position'" >&2
      exit 2
      ;;
  esac
  profile_environment "${gripper}"

  local adapters=""
  ${with_keyboard} && adapters="${adapters} keyboard"
  ${with_gello} && adapters="${adapters} gello"
  adapters="$(echo "${adapters}" | xargs || true)"

  cd "${TASK3_ROOT}"
  local disabled_services=()
  ${with_republisher} || disabled_services+=(ros_republisher)
  [[ "${controller_mode}" == "position" ]] || disabled_services+=(position_controller)
  [[ -n "${adapters}" ]] || disabled_services+=(teleop_adapters)
  ${with_browser} || disabled_services+=(browser_controller)
  if [[ ${#disabled_services[@]} -gt 0 ]]; then
    docker compose --profile "*" rm -sf "${disabled_services[@]}"
  fi
  if ${with_republisher}; then
    local republisher_env=()
    ${with_browser} || republisher_env=("REPUBLISHER_DISABLE_BROWSER_COMMAND_TOPICS=true")
    env "${republisher_env[@]}" \
      "REPUBLISHER_GRIPPER_OPEN_POSITION=${PROFILE_OPEN_POSITION}" \
      "REPUBLISHER_GRIPPER_CLOSED_POSITION=${PROFILE_CLOSED_POSITION}" \
      "REPUBLISHER_GRIPPER_INVERT=${PROFILE_INVERT}" \
      docker compose up -d --no-deps ros_republisher
  fi
  if [[ "${controller_mode}" == "position" ]]; then
    compose_with_profile --profile position up -d --no-deps position_controller
  fi
  if [[ -n "${adapters}" ]]; then
    env "TELEOP_ADAPTERS=${adapters}" \
      "REPUBLISHER_GRIPPER_OPEN_POSITION=${PROFILE_OPEN_POSITION}" \
      "REPUBLISHER_GRIPPER_CLOSED_POSITION=${PROFILE_CLOSED_POSITION}" \
      "REPUBLISHER_GRIPPER_INVERT=${PROFILE_INVERT}" \
      docker compose --profile teleop up -d --no-deps teleop_adapters
  fi
  if ${with_browser}; then
    compose_with_profile up -d --no-deps browser_controller
    echo "Browser UI: http://localhost:8090"
  fi
  echo "Task 3 helper profile: ${gripper}"
}

command="${1:-}"
shift || true
case "${command}" in
  up)
    cmd_up "$@"
    ;;
  down)
    (cd "${TASK3_ROOT}" && docker compose --profile "*" down)
    ;;
  status)
    (cd "${TASK3_ROOT}" && docker compose --profile "*" ps)
    ;;
  logs)
    (cd "${TASK3_ROOT}" && docker compose --profile "*" logs -f "$@")
    ;;
  --help|-h|"")
    usage
    ;;
  *)
    echo "Unknown command: ${command}" >&2
    usage >&2
    exit 2
    ;;
esac
