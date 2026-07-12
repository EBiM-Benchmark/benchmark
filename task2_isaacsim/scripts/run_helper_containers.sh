#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

# --------------------------------------------------------------------------
# Lifecycle for the Task 2 ROS 2 helper containers
# (task2_isaacsim/docker-compose.yml: ros_republisher, position_controller,
# teleop_adapters, browser_controller — all reusing the Task 1 scripts).
#
# Used by run_isaacsim_teleop.sh to start the helpers before launching the
# simulator; also usable standalone.
# --------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK2_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  task2_isaacsim/scripts/run_helper_containers.sh <command> [options]

Commands:
  up [options]     Start the helper containers (idempotent)
  down             Stop and remove all helper containers (all profiles)
  status           Show helper container status
  logs [SERVICE]   Follow logs (all services, or one of: ros_republisher,
                   position_controller, teleop_adapters, browser_controller)

Options for `up` (same conventions as run_isaacsim_teleop.sh):
  --with-keyboard-teleop     Start the keyboard->base teleop adapter
  --with-gello-teleop        Start the GELLO->bridge teleop adapter
  --with-gello-pedal-teleop  Alias of --with-gello-teleop
  --controller-mode MODE     none|position (default: position)
  --no-browser               Do not start browser_controller
  --no-republisher           Do not start ros_republisher

Helper defaults (gripper calibration, adapter selection) come from
task2_isaacsim/.env if present; see .env.example.
EOF
}

cmd_up() {
  local controller_mode="${CONTROLLER_MODE:-position}"
  local with_keyboard=false
  local with_gello=false
  local with_browser=true
  local with_republisher=true

  while [[ $# -gt 0 ]]; do
    case "$1" in
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
        echo "Unknown 'up' option: $1" >&2
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

  local teleop_adapters=""
  ${with_keyboard} && teleop_adapters="${teleop_adapters} keyboard"
  ${with_gello} && teleop_adapters="${teleop_adapters} gello"
  teleop_adapters="$(echo "${teleop_adapters}" | xargs || true)"

  if ${with_republisher}; then
    echo "Starting ros_republisher..."
    local republisher_env=()
    if ! ${with_browser}; then
      republisher_env=("REPUBLISHER_DISABLE_BROWSER_COMMAND_TOPICS=true")
    fi
    (cd "${TASK2_ROOT}" && env "${republisher_env[@]}" docker compose up -d --no-deps ros_republisher)
  fi

  if [[ "${controller_mode}" == "position" ]]; then
    echo "Starting position_controller..."
    (cd "${TASK2_ROOT}" && docker compose --profile position up -d --no-deps position_controller)
  fi

  if [[ -n "${teleop_adapters}" ]]; then
    echo "Starting teleop_adapters (${teleop_adapters})..."
    (cd "${TASK2_ROOT}" && env "TELEOP_ADAPTERS=${teleop_adapters}" docker compose --profile teleop up -d --no-deps teleop_adapters)
  fi

  if ${with_browser}; then
    echo "Starting browser_controller..."
    (cd "${TASK2_ROOT}" && docker compose up -d --no-deps browser_controller)
    echo "Browser UI: http://localhost:8090"
  fi
}

cmd_down() {
  (cd "${TASK2_ROOT}" && docker compose --profile "*" down)
}

cmd_status() {
  (cd "${TASK2_ROOT}" && docker compose --profile "*" ps)
}

cmd_logs() {
  (cd "${TASK2_ROOT}" && docker compose --profile "*" logs -f "$@")
}

COMMAND="${1:-}"
shift || true
case "${COMMAND}" in
  up)
    cmd_up "$@"
    ;;
  down)
    cmd_down
    ;;
  status)
    cmd_status
    ;;
  logs)
    cmd_logs "$@"
    ;;
  --help|-h|"")
    usage
    exit 0
    ;;
  *)
    echo "Unknown command: ${COMMAND}" >&2
    usage >&2
    exit 2
    ;;
esac
