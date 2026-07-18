#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

# --------------------------------------------------------------------------
# Launcher for the Task 2 LeRobot demonstration recorder service
# (task2_isaacsim/docker-compose.yml: lerobot_recorder, profile "record";
# code + config in task2_isaacsim/services/recording/).
#
# The recorder is interactive: stdin drives episode control (1 clear,
# 2 start, 3 save, 0 discard, 4 visualize, q quit), so `record` runs in the
# foreground with a TTY. Defaults come from services/recording/recording.yaml;
# flags after `--` are passed to record_task2.py and override the config.
# --------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK2_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${TASK2_ROOT}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  task2_isaacsim/scripts/run_recorder.sh [command] [options] [-- RECORDER ARGS...]

Commands:
  record (default)  Run an interactive recording session
  build             Build/rebuild the recorder image
  shell             Open a bash shell in the recorder container (debugging)

Options for `record`:
  --config PATH       Recording config YAML (default:
                      services/recording/recording.yaml). Must live inside
                      the repository — the container reads it through the
                      /repo bind mount.
  --resume            Append to the latest existing dataset version
  --resume-version N  Append to version N (implies resume)
  --build             Rebuild the recorder image before running

Everything after `--` goes to record_task2.py verbatim and overrides the
config file (precedence: argparse defaults < config YAML < CLI flags),
e.g.:
  run_recorder.sh record -- --fps 20 --record-depth
  run_recorder.sh record --config my_recording.yaml -- --max_episodes 5

Requires the Isaac Sim scene running with recording topics enabled
(run_isaacsim_teleop.sh --scene room -- --record).
EOF
}

# Datasets on the bind mount should belong to the caller, not root.
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

cmd_build() {
  (cd "${TASK2_ROOT}" && docker compose --profile record build lerobot_recorder)
}

cmd_shell() {
  (cd "${TASK2_ROOT}" && docker compose --profile record run --rm --entrypoint bash lerobot_recorder)
}

cmd_record() {
  local build_first=false
  local recorder_args=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config)
        [[ $# -ge 2 ]] || { echo "--config needs a path" >&2; exit 2; }
        local config_path=""
        if [[ -f "$2" ]]; then
          config_path="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"
        elif [[ -f "${TASK2_ROOT}/$2" ]]; then
          # task2-relative path (e.g. services/recording/foo.yaml).
          config_path="${TASK2_ROOT}/$2"
        else
          echo "Config not found: $2" >&2
          exit 2
        fi
        if [[ "${config_path}" != "${REPO_ROOT}"/* ]]; then
          echo "Config must be inside the repository (${REPO_ROOT}) —" \
            "the container reads it via the /repo bind mount." >&2
          exit 2
        fi
        export RECORDER_CONFIG="/repo/${config_path#"${REPO_ROOT}/"}"
        shift 2
        ;;
      --resume)
        recorder_args+=("--resume")
        shift
        ;;
      --resume-version)
        [[ $# -ge 2 ]] || { echo "--resume-version needs a number" >&2; exit 2; }
        recorder_args+=("--resume_version" "$2")
        shift 2
        ;;
      --build)
        build_first=true
        shift
        ;;
      --help|-h)
        usage
        exit 0
        ;;
      --)
        shift
        recorder_args+=("$@")
        break
        ;;
      *)
        echo "Unknown 'record' option: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done

  ${build_first} && cmd_build

  # Append to any RECORDER_ARGS already in the environment so existing
  # RECORDER_ARGS-based workflows keep working.
  export RECORDER_ARGS="${RECORDER_ARGS:-}${recorder_args[@]+ ${recorder_args[*]}}"

  echo "Recorder config: ${RECORDER_CONFIG:-services/recording/recording.yaml}"
  [[ -n "${RECORDER_ARGS// /}" ]] && echo "Recorder args:  ${RECORDER_ARGS}"
  (cd "${TASK2_ROOT}" && docker compose --profile record run --rm lerobot_recorder)
}

COMMAND="${1:-record}"
case "${COMMAND}" in
  record|build|shell)
    shift || true
    ;;
  --help|-h)
    usage
    exit 0
    ;;
  --*)
    # Options without a command imply `record`.
    COMMAND="record"
    ;;
  *)
    echo "Unknown command: ${COMMAND}" >&2
    usage >&2
    exit 2
    ;;
esac

case "${COMMAND}" in
  record) cmd_record "$@" ;;
  build) cmd_build ;;
  shell) cmd_shell ;;
esac
