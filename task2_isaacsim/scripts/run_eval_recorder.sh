#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

# --------------------------------------------------------------------------
# Launcher for the Task 2 evaluation audit recorder service
# (task2_isaacsim/docker-compose.yml: eval_recording, profile
# "eval_recording"; code + config in task2_isaacsim/services/eval_recording/).
#
# The recorder is a passive observer with an interactive console (idle:
# 1 reset request, 2 force-start, e evaluate, s status, q quit; recording:
# 3 stop, 0 discard, e, s, q), so `record` runs in the foreground with a
# TTY. Defaults come from services/eval_recording/eval_recording.yaml;
# flags after `--` are passed to record_eval_task2.py and override the
# config. Before attaching, `record` captures host-side session
# provenance (git state, contract hashes, container image ids) into
# <out-root>/<eval-name>/.session_provenance.json — the recorder copies
# it into each episode as provenance.json.
# --------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK2_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${TASK2_ROOT}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  task2_isaacsim/scripts/run_eval_recorder.sh [command] [options] [-- RECORDER ARGS...]

Commands:
  record (default)  Run an interactive audit session
  build             Build/rebuild the recorder image
  shell             Open a bash shell in the recorder container (debugging)
  mock-sim          Run the synthetic contract publisher (tests/mock_sim.py)

Options for `record`:
  --eval-name NAME    Session name (required); output lands in
                      <out-root>/NAME/ep_NNN/.
  --episode NAME      Name for the first episode (default ep_NNN).
  --config PATH       Recorder config YAML (default:
                      services/eval_recording/eval_recording.yaml). Must
                      live inside the repository — the container reads it
                      through the /repo bind mount.

Everything after `--` goes to record_eval_task2.py (or mock_sim.py for
`mock-sim`) verbatim and overrides the config file (precedence: argparse
defaults < config YAML < CLI flags), e.g.:
  run_eval_recorder.sh record --eval-name sub_16 -- --manual
  run_eval_recorder.sh record --eval-name dry_run -- --no-auto-evaluate
  run_eval_recorder.sh mock-sim -- --cameras head,eval_camera --reset-every 25

Environment (see .env.example): EVAL_NAME, EVAL_RAW_OUT,
EVAL_RECORDER_CONFIG, EVAL_RECORDER_ARGS, ISAAC_DOCKER_ROOT,
EVAL_TARGET_DIR, ISAAC_SIM_5_CONTAINER, EVAL_SERVICE_CONTAINER.

Requires the Isaac Sim scene running with recording topics enabled
(run_isaacsim_teleop.sh --scene room -- --record) and, for evaluator
capture, the eval service up (bash scripts/evaluation/task2/run.sh up).
EOF
}

# Outputs on the bind mount should belong to the caller, not root.
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

compose() {
  (cd "${TASK2_ROOT}" && docker compose --profile eval_recording "$@")
}

cmd_build() {
  compose build eval_recording
}

cmd_shell() {
  compose run --rm --entrypoint bash eval_recording
}

cmd_mock_sim() {
  local args=()
  if [[ "${1:-}" == "--" ]]; then
    shift
    args=("$@")
  elif [[ $# -gt 0 ]]; then
    echo "mock-sim takes no options; pass mock_sim.py flags after --" >&2
    exit 2
  fi
  local t_flag=()
  [[ -t 0 ]] || t_flag=(-T)
  compose run --rm "${t_flag[@]}" eval_recording bash -lc \
    "source /opt/ros/jazzy/setup.bash && exec python3 \
     services/eval_recording/tests/mock_sim.py ${args[*]:-}"
}

capture_provenance() {
  local host_out="$1"
  REPO_ROOT="${REPO_ROOT}" HOST_OUT="${host_out}" \
  EVAL_TARGET_DIR="${EVAL_TARGET_DIR:-}" \
  ISAAC_SIM_5_CONTAINER="${ISAAC_SIM_5_CONTAINER:-isaac-sim-5-1-0-workshop}" \
  EVAL_SERVICE_CONTAINER="${EVAL_SERVICE_CONTAINER:-eval_task2}" \
  EVAL_RECORDER_IMAGE="${EVAL_RECORDER_IMAGE:-task2-eval-recording}" \
  python3 - <<'EOF'
import hashlib
import json
import os
import subprocess
from datetime import datetime, timezone

repo = os.environ["REPO_ROOT"]
out = os.environ["HOST_OUT"]


def run(*cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=30).stdout.strip()
    except Exception as exc:  # noqa: BLE001
        return f"<error: {exc}>"


def sha256(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError as exc:
        return f"<error: {exc}>"


t2 = os.path.join(repo, "task2_isaacsim")
target = os.environ.get("EVAL_TARGET_DIR", "")
provenance = {
    "wall_time_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    "bench_tree": repo,
    "bench_head": run("git", "-C", repo, "rev-parse", "HEAD"),
    "bench_status": run("git", "-C", repo, "status", "--porcelain"),
    "bench_diff": run("git", "-C", repo, "diff"),
    "sha256": {
        "topics.py": sha256(os.path.join(t2, "scripts", "topics.py")),
        "topics.yaml": sha256(os.path.join(t2, "config", "topics.yaml")),
    },
    "eval_target_dir": target or None,
    "eval_target_head": (run("git", "-C", target, "rev-parse", "HEAD")
                         if target and os.path.isdir(target) else None),
    "images": {
        name: run("docker", "inspect", "--format",
                  "{{.Image}} ({{.Config.Image}})", name)
        for name in (os.environ["ISAAC_SIM_5_CONTAINER"],
                     os.environ["EVAL_SERVICE_CONTAINER"])
    },
    "recorder_image": run("docker", "image", "inspect", "--format",
                          "{{.Id}}", os.environ["EVAL_RECORDER_IMAGE"]),
}
path = os.path.join(out, ".session_provenance.json")
with open(path, "w") as f:
    json.dump(provenance, f, indent=2)
print(f"[PASS] provenance captured: {path}")
EOF
}

cmd_record() {
  local eval_name="" episode=""
  local recorder_args=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --eval-name)
        [[ $# -ge 2 ]] || { echo "--eval-name needs a value" >&2; exit 2; }
        eval_name="$2"
        shift 2
        ;;
      --episode)
        [[ $# -ge 2 ]] || { echo "--episode needs a name" >&2; exit 2; }
        episode="$2"
        shift 2
        ;;
      --config)
        [[ $# -ge 2 ]] || { echo "--config needs a path" >&2; exit 2; }
        local config_path=""
        if [[ -f "$2" ]]; then
          config_path="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"
        elif [[ -f "${TASK2_ROOT}/$2" ]]; then
          # task2-relative path (e.g. services/eval_recording/foo.yaml).
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
        export EVAL_RECORDER_CONFIG="/repo/${config_path#"${REPO_ROOT}/"}"
        shift 2
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

  if [[ -z "${eval_name}" ]]; then
    echo "--eval-name is required" >&2
    usage >&2
    exit 2
  fi
  export EVAL_NAME="${eval_name}"

  # Pre-create the evaluator-artifacts bind source user-owned; docker
  # would auto-create a missing one as root.
  mkdir -p "${ISAAC_DOCKER_ROOT:-${HOME}/docker/ebim-challenge}/eval-task2"

  # Host-side session provenance (the container has no git/docker).
  local out_rel="${EVAL_RAW_OUT:-services/eval_recording/eval_raw_out}"
  if [[ "${out_rel}" = /* ]]; then
    echo "[WARN] EVAL_RAW_OUT is an absolute container path; skipping" \
      "host-side provenance capture" >&2
  else
    local host_out="${TASK2_ROOT}/${out_rel}/${eval_name}"
    mkdir -p "${host_out}"
    capture_provenance "${host_out}"
  fi

  # Append to any EVAL_RECORDER_ARGS already in the environment so
  # existing EVAL_RECORDER_ARGS-based workflows keep working.
  local extra=""
  [[ -n "${episode}" ]] && extra="--episode ${episode}"
  export EVAL_RECORDER_ARGS="${EVAL_RECORDER_ARGS:-}${extra:+ ${extra}}${recorder_args[@]+ ${recorder_args[*]}}"

  echo "Recorder config: ${EVAL_RECORDER_CONFIG:-services/eval_recording/eval_recording.yaml}"
  echo "Eval name:       ${EVAL_NAME}"
  [[ -n "${EVAL_RECORDER_ARGS// /}" ]] && echo "Recorder args:   ${EVAL_RECORDER_ARGS}"
  local t_flag=()
  [[ -t 0 ]] || t_flag=(-T)   # piped stdin: line-mode console, no TTY
  compose run --rm "${t_flag[@]}" eval_recording
}

COMMAND="${1:-record}"
case "${COMMAND}" in
  record|build|shell|mock-sim)
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
  mock-sim) cmd_mock_sim "$@" ;;
esac
