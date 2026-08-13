#!/usr/bin/env bash
# Lifecycle wrapper for the task2 eval container.
#   up | down | status | evaluate | logs
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

COMPOSE_FILE="${EVAL_TASK2_COMPOSE_FILE:-${SCRIPT_DIR}/docker-compose.yml}"
ENV_FILE="${EVAL_TASK2_ENV_FILE:-${SCRIPT_DIR}/.env}"
SERVICE_NAME="${EVAL_TASK2_SERVICE_NAME:-eval_task2}"
EVALUATE_SERVICE="${EVAL_TASK2_EVALUATE_SERVICE:-/isaac/eval_camera/evaluate}"

# Fall back to live host UID/GID if no .env yet.
export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

compose_args=(-f "${COMPOSE_FILE}")
if [ -f "${ENV_FILE}" ]; then
    compose_args=(--env-file "${ENV_FILE}" "${compose_args[@]}")
fi

docker_compose() {
    docker compose "${compose_args[@]}" "$@"
}

usage() {
    cat <<EOF
Usage: bash ${BASH_SOURCE[0]##*/} <up|down|status|evaluate|logs>

Commands:
  up        Build (if needed) and start the eval_task2 container (profile: eval)
  down      Stop and remove the eval_task2 container
  status    Check the container is running and the evaluate service is visible
  evaluate  Capture the current frame and compute IoU (alias: capture)
  logs      Follow the container logs

Run setup.sh first to create the persistent volume and .env.

Environment overrides:
  EVAL_TASK2_COMPOSE_FILE  EVAL_TASK2_ENV_FILE
  EVAL_TASK2_SERVICE_NAME  EVAL_TASK2_EVALUATE_SERVICE
  HOST_UID / HOST_GID
EOF
}

wait_for_service() {
    local timeout_s="${1:-30}"
    local deadline=$((SECONDS + timeout_s))
    while ((SECONDS < deadline)); do
        if docker_compose ps --status running "${SERVICE_NAME}" | grep -q "${SERVICE_NAME}"; then
            return 0
        fi
        sleep 1
    done
    return 1
}

main() {
    command -v docker >/dev/null 2>&1 || { echo "[FAIL] Missing command: docker"; exit 1; }

    [[ $# -ge 1 ]] || { usage; exit 1; }

    case "$1" in
        up)
            export EVAL_VERSION="$(git -C "${SCRIPT_DIR}/../../.." rev-parse --short HEAD 2>/dev/null || echo unknown)"
            docker_compose --profile eval up -d --build "${SERVICE_NAME}"
            if wait_for_service 30; then
                echo "[PASS] ${SERVICE_NAME} is running"
            else
                echo "[WARN] ${SERVICE_NAME} did not report as running within 30s"
            fi
            ;;
        down)
            docker_compose --profile eval down
            ;;
        status)
            if docker_compose ps --status running "${SERVICE_NAME}" | grep -q "${SERVICE_NAME}"; then
                echo "[PASS] ${SERVICE_NAME} is running"
            else
                echo "[FAIL] ${SERVICE_NAME} is not running"
                exit 1
            fi
            if docker_compose exec -T "${SERVICE_NAME}" bash -lc \
                "source /opt/ros/jazzy/setup.bash && ros2 service list | grep -qx '${EVALUATE_SERVICE}'"; then
                echo "[PASS] Evaluate service available: ${EVALUATE_SERVICE}"
            else
                echo "[WARN] Evaluate service not yet visible: ${EVALUATE_SERVICE}"
            fi
            ;;
        evaluate | capture)
            if ! docker_compose ps --status running "${SERVICE_NAME}" | grep -q "${SERVICE_NAME}"; then
                echo "[FAIL] ${SERVICE_NAME} is not running"
                exit 1
            fi
            docker_compose exec -T "${SERVICE_NAME}" bash -lc \
                "source /opt/ros/jazzy/setup.bash && ros2 service call ${EVALUATE_SERVICE} std_srvs/srv/Trigger '{}'"
            ;;
        logs)
            docker_compose logs -f "${SERVICE_NAME}"
            ;;
        -h | --help | help)
            usage
            ;;
        *)
            echo "[FAIL] Unknown command: $1"
            usage
            exit 1
            ;;
    esac
}

main "$@"
