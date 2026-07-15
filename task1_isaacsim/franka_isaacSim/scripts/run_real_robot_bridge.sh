#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

BRIDGE_CONFIG=""
CYCLONEDDS_URI=""
EXTRA_ARGS=()

print_help() {
  cat <<'EOF'
Usage: bash scripts/run_real_to_sim_bridge.sh [options]

Bridge real robot joint states (cyclonedds / domain 100) into Isaac sim commands
(fastrtps / domain 0). Requires both rmw_cyclonedds_cpp and rmw_fastrtps_cpp
to be installed. The sim must be running in position-controller mode.

Configuration is loaded from services/real_to_sim_bridge/bridge_config.yaml.
Edit that file to customize topic mappings, domain IDs, and RMW settings.

Options:
  --bridge-config <path>        Path to bridge_config.yaml (default: auto-detect)
  --cyclonedds-uri <uri>        CYCLONEDDS_URI for the reader subprocess (override config)
  -h, --help                    Show this help

Any additional flags are forwarded to real_to_sim_bridge.py.

Legacy CLI Arguments (DEPRECATED - use bridge_config.yaml instead):
  --left-topic, --right-topic, --left-gripper-topic, --right-gripper-topic
  --real-rmw, --real-domain-id
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bridge-config)       BRIDGE_CONFIG="$2"; shift 2 ;;
    --cyclonedds-uri)      CYCLONEDDS_URI="$2"; shift 2 ;;
    -h|--help)             print_help; exit 0 ;;
    *)                     EXTRA_ARGS+=("$1"); shift ;;
  esac
done

# Get repository root for docker-compose
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

echo "Starting real-to-sim bridge via Docker Compose"
echo "  Configuration: bridge_config.yaml"
echo "  Real robot:    CycloneDDS / domain 100 (see config)"
echo "  Sim:           FastRTPS / domain 0"
echo ""
echo "To customize topic mappings, edit:"
echo "  services/real_to_sim_bridge/bridge_config.yaml"

# Set environment variables for docker-compose
if [[ -n "${CYCLONEDDS_URI}" ]]; then
  export TWIN_CYCLONEDDS_URI="${CYCLONEDDS_URI}"
fi

# Build command line for docker-compose run
docker_args=()
[[ -n "${BRIDGE_CONFIG}" ]]       && docker_args+=(--bridge-config "${BRIDGE_CONFIG}")
[[ -n "${CYCLONEDDS_URI}" ]]      && docker_args+=(--cyclonedds-uri "${CYCLONEDDS_URI}")
[[ ${#EXTRA_ARGS[@]} -gt 0 ]]     && docker_args+=("${EXTRA_ARGS[@]}")

cd "${REPO_ROOT}"
exec docker compose run --rm real_to_sim_bridge \
  bash -lc "source /opt/ros/jazzy/setup.bash && python3 /workspace/services/real_to_sim_bridge/real_to_sim_bridge.py $(printf '%q ' "${docker_args[@]}")"
