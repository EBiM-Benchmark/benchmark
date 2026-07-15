#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

LEFT_TOPIC="/left/joint_states"
RIGHT_TOPIC="/right/joint_states"
LEFT_GRIPPER_TOPIC=""
RIGHT_GRIPPER_TOPIC=""
REAL_RMW="rmw_cyclonedds_cpp"
REAL_DOMAIN_ID="100"
CYCLONEDDS_URI=""
EXTRA_ARGS=()

print_help() {
  cat <<'EOF'
Usage: bash scripts/run_real_robot_follower.sh [options]

Mirror real robot joint states into Isaac sim position commands.
Uses a cross-RMW subprocess to bridge real robot data (cyclonedds / domain 53)
into the sim stack (fastrtps / domain 0).  Requires the sim to be running in
position-controller mode.

Options:
  --left-topic <topic>          JointState topic for the real left arm
                                (default: /left/joint_states)
  --right-topic <topic>         JointState topic for the real right arm
                                (default: /right/joint_states)
  --left-gripper-topic <topic>  JointState topic for the real left gripper (optional)
  --right-gripper-topic <topic> JointState topic for the real right gripper (optional)
  --real-rmw <implementation>   RMW of the real robots (default: rmw_cyclonedds_cpp)
  --real-domain-id <id>         ROS_DOMAIN_ID of the real robots (default: 100)
  --cyclonedds-uri <uri>        CYCLONEDDS_URI for the reader subprocess (optional)
  -h, --help                    Show this help

Any additional flags are forwarded to real_robot_follower.py
(e.g. --publish-rate, --stale-timeout, --gripper-joint-index).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --left-topic)
      LEFT_TOPIC="$2"; shift 2 ;;
    --right-topic)
      RIGHT_TOPIC="$2"; shift 2 ;;
    --left-gripper-topic)
      LEFT_GRIPPER_TOPIC="$2"; shift 2 ;;
    --right-gripper-topic)
      RIGHT_GRIPPER_TOPIC="$2"; shift 2 ;;
    --real-rmw)
      REAL_RMW="$2"; shift 2 ;;
    --real-domain-id)
      REAL_DOMAIN_ID="$2"; shift 2 ;;
    --cyclonedds-uri)
      CYCLONEDDS_URI="$2"; shift 2 ;;
    -h|--help)
      print_help; exit 0 ;;
    *)
      EXTRA_ARGS+=("$1"); shift ;;
  esac
done

args=(
  --left-topic  "${LEFT_TOPIC}"
  --right-topic "${RIGHT_TOPIC}"
  --real-rmw    "${REAL_RMW}"
  --real-domain-id "${REAL_DOMAIN_ID}"
)
if [[ -n "${LEFT_GRIPPER_TOPIC}" ]]; then
  args+=(--left-gripper-topic "${LEFT_GRIPPER_TOPIC}")
fi
if [[ -n "${RIGHT_GRIPPER_TOPIC}" ]]; then
  args+=(--right-gripper-topic "${RIGHT_GRIPPER_TOPIC}")
fi
if [[ -n "${CYCLONEDDS_URI}" ]]; then
  args+=(--cyclonedds-uri "${CYCLONEDDS_URI}")
fi
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  args+=("${EXTRA_ARGS[@]}")
fi

# Parent process runs on the sim domain (fastrtps / domain 0).
# The reader subprocess uses the real-robot RMW/domain passed via CLI args.
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export FASTDDS_BUILTIN_TRANSPORTS="UDPv4"
unset ROS_DOMAIN_ID

# Ensure scripts/ is on the Python path for local imports
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

echo "Starting real robot follower"
echo "  Sim side:    rmw_fastrtps_cpp / domain 0"
echo "  Real robot:  ${REAL_RMW} / domain ${REAL_DOMAIN_ID}"
echo "  Left arm:    ${LEFT_TOPIC}"
echo "  Right arm:   ${RIGHT_TOPIC}"
[[ -n "${LEFT_GRIPPER_TOPIC}" ]]  && echo "  Left gripper:  ${LEFT_GRIPPER_TOPIC}"
[[ -n "${RIGHT_GRIPPER_TOPIC}" ]] && echo "  Right gripper: ${RIGHT_GRIPPER_TOPIC}"

exec python3 "${SCRIPT_DIR}/real_robot/real_robot_follower.py" "${args[@]}"
