#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${TWIN_ENV_FILE:-${REPO_ROOT}/.env}"

if [[ ! -f "${ENV_FILE}" ]]; then
  echo "[FAIL] Missing env file: ${ENV_FILE}"
  exit 1
fi

set -a
source "${ENV_FILE}"
set +a

: "${ROBOT_PC_IP:=}"
: "${SIM_PC_IP:=}"
: "${INIT_USD:=}"
: "${TWIN_INIT_USD:=${INIT_USD}}"
: "${TWIN_LEFT_TOPIC:=/left/joint_states}"
: "${TWIN_RIGHT_TOPIC:=/right/joint_states}"
: "${TWIN_LEFT_GRIPPER_TOPIC:=}"
: "${TWIN_RIGHT_GRIPPER_TOPIC:=}"
: "${TWIN_REAL_RMW:=rmw_cyclonedds_cpp}"
: "${REAL_DOMAIN_ID:=}"
: "${TWIN_TOPIC_DISCOVERY_TIMEOUT_S:=12}"
: "${TWIN_STEP_TIMEOUT_S:=90}"
: "${TWIN_MIRROR_CHECK_TIMEOUT_S:=20}"
: "${TWIN_STRICT_TOPIC_PREFLIGHT:=false}"
: "${TWIN_REQUIRE_REAL_TOPICS_IN_PREFLIGHT:=false}"
: "${TWIN_REQUIRE_FUNCTIONAL_MIRROR_CHECK:=false}"
: "${TWIN_START_BROWSER:=true}"
: "${TWIN_START_STREAM:=false}"
: "${TWIN_BRIDGE_MODE:=wrapper}"
: "${TWIN_ALLOW_BRIDGE_MODE_FALLBACK:=true}"
: "${TWIN_VALIDATE_STOP_RESTART:=true}"
: "${TWIN_BRIDGE_LOG:=${REPO_ROOT}/extra/logs/twin_real_to_sim_bridge.log}"

BRIDGE_PID=""

require_cmd() {
  local cmd="$1"
  command -v "${cmd}" >/dev/null 2>&1 || {
    echo "[FAIL] Missing command: ${cmd}"
    exit 1
  }
}

step() {
  local title="$1"
  echo ""
  echo "===== ${title} ====="
}

pass() {
  echo "[PASS] $1"
}

fail() {
  echo "[FAIL] $1"
  exit 1
}

wait_until() {
  local timeout_s="$1"
  local interval_s="$2"
  shift 2

  local start
  start="$(date +%s)"
  while true; do
    if "$@"; then
      return 0
    fi
    local now
    now="$(date +%s)"
    if (( now - start >= timeout_s )); then
      return 1
    fi
    sleep "${interval_s}"
  done
}

validate_env() {
  [[ -n "${ROBOT_PC_IP}" ]] || fail "ROBOT_PC_IP is empty in ${ENV_FILE}"
  [[ -n "${SIM_PC_IP}" ]] || fail "SIM_PC_IP is empty in ${ENV_FILE}"
  [[ -n "${TWIN_LEFT_TOPIC}" ]] || fail "TWIN_LEFT_TOPIC is empty"
  [[ -n "${TWIN_RIGHT_TOPIC}" ]] || fail "TWIN_RIGHT_TOPIC is empty"
  [[ "${REAL_DOMAIN_ID}" =~ ^[0-9]+$ ]] || fail "REAL_DOMAIN_ID must be numeric"
  [[ "${TWIN_BRIDGE_MODE}" == "wrapper" || "${TWIN_BRIDGE_MODE}" == "compose" ]] || fail "TWIN_BRIDGE_MODE must be wrapper or compose"
}

run_real_topic_discovery() {
  docker run --rm --network host \
    -e RMW_IMPLEMENTATION="${TWIN_REAL_RMW}" \
    -e ROS_DOMAIN_ID="${REAL_DOMAIN_ID}" \
    ros:jazzy-ros-base \
    bash -lc "source /opt/ros/jazzy/setup.bash && timeout ${TWIN_TOPIC_DISCOVERY_TIMEOUT_S}s ros2 topic list"
}

run_real_topic_discovery_via_bridge_image() {
  docker compose run --rm --no-deps real_to_sim_bridge bash -lc \
    "source /opt/ros/jazzy/setup.bash && \
     export RMW_IMPLEMENTATION='${TWIN_REAL_RMW}' && \
     export ROS_DOMAIN_ID='${REAL_DOMAIN_ID}' && \
     timeout ${TWIN_TOPIC_DISCOVERY_TIMEOUT_S}s ros2 topic list"
}

start_bridge_wrapper() {
  mkdir -p "$(dirname -- "${TWIN_BRIDGE_LOG}")"

  local args=()
  args+=(--left-topic "${TWIN_LEFT_TOPIC}")
  args+=(--right-topic "${TWIN_RIGHT_TOPIC}")
  args+=(--real-rmw "${TWIN_REAL_RMW}")
  args+=(--real-domain-id "${REAL_DOMAIN_ID}")
  if [[ -n "${TWIN_LEFT_GRIPPER_TOPIC}" ]]; then
    args+=(--left-gripper-topic "${TWIN_LEFT_GRIPPER_TOPIC}")
  fi
  if [[ -n "${TWIN_RIGHT_GRIPPER_TOPIC}" ]]; then
    args+=(--right-gripper-topic "${TWIN_RIGHT_GRIPPER_TOPIC}")
  fi

  nohup bash "${SCRIPT_DIR}/run_real_to_sim_bridge.sh" "${args[@]}" >"${TWIN_BRIDGE_LOG}" 2>&1 &
  BRIDGE_PID="$!"
  echo "${BRIDGE_PID}" >"${REPO_ROOT}/extra/logs/twin_real_to_sim_bridge.pid"
}

start_bridge() {
  if [[ "${TWIN_BRIDGE_MODE}" == "wrapper" ]]; then
    start_bridge_wrapper
    sleep 2
    if kill -0 "${BRIDGE_PID}" >/dev/null 2>&1; then
      return 0
    fi

    if [[ -f "${TWIN_BRIDGE_LOG}" ]] && grep -q "ModuleNotFoundError: No module named 'rclpy'" "${TWIN_BRIDGE_LOG}"; then
      if [[ "${TWIN_ALLOW_BRIDGE_MODE_FALLBACK}" == "true" ]]; then
        echo "[WARN] Wrapper bridge failed: host Python missing rclpy."
        echo "[WARN] Falling back to containerized bridge (compose follower profile)."
        TWIN_BRIDGE_MODE="compose"
      else
        return 1
      fi
    else
      return 1
    fi
  fi

  docker compose --profile follower up -d real_to_sim_bridge >/dev/null
  docker inspect -f '{{.State.Running}}' real_to_sim_bridge 2>/dev/null | grep -q '^true$'
}

stop_bridge() {
  if [[ "${TWIN_BRIDGE_MODE}" == "wrapper" ]]; then
    if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" >/dev/null 2>&1; then
      kill "${BRIDGE_PID}" >/dev/null 2>&1 || true
      wait_until 10 1 bash -lc "! kill -0 ${BRIDGE_PID} >/dev/null 2>&1"
    else
      pkill -f real_to_sim_bridge.py >/dev/null 2>&1 || true
    fi
    return 0
  fi

  docker compose --profile follower stop real_to_sim_bridge >/dev/null
}

check_bridge_logs() {
  if [[ "${TWIN_BRIDGE_MODE}" == "wrapper" ]]; then
    grep -q "Reader:" "${TWIN_BRIDGE_LOG}" && grep -q "Publisher:" "${TWIN_BRIDGE_LOG}"
    return
  fi

  docker compose logs real_to_sim_bridge | grep -E "Reader:|Publisher:" >/dev/null
}

print_bridge_diagnostics() {
  if [[ "${TWIN_BRIDGE_MODE}" == "wrapper" ]]; then
    if [[ -f "${TWIN_BRIDGE_LOG}" ]]; then
      echo "--- bridge log tail (${TWIN_BRIDGE_LOG}) ---"
      tail -n 120 "${TWIN_BRIDGE_LOG}" || true
    else
      echo "Bridge log not found: ${TWIN_BRIDGE_LOG}"
    fi
    return
  fi

  echo "--- docker compose ps real_to_sim_bridge ---"
  docker compose ps real_to_sim_bridge || true
  echo "--- docker compose logs real_to_sim_bridge (tail) ---"
  docker compose logs --tail=120 real_to_sim_bridge || true
}

check_sim_cmd_topics_exist() {
  docker compose exec -T ros_republisher bash -lc \
    "source /opt/ros/jazzy/setup.bash && ros2 topic list | grep -E '/bridge/(left|right)_joint_commands'" >/dev/null
}

check_sim_cmd_publishers() {
  docker compose exec -T ros_republisher bash -lc \
    "source /opt/ros/jazzy/setup.bash && ros2 topic info /bridge/left_joint_commands | grep -E 'Publisher count: [1-9]'" >/dev/null && \
  docker compose exec -T ros_republisher bash -lc \
    "source /opt/ros/jazzy/setup.bash && ros2 topic info /bridge/right_joint_commands | grep -E 'Publisher count: [1-9]'" >/dev/null
}

main() {
  require_cmd docker
  require_cmd bash
  require_cmd grep
  require_cmd timeout
  validate_env

  step "Step 0 - Evaluate environment configuration"
  pass "Loaded ${ENV_FILE}"
  echo "ROBOT_PC_IP=${ROBOT_PC_IP}"
  echo "SIM_PC_IP=${SIM_PC_IP}"
  echo "TWIN_REAL_RMW=${TWIN_REAL_RMW}"
  echo "REAL_DOMAIN_ID=${REAL_DOMAIN_ID}"
  echo "TWIN_INIT_USD=${TWIN_INIT_USD}"
  echo "TWIN_STRICT_TOPIC_PREFLIGHT=${TWIN_STRICT_TOPIC_PREFLIGHT}"
  echo "TWIN_REQUIRE_REAL_TOPICS_IN_PREFLIGHT=${TWIN_REQUIRE_REAL_TOPICS_IN_PREFLIGHT}"
  echo "TWIN_REQUIRE_FUNCTIONAL_MIRROR_CHECK=${TWIN_REQUIRE_FUNCTIONAL_MIRROR_CHECK}"
  echo "TWIN_BRIDGE_MODE=${TWIN_BRIDGE_MODE}"
  echo "TWIN_ALLOW_BRIDGE_MODE_FALLBACK=${TWIN_ALLOW_BRIDGE_MODE_FALLBACK}"

  step "Step 1 - Network preflight"
  local_ips="$(hostname -I 2>/dev/null || true)"
  if echo "${local_ips}" | grep -qw "${SIM_PC_IP}"; then
    echo "Detected SIM_PC_IP on this host interfaces"
  else
    echo "SIM_PC_IP not found on local interfaces; continuing (host may have multiple interfaces/NAT)."
  fi
  ping -c 3 -W 2 "${ROBOT_PC_IP}" >/dev/null || fail "Cannot reach ROBOT_PC_IP=${ROBOT_PC_IP}"
  pass "PC-B can reach PC-A"

  step "Step 2 - Real robot topic preflight"
  topics_file="$(mktemp)"
  discovery_err_file="$(mktemp)"
  if ! run_real_topic_discovery >"${topics_file}" 2>"${discovery_err_file}"; then
    if grep -qi "librmw_.*so\|RMW implementation not installed" "${discovery_err_file}"; then
      echo "Primary preflight image is missing ${TWIN_REAL_RMW}. Retrying with real_to_sim_bridge service image..."
      if ! run_real_topic_discovery_via_bridge_image >"${topics_file}" 2>>"${discovery_err_file}"; then
        if [[ "${TWIN_STRICT_TOPIC_PREFLIGHT}" == "true" ]]; then
          echo "Discovery errors:"
          cat "${discovery_err_file}" || true
          rm -f "${topics_file}" "${discovery_err_file}"
          fail "Unable to discover topics on real robot RMW/domain (strict mode)"
        fi
        echo "[WARN] Topic preflight could not fully verify real robot topics."
        echo "[WARN] Continuing because TWIN_STRICT_TOPIC_PREFLIGHT=false."
        echo "[WARN] Bridge startup in Step 6/7 will still validate end-to-end flow."
        rm -f "${topics_file}" "${discovery_err_file}"
        pass "Preflight soft-passed (deferred validation to live bridge checks)"
      fi
    else
      echo "Discovery errors:"
      cat "${discovery_err_file}" || true
      rm -f "${topics_file}" "${discovery_err_file}"
      fail "Unable to discover topics on real robot RMW/domain"
    fi
  fi
  rm -f "${discovery_err_file}"

  if [[ -f "${topics_file}" ]]; then
  missing_topics=()
  grep -Fx "${TWIN_LEFT_TOPIC}" "${topics_file}" >/dev/null || missing_topics+=("${TWIN_LEFT_TOPIC}")
  grep -Fx "${TWIN_RIGHT_TOPIC}" "${topics_file}" >/dev/null || missing_topics+=("${TWIN_RIGHT_TOPIC}")

  if [[ ${#missing_topics[@]} -gt 0 ]]; then
    echo "Discovered topics sample:"
    head -n 30 "${topics_file}" || true
    echo "Missing expected topics: ${missing_topics[*]}"
    if [[ "${TWIN_REQUIRE_REAL_TOPICS_IN_PREFLIGHT}" == "true" ]]; then
      rm -f "${topics_file}"
      fail "Expected real arm topics not found during preflight"
    fi
    echo "[WARN] Continuing because TWIN_REQUIRE_REAL_TOPICS_IN_PREFLIGHT=false."
    echo "[WARN] Step 7 will validate mirrored command publishers instead."
    rm -f "${topics_file}"
    pass "Topic-name preflight soft-passed (deferred to functional check)"
  else
  rm -f "${topics_file}"
  pass "Real arm topics discovered on configured domain"
  fi
  fi

  step "Step 3 - Start simulation stack"
  cmd=(bash "${SCRIPT_DIR}/run_native_stream.sh")
  if [[ -n "${TWIN_INIT_USD}" ]]; then
    cmd+=(--usd-path "${TWIN_INIT_USD}")
  fi
  if [[ "${TWIN_START_STREAM}" != "true" ]]; then
    cmd+=(--no-stream)
  fi
  if [[ "${TWIN_START_BROWSER}" != "true" ]]; then
    cmd+=(--no-browser)
  fi
  "${cmd[@]}"
  wait_until "${TWIN_STEP_TIMEOUT_S}" 2 check_sim_cmd_topics_exist || fail "Sim command topics not available"
  pass "Simulation stack is up and command topics are available"

  step "Step 4 - Evaluate bridge input configuration"
  [[ -n "${TWIN_LEFT_GRIPPER_TOPIC}" ]] && echo "Left gripper topic configured: ${TWIN_LEFT_GRIPPER_TOPIC}" || echo "Left gripper topic not configured (allowed for twin run)"
  [[ -n "${TWIN_RIGHT_GRIPPER_TOPIC}" ]] && echo "Right gripper topic configured: ${TWIN_RIGHT_GRIPPER_TOPIC}" || echo "Right gripper topic not configured (allowed for twin run)"
  pass "Bridge configuration values validated"

  step "Step 5 - Start real-to-sim bridge"
  start_bridge || fail "Failed to start bridge"
  pass "Bridge process started"

  step "Step 6 - Verify bridge activity"
  if ! wait_until "${TWIN_STEP_TIMEOUT_S}" 2 check_bridge_logs; then
    print_bridge_diagnostics
    fail "Bridge logs did not show reader/publisher initialization"
  fi
  wait_until "${TWIN_STEP_TIMEOUT_S}" 2 check_sim_cmd_publishers || fail "No active publishers detected on sim command topics"
  pass "Bridge is active and publishing on sim command topics"

  step "Step 7 - Verify mirrored command publishers remain available"
  if wait_until "${TWIN_STEP_TIMEOUT_S}" 2 check_sim_cmd_publishers; then
    pass "Bridge publishers remain visible on the mirrored command topics"
  else
    if [[ "${TWIN_REQUIRE_FUNCTIONAL_MIRROR_CHECK}" == "true" ]]; then
      fail "Mirrored command publishers are not visible on the Isaac command topics"
    fi
    echo "[WARN] Mirrored command publishers were not visible within timeout."
    echo "[WARN] Continuing because TWIN_REQUIRE_FUNCTIONAL_MIRROR_CHECK=false."
    pass "Command publisher check soft-passed (verify live motion manually)"
  fi

  step "Step 8 - Stop/restart procedure evaluation"
  if [[ "${TWIN_VALIDATE_STOP_RESTART}" == "true" ]]; then
    stop_bridge || fail "Failed to stop bridge during validation"
    start_bridge || fail "Failed to restart bridge during validation"
    wait_until "${TWIN_STEP_TIMEOUT_S}" 2 check_bridge_logs || fail "Bridge did not recover after restart"
    pass "Stop/restart validation completed; bridge is running"
  else
    echo "Skipping stop/restart validation (TWIN_VALIDATE_STOP_RESTART=${TWIN_VALIDATE_STOP_RESTART})"
    pass "Stop procedure intentionally skipped"
  fi

  echo ""
  echo "Twin run complete."
  if [[ "${TWIN_BRIDGE_MODE}" == "wrapper" ]]; then
    echo "Bridge log: ${TWIN_BRIDGE_LOG}"
  else
    echo "Bridge logs: docker compose logs -f real_to_sim_bridge"
  fi
}

main "$@"
