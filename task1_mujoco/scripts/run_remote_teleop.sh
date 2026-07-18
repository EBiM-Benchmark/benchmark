#!/usr/bin/env bash
# Remote MuJoCo teleop for AWS / headless hosts — analogue of Isaac
# `--livestream 1`. Starts a virtual X display, streams it over noVNC
# (browser), then launches the sim via docker-run.sh.
#
#   PUBLIC_IP=<ec2-public-ip> bash task1_mujoco/scripts/run_remote_teleop.sh
#   bash task1_mujoco/scripts/run_remote_teleop.sh --input gamepad
#   bash task1_mujoco/scripts/run_remote_teleop.sh --no-viewer   # skip VNC
#
# View from your laptop: http://<PUBLIC_IP>:6080/vnc.html
# Click the black desktop once, then use the sim's keyboard bindings.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DISPLAY_NUM="${DISPLAY_NUM:-99}"
export DISPLAY=":${DISPLAY_NUM}"
VNC_PORT="${VNC_PORT:-5900}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
GEOMETRY="${GEOMETRY:-1920x1080x24}"
RUN_DIR="${XDG_RUNTIME_DIR:-/tmp}/ebim-mujoco-remote"
PID_DIR="${RUN_DIR}/pids"
LOG_DIR="${RUN_DIR}/logs"

mkdir -p "${PID_DIR}" "${LOG_DIR}"

log() { echo "[remote-teleop] $*"; }

need_pkg() {
  local missing=()
  for p in xvfb x11vnc novnc websockify openbox; do
    if ! dpkg -s "$p" >/dev/null 2>&1; then
      missing+=("$p")
    fi
  done
  if ((${#missing[@]})); then
    log "installing packages: ${missing[*]}"
    sudo apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "${missing[@]}"
  fi
}

is_running() {
  local pid_file="$1"
  [[ -f "$pid_file" ]] || return 1
  local pid
  pid="$(cat "$pid_file")"
  kill -0 "$pid" 2>/dev/null
}

start_bg() {
  local name="$1"; shift
  local pid_file="${PID_DIR}/${name}.pid"
  local log_file="${LOG_DIR}/${name}.log"
  if is_running "$pid_file"; then
    log "${name} already running (pid $(cat "$pid_file"))"
    return 0
  fi
  "$@" >"$log_file" 2>&1 &
  echo $! >"$pid_file"
  log "started ${name} (pid $!, log ${log_file})"
}

stop_bg() {
  local name="$1"
  local pid_file="${PID_DIR}/${name}.pid"
  if is_running "$pid_file"; then
    kill "$(cat "$pid_file")" 2>/dev/null || true
    rm -f "$pid_file"
    log "stopped ${name}"
  fi
}

cleanup() {
  # leave the display stack up by default so reconnecting is easy;
  # pass CLEANUP=1 to tear it down when the sim exits
  if [[ "${CLEANUP:-0}" == "1" ]]; then
    stop_bg websockify
    stop_bg x11vnc
    stop_bg openbox
    stop_bg xvfb
  fi
}
trap cleanup EXIT

case "${1:-}" in
  down|stop)
    stop_bg websockify
    stop_bg x11vnc
    stop_bg openbox
    stop_bg xvfb
    log "remote display stack stopped"
    exit 0
    ;;
esac

# --no-viewer: just forward to docker-run (headless smoke, no VNC stack)
for arg in "$@"; do
  if [[ "$arg" == "--no-viewer" ]]; then
    log "forwarding --no-viewer (no remote display)"
    exec bash "${ROOT}/docker-run.sh" "$@"
  fi
done

need_pkg

start_display_stack() {
  # 1) virtual X
  if ! is_running "${PID_DIR}/xvfb.pid"; then
    start_bg xvfb Xvfb "${DISPLAY}" -screen 0 "${GEOMETRY}" -ac +extension GLX +render -noreset
    for _ in $(seq 1 50); do
      [[ -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]] && break
      sleep 0.1
    done
    if [[ ! -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]]; then
      log "ERROR: Xvfb failed to create ${DISPLAY} — see ${LOG_DIR}/xvfb.log"
      exit 1
    fi
  fi

  # allow the docker (root) sim process to open windows on this display
  xhost +local: >/dev/null 2>&1 || true
  xhost +local:root >/dev/null 2>&1 || true

  # 2) lightweight WM so GLFW windows map/focus correctly
  start_bg openbox env DISPLAY="${DISPLAY}" openbox --startup true

  # 3) VNC of the virtual display
  start_bg x11vnc x11vnc \
    -display "${DISPLAY}" \
    -rfbport "${VNC_PORT}" \
    -localhost \
    -forever \
    -shared \
    -nopw \
    -xkb \
    -ncache 10 \
    -ncache_cr \
    -wait 10 \
    -defer 10

  # 4) noVNC (browser client)
  local novnc_web=""
  for candidate in /usr/share/novnc /usr/share/novnc/www; do
    if [[ -d "$candidate" ]]; then
      novnc_web="$candidate"
      break
    fi
  done
  if [[ -z "$novnc_web" ]]; then
    log "ERROR: noVNC web root not found (is the novnc package installed?)"
    exit 1
  fi
  start_bg websockify websockify \
    --web="${novnc_web}" \
    "0.0.0.0:${NOVNC_PORT}" \
    "localhost:${VNC_PORT}"
}

print_view_url() {
  local public_ip="${PUBLIC_IP:-}"
  if [[ -z "$public_ip" ]]; then
    public_ip="$(curl -4 -s --max-time 2 ifconfig.me 2>/dev/null || true)"
  fi
  local view_url="http://${public_ip:-<ec2-public-ip>}:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=remote"
  echo
  log "remote display ready on ${DISPLAY}"
  log "open in your browser:  ${view_url}"
  log "security group: inbound TCP ${NOVNC_PORT} from your IP"
  log "click the VNC desktop once, then drive the robot with the sim keybinds"
  echo
}

start_display_stack
print_view_url

if [[ "${1:-}" == "display" ]]; then
  log "display-only mode; sim not started (run again without 'display' to launch)"
  trap - EXIT
  exit 0
fi

cd "${ROOT}"
exec bash ./docker-run.sh "$@"