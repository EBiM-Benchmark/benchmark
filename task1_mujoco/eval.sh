#!/usr/bin/env bash
# One-click ManipulationNet eval stack (Docker), Ubuntu / WSL2:
#   ./eval.sh sim      # build (first run) + start the simulator with --mnet
#   ./eval.sh gamepad  # same, driven by a gamepad (native Linux only)
#   ./eval.sh client   # in a SECOND terminal: interactive mnet client
#   ./eval.sh build    # rebuild the image only
#   ./eval.sh down     # stop everything
#
# Before the first eval: edit mnet_client-ros_2/config/team_config.json
#   camera_image_topic=/mujoco/camera/image_raw, autonomy_level=0,
#   file_dir=/ws/out
# Native Linux (not WSL2) additionally needs:  xhost +local:docker
set -e
cd "$(dirname "$0")"
COMPOSE=(docker compose -f robotiq_duo_full_scene_minimal_core/release/compose.yaml)

# NVIDIA container runtime present -> merge the GPU passthrough overlay
# automatically (no compose.yaml editing). See compose.gpu.yaml for why it
# is a separate file and why services start via `up -d` + attach below.
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    COMPOSE+=(-f robotiq_duo_full_scene_minimal_core/release/compose.gpu.yaml)
    echo "[eval] NVIDIA container runtime detected - GPU passthrough enabled"
fi

# GPU reservations (deploy:) only apply with `compose up`, but plain `up`
# never attaches stdin and the scored session needs typed input
# ('code <TEXT>'). So: start detached, then `docker attach` the terminal.
run_attached() {
    "${COMPOSE[@]}" up -d "$1"
    cid=""
    for _ in $(seq 20); do
        cid=$("${COMPOSE[@]}" ps -q "$1")
        [ -n "$cid" ] && break
        sleep 0.5
    done
    if [ -z "$cid" ]; then
        echo "[eval] $1 did not start; recent logs:"
        "${COMPOSE[@]}" logs --tail 50 "$1" || true
        exit 1
    fi
    trap 'docker stop "$cid" >/dev/null 2>&1 || true' EXIT
    echo "[eval] attached to $1 - type 'code <TEXT>' HERE when the client shows the one-time code (Ctrl+C stops the sim)"
    sleep 1
    docker logs "$cid" 2>&1 || true
    docker attach "$cid"
}

# WSL2/WSLg only: steer Mesa's D3D12 translation layer to a discrete GPU
# when present (the integrated GPU's OpenGL support can segfault mid-render)
if [ -n "$WSL_DISTRO_NAME" ] && [ -z "$MESA_D3D12_DEFAULT_ADAPTER_NAME" ] && command -v powershell.exe >/dev/null 2>&1; then
    dgpu=$(powershell.exe -NoProfile -Command \
        "Get-CimInstance Win32_VideoController | Select-Object -ExpandProperty Name" \
        2>/dev/null | grep -Ei "NVIDIA|AMD|Radeon|GeForce|RTX" | head -1 | tr -d '\r')
    if [ -n "$dgpu" ]; then
        export MESA_D3D12_DEFAULT_ADAPTER_NAME="$dgpu"
        echo "[eval] WSL: rendering on discrete GPU ($dgpu) instead of the integrated one"
    fi
fi

# WSL2: an SSH-forwarded DISPLAY (localhost:N.0) left over in the shell
# points at a non-existent X server and kills GLFW ("Failed to open
# display"); WSLg's real display is :0. Only that pattern is rewritten.
if [ -n "$WSL_DISTRO_NAME" ] && [[ "$DISPLAY" == localhost:* ]]; then
    echo "[eval] WSL: DISPLAY=$DISPLAY looks SSH-forwarded; using WSLg's :0 instead"
    export DISPLAY=:0
fi

case "${1:-sim}" in
    build)   "${COMPOSE[@]}" build ;;
    sim)     "${COMPOSE[@]}" build sim && run_attached sim ;;
    # gamepad passthrough only exists on native Linux (not WSL2)
    gamepad) "${COMPOSE[@]}" build sim && run_attached sim-gamepad ;;
    client)  "${COMPOSE[@]}" run --rm client ;;
    down)    "${COMPOSE[@]}" down ;;
    *) echo "usage: ./eval.sh [sim|gamepad|client|build|down]"; exit 1 ;;
esac
