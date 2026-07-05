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
    sim)     "${COMPOSE[@]}" up --build sim ;;
    # gamepad passthrough only exists on native Linux (not WSL2)
    gamepad) "${COMPOSE[@]}" build sim && "${COMPOSE[@]}" run --rm sim-gamepad ;;
    client)  "${COMPOSE[@]}" run --rm client ;;
    down)    "${COMPOSE[@]}" down ;;
    *) echo "usage: ./eval.sh [sim|gamepad|client|build|down]"; exit 1 ;;
esac
