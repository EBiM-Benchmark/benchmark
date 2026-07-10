#!/usr/bin/env bash
# One-click Docker test, native Linux: driving the sim without needing
# ROS 2 or any native conda/mujoco install on the host at all.
#   ./docker-run.sh                    # keyboard (default)
#   ./docker-run.sh --input gamepad    # gamepad (native Linux only, see below)
#   ./docker-run.sh --no-viewer        # headless self-check
#   ./docker-run.sh build              # rebuild the image only
#   ./docker-run.sh down               # stop/remove the container
#
# For the ManipulationNet ROS 2 eval stack instead, use ./eval.sh.
#
# Native Linux (not WSL2) needs once per session:  xhost +local:docker
# Gamepad passthrough only works on native Linux (WSL2 has no /dev/input
# passthrough): uncomment the `devices:` block for the runtime service in
# robotiq_duo_full_scene_minimal_core/release/compose.yaml.
# VR cannot run inside any container (needs direct host device access) —
# use start.sh/start.bat natively for VR testing.
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
        echo "[docker-run] WSL: rendering on discrete GPU ($dgpu) instead of the integrated one"
    fi
fi

# WSL2: an SSH-forwarded DISPLAY (localhost:N.0) left over in the shell
# points at a non-existent X server and kills GLFW ("Failed to open
# display"); WSLg's real display is :0. Only that pattern is rewritten.
if [ -n "$WSL_DISTRO_NAME" ] && [[ "$DISPLAY" == localhost:* ]]; then
    echo "[docker-run] WSL: DISPLAY=$DISPLAY looks SSH-forwarded; using WSLg's :0 instead"
    export DISPLAY=:0
fi

case "${1:-}" in
    build) "${COMPOSE[@]}" build runtime ;;
    down)  "${COMPOSE[@]}" down ;;
    *)     "${COMPOSE[@]}" run --rm runtime python3 main.py "$@" ;;
esac
