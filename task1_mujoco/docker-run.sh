#!/usr/bin/env bash
# The one Docker entry point (native Linux). Practice and eval share a
# single image; eval is just a flag. Driving the sim needs no ROS 2 or
# conda/mujoco install on the host at all.
#
#   ./docker-run.sh                          # keyboard (default)
#   ./docker-run.sh --input gamepad          # gamepad (auto /dev/input passthrough)
#   ./docker-run.sh --input vr               # VR via WiVRn on the host (auto-detected)
#   ./docker-run.sh --input ros_teleop       # sim consuming ROS 2 teleop topics
#   ./docker-run.sh publisher keyboard       # 2nd terminal: ros_teleop publisher
#   ./docker-run.sh publisher gamepad        #   (add --pattern 60 for a self-test)
#   ./docker-run.sh --no-viewer              # headless self-check
#   ./docker-run.sh <flags> --mnet           # ManipulationNet eval bridge
#   ./docker-run.sh client                   # 2nd terminal: official mnet client
#   ./docker-run.sh shell                    # ROS-sourced shell inside the image
#   ./docker-run.sh build                    # rebuild the image only
#   ./docker-run.sh down                     # stop/remove containers
#
# The image is rebuilt incrementally before every sim start, so a `git pull`
# is picked up automatically (seconds when nothing changed).
#
# VR needs WiVRn (Flathub) running on the host with the headset connected —
# the sim then reaches it through its IPC socket (see compose.vr.yaml).
# Quest Link / SteamVR / GELLO cannot run inside any container (direct host
# device/driver access) — use start.sh natively for those.
set -e
cd "$(dirname "$0")"
COMPOSE=(docker compose -f robotiq_duo_full_scene_minimal_core/release/compose.yaml)

# NVIDIA container runtime present -> merge the GPU passthrough overlay
# automatically. See compose.gpu.yaml for why it is a separate file and why
# it sets `runtime: nvidia` (not just `deploy:` reservations, which
# `compose run` silently ignores).
if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    COMPOSE+=(-f robotiq_duo_full_scene_minimal_core/release/compose.gpu.yaml)
    echo "[docker-run] NVIDIA container runtime detected - GPU passthrough enabled"
fi

# /dev/input present (native Linux) -> merge the gamepad passthrough overlay.
# Separate file because a `devices:` entry for a missing host path aborts
# container start (WSL2 has no /dev/input at all).
if [ -d /dev/input ]; then
    COMPOSE+=(-f robotiq_duo_full_scene_minimal_core/release/compose.input.yaml)
fi

# Flathub WiVRn present -> merge the VR passthrough overlay (--input vr).
# With WiVRn the headset is a streamed device: the sim in the container only
# needs the WiVRn IPC socket and its OpenXR client library - see
# compose.vr.yaml for the details (including why this is also the ONLY VR
# path on old-glibc hosts like Ubuntu 20.04).
for _wivrn_dir in "$HOME/.local/share/flatpak/app/io.github.wivrn.wivrn" \
                  /var/lib/flatpak/app/io.github.wivrn.wivrn; do
    _wivrn_json="$_wivrn_dir/current/active/files/share/openxr/1/openxr_wivrn.json"
    if [ -f "$_wivrn_json" ] && [ -n "$XDG_RUNTIME_DIR" ]; then
        export WIVRN_APP_DIR="$_wivrn_dir"
        export WIVRN_RUNTIME_JSON="$_wivrn_json"
        COMPOSE+=(-f robotiq_duo_full_scene_minimal_core/release/compose.vr.yaml)
        echo "[docker-run] WiVRn detected - VR passthrough enabled (--input vr)"
        break
    fi
done

# native Linux: let the container open windows on this display (idempotent)
if [ -z "$WSL_DISTRO_NAME" ] && [ -n "$DISPLAY" ] && command -v xhost >/dev/null 2>&1; then
    xhost +local:docker >/dev/null 2>&1 || true
fi

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
    build)  "${COMPOSE[@]}" build sim ;;
    down)   "${COMPOSE[@]}" down ;;
    client) "${COMPOSE[@]}" run --rm client ;;
    shell)  "${COMPOSE[@]}" run --rm sim ebim shell ;;
    publisher)
        shift
        dev="${1:-keyboard}"
        [ $# -gt 0 ] && shift
        "${COMPOSE[@]}" run --rm sim ebim ros2 run "${dev}_teleop_publisher" "${dev}_teleop_publisher" "$@"
        ;;
    *)
        "${COMPOSE[@]}" build sim
        "${COMPOSE[@]}" run --rm sim ebim python3 main.py "$@"
        ;;
esac
