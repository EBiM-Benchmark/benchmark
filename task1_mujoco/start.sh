#!/usr/bin/env bash
# One-click start on Ubuntu / WSL2 (native, no Docker): installs Miniconda
# if missing, creates the env on first run, launches the teleop.
# Flags pass through:  ./start.sh --input gamepad
set -e
cd "$(dirname "$0")/robotiq_duo_full_scene_minimal_core"

# pick up an existing install that is not on PATH yet (e.g. the same
# terminal session that just installed it, before ~/.bashrc is re-sourced)
for base in "$HOME/miniconda3" "$HOME/anaconda3"; do
    if ! command -v conda >/dev/null 2>&1 && [ -x "$base/bin/conda" ]; then
        export PATH="$base/bin:$PATH"
    fi
done

if ! command -v conda >/dev/null 2>&1; then
    echo "[start] conda not found - installing Miniconda to ~/miniconda3 ..."
    wget -q --show-progress \
        https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh \
        -O /tmp/miniconda.sh
    # -u tolerates a leftover/partial ~/miniconda3 from an aborted install
    bash /tmp/miniconda.sh -b -u -p "$HOME/miniconda3"
    "$HOME/miniconda3/bin/conda" init bash >/dev/null
    export PATH="$HOME/miniconda3/bin:$PATH"
    echo "[start] Miniconda installed (new terminals will have conda on PATH)"
fi

# WSL2/WSLg only: on hybrid-graphics laptops Mesa's D3D12 translation layer
# (Dozen) can default to the integrated GPU, whose OpenGL support is
# incomplete enough to segfault mid-render. Steer it to a discrete GPU when
# one is present; a user-set MESA_D3D12_DEFAULT_ADAPTER_NAME always wins.
if [ -n "$WSL_DISTRO_NAME" ] && [ -z "$MESA_D3D12_DEFAULT_ADAPTER_NAME" ] && command -v powershell.exe >/dev/null 2>&1; then
    dgpu=$(powershell.exe -NoProfile -Command \
        "Get-CimInstance Win32_VideoController | Select-Object -ExpandProperty Name" \
        2>/dev/null | grep -Ei "NVIDIA|AMD|Radeon|GeForce|RTX" | head -1 | tr -d '\r')
    if [ -n "$dgpu" ]; then
        export MESA_D3D12_DEFAULT_ADAPTER_NAME="$dgpu"
        echo "[start] WSL: rendering on discrete GPU ($dgpu) instead of the integrated one"
    else
        echo "[start] WSL: no discrete GPU reported by Windows - rendering may fall back to"
        echo "[start]      the integrated GPU or software. To force one manually:"
        echo "[start]      export MESA_D3D12_DEFAULT_ADAPTER_NAME=\"<GPU name substring>\""
    fi
fi

# WSL2: an SSH-forwarded DISPLAY (localhost:N.0) left over in the shell
# points at a non-existent X server and kills GLFW ("Failed to open
# display"); WSLg's real display is :0. Only that pattern is rewritten.
if [ -n "$WSL_DISTRO_NAME" ] && [[ "$DISPLAY" == localhost:* ]]; then
    echo "[start] WSL: DISPLAY=$DISPLAY looks SSH-forwarded; using WSLg's :0 instead"
    export DISPLAY=:0
fi

# always report the renderer actually picked, so a slow run is diagnosable
# without a separate glxinfo step
if [ -n "$WSL_DISTRO_NAME" ] && command -v glxinfo >/dev/null 2>&1; then
    renderer=$(glxinfo -B 2>/dev/null | grep -i "OpenGL renderer" | sed 's/^ *//')
    [ -n "$renderer" ] && echo "[start] $renderer"
elif [ -n "$WSL_DISTRO_NAME" ]; then
    echo "[start] (install mesa-utils for renderer diagnostics: sudo apt install -y mesa-utils)"
fi

exec python3 start.py "$@"
