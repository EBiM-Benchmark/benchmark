#!/usr/bin/env bash
# One-click setup for the fully Docker-free scored evaluation on Ubuntu,
# ANY version - ROS 2 Humble comes from RoboStack (conda), not apt, so
# this doesn't need Ubuntu 22.04 (apt Humble packages target Jammy only)
# and never touches an existing ROS 1 (e.g. Noetic) install: everything
# lives in one isolated conda env. Practice mode (no --mnet) needs none
# of this: see start.sh. The Docker path (eval.sh) is already one-click
# and needs no setup script of its own.
# Safe to re-run any time (after git pull, or to rebuild the workspace):
# every step below checks before it acts.
#
#   ./setup_eval.sh
#   conda activate ros-humble && source ros_ws/install/setup.bash
#   python robotiq_duo_full_scene_minimal_core/main.py --input keyboard --mnet
#   ros2 run mnet_client local_test
set -e
cd "$(dirname "$0")"
REPO_DIR="$(pwd)"
OUT_DIR="$REPO_DIR/mnet_out_native"
ROS_ENV_NAME="${EBIM_ROS_ENV_NAME:-ros-humble}"

# bootstrap conda if missing (same logic as start.sh, duplicated so this
# script has no hard dependency on having run start.sh first)
for base in "$HOME/miniconda3" "$HOME/anaconda3"; do
    if ! command -v conda >/dev/null 2>&1 && [ -x "$base/bin/conda" ]; then
        export PATH="$base/bin:$PATH"
    fi
done
if ! command -v conda >/dev/null 2>&1; then
    echo "[setup] conda not found - installing Miniconda to ~/miniconda3 ..."
    curl -fsSL -o /tmp/miniconda.sh \
        https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash /tmp/miniconda.sh -b -u -p "$HOME/miniconda3"
    "$HOME/miniconda3/bin/conda" init bash >/dev/null
    export PATH="$HOME/miniconda3/bin:$PATH"
fi
# shellcheck disable=SC1091
source "$(conda info --base)/etc/profile.d/conda.sh"

if conda env list | grep -qE "^${ROS_ENV_NAME}[[:space:]]"; then
    echo "[setup] $ROS_ENV_NAME env already exists - skipping conda create"
else
    echo "[setup] creating the $ROS_ENV_NAME conda env - downloads several GB, a few minutes..."
    conda create -y -n "$ROS_ENV_NAME" --override-channels -c robostack-staging -c conda-forge \
        python=3.11 ros-humble-ros-base ros-humble-cv-bridge colcon-common-extensions
fi
conda activate "$ROS_ENV_NAME"

echo "[setup] installing sim + client Python dependencies..."
pip install -q mujoco==3.9.0 "numpy>=1.24,<2" glfw==2.10.0 pygame==2.6.1 \
    "pillow>=10" pyopenxr==1.1.5301 PyOpenGL==3.1.10 openvr==2.12.1401 \
    opencv-python "pydantic>=2,<3" requests tqdm pupil-apriltags pybullet \
    python-xlib

echo "[setup] copying the client and ros_teleop publishers into ros_ws/src..."
mkdir -p ros_ws/src
for pkg in mnet_client-ros_2:mnet_client \
           teleop_ros2/keyboard_teleop_publisher:keyboard_teleop_publisher \
           teleop_ros2/gamepad_teleop_publisher:gamepad_teleop_publisher; do
    src="${pkg%%:*}"; name="${pkg##*:}"
    ln -sfn "$REPO_DIR/$src" "ros_ws/src/$name"
done

echo "[setup] colcon build (a few minutes on first run)..."
( cd ros_ws && colcon build )

echo "[setup] applying cross-platform fixes (file_dir, stdin)..."
python3 "$REPO_DIR/robotiq_duo_full_scene_minimal_core/release/mnet_client_postpatch.py" \
    "$REPO_DIR/ros_ws" "$OUT_DIR"

echo
echo "[setup] done. Results will land in $OUT_DIR. Run the eval with:"
echo "  conda activate $ROS_ENV_NAME && source ros_ws/install/setup.bash"
echo "  cd robotiq_duo_full_scene_minimal_core && python main.py --input keyboard --mnet"
echo "  # second terminal (same two activate/source lines first):"
echo "  ros2 run mnet_client local_test"
echo "Re-run this script any time after pulling updates or before a submission."
