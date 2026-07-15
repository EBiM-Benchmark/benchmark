#!/bin/bash
set -e

# Resolve paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
FRANKA_DESC_DIR="$REPO_ROOT/assets/franka_description"
HOST_DIR="/workspace/assets/franka_description"

if [ ! -d "$FRANKA_DESC_DIR" ]; then
    echo "Error: franka_description submodule not found at $FRANKA_DESC_DIR"
    exit 1
fi

echo "Delegating to assets/franka_description/scripts/create_urdf.sh..."
echo "Running in: $FRANKA_DESC_DIR"

# Switch to submodule dir so docker context works
cd "$FRANKA_DESC_DIR"

# Check if user asked for help or provided arguments
if [ "$#" -eq 0 ]; then
    echo "No arguments provided. Showing help..."
    ./scripts/create_urdf.sh --help
    exit 0
fi

# Run the generation script
# We automatically inject --abs-path and --host-dir to ensure standard Isaac Sim compatibility
# checking if they are already provided to avoid duplication could be nice, but the script might handle it.
# Ideally we just pass "$@" and let the user control it, BUT the pathing for Isaac Sim is specific.

echo "NOTE: Automatically adding '--abs-path --host-dir $HOST_DIR' for Isaac Sim compatibility."
./scripts/create_urdf.sh "$@" --abs-path --host-dir "$HOST_DIR"
