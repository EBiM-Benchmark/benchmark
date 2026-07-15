#!/bin/bash
set -e

. /opt/ros/jazzy/setup.sh

COLCON_WS=$(mktemp -d)
mkdir -p "${COLCON_WS}/src"
ln -sf "${WORKSPACE}" "${COLCON_WS}/src/franka_description"
cd "${COLCON_WS}"

colcon build --packages-select franka_description
colcon test --packages-select franka_description \
  --return-code-on-test-failure \
  --event-handlers console_direct+
