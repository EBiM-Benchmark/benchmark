#!/bin/bash
source /opt/ros/jazzy/setup.bash
if [ -f /dependencies_ws/install/setup.bash ]; then
  source /dependencies_ws/install/setup.bash
fi
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
exec "$@"
