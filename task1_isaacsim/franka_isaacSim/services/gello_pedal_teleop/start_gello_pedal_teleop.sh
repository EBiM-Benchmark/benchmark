#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

cd /workspace/FRANKA_GELLO_ROS2
colcon build --symlink-install --packages-select franka_gello_state_publisher pedal_state_publisher
source install/setup.bash

python3 /workspace/FRANKA_GELLO_ROS2/scripts/gello_to_bridge.py &
gello_bridge_pid=$!

ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml &
gello_publisher_pid=$!

echo "GELLO teleop is running."
echo "Start the keyboard pedal publisher separately in an interactive terminal with:"
echo "  docker exec -it gello_pedal_teleop bash -lc 'cd /workspace/FRANKA_GELLO_ROS2 && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run pedal_state_publisher pedal_state_publisher'"

cleanup() {
  kill "${gello_bridge_pid}" "${gello_publisher_pid}" 2>/dev/null || true
  wait "${gello_bridge_pid}" "${gello_publisher_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n "${gello_bridge_pid}" "${gello_publisher_pid}"
