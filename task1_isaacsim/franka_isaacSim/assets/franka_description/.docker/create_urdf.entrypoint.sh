#!/bin/bash

args=$*
shift $#

source /ros_entrypoint.sh

cd /workspaces
PACKAGES=(franka_description)
if [ -d /workspaces/src/robotiq_description ]; then
    PACKAGES+=(robotiq_description)
fi
colcon build --packages-select "${PACKAGES[@]}" > /dev/null
source install/setup.bash

cd src/franka_description

python3 scripts/create_urdf.py $args