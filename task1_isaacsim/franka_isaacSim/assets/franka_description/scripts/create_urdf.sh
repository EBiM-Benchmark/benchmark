#!/bin/bash

docker build -t urdf_creation \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    ./.docker

echo 

DOCKER_VOLUMES=(-v "$(pwd):/workspaces/src/franka_description")
ROBOTIQ_DESCRIPTION_DIR="$(pwd)/../ros2_robotiq_gripper/robotiq_description"
if [ -d "${ROBOTIQ_DESCRIPTION_DIR}" ]; then
    DOCKER_VOLUMES+=(-v "${ROBOTIQ_DESCRIPTION_DIR}:/workspaces/src/robotiq_description")
fi

docker run -u $(id -u) \
    "${DOCKER_VOLUMES[@]}" \
    -w /workspaces/src/franka_description \
    urdf_creation \
    .docker/create_urdf.entrypoint.sh $*
