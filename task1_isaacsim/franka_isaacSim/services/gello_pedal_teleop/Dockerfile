FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY FRANKA_GELLO_ROS2/requirements.txt /tmp/gello_requirements.txt
RUN pip3 install --break-system-packages -r /tmp/gello_requirements.txt evdev

WORKDIR /workspace/FRANKA_GELLO_ROS2
