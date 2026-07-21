FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-jazzy-control-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages \
    dynamixel_sdk \
    evdev \
    pyserial \
    PyYAML \
    tyro

WORKDIR /workspace/teleoperation
