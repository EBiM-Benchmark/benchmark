# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
#
# ROS 2 Jazzy + LeRobot environment for tools/record_task2.py (the Task 2
# demonstration recorder). CPU-only torch keeps the image reasonably small;
# the recorder does no training, only dataset writing/encoding.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ffmpeg \
        python3-pip \
        ros-jazzy-vision-msgs \
        ros-jazzy-rosgraph-msgs \
    && rm -rf /var/lib/apt/lists/*

# --ignore-installed: Debian-installed python packages (packaging, numpy,
# ...) have no RECORD file, so pip cannot uninstall them during upgrades —
# install fresh copies over them instead. The PyTorch CPU extra-index keeps
# lerobot's torch dependency on the (+cpu) wheels.
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
        --extra-index-url https://download.pytorch.org/whl/cpu \
        "lerobot[dataset,viz]" \
        opencv-python-headless

CMD ["bash"]
