FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-opencv \
        python3-numpy \
        python3-yaml \
        ros-jazzy-vision-msgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ARG EVAL_VERSION=unknown
ENV EVAL_TASK2_VERSION_BUILD=${EVAL_VERSION}

# Keep the flat eval modules importable when the repo is bind-mounted.
ENV PYTHONPATH=/workspace/scripts/evaluation/task2

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && python3 /workspace/scripts/evaluation/task2/main.py --output-dir /output"]
