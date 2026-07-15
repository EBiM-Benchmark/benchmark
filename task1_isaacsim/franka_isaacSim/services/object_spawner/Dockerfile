FROM ros:jazzy-ros-base

# Install Python YAML support
RUN apt-get update && apt-get install -y \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy the service sources
COPY . /workspace/services/object_spawner/

# Make services and scripts on Python path
ENV PYTHONPATH=/workspace/services/object_spawner:/workspace/services:/workspace/scripts

CMD ["bash"]
