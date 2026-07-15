FROM ros:jazzy-ros-base

# Install geometry_msgs for WrenchStamped support
RUN apt-get update && apt-get install -y \
    ros-jazzy-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy the service sources
COPY . /workspace/services/browser_controller/

# Make scripts and services on the Python path so stack_config / isaac_camera_service
# imports work when the assets volume is mounted
ENV PYTHONPATH=/workspace/services/browser_controller:/workspace/services:/workspace/scripts

EXPOSE 8090

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && python3 /workspace/services/browser_controller/app.py"]
