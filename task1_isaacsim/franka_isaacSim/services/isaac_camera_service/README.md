# Isaac Camera Service

**Purpose**: Manages camera creation, configuration, and ROS2 publishing for Isaac Sim environments.

## Architecture

The Isaac Camera Service is responsible for:
- Loading camera configurations from embodiment folders
- Creating USD camera primitives in the Isaac Sim stage
- Configuring camera optical parameters (FOV, sensor size, focal length, etc.)
- Setting up ROS2 publishing graphs for camera images and info
- Validating camera configurations against data contracts

## Components

### `camera_config.py`
Configuration loading and normalization:
- Loads camera sensor configs from YAML files
- Normalizes camera specifications (resolution, optics, topics)
- Calculates focal length from horizontal FOV and sensor dimensions
- Validates camera contract alignment
- Provides camera feed configurations for ROS2 subscribers

### `camera_service.py`
Runtime camera integration:
- Resolves attachment frames in USD stage
- Creates camera primitives with proper transforms
- Builds ROS2 publishing OmniGraphs
- Manages camera lifecycle during simulation

## Configuration Structure

Camera configurations are stored in embodiment folders:
```
assets/embodiments/<embodiment>/camera_sensors.yaml
```

Example structure:
```yaml
cameras:
  left_wrist_camera:
    label: Left Wrist Camera
    device_model: Intel RealSense D405
    contract_video_key: wrist_left
    attachment_frame_name: left_fr3v2_d405_color_frame
    frame_id: left_fr3v2_d405_color_optical_frame
    camera_prim_name: rgb_sensor
    namespace: /isaac/left_wrist_camera
    render_resolution:
      width: 848
      height: 480
    publish_hz: 24.0
    horizontal_fov_deg: 87.0
    sensor_width_mm: 4.8
    local_translation_m: [0.0, 0.0, 0.0]
    local_rotation_deg: [0.0, 180.0, 0.0]
    # ... more parameters
```

## Usage

### From Isaac Bridge Session

```python
from services.isaac_camera_service import IsaacCameraService

# Initialize service
camera_service = IsaacCameraService()

# Attach configured cameras to the stage
summary = camera_service.attach_configured_cameras(
    stage=stage,
    config_path="/path/to/embodiment/camera_sensors.yaml",
    render_hz=60.0
)

print(f"Attached {summary['attached_count']} cameras")
```

### From Browser Controller or Other Services

```python
from services.isaac_camera_service import camera_feed_configs

# Get camera feed configurations for ROS2 subscription
feeds = camera_feed_configs(config_path="/path/to/camera_sensors.yaml")

for feed in feeds:
    print(f"Camera: {feed['label']}")
    print(f"  Topic: {feed['topic_name']}")
    print(f"  Resolution: {feed['width']}x{feed['height']}")
```

## Camera Creation Process

1. **Configuration Loading**: Load and parse camera YAML configuration
2. **Normalization**: Compute derived parameters (focal length, resolutions, etc.)
3. **Frame Resolution**: Find attachment frames in USD stage by name or path
4. **Prim Creation**: Create USD Camera primitives with proper transforms
5. **Graph Setup**: Build ROS2 publishing OmniGraphs for each camera
6. **Validation**: Verify contract compliance and report status

## Data Contract Integration

Cameras are validated against the data contract to ensure:
- All expected video keys have corresponding cameras
- Camera outputs match required format (uint8/HWC/RGB)
- No duplicate or missing camera mappings
- Frame IDs and attachment information are complete

## Migration Notes

This service consolidates camera functionality previously split across:
- `scripts/isaac_camera_config.py` (configuration)
- `scripts/isaac_camera_runtime.py` (runtime)

Camera parameters remain in embodiment folders, maintaining separation of
configuration from integration logic.
