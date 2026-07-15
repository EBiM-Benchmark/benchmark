# Real-to-Sim Bridge Configuration

This directory contains the configuration-based real-to-sim bridge that forwards joint states from real Franka robots to the Isaac Sim environment.

## Configuration File

All topic mappings, domain settings, and RMW configurations are defined in **`bridge_config.yaml`**.

### Configuration Structure

```yaml
# Domain and RMW settings
real_robot:
  rmw_implementation: rmw_cyclonedds_cpp
  domain_id: 100
  cyclonedds_uri: file:///path/to/cyclonedds.xml  # optional

simulation:
  rmw_implementation: rmw_fastrtps_cpp
  domain_id: 0

# Topic mappings between real robot and bridge
mappings:
  - name: left_arm
    input_topic: /left/joint_states          # Real robot topic (domain 100)
    output_topic: /bridge/left_joint_commands  # Bridge output (domain 0)
    type: sensor_msgs/JointState
    num_joints: 7
    enabled: true
    expected_joint_names: [...]
    
  - name: left_gripper
    input_topic: /left/robotiq_2f85_controller/joint_states
    output_topic: /bridge/left_robotiq_joint_commands
    enabled: false  # Set to true when using physical grippers
    gripper_joint_index: 0
    gripper_opening_joint: left_robotiq_2f85_left_driver_joint

# Gripper normalization settings
gripper:
  driver_joint_closed: 0.0
  driver_joint_open: 0.8
  invert: false
```

## Usage

### Via Docker Compose (Recommended)

```bash
# Edit configuration if needed
vim services/real_to_sim_bridge/bridge_config.yaml

# Start the bridge
docker compose --profile follower up real_to_sim_bridge
```

### Via Shell Script

```bash
# Uses configuration from bridge_config.yaml
bash scripts/run_real_robot_bridge.sh

# Override CycloneDDS config only
bash scripts/run_real_robot_bridge.sh --cyclonedds-uri file:///path/to/cyclonedds.xml
```

### Direct Python Execution

```bash
# With default config
python3 services/real_to_sim_bridge/real_to_sim_bridge.py

# With custom config
python3 services/real_to_sim_bridge/real_to_sim_bridge.py --bridge-config /path/to/bridge_config.yaml
```

## Adding New Mappings

To add a new topic mapping:

1. Edit `bridge_config.yaml`
2. Add a new entry under `mappings:`
3. Set `enabled: true`
4. Restart the bridge

Example - adding a third arm:

```yaml
mappings:
  - name: center_arm
    input_topic: /center/joint_states
    output_topic: /bridge/center_joint_commands
    type: sensor_msgs/JointState
    num_joints: 7
    enabled: true
    expected_joint_names:
      - center_fr3v2_joint1
      - center_fr3v2_joint2
      - center_fr3v2_joint3
      - center_fr3v2_joint4
      - center_fr3v2_joint5
      - center_fr3v2_joint6
      - center_fr3v2_joint7
```

## Enabling Grippers

To enable physical gripper support:

1. Set `enabled: true` for the gripper mapping
2. Verify `gripper_joint_index` matches your JointState message
3. Verify `gripper_opening_joint` matches your simulation URDF
4. Restart the bridge

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  real_to_sim_bridge.py  (rmw_fastrtps_cpp / domain 0)               │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  real_to_sim_bridge_reader.py  (rmw_cyclonedds_cpp / domain 100)│
│  │  → subscribes to real robot topics (defined in config)       │  │
│  │  → writes JSON lines to stdout                               │  │
│  └──────────────────────────┬───────────────────────────────────┘  │
│                             │ JSON pipe                             │
│                             ▼                                       │
│  BridgePublisher node reads JSON, publishes to:                     │
│    /bridge/{mapping.name}_joint_commands (per config)               │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
                            Republisher service
                                    │
                                    ▼
                        Isaac Sim (/isaac/browser/* topics)
```

## Configuration Loader API

The bridge uses `bridge_config_loader.py` which provides:

```python
from bridge_config_loader import load_bridge_config

# Load configuration
config = load_bridge_config("path/to/bridge_config.yaml")

# Access settings
print(config.real_robot.domain_id)  # 100
print(config.simulation.rmw_implementation)  # rmw_fastrtps_cpp

# Get enabled mappings
for mapping in config.get_enabled_mappings():
    print(f"{mapping.name}: {mapping.input_topic} → {mapping.output_topic}")

# Filter by type
arm_mappings = config.get_arm_mappings()
gripper_mappings = config.get_gripper_mappings()
```

## Migration from CLI Arguments

**Old approach (deprecated):**
```bash
python3 real_to_sim_bridge.py \
  --left-topic /left/joint_states \
  --right-topic /right/joint_states \
  --real-domain-id 100 \
  --real-rmw rmw_cyclonedds_cpp
```

**New approach:**
```bash
# Edit bridge_config.yaml once
python3 real_to_sim_bridge.py
```

## Troubleshooting

### Bridge not starting

- Check that `bridge_config.yaml` exists and is valid YAML
- Verify `BRIDGE_CONFIG_PATH` environment variable in reader subprocess
- Check logs: `docker compose logs real_to_sim_bridge`

### No messages forwarded

- Verify real robot is publishing on configured `input_topic`
- Check domain IDs match your setup
- Verify CycloneDDS can discover real robot nodes
- Check `enabled: true` for the mapping

### Gripper not working

- Verify `gripper_joint_index` is correct for your JointState message
- Check gripper calibration values (`driver_joint_closed`, `driver_joint_open`)
- Try setting `invert: true` if gripper moves in wrong direction

## Files

- **`bridge_config.yaml`** - Topic mapping configuration (edit this!)
- **`bridge_config_loader.py`** - Configuration loader module
- **`real_to_sim_bridge.py`** - Main bridge process (domain 0, FastRTPS)
- **`real_to_sim_bridge_reader.py`** - Subprocess reader (domain 100, CycloneDDS)
- **`Dockerfile`** - Container image definition
- **`README.md`** - This file
