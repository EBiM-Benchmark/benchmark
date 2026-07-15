# Bridge Configuration System - Design and Implementation

## Configuration Concept: YAML-Based Topic Mapping

### Why YAML?

After evaluating several configuration approaches, **YAML** was chosen as the optimal solution for the following reasons:

1. **ROS 2 Standard**: YAML is the de facto standard for ROS 2 configuration files
2. **Human-Readable**: Easy to read and edit without programming knowledge
3. **Comments**: Supports inline documentation and explanations
4. **Structure**: Natural hierarchical structure for complex configurations
5. **Tooling**: Excellent IDE support (syntax highlighting, validation, autocomplete)
6. **Consistency**: Matches existing project files like `docker-compose.yml`
7. **Dependencies**: PyYAML is already available in ROS 2, no extra dependencies

### Alternative Approaches Considered

| Approach | Pros | Cons | Decision |
|----------|------|------|----------|
| **YAML Config** ✅ | Standard, readable, tooling | None significant | **CHOSEN** |
| ROS 2 Parameters | Native ROS 2, runtime reconfigurable | Complex setup, requires param server | Not needed for static config |
| JSON | Simple, widely supported | No comments, less readable | YAML is superior |
| TOML | Modern, clear | Less common in ROS ecosystem | YAML more standard |
| Python Config | Flexible, programmable | Code rather than data | Over-engineered |
| Hardcoded | Simple | Requires code changes, not flexible | Current problem! |

## Implementation Architecture

### Component Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    bridge_config.yaml                       │
│  - Real robot domain/RMW configuration                      │
│  - Simulation domain/RMW configuration                      │
│  - Topic mappings (input → output)                          │
│  - Gripper calibration settings                             │
└─────────────────────────────────────────────────────────────┘
                           │
                           ↓ loaded by
┌─────────────────────────────────────────────────────────────┐
│              bridge_config_loader.py                        │
│  - BridgeConfig dataclass                                   │
│  - TopicMapping dataclass                                   │
│  - DomainConfig dataclass                                   │
│  - GripperConfig dataclass                                  │
│  - Validation and filtering methods                         │
└─────────────────────────────────────────────────────────────┘
                           │
                           ↓ used by
┌─────────────────────────────────────────────────────────────┐
│         real_to_sim_bridge.py (main process)                │
│  - Loads config at startup                                  │
│  - Creates publishers based on config mappings              │
│  - Passes config to reader subprocess                       │
└─────────────────────────────────────────────────────────────┘
                           │
                           ↓ spawns
┌─────────────────────────────────────────────────────────────┐
│       real_to_sim_bridge_reader.py (subprocess)             │
│  - Loads config from BRIDGE_CONFIG_PATH env var             │
│  - Creates subscribers based on config mappings             │
│  - Forwards messages via JSON pipe to parent                │
└─────────────────────────────────────────────────────────────┘
```

### Configuration File Structure

```yaml
real_robot:
  rmw_implementation: rmw_cyclonedds_cpp
  domain_id: 100
  cyclonedds_uri: file:///workspace/assets/isaac_assets/config/cyclonedds_localhost.xml

simulation:
  rmw_implementation: rmw_fastrtps_cpp
  domain_id: 0

mappings:
  - name: left_arm
    input_topic: /left/joint_states
    output_topic: /bridge/left_joint_commands
    type: sensor_msgs/JointState
    description: "Left FR3 arm joint states from real robot → bridge commands"
    num_joints: 7
    enabled: true
    expected_joint_names: [...]
    
  - name: left_gripper
    input_topic: /left/robotiq_2f85_controller/joint_states
    output_topic: /bridge/left_robotiq_joint_commands
    type: sensor_msgs/JointState
    enabled: false  # Disabled by default
    gripper_joint_index: 0
    gripper_opening_joint: left_robotiq_2f85_left_driver_joint

gripper:
  driver_joint_closed: 0.0
  driver_joint_open: 0.8
  invert: false
```

### Key Design Decisions

1. **Declarative Over Imperative**
   - Configuration declares *what* to map, not *how* to map it
   - Logic stays in code, data stays in config

2. **Type Safety with Dataclasses**
   - Python dataclasses provide structure and validation
   - IDE autocomplete works correctly
   - Runtime type checking catches errors early

3. **Filtering and Querying**
   - `config.get_enabled_mappings()` - only active mappings
   - `config.get_arm_mappings()` - arm-specific mappings
   - `config.get_gripper_mappings()` - gripper-specific mappings

4. **Backward Compatibility**
   - Old CLI arguments still work but are marked as deprecated
   - Allows gradual migration of existing scripts

5. **Environment Variable Bridge**
   - Parent process passes `BRIDGE_CONFIG_PATH` to subprocess
   - Subprocess loads same config file
   - Single source of truth

## What Was Moved Out of Code

### Before (Hardcoded)

```python
# In real_to_sim_bridge.py
_BRIDGE_CMD_TOPIC: dict[str, str] = {
    "left":          "/bridge/left_joint_commands",
    "right":         "/bridge/right_joint_commands",
    "left_gripper":  "/bridge/left_robotiq_joint_commands",
    "right_gripper": "/bridge/right_robotiq_joint_commands",
}

_ARM_JOINT_NAMES: dict[str, list[str]] = {
    "left":  LEFT_JOINTS,
    "right": RIGHT_JOINTS,
}

# In real_to_sim_bridge_reader.py
left_topic = os.environ.get("BRIDGE_LEFT_TOPIC", "")
right_topic = os.environ.get("BRIDGE_RIGHT_TOPIC", "")
```

### After (Configuration-Driven)

```yaml
# In bridge_config.yaml
mappings:
  - name: left_arm
    input_topic: /left/joint_states
    output_topic: /bridge/left_joint_commands
    expected_joint_names: [left_fr3v2_joint1, ...]
  - name: right_arm
    input_topic: /right/joint_states
    output_topic: /bridge/right_joint_commands
    expected_joint_names: [right_fr3v2_joint1, ...]
```

## Benefits Achieved

### 1. **Separation of Concerns**
- Configuration is data, not code
- Non-programmers can modify topic mappings
- Changes don't require code review

### 2. **Single Source of Truth**
- One file defines all mappings
- No need to search through multiple files
- Easy to see the complete bridge configuration

### 3. **Easy Extensibility**
- Adding a new arm: Add one YAML block
- Enabling grippers: Set `enabled: true`
- No code changes required

### 4. **Environment-Specific Configs**
- Production: `bridge_config.yaml`
- Testing: `bridge_config.test.yaml`
- Development: `bridge_config.dev.yaml`
- Use `--bridge-config` flag to switch

### 5. **Version Control Friendly**
- YAML diffs are readable
- Can track configuration changes over time
- Easy to revert problematic changes

### 6. **Documentation as Code**
- YAML comments document each mapping
- Self-documenting configuration
- Reduces need for external documentation

## Usage Examples

### Basic Usage

```bash
# Uses default config (services/real_to_sim_bridge/bridge_config.yaml)
docker compose --profile follower up real_to_sim_bridge
```

### Custom Configuration

```bash
# Use a different config file
python3 services/real_to_sim_bridge/real_to_sim_bridge.py \
  --bridge-config /path/to/custom_config.yaml
```

### Environment-Specific

```bash
# Production
export BRIDGE_CONFIG=/workspace/services/real_to_sim_bridge/bridge_config.yaml

# Testing
export BRIDGE_CONFIG=/workspace/services/real_to_sim_bridge/bridge_config.test.yaml
```

### Adding a New Arm

Just edit `bridge_config.yaml`:

```yaml
mappings:
  # Existing mappings...
  
  # New arm
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

Restart the bridge - done!

## Files Created/Modified

### Created Files
- ✅ `services/real_to_sim_bridge/bridge_config.yaml` - Main configuration file
- ✅ `services/real_to_sim_bridge/bridge_config_loader.py` - Configuration loader
- ✅ `services/real_to_sim_bridge/README.md` - Documentation

### Modified Files
- ✅ `services/real_to_sim_bridge/real_to_sim_bridge.py` - Uses config instead of hardcoded values
- ✅ `services/real_to_sim_bridge/real_to_sim_bridge_reader.py` - Loads config from env var
- ✅ `docker-compose.yml` - Simplified command (no CLI args needed)
- ✅ `scripts/run_real_robot_bridge.sh` - Updated to use config-first approach

## Validation

The configuration system has been validated:

```bash
✓ Configuration loaded successfully
✓ Real robot: rmw_cyclonedds_cpp / domain 100
✓ Simulation: rmw_fastrtps_cpp / domain 0
✓ Enabled mappings: 2
  - left_arm: /left/joint_states → /bridge/left_joint_commands
  - right_arm: /right/joint_states → /bridge/right_joint_commands
✓ Gripper config: closed=0.0, open=0.8
```

## Migration Path

Old scripts using CLI arguments will continue to work (deprecated):

```bash
# Old way (still works)
python3 real_to_sim_bridge.py --left-topic /left/joint_states --right-topic /right/joint_states

# New way (recommended)
python3 real_to_sim_bridge.py
```

To migrate:
1. Copy your CLI arguments into `bridge_config.yaml`
2. Test with the config file
3. Remove CLI arguments from scripts
4. Delete deprecated CLI argument code in future version

## Future Enhancements

Possible future improvements:

1. **Schema Validation**: Add JSON Schema validation for config file
2. **Hot Reload**: Watch config file and reload on changes
3. **Config Generator**: CLI tool to generate config from introspection
4. **Multiple Configs**: Support merging multiple config files
5. **Environment Variables**: Support env var substitution in YAML (e.g., `${ROS_DOMAIN_ID}`)
6. **Config Validation**: Add `--validate-config` flag to check config without running

## Summary

The YAML-based configuration system provides:
- ✅ Clear separation between configuration and code
- ✅ Easy maintenance and modification
- ✅ Type-safe configuration loading
- ✅ Excellent documentation and tooling support
- ✅ Extensible design for future requirements
- ✅ Backward compatibility with existing scripts

This is the standard approach for ROS 2 configuration management and aligns with industry best practices.
