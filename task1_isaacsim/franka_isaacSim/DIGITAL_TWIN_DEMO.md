# Digital Twin Demo - Quick Reference

## ⚠️ CRITICAL FOR DEMO NEXT WEEK

This document ensures the digital twin functionality remains working during repository restructuring.

## Quick Start

### 1. Test Before Any Changes
**ALWAYS run this before making structural changes:**
```bash
make test-digital-twin
```

All 20+ tests must pass. If any fail, **DO NOT proceed** with repo changes.

### 2. Start Digital Twin for Demo
```bash
# Start the simulation stack
bash scripts/run_native_stream.sh

# In another terminal, start the bridge
make follower-up

# Or use the wrapper script:
bash scripts/run_real_robot_bridge.sh
```

### 3. Verify It's Working
```bash
# Check bridge is running
docker compose ps | grep real_to_sim_bridge

# Check message flow (should be ~1000 Hz for arms, ~500 Hz for grippers)
docker compose exec -T ros_republisher bash -lc "
  source /opt/ros/jazzy/setup.bash && 
  ros2 topic hz /bridge/left_joint_commands /bridge/right_joint_commands
"

# Check browser controller mode (should be "digital_twin")
curl http://localhost:8090/api/control_mode
```

## Critical Files - DO NOT MOVE OR DELETE

### Configuration
- `services/real_to_sim_bridge/bridge_config.yaml` - **CRITICAL** topic mappings
- `docker-compose.yml` - Contains `real_to_sim_bridge` service definition
- `Makefile` - Has `follower-up`/`follower-down` targets

### Source Code
- `services/real_to_sim_bridge/real_to_sim_bridge.py` - Main bridge process
- `services/real_to_sim_bridge/real_to_sim_bridge_reader.py` - CycloneDDS subprocess
- `services/real_to_sim_bridge/bridge_config_loader.py` - Configuration loader

### Tests
- `tests/test_real_to_sim_bridge.py` - **20 tests** that must pass

### Scripts
- `scripts/run_real_robot_bridge.sh` - Convenience wrapper
- `scripts/run_native_stream.sh` - Starts full sim stack

## Current Configuration (Verified Working)

### Topic Mappings
```yaml
Left Arm:
  Input:  /left/franka_robot_state_broadcaster/measured_joint_states  (1000 Hz)
  Output: /bridge/left_joint_commands

Right Arm:
  Input:  /right/franka_robot_state_broadcaster/measured_joint_states (1000 Hz)
  Output: /bridge/right_joint_commands

Left Gripper:
  Input:  /gripper/left/joint_states  (500 Hz)
  Output: /bridge/left_robotiq_joint_commands

Right Gripper:
  Input:  /gripper/right/joint_states (500 Hz)
  Output: /bridge/right_robotiq_joint_commands
```

### Domains
- **Real robots**: Domain 100, CycloneDDS
- **Simulation**: Domain 0, FastRTPS

### Data Flow
```
Real Robots (Domain 100)
  ↓
real_to_sim_bridge (cross-RMW bridge)
  ↓
/bridge/* topics (Domain 0)
  ↓
ros_republisher
  ↓
/isaac/browser/* topics
  ↓
Isaac Sim
```

## Demo Checklist

### Pre-Demo Setup
- [ ] Run `make test-digital-twin` - all tests pass
- [ ] Start simulation: `bash scripts/run_native_stream.sh`
- [ ] Wait for Isaac to fully load (~60 seconds)
- [ ] Start bridge: `make follower-up`
- [ ] Verify browser mode: `curl http://localhost:8090/api/control_mode`
- [ ] Check topics are publishing at correct rates
- [ ] Real robots connected and publishing on domain 100

### During Demo
- [ ] Open browser UI: http://localhost:8090
- [ ] Mode should be "digital_twin"
- [ ] Robot movements in real life should be mirrored in Isaac Sim
- [ ] Arms: 1000 Hz update rate
- [ ] Grippers: 500 Hz update rate

### Troubleshooting During Demo

**Sim robot not moving:**
```bash
# Check bridge is running
docker compose logs real_to_sim_bridge --tail=20

# Check if real robot topics exist
docker compose exec -T real_to_sim_bridge bash -lc "
  export ROS_DOMAIN_ID=100
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source /opt/ros/jazzy/setup.bash
  ros2 topic list | grep joint_states
"

# Restart bridge
make follower-down && make follower-up
```

**Wrong control mode:**
```bash
# Check current mode
curl http://localhost:8090/api/control_mode

# Switch to digital_twin mode
curl -X POST http://localhost:8090/api/control_mode \
  -H "Content-Type: application/json" \
  -d '{"mode":"digital_twin"}'
```

## Safe Repo Restructuring

### Before Moving Files
1. **Run tests:** `make test-digital-twin`
2. **Note current paths** of all critical files listed above
3. **Update tests** if you plan to move files

### After Moving Files
1. **Update all imports** in Python files
2. **Update docker-compose.yml** service paths
3. **Update Makefile** paths if scripts moved
4. **Update bridge_config.yaml path** in loader
5. **Run tests again:** `make test-digital-twin`
6. **Test live system:** Start bridge and verify message flow

### Path Update Checklist
If you move `services/real_to_sim_bridge/` to a new location:
- [ ] Update `docker-compose.yml` - `dockerfile:` and `command:` paths
- [ ] Update `Makefile` - any scripts that reference the path
- [ ] Update `tests/test_real_to_sim_bridge.py` - `SERVICES_DIR` path
- [ ] Update any wrapper scripts in `scripts/`
- [ ] Update Python imports if module structure changes
- [ ] Update documentation

## Performance Benchmarks

These are the expected message rates for a working system:

| Topic | Rate | Notes |
|-------|------|-------|
| `/bridge/left_joint_commands` | ~1000 Hz | Measured joint states |
| `/bridge/right_joint_commands` | ~1000 Hz | Measured joint states |
| `/bridge/left_robotiq_joint_commands` | ~500 Hz | Gripper states |
| `/bridge/right_robotiq_joint_commands` | ~500 Hz | Gripper states |
| `/isaac/left_joint_states` | ~10 Hz | Isaac simulation output |

**If rates are significantly lower**, something is broken.

## Emergency Recovery

If tests fail after changes:
```bash
# Revert to last known good commit
git log --oneline | head -10  # Find last good commit
git checkout <commit-hash>

# Or restore specific files
git checkout HEAD -- services/real_to_sim_bridge/
git checkout HEAD -- docker-compose.yml
git checkout HEAD -- Makefile

# Run tests again
make test-digital-twin
```

## Contact Information

**For demo day issues:**
- Check this guide first
- Run diagnostic commands above
- Check logs: `docker compose logs real_to_sim_bridge`

**Last verified working:** $(date)
**Configuration hash:** $(git rev-parse HEAD)

---

**Remember: Run `make test-digital-twin` before ANY structural changes!**
