# Franka Isaac Sim + ROS2 Control

Containerized Isaac Sim with ROS2-control for the FR3 duo + Robotiq.

## Architecture

Docker services (`docker-compose.yml`):
- `isaac-sim` — Isaac runtime; bridge script launched via `--exec`
- `ros_republisher` — `/isaac/*` ↔ `/bridge/*` translation + gripper normalization
- `browser_controller` — browser UI at `http://localhost:8090`
- `franka_controller` — ROS2 workspace for `ros2_control` and Franka controllers

Topic namespaces:
- Isaac side: `/isaac/*_joint_states`, `/isaac/*_joint_commands`
- Browser overrides: `/isaac/browser/*_joint_commands`
- Controller/bridge side: `/bridge/*`

## Submodules

| Path | Source |
|---|---|
| `assets/franka_description` | `ssh://git@bitbucket.fe.lan:7999/moctrl/franka_description.git` |
| `assets/franka_ros2` | `ssh://git@bitbucket.fe.lan:7999/moctrl/franka_ros2.git` |
| `assets/ros2_robotiq_gripper` | `https://github.com/PickNikRobotics/ros2_robotiq_gripper.git` |

```bash
git submodule update --init --recursive
```

## Supported Embodiments

Configurations live in `assets/embodiments/`:

| Name | Description | DOF |
|---|---|---|
| `fr3duo_m+v` | Fixed-base dual-arm (default) | 14 |
| `fr3duo_mobile` | Mobile-base dual-arm (sim only) | 17 |

Pass `--embodiment <name>` to scripts that support it.

## Prerequisites
- Linux with NVIDIA GPU + driver
- Docker + NVIDIA container toolkit

## 1) Initial Setup
```bash
bash setup.sh
xhost +local:root
```

## 2) Start Simulation Stack
```bash
bash scripts/run_native_stream.sh
```

Key options:
```bash
--asset-name ai_cell_robotiq_2f85          # select a named USD asset
--usd-path /workspace/assets/my.usd        # explicit USD path
--controller-mode position                 # or: effort (default)
--ros-publish-rate 120
--physics-hz 240 --render-hz 60 --physics-substeps 2
--foreground                               # attach terminal for debugging
--no-browser / --no-stream
--list-assets                              # show available named assets
--robot-prim-path /your/articulation/root  # if robot isn't auto-detected
```

Endpoints:
- Browser UI: `http://localhost:8090`
- WebRTC streaming client: `127.0.0.1:49100`
- Browser topic API: `http://localhost:8090/api/topics`

Omniverse asset cache persists in `assets/3D_assets/omniverse_asset_storage/` (mounted from `/tmp/isaac_portable`).

To invert gripper normalization, set `REPUBLISHER_GRIPPER_INVERT="true"` in `.env`, then restart `ros_republisher`.

## 3) Digital Twin (Real Robot → Isaac Sim)

Mirrors a live FR3 duo (domain 100, CycloneDDS) into Isaac.

```bash
make digital-twin-up      # sim stack + bridge + sets digital_twin mode
make digital-twin-status  # verify containers and topic flow
make digital-twin-down
```

Or start separately:
```bash
bash scripts/run_native_stream.sh   # terminal 1
make follower-up                    # terminal 2
```

Topic flow:
```
Real robots (domain 100, CycloneDDS)  ~1000 Hz arms / ~500 Hz grippers
  ↓ real_to_sim_bridge
/bridge/{left,right}_joint_commands
/bridge/{left,right}_robotiq_joint_commands
  ↓ ros_republisher → /isaac/browser/* → Isaac Sim
```

Config: `services/real_to_sim_bridge/bridge_config.yaml`
Demo checklist + troubleshooting: `DIGITAL_TWIN_DEMO.md`

## 4) ROS2 Control (`franka_controller`)

Required only for **effort mode**. Skip for position mode.

```bash
# Build
docker compose build franka_controller
docker compose up -d franka_controller
docker compose exec -it franka_controller bash -lc "
  source /opt/ros/jazzy/setup.bash &&
  source /dependencies_ws/install/setup.bash &&
  cd /ros2_ws/src &&
  [ -d topic_based_ros2_control ] || git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git &&
  cd /ros2_ws && colcon build --symlink-install"

# Launch
docker compose exec -it franka_controller bash -lc "
  source /opt/ros/jazzy/setup.bash &&
  source /dependencies_ws/install/setup.bash &&
  source /ros2_ws/install/setup.bash &&
  ros2 launch franka_example_controllers fr3_duo_isaac.launch.py \
    ee_id:=robotiq_2f85 load_gripper:=true \
    use_robotiq_controllers:=true \
    controller_manager_name:=isaac_controller_manager"
```

Verify:
```bash
docker compose exec -it franka_controller bash -lc "source /opt/ros/jazzy/setup.bash && ros2 control list_controllers -c /isaac_controller_manager"
```

## 5) Controller Modes

| Mode | How | When to use |
|---|---|---|
| `effort` (default) | ROS2 impedance controller computes torques | needs `franka_controller` running |
| `position` | PhysX drives track position targets directly | standalone, no `franka_controller` needed |

Switch at startup — cannot hot-swap:
```bash
docker compose down
bash scripts/run_native_stream.sh --controller-mode position
```

Persist default: set `controller.mode` in `assets/isaac_assets/config/stack_defaults.yaml`.

## 6) Config

Most defaults live in `assets/isaac_assets/config/stack_defaults.yaml` (simulation, bridge, drives, cameras, controller gains, test settings).

```bash
python3 scripts/stack_config.py --sections simulation bridge
```

## 7) Tests & Optimizations

All tests take `--config assets/isaac_assets/config/stack_defaults.yaml`. Artifacts land in `extra/`.

```bash
# Test suites
python3 scripts/test_suites/run_libfranka_duo_test.py
python3 scripts/test_suites/run_target_pose_test.py
python3 scripts/test_suites/run_current_target_hold_test.py
python3 scripts/test_suites/run_episode_replay_test.py
python3 scripts/test_suites/run_fr3v2_isaac_setup_audit.py
python3 scripts/test_suites/run_camera_output_test.py
python3 scripts/test_suites/run_sim_burn_test.py

# Joint velocity probe (from inside ros_republisher container)
python3 /workspace/scripts/test_suites/joint_velocity_probe.py --duration 15

# Optimizations
python3 scripts/optimizations/optimize_isaac_joint_drives.py
python3 scripts/optimizations/optimize_control.py
```

## 8) Make Targets

```bash
make digital-twin-up      # sim stack + real-to-sim bridge
make digital-twin-down    # stop everything
make digital-twin-status  # check containers and topic flow
make follower-up/down     # real-to-sim bridge only
make stack-up/down        # sim containers only
make stack-restart        # force-recreate + relaunch Isaac
```
