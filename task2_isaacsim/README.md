# Task 2 — Mobile FR3 Duo Teleoperation (Isaac Sim 5.1.0 / PhysX)

## Overview

Teleoperation of the mobile FR3 Duo for Task 2 (deformable thermal pad placement)
in **Isaac Sim 5.1.0 (PhysX)**, because the thermal pad asset uses `PhysxDeformableBodyAPI`, which requires PhysX GPU deformables.

### Objectives
- Transport the highly deformable pad without damaging it.
- Align and attach the pad onto the designated PCB target area.

### Technical Challenges
- The thermal pad is deformable (and attached to a liner), so manipulation requires contact-safe handling rather than rigid pick-and-place assumptions.
- Isaac Sim Task 2 uses the CUDA torch backend for deformables, while Lula expects numpy inputs; teleop IK must bridge tensor/numpy safely.
- The Aloha and Robotiq variants share FR3v2 arms but differ in gripper wiring (`single_joint` vs `drive_plus_mimic`), so gripper control must be config-driven.
- Keyboard/pedal control is ROS2-based in this framework (`/keyboard/state`, `/pedal/state`, `/task2/keyboard/arm_jog`), not local `carb.input` polling in the launcher.
- The bare scene launcher composes the stage only; a monolithic scene+control loop process is required to actually drive the robot.

### Scoring
- **Primary:** valid-placement IoU — Pick Success × Placement Orientation Success × Placement IoU (0–1); wrong orientation scores 0.
- **Tie-breaker:** completion time - the faster the better.

Detailed evaluation calculation can be found in the [Task 2 evaluation stack](../scripts/evaluation/task2/README.md#evaluation-metric).


## Prerequisites

1. **Isaac Sim 5.1.0 container running** with this repo bind-mounted at
   `/workspace/EBiM_Challenge` (default container name
   `isaac-sim-5-1-0-workshop`; override with `ISAACSIM_CONTAINER`). The
   container needs no ROS 2 install — the bridge uses the ROS 2 jazzy
   libraries bundled with Isaac Sim's `isaacsim.ros2.bridge` extension.
   Start the container (from root directory) with:
   ```bash
   docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
   --profile isaac-sim-5.1.0 up -d
   ```
2. **Robot USD downloaded**: run `task1_isaacsim/scripts/download_large_assets.sh` and ensure the robot USD is present at `task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd`.
3. **Teleop input devices** (keyboard / GELLO / pedal) publisher from the [`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) repository on the host.

## Quickstart

Scripts run from the repository root. The USD paths are relative to `task2_isaacsim/`.

Start the Isaac Sim 5.1.0 container (if not already running):

```bash
xhost +local:docker
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
touch "$XAUTHORITY"

docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
   --profile isaac-sim-5.1.0 up -d
```

Two scenes are available via `--scene` (both use the same robot USD and ROS topic contract):
- `--scene room` (default): the full robot room from `scripts/scenes/scene_robot_room_keyboard.py --task task2`.
- `--scene barebone`: barebone robot, ground plane, and task 2 objects.

### Barebone (empty scene) with keyboard base + browser arms (no special hardware)

```bash
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
   --scene barebone
   --with-keyboard-teleop
```

Then, in the teleoperation ROS 2 environment on the host, start the keyboard
publisher and drive the base with `w/a/s/d/q/e`:

```bash
ros2 run keyboard_state_publisher keyboard_state_publisher
```

Open the browser UI at <http://localhost:8090> to move the arms/grippers.
The spine keyboard control is `Up/Down`, with GUI focused.

### Room scene with GELLO arms

```bash
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
   --scene room \
   --with-gello-teleop
   --no-browser
```

On the host (teleoperation env): launch the GELLO publisher and the pedal
publisher (see the teleoperation README):

```bash
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
ros2 run pedal_state_publisher pedal_state_publisher
```

## Architecture

Same pipeline as Task 1, only the last stage (the simulator process) differs.
See [task1_isaacsim/README.md](../task1_isaacsim/README.md#architecture) for
the documentation. The stages are:

- **Host device publishers** (EBiM `teleoperation` repo, on the host) publish:
  - `/keyboard/state` (`std_msgs/String`, keys `w/s/a/d/q/e`)
  - `/{left,right}/gello/joint_states` (`sensor_msgs/JointState`)
  - `/{left,right}/gripper/gripper_client/target_gripper_width_percent`
  (`std_msgs/Float32`).
- **Teleop adapters** (`task2_teleop_adapters`, `ros:jazzy-ros-base`) remap
  device topics:
  - `keyboard_to_base.py` turns `/keyboard/state` into
  `/pedal/state` base-driving tokens
  - `gello_to_bridge.py` turns the GELLO
  topics into `/bridge/{left,right}_joint_commands` and
  `/bridge/{left,right}_robotiq_joint_commands`.
- **Browser controller** (`task2_browser_controller`, web UI on port 8090) is
  the no-hardware alternative: it publishes the same `/bridge/*` command
  topics from browser sliders.
- **Republisher and position controller** move commands from `/bridge/*` to
  `/isaac/*`:
  - `ros_joint_republisher` (`task2_ros_republisher`) handles the
  grippers with open/close calibration
  - `joint_position_controller`
  (`task2_position_controller`, compose profile `position`) forwards the arm
  joint commands.
- **Scene script** (`scene_room.py` or `scene_barebone.py`, run with
  `/isaac-sim/python.sh` inside `isaac-sim-5-1-0-workshop` container).

  It subscribes to:
  - `/isaac/{left,right}_joint_commands`
  - `/isaac/{left,right}_robotiq_joint_commands`
  - `/isaac/browser/*` variants
  - `/pedal/state` (swerve base)

  It publishes:
  - `/isaac/{left,right}_joint_states`
  - `/isaac/{left,right}_robotiq_joint_states` at 60 Hz, plus — room scene only —
  - `/isaac/eval_camera/{image_raw,depth,camera_info,semantic_segmentation,bbox_2d_tight}` for the Task 2 evaluation stack.

The scene scripts are thin stage builders; the shared teleop runtime
(`/isaac/*` ROS node, name-based joint resolution, Robotiq driver + PhysX
mimic handling, swerve-base pedal driving, spine keyboard control, main loop)
lives in `scripts/isaacsim_fr3duo_teleop_bridge_core.py`, with the shared CLI
flags in `scripts/isaacsim_fr3duo_teleop_bridge_args.py`.

### Mapping to Task 1 counterparts

| Task 2 | Task 1 counterpart | Relationship |
|---|---|---|
| `scripts/scene_barebone.py`, `scripts/scene_room.py`, `scripts/isaacsim_fr3duo_teleop_bridge_core.py` | `scripts/isaaclab_fr3duo_newton_bridge.py` | Reimplementation for plain Isaac Sim 5.1.0 / PhysX (Isaac Lab + Newton cannot run the deformable pad). Same topics, joint names, defaults; ports task1's swerve-base math, spine keyboard control, and articulation-root fix. Imports task1's `isaac_bridge_constants.py` directly. |
| `scripts/run_isaacsim_teleop.sh` | `scripts/run_isaaclab_newton_teleop.sh` | Same flag conventions; simpler (expects the Isaac Sim container to be already running; adds `--scene room\|barebone`). |
| `docker-compose.yml` (containers `task2_*`) | Same-named services in `task1_isaacsim/docker-compose.yml` (containers `task1_*`) | Same images, commands, env, profiles — only the volume differs: task2 mounts `../task1_isaacsim` at `/workspace`, so the containers execute the Task 1 scripts unmodified. |
| *(no copy — reused via mount)* | `scripts/adapters/keyboard_to_base.py`, `scripts/adapters/gello_to_bridge.py` | Pure topic remappers, task-agnostic. |
| *(no copy — reused via mount)* | `scripts/controllers/ros_joint_republisher.py`, `scripts/controllers/joint_position_controller.py` | Isaac-agnostic rclpy nodes. |
| *(no copy — reused via mount)* | `services/teleop_adapters/`, `services/browser_controller/` | Adapter launcher + web UI (port 8090). |
| *(imported directly)* | `scripts/isaac_bridge_constants.py` | Joint name lists, Robotiq driver/coupled-joint constants, topic layout. |
| `.env.example` | `task1_isaacsim/.env.example` | Same variables and defaults. |
| Robot USD `task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd` | (same file) | Shared asset. Under PhysX the bridge additionally deactivates the OmniGraph graphs embedded in this USD (they crash plain Isaac Sim) and relies on the USD-authored `PhysxMimicJointAPI` for the gripper linkage. |

Task-2-only pieces with no Task 1 counterpart: the scene composition
(`assets/task2_objects/` deformable thermal pad + RAM boards; the robot room
via `scripts/scenes/scene_robot_room_keyboard.py`), PhysX GPU-dynamics setup,
and the `/isaac/eval_camera/*` publishers for the Task 2 evaluation stack.


## Notes

- **Do not run the Task 1 and Task 2 helper stacks at the same time** — they
  bind identical topics on the host network and the same browser port 8090.
- The browser controller streams its current slider pose continuously. If you
  restart the simulator while the helper stack keeps running, the robot is
  yanked from its spawn pose to the stale browser pose at startup — restart
  the browser controller together with the simulator (or `--no-browser`).
- Helper defaults live in [.env.example](.env.example) (copy to `.env` to
  override): gripper open/closed calibration, adapter selection, controller
  mode.
- The bridge defaults match Task 1: physics 240 Hz / render 60 Hz, joint
  states on `/isaac/*` at 60 Hz, pedal base driving at 0.5 m/s / 1.2 rad/s
  with a 1 s timeout, spine height on keyboard Up/Down.
