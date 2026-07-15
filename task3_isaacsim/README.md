# Task 3 — Assisted Living & Feeding (Isaac Sim 5.1.0)

## Overview

This folder contains the participant runtime for Task 3: Table Setup, Feed,
Bean Recovery, and Clean Up with the mobile dual-FR3 robot in the shared robot
room. The ROS bridge supports browser, keyboard-base, and GELLO/pedal input.

The local [Task 3 evaluation](../scripts/evaluation/task3/README.md) is a
development facilitator. Official scoring follows the rules published on the
[competition page](https://ebim-benchmark.github.io/competition.html#tasks).

## Task workflow

1. **Table Setup:** move the tray and dining items from the Kitchen Area to the
   Dining Area.
2. **Feed:** scoop beans, hold the spoon at the feeding pose for at least three
   seconds, then return the beans to the bowl.
3. **Bean Recovery:** transfer the beans into the designated recycling
   container in the Kitchen Area.
4. **Clean Up:** return the utensils to the marked sink region.

## Development grading helpers

Task 3 grading helpers live in
[`scripts/evaluation/task3/`](../scripts/evaluation/task3/README.md), outside
the participant runtime. They provide deterministic development-time scoring
for all four stages; they are not the official competition scorer.

Run the pure scoring tests without Isaac Sim:

```bash
python -B scripts/evaluation/task3/tests/test_grading.py
```

Pass `stage1`, `stage2`, `stage3`, or `stage4` to test one stage. The Isaac Sim
integration validator and its container command are documented in the
[Task 3 evaluation guide](../scripts/evaluation/task3/README.md). That validator
builds a separate scene and moves objects through deterministic test motions;
it does not grade the active browser, ROS, keyboard, or GELLO teleoperation
session.

### TODO: realtime teleoperation grading (baseline team)

- [ ] Connect the grading helpers to the live `scene_room.py` stage.
- [ ] Read object and bean state from the active teleoperation run for Stages
  1, 3, and 4.
- [ ] Track Stage 2 spoon motion, bean retention, feed-zone entry, and the
  continuous three-second hold using simulation time.
- [ ] Add a participant-facing trigger and machine-readable per-stage and total
  score output.
- [ ] Validate the live grader with both Robotiq and Panda profiles and document
  the final workflow.

## Gripper profiles

Select the robot and its complete ROS gripper contract with `--gripper`:

| Profile | Robot asset | ROS command mapping | Status |
|---|---|---|---|
| `robotiq` (default) | Competition Mobile FR3 Duo with Robotiq 2F-85 grippers | Robotiq driver and USD mimic joints, 0–0.8 driver range | Recommended |
| `panda` | Current `franka_description` Mobile FR3 Duo with Franka/Panda fingers | Robotiq-compatible opening topics mapped to `finger_joint1/2`, 0–0.04 m range | Compatibility |

The ROS-facing gripper topics remain stable for both profiles. Changing the
profile requires restarting the Task 3 simulator and helper containers; it is
not a live end-effector swap.

## Prerequisites

1. Linux with a supported NVIDIA GPU, Docker Compose v2, and NVIDIA Container
   Toolkit.
2. The repository cloned with submodules.
3. X11 access for the Isaac Sim GUI.
4. The shared Isaac Sim 5.1.0 container built and running.
5. For `robotiq`, download the large Task 1 robot asset:

   ```bash
   bash task1_isaacsim/scripts/download_large_assets.sh
   ```

   If the OneDrive download returns HTTP 403 and the fetched
   `origin/Robotiq_DEMO` branch is available, extract the Task 3 robot asset
   directly from Git instead:

   ```bash
   git show \
     origin/Robotiq_DEMO:DEMO/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd \
     > task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd
   ```

   This fallback restores only the robot USD needed by Task 3. The destination
   is ignored by Git.

Start the simulator container from the repository root:

```bash
xhost +local:docker
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 up -d isaac-sim-5-1-0
```

## Quick start: browser arms and grippers

No GELLO hardware is required. The launcher starts the Task 3 ROS republisher,
position controller, and browser UI, then opens the Isaac Sim GUI.

Robotiq competition robot:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper robotiq
```

Panda/Franka-hand compatibility robot:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper panda
```

Open <http://localhost:8090> to command both arms and grippers. This is a local
web application and does not require internet access. Both profiles use the
same normalized browser/GELLO opening contract; the helper stack applies the
profile-specific physical joint range. In the Isaac Sim window, the Up/Down
arrow keys control the vertical spine.

The browser does not drive the mobile base. Use the ROS command below, the
keyboard-base publisher, or a foot pedal for base motion.

## Argument combinations

Stop the active simulator with Ctrl+C before starting another combination. To
reset all Task 3 helper containers between modes, run:

```bash
bash task3_isaacsim/scripts/run_helper_containers.sh down
```

Use the Panda compatibility robot with the browser:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh --gripper panda
```

Run without the Isaac Sim GUI while retaining browser control:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper robotiq \
  --headless
```

Choose a deterministic head placement (`A` through `I`) and keep the beans
static:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper robotiq \
  --head-placement A \
  --no-dynamic-beans
```

Arguments after `--` are forwarded to `scene_room.py`. For example, lower the
physics and rendering rates with:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper robotiq \
  -- \
  --physics-hz 120 \
  --render-hz 30
```

## ROS command-line smoke controls

With the default launcher still running, open the left gripper (`1.0` means
open and `0.0` means closed):

```bash
docker exec -it task3_ros_republisher bash -lc \
  'source /opt/ros/jazzy/setup.bash &&
   ros2 topic pub --once /bridge/left_robotiq_joint_commands \
   sensor_msgs/msg/JointState \
   "{name: [left_robotiq_opening], position: [1.0]}"'
```

Drive the base forward while the publisher runs:

```bash
docker exec -it task3_ros_republisher bash -lc \
  'source /opt/ros/jazzy/setup.bash &&
   ros2 topic pub -r 10 /pedal/state std_msgs/msg/String "{data: FWD}"'
```

Press Ctrl+C to stop publishing; the simulator stops the base after its
one-second command timeout. Other base tokens are `BACK`, `A`/`B` for strafe,
and `A+C`/`B+C` for rotation.

## Isaac-window keyboard arms

To control both arms and grippers directly from the Isaac Sim window without
the browser or external ROS publishers, launch:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper robotiq \
  --no-browser \
  --no-republisher \
  --controller-mode none \
  -- \
  --arm-keyboard-teleop
```

Click the Isaac Sim window before using the keys. Left-arm translation uses
`W/S`, `A/D`, `Q/E`, with `F` toggling its gripper. Right-arm translation uses
`O/L`, `K/;`, `I/P`, with `'` toggling its gripper. Press `R` to reset both arm
targets. Left-arm rotation uses `Z/X`, `T/G`, `C/V`; right-arm rotation uses
`N/M`, `U/J`, `,/.`. Use `--gripper panda` with the same command to test the
Panda profile. This mode does not provide direct keyboard base driving.

## GELLO arms and foot-pedal base

Launch Task 3 without the browser controller:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper robotiq \
  --with-gello-pedal-teleop \
  --no-browser
```

On the host, use the publishers from the separate
[`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation)
repository:

```bash
ros2 launch franka_gello_state_publisher main.launch.py \
  config_file:=franka_gello_duo.yaml
ros2 run pedal_state_publisher pedal_state_publisher
```

Use `--gripper panda` with the same command to test GELLO arm commands against
the Panda-finger robot. The adapter continues to publish normalized gripper
opening; Task 3 changes only the simulator-side calibration and joint mapping.

## Keyboard base input

Start the keyboard-to-base adapter:

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh \
  --gripper robotiq \
  --with-keyboard-teleop
```

Then run the teleoperation repository's `keyboard_state_publisher` on the host.
It publishes `w/a/s/d/q/e` state; Task 3 converts that input to the shared
`/pedal/state` base contract.

## Panda direct-keyboard RMPflow demo

The pre-existing no-ROS Panda/Franka-hand demo remains available. With the
Isaac Sim container running:

```bash
docker exec -it isaac-sim-5-1-0-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && \
   /isaac-sim/python.sh task3_isaacsim/scripts/scene_robot_room_rmpflow.py'
```

This path uses in-window dual-arm, gripper, and base keys. It does not start the
ROS helper stack and only supports the Panda profile. Hold Shift for base
control: `H/N` moves forward/backward, `B/M` moves left/right, and `G/J`
rotates counter-clockwise/clockwise.

## Helper container lifecycle

```bash
bash task3_isaacsim/scripts/run_helper_containers.sh status
bash task3_isaacsim/scripts/run_helper_containers.sh logs
bash task3_isaacsim/scripts/run_helper_containers.sh down
```

Do not run the Task 1, Task 2, and Task 3 helper stacks simultaneously. They
share host ROS topics and browser port 8090.

On a cold first launch, Isaac Sim may spend additional time building renderer
caches. If Kit crashes immediately after printing `app ready`, retry the same
launcher once after confirming that the container and GPU remain available.

## Layout

```text
task3_isaacsim/
├── assets/lula/mobile_fr3_duo/   # Panda direct-keyboard motion configs
├── scripts/
│   ├── common/                   # Task 3 direct-keyboard command/IK helpers
│   ├── gripper_profiles.py       # Atomic robot/gripper profile definitions
│   ├── scene_room.py             # ROS-capable Task 3 room
│   ├── scene_robot_room_rmpflow.py
│   ├── run_helper_containers.sh
│   └── run_isaacsim_teleop.sh
├── tests/
├── docker-compose.yml            # ROS/browser/GELLO helper services
└── .env.example                  # Helper defaults
```

The shared room remains at `assets/robot_room.usd`. The Task 3 ROS runtime
reuses the Task 2 plain-Isaac-Sim bridge and Task 1 device adapters rather than
duplicating those implementations.

## Current limitations

- Browser/ROS behavior can be tested without special hardware; the physical
  GELLO + pedal path still requires verification by a device owner.
- Force-limited grasping and a complete four-stage human teleoperation run are
  not yet verified.
- Local grading helpers are not connected to the live teleoperation loop.
- The Robotiq large robot USD is downloaded separately and is not stored in Git.
