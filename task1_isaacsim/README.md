# Task 1 - Mobile FR3 Duo Teleoperation (Isaac Lab + Newton)

Task 1 simulates a mobile Franka FR3 Duo equipped with two Robotiq 2F-85
grippers and wrist D405 cameras. The robot runs in Isaac Lab with the Newton
MJWarp backend. A second Newton process runs the VBD cable, board, and fixture
world, while Isaac Sim provides the interactive viewport and visual debugging.

The standard launcher is:

```bash
task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh
```

Current launcher behavior:

- The default robot asset is
  `task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd`.
- The room at `assets/robot_room_v2/robot_room_v2.usdc` is loaded by default.
- `--with-gello-pedal-teleop` Use the Franka GELLO to teleoperate the joints of both arms, while using a foot pedal to control the motion of the mobile base.
- `--with-keyboard-teleop` Both arm TCPs and grippers are controlled directly using the keyboard. Meanwhile, the mobile base is also controlled via the keyboard.
- The Up/Down arrow keys control `franka_spine_vertical_joint` in every
  visible Kit session.
- Base motion comes from `/pedal/state`.

## Architecture

```text
 HOST / DEVICE INPUTS                         THIS REPO (task1_isaacsim)
 ┌───────────────────────────┐
 │ keyboard_state_publisher  │──/keyboard/state──► task1_teleop_adapters
 │                           │                       keyboard_to_base.py
 │ pedal_state_publisher     │──/pedal/state───────────────┐
 │ gello_publisher (L/R)     │──/*/gello/*──┐              │
 └───────────────────────────┘              │              │
                                            ▼              │
                              task1_gello_pedal_teleop     │
                              gello_to_bridge.py           │
                                            │ /bridge/*    │
                                            ▼              │
                                  task1_ros_republisher    │
                                  (topic mapping +         │
                                   gripper calibration)    │
                                            │ /isaac/*     │
        task1_position_controller ──────────┤              │
        task1_browser_controller            │              │
        (optional /isaac/browser/*) ────────┘              │
                                                           │
 KIT WINDOW KEYBOARD (direct, no ROS)                      │
 ┌──────────────────────────────────────────────────────┐  │
 │ --with-keyboard-teleop                               │  │
 │   W/S... + O/L... -> DualArmKeyboardTeleop           │  │
 │                      -> dual RMPflow -> arms/grippers├──┤
 │   Up/Down arrows   -> SpineKeyboardController        ├──┤
 └──────────────────────────────────────────────────────┘  │
                                                           ▼
 ISAAC LAB CONTAINER (ros2_jazzy)             isaaclab_fr3duo_newton_bridge.py
 ┌────────────────────────────────────┐          (Newton/MJWarp robot)
 │ run_cable_vbd_ros_headless.py      │                    │
 │   Newton SolverVBD cable           │◄─/isaac/robotiq_finger_targets
 │   board + fixture collisions       │                    │
 │   4 kinematic finger boxes         │──/cable/body_centers──────────►
 │                                    │──/cable/gripper_collision_boxes►
 └────────────────────────────────────┘
              cable process is always started by the launcher
```

- `/isaac/*`: joint state and command topics published/subscribed by the
  bridge. Browser commands are included only when browser control is enabled.
- `/bridge/*`: raw GELLO commands. `task1_ros_republisher` maps them to
  `/isaac/*` and applies Robotiq open/close calibration.
- `/pedal/state`: base motion tokens (`A`, `B`, `A+C`,
  `B+C`) converted by the bridge into steering and wheel targets.
- Kit keyboard arm control bypasses ROS. While `--with-keyboard-teleop` is
  active, the bridge ignores ROS arm/gripper commands and RMPflow owns those
  targets; base and spine control remain available.
- The bridge sends four live Robotiq inner-finger targets to the cable process.
  The cable process applies them as kinematic collision boxes and returns cable
  points and box poses for Isaac Sim visualization. Cable contact forces are
  not fed back into the robot articulation.

## Repository Layout

```text
benchmark/
|-- assets/
|   `-- robot_room_v2/robot_room_v2.usdc
`-- task1_isaacsim/
    |-- README.md
    |-- docker-compose.yml
    |-- assets/
    |   |-- Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd
    |   `-- embodiments/fr3duo_mobile/
    |-- cable_world/
    |   |-- configs/table_board_fixture_cable.yaml
    |   |-- configs/gripper.yaml
    |   `-- assets/
    |-- scripts/
    |   |-- run_isaaclab_newton_teleop.sh
    |   |-- isaaclab_fr3duo_newton_bridge.py
    |   |-- run_cable_vbd_ros_headless.py
    |   |-- adapters/
    |   `-- controllers/
    |-- services/
    |   |-- browser_controller/
    |   |-- gello_pedal_teleop/
    |   `-- teleop_adapters/
    `-- isaaclab_overlay/
```

## Prerequisites

1. Linux with an NVIDIA GPU and a compatible driver.
2. Docker Engine, Docker Compose v2, and NVIDIA Container Toolkit.
3. `git`, `curl`, and `unzip`.
4. X11 access for the Isaac Sim window:

   ```bash
   xhost +local:docker
   export DISPLAY=${DISPLAY:-:0}
   ```

5. GELLO/pedal operation additionally requires access to the relevant
   `/dev/ttyACM*` and input devices.

## One-time setup

### 1. Clone this repo with submodules

```bash
git clone --recurse-submodules https://github.com/EBiM-Benchmark/benchmark.git
cd benchmark
```

### 2. Download the large Task 1 assets (OneDrive)

The robot USD and cable board meshes are not stored in git. Fetch them with:

```bash
task1_isaacsim/scripts/download_large_assets.sh
```

This downloads a zip from OneDrive and unpacks it into `task1_isaacsim/`, placing:
- `assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd`
- `cable_world/assets/table_board_fixture/Assets/board_segment.usd`
- `cable_world/assets/table_board_fixture/Assets/board_segment_upper_right.usd`

If the direct download fails (OneDrive sometimes needs a manual click), download the
zip from the share link in the script and unzip it into `task1_isaacsim/` yourself,
or pass an override: `LARGE_ASSETS_URL="…" task1_isaacsim/scripts/download_large_assets.sh`.

### 3. Set up the Newton-enabled Isaac Lab container

Task 1 needs Isaac Lab `release/3.0.0-beta2` plus a small `ros2_jazzy` overlay.
This is **not** the repo's `docker/isaac-lab-2.3.2` profile. Full details in
[`isaaclab_overlay/README.md`](isaaclab_overlay/README.md):

```bash
# Clone Isaac Lab next to this repo at the pinned commit.
cd ..
git clone https://github.com/isaac-sim/IsaacLab.git
git -C IsaacLab checkout 0916ea3c0f126821ef1783c7119d248834fc8d0b
cd benchmark

# Apply the overlay (auto-detects ../IsaacLab and this repo).
task1_isaacsim/isaaclab_overlay/apply_overlay.sh

# Build + start the container.
cd ../IsaacLab && ./docker/container.py start ros2_jazzy && cd -
```

After this, `docker ps` lists `isaac-lab-ros2_jazzy` with this repo mounted at
`/workspace/EBiM_Challenge`. Override the checkout location with `ISAACLAB_ROOT`.

### 4. Prepare the GELLO/pedal device repository (optional)
Clone the teleoperation repository. It provides the keyboard / GELLO / pedal publishers.
```bash
cd ..
git clone https://github.com/EBiM-Benchmark/teleoperation.git
cd benchmark
```

## Quick Start

Run these commands from the benchmark repository root.

### Keyboard Teleoperation

```bash
EMBODIMENT=fr3duo_mobile \
bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --with-keyboard-teleop \
  --no-browser
```

`--with-keyboard-teleop` enables arm and gripper control from the Isaac Sim
window. Keyboard base control requires the
keyboard adapter plus the keyboard publisher.

#### Base keyboard control

In a second terminal, start `keyboard_to_base.py` through the helper container:

```bash
cd task1_isaacsim
TELEOP_ADAPTERS=keyboard \
docker compose --profile teleop up -d --no-deps teleop_adapters
docker exec -it task1_teleop_adapters bash
source /opt/ros/jazzy/setup.bash
python3 /workspace/scripts/adapters/keyboard_to_base.py
```

In a third terminal, enter the built
[`teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) workspace and
start its keyboard publisher:

```bash
cd ../teleoperation
pixi shell
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run keyboard_state_publisher keyboard_state_publisher
```

Keep the keyboard-publisher terminal focused while driving the base:

| Key | Base motion | `/pedal/state` token |
| --- | --- | --- |
| `w` | Move forward | `FWD` |
| `s` | Move backward | `BACK` |
| `a` | Strafe left | `A` |
| `d` | Strafe right | `B` |
| `q` | Rotate left | `A+C` |
| `e` | Rotate right | `B+C` |

The publisher emits messages while a key is held or auto-repeated. After no
new message arrives for `--pedal-timeout` (default: 1.0 s), the bridge stops
the base. `FWD/BACK` are accepted from both `keyboard_to_base.py` and the Task
1 browser controller.

The base input path is:

```text
keyboard_state_publisher (/keyboard/state)
  -> task1_teleop_adapters / keyboard_to_base.py
  -> /pedal/state
  -> IsaacLab bridge steering-position and wheel-velocity targets
```

#### Arm and gripper keyboard control

Click the Isaac Sim viewport before using the arm and gripper keys:

| Function | Left arm | Right arm |
| --- | --- | --- |
| TCP +/-X | `W` / `S` | `O` / `L` |
| TCP +/-Y | `A` / `D` | `K` / `;` |
| TCP +/-Z | `Q` / `E` | `I` / `P` |
| Roll +/- | `Z` / `X` | `N` / `M` |
| Pitch +/- | `T` / `G` | `U` / `J` |
| Yaw +/- | `C` / `V` | `,` / `.` |
| Toggle gripper | `F` | `'` |

Additional controls:

- `R`: reset both TCP targets to their startup poses.
- Up/Down arrows: raise/lower the spine.

Keyboard arm control requires a visible Kit window and cannot be used with
`--headless`. The arm keys require the Isaac Sim viewport to have focus, while
the base keys require the keyboard-publisher terminal to have focus. While arm
keyboard control is active, incoming ROS arm and gripper commands are ignored;
base commands continue through `/pedal/state`, and spine control continues
through the Isaac Sim Up/Down key handler.

### GELLO arms and pedal base Teleoperation

```bash
EMBODIMENT=fr3duo_mobile \
bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --with-gello-pedal-teleop \
  --no-browser
```

This starts `task1_gello_pedal_teleop`, which runs the GELLO publisher and
`gello_to_bridge.py`. Start the pedal publisher in a second terminal:

```bash
docker exec -it task1_gello_pedal_teleop bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /tmp/task1_teleop_install/setup.bash && \
   ros2 run pedal_state_publisher pedal_state_publisher'
```

To use two three-button pedals for all six planar base motions, launch with:

```bash
EMBODIMENT=fr3duo_mobile \
bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --with-dual-pedal-teleop \
  --no-browser
```

The dual-pedal publisher starts automatically and uses this mapping:

| Device | Left button | Middle button | Right button |
|---|---|---|---|
| Pedal 1 | Forward | Backward | Translate left |
| Pedal 2 | Translate right | Rotate counterclockwise | Rotate clockwise |

It auto-detects exactly two `PCsensor FootSwitch Keyboard` devices through
`/dev/input/by-path`. Because these pedals expose the same USB ID and no unique
serial number, set stable port paths in `task1_isaacsim/.env` when their order
needs to be explicit:

```bash
PEDAL_ONE_DEVICE=/dev/input/by-path/<first-pedal>-event-kbd
PEDAL_TWO_DEVICE=/dev/input/by-path/<second-pedal>-event-kbd
```

The input devices are grabbed exclusively in dual mode, which prevents their
`a`/`b`/`c` keystrokes from also reaching a focused terminal. Pressing more
than one pedal button at once publishes `STOP`.

### Browser control

The browser service starts by default and is available at:

```text
http://localhost:8090
```

Use `--no-browser` to disable it. When browser control is disabled, the
republisher is configured not to subscribe to `/isaac/browser/*` topics.

### Base and spine

The bridge converts `/pedal/state` into steering position targets and wheel
velocity targets. The relevant runtime parameters are:

```text
--pedal-linear-speed       default 0.5 m/s
--pedal-angular-speed      default 0.5 rad/s
--pedal-timeout            default 1.0 s
--spine-keyboard-step      default 0.01 m
--spine-keyboard-min       default 0.0 m
--spine-keyboard-max       default 0.850 m
```

Pass bridge parameters after the launcher's `--` separator.

## Data Recording

The bridge loads
`assets/embodiments/fr3duo_mobile/data_contract.yaml` at startup and publishes
one synchronized sample at the contract rate (`60 Hz`, normally every four
`240 Hz` physics steps). The actual rate must remain within the contract's 5%
tolerance. Start the simulator first, then run the recorder from a second host
terminal:

An example recording produced by this workflow is available in the public
[Task 1 Isaac Sim Recordings dataset](https://huggingface.co/datasets/Shiqun/task1-isaacsim-recordings/tree/main)
on Hugging Face. It contains a sample MCAP recording with the synchronized
robot state, action, metadata, and compressed RGB topics described below.

```bash
# Run these two commands on the host once to create a writable recording directory.
docker exec -u root isaac-lab-ros2_jazzy \
  mkdir -p /workspace/EBiM_Challenge/task1_isaacsim/recordings
docker exec -u root isaac-lab-ros2_jazzy \
  chown -R ubuntu:ubuntu \
  /workspace/EBiM_Challenge/task1_isaacsim/recordings

# Enter the Isaac Lab container.
docker exec -it isaac-lab-ros2_jazzy bash
```

Inside the container, configure writable ROS log directories and start the
recorder:

```bash
export HOME=/tmp/task1_recorder
export ROS_LOG_DIR="$HOME/ros-log"
mkdir -p "$ROS_LOG_DIR"

cd /workspace/EBiM_Challenge
task1_isaacsim/scripts/record_task1_dataset.sh \
  task1_isaacsim/recordings/experiment_001
```

The recorder stores the fields required by `data_contract.yaml` plus three
JPEG-compressed RGB streams. Robot state and action remain at 60 Hz; images
default to 10 Hz with JPEG quality 85. Publishing the original three raw
`rgb8` streams at 60 Hz would require about 312 MB/s (roughly 560 GB for 30
minutes), so raw image topics are deliberately not recorded.

| Data | ROS topic | Message and order |
| --- | --- | --- |
| Left wrist RGB | `/isaac/left_wrist_camera/image_compressed` | `sensor_msgs/CompressedImage`, JPEG, 848x480 |
| Right wrist RGB | `/isaac/right_wrist_camera/image_compressed` | `sensor_msgs/CompressedImage`, JPEG, 848x480 |
| Head RGB | `/isaac/head_camera/image_compressed` | `sensor_msgs/CompressedImage`, JPEG, 1280x720 |
| Base state | `/isaac/data_contract/base_state` | `Float32MultiArray`: `[x, y, theta, vx, vy, omega]` |
| Arm state | `/isaac/data_contract/arm_state` | `JointState`: 7 left then 7 right joints, with position, velocity, and applied effort |
| Gripper state | `/isaac/data_contract/gripper_state` | `Float32MultiArray`: `[left_open_fraction, right_open_fraction]` |
| Applied action | `/isaac/data_contract/action` | `Float32MultiArray`: base 3, left arm 7, right arm 7, grippers 2 |
| Sample timestamp | `/isaac/data_contract/timestamp` | `Float64`, seconds |
| Episode step count | `/isaac/data_contract/step_count` | `UInt64`, zero-based recorded step |

The 19-dimensional action is ordered exactly as specified by
`data_contract.yaml`: `[vx, vy, omega, left_arm_0..6, right_arm_0..6,
left_opening, right_opening]`. Arm actions are the position targets actually
held by Isaac Lab after command arbitration. Gripper state and action use the
contract's normalized semantics: `1.0` is fully open and `0.0` is fully
closed. Base pose is relative to the startup pose, and base velocity is
expressed in the base frame.

Compatibility and visualization topics may still be published by the bridge,
but the recorder deliberately excludes them. The explicit float64 timestamp
and sample index synchronize the headerless contract arrays with the
header-bearing arm state. Each compressed image uses the timestamp of the
corresponding 60 Hz contract sample. Override the image rate and quality after
the launcher's `--` separator, for example:

```bash
-- --camera-publish-rate 15 --camera-jpeg-quality 90
```

Higher rates and JPEG quality increase ROS bandwidth, CPU encoding load, and
recording size.

Press `Ctrl+C` once in the recorder terminal and wait for rosbag to finish
closing the file. Inspect the result with:

```bash
docker exec -it isaac-lab-ros2_jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   ros2 bag info \
   /workspace/EBiM_Challenge/task1_isaacsim/recordings/experiment_001'
```

To replay a recording and inspect one of its JPEG-compressed camera streams,
open three terminals in the Isaac Lab container. In the first terminal, replay
the bag in a loop:

```bash
ros2 bag play \
  /workspace/EBiM_Challenge/task1_isaacsim/recordings/experiment_001 \
  --loop
```

In the second terminal, decode the selected JPEG stream into a raw image topic:

```bash
ros2 run image_transport republish \
  --ros-args \
  -p in_transport:=compressed \
  -p out_transport:=raw \
  --remap in/compressed:=/isaac/left_wrist_camera/image_compressed \
  --remap out:=/isaac/left_wrist_camera/image_view
```

In the third terminal, start the image viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/isaac/left_wrist_camera/image_view` in `rqt_image_view`. To inspect a
different camera, replace `left_wrist_camera` in both remappings with
`right_wrist_camera` or `head_camera`. Source `/opt/ros/jazzy/setup.bash` first
in any shell where the ROS 2 commands are not already available. Press
`Ctrl+C` in the playback terminal to stop replaying the recording.

The `docker exec -u root` commands above modify only the ownership of the
bind-mounted `recordings` directory. They do not run the recorder as root.
`HOME` and `ROS_LOG_DIR` prevent ROS 2 from trying to write logs under
`/root`; these environment variables apply only to the current container
shell and must be set again after opening a new shell.

## Cable World

The launcher always starts the raw Newton VBD cable process. There is no cable
enable/disable command-line switch. The defaults are:

```text
config:            cable_world/configs/table_board_fixture_cable.yaml
gripper config:    cable_world/configs/gripper.yaml
device:            cuda:0
world translation: (1.5, 0.0, 0.73) m
world yaw:         90 degrees
finger box size:   (0.02, 0.007, 0.03) m
finger offsets:    X=0.01, Y=-0.045, Z=-0.010 m
invert opening:    false
```

Override bridge-side cable placement after `--`, for example:

```bash
bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --with-keyboard-teleop --no-browser -- \
  --cable-world-position-offset 1.5 0.0 0.73 \
  --cable-world-yaw-deg 90 \
  --cable-robotiq-finger-size 0.02 0.007 0.03
```

`--cable-robotiq-invert-opening` uses
`argparse.BooleanOptionalAction`, so both forms exist:

```text
--cable-robotiq-invert-opening
--no-cable-robotiq-invert-opening
```

The default is `false`, so omitting both is equivalent to the `--no-...` form.
Use `--show-table-board-fixture-collisions` to display collision meshes under
`/World/TableBoardFixtureVisual` for debugging.

Cable log:

```bash
docker exec isaac-lab-ros2_jazzy tail -f /tmp/task1_cable_vbd.log
```

## Launcher Options

```text
--embodiment NAME
--usd-path PATH
--controller-mode none|position
--with-gello-pedal-teleop
--with-keyboard-teleop
--no-browser
--no-republisher
--headless
--
```

`--usd-path` is relative to `task1_isaacsim/`. Its default is the Robotiq
robot USD listed above. Arguments after `--` are forwarded to
`isaaclab_fr3duo_newton_bridge.py`.

Useful environment overrides:

```text
ISAACLAB_ROOT
ISAACLAB_CONTAINER
CONTAINER_REPO
CABLE_DEVICE
CABLE_CONFIG_PATH
CABLE_GRIPPER_CONFIG_PATH
CABLE_LOG_PATH
```

## Helper Containers

| Container | Started when | Purpose |
| --- | --- | --- |
| `isaac-lab-ros2_jazzy` | Always | Isaac Lab robot process and cable VBD process |
| `task1_ros_republisher` | Default | `/bridge/*` to `/isaac/*`; gripper calibration |
| `task1_position_controller` | `--controller-mode position` (default) | Holds commanded arm/gripper targets |
| `task1_browser_controller` | Unless `--no-browser` | Browser control on port 8090 |
| `task1_gello_pedal_teleop` | `--with-gello-pedal-teleop` | GELLO publisher, GELLO adapter, pedal package |
| `task1_teleop_adapters` | Manual Compose profile | Optional keyboard-to-base adapter |

The launcher does not currently start `task1_teleop_adapters`. Start it
manually when an external `/keyboard/state` publisher should drive the base:

```bash
cd task1_isaacsim
docker compose --profile teleop up -d --no-deps teleop_adapters
```

## Troubleshooting

- **Default USD not found:** run `download_large_assets.sh` and confirm
  `task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd`.
- **Room USD not found:** confirm
  `assets/robot_room_v2/robot_room_v2.usdc`, or pass `-- --no-room`.
- **Repository is not mounted in IsaacLab:** reapply
  `task1_isaacsim/isaaclab_overlay/apply_overlay.sh` and recreate
  `isaac-lab-ros2_jazzy`.
- **Keyboard arms do not move:** use a visible Kit window, click the viewport,
  and confirm `--with-keyboard-teleop` appears before the `--` separator.
- **GELLO container restarts:** inspect
  `docker logs task1_gello_pedal_teleop --tail=200` and verify
  `TELEOPERATION_ROOT` plus `/dev/serial/by-id` access.
- **Cable is missing:** inspect `/tmp/task1_cable_vbd.log` and verify
  `/cable/body_centers` is being published.
- **ROS topics do not cross host/container boundaries:** use
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and the same `ROS_DOMAIN_ID`.
- **No GUI window:** run `xhost +local:docker`, verify `DISPLAY`, and recreate
  the IsaacLab container if its X11 mount is stale.
