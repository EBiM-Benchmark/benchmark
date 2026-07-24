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
| `a` | Strafe left | `A` |
| `d` | Strafe right | `B` |
| `q` | Rotate left | `A+C` |
| `e` | Rotate right | `B+C` |

The publisher emits messages while a key is held or auto-repeated. After no
new message arrives for `--pedal-timeout` (default: 1.0 s), the bridge stops
the base. The adapter also maps `w/s` to `FWD/BACK`, but the current Task 1
bridge does not handle those two tokens, so keyboard forward/backward motion
is not currently available.

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

The bridge publishes one synchronized data sample every four physics steps by
default (`240 Hz / 4 = 60 Hz`). Change the requested rate with
`-- --ros-publish-rate RATE`. Start the simulator first, then run the recorder
from a second host terminal:

```bash
docker exec -it isaac-lab-ros2_jazzy bash -lc \
  '/workspace/EBiM_Challenge/task1_isaacsim/scripts/record_task1_dataset.sh \
   /workspace/EBiM_Challenge/task1_isaacsim/recordings/experiment_001'
```

The recorder writes an MCAP rosbag containing:

| Data | ROS topic | Message |
| --- | --- | --- |
| Left wrist RGB | `/isaac/left_wrist_camera/image_raw` | `sensor_msgs/Image` |
| Right wrist RGB | `/isaac/right_wrist_camera/image_raw` | `sensor_msgs/Image` |
| Head RGB | `/isaac/head_camera/image_raw` | `sensor_msgs/Image` |
| Left arm joint angles | `/isaac/left_joint_states` | `sensor_msgs/JointState` |
| Right arm joint angles | `/isaac/right_joint_states` | `sensor_msgs/JointState` |
| Left gripper opening | `/isaac/left_robotiq_joint_states` | `sensor_msgs/JointState` |
| Right gripper opening | `/isaac/right_robotiq_joint_states` | `sensor_msgs/JointState` |
| Base pose relative to startup | `/isaac/base_pose_relative` | `geometry_msgs/PoseStamped` |
| Base command token | `/isaac/base_command` | `std_msgs/String` |

For each gripper topic, `position[0]` is the Robotiq driver-joint position in
radians. With the current model, approximately `0.0` means fully open and
`0.8` means fully closed. Base commands are recorded as `A`, `B`, `A+C`,
`B+C`, or `NONE`. Camera images, joint states, and base pose from a sample
share one ROS header timestamp; the headerless base-command message is emitted
in the same sample cycle.

Press `Ctrl+C` once in the recorder terminal and wait for rosbag to finish
closing the file. Inspect the result with:

```bash
docker exec -it isaac-lab-ros2_jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   ros2 bag info \
   /workspace/EBiM_Challenge/task1_isaacsim/recordings/experiment_001'
```

If the container reports `Permission denied` while creating `recordings`,
create the directory on the host and grant the container's UID 1000 access
(no `sudo` is required when the repository belongs to the current user):

```bash
mkdir -p task1_isaacsim/recordings
setfacl -m u:1000:rwx,d:u:1000:rwx task1_isaacsim/recordings
```

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
