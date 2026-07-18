# Task 2 — Mobile FR3 Duo Teleoperation (Isaac Sim 5.1.0 / PhysX)

## Overview

Teleoperation of the mobile FR3 Duo for Task 2 (deformable thermal pad placement)
in **Isaac Sim 5.1.0 (PhysX)**. This task requires PhysX GPU deformables, since the thermal pad asset uses `PhysxDeformableBodyAPI`.

### Objectives
- Transport the highly deformable pad without damaging it.
- Align and attach the pad onto the designated PCB target area.

### Scoring
- **Primary:** valid-placement IoU — Pick Success × Placement Orientation Success × Placement IoU (0–1); wrong orientation scores 0.
- **Tie-breaker:** completion time — faster is better.

The evaluation code in this repository ([Task 2 evaluation](../scripts/evaluation/task2/README.md#evaluation-metric)) is a **development facilitator**; official scoring follows the rules and scoring
published on the **[competition page](https://ebim-benchmark.github.io/competition.html#tasks)**.

## Prerequisites

1. Linux host with a supported NVIDIA GPU + recent driver.
2. Docker Engine with Docker Compose v2 and the NVIDIA Container Toolkit.
3. **Isaac Sim 5.1.0 container running** with this repo bind-mounted at
   `/workspace/EBiM_Challenge` (default container name
   `isaac-sim-5-1-0-workshop`). The
   container needs no ROS 2 install — the bridge uses the ROS 2 jazzy
   libraries bundled with Isaac Sim's `isaacsim.ros2.bridge` extension.
   Start the container (from root directory) with:
   ```bash
   docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
   --profile isaac-sim-5.1.0 up -d
   ```
4. **Robot USD downloaded**: run `task1_isaacsim/scripts/download_large_assets.sh` and ensure the robot USD is present at `task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd`.
5. **Teleop input device publishers** (keyboard / GELLO / pedal) from the [`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) repository on the host.

## Teleoperation

This repository provides a **ROS 2 bridge** between the teleoperation input devices and the Isaac Sim 5.1.0 simulator.

### Mobile base

Mobile base teleoperation is supported via:
- **Keyboard**: via `keyboard_state_publisher` from the [`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) repository.
- **USB foot pedal**: via `pedal_state_publisher` from the [`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) repository.

### Spine

Robot vertical spine is controlled via:
- **Keyboard**: Up/Down keys, with the Isaac Sim window focused.

### Arms

Dual arms teleoperation is supported via:
- **Keyboard**: via RMPflow (Lula) policies, with the Isaac Sim window focused.
- **GELLO**: via the `franka_gello_state_publisher` from the [`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) repository.
- **Web UI**: via the `task2_browser_controller` Docker Compose service (accessed via <http://localhost:8090>) in the helper stack. This is a no-hardware alternative to the GELLO arms: it controls the joint states directly from UI sliders.

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

### Barebone (empty scene) with keyboard arms and base (no special hardware)

```bash
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
   --scene barebone \
   --with-keyboard-teleop \
   --with-arm-keyboard-teleop \
   --controller-mode none \
   --no-republisher \
   --no-browser
```

#### Base: Keyboard

Then, in the teleoperation ROS 2 environment on the host, start the keyboard
publisher and drive the base with `w/a/s/d` and rotate with `q/e`. The terminal window must have focus for the keyboard to work.

```bash
ros2 run keyboard_state_publisher keyboard_state_publisher
```

#### Arms: Keyboard

While the Isaac Sim window has focus, drive the arms with the following keys:

| Keys | Action |
| --- | --- |
| `W/S` `A/D` `Q/E` | LEFT arm: move TCP fwd/back, left/right, up/down |
| `Z/X` `T/G` `C/V` | LEFT arm: roll / pitch / yaw |
| `F` | LEFT gripper toggle |
| `O/L` `K/;` `I/P` | RIGHT arm: move TCP fwd/back, left/right, up/down |
| `N/M` `U/J` `,/.` | RIGHT arm: roll / pitch / yaw |
| `'` | RIGHT gripper toggle |
| `R` | reset both arm targets to the ready pose |

This drives both arm end-effectors from the Isaac Sim window keyboard (with GUI focused) through per-arm RMPflow (Lula) policies. Targets are held in the robot base frame, so the arms ride along while the base drives and the keys always move the gripper relative to the robot's heading. Each arm has its own key cluster so both arms can move at once.

Notes:
- Conflicting bare-key viewport hotkeys (`F` frame selection, `Q/W/E/R`
  transform tools, ...) are deregistered at startup.
- Speeds are tunable via `-- --arm-teleop-linear-speed 0.18
  --arm-teleop-angular-speed-deg 60`.
- In `--headless` runs the teleop is disabled (with a warning) and ROS arm
  commands stay active.

#### Spine: Keyboard

The spine keyboard control is `Up/Down`, with Isaac Sim GUI focused.

### Room scene with keyboard base + web browser arms (no special hardware)

```bash
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
   --scene room \
   --with-keyboard-teleop
```

#### Base: Keyboard

Then, in the teleoperation ROS 2 environment on the host, start the keyboard
publisher and drive the base with `w/a/s/d/q/e`:

```bash
ros2 run keyboard_state_publisher keyboard_state_publisher
```

#### Arms: Web UI

Open the web UI at <http://localhost:8090> to directly control the joint state of the arms/grippers.

#### Spine: Keyboard

The spine keyboard control is `Up/Down`, with Isaac Sim GUI focused.

### Room scene with foot pedal base + GELLO arms

```bash
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
   --scene room \
   --with-gello-teleop \
   --no-browser
```

#### Base: Foot Pedal + Arms: GELLO

On the host (teleoperation env): launch the GELLO publisher and the pedal
publisher (see the `teleoperation` repo README):

```bash
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
ros2 run pedal_state_publisher pedal_state_publisher
```

#### Spine: Keyboard

The spine keyboard control is `Up/Down`, with Isaac Sim GUI focused.

## Demonstration recording (LeRobot dataset)

Record teleoperation demonstrations as a **LeRobot dataset** (20-dim
action, 37-dim state, four RGB video streams) plus a `task2_extras/`
ground-truth sidecar. The recorder runs as its own docker compose service
in [services/recording/](services/recording/); the dataset schema and
stream details are documented in the
[Pipeline Reference](PIPELINE_REF.md#dataset-schema).

1. **Simulator** (Isaac Sim container) — room or barebone scene with all
   recording publishers (`--record` enables the robot cameras + the scene
   cameras + `/isaac/clock`, the recording streams, the ground-truth
   publishers, and the scene-reset hotkey):

   ```bash
   task2_isaacsim/scripts/run_isaacsim_teleop.sh --scene room -- --record --arm-keyboard-teleop
   ```

   (add `--randomize-objects` to jitter the object spawns on every reset,
   and `--robot-camera-depth` if you want depth topics.)

   Robot cameras are described by
   [assets/embodiments/fr3duo_mobile_task2/camera_sensors.yaml](assets/embodiments/fr3duo_mobile_task2/camera_sensors.yaml)
   (override with `--camera-sensors-yaml`); scene cameras — the top-down
   `eval_camera` and any you add — by
   [config/cameras_room.yaml](config/cameras_room.yaml) /
   [config/cameras_barebone.yaml](config/cameras_barebone.yaml)
   (`--enable-scene-cameras` to turn them on without the full `--record`
   bundle, `--scene-cameras-config` to point at your own file).

2. **Teleop helper stack** as needed (keyboard/GELLO/browser — see
   Quickstart).

3. **Recorder**:

   ```bash
   # first run builds its container image
   task2_isaacsim/scripts/run_recorder.sh
   ```

   Terminal controls (single keypress, no Enter needed) — idle: `1`
   reset/randomize the scene, then start recording · `2` start recording
   without reset (the episode starts at the current sim time; useful
   after manually reposing the scene) · `5` reset/randomize only (same
   key as the reset hotkey in the Isaac Sim window) · `4` visualize ·
   `q` quit. While recording: `3` stop + save (confirms the success
   label, showing the IoU suggestion) · `0` stop + discard · `q` quit
   (discards the episode). Only reset between episodes, never while
   recording (the recorder detects the clock jump and discards).

   Default dataset save path is `task2_isaacsim/dataset/task2_thermalpad_vN/`;
   each launch starts a new version. Append to an existing version with
   `run_recorder.sh record --resume` (or `--resume-version N`).

   Recording defaults (repo name, fps, cameras, episode limits, …) live in
   [services/recording/recording.yaml](services/recording/recording.yaml).
   To override, put parameters after `--` when using helper script:

   ```bash
   task2_isaacsim/scripts/run_recorder.sh record -- --fps 20 --record-depth

   # or when using docker compose
   RECORDER_ARGS="--fps 20 --record-depth" \
   cd task2_isaacsim && docker compose --profile record run --rm lerobot_recorder
   ```

## Architecture

Same five-stage pipeline as Task 1, only the last stage (the simulator
process) differs: host device publishers → teleop adapters (or the browser
UI) → republisher / position controller → the scene script inside the Isaac
Sim container. The scene scripts (`scene_room.py` / `scene_barebone.py`) are
thin stage builders on top of the shared teleop runtime in
`scripts/isaacsim_fr3duo_teleop_bridge_core.py`. All Task 2 ROS topic names
come from a single contract file,
[`config/topics.yaml`](config/topics.yaml).

The full technical reference — container topology, per-topic contract
tables, configuration precedence, dataset schema, and the mapping to the
reused Task 1 scripts — lives in the
**[Pipeline Reference](PIPELINE_REF.md)**.

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
