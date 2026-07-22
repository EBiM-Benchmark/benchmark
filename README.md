# EBiM Benchmark

**Current capability status: see [STATUS.md](STATUS.md)** — this is a developer preview; check what's usable before you build.

## Competition tasks

| Task | Engines | Where in this repo | Status |
|---|---|---|---|
| Task 1 — Cable Routing & Plugging | Isaac Sim, MuJoCo | [`task1_isaacsim/`](task1_isaacsim/), [`task1_mujoco/`](task1_mujoco/) | see [STATUS.md](STATUS.md) |
| Task 2 — Deformable Material Handling (Thermal Pad Placement) | Isaac Sim (Genesis committed) | [`task2_isaacsim/`](task2_isaacsim/), [`assets/task2_objects/`](assets/task2_objects/), [`scripts/evaluation/task2/`](scripts/evaluation/task2/) | see [STATUS.md](STATUS.md) |
| Task 3 — Assisted Living & Feeding | Isaac Sim (MuJoCo committed) | [`task3_isaacsim/`](task3_isaacsim/), [`scripts/evaluation/task3/`](scripts/evaluation/task3/) | see [STATUS.md](STATUS.md) |

Full rules and official scoring are on the competition page: https://ebim-benchmark.github.io/competition.html#tasks . The evaluation code in this repository is a development facilitator; official scoring follows the rules published there.

This repository provides a workshop-focused environment for an international competition. The active workflow uses `assets/robot_room.usd` as the base scene and launches the mobile dual-arm robot through Isaac Sim. Older tabletop scene generators are kept only for reference.

For the full developer workflow, see [`docs/developer_setup.md`](docs/developer_setup.md).

## Task 1 — Mobile FR3 Duo Teleoperation (Isaac Lab + Newton)

[`task1_isaacsim/`](task1_isaacsim/README.md) contains the Isaac Sim / Isaac Lab
implementation of Task 1: teleoperating the mobile dual-arm FR3 Duo on the
Newton / MJWarp backend, with an optional deformable-cable board-plugging world.
Keyboard is the default mobile-base input; GELLO leader arms + USB foot pedal is
the tested configuration. The teleoperation input devices come from the separate
[`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation)
repository. The MuJoCo variant lives in [`task1_mujoco/`](task1_mujoco/README.md)
(next section).

See [`task1_isaacsim/README.md`](task1_isaacsim/README.md) for full setup and run
instructions. Quick start (from the repo root, after the one-time setup):

```bash
EMBODIMENT=fr3duo_mobile bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --usd-path assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd \
  --controller-mode position --with-keyboard-teleop
```

## Task 1 — Cable Management (MuJoCo)

[`task1_mujoco/`](task1_mujoco/README.md) contains the MuJoCo implementation of
Task 1 for the ManipulationNet **cable_management** benchmark: a mobile
dual-arm FR3 platform with Robotiq 2F-85 grippers routing a deformable cable
across a fixture board. Five input modes (keyboard / gamepad / VR / GELLO /
unified ROS 2 teleop), a single in-sim IK shared by all of them. The
directory is self-contained — a native one-click launcher, or via Docker:

```bash
cd task1_mujoco
./start.sh              # native teleoperation (Windows: double-click start.bat)
./docker-run.sh         # or via Docker (no ROS 2 / conda install needed)
```

See [`task1_mujoco/README.md`](task1_mujoco/README.md) for the full participant
guide (paths, input modes, controls, troubleshooting).

## Task 2 — Mobile FR3 Duo Teleoperation (Isaac Sim 5.1.0 / PhysX)

[`task2_isaacsim/`](task2_isaacsim/README.md) contains the Task 2 teleoperation
stack: driving the mobile FR3 Duo to place the deformable thermal pad, running
in plain **Isaac Sim 5.1.0 (PhysX)** because the pad asset needs PhysX GPU
deformables (Isaac Lab + Newton cannot run it). It reuses the Task 1 helper
containers (adapters, browser controller, republisher/position controller) and
the same ROS topic contract, and works with either the full robot room
(`--scene room`, which also publishes the `/isaac/eval_camera/*` topics for the
[Task 2 evaluation stack](scripts/evaluation/task2/)) or a barebone scene.
Input devices come from the same
[`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation)
repository.

Quick start (from the repo root, with the Isaac Sim 5.1.0 container running and
the robot USD downloaded — no special hardware, keyboard base + browser arms):

```bash
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
  --scene barebone \
  --with-keyboard-teleop
```

See [`task2_isaacsim/README.md`](task2_isaacsim/README.md) for prerequisites,
the GELLO + foot-pedal configuration, and the architecture.

## Task 3 — Assisted Living & Feeding (Isaac Sim 5.1.0)

[`task3_isaacsim/`](task3_isaacsim/README.md) contains the runnable Task 3
preview with direct keyboard control and a ROS/browser/GELLO bridge. The ROS
launcher selects the complete robot/gripper profile with `--gripper`: Robotiq
is the competition default, while Panda preserves the current Franka-hand
asset.

```bash
bash task3_isaacsim/scripts/run_isaacsim_teleop.sh --gripper robotiq
```

See [`task3_isaacsim/README.md`](task3_isaacsim/README.md) for Docker setup,
no-hardware browser control, GELLO/pedal commands, and current limitations.

## Repository Layout

```text
benchmark/
├── task1_isaacsim/              # Task 1: mobile FR3 Duo teleoperation (Isaac Lab + Newton)
├── task1_mujoco/                # Task 1: cable-management teleoperation + eval (MuJoCo)
├── task2_isaacsim/              # Task 2: thermal-pad teleoperation (Isaac Sim 5.1.0 / PhysX)
├── task3_isaacsim/              # Task 3: assisted-living teleoperation (Isaac Sim 5.1.0)
├── assets/                      # USD assets and generated scene files
│   └── tabletop_task_scene_DEMO # Scene with Commandable via ROS mobile_Fr3_duo
├── docker/                      # Docker Compose runtimes for Isaac Sim and Isaac Lab
├── docs/                        # Images and supporting documentation assets
├── newton/                      # Newton physics engine submodule
├── scripts/
│   ├── common/                  # Shared path and control helpers
│   ├── manual_tests/            # Small validation scenes for assets
│   ├── newton_examples/         # Standalone Newton quick-launch examples
│   ├── scenes/                  # Main workshop demos and scene scripts
│   └── tools/                   # USD composition and inspection utilities
├── third_party/
│   └── franka_description/      # Franka robot description submodule
├── .gitmodules                  # Submodule metadata
├── pyproject.toml               # Repository-wide lint/type-check configuration
└── README.md
```

## Cloning With Submodules

Clone this repository with all submodules initialized:

```bash
git clone --recurse-submodules <repository-url>
```

If the repository was already cloned without submodules, initialize them afterward:

```bash
git submodule update --init --recursive
```

To update submodules to the commits recorded by the current checkout:

```bash
git submodule update --init --recursive
```

The current submodules are:
- `newton/`
- `third_party/franka_description/`

## Git LFS Notes

Some large workshop assets may be tracked with Git LFS instead of regular Git blobs.

Before cloning or pulling LFS-tracked assets, install and enable Git LFS once on your machine:

```bash
git lfs install
```

After that, normal Git commands are usually enough:

```bash
git clone --recurse-submodules <repository-url>
git pull
```

If Git LFS is installed, the real large files are downloaded automatically during clone and pull. If Git LFS is not installed, Git will only check out small pointer files instead of the actual `.usd` or `.blend` assets. If that happens, run:

```bash
git lfs pull
```

To inspect which files are currently tracked through Git LFS:

```bash
git lfs ls-files
```

GitHub charges Git LFS storage and download bandwidth to the repository owner. If this repository is owned by an organization such as `HCIS-Lab`, pushes to its LFS-tracked files consume the organization's Git LFS quota, not the pusher's personal quota.

On a local checkout, Git LFS stores downloaded objects under `.git/lfs/objects`. On GitHub, the repository history stores pointer files, while the actual large-file content is stored in GitHub's managed Git LFS object storage for the repository.

## Supported Container Targets

The Docker stack is parameterized in `docker/.env.base` and `docker/docker-compose.yaml`.

### Isaac Sim 5.1.0
- Image: `nvcr.io/nvidia/isaac-sim:5.1.0`
- Local tag: `isaac-sim-5.1.0:ebim2026`
- Compose profile: `isaac-sim-5.1.0`
- Intended for GUI and simulation workflows with X11 support.

### Isaac Sim 6.0.0-dev2
- Image: `nvcr.io/nvidia/isaac-sim:6.0.0-dev2`
- Local tag: `isaac-sim-6.0.0-dev2:ebim2026`
- Compose profile: `isaac-sim-6.0.0`
- Uses the currently documented pre-GA container tag.

### Isaac Lab 2.3.2
- Image: `nvcr.io/nvidia/isaac-lab:2.3.2`
- Local tag: `isaac-lab-2.3.2:ebim2026`
- Compose profile: `isaac-lab-2.3.2`
- Documented as an alternative runtime. The primary workshop workflow remains the Isaac Sim images above.

## Prerequisites

1. Linux host with a supported NVIDIA GPU.
2. Docker Engine with Docker Compose v2.
3. NVIDIA Container Toolkit configured for Docker.
4. X11 available on the host for GUI workflows.
5. Permission to pull NVIDIA NGC images.

Before launching GUI containers, allow local X11 access on the host:

```bash
xhost +local:docker
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
touch "$XAUTHORITY"
```

## Persistent Docker Storage

All container caches and runtime data are stored under:

```text
${HOME}/docker/ebim-challenge
```

Create the required directories before the first launch. A typical layout is:

```text
~/docker/ebim-challenge/
├── isaac-sim-5.1.0/
│   ├── cache/main/ov
│   ├── cache/main/warp
│   ├── cache/computecache
│   ├── config
│   ├── data/documents
│   ├── data/Kit
│   ├── logs
│   └── pkg
├── isaac-sim-6.0.0/
│   ├── cache/main/ov
│   ├── cache/main/warp
│   ├── cache/computecache
│   ├── config
│   ├── data/documents
│   ├── data/Kit
│   ├── logs
│   └── pkg
└── isaac-lab-2.3.2/
    ├── cache/kit
    ├── cache/ov
    ├── cache/pip
    ├── cache/glcache
    ├── cache/computecache
    ├── data
    ├── documents
    └── logs
```

For writable bind mounts from both the host and containers, the Isaac Sim
services run with `${HOST_UID}:${HOST_GID}` as their UID/GID and add
`${ISAAC_SIM_GID}` as a supplemental group so they can still access
`/isaac-sim`. Their `HOME` and XDG cache/data/config paths are pinned under
`/isaac-sim` so Omniverse does not try to write under `/`. `HOST_UID`/`HOST_GID`
must match the owner of this repository; the defaults in `docker/.env.base` are
set for this workspace. If your host user uses different IDs, export them before
building and running Compose:

```bash
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)
```

Bootstrap the versioned cache layout with:

```bash
python3 scripts/tools/validate_docker_runtimes.py --prepare-dirs --skip-script-check
sudo chown -R "${HOST_UID:-$(id -u)}:${HOST_GID:-$(id -g)}" \
  "$HOME/docker/ebim-challenge/isaac-sim-5.1.0" \
  "$HOME/docker/ebim-challenge/isaac-sim-6.0.0"
sudo chmod -R g+rwX \
  "$HOME/docker/ebim-challenge/isaac-sim-5.1.0" \
  "$HOME/docker/ebim-challenge/isaac-sim-6.0.0"
```

The compose stack persists the main Kit cache, CUDA compute cache,
Omniverse data/config, Kit data, logs, and package data. It intentionally does
not bind-mount `/isaac-sim/extscache`, because those extension cache folders
also contain required bundled shader resources; an empty host directory there
would hide them and break RTX shader loading.

## Docker Quick Start

Run all commands from the repository root.

The compose file depends on values from `docker/.env.base`. Pass it explicitly:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml config --profiles
```

### Build and validate all runtimes

```bash
python3 scripts/tools/validate_docker_runtimes.py \
  --prepare-dirs \
  --build \
  --up
```

This builds the three local images in parallel, starts the containers, and checks workspace mounts, cache mounts, X11, host networking, and script/USD smoke tests.

### Start Isaac Sim 5.1.0

Build the local Isaac Sim 5.1.0 runtime image:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 build isaac-sim-5-1-0
```

Start the container:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 up -d
```

Enter the container:

```bash
docker exec -it isaac-sim-5-1-0-workshop bash
```

Typical GUI launch inside the container:

```bash
./runapp.sh
```

### Launch Mobile FR3 In The Robot Room

After starting a runtime, use the participant launcher documented in the
corresponding task folder. The shared robot-room builder is an implementation
module, not the participant entry point. See the Task 1, Task 2, and Task 3
README links in the task overview above.

### Start Isaac Sim 6.0.0-dev2

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-6.0.0 up -d
```

Enter the container:

```bash
docker exec -it isaac-sim-6-0-0-workshop bash
```

Typical GUI launch inside the container:

```bash
./runapp.sh
```

### Start Isaac Lab 2.3.2

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-lab-2.3.2 up -d
```

Enter the container:

```bash
docker exec -it isaac-lab-2-3-2-workshop bash
```

Stop all containers again with:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml down
```

## Workspace Mounts

The full repository is mounted into each container at:

```text
/workspace/EBiM_Challenge
```

This makes live editing from the host available in all supported container targets.

## X11 Notes

The compose file mounts:
- `${DISPLAY}`
- `${XAUTHORITY}`
- `/tmp/.X11-unix`

If GUI applications fail to open:
1. confirm `xhost +local:docker` has been executed for the current graphical session,
2. verify `DISPLAY` is exported,
3. verify `XAUTHORITY` points to a valid file,
4. restart the container after changing those variables.

## Main Workshop Scripts

### Demo Scenes
- `scripts/scenes/scene_robot_room_keyboard.py` — shared robot-room stage builder used by task-specific launchers.

### Utilities
- `scripts/tools/inspect_usd.py` — print the prim hierarchy of a USD file.

<details>
<summary>Outdated scene generators and demos</summary>

- `scripts/deprecated/scene_robot_keyboard.py` — older tabletop scene with keyboard control.
- `scripts/deprecated/scene_robot_tables.py` — older tabletop scene with robot but without keyboard control.
- `scripts/deprecated/scene_11_tables.py` — older 11-table composition utility and preview.
- `scripts/deprecated/scene_with_table.py` — older single-table placement example.
- `scripts/deprecated/keyboard_control.py` — older reduced robot keyboard-control demo.
- `scripts/deprecated/launch_random_heads_scene.py` — older tabletop head randomization launcher.
- `scripts/deprecated/create_wall_room.py` — older wall-room USD generator. The current base room is `assets/robot_room.usd`.
- `scripts/deprecated/compose_scene_usd.py` — deprecated tabletop scene composer kept for reference. Active task scene composition is documented in each task folder.

</details>

### Manual Validation Scenes
- `scripts/manual_tests/test_table_cutlery.py` — validate table plus cutlery placement.
- `scripts/manual_tests/test_table_letter.py` — validate table plus letter placement.

### Keyboard Teleoperation  tabletop_task_scene_DEMO

All keyboard teleoperation logic is fully integrated into the Action Graph within the USD file. You can control the robotic arms, grippers, and the waist vertical joint directly through your keyboard simply by switching to the viewpoint:

Instant Activation: Click the viewpoint in the viewport to immediately enable keyboard control.

Unified Control: No external terminal scripts are required; the Action Graph handles all key mappings internally for seamless bimanual and chassis coordination.

#### 1.1 Control the TMR Chassis Motion

Run the keyboard teleop node to control the movement of the TMR omnidirectional chassis:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p holonomic:=true

```


#### 2.1 Grpper joints Control

Numpad 1 & 2: Control the right arm gripper (Open / Close).

Numpad 3 & 4: Control the left arm gripper (Open / Close).

#### 2.2 Waist Vertical Control

Numpad 5: Raises the waist vertical joint by 0.1m (adjustable range: 0.0m to 0.85m).



#### 2.2 Arm Joint Control

##### Right Arm Control (Viewed on the Left Side)
* **Translation (X, Y):** Press `W` / `A` / `S` / `D` to move along the X and Y axes.
* **Translation (Z):** Press `Q` / `E` to move up and down along the Z axis.
* **Rotation:** Hold `Left Shift` + `W` / `A` / `S` / `D` / `Q` / `E` to rotate the end-effector around the respective axes.

##### Left Arm Control (Viewed on the Right Side)
* **Translation (X, Y):** Press `I` / `J` / `K` / `L` to move along the X and Y axes.
* **Translation (Z):** Press `U` / `O` to move up and down along the Z axis.
* **Rotation:** Hold `Left Shift` + `I` / `J` / `K` / `L` / `U` / `O` to rotate the end-effector around the respective axes.

## LeRobot dataset recording
The `DEMO/record.py` script automatically subscribes to the corresponding ROS 2 topics, synchronizes the multi-modal streams, and aggregates them into the structured dataset:

* **State Data (States):**
  * **Manipulators:** 14 joint positions and velocities across both arms (14 joints total).
  * **End-Effectors:** Gripper poses for both left and right grippers.
  * **Mobile Base:** Linear velocity and angular velocity of the chassis.
* **Camera Views (Visual Inputs):**
  * `camera_left`: Wrist camera mounted on the left gripper.
  * `camera_right`: Wrist camera mounted on the right gripper.
  * `camera_head`: Head-mounted camera.
  * `camera_front`: Static observer/front camera.
###  Environment Setup

Before recording, you must set up the required virtual environment. Please follow the detailed installation guidelines available at:

🔗 [lerobot_ros2 Environment Setup Guide](https://github.com/fiveages-sim/lerobot_ros2/tree/main)
###  Dataset Recording Steps

1. Ensure your virtual environment is activated and the required ROS 2 topics are active and publishing data.
2. Navigate to the `DEMO` directory:
   ```bash
   cd IROS_Workshop/DEMO
3. Execute the recording script:
      ```bash
   python record.py

 Recording Control via Terminal:
  *  Enter `2`: Start recording the dataset.
  *  Enter `3`: Stop recording and automatically save the episode.
###  Dataset Visualization
Once the recording is complete, you can inspect and replay the collected dataset using the visualize_dataset tool from LeRobot.
#### Base Command
   ```bash
PYTHONPATH=submodules/lerobot/src python -m lerobot.scripts.visualize_dataset
```
#### Argument Descriptions:

You can append the following arguments to specify the target dataset and subset:

* `--repo-id`: The unique identifier/name of the dataset repository.
* `--root`: The root directory path where your local datasets are stored.
* `--episode-index`: Specifies which episode to visualize (e.g., `--episode-index 0` loads the first recorded episode).

#### Complete Example:

```bash
PYTHONPATH=submodules/lerobot/src python -m lerobot.scripts.visualize_dataset --root ./data --repo-id mobile_dual_arm_test --episode-index 0
```


## Running Scripts

Task-specific launchers use the prebuilt `assets/robot_room.usd` base scene
through the shared `scripts/scenes/scene_robot_room_keyboard.py` builder. New
workshop task work should build on that room instead of generating new base
scenes.

Inspect the active robot-room USD hierarchy:

```bash
python scripts/tools/inspect_usd.py assets/robot_room.usd
```

<details>
<summary>Outdated scene generators</summary>

These scripts are kept for reference only. They do not define the current
competition base scene.

`scripts/deprecated/create_wall_room.py` creates a room USD asset.

- `--output PATH`: base output path. Default: `assets/plain_white_room.usd`. The script appends room dimensions, and `_partition` when enabled.
- `--length METERS`: inside room length along Y. Default: `30.0`.
- `--width METERS`: inside room width along X. Default: `20.0`.
- `--height METERS`: wall height. Default: `3.0`.
- `--wall-thickness METERS`: wall thickness. Default: `0.1`.
- `--material-preset NAME`: room material, one of `plain-white`, `matte-gray`, or `warm-white`. Default: `plain-white`.
- `--floor-only`: create only the floor, without walls.
- `--ceiling`: add a ceiling panel.
- `--light-density METERS`: target spacing between ceiling rect lights. Smaller values create more lights. Default: `1.8`.
- `--light-size NAME`: ceiling light panel shape, either `square` or `rectangle`. Default: `square`.
- `--partition`: add a 5m partition wall with a 1m x 2m door opening.

Official room generation example:

```bash
python scripts/deprecated/create_wall_room.py --length 30.0 --width 20.0 --height 3.0 --ceiling --partition
```

`scripts/deprecated/compose_scene_usd.py` composes the older tabletop task scene.
It is kept for reference and for inspecting the previous coffee bean setup, but
new robot-room task work should use the task-specific launcher and shared room
builder.

- `--output PATH`: USD file to write when `--save` is set. Default: `assets/tabletop_task_scene.usd`.
- `--save`: write the composed scene to `--output`.
- `--preview`: open the composed scene in Isaac Sim for visual checking.
- `--include-top-table`: add the top-center table. Do not combine this with `--with-robot`, because they occupy the same area.
- `--with-robot`: also reference the robot USD at `/World/Robot` for GUI validation.
- `--env PATH_OR_NONE`: optional environment USD. Use `none` or a USD path; relative paths resolve from the repository root. Default: `none`.
- `--randomize-cutlery-color`: apply random preview colors to cutlery assets.
- `--randomize-cutlery-placement`: randomize cutlery placement around the cutlery table.
- `--add-head`: add head payloads on the tables that have text labels.
- `--bean-count COUNT`: number of coffee bean rigid bodies to place in the bowl. Default: `150`.
- `--bean-color R G B`: coffee bean RGB color as three floats in `[0, 1]`. Default: `0.20 0.12 0.07`.
- `--bean-density VALUE`: coffee bean density for USD physics mass properties. Default: `850.0`.

Official scene composition example:

```bash
python scripts/deprecated/compose_scene_usd.py --env assets/plain_white_room_20_30_3_partition.usd --bean-count 300 --save
```

</details>

## Submodules

This repository uses Git submodules for external dependencies that should stay pinned to known commits:

```text
newton
third_party/franka_description
```

For fresh clones, use:

```bash
git clone --recurse-submodules <repository-url>
```

For existing clones, use:

```bash
git submodule update --init --recursive
```

## Asset and Path Handling

Workshop scripts use shared helpers from `scripts/common/path_utils.py` to resolve:
- repository root,
- `assets/` paths,
- `third_party/franka_description/urdfs/...` paths.

This removes the old assumption that runnable scripts must remain at the repository root.

## Physics and Control Notes

The mobile base follows a diagonal steer-drive layout. Shared helper logic in `scripts/common/tmr_base_control.py` provides:
- keyboard twist generation,
- wheel steering targets,
- wheel velocity targets,
- heading-hold compensation during translation.

This is still a simulation convenience layer, not a production-grade mobile
base controller. Physical-robot use still requires an external emergency stop
and watchdog.

### Simulation Performance

For scenes with many moving rigid bodies, such as hundreds of beans in a bowl,
enable PhysX Fabric in the Isaac Sim GUI:

1. Open `Window > Extensions`.
2. Search for `omni.physx.fabric`.
3. Enable the extension.
4. Open `Edit > Preferences > Physics > Fabric`.
5. Ensure Fabric is enabled.

Fabric improves performance by avoiding expensive per-frame USD transform
write-back for every moving rigid body. Without Fabric, PhysX updates are
written through USD transform attributes, USD notices, observer callbacks, and
Hydra render-transform synchronization. With Fabric, USD remains the authoring
format, but runtime body transforms are propagated through Fabric's simulation
data path to the renderer. This is much cheaper for dense dynamic scenes.

When Fabric is enabled, USD may not contain the latest live transforms(xform
transforms will be stale) duringsimulation. Use PhysX, Fabric-aware, or tensor
APIs for runtime state queriesinstead of reading moving body poses directly
from USD.

## Runtime Troubleshooting

If Isaac Sim reports permission errors for `/isaac-sim/kit/logs` or
`/isaac-sim/kit/data/Kit/.../user.config.json`, recreate the container after
updating the compose mounts and ensure the host cache directories are owned by
your container UID/GID:

```bash
python3 scripts/tools/validate_docker_runtimes.py --prepare-dirs --skip-script-check
sudo chown -R "${HOST_UID:-$(id -u)}:${HOST_GID:-$(id -g)}" \
  "$HOME/docker/ebim-challenge/isaac-sim-5.1.0" \
  "$HOME/docker/ebim-challenge/isaac-sim-6.0.0"
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 up -d --force-recreate isaac-sim-5-1-0
```

If ROS2 bridge startup fails with missing `libament_index_cpp.so`, launch with
`--ros2-bridge fastdds` or `--ros2-bridge cyclonedds` so the bundled ROS2
library path is configured before Isaac Sim starts. The launcher re-execs
itself once in ROS mode so `LD_LIBRARY_PATH` is visible to the dynamic loader
from process startup, and stores ROS logs under `/isaac-sim/kit/logs/ros`.

## Contributing

Linting and formatting run through [pre-commit](https://pre-commit.com/). Install the hooks once after cloning:

```bash
pip install pre-commit
pre-commit install
```

Run them across the repository before pushing:

```bash
pre-commit run --all-files
```

CI runs the same command on every pull request (`.github/workflows/pre-commit.yaml`), and `pre-commit` is the required status check on `main`, so a pull request cannot merge while it is red. The hooks cover Ruff (lint and format), codespell, license headers, and a set of file checks. Their configuration lives in `.pre-commit-config.yaml`, with Ruff's rules in `pyproject.toml`; several directories are excluded, listed under `exclude` at the end of `.pre-commit-config.yaml`.

`pyproject.toml` here holds tool configuration only — this repository is not a pip-installable package, so there is no `pip install -e .` step.

For runtime and environment setup — host requirements, Docker targets, and the simulation stack — see [docs/developer_setup.md](docs/developer_setup.md).

## Validation Checklist

After changes, verify the following:

1. `docker compose` resolves all configured profiles.
2. The repository appears inside each container at `/workspace/EBiM_Challenge`.
3. Isaac Sim GUI launches correctly through X11.
4. Each task-specific participant launcher starts and resolves its required USD assets.
5. `third_party/franka_description/urdfs/mobile_fr3_duo_v0_2_franka_hand.usd` is available.
6. No tools or docs still reference the removed `source/robot_lab` tree.

## Known Follow-Up Items

- Keep submodule URLs and pinned commits in `.gitmodules` up to date.
- Clean any generated URDF files in `third_party/franka_description/urdfs/` that still contain absolute paths from previous machines.
- Optionally add helper shell scripts for directory bootstrap of the Docker cache layout.

## References

- Isaac Sim 5.1.0 container documentation: <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html>
- Isaac Sim 6.0.0 container documentation: <https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_container.html>
- Isaac Lab Docker guide: <https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html>
