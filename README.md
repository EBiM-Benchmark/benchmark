# EBiM Challenge Repository

**Current capability status: see [STATUS.md](STATUS.md)** — this is a developer preview; check what's usable before you build.

## Competition tasks

| Task | Engines | Where in this repo | Status |
|---|---|---|---|
| Task 1 — Cable Routing & Plugging | Isaac Sim, MuJoCo | [`task1_isaacsim/`](task1_isaacsim/), [`task1_mujoco/`](task1_mujoco/) | see [STATUS.md](STATUS.md) |
| Task 2 — Deformable Material Handling (Thermal Pad Placement) | Isaac Sim (Genesis committed) | [`assets/task2_objects/`](assets/task2_objects/), [`scripts/evaluation/task2/`](scripts/evaluation/task2/) | see [STATUS.md](STATUS.md) |
| Task 3 — Assisted Living & Feeding | Isaac Sim (MuJoCo committed) | Isaac Sim: [`scripts/scenes/scene_robot_room_keyboard.py`](scripts/scenes/scene_robot_room_keyboard.py), [`assets/robot_room.usd`](assets/robot_room.usd); MuJoCo: in development — see [STATUS.md](STATUS.md) | see [STATUS.md](STATUS.md) |

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
unified ROS 2 teleop), a single in-sim IK shared by all of them, and the
official ManipulationNet client vendored for end-to-end scored evaluation.
The directory is self-contained — native one-click launchers plus a Docker
evaluation stack:

```bash
cd task1_mujoco
./start.sh              # native teleoperation (Windows: double-click start.bat)
./eval.sh sim           # scored ManipulationNet evaluation (Docker), terminal 1
./eval.sh client        # terminal 2: official mnet client
```

See [`task1_mujoco/README.md`](task1_mujoco/README.md) for the full participant
guide (paths, input modes, controls, troubleshooting).

## Repository Layout

```text
benchmark/
├── task1_isaacsim/              # Task 1: mobile FR3 Duo teleoperation (Isaac Lab + Newton)
├── task1_mujoco/                # Task 1: cable-management teleoperation + eval (MuJoCo)
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

The robot-room launch script loads `assets/robot_room.usd` by default and
places the mobile FR3 from a task preset. Presets are `task1` at
`(4.4, -2.5, 0.0)`, `task2` at `(4.4, 2.6, 0.0)`, and `task3` at
`(-4.6, 2.7, 0.0)`. The preset robot yaw is `90` degrees for `task1` and
`-90` degrees for `task2`/`task3`. For `task3`, the launcher also adds 300
coffee beans to the bowl at `(-4.3, -1.467124258834126, 0.7542400587544876)`
and sets the built-in
Perspective viewport to
`(-8.12589, -3.29067, 2.79653, 73.13762, 0.0, -50.88313)`.

#### Task3 Feeding Workflow

Task3 is a four-stage feeding task:

1. Table Setup: move the plate holding the bowl, cup, disk, and spoon from the
   Kitchen Area to the Dining Area.
2. Feed: scoop coffee beans from the bowl with the spoon, move the spoon to a
   feeding position in front of the robot head, and hold that feeding position
   for at least 3 seconds. After feeding is complete, return the coffee beans to
   the bowl.
3. Bean Recovery: transfer the coffee beans from the bowl into the designated
   recycling container in the Kitchen Area.
4. Clean Up: return all utensils to the Kitchen Area and place them inside the
   designated sink region, marked by the black tap area.

Launch the keyboard-controlled Task3 Lula-IK demo from the host. Start its
Isaac Lab container first:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-lab-2.3.2 up -d isaac-lab-2-3-2
```

Then run the launcher:

```bash
docker exec -it isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python scripts/scenes/scene_robot_room_keyboard.py --task task3'
```

The Isaac Lab and plain Isaac Sim launchers share this direct keyboard map:

- Left arm: `W/S`, `A/D`, `Q/E` translate X/Y/Z; `Z/X`, `T/G`, `C/V` rotate
  roll/pitch/yaw; `F` toggles the gripper.
- Right arm: `O/L`, `K/;`, `I/P` translate X/Y/Z; `N/M`, `U/J`, `,/.` rotate
  roll/pitch/yaw; `'` toggles the gripper.
- Hold `Shift` for the mobile base: `H/N` forward/back, `B/M` left/right, and
  `G/J` rotate counter-clockwise/clockwise. These overlapping arm rotation
  keys are suppressed while the modifier is held.
- `R` resets both arm targets. `Esc` or `Ctrl+C` exits.

Motion keys are continuous and scaled by simulation time. Cartesian targets
are stored in the moving robot-base frame, so moving or rotating the mobile
base carries both end-effector targets with it. The mobile-base defaults are
set to `0.25 m/s` translation and `0.75 rad/s` yaw.

`task3` enables keyboard control by default. To load the scene as a passive
Isaac Sim viewer instead, pass `--no-keyboard-control`; in that mode, click
Play in the Isaac Sim GUI or add `--autoplay` to start the timeline.
For real-time keyboard control, task3 coffee beans are static by default; pass
`--dynamic-beans` to make them rigid bodies. Use `--stabilization-steps N` if
you want to run warmup physics steps before enabling the keyboard loop.

Isaac Sim remains the simulator and supplies the Lula motion-generation
solvers. Isaac Lab owns scene stepping, measured joint state, and articulation
actuator targets. A small raw-Lula bridge copies the first environment's
current joint positions from the Isaac Lab CUDA tensor to a CPU NumPy warm
start, slices each arm in exact cspace order, and maps solutions back to global
joint indices. This avoids passing either CUDA tensors or an Isaac Lab
`Articulation` through `ArticulationKinematicsSolver`. Dual-arm solver
construction and portable configuration are adapted from
[`EBiM-Benchmark/benchmark-archive`](https://github.com/EBiM-Benchmark/benchmark-archive),
branch `Robotiq_DEMO` (commit `78c28ea`). The first version deliberately
supports exactly one environment and keyboard input only.

Each arm is solved independently. If one arm has no IK solution, its last
valid joint target is retained while the other arm, grippers, spine, and base
continue to operate; warnings are rate-limited. Persistent targets are seeded
once from measured joint positions after stabilization, so the control loop
does not restore the default arm pose every frame.

Future ROS 2, GELLO, gripper, and foot-pedal integration belongs upstream of
the simulator-independent `TeleopCommand` boundary. An adapter for the
separate [`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation)
repository can translate `/keyboard/state`, namespaced GELLO joint states,
gripper values, and pedal state into that boundary without replacing target
tracking, IK, joint composition, or the Isaac Sim/Isaac Lab runtime adapter.
`TeleopCommand` already carries optional canonical seven-value left/right arm
joint tuples for a direct GELLO-style source. A fresh absolute joint tuple has
per-arm priority over Cartesian IK at the selective composer boundary, so one
arm may use direct joints while the other uses IK. Every incoming direct tuple
is clamped to that arm's configured articulation soft joint limits before it
can be latched. Once a direct source owns an arm, stale or inactive input holds
the last bounded direct target rather than silently falling back to an old
Cartesian target. Future source arbitration must explicitly release that
arm's direct ownership and reseed its Cartesian tracker before IK takeover;
arbitration otherwise remains upstream of the winning command.

For an isolated **plain Isaac Sim 5.1** dual-arm RMPflow demo (no Isaac Lab),
start the plain Isaac Sim container:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 up -d isaac-sim-5-1-0
```

Then run the separate entry point:

```bash
docker exec -it isaac-sim-5-1-0-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && /isaac-sim/python.sh scripts/scenes/scene_robot_room_rmpflow.py'
```

This executable builds the same Task 3 robot room and Franka-hand robot USD,
then uses two independent RMPflow policies with Kit-window keyboard control.
Run it separately from `scene_robot_room_keyboard.py`; the two runtimes do not
share an articulation or controller. `--headless` loads and steps the plain
Isaac Sim scene but intentionally disables arm keyboard teleoperation. This
isolated demo has no ROS/GELLO dependency. Hold `Shift` for mobile-base
control: `H/N` forward/backward, `B/M` left/right, and `G/J`
counter-clockwise/clockwise. The modifier prevents these keys from also
commanding the overlapping arm rotations. Its default base speeds are `0.25
m/s` and `0.75 rad/s`; override them with `--base-linear-speed` and
`--base-angular-speed` only after confirming the robot remains stable.

#### Task3 Grading Unit Tests

The Task3 grading helpers live in `scripts/evaluation/task3/grading.py`, with
direct unit tests in `scripts/evaluation/task3/tests/test_grading.py`. These
tests validate the grading math for each stage; they do not launch the full
Isaac Sim scene or run robot physics.

Start the Isaac Lab container if it is not already running:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-lab-2.3.2 up -d isaac-lab-2-3-2
```

Run all Task3 grading unit tests inside the container:

```bash
docker exec isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python -B scripts/evaluation/task3/tests/test_grading.py'
```

Run a single stage by passing `stage1`, `stage2`, `stage3`, or `stage4`:

```bash
docker exec isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python -B scripts/evaluation/task3/tests/test_grading.py stage1'

docker exec isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python -B scripts/evaluation/task3/tests/test_grading.py stage2'

docker exec isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python -B scripts/evaluation/task3/tests/test_grading.py stage3'

docker exec isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python -B scripts/evaluation/task3/tests/test_grading.py stage4'
```

#### Task3 Grading Integration Tests

The integration runner launches a real Isaac Sim app, builds the Task3 robot
room, drives live scene prims through stage-specific validation motions, lets
the app update, and then grades the live stage. Use this when you want to
confirm that the grading mechanism works against the actual USD scene.

Run all four integration tests with the GUI enabled:

```bash
docker exec -it isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && exec python -B scripts/evaluation/task3/integration_test.py all'
```

GUI integration runs print the grade and then keep Isaac Sim open for
inspection. Close the Isaac Sim window or press Ctrl+C in the terminal when you
are done. Run the same tests without opening a GUI window when you want the
runner to exit automatically:

```bash
docker exec isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python -B scripts/evaluation/task3/integration_test.py --headless all'
```

Run a single GUI integration stage by passing `stage1`, `stage2`, `stage3`, or
`stage4`:

```bash
docker exec -it isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && exec python -B scripts/evaluation/task3/integration_test.py stage1'
```

Add `--headless` to the single-stage command if you want it to shut down
automatically after printing the result.

Each integration stage prints one `STAGE_RESULT` JSON line with `score`,
`max_score`, `passed`, and stage-specific measurements. For example, Stage 1
prints utensil and bean dining-area counts, Stage 2 prints bean retention and
hold-time details, Stage 3 prints the percentage of beans inside the recovery
sphere, and Stage 4 prints sink-return counts. In the deterministic headless
validation run, the stage grades were:

```text
stage1: 5/5
stage2: 4/4
stage3: 4/4
stage4: 5/5
```

Stage 1 disables rigid bodies on the utensil set, keeps coffee beans rigid-body
enabled but kinematic, then gradually translates the tray, bowl, spoon, plate,
cup, and existing beans together into the Dining Area without rotating them,
moving along y before x to avoid the wall. Stage 2 makes the spoon a kinematic
rigid body, rotates it +90
degrees around z to point toward the mouth, keeps the coffee beans as dynamic
rigid bodies, moves the spoon to a pose 20 cm in front of the fake head, places
five existing beans 1 cm above the spoon, moves the spoon forward 10 cm for
three seconds, retracts 10 cm, and counts beans still on the spoon. Stage 3
keeps beans dynamic and respawns them with the same bowl-style pattern used by
`scene_robot_room_keyboard` above the recovery container. Stage 4 removes the
coffee-bean scope before moving utensils to the sink boundary.

To use a different room USD, pass `--room-usd`, for example:

```bash
docker exec -it isaac-sim-5-1-0-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python scripts/scenes/scene_robot_room_keyboard.py --room-usd assets/robot_room.usd'
```

You can still override the preset with `--robot-x`, `--robot-y`, and
`--robot-z` when manually checking a placement.

The first launch can spend time compiling Isaac/RTX shader caches before the
scene becomes interactive.

The `--head-placement` option is only for the task3 head prop. Use `a` through
`i` to select a fixed placement, or `random` to choose one automatically.
Lowercase and uppercase letters are both accepted:

```bash
docker exec -it isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python scripts/scenes/scene_robot_room_keyboard.py --task task3 --head-placement e'
```

For randomized task3 head placement:

```bash
docker exec -it isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python scripts/scenes/scene_robot_room_keyboard.py --task task3 --head-placement random'
```

This option is unrelated to the keyboard `E` key used for rotation in the older
keyboard-control demos.

In passive viewer mode, the script launches native Isaac Sim and forwards the
task preset into Kit, so switching `--task task1`, `--task task2`, or
`--task task3` changes the spawn position without editing the command
coordinates.

For ROS2 bridge development, enable the bridge explicitly. The launcher sets
`ROS_DISTRO`, `RMW_IMPLEMENTATION`, and the bundled bridge library path before
Isaac Sim starts:

```bash
docker exec -it isaac-sim-5-1-0-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && python scripts/scenes/scene_robot_room_keyboard.py --task task3 --no-keyboard-control --ros2-bridge fastdds'
```

Use `--ros2-bridge cyclonedds` if you want CycloneDDS instead of FastDDS. The
default `--experience base` avoids loading optional extensions such as the ROS2
bridge unless requested; use `--experience full` when you need the full Isaac
Sim extension set.

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
- `scripts/scenes/scene_robot_room_keyboard.py` — robot room scene launcher with task-based mobile FR3 spawn presets. `task3` includes coffee beans in the bowl and runs Isaac Lab keyboard base control by default; pass `--no-keyboard-control` for passive Isaac Sim viewing.

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
- `scripts/deprecated/compose_scene_usd.py` — deprecated tabletop scene composer kept for reference. The active task3 bean setup now lives in `scripts/scenes/scene_robot_room_keyboard.py`.

</details>

### Manual Validation Scenes
- `scripts/manual_tests/test_table_cutlery.py` — validate table plus cutlery placement.
- `scripts/manual_tests/test_table_letter.py` — validate table plus letter placement.

### Manipulation tabletop_task_scene_DEMO

#### 1.1 Left Sideways Translation Test

The robot moves directly to the left without changing its heading.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

```

#### 1.2 Right Sideways Translation Test

The robot moves directly to the right without changing its heading.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

```

#### 1.3 Diagonal 45° Movement Test

Tests the smoothness of x and y-axis velocity composition. The robot should translate linearly along the diagonal.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

```

#### 1.4 "Drift" Movement Test (Translation + Rotation)

The robot moves forward while translating sideways and rotating simultaneously. This tests the composite command logic.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"

```

#### 2.1 Waist Vertical Control

Raises the waist vertical joint to the 0.5m position.

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: ['franka_spine_vertical_joint'], position: [0.5]}"

```

#### 2.2 Arm Joint Control

Commands the left arm's joints (joint1 and joint2) to move to the specified positions.

```bash
ros2 topic pub --once /joint_command sensor_msgs/msg/JointState "{name: ['left_fr3v2_joint1', 'left_fr3v2_joint2'], position: [0.5, -0.5]}"

```

## Running Scripts

Inside an Isaac Sim runtime, use the prebuilt `assets/robot_room.usd` base scene
through `scripts/scenes/scene_robot_room_keyboard.py`. New workshop task work
should build on that room instead of generating new base scenes.

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
new robot-room task work should use `scripts/scenes/scene_robot_room_keyboard.py`.

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
base controller. The active Task3 robot-room launcher uses these helpers at
runtime alongside selective dual-arm, gripper, and spine position targets.
Physical-robot use still requires an external emergency stop and watchdog.

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

## Validation Checklist

After changes, verify the following:

1. `docker compose` resolves all configured profiles.
2. The repository appears inside each container at `/workspace/EBiM_Challenge`.
3. Isaac Sim GUI launches correctly through X11.
4. `scripts/scenes/scene_robot_room_keyboard.py --task task3` starts and resolves all required USD assets.
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
