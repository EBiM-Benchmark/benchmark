# IROS Workshop Competition Repository

This repository provides a workshop-focused Isaac Sim environment for an international competition. It contains tabletop scene composition scripts, keyboard-controlled mobile dual-arm robot demos, USD utilities, Docker runtimes for Isaac Sim and Isaac Lab, and third-party robot description assets.

For the full developer workflow, see [`docs/developer_setup.md`](docs/developer_setup.md).

## Repository Layout

```text
IROS_Workshop/
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

## Supported Container Targets

The Docker stack is parameterized in `docker/.env.base` and `docker/docker-compose.yaml`.

### Isaac Sim 5.1.0
- Image: `nvcr.io/nvidia/isaac-sim:5.1.0`
- Local tag: `isaac-sim-5.1.0:iros2026-ebim`
- Compose profile: `isaac-sim-5.1.0`
- Intended for GUI and simulation workflows with X11 support.

### Isaac Sim 6.0.0-dev2
- Image: `nvcr.io/nvidia/isaac-sim:6.0.0-dev2`
- Local tag: `isaac-sim-6.0.0-dev2:iros2026-ebim`
- Compose profile: `isaac-sim-6.0.0`
- Uses the currently documented pre-GA container tag.

### Isaac Lab 2.3.2
- Image: `nvcr.io/nvidia/isaac-lab:2.3.2`
- Local tag: `isaac-lab-2.3.2:iros2026-ebim`
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
${HOME}/docker/iros-workshop
```

Create the required directories before the first launch. A typical layout is:

```text
~/docker/iros-workshop/
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
  "$HOME/docker/iros-workshop/isaac-sim-5.1.0" \
  "$HOME/docker/iros-workshop/isaac-sim-6.0.0"
sudo chmod -R g+rwX \
  "$HOME/docker/iros-workshop/isaac-sim-5.1.0" \
  "$HOME/docker/iros-workshop/isaac-sim-6.0.0"
```

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
/workspace/IROS_Workshop
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
- `scripts/scenes/scene_robot_keyboard.py` — complete tabletop scene with keyboard control.
- `scripts/scenes/scene_robot_tables.py` — complete tabletop scene with robot but without keyboard control.
- `scripts/scenes/scene_11_tables.py` — 11-table composition utility and preview.
- `scripts/scenes/scene_with_table.py` — simple single-table placement example.
- `scripts/scenes/keyboard_control.py` — reduced robot keyboard-control demo.

### Utilities
- `scripts/tools/compose_scene_usd.py` — compose the tabletop task scene directly as USD.
- `scripts/tools/create_wall_room.py` — generate a simple wall-room USD asset.
- `scripts/tools/inspect_usd.py` — print the prim hierarchy of a USD file.

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

Inside an Isaac Sim runtime, construct reusable USD scenes with these tools.

`scripts/tools/create_wall_room.py` creates a room USD asset.

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

`scripts/tools/compose_scene_usd.py` composes the tabletop task scene.

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

Official room generation example:

```bash
python scripts/tools/create_wall_room.py --length 30.0 --width 20.0 --height 3.0 --ceiling --partition
```

Official scene composition example:

```bash
python scripts/tools/compose_scene_usd.py --env assets/plain_white_room_20_30_3_partition.usd --bean-count 300 --save
```

Inspect the composed USD hierarchy:

```bash
python scripts/tools/inspect_usd.py assets/tabletop_task_scene_with_robot.usd
```

Inside an Isaac Lab runtime, run robot manipulation and control scenes with:

```bash
python scripts/scenes/scene_robot_keyboard.py
python scripts/scenes/scene_robot_tables.py
python scripts/manual_tests/test_table_cutlery.py
```

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

This is still a simulation convenience layer, not a production-grade mobile base controller.

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

## Validation Checklist

After changes, verify the following:

1. `docker compose` resolves all configured profiles.
2. The repository appears inside each container at `/workspace/IROS_Workshop`.
3. Isaac Sim GUI launches correctly through X11.
4. `scripts/scenes/scene_robot_keyboard.py` starts and resolves all required USD assets.
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
