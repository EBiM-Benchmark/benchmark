# Developer Setup And Runtime Guide

This guide is the working path for developers who need to set up the
workspace, construct Isaac Sim scenes, and use Isaac Lab for robot
manipulation or other advanced tasks.

## Host Requirements

- Linux workstation with an NVIDIA GPU.
- Docker Engine and Docker Compose v2 or newer.
- NVIDIA Container Toolkit configured for Docker.
- X11 session available on the host.
- NVIDIA NGC access for `nvcr.io/nvidia/isaac-sim` and
  `nvcr.io/nvidia/isaac-lab` images.

Check the local tools:

```bash
docker --version
docker compose version
nvidia-smi
```

Allow local Docker containers to use X11:

```bash
xhost +local:docker
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
touch "$XAUTHORITY"
```

The compose file uses variables from `docker/.env.base`. Always pass that
file with `--env-file`; `env_file:` alone only sets container environment
variables and does not expand `${...}` values in the compose model.

## Repository Layout

```text
benchmark/
|-- task1_isaacsim/             # Task 1: mobile FR3 Duo teleoperation (Isaac Lab + Newton)
|-- task1_mujoco/               # Task 1: cable-management teleoperation + eval (MuJoCo)
|-- task2_isaacsim/             # Task 2: thermal-pad teleoperation (Isaac Sim 5.1.0 / PhysX)
|-- assets/                     # USD assets and generated scenes
|-- docker/                     # Dockerfile, compose stack, runtime env
|-- docs/                       # Developer docs and images
|-- scripts/
|   |-- common/                 # Shared path and robot-control helpers
|   |-- manual_tests/           # Small asset validation scenes
|   |-- scenes/                 # Isaac Lab scene and robot demos
|   `-- tools/                  # USD composition and inspection utilities
`-- third_party/
    `-- franka_description/     # Robot description assets
```

All containers mount the repository at:

```text
/workspace/EBiM_Challenge
```

## Docker Runtime Targets

The project defines three local runtime images, each built from an NVIDIA base
image:

| Profile | Service | Local image tag | Base image |
| --- | --- | --- | --- |
| `isaac-sim-5.1.0` | `isaac-sim-5-1-0` | `isaac-sim-5.1.0:ebim2026` | `nvcr.io/nvidia/isaac-sim:5.1.0` |
| `isaac-sim-6.0.0` | `isaac-sim-6-0-0` | `isaac-sim-6.0.0-dev2:ebim2026` | `nvcr.io/nvidia/isaac-sim:6.0.0-dev2` |
| `isaac-lab-2.3.2` | `isaac-lab-2-3-2` | `isaac-lab-2.3.2:ebim2026` | `nvcr.io/nvidia/isaac-lab:2.3.2` |

Persistent cache and runtime data live under:

```text
${HOME}/docker/ebim-challenge
```

The Isaac Sim roots intentionally mirror NVIDIA's documented cache layout, but
with one version-specific suffix per image:

```text
~/docker/ebim-challenge/
|-- isaac-sim-5.1.0/
|   |-- cache/main/ov
|   |-- cache/main/warp
|   |-- cache/computecache
|   |-- config
|   |-- data/documents
|   |-- data/Kit
|   |-- logs
|   `-- pkg
`-- isaac-sim-6.0.0/
    |-- cache/main/ov
    |-- cache/main/warp
    |-- cache/computecache
    |-- config
    |-- data/documents
    |-- data/Kit
    |-- logs
    `-- pkg
```

Create the directories through the helper:

```bash
python3 scripts/tools/validate_docker_runtimes.py --prepare-dirs --skip-script-check
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

If GUI startup later reports cache write errors, fix ownership:

```bash
sudo chown -R "${HOST_UID:-$(id -u)}:${HOST_GID:-$(id -g)}" \
  "$HOME/docker/ebim-challenge/isaac-sim-5.1.0" \
  "$HOME/docker/ebim-challenge/isaac-sim-6.0.0"
sudo chmod -R g+rwX \
  "$HOME/docker/ebim-challenge/isaac-sim-5.1.0" \
  "$HOME/docker/ebim-challenge/isaac-sim-6.0.0"
```

## Build And Validate All Containers

The validation helper prepares host cache directories, builds all three local
runtime images in parallel, starts the containers, then validates:

- repository mount at `/workspace/EBiM_Challenge`,
- runtime cache/data mounts,
- X11 environment and mounted X11 files,
- host network mode,
- script syntax,
- USD path resolution and static scene composition.

Run:

```bash
python3 scripts/tools/validate_docker_runtimes.py \
  --prepare-dirs \
  --build \
  --up
```

Add `--external-network-check` if you also want each container to open an
outbound HTTPS connection to `nvcr.io`.

Stop the containers after validation:

```bash
python3 scripts/tools/validate_docker_runtimes.py --down --skip-script-check
```

Equivalent direct compose commands:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 \
  --profile isaac-sim-6.0.0 \
  --profile isaac-lab-2.3.2 \
  build isaac-sim-5-1-0 isaac-sim-6-0-0 isaac-lab-2-3-2

docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 \
  --profile isaac-sim-6.0.0 \
  --profile isaac-lab-2.3.2 \
  up -d isaac-sim-5-1-0 isaac-sim-6-0-0 isaac-lab-2-3-2
```

## Enter A Runtime

Isaac Sim 5.1.0:

```bash
docker exec -it isaac-sim-5-1-0-workshop bash
```

Isaac Sim 6.0.0-dev2:

```bash
docker exec -it isaac-sim-6-0-0-workshop bash
```

Isaac Lab 2.3.2:

```bash
docker exec -it isaac-lab-2-3-2-workshop bash
```

Inside any container:

```bash
cd /workspace/EBiM_Challenge
```

## Isaac Sim Robot Room Workflow

Use Isaac Sim with the prebuilt `assets/robot_room.usd` base scene. New workshop
task work should build on that room through
`scripts/scenes/scene_robot_room_keyboard.py`, not by generating new base
scenes.

The local runtime image maps `python` to the matching Isaac runtime. In Isaac
Sim containers it delegates to that image's `/isaac-sim/python.sh`; in the
Isaac Lab container it delegates to `/workspace/isaaclab/isaaclab.sh -p`.
The same wrapper also exposes the USD `pxr` libraries needed by plain USD
tools.

Inspect the active robot-room USD:

```bash
python scripts/tools/inspect_usd.py assets/robot_room.usd
```

Launch the active robot-room scene:

```bash
python scripts/scenes/scene_robot_room_keyboard.py --task task3
```

Open Isaac Sim GUI from inside an Isaac Sim container:

```bash
./runapp.sh
```

<details>
<summary>Deprecated scene generators and demos</summary>

The scripts below are kept for reference only. They do not define the current
competition base scene.

Generate the old tabletop task USD:

```bash
python scripts/deprecated/compose_scene_usd.py \
  --output assets/tabletop_task_scene.usd
```

Generate an old preview USD that also references the robot:

```bash
python scripts/deprecated/compose_scene_usd.py \
  --output assets/tabletop_task_scene_with_robot.usd \
  --with-robot
```

Create the old simple wall-room asset:

```bash
python scripts/deprecated/create_wall_room.py
```

## Isaac Lab Robot Manipulation And Advanced Tasks

Use the Isaac Lab container for scripts that create `InteractiveScene`,
`ArticulationCfg`, actuators, and robot control loops.

Run the old complete tabletop scene with keyboard-controlled mobile dual-arm robot:

```bash
python scripts/deprecated/scene_robot_keyboard.py
```

Run the old robot scene without keyboard control:

```bash
python scripts/deprecated/scene_robot_tables.py
```

Run the old reduced keyboard-control robot demo:

```bash
python scripts/deprecated/keyboard_control.py
```

Run the old static 11-table scene:

```bash
python scripts/deprecated/scene_11_tables.py
```

Keyboard controls in the robot demos:

| Key | Command |
| --- | --- |
| `W` / `S` | forward / backward |
| `A` / `D` | strafe left / right |
| `Q` / `E` | rotate left / right |
| arrow left / arrow right | rotate left / right |
| `Esc` or `Ctrl+C` | exit |

If `pynput` is missing in a runtime:

```bash
python -m pip install pynput
```

</details>

For advanced manipulation work, keep the active base scene in
`assets/robot_room.usd` and keep robot behavior in
`scripts/scenes/scene_robot_room_keyboard.py` or a new Isaac Lab task module.
That keeps USD asset composition stable while allowing Isaac Lab to own
articulations, actuators, observations, rewards, and control policies.

## Manual Asset Checks

Use the manual scenes when adjusting scale, rotation, or placement of a single
asset class:

```bash
python scripts/manual_tests/test_table_letter.py
python scripts/manual_tests/test_table_cutlery.py
```

## Newton Quick-Launch Examples

Standalone Newton scripts live in `scripts/newton_examples/`. They are useful
when a developer only needs to start a Newton simulation quickly.

Set up Newton once from the repository root:

```bash
uv venv --python 3.12
source .venv/bin/activate
uv pip install -e "./newton[examples]"
```

Run the examples:

```bash
source .venv/bin/activate
python scripts/newton_examples/example_mpm_moving_funnel.py --viewer gl
python scripts/newton_examples/example_mpm_table_container.py --viewer gl
python scripts/newton_examples/example_rigid_table_bowl_beans.py --viewer gl
```

The tabletop examples resolve USD assets from `assets/` by default. Use
`--assets-dir /path/to/assets` or set `EBIM_CHALLENGE_ASSETS_DIR` to point at a
different asset directory.

## Troubleshooting

Validate compose interpolation:

```bash
docker compose --env-file docker/.env.base \
  -f docker/docker-compose.yaml config --profiles
```

Check running containers:

```bash
docker compose --env-file docker/.env.base \
  -f docker/docker-compose.yaml ps
```

If X11 fails:

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
ls -la /tmp/.X11-unix
xhost +local:docker
```

If a container cannot write cache files, fix ownership of the relevant
directory under `${HOME}/docker/ebim-challenge`.

If scene scripts cannot find assets, confirm you are running from the mounted
repository:

```bash
pwd
ls assets/table_edit.usd
ls third_party/franka_description/urdfs/mobile_fr3_duo_v0_2_franka_hand.usd
```
