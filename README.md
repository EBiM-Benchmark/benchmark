# IROS Workshop Competition Repository

This repository provides a workshop-focused Isaac Sim environment for an international competition. It contains tabletop scene composition scripts, keyboard-controlled mobile dual-arm robot demos, USD utilities, Docker runtimes for Isaac Sim and Isaac Lab, and third-party robot description assets.

For the full developer workflow, see [`docs/developer_setup.md`](docs/developer_setup.md).

## Repository Layout

```text
IROS_Workshop/
├── assets/                      # USD assets and generated scene files
│   └── Tabletop_DEMO.usd        # Scene with Commandable via ROS mobile_Fr3_duo
├── DEMO/                        # Lula solver widget and robotdescription of DEMO robot
│   └── dual_arm_rmp_widget      # Lula solver widget intergrated with issac sim
│   └── robot_description        # urdf assets and armconfiguration that Lula needs
│   └── Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd
                                 # pure usd assets intergrted with ROS communication and Robotiq grippers
│   └── record.py                # lerobot dataset tool
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

### Keyboard Teleoperation Integration Tabletop_DEMO
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


## Running Scripts

Inside an Isaac Sim runtime, construct and inspect reusable USD scenes with:

```bash
python scripts/tools/compose_scene_usd.py --output assets/tabletop_task_scene.usd
python scripts/tools/compose_scene_usd.py --output assets/tabletop_task_scene_with_robot.usd --with-robot
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

## LeRobot dataset recording
The `DEMO/record.py` script automatically subscribes to the corresponding ROS 2 topics, synchronizes the multi-modal streams, and aggregates them into the structured dataset:

* **State Data (States):**
  * **Manipulators:** 14 joint positions and velocities across both arms (14 joints total).
  * **End-Effectors:** Gripper poses for both left and right arms.
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
   cd DEMO
3. Execute the recording script:
      ```bash
   python record.py
   Recording Control via Terminal:
     * Enter 2: Start recording the dataset.
     * Enter 3: Stop recording and automatically save the episode.

## References

- Isaac Sim 5.1.0 container documentation: <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html>
- Isaac Sim 6.0.0 container documentation: <https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_container.html>
- Isaac Lab Docker guide: <https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html>
