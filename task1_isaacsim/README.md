# Task 1 — Mobile FR3 Duo Teleoperation (Isaac Lab + Newton)

Task 1 teleoperates the **mobile Franka FR3 Duo** (dual arm + Robotiq 2F-85
grippers + wrist D405 cameras on a steer-drive mobile base) in Isaac Lab, using
the **Newton / MJWarp** physics backend, and includes an optional **deformable
cable** (Vertex Block Descent) board-plugging world.

The simulator runs **inside an Isaac Lab container**. Device publishers come
from the separate
[`EBiM-Benchmark/teleoperation`](https://github.com/EBiM-Benchmark/teleoperation)
checkout. The `task1_gello_pedal_teleop` container mounts that checkout and
runs the GELLO publisher and bridge; the pedal publisher is launched
interactively in the same container.

> **Teleop input:** keyboard is the recommended default for the mobile base.
> The **tested** configuration is GELLO leader arms + USB foot pedal (see
> [Teleoperation options](#teleoperation-options)).

---

## How it works

[Newton](https://github.com/newton-physics/newton) has already been integrated into
Isaac Lab as the physics engine backend. We use it to simulate the Franka FR3 Duo
Mobile robot, the cable, and the objects in the environment. Isaac Sim is mainly used
for visualization.

For robot control, we currently use the open-source
[Franka GELLO Duo](https://franka.de/gello) system to teleoperate the robot arms, while
the robot's mobile base is controlled using a foot pedal ([e.g. this 3-pedal USB foot
switch](https://www.amazon.de/Caswynlife-8A8V43M93ON473C79CT8/dp/B0H4GHBLTK/ref=sr_1_22_sspa?crid=37X5IAY4L6BCM&dib=eyJ2IjoiMSJ9.ZT9jubVbVBH0AQnZdPXibDmdQAr84dAIkFcrqwo3qD61otSj1mte7H9LfuoAEnjtbEWxZ10hY7kxSIBsMXwBpn4FPKufnAXJ8ZhQ_05VFpMtUCY7ojAB_C3babFMpFlcwHlBkKaOHCyfIwnQSxhnEkABOwbZtXvX3PjDfO3A7GyhgzopVuo0L2AJO4zMJyRs6QG3-hqbbHaG85_tDKg8qIn-ZbrdP5xdJrfh90p4XZaTR7fqzfQo9APuOhooyjY0hW1Y7dgR8NwNmdRpdPDZQpzA2d5ZL5FhSsx20QU_a6k.PqEqHKd38_ctTM289kJUqfsaeQvSznYQ8J_UrEiVGJg&dib_tag=se&keywords=electronic+3+pedals&qid=1783196050&sr=8-22-spons&psc=1)).

The scene is split into **two separate worlds**: one containing the robot, and another
containing the table, board, fixtures, and cable. In the robot world the robot is
simulated with the **MJWarpSolver**; in the cable world the cable is simulated with the
**VBDSolver**.

**Coupling approach:** the gripper pose from the robot world is continuously transferred
to the cable world. There, the pose is used to generate four box-shaped collision bodies
representing the four fingers of the two grippers. These four (red) boxes are used to
compute contact with the cable, and the cable state is then updated by the VBDSolver.

---

## Architecture

```
 teleoperation checkout
          │ mounted into container
          ▼
 task1_gello_pedal_teleop
 ├── gello_publisher ── /*/gello/* ──► gello_to_bridge.py ── /bridge/*
 └── pedal_state_publisher (interactive) ────────────────► /pedal/state
                                                           │
 task1_ros_republisher ◄────────────── /bridge/*            │
          │ /isaac/*                                       │
          ▼                                                ▼
 isaaclab_fr3duo_newton_bridge.py ◄─────────────────────────┘
          │
          ├── Newton/MJWarp robot simulation
          └── run_cable_vbd_ros_headless.py (optional cable VBD)

 keyboard_state_publisher ── /keyboard/state ──► teleop_adapters
                                                └── /pedal/state
```

- `/isaac/*` — joint states and command topics the bridge publishes/subscribes.
- `/bridge/*` — raw teleop commands; `ros_republisher` maps them to `/isaac/*`
  and applies gripper open/close calibration.
- `/pedal/state` — base motion tokens (`FWD`, `BACK`, `A`, `B`, `A+C`, `B+C`).
- Cable process exchanges gripper pose/gap and cable body centers with the bridge.

---

## Directory layout

```text
task1_isaacsim/
├── scripts/
│   ├── run_isaaclab_newton_teleop.sh      # launcher / orchestrator
│   ├── isaaclab_fr3duo_newton_bridge.py   # Newton/MJWarp ROS bridge (runs in Isaac Lab)
│   ├── run_cable_vbd_ros_headless.py       # headless cable VBD ROS process
│   ├── isaac_bridge_constants.py           # joint-name constants (browser UI dep)
│   ├── adapters/
│   │   ├── keyboard_to_base.py              # /keyboard/state → /pedal/state
│   │   └── gello_to_bridge.py               # /*/gello/joint_states → /bridge/*
│   └── controllers/
│       ├── ros_joint_republisher.py         # /bridge/* → /isaac/* + gripper calib
│       └── joint_position_controller.py     # holds joint targets each step
├── cable_world/                            # Newton VBD cable + board fixture assets
│   ├── configs/*.yaml
│   └── assets/…                            # table_board_fixture (large meshes via OneDrive)
├── assets/
│   ├── Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd   # robot USD (via OneDrive)
│   └── embodiments/fr3duo_mobile/*.yaml    # embodiment data contract / joint drives
├── services/
│   ├── teleop_adapters/start_teleop_adapters.sh
│   ├── gello_pedal_teleop/                # GELLO + pedal ROS 2 container
│   └── browser_controller/                 # optional no-hardware web UI (port 8090)
├── isaaclab_overlay/                       # ros2_jazzy Isaac Lab overlay (see its README)
├── docker-compose.yml                      # ROS 2 helper services
└── .env                                    # helper-service configuration
```

---

## Prerequisites

1. Linux host with a supported NVIDIA GPU + recent driver.
2. Docker Engine with Docker Compose v2 and the NVIDIA Container Toolkit.
3. `git`, plus `curl` and `unzip` (to fetch the large assets — see below).
4. X11 for the GUI window:
   ```bash
   xhost +local:docker
   export DISPLAY=${DISPLAY:-:0}
   ```
5. For GELLO / pedal only: the physical devices and `dialout` / `input` group
   permissions (see the `teleoperation` repo README).

> **Large assets are not in git.** The robot USD (~68 MB) and the two cable board
> meshes (~285 MB and ~1.5 GiB) are hosted on OneDrive and downloaded separately
> (see step 2). They are gitignored, so the repo clone stays small.

---

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

### 4. Set up the teleoperation device layer

Clone [`teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) next
to the benchmark repository. The GELLO + pedal container mounts this checkout
and builds the two required ROS packages on startup:

```bash
cd ..
git clone https://github.com/EBiM-Benchmark/teleoperation.git
cd benchmark
```

Set `TELEOPERATION_ROOT` in `task1_isaacsim/.env` when the checkout is elsewhere.

---

## Quick start

All commands run from the **repository root**. `--usd-path` is relative to `task1_isaacsim/`.

### Keyboard base + browser arms (no special hardware)

```bash
EMBODIMENT=fr3duo_mobile bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --usd-path assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd \
  --controller-mode position --with-keyboard-teleop \
  -- --spine-keyboard-control --spine-keyboard-step 0.02 \
     --spine-keyboard-min 0.0 --spine-keyboard-max 0.5
```

Then, in the teleoperation ROS 2 environment on the host, start the keyboard
publisher and drive the base with `w/a/s/d/q/e`:

```bash
ros2 run keyboard_state_publisher keyboard_state_publisher
```

Open the browser UI at <http://localhost:8090> to move the arms/grippers.

### Tested configuration (GELLO arms + foot pedal + cable)

This mirrors the command the pipeline was validated with:

```bash
EMBODIMENT=fr3duo_mobile bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --usd-path assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd \
  --controller-mode position --with-gello-pedal-teleop --with-cable --no-browser \
  -- --cable-world-position-offset 1.8 0.0 0.73 \
     --cable-robotiq-contact-x-offset 0.01 \
     --cable-robotiq-contact-y-offset -0.045 \
     --cable-robotiq-finger-size 0.025 0.006 0.04 \
     --cable-robotiq-contact-z-offset -0.02 \
     --spine-keyboard-step 0.02 --spine-keyboard-min 0.0 --spine-keyboard-max 0.5
```

The launcher creates `task1_gello_pedal_teleop` and starts GELLO automatically.
Start the pedal publisher interactively in the same container:

```bash
docker exec -it task1_gello_pedal_teleop bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /tmp/task1_teleop_install/setup.bash && \
   ros2 run pedal_state_publisher pedal_state_publisher'
```

---

## Teleoperation options

| Input | Publisher location | Sim-side glue | Drives |
| --- | --- | --- | --- |
| **Keyboard** (default) | `keyboard_state_publisher` → `/keyboard/state` | `keyboard_to_base.py` | Mobile base (`w/s` fwd/back, `a/d` strafe, `q/e` yaw) |
| **Foot pedal** (tested) | `task1_gello_pedal_teleop` → `/pedal/state` | none (direct) | Mobile base (strafe + yaw) |
| **GELLO** (tested) | `task1_gello_pedal_teleop` → `/*/gello/joint_states` | `gello_to_bridge.py` | Both arms + grippers |
| **Browser UI** | — | `browser_controller` (`/isaac/browser/*`) | Arms, grippers, base — no hardware |
| **Spine (height)** | — | in bridge (`--spine-keyboard-control`) | Vertical spine joint (Up/Down arrows) |

`--with-gello-pedal-teleop` starts the dedicated GELLO + pedal container.
`teleop_adapters` remains available for keyboard-to-base conversion and
defaults to the keyboard adapter only.

The keyboard publisher only emits while a key is held; when you release, the
bridge's `--pedal-timeout` stops the base automatically.

> **Run the pedal publisher in a foreground terminal.** It reads directly from
> terminal input, so launching it detached can freeze its input loop. GELLO and
> `gello_to_bridge.py` are managed by `task1_gello_pedal_teleop`.

**Foot pedal note:** after starting `pedal_state_publisher`, click once inside its
terminal window to give it keyboard focus, switch your keyboard input method to
English, then use the pedal to drive the mobile base. Pedal tokens: `A`/`B` strafe,
`A+C`/`B+C` yaw.

---

## Cable world (`--with-cable`)

`--with-cable` starts `run_cable_vbd_ros_headless.py` inside the Isaac Lab
container as a separate process. It simulates the deformable cable + board
fixture (`cable_world/`) and exchanges gripper pose/gap and cable body centers
with the bridge. Configs live in `cable_world/configs/`; the default
`table_board_fixture_cable.yaml` references board meshes under
`cable_world/assets/` (relative paths, no machine-specific paths). Tail its log:

```bash
docker exec isaac-lab-ros2_jazzy tail -f /tmp/task1_cable_vbd.log
```

The `-- --cable-*` arguments in the tested command position and size the cable /
Robotiq contact model; pass any bridge argument after the `--` separator.

---

## Launcher reference

`task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh [options]`:

| Option | Meaning |
| --- | --- |
| `--embodiment NAME` | Embodiment key under `task1_isaacsim/assets/embodiments` (default `fr3duo_mobile`). |
| `--usd-path PATH` | USD relative to `task1_isaacsim/` (or absolute). |
| `--controller-mode none\|position` | Start `position_controller` (default `position`). |
| `--with-keyboard-teleop` | Start the keyboard→base adapter. |
| `--with-gello-teleop` | Start the GELLO→bridge adapter (alias `--with-gello-pedal-teleop`). |
| `--no-browser` | Do not start `browser_controller`. |
| `--no-republisher` | Do not start `ros_republisher`. |
| `--with-cable` | Run the Newton cable VBD world. |
| `--headless` | No visible Kit window. |
| `--` | Pass the rest to `isaaclab_fr3duo_newton_bridge.py`. |

Environment overrides: `ISAACLAB_ROOT`, `ISAACLAB_CONTAINER`, `CONTAINER_REPO`,
`CABLE_DEVICE`, `CABLE_CONFIG_PATH`, `CABLE_GRIPPER_CONFIG_PATH`.

---

## Troubleshooting

- **"repository not mounted at /workspace/EBiM_Challenge"** — re-run
  `task1_isaacsim/isaaclab_overlay/apply_overlay.sh` and recreate the container.
- **`git apply` fails in the overlay** — your Isaac Lab checkout isn't at the
  pinned commit; `git -C ../IsaacLab checkout 0916ea3c0f…` and retry.
- **Cable world can't find board USD / missing meshes** — run `task1_isaacsim/scripts/download_large_assets.sh`.
- **Teleop topics not seen across host↔container** — match
  `RMW_IMPLEMENTATION` (use `rmw_fastrtps_cpp` everywhere) and `ROS_DOMAIN_ID`.
- **Robot falls / joints soft** — ensure `position_controller` is running
  (`--controller-mode position`) so joint targets are held every step.
- **No GUI window** — `xhost +local:docker`, verify `DISPLAY`, restart the
  container.
