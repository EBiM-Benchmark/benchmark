# EBiM Benchmark Task 1 — Cable Management Simulation (MuJoCo)

> **Eval mode is temporarily on hold.** This README currently documents
> practice mode only (`--input keyboard|gamepad|vr|gello|ros_teleop`, no
> `--mnet`); scored evaluation (`--mnet`, `local_test`/`submission`) is
> paused pending an internal decision after a meeting with the
> ManipulationNet team. The original instructions are kept in the source
> as HTML comments (search for `EVAL SECTION HIDDEN`) and will come back
> once that's resolved.

Task 1 teleoperates a mobile dual-arm Franka FR3 platform (Robotiq 2F-85
grippers on a planar-drive base with a vertical spine) to route a deformable
cable across a fixture board — the ManipulationNet **cable_management**
benchmark (Tier 2). The simulator supports **keyboard, gamepad, VR, GELLO,
and a unified ROS 2 teleop mode**.

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by unwrapping.
It also integrates the official ManipulationNet ROS 2 client for
end-to-end scored evaluation: fixed overhead evidence camera, automatic
tier skipping, client-driven fixture randomization, and an in-scene
one-time-code display.
-->

## How it works

Everything runs in a **single MuJoCo process** (`main.py`): robot, cable,
board and physics live in one simulation — no separate worlds, no coupling
layers. Every input device feeds the same control stack (grasp-aware
scaling → smoothing → contact clamp → damped-least-squares IK → force-servo
grasping), so the robot feels identical no matter which device or which
mode below you use.

This README currently covers one choice — **which device drives the
robot**: `--input keyboard|gamepad|vr|gello|ros_teleop`. `gello` and
`ros_teleop` need ROS 2 even without scoring (they *are* ROS topics); the
rest need no ROS at all in practice mode.

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by unwrapping.
Two independent choices determine how you run it:

- **Which device drives the robot** — `--input keyboard|gamepad|vr|gello|ros_teleop`.
  Works the same in every mode below.
- **Whether you're scoring** — add `--mnet` (or don't). Without it you get
  a plain simulator: no ROS required unless the input device itself needs
  it (`gello` and `ros_teleop` always need ROS — they *are* ROS topics).
  With it, a ManipulationNet client (a separate ROS 2 process) drives the
  session: evidence camera, tier skipping, fixture randomization, one-time
  code, scored video. `local_test` and `submission` are just two modes of
  that same client — `local_test` runs the full scored flow for practice
  (unlimited, nothing is submitted); `submission` is the real thing
  (rate-limited). Getting to `submission` needs nothing beyond what
  `local_test` needs, plus team registration.
-->

## Capability matrix

| | Windows | Ubuntu |
|---|---|---|
| Practice | ✅ | ✅ |

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by unwrapping.
| Eval test (`local_test`) | ✅ | ✅ |
| Submission | ⚠️ | ⚠️ |
-->

✅ verified. *How* practice runs (Docker or not) is a setup detail — see
the OS sections below for exact commands and per-input-device detail.

## Choose your path

```
your OS
├── Windows ── practice (no ROS) ── all via start.bat, no Docker required
└── Ubuntu   ── practice (no ROS) ── all via start.sh, no Docker required

    every branch ends the same way:
    --input keyboard | gamepad | vr | gello | ros_teleop
    (gello/ros_teleop need ROS even in "practice")
```

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by swapping the
tree above back to this:
your OS
├── Windows ─────────────────────────────────────┐  all via ros_native.bat,
│   ├── practice   (no ROS)                       │  no Docker required —
│   ├── eval test  (--mnet, local_test)            │  see the Windows section
│   └── submission (--mnet, submission)            │
│                                                  ┘
└── Ubuntu ───────────────────────────────────────┐
    ├── practice   (no ROS)                       │  practice: no Docker
    ├── eval test  (--mnet, local_test)            │  eval/submission: Docker
    └── submission (--mnet, submission)            │  (GPU host) or the
                                                    │  Docker-free appendix
                                                    ┘
-->


Jump to: [Windows](#windows) · [Ubuntu](#ubuntu) ·
[Controls](#controls) · [Input devices in depth](#input-devices-in-depth) ·
[Troubleshooting](#troubleshooting)

## Get the code

```bash
git clone https://github.com/2houyuhang/EBiM_Benchmark_task1.git
cd EBiM_Benchmark_task1
```

That's it — no large-asset downloads, no manual dependency install. Every
launcher below bootstraps itself on first run (conda env / Docker image /
ROS workspace, depending on which path you picked above). Now jump to your
OS: [Windows](#windows) · [Ubuntu](#ubuntu).

## Windows

| Input device | Practice |
|---|---|
| Keyboard | ✅ |
| Gamepad | ✅ |
| VR | ✅ |
| GELLO¹ | ⚠️ |
| ros_teleop¹ | ✅ real keyboard + gamepad |

✅ verified · ⚠️ not yet verified. ¹ needs ROS 2 even in practice.

**Setup once**: Miniconda on `PATH` (installer sets this) covers practice —
nothing else to install.

**Practice** (no ROS):

```bat
start.bat                      :: keyboard (default)
start.bat --input gamepad
start.bat --input vr
```

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by unwrapping.
Original table (three more columns) and eval/submission instructions:

| Input device | Practice | Eval test (`local_test`) | Submission |
|---|---|---|---|
| Keyboard | ✅ | ✅ | ⚠️ |
| Gamepad | ✅ | ✅ | ⚠️ |
| VR | ✅ | ✅ | ⚠️ |
| GELLO¹ | ⚠️ | ⚠️ in progress | ⚠️ |
| ros_teleop¹ | ✅ real keyboard + gamepad | ✅ | ⚠️ |

✅ verified · ⚠️ not yet verified. ¹ needs ROS 2 even in practice (every
other row needs it only for eval test/submission) — see the
[Windows quick start](#quick-start--windows-robostack-eval-step-by-step). Eval
test detail (scores, video, camera fps) is in that same section.

**Eval/submission have no Docker option on Windows** — Docker Desktop's
backend is WSL2, whose software rendering can't reach the evidence
camera's 25 fps minimum, so the RoboStack setup below isn't an alternative
to a Docker path, it's the only path here. One command sets it all up:

```bat
setup_eval.bat
```

(what it does, and how to redo the steps by hand, is in the
[Windows quick start](#quick-start--windows-robostack-eval-step-by-step)). After
that, every command below just works.

**Eval test** (`local_test` — unlimited, nothing is submitted):

```bat
ros_native.bat python robotiq_duo_full_scene_minimal_core\main.py --input keyboard --mnet
:: or --input gamepad / vr / gello
```

second terminal:

```bat
ros_native.bat ros2 run mnet_client local_test
```

Full walkthrough (tiers, one-time code, collecting results): see
[Windows quick start](#quick-start--windows-robostack-eval-step-by-step).

**Submission** (real attempt, rate-limited — scored by the official
[ManipulationNet](https://manipulation-net.org) service, not anything this
repo hosts): register your team, put the `team_unique_code` into
`team_config.json`, then run the exact same two commands as above but swap
the client's mode:

```bat
ros_native.bat ros2 run mnet_client connection_test    :: sanity-check credentials first — free, unlimited
ros_native.bat ros2 run mnet_client submission           :: instead of local_test
```

See [registration + connection_test](#registration--connection_test-before-a-real-submission) below.
-->

## Ubuntu

| Input device | Practice |
|---|---|
| Keyboard | ✅ |
| Gamepad | ✅ |
| VR | ⚠️ |
| GELLO¹ | ⚠️ in progress |
| ros_teleop¹ | ⚠️ |

✅ verified · ⚠️ not yet verified. ¹ needs ROS 2 even in practice.

**Setup once**: nothing for practice (`start.sh` bootstraps its own conda
env on first run).

**Practice** (no ROS):

```bash
./start.sh                     # keyboard (default)
./start.sh --input gamepad
```

<!-- COMMAND HIDDEN (VR on Ubuntu not yet verified - see the capability
matrix. Restore once confirmed working: ./start.sh --input vr -->

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by unwrapping.
Original table (three more columns) and eval/submission instructions:

| Input device | Practice | Eval test (`local_test`) | Submission |
|---|---|---|---|
| Keyboard | ✅ | ✅ | ⚠️ |
| Gamepad | ✅ | ✅ | ⚠️ |
| VR | ⚠️ | ⚠️² | ⚠️ |
| GELLO¹ | ⚠️ in progress | ⚠️ in progress | ⚠️ |
| ros_teleop¹ | ⚠️ | ⚠️ | ⚠️ |

✅ verified · ⚠️ not yet verified. ¹ needs ROS 2 even in practice (every
other row needs it only for eval test/submission) · ² VR can't enter a
container — use the native path below instead of `eval.sh`.

Eval/submission need Docker Engine + Compose v2 and,
for the evidence camera's 25 fps minimum, **native Linux with a real GPU**
(nvidia-container-toolkit) — nothing else to set up, `eval.sh` builds its
own image on first run. No Docker at all?

```bash
./setup_eval.sh
```

sets up the fully native ROS 2 path in one shot (what it does, and how to
redo the steps by hand, is in the
[Docker-free appendix](#appendix--ubuntu-docker-free-eval)).

**Eval test** (`local_test` — unlimited, nothing is submitted). Docker
bakes the input device into which script you run (no `--input` flag here):

```bash
xhost +local:docker            # once per login session
./eval.sh sim                  # keyboard
# or: ./eval.sh gamepad
```

second terminal:

```bash
./eval.sh client
```

Full walkthrough: see [Quick start — Docker eval (Ubuntu), step by step](#quick-start--docker-eval-ubuntu-step-by-step).
VR and GELLO can't go through `eval.sh` (VR needs direct host device
access; there's no dedicated compose service for GELLO yet) — run the sim
half natively instead, client still via `./eval.sh client`:

```bash
python main.py --input vr --mnet      # or gello — see the Docker-free appendix for the native ROS setup
```

**Submission** (real attempt, rate-limited — scored by the official
[ManipulationNet](https://manipulation-net.org) service, not anything this
repo hosts): register your team, put the `team_unique_code` into
`mnet_client-ros_2/config/team_config.json` (already mounted into the
container — no rebuild needed), then pick `connection_test` first, then
`submission`, from the same client menu `local_test` above came from.
-->

## Controls

**Keyboard** (motion uses the arrow-key cluster; letters stay free):
`7/8/9` select base / left arm / right arm ·
base mode: arrows drive in **screen directions**, `Home`/`End` turn,
`PageUp`/`PageDown` move the spine ·
arm modes: arrows translate, `PageUp`/`PageDown` for Z, `R` toggles rotation
(arrows = yaw/pitch, `PageUp`/`PageDown` = roll) ·
`G` close gripper (force servo) · `V`/`Space` open · `-`/`=` speed ·
`F`/`H` report task finished / skipped (eval mode)

**Gamepad** (identical layout on every OS via SDL's controller mapping):
`Share` base · `L1`/`R1` select right/left arm (operator-facing: the robot's
own left arm sits on your right when facing it) · left stick translate ·
right stick turn / orient · `L2`/`R2` vertical · `○/B` close · `×/A` open ·
click left/right stick = speed up/down · one rumble pulse on new contact

**VR** (mirror teleop, operator faces the screen): hold **grip** to drive the
corresponding arm, release to lock · **trigger** close, `A/X` open ·
right stick base translation, left stick turn/spine ·
**click left/right stick = speed up/down** · one haptic pulse on new contact ·
monitor view by default, `--hmd-view` for an in-headset screen

**GELLO**: move the physical GELLO Duo arms — sim arms follow 1:1 in joint
space (no clutch, no IK: GELLO already gives absolute joint angles) ·
squeeze the GELLO gripper to grasp (same force-servo physics as every other
input mode) · keyboard drives the mobile base (arrows/`Home`/`End`/
`PageUp`/`PageDown`, same as keyboard mode)

## Input devices in depth

Topic contracts and verification detail beyond the capability matrices above.

### GELLO (official EBiM competition input device)

The GELLO publisher itself — building it, calibrating it, running it — is
**out of scope for this repo and for `setup_eval.bat`/`setup_eval.sh`**:
it lives in, and is set up per, the official
[**EBiM-Benchmark/teleoperation**](https://github.com/EBiM-Benchmark/teleoperation)
repo (its own README documents two setup paths: a Pixi-based conda
environment, or a devcontainer). This section only covers what happens on
*our* side once that publisher is already running.

`--input gello` subscribes to the ROS 2 topics published by the official
`franka_gello_state_publisher` — it does not talk to the Dynamixel/OpenRB-150
hardware directly, so that reference node (with its calibrated
`assembly_offsets`/`joint_signs`/gripper range per your physical rig) must
already be running and publishing on `/left` and `/right`. This needs
ROS 2 (rclpy) whether or not `--mnet` is also on — run it inside the eval
Docker image or a native ROS 2 / RoboStack environment.

<!-- COMMANDS HIDDEN (GELLO integration still in progress, not verified end
to end - see the capability matrix. Restore by unwrapping once confirmed
working, so participants don't try commands before they're ready.
```bash
# terminal 1: the official GELLO publisher (from the teleoperation repo)
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
# terminal 2: this sim, subscribing to it
python main.py --input gello
```
-->

Topic contract (per-arm namespace from that launch config):
`<ns>/gello/joint_states` (`sensor_msgs/JointState`, 7 joint angles, already
offset/sign-corrected and clamped to the real FR3 limits) and
`<ns>/gripper/gripper_client/target_gripper_width_percent`
(`std_msgs/Float32`, 0.0–1.0). The gripper open/closed direction (1.0 =
open) is inferred from the topic name and not yet confirmed against real
hardware — flag it if it's inverted on your rig
(`teleop/config.py`: `GELLO_GRIPPER_CLOSE_BELOW`/`GELLO_GRIPPER_OPEN_ABOVE`).

The USB foot pedal (same reference repo, `pedal_state_publisher`) drives
the mobile base in the GELLO workflow (GELLO occupies both hands):
`/pedal/state` (`std_msgs/String`, one of `A`/`B`/`C`/`A+C`/`B+C`/`NONE`).
The state → motion mapping matches the reference repo's own
`pedal_state_subscriber.py` example: `A` = forward, `B` = turn left,
`A+C` = backward, `B+C` = turn right (`C` alone is left unmapped there, so
it's a no-op here too); see `teleop/config.py`'s `PEDAL_BASE_COMMANDS` to
change it.

### Unified ROS 2 teleop (`--input ros_teleop`)

Splits teleoperation into two processes connected by ROS 2 topics: a
**publisher node** reads the physical device and publishes device-agnostic
Cartesian commands; the sim (`--input ros_teleop`) subscribes and applies
them through the exact same IK/grasp/base-drive code the local modes use.
Use it when the device and the sim must live in different processes,
containers, or machines. Two publishers ship in `teleop_ros2/`
(keyboard and gamepad); both are baked into the eval image:

```bash
# terminal 1: the sim as consumer
python main.py --input ros_teleop
# terminal 2: a publisher (device attached to THIS machine)
ros2 run keyboard_teleop_publisher keyboard_teleop_publisher
# or: ros2 run gamepad_teleop_publisher gamepad_teleop_publisher
# no device handy? each publisher has a scripted self-test: --pattern 60
```

Topic contract: `/cmd_vel` (`geometry_msgs/Twist`, base — REP-103
`base_link` frame, `linear.z` repurposed for the spine lift rate),
`<side>/teleop_cmd` (`geometry_msgs/Twist`, per-arm Cartesian TCP twist),
`<side>/gripper_cmd` (`std_msgs/Float32`, >0.5 = close intent). `/cmd_vel`
matches the topic EBiM_Challenge's own Isaac Sim test commands already use.
GELLO is intentionally not part of this contract — it stays joint-space and
IK-free (`--input gello` above). Note: contact rumble/haptics only exist in
the local modes; the ROS contract has no haptic feedback channel yet.

Both publishers poll their device continuously (Windows: `GetAsyncKeyState`
for the keyboard, SDL for the gamepad; Linux: X11 `query_keymap` for the
keyboard) and drive base/arm motion through the exact same screen-relative
frame math the local input modes use, fed by the sim's
`/mujoco/teleop_feedback` (camera azimuth + robot yaw, 30 Hz) — so direction
feel is identical to driving locally.

**Verification status**: the full chain (publisher → topics → sim) has been
verified end-to-end with the publishers' `--pattern` synthetic self-test
and with real keyboard/gamepad hardware on Windows. Real devices on
Ubuntu are in community testing — reports welcome.

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by removing this
comment wrapper (and its matching closing marker further down).

## Quick start — Docker eval (Ubuntu), step by step

1. Once per login session (native Linux):

   ```bash
   xhost +local:docker
   ```

2. **Terminal 1 — simulator + eval bridge** (the first run builds the
   image from source, several minutes):

   ```bash
   ./eval.sh sim
   ```

   A viewer window opens — **this is the robot you will drive**. Ready
   when the log shows `[mnet] bridge up: camera ...`.

3. **Terminal 2 — the official client**; pick `cable_management` →
   `local_test` in its menu:

   ```bash
   ./eval.sh client
   ```

   `mnet_client-ros_2/config/team_config.json` ships pre-filled for local
   testing — no registration, nothing to edit.

4. **Let the tiers advance**: tiers other than Tier 2 are skipped
   automatically. When Tier 2 starts, the board fixtures rearrange
   (randomized per the client's coordinates) and the cable is re-laid.

5. **Type the one-time code**: when the client prints it, switch to
   **terminal 1** and type `code <TEXT>`. The code appears on the plate
   next to the board, inside the evidence camera's frame.

6. **Do the task**: click into the viewer window and route the cable with
   the keyboard — `7/8/9` select base/left/right, arrows move,
   `G`/`V` close/open the gripper (full list under [Controls](#controls)).
   When done, press `F` in the viewer to report completion. Remaining
   tiers auto-skip and the client finishes on its own.

7. **Collect results**: video and logs are in
   `robotiq_duo_full_scene_minimal_core/release/mnet_out/` — scoring uses
   the fixed overhead evidence camera, not your viewer perspective. Shut
   everything down with `./eval.sh down`.

**Evaluating with a gamepad** (native Linux Docker only): plug the pad in
and replace step 2 with

```bash
./eval.sh gamepad          # terminal 2 stays ./eval.sh client
```

### Registration + `connection_test` before a real submission

From the vendored client's own [README](mnet_client-ros_2/README.md):

> Select your interested benchmark task
> [here](https://manipulation-net.org/index.html#tasks), and get
> registered [here](https://manipulation-net.org/registration.html).

That gives you the `team_unique_code` for `team_config.json`. Before
spending a real attempt, sanity-check it — from
[`connection_test.py`](mnet_client-ros_2/mnet_client/connection_test.py):

> This is the entry script for connection test with the server and check
> the qualification of the team

Run it with `ros2 run mnet_client connection_test` (Windows:
`ros_native.bat ros2 run mnet_client connection_test`); it does not count
against the submission rate limit. Once it passes, run the client's
`submission` mode instead of `local_test` (attempts themselves are
rate-limited — see the [mnet docs](https://mnet-client.readthedocs.io/)).
-->

## Other ways to run it

**Docker teleop without ROS** (`docker-run.sh`, native Linux) — a third
practice option: same image as `eval.sh` but ROS-free and smaller, for
driving the sim without installing conda/mujoco natively at all (see
[RELEASE.md](robotiq_duo_full_scene_minimal_core/RELEASE.md) for how it
compares to the eval image):

```bash
./docker-run.sh                    # keyboard (default)
./docker-run.sh --input gamepad    # gamepad (native Linux only)
./docker-run.sh --no-viewer        # headless self-check
./docker-run.sh build              # rebuild the image only
```

Two things cannot work inside any container, by nature of what they need:
**gamepad** requires `/dev/input` passthrough (native Linux Docker only —
uncomment the `devices:` line in `release/compose.yaml`), and **VR**
requires direct host device/driver access — practice VR natively.

**Self-testing without ROS**:

```bash
./start.sh --no-viewer                    # headless smoke test
./start.sh --randomize-board              # fixture randomization, same
                                          # distribution the client uses
./start.sh --display-code TEST1234        # preview the one-time-code plate
```

## Directory layout

```
robotiq_duo_full_scene_minimal_core/   simulator (entry: main.py, code: teleop/)
mnet_client-ros_2/                     official ManipulationNet ROS 2 client (vendored)
teleop_ros2/                           ROS 2 teleop publisher packages (keyboard, gamepad)
start.bat / start.sh                   native practice launchers (Windows / Ubuntu)
eval.sh                                scored ManipulationNet evaluation (Docker + ROS 2)
docker-run.sh                          ROS-free Docker teleop (installation-free practice)
setup_eval.bat / setup_eval.sh         one-click Docker-free eval setup (Windows / Ubuntu)
ros_native.bat                         Windows ROS 2 (RoboStack) command wrapper (see appendix)
```

## Launcher reference

`start.bat` / `start.sh` / `python main.py` accept the same flags:

| Flag | Meaning |
|---|---|
| `--input keyboard\|gamepad\|vr\|gello\|ros_teleop` | input device (default: keyboard) |
| `--no-viewer` | headless smoke test, exits after `init/smoke ok` |
| `--randomize-board [--seed N]` | randomize fixtures offline (client's distribution) |
| `--display-code TEXT` | preview the one-time-code plate |
| `--profile` | print loop/control/physics/render timing once per second |
| `--render-hz N` | cap the viewer refresh rate |
| `--help` | the full per-mode option list |

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by re-inserting
this row after `--input`:
| `--mnet` | start the ManipulationNet eval bridge (needs ROS 2; `eval.sh sim` does this for you) |
-->

## Building the Docker images

Distribution is by `git clone`: the images are **built locally from this
repo**, there is nothing to pull from a registry. Normally you never do
this by hand — `./docker-run.sh` builds its image on first use
(`./docker-run.sh build` forces a rebuild).

To build manually:

```bash
# ROS-free practice image (smaller; keyboard/gamepad teleop only)
docker compose -f robotiq_duo_full_scene_minimal_core/release/compose.yaml build runtime
```

The first build downloads the base image and Python wheels (several
minutes); later builds reuse the cache and only re-copy changed sources.

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by unwrapping.
Original intro also said "... `./eval.sh sim` rebuilds its image
incrementally on every start (code changes are picked up automatically),
and ..." before "`./docker-run.sh` builds its image on first use".
Original build block also had this entry above the runtime one:
# evaluation image (ROS 2 Humble + official mnet client + simulator, ~3.7 GB)
docker compose -f robotiq_duo_full_scene_minimal_core/release/compose.yaml build sim
-->

<!-- EVAL SECTION HIDDEN (pending mnet meeting) — restore by removing this
comment wrapper (and its matching closing marker further down).

## Appendix — Ubuntu Docker-free eval

Community testing, **unverified** (no Ubuntu host was available to test
this on — report back before treating it as verified). The standard scored
path is Docker (`eval.sh`); if you cannot use Docker at all:

```bash
./setup_eval.sh
```

sets up both halves in one shot. Unlike the old recipe (still described
below for what it does under the hood), it does **not** need Ubuntu 22.04
or an apt-installed ROS: ROS 2 Humble comes from
[RoboStack](https://robostack.github.io/) (conda), the same one conda env
covers the simulator *and* the client (no numpy-version split between
halves — `numpy<2` throughout), and it never touches an existing ROS 1
install (e.g. Noetic) since everything lives in that isolated env.

### What it does

One-time env setup:

```bash
conda create -n ros-humble --override-channels -c robostack-staging -c conda-forge \
    python=3.11 ros-humble-ros-base ros-humble-cv-bridge colcon-common-extensions
conda activate ros-humble
pip install mujoco==3.9.0 "numpy>=1.24,<2" glfw==2.10.0 pygame==2.6.1 \
    "pillow>=10" pyopenxr==1.1.5301 PyOpenGL==3.1.10 openvr==2.12.1401 \
    opencv-python "pydantic>=2,<3" requests tqdm pupil-apriltags pybullet \
    python-xlib
```

Build the client + the `ros_teleop` publishers into `ros_ws/` (symlinked
sources, so `git pull` updates are picked up without re-copying):

```bash
mkdir -p ros_ws/src
ln -sfn "$(pwd)/mnet_client-ros_2" ros_ws/src/mnet_client
ln -sfn "$(pwd)/teleop_ros2/keyboard_teleop_publisher" ros_ws/src/keyboard_teleop_publisher
ln -sfn "$(pwd)/teleop_ros2/gamepad_teleop_publisher" ros_ws/src/gamepad_teleop_publisher
( cd ros_ws && colcon build )
```

Then the same `team_config.json` `file_dir` fix as Windows (see the
[registration section](#registration--connection_test-before-a-real-submission)
below for why this goes through Python, not a shell redirect) — `setup_eval.sh`
does this automatically via the shared
[`mnet_client_postpatch.py`](robotiq_duo_full_scene_minimal_core/release/mnet_client_postpatch.py).

### Running the eval

```bash
conda activate ros-humble && source ros_ws/install/setup.bash
cd robotiq_duo_full_scene_minimal_core
python main.py --input keyboard --mnet      # or gamepad / vr / gello
```

second terminal (same `conda activate` + `source` first):

```bash
ros2 run mnet_client local_test
```

The session flow is identical to the [Docker walkthrough](#quick-start--docker-eval-ubuntu-step-by-step).

## Quick start — Windows RoboStack eval, step by step

Verified end-to-end: with keyboard input, the vendored client's
`local_test` scored a Tier2 session to completion (recorded evaluation
video); gamepad and VR input were each confirmed working through the same
client session. The evidence camera holds a constant 30 fps (measured over
a 75 s run: 30.00 fps, max inter-frame gap 44.6 ms — the camera renders and
publishes from its own process, so viewer load does not affect it).

### One-time setup

ROS 2 Humble on Windows comes from [RoboStack](https://robostack.github.io/)
— one conda env runs both halves:

```bat
conda create -n ros-humble --override-channels -c robostack-staging -c conda-forge ^
    python=3.11 ros-humble-ros-base ros-humble-cv-bridge colcon-common-extensions
conda run -n ros-humble pip install mujoco==3.9.0 "numpy>=1.24,<2" glfw==2.10.0 ^
    pygame==2.6.1 "pillow>=10" pyopenxr==1.1.5301 PyOpenGL==3.1.10 openvr==2.12.1401 ^
    opencv-python "pydantic>=2,<3" requests tqdm pupil-apriltags pybullet
```

Every ROS command from here on goes through `ros_native.bat` (repo root). It
exists because other installed software (base conda, Docker, Git) ships
same-named older DLLs that shadow RoboStack's and break `rclpy` on import —
the wrapper rebuilds a minimal `PATH`, applies the ROS env, and injects the
shared-memory Fast DDS profile (`release/fastdds_shm.xml`; Windows UDP
loopback cannot sustain reliable ~1 MB camera frames).

Build the client once (a real copy, not a junction — colcon fails through
junctions):

```bat
xcopy /E /I mnet_client-ros_2 ros_ws\src\mnet_client
cd ros_ws & ..\ros_native.bat colcon build & cd ..
```

Two Windows-specific fixes after building — do these once, and again after
any rebuild of `ros_ws`:

- **`file_dir`**: edit `ros_ws\install\share\mnet_client\config\team_config.json`
  and point `file_dir` at a writable local directory (this is where results
  land in step 7 below). Use a plain-text editor or Python — PowerShell's
  `Out-File`/`Set-Content` writes a BOM that breaks the client's JSON
  parsing.
- **stdin crash**: the client polls the keyboard with `select.select()`,
  which is POSIX-only and crashes on Windows (`WinError 10038`). Until
  upstream ships a fix, guard those calls with `msvcrt.kbhit()`: 3 call
  sites in `local_test_client.py` (under
  `ros_ws\install\Lib\site-packages\mnet_client\clients\`).
  `submission_client.py` polls the same way — apply the same fix there
  before a real submission run.

Same one-time build is needed for the two `teleop_ros2/` publishers if you
want `--input ros_teleop` with real hardware (not just `--pattern`):

```bat
xcopy /E /I teleop_ros2\keyboard_teleop_publisher ros_ws\src\keyboard_teleop_publisher
xcopy /E /I teleop_ros2\gamepad_teleop_publisher ros_ws\src\gamepad_teleop_publisher
cd ros_ws & ..\ros_native.bat colcon build --merge-install & cd ..
```

### Running the eval

1. **Before every session** — clear leftovers from previously force-killed
   runs; stale Fast DDS shared-memory segments silently degrade the camera
   stream (see [Troubleshooting](#troubleshooting)):

   ```powershell
   Get-Process python -EA 0 | ? { $_.Path -like '*ros-humble*' } | Stop-Process -Force
   Remove-Item $env:TEMP\fastrtps_* -Force -EA 0
   ```

2. **Terminal 1 — simulator + eval bridge**:

   ```bat
   ros_native.bat python robotiq_duo_full_scene_minimal_core\main.py --input keyboard --mnet
   :: or: --input gamepad
   ```

   A viewer window opens — **this is the robot you will drive**. Ready when
   the log shows `[mnet] bridge up: camera ...`.

3. **Terminal 2 — the official client**; pick `cable_management` →
   `local_test` in its menu:

   ```bat
   ros_native.bat ros2 run mnet_client local_test
   ```

4. **Let the tiers advance**: tiers other than Tier 2 are skipped
   automatically. When Tier 2 starts, the board fixtures rearrange
   (randomized per the client's coordinates) and the cable is re-laid.

5. **Type the one-time code**: when the client prints it, switch to
   **terminal 1** and type `code <TEXT>`. The code appears on the plate
   next to the board, inside the evidence camera's frame.

6. **Do the task**: click into the viewer window and route the cable
   (keyboard or gamepad — full control list under [Controls](#controls)).
   When done, press `F` in the viewer to report completion. Remaining
   tiers auto-skip and the client finishes on its own.

7. **Collect results**: video and logs land in the `file_dir` you set in
   `team_config.json` during setup — scoring uses the fixed overhead
   evidence camera, not your viewer perspective. Close both terminal
   windows (or Ctrl+C) to shut down; there is no `eval.sh down` step here.

For an **official submission**, see
[registration + connection_test](#registration--connection_test-before-a-real-submission)
above — same steps, just prefix each `ros2` command with `ros_native.bat`.
-->

## Troubleshooting

- **"init/smoke ok"** = environment is healthy
- **`[keyboard] held-key backend: press-timeout fallback (...)`** (Linux) —
  held keys will cut out; the message names the cause. Usually python-xlib
  is missing (the launcher auto-installs it on the next run) or the session
  has no X server.
- **`[mnet] WARNING: evidence camera publishing at N fps`** — the camera process
  cannot reach the client's 25 fps minimum (it will refuse the session).
  Typical cause: a container without GPU access. On a Linux host
  with nvidia-container-toolkit, `eval.sh` merges the GPU overlay
  (`release/compose.gpu.yaml`) automatically — look for its
  "NVIDIA container runtime detected" line at startup; if it's missing,
  install the toolkit. (The overlay uses a `deploy:` reservation, which
  only applies with `compose up` — that's why eval.sh starts the sim
  detached and attaches your terminal for the `code <TEXT>` input.)
- **Client reports low camera FPS while the sim shows none of the warnings
  above** — stale Fast DDS state from previous force-killed runs silently
  degrades delivery (the publisher runs at 30 fps, subscribers receive ~5).
  Kill leftover sim/client Python processes and delete the orphaned
  shared-memory segments: `%TEMP%\fastrtps_*` on Windows (they are
  disk-backed — **a reboot does not remove them**), `/dev/shm/fastrtps_*`
  on Linux. On Windows:

  ```powershell
  Get-Process python -EA 0 | ? { $_.Path -like '*ros-humble*' } | Stop-Process -Force
  Remove-Item $env:TEMP\fastrtps_* -Force -EA 0
  ```
- **"Failed to open video writer" / "Could not find encoder for
  codec_id=27"** (Linux, client) — the client records the session with
  `cv2.VideoWriter(*"avc1")` (H.264), but PyPI's `opencv-python` wheel never
  ships an H.264 encoder (licensing) and this failure isn't caught until the
  point where a run tries to save video. Install the apt package
  **`python3-opencv`** instead — it's what the client's own `package.xml`
  declares (`python3-opencv` rosdep) and links the system `ffmpeg`, which
  does carry `libx264` on Ubuntu. Do not `pip install opencv-python`
  alongside it: pip's copy shadows the working apt one. Windows does not hit
  this (OpenCV falls back to the OS's Media Foundation encoder there).
- **`DLL load failed while importing _rclpy_pybind11`** (Windows,
  RoboStack) — another installed program's directory on `PATH` shadows
  RoboStack's DLLs with older same-named copies. Run everything through
  `ros_native.bat`, which rebuilds a minimal `PATH` first.
- **`FormFactorUnavailable`** (VR) — the streaming app is not connected or the
  headset is not being worn
- **"Camera topic has no publishers"** (client) — start `./eval.sh sim` before
  `./eval.sh client`
- **No window from Docker on native Linux** — run `xhost +local:docker` once
  per session
- Scoring uses the overhead evidence camera, not your viewer perspective

## License & attribution

Copyright (c) 2026 **2houyuhang**. Licensed under the
[Apache License 2.0](LICENSE) — the source is open; keep the copyright and
attribution notices when using or modifying it.

`mnet_client-ros_2` is the official [ManipulationNet](https://manipulation-net.org)
client (Apache-2.0), vendored with non-runtime content removed: other
tasks' assets, the physical board's reference CAD files, and documentation
images. The complete client — including everything trimmed here — is
available upstream via the [mnet client docs](https://mnet-client.readthedocs.io/)
and [manipulation-net.org](https://manipulation-net.org/). Scene and robot
assets are provided for research and benchmark use.
