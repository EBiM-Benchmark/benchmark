> **Location note**: this directory (`task1_mujoco/`) is the MuJoCo
> implementation of Task 1 (cable management) inside the
> EBiM-Benchmark/benchmark repository, the sibling of `task1_isaacsim/`.
> It is self-contained: simulator, ManipulationNet client, teleop publisher
> packages, Docker stack and one-click scripts all live here. The
> `teleop_ros2/` packages are also proposed upstream to the
> `EBiM-Benchmark/teleoperation` repository (branch
> `2houyuhang/feature/mujoco-teleop-publishers`); once merged there, the
> copy here can be swapped for a submodule reference.

# EBiM Benchmark Task 1 — Cable Management Simulation (MuJoCo)

Task 1 teleoperates a mobile dual-arm Franka FR3 platform (Robotiq 2F-85
grippers on a planar-drive base with a vertical spine) to route a deformable
cable across a fixture board — the ManipulationNet **cable_management**
benchmark (Tier 2). The simulator supports **keyboard, gamepad, VR, GELLO,
and a unified ROS 2 teleop mode**, and integrates the official
ManipulationNet ROS 2 client for end-to-end scored evaluation: fixed
overhead evidence camera, automatic tier skipping, client-driven fixture
randomization, and an in-scene one-time-code display.

## How it works

Everything runs in a **single MuJoCo process** (`main.py`): robot, cable,
board and physics live in one simulation — no separate worlds, no coupling
layers. Every input device feeds the same control stack (grasp-aware
scaling → smoothing → contact clamp → damped-least-squares IK → force-servo
grasping), so the robot feels identical no matter which device you hold.

There are two ways to run that process, for two different purposes:

- **Native — for practice.** `start.bat` / `start.sh` run the sim directly
  in a conda environment the launcher creates and maintains by itself.
  Best performance and feel; all local devices (keyboard / gamepad / VR).
  No ROS, no Docker — and no scoring.
- **Docker — for the scored evaluation.** `eval.sh` runs the same sim in a
  container that also carries ROS 2 Humble and the official ManipulationNet
  client. The sim publishes the overhead evidence camera over ROS; the
  client watches it, drives the session (tier skipping, fixture
  randomization, one-time code) and records the scored video. **You drive
  the containerized sim's viewer window with the keyboard.**

Practice natively until you are comfortable, then score in Docker —
physics, controls and IK are identical on both sides.

**What works where** (details and verification status in
[Teleoperation options](#teleoperation-options)):

| You want to... | Windows | Native Ubuntu |
|---|---|---|
| Practice — keyboard / gamepad / VR | ✅ `start.bat` | ✅ keyboard + gamepad (`./start.sh`) · ⚠️ VR untested |
| Scored eval — keyboard | ✅ native, no Docker ([appendix](#appendix--fully-docker-free-evaluation-windows-robostack)) | ⚠️ retest pending — `./eval.sh sim` + `client` |
| Scored eval — gamepad | ✅ native ([appendix](#appendix--fully-docker-free-evaluation-windows-robostack)) | ⚠️ retest pending — `./eval.sh gamepad` + `client` |
| Scored eval — VR | ⚠️ community testing (native, same appendix) | ⚠️ untested — native sim + Docker client ([recipe](#quick-start--the-scored-evaluation-docker-step-by-step)) |
| GELLO (needs ROS 2) | ⚠️ untested (RoboStack ROS 2 works, appendix) | ⚠️ untested — Docker image or natively ([appendix](#appendix--fully-docker-free-evaluation-ubuntu)) |
| Eval without Docker at all | ✅ verified ([appendix](#appendix--fully-docker-free-evaluation-windows-robostack)) | ⚠️ possible ([appendix](#appendix--fully-docker-free-evaluation-ubuntu)) |

## Architecture

```
 OPERATOR SIDE                                      SIMULATOR (this repo, main.py)
 ─────────────                                      ──────────────────────────────
 local devices (no ROS)
   keyboard / gamepad / OpenXR headset ───────────► --input keyboard|gamepad|vr

 teleop publisher nodes (teleop_ros2/, or your own)
   keyboard_teleop_publisher ──┬── /cmd_vel
   gamepad_teleop_publisher  ──┤   /left|right/teleop_cmd ────► --input ros_teleop
                               └── /left|right/gripper_cmd
              ▲ /mujoco/teleop_feedback (camera azimuth + robot yaw, 30 Hz)

 official EBiM teleoperation repo (GELLO rig)
   franka_gello_state_publisher ── /left|right/gello/joint_states ─► --input gello
   pedal_state_publisher ───────── /pedal/state (drives the base) ─►

         every input lands in the same control stack: grasp-aware scaling →
         smoothing → contact clamp → damped-least-squares IK → force-servo
         grasping   (gello is the exception: joint-space 1:1, no IK)

 evaluation stack (Docker, ./eval.sh)
   official mnet client ◄── /mujoco/camera/image_raw   (fixed overhead camera —
                        ◄── F / H completion reports    rendered+published by a
                        ──► fixture randomization,      dedicated process at a
                            one-time code               constant 30 fps)
```

## Directory layout

```
robotiq_duo_full_scene_minimal_core/   simulator (entry: main.py, code: teleop/)
mnet_client-ros_2/                     official ManipulationNet ROS 2 client (vendored)
teleop_ros2/                           ROS 2 teleop publisher packages (keyboard, gamepad)
start.bat / start.sh                   native practice launchers (Windows / Ubuntu)
eval.sh                                scored ManipulationNet evaluation (Docker + ROS 2)
docker-run.sh                          ROS-free Docker teleop (installation-free practice)
ros_native.bat                         Windows ROS 2 (RoboStack) command wrapper (see appendix)
```

## Prerequisites

- **Native practice**: Miniconda on PATH (Windows: install it once;
  Ubuntu: `start.sh` installs it for you). A GPU helps but is not required.
- **Docker evaluation**: Docker Engine with Compose v2. **Scored runs need
  native Linux** (or a 3D-accelerated VM) — WSL2's software rendering
  cannot reach the evidence camera's required 25 fps; the client refuses
  lower rates and the sim warns you within ~15 s. X11 for the viewer:
  `xhost +local:docker` once per login session. On Windows, skip Docker and
  use the native evaluation (appendix below) — verified end-to-end.
- **VR only**: an OpenXR headset and runtime (see VR setup below).
- **GELLO only**: the physical GELLO Duo plus the
  [teleoperation](https://github.com/EBiM-Benchmark/teleoperation) repo's
  publisher (see the GELLO section).

No large-asset downloads and no container overlays: everything is in the
repo, and each launcher bootstraps itself on first run (conda env / Docker
image).

## One-time setup

```bash
git clone --recurse-submodules https://github.com/EBiM-Benchmark/benchmark.git
cd benchmark/task1_mujoco
```

(The submodules belong to the Isaac Sim task; this directory needs none of
them and no extra downloads.) That's it — the launchers handle the rest on
their first run.

## Quick start — native practice (keyboard / gamepad / VR)

Windows (after installing Miniconda, double-click `start.bat`, or):

```bat
start.bat                      :: keyboard
start.bat --input gamepad      :: gamepad
start.bat --input vr           :: VR (see VR setup below)
start.bat --no-viewer          :: headless self-check ("init/smoke ok")
```

Ubuntu:

```bash
./start.sh                     # keyboard
./start.sh --input gamepad
./start.sh --input vr
```

The first run creates the `duo-teleop` conda env (a few minutes); later
runs self-check the env (installing anything missing) and start in seconds.

### VR setup

The VR code is pure OpenXR and works with any active runtime:

| Headset | Windows | Ubuntu |
|---|---|---|
| Quest 2/3 | Meta Quest Link (set as active OpenXR runtime) | [WiVRn](https://github.com/WiVRn/WiVRn) (no Steam) or ALVR + SteamVR |
| Index / Vive | SteamVR | SteamVR |

Wear the headset (or cover the proximity sensor) when starting — the runtime
reports the device unavailable otherwise. On Ubuntu an X11 session is
required (the launcher forces XWayland automatically under Wayland).

**When you are comfortable driving the robot, move on to the scored run
below — the simulator and controls there are identical.**

## Quick start — the scored evaluation (Docker), step by step

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
   `G`/`V` close/open the gripper (full list under Controls below). When
   done, press `F` in the viewer to report completion. Remaining tiers
   auto-skip and the client finishes on its own.

7. **Collect results**: video and logs are in
   `robotiq_duo_full_scene_minimal_core/release/mnet_out/` — scoring uses
   the fixed overhead evidence camera, not your viewer perspective. Shut
   everything down with `./eval.sh down`.

For an **official submission**, run the client's `submission` mode instead
of `local_test` and put your real `team_unique_code` into
`mnet_client-ros_2/config/team_config.json` (attempts are rate-limited —
see the [mnet docs](https://mnet-client.readthedocs.io/)).

**Evaluating with a gamepad** (native-Linux Docker only — WSL2 has no
`/dev/input` passthrough): plug the pad in and replace step 2 with

```bash
./eval.sh gamepad          # terminal 2 stays ./eval.sh client
```

**Evaluating with VR — native sim + native ROS 2** (community testing): VR
cannot enter a container, but the sim itself runs the eval bridge natively
whenever `rclpy` is importable. Note the conda env cannot see an apt-based
ROS (Python version mismatch) — use the system Python via a venv instead.
On Ubuntu with ROS 2 Humble installed and your headset connected (WiVRn):

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages ~/.venv-duo-teleop-ros
source ~/.venv-duo-teleop-ros/bin/activate
pip install mujoco==3.9.0 "numpy>=2,<3" glfw==2.10.0 pygame==2.6.1 \
    "pillow>=10" pyopenxr==1.1.5301 PyOpenGL==3.1.10 openvr==2.12.1401 \
    python-xlib
cd robotiq_duo_full_scene_minimal_core
python main.py --input vr --mnet     # terminal 1: native VR sim + eval bridge
```

Terminal 2 stays `./eval.sh client` — its container runs on the host
network and discovers the native sim's topics directly. The session flow is
unchanged (type `code <TEXT>` into terminal 1; report completion with `F`
in the desktop viewer window). A conda-based alternative is RoboStack
(ROS 2 inside the conda env, works on Windows too). Both recipes are in
community testing — reports welcome.

## Teleoperation options

| Input | Launch | Needs | Verified so far |
|---|---|---|---|
| **Keyboard** (default) | `./start.sh`, or the eval sim window | nothing | Windows + Ubuntu with real keyboards (Linux needs X11/XWayland — forced automatically) |
| **Gamepad** | `--input gamepad` | any SDL-recognized pad (Xbox / PS / generic) | Windows + Ubuntu with a real pad (same SDL mapping → identical layout) |
| **VR** | `--input vr` (native only) | OpenXR headset + runtime | Windows via Quest Link; Ubuntu via WiVRn pending |
| **GELLO** | `--input gello` (needs ROS 2) | GELLO Duo + official publisher running | topic contract with synthetic data; real hardware pending |
| **ROS 2 topics** | `--input ros_teleop` (needs ROS 2) | a publisher node (two ship in `teleop_ros2/`) | end-to-end with the publishers' `--pattern` self-test; real devices pending |

"Pending" = implemented and passing automated / synthetic tests, but not yet
validated by us on that platform or hardware — those combinations are in
**community testing**. If you run one, a quick report (works / broken, plus
the terminal log) via GitHub Issues helps every team.

## Docker teleop without ROS (installation-free practice)

For driving the sim without installing conda/mujoco natively — and for
verifying it behaves identically on Windows and Linux (a smaller, ROS-free
image; see [RELEASE.md](robotiq_duo_full_scene_minimal_core/RELEASE.md) for
how it compares to the eval image):

```bash
./docker-run.sh                    # keyboard (default)
./docker-run.sh --input gamepad    # gamepad (native Linux only)
./docker-run.sh --no-viewer       # headless self-check
./docker-run.sh build              # rebuild the image only
```

Two things cannot work inside any container, by nature of what they need:
**gamepad** requires `/dev/input` passthrough (native Linux Docker only —
uncomment the `devices:` line in `release/compose.yaml`), and **VR**
requires direct host device/driver access — practice VR natively.

## Building the Docker images

Distribution is by `git clone`: the images are **built locally from this
directory**, there is nothing to pull from a registry. Normally you never
do this by hand — `./eval.sh sim` rebuilds its image incrementally on every
start (code changes are picked up automatically), and `./docker-run.sh`
builds its image on first use (`./docker-run.sh build` forces a rebuild).

To build manually:

```bash
# evaluation image (ROS 2 Humble + official mnet client + simulator, ~3.7 GB)
docker compose -f robotiq_duo_full_scene_minimal_core/release/compose.yaml build sim

# ROS-free practice image (smaller; keyboard/gamepad teleop only)
docker compose -f robotiq_duo_full_scene_minimal_core/release/compose.yaml build runtime
```

The first build downloads the base image and Python wheels (several
minutes); later builds reuse the cache and only re-copy changed sources.

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
`Share` base · `L1`/`R1` arms · left stick translate ·
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

## GELLO (official EBiM competition input device)

`--input gello` subscribes to the ROS 2 topics published by the official
[`franka_gello_state_publisher`](https://github.com/EBiM-Benchmark/teleoperation)
— it does not talk to the Dynamixel/OpenRB-150 hardware directly, so that
reference node (with its calibrated `assembly_offsets`/`joint_signs`/gripper
range per your physical rig) must already be running and publishing on
`/left` and `/right`. This needs ROS 2 (rclpy), same as `--mnet` — run it
inside the eval Docker image or a native ROS 2 / RoboStack environment:

```bash
# terminal 1: the official GELLO publisher (from the teleoperation repo)
ros2 launch franka_gello_state_publisher main.launch.py config_file:=franka_gello_duo.yaml
# terminal 2: this sim, subscribing to it
python main.py --input gello
```

Topic contract (per-arm namespace from that launch config):
`<ns>/gello/joint_states` (`sensor_msgs/JointState`, 7 joint angles, already
offset/sign-corrected and clamped to the real FR3 limits) and
`<ns>/gripper/gripper_client/target_gripper_width_percent`
(`std_msgs/Float32`, 0.0–1.0). The gripper open/closed direction (1.0 =
open) is inferred from the topic name and not yet confirmed against real
hardware — flag it if it's inverted on your rig
(`teleop/config.py`: `GELLO_GRIPPER_CLOSE_BELOW`/`GELLO_GRIPPER_OPEN_ABOVE`).

## Unified ROS 2 teleop (`--input ros_teleop`)

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

**Verification status**: the full chain (publisher → topics → sim) has been
verified end-to-end with the publishers' `--pattern` synthetic self-test
(resulting TCP motion bit-identical to raw topic publishing). Physical
keyboard/gamepad runs over ROS are in community testing — reports welcome.

## Self-testing without ROS

```bash
./start.sh --no-viewer                    # headless smoke test
./start.sh --randomize-board              # fixture randomization, same
                                          # distribution the client uses
./start.sh --display-code TEST1234        # preview the one-time-code plate
```

## Launcher reference

`start.bat` / `start.sh` / `python main.py` accept the same flags:

| Flag | Meaning |
|---|---|
| `--input keyboard\|gamepad\|vr\|gello\|ros_teleop` | input device (default: keyboard) |
| `--mnet` | start the ManipulationNet eval bridge (needs ROS 2; `eval.sh sim` does this for you) |
| `--no-viewer` | headless smoke test, exits after `init/smoke ok` |
| `--randomize-board [--seed N]` | randomize fixtures offline (client's distribution) |
| `--display-code TEXT` | preview the one-time-code plate |
| `--profile` | print loop/control/physics/render timing once per second |
| `--render-hz N` | cap the viewer refresh rate |
| `--help` | the full per-mode option list |

## Appendix — fully Docker-free evaluation (Ubuntu)

Community testing — the standard scored path is Docker (`eval.sh`). If you
cannot use Docker at all, both halves of the eval stack can run natively on
Ubuntu 22.04 with ROS 2 Humble:

**1. The simulator with the eval bridge** — the conda env cannot import an
apt-installed `rclpy` (interpreter mismatch), so use the system Python via
a venv (same recipe as the VR evaluation above; pick any `--input`):

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages ~/.venv-duo-teleop-ros
source ~/.venv-duo-teleop-ros/bin/activate
pip install mujoco==3.9.0 "numpy>=2,<3" glfw==2.10.0 pygame==2.6.1 \
    "pillow>=10" pyopenxr==1.1.5301 PyOpenGL==3.1.10 openvr==2.12.1401 \
    python-xlib
cd robotiq_duo_full_scene_minimal_core
python main.py --input keyboard --mnet      # or gamepad / vr / gello
```

**2. The official client, built natively** (its own terminal, system
Python — not the venv: `cv_bridge` needs `numpy<2`, which is why the two
halves keep separate environments):

```bash
sudo apt install ros-humble-ros-base ros-humble-cv-bridge \
    python3-colcon-common-extensions python3-pip
pip install "numpy>=1.24,<2" opencv-python "pydantic>=2,<3" requests tqdm \
    pupil-apriltags pybullet
mkdir -p ~/mnet_ws/src && ln -s "$(pwd)/mnet_client-ros_2" ~/mnet_ws/src/mnet_client
( cd ~/mnet_ws && source /opt/ros/humble/setup.bash && colcon build )
```

Before running it, edit `mnet_client-ros_2/config/team_config.json` and
change `file_dir` from the container path `/ws/out` to a writable local
directory (e.g. `/tmp/mnet_out`). Then:

```bash
source /opt/ros/humble/setup.bash && source ~/mnet_ws/install/setup.bash
ros2 run mnet_client local_test
```

The session flow is identical to the Docker walkthrough. This Ubuntu recipe
is community testing (the same two-halves stack is verified end-to-end on
Windows below) — reports welcome.

## Appendix — fully Docker-free evaluation (Windows, RoboStack)

Verified end-to-end: with keyboard input, the vendored client's
`local_test` scored a Tier2 session to completion (recorded evaluation
video); gamepad input was confirmed working through the same client
session. The evidence camera holds a constant 30 fps (measured over a 75 s
run: 30.00 fps, max inter-frame gap 44.6 ms — the camera renders and
publishes from its own process, so viewer load does not affect it). ROS 2
Humble on Windows comes from
[RoboStack](https://robostack.github.io/) — one conda env runs both halves:

```bat
conda create -n ros-humble --override-channels -c robostack-staging -c conda-forge ^
    python=3.11 ros-humble-ros-base ros-humble-cv-bridge colcon-common-extensions
conda run -n ros-humble pip install mujoco==3.9.0 "numpy>=1.24,<2" glfw==2.10.0 ^
    pygame==2.6.1 "pillow>=10" pyopenxr==1.1.5301 PyOpenGL==3.1.10 openvr==2.12.1401 ^
    opencv-python "pydantic>=2,<3" requests tqdm pupil-apriltags pybullet
```

Every ROS command below goes through `ros_native.bat` (repo root). It exists
because other installed software (base conda, Docker, Git) ships same-named
older DLLs that shadow RoboStack's and break `rclpy` on import — the wrapper
rebuilds a minimal `PATH`, applies the ROS env, and injects the shared-memory
Fast DDS profile (`release/fastdds_shm.xml`; Windows UDP loopback cannot
sustain reliable ~1 MB camera frames).

**1. Build the client once** (a real copy, not a junction — colcon fails
through junctions):

```bat
xcopy /E /I mnet_client-ros_2 ros_ws\src\mnet_client
cd ros_ws & ..\ros_native.bat colcon build & cd ..
```

Two Windows-specific fixes after building:

- `team_config.json` (`ros_ws\install\share\mnet_client\config\`): set
  `file_dir` to a writable local directory. Edit it with a plain-text
  editor or Python — PowerShell's `Out-File`/`Set-Content` writes a BOM
  that breaks the client's JSON parsing.
- The client polls the keyboard with `select.select()` on stdin, which is
  POSIX-only and crashes on Windows (`WinError 10038`). Until upstream
  ships a fix, guard those calls with `msvcrt.kbhit()` on Windows: 3 call
  sites in `local_test_client.py` (built copy lives under
  `ros_ws\install\Lib\site-packages\mnet_client\clients\`; re-apply after
  any rebuild). `submission_client.py` polls the same way — apply the same
  fix there before a real submission run.

**2. Run the eval** (two terminals, both via the wrapper):

```bat
ros_native.bat python robotiq_duo_full_scene_minimal_core\main.py --input keyboard --mnet
:: or: --input gamepad
ros_native.bat ros2 run mnet_client local_test
```

Before each run, clear leftovers from previous force-killed sessions —
they silently degrade the camera stream (see Troubleshooting):

```powershell
Get-Process python -EA 0 | ? { $_.Path -like '*ros-humble*' } | Stop-Process -Force
Remove-Item $env:TEMP\fastrtps_* -Force -EA 0
```

## Troubleshooting

- **"init/smoke ok"** = environment is healthy
- **`[keyboard] held-key backend: press-timeout fallback (...)`** (Linux) —
  held keys will cut out; the message names the cause. Usually python-xlib
  is missing (the launcher auto-installs it on the next run) or the session
  has no X server.
- **`[mnet] WARNING: evidence camera publishing at N fps`** — the camera process
  cannot reach the client's 25 fps minimum (it will refuse the session).
  Typical cause: WSL2 or a container without GPU access. Run the eval on
  native Linux, or enable the GPU blocks in `release/compose.yaml` — for
  **both** the `sim` and `sim-gamepad` services (nvidia-container-toolkit).
- **Client reports low camera FPS while the sim shows none of the warnings
  above** — stale Fast DDS state from previous force-killed runs silently
  degrades delivery (the publisher runs at 30 fps, subscribers receive ~5).
  Kill leftover sim/client Python processes and delete the orphaned
  shared-memory segments: `%TEMP%\fastrtps_*` on Windows (they are
  disk-backed — **a reboot does not remove them**), `/dev/shm/fastrtps_*`
  on Linux. The Windows appendix has a copy-paste cleanup.
- **`DLL load failed while importing _rclpy_pybind11`** (Windows,
  RoboStack) — another installed program's directory on `PATH` shadows
  RoboStack's DLLs with older same-named copies. Run everything through
  `ros_native.bat`, which rebuilds a minimal `PATH` first.
- **"OpenGL version 1.5 or higher required"** (WSL2) — usually a stale
  `LIBGL_ALWAYS_INDIRECT=1` or a manual `DISPLAY=<ip>:0` left in `~/.bashrc`
  from old X-server setups: remove them (WSLg sets `DISPLAY` itself). Then
  try `wsl --update` + `wsl --shutdown` from Windows. Last resort:
  `LIBGL_ALWAYS_SOFTWARE=1 ./start.sh` (software rendering, slower).
- **"X11: Failed to open display localhost:N.0" / "could not initialize
  GLFW"** (WSL2) — an SSH-forwarded `DISPLAY` is set in this shell (sshd
  sets `localhost:10.0`-style values); WSLg's real display is `:0`. The
  launch scripts rewrite that pattern automatically; when running things by
  hand, prefix the command with `DISPLAY=:0`.
- **Window opens then goes black / crashes** (WSL2, hybrid-graphics laptop)
  — the integrated GPU's OpenGL support through Mesa's D3D12 translation
  layer can segfault mid-render. `start.sh`/`eval.sh` auto-detect a
  discrete NVIDIA/AMD GPU and steer rendering to it; if you still hit this,
  set it manually: `export MESA_D3D12_DEFAULT_ADAPTER_NAME="NVIDIA"` (or
  `"AMD"`/`"Radeon"`, matching your GPU name) before running.
- **`FormFactorUnavailable`** (VR) — the streaming app is not connected or the
  headset is not being worn
- **"Camera topic has no publishers"** (client) — start `./eval.sh sim` before
  `./eval.sh client`
- **No window from Docker on native Linux** — run `xhost +local:docker` once
  per session (WSL2 needs nothing; WSLg handles the display)
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
