# Task 3 — Assisted Living & Feeding (MuJoCo)

## Overview

This folder contains the MuJoCo implementation of Task 3: a mobile dual-FR3
robot with Robotiq 2F-85 grippers, a bowl of coffee beans, a spoon, a plate, a
cup, and an IKEA scale station. It is the MuJoCo counterpart to the Isaac Sim
runtime in [`task3_isaacsim/`](../task3_isaacsim/README.md), and runs natively —
no Docker, no GPU container, no ROS.

It supports:

- mobile-base, spine, left-arm, and right-arm teleoperation;
- Cartesian arm control through damped least-squares inverse kinematics and
  position actuators;
- optional direct joint-position control;
- contact-force-limited gripper closing;
- selectable world-, base-, or viewer-camera-relative motion frames;
- 100-bean and 300-bean scenes;
- optional RGB windows for the head and wrist cameras;
- a force-sensing scale platform with auto-tare;
- configurable initial robot and object poses.

Current capability status is tracked in [STATUS.md](../STATUS.md). Official
scoring follows the rules published on the
[competition page](https://ebim-benchmark.github.io/competition.html#tasks);
anything in this repository is a development facilitator.

## Provenance

Ported from the upstream `Mujoco_Genisis_Model` repository, branch `main`
(`09e2f89`, "updated MuJoCo scene with moved table and IKEA assets"). The
simulation code, scene XML, and assets are upstream work, carried over unmodified
apart from the changes listed under
[Differences from upstream](#differences-from-upstream).

**Licensing.** The upstream repository carries no LICENSE file of its own. This
code was contributed to the EBiM Benchmark under the **Apache License, Version
2.0** with the author's agreement, so it is covered by this repository's
[LICENSE](../LICENSE) like any other original work here. The terms are recorded
in [NOTICE](../NOTICE), the author is listed in
[CONTRIBUTORS.md](../CONTRIBUTORS.md), and `teleop.py` carries the standard EBiM
SPDX header.

Because the tuned control code in `teleop.py` tracks an external repository, this
directory is excluded from **Ruff** only (via `extend-exclude` in
[`pyproject.toml`](../pyproject.toml)) — reformatting to line-length 79 would
churn it and make upstream syncs conflict-prone. Every other hook, including the
license-header and file-safety hooks, applies normally; see
[`.pre-commit-config.yaml`](../.pre-commit-config.yaml).

## Package contents

```text
task3_mujoco/
├── assets/             meshes and textures
├── config.json         runtime configuration
├── requirements.txt    Python dependencies
├── robot.xml           robot model
├── run.sh              default launcher
├── scene_100.xml       scene containing 100 coffee beans
├── scene_300.xml       scene containing 300 coffee beans
├── scripts/            large-asset download helper and its manifest
└── teleop.py           simulation and teleoperation program
```

## Prerequisites

- Linux desktop with a working graphical display (the viewer is interactive;
  there is no headless mode)
- Python 3.10 or newer
- MuJoCo-compatible OpenGL/GLFW graphics drivers
- No GPU required, though the 300-bean scene benefits from one

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r task3_mujoco/requirements.txt
```

`glfw` is required because `teleop.py` imports it at module scope.
`opencv-python` is needed only for the optional camera windows, but it is
installed by the command above.

## Step 1 — Fetch the large assets (required)

Twenty-one visual meshes and textures (~173 MB raw, ~41 MB zipped) exceed this
repository's 2 MB per-file limit and are **not tracked in git**. They are hosted
on OneDrive, following the same flow as
[`task1_isaacsim`](../task1_isaacsim/README.md). The scenes will not compile
without them. Fetch them once after cloning:

```bash
task3_mujoco/scripts/download_large_assets.sh
```

If the direct download fails (OneDrive usually needs a manual click — its share
links render a JavaScript viewer page, so `curl` gets a 403 or that HTML page
instead of the zip), open the share link printed by the script in a browser,
download the zip, and unpack it into `task3_mujoco/` yourself — it already has
the correct internal layout:

```bash
unzip -o ~/Downloads/task3_mujoco_large_assets.zip -d task3_mujoco/
```

Or pass an override:

```bash
LARGE_ASSETS_URL="https://…" task3_mujoco/scripts/download_large_assets.sh
```

Either way, confirm what is present without downloading:

```bash
task3_mujoco/scripts/download_large_assets.sh --check
```

The authoritative file list is
[`scripts/large_assets.txt`](scripts/large_assets.txt); both the download script
and `teleop.py`'s preflight check read it, so they cannot drift apart. The script
validates the archive before unpacking, so a OneDrive HTML page returned in place
of the zip is reported rather than silently extracted.

## Step 2 — Run

```bash
cd task3_mujoco
./run.sh
```

The launcher executes `python3 teleop.py --config config.json`, forwarding any
extra arguments. Keep the main MuJoCo window focused while using the keyboard.

## Keyboard controls

### Select the controlled part

| Key | Controlled part |
|---|---|
| `7` | Mobile base and spine |
| `8` | Left arm and left gripper |
| `9` | Right arm and right gripper |

### Base and spine mode (`7`)

| Keys | Motion |
|---|---|
| `W` / `S` or `Up` / `Down` | Move forward / backward |
| `A` / `D` or `Left` / `Right` | Move left / right |
| `Q` / `E` or `Home` / `End` | Yaw left / right |
| `U` / `J` or `Page Up` / `Page Down` | Move the spine up / down |

### Arm translation mode (`8` or `9`)

Translation mode is active by default when an arm is selected.

| Keys | Motion in the configured end-effector frame |
|---|---|
| `W` / `S` or `Up` / `Down` | Forward / backward (`+X` / `-X`) |
| `A` / `D` or `Left` / `Right` | Left / right (`+Y` / `-Y`) |
| `U` / `J` or `Page Up` / `Page Down` | Up / down (`+Z` / `-Z`) |

### Arm rotation mode

Press `R` to toggle the selected arm between translation and rotation modes.

| Keys | Rotation |
|---|---|
| `U` / `J` or `Page Up` / `Page Down` | Positive / negative roll about frame `X` |
| `W` / `S` or `Up` / `Down` | Positive / negative pitch about frame `Y` |
| `A` / `D` or `Left` / `Right` | Positive / negative yaw about frame `Z` |

### Gripper and utility controls

| Key | Action |
|---|---|
| `G` | Close the selected gripper until the force threshold is reached |
| `V` or `Space` | Open the selected gripper |
| `P` | Print measured state, targets, commands, and gripper forces |
| `L` | Reload `joint_position_targets` from `config.json` in direct-joint mode |
| `Esc` | Exit and close the viewer and camera windows |

In `direct_joint_position` mode the arm is driven toward the targets in
`joint_position_targets`; Cartesian keyboard motion and the `R` toggle are unused.

This matches [`task1_mujoco`](../task1_mujoco/README.md)'s keyboard contract —
same `7`/`8`/`9` selection, arrow cluster, `Home`/`End`, `PageUp`/`PageDown`,
`R`, `G`, `V`/`Space` — so operators can move between the two tasks without
relearning the mapping.

## Configuration

Everything is driven by [`config.json`](config.json); most sections are read at
startup, so restart after editing.

### Bean count

```json
"scene": { "bean_count": 100 }
```

`100` loads `scene_100.xml`, `300` loads `scene_300.xml`. The 300-bean scene is
substantially slower because of the bean-bean and bean-object contact count.

### Motion frames

```json
"motion_frames": { "base": "base", "end_effector": "base" }
```

Both accept `"world"` (fixed MuJoCo world axes), `"base"` (current mobile-base
orientation), or `"camera"`. `"camera"` means the **main interactive viewer
camera**, not `head_cam` or the wrist cameras — changing the viewer angle changes
the command directions. `motion_frames.base` affects planar base translation
only; base yaw and spine keep their normal directions.

### Camera windows

```json
"camera_views": {
  "enabled": false,
  "width": 640, "height": 480, "render_hz": 20.0,
  "head_cam": true, "left_wrist_cam": true, "right_wrist_cam": true
}
```

Set `enabled` to `true` for separate OpenCV windows per selected camera. Each
camera toggles independently. Images come from `mujoco.Renderer`; they are not
values in `data.sensordata`.

**Performance:** every enabled view adds offscreen rendering and image transfer.
Cost scales with camera count, resolution, `render_hz`, and the 300-bean scene.
Keep `enabled` false unless you need it.

### Scale sensor

```json
"scale_sensor": { "auto_tare": true, "tare_force_n": 0.0 }
```

A force sensor on the welded scale platform. The controller exposes
`scale_weight_force_n` and `scale_weight_kg` after subtracting the tare. With
`auto_tare` true, the platform and empty knock-box weight is measured at startup
and subtracted — note that an object already resting on the scale at startup is
included in that tare value, so start with the platform clear.

### Initial poses

`initial_robot` sets the base pose, spine height, and per-arm joint positions;
`initial_objects` sets bowl, plate, spoon, and cup poses. Positions are metres,
joint angles radians, quaternions MuJoCo `[w, x, y, z]` order. Arm and spine
values are clipped to the hard ranges in `robot.xml`. With
`initial_objects.move_beans_with_bowl` true, beans keep their pose relative to
the bowl when the bowl is moved.

After changing object poses, check that nothing initially intersects the table,
plate, bowl, robot, or another object — interpenetration produces large contact
forces and unstable motion.

## Verification status

Both scenes compile and instantiate on MuJoCo 3.11.0 / Python 3.14.6:

| Scene | Bodies | Geoms | Meshes | Textures | Cameras |
|---|---:|---:|---:|---:|---:|
| `scene_100.xml` | 223 | 884 | 247 | 20 | 4 |
| `scene_300.xml` | 423 | 1284 | 247 | 20 | 4 |

Cameras present: `overview`, `head_cam`, `left_wrist_cam`, `right_wrist_cam`.

> [!IMPORTANT]
> Upstream `main` ships **no automated tests or validation scripts**, and
> `teleop.py` requires an interactive display, so there is no headless check that
> exercises the controller. Verification so far covers model compilation, config
> loading, and the asset flow only. Teleoperation, force-limited grasping, and a
> full four-stage run still need a maintainer at a display — [STATUS.md](../STATUS.md)
> reflects that.

## Troubleshooting

**Keyboard commands not detected.** Click the main MuJoCo viewer so it has focus.
The program tries `pynput` for press/release tracking and falls back to GLFW
polling.

**`ModuleNotFoundError: No module named 'glfw'`.** Install from
`requirements.txt`; `teleop.py` imports `glfw` at module scope.

**`Error opening file 'assets/...obj'`.** The large assets are missing — run
`scripts/download_large_assets.sh` (see Step 1). The preflight check normally
catches this first with a clearer message.

**Camera windows make it slow.** Set `camera_views.enabled` to `false`, or
enable fewer cameras and reduce resolution / `render_hz`.

**The 300-bean scene is very slow.** Expected — contact count rises sharply.
Develop against 100 beans and switch to 300 for the required experiment.

## Differences from upstream

Everything else is byte-identical to `main` at `09e2f89`.

1. **`requirements.txt` gained `glfw>=2.7`.** `teleop.py` imports `glfw` at
   module scope (line 32, unconditional), so following upstream's documented
   setup produced a `ModuleNotFoundError` on a clean environment. Upstream's own
   README says glfw "is listed explicitly", but it was absent from the file.
2. **`teleop.py` gained a `check_large_assets()` preflight** that fails with an
   actionable message when the externally-hosted assets are absent, instead of
   MuJoCo's bare `Error opening file '...obj'`.
3. **Redundant assets dropped** (~64 MB): `assets/usd_sources/robot_room(1).usd`
   (53 MB, referenced by no scene; the repo already ships
   [`assets/robot_room.usd`](../assets/robot_room.usd)),
   `assets/ikea_exact/ikea_scale.glb` (4.5 MB, byte-identical to
   [`assets/ikea_scale.glb`](../assets/ikea_scale.glb)), and 12 unreferenced
   `.jpg`/`.jpeg` source copies of `.png` textures that no scene loads. Verified
   by parsing every `<mesh>` and `<texture>` element; all 264 referenced assets
   resolve. Upstream retains them if they are ever needed.
4. **`__pycache__/teleop.cpython-313.pyc` dropped** — a committed build artifact.

New files added by this integration: `scripts/download_large_assets.sh` and
`scripts/large_assets.txt`.

### Known upstream issue, not fixed here

Upstream's README documents the scale sensor in a section that is truncated
mid-sentence (it begins `o_tare=true\`, the platform and empty knock-box…`). The
feature itself is real and configured under `scale_sensor`; this README documents
it properly above, but the upstream file still carries the broken text.
