# Task 3 — Assisted Living & Feeding (MuJoCo)

## Overview

This folder contains the MuJoCo implementation of Task 3: the mobile dual-FR3
robot with Robotiq 2F-85 grippers scooping coffee beans with a segmented-mesh
spoon in the assisted-living room scene. It is the MuJoCo counterpart to the
Isaac Sim runtime in [`task3_isaacsim/`](../task3_isaacsim/README.md), and runs
natively — no Docker, no GPU container, no ROS.

Current capability status is tracked in [STATUS.md](../STATUS.md). Official
scoring follows the rules published on the
[competition page](https://ebim-benchmark.github.io/competition.html#tasks);
anything in this repository is a development facilitator.

## Provenance

Ported from the upstream `Mujoco_Genisis_Model` repository, branch
`updated_scene_100_beans` (`09e2f89`, "updated MuJoCo scene with moved table and
IKEA assets"). The simulation code, scene XML, and assets are upstream work,
carried over unmodified apart from the two integration-specific changes noted
under [Differences from upstream](#differences-from-upstream). Because this
directory tracks an external repository, it is excluded from this repo's Ruff
lint/format hooks and from the license-header hook — see
[`.pre-commit-config.yaml`](../.pre-commit-config.yaml) and
[`pyproject.toml`](../pyproject.toml).

## Prerequisites

- Python 3.10+
- No GPU required; MuJoCo's software renderer is sufficient, though a GPU helps
  considerably with the 300-bean scene.
- On Linux, `pynput` needs an X11 session (`python-xlib` is pulled in by
  `requirements.txt`).

```bash
python -m pip install -r task3_mujoco/requirements.txt
```

`requirements_teleop.txt` is a lighter subset for teleoperation only; it omits
`trimesh`/`scipy`, which some validation scripts need.

## Step 1 — Fetch the large assets (required)

Twenty visual meshes and textures (~169 MB raw, ~37 MB zipped) exceed this
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

The script verifies the archive before unpacking, so a OneDrive HTML page
returned in place of the zip is reported rather than silently extracted.

The authoritative file list is
[`scripts/large_assets.txt`](scripts/large_assets.txt); both the download script
and the launcher's preflight check read it, so they cannot drift apart. The
collision meshes, the spoon, the beans, and every scene XML **are** tracked in
git — only large visual geometry and two large textures are external.

## Step 2 — Run

```bash
cd task3_mujoco
./run_simulation.sh --beans 100
```

For the required 300-bean scene:

```bash
./run_simulation.sh --beans 300
```

`run_simulation.py` defaults to 300 beans and robot teleoperation. The retained
`--gripper-collision primitives` spelling is only a compatibility alias; both
current scene variants include `robot_task3_small_model_mesh.xml` and use the
exact small-model contact geometry. Unknown arguments are forwarded to the
selected controller. `--dry-run` prints the command without running it.

## Controls

- `7`: base and spine
- `8`: left arm
- `9`: right arm
- `G`: close the selected gripper to the fixed `0.735 rad` target
- `V` or `Space`: open the selected gripper
- `R`: toggle arm translation/rotation mode
- `N`: show or hide collision geometry
- `B`: print contacts

Arm translation uses the robot-base frame by default, while arm rotation uses
the lowest-point TCP frame. Wheel and caster collision/friction are disabled;
the base is moved through the virtual planar joints.

### Vertical arm control

The default arm translation frame is the robot base frame:

- `U` or `PageUp`: move the selected gripper upward in world Z.
- `J` or `PageDown`: move the selected gripper downward in world Z.
- `W/S` and `A/D`: horizontal motion relative to the robot base.

At the initial spoon pose the TCP local +Z axis points downward, so under
TCP-frame translation `U`/`PageUp` would move the gripper *down*. Use
`--arm-frame tcp` only when tool-local translation is intentionally required.

## Scene variants

| Scene XML | Beans | Robot include | Reached by |
|---|---:|---|---|
| `final_scene_100_beans_mesh.xml` | 100 | `robot_task3_small_model_mesh.xml` | `--beans 100` |
| `final_scene_300_beans_mesh.xml` | 300 | `robot_task3_small_model_mesh.xml` | `--beans 300` (default) |
| `final_scene_{100,300}_beans_primitives.xml` | 100/300 | `robot_task3_small_model_mesh.xml` | `--gripper-collision primitives` alias |
| `final_scene_*_spoon_mocap_runtime.xml` | 100/300 | `robot_task3_small_model_mesh.xml` | `--control spoon` |
| `final_scene.xml`, `final_scene_{100,300}_beans.xml` | — | `robot.xml` (legacy) | not reachable from the launcher |

The `robot.xml` scenes are the superseded pre-transfer robot, kept for reference
and for `generate_bean_scenes.py`. See [Known issues](#known-issues) for the
validator consequence.

## Exact transferred spoon and gripper model

### Spoon

- Original visual OBJ: `dynamic_spoon2_centered.obj`
- Fourteen original handle collision mesh sections:
  `spoon_stem_mesh_00.obj` … `spoon_stem_mesh_13.obj`
- Original small-model neck box and two bowl ellipsoids
- Spoon contact: `condim=6`, friction `1.0 0.02 0.002`

### Both Robotiq grippers

- Original visual meshes
- Original collision STL meshes on all nine physical links for environment
  contact
- A second copy of each original fingertip collision STL for the dedicated
  spoon contact
- Grasp contact: `condim=6`, friction `2.5 0.10 0.03`, margin `0.00025`
- Joint dynamics: `armature=0.01`, `damping=1.0`, `frictionloss=0.1`
- Mimic constraints: `solref="0.006 1"`, `solimp="0.995 0.9999 0.0001 0.5 2"`
- Position actuator: `kp=500`, `kv=30`, force range `-100 100`
- Fixed closing command: `0.735 rad`

The spoon is not welded to the gripper.

## MuJoCo physical parameters

The task scenes use the small model's settings:

```xml
<option
    timestep="0.001"
    gravity="0 0 -9.81"
    integrator="implicitfast"
    solver="Newton"
    iterations="100"
    ls_iterations="20"
    cone="elliptic"
    impratio="10">
  <flag contact="enable"/>
</option>
```

Default contact parameters are:

```xml
solref="0.008 1"
solimp="0.96 0.995 0.001 0.5 2"
```

The FR3 joints remain position-controlled, but the unsafe ultra-stiff servos
were replaced by moderate joint-dependent gains and FR3-like torque limits.
Gravity/bias compensation is applied every simulation step:

| Joint | kp | kv | Force/torque limit |
|---|---:|---:|---:|
| 1–2 | 6000 | 180 | ±87 |
| 3–4 | 5000 | 160 | ±87 |
| 5 | 1800 | 70 | ±12 |
| 6 | 1400 | 60 | ±12 |
| 7 | 1000 | 50 | ±12 |

The controller integrates IK velocity into `q_ref`, but immediately freezes and
resets `q_ref` to measured joint positions when the selected arm contacts a
static room wall. The collision latch remains active until the motion key is
released and contact has cleared, preventing repeated controller wind-up.

## Workspace alignment

The table and Task-3 objects were repositioned so that, after normal settling,
the spoon and gripper reproduce the working small-model geometry. There is no
initial gripper–table penetration.

## Collision filtering

- Dedicated fingertip STL grasp meshes contact the spoon collision model only.
- Ordinary Robotiq STL collision meshes contact the table, bowl, plate, beans,
  floor, walls, doors, and furniture.
- This prevents duplicate fingertip–spoon contacts while retaining full
  environment collision.
- Wheel and caster geoms use `contype=0`, `conaffinity=0`, and zero friction.

## Dynamic spine control

The vertical spine remains a slide joint with a position actuator, but it is no
longer moved by directly overwriting `qpos` and `qvel`. The controller only
updates the actuator target at the requested speed (default `0.18 m/s`) and
applies bias/gravity compensation. MuJoCo therefore integrates a real spine
velocity and acceleration. The arm flanges move dynamically, the stiff flange
welds transmit force to the free gripper bodies, and fingertip friction can lift
a grasped spoon during spine motion.

Spine actuator settings:

```xml
<position name="franka_spine_vertical_joint"
          kp="12000" kv="800"
          forcerange="-10000 10000"
          forcelimited="true"/>
```

No runtime spine `qpos`/`qvel` teleportation and no per-step gripper projection
are used. The gripper free bodies are projected to the flanges only once during
initialization; normal motion is transmitted through the physical equality
welds.

## Static-wall collision guard

Room walls are identified by the fixed `collision_wall_mat` geoms. If the
selected arm or its attached gripper contacts one of these walls, the
controller:

1. stops integrating the IK-derived `qdot` into `q_ref`;
2. resets `q_ref` to the measured seven joint positions;
3. holds those positions with gravity compensation;
4. latches the block until the user releases the movement key and the contact
   clears.

This prevents the position target from accumulating behind an immovable wall and
avoids the large opposing actuator/contact forces that caused
`Nan, Inf or huge value in QACC` warnings.

## Curved scoop collision model

The 14 segmented stem meshes and the existing neck box are unchanged. Only the
two coarse scoop ellipsoids are replaced by 139 fitted boxes (110 surface, 24
rim, 5 leading-edge). Every fitted scoop box retains the proven working spoon
contact configuration:

```xml
contype="1" conaffinity="16" condim="6"
friction="1.0 0.02 0.002" margin="0"
solref="0.008 1"
solimp="0.96 0.995 0.001 0.5 2"
priority="0"
```

This prevents the ordinary low-friction gripper collision meshes from contacting
the scoop while preserving scoop contact with the dedicated fingertip grasp
meshes, table, beans, bowl, plate, and other environment objects.

## Two-part spoon collision hierarchy

The spoon collision model is split into two rigid child bodies:

- `spoon_handle_neck_part`: the 14 stem meshes and neck box.
- `spoon_scoop_part`: the 139 fitted curved scoop boxes.

Both child bodies have no joint, so they are exactly rigid relative to the
parent spoon.

## Lowest-point TCP rotation

Arm rotations use a dedicated TCP at the midpoint of the lowest surfaces of the
two closed fingertip collision meshes. In the gripper-base frame this point is
approximately:

```text
[0.0, -0.000375, 0.163052783] m
```

The equivalent point in the FR3 flange frame is encoded in
`robot_task3_small_model_mesh.xml` for both arms. The arm Jacobian is evaluated
at the explicit `*_arm_control_tcp_pivot` site, not at the flange. When rotation
mode is entered (`R`), the current pivot position is stored and proportional
position feedback prevents numerical drift while angular commands are applied.
The visible TCP marker is in MuJoCo site group 4.

## Validation

The scripts below are headless structural and physics regressions. Run them from
this directory after fetching the large assets:

```bash
cd task3_mujoco
python validate_exact_small_model_transfer.py
python validate_grasp_lift.py
python validate_gripper_table_contact.py
python validate_all_scene_contacts.py
python validate_dual_gripper_attachment.py
python validate_motion_speed.py
python validate_safe_position_control.py
python validate_wall_collision_guard.py
python validate_dynamic_spine_lift.py
python validate_curved_scoop.py
python validate_two_part_spoon.py
python validate_lowest_tcp_rotation.py
python validate_mode_switch_fix.py
python validate_vertical_key_mapping.py
python validate_nonaccumulating_targets.py
```

At the time of this port, 14 of the 15 pass on MuJoCo 3.11.0 / Python 3.14.6;
`validate_safe_position_control.py` fails for a pre-existing upstream reason
documented under [Known issues](#known-issues).

The table-supported dynamic regression closes to `0.735 rad`, obtains four
six-dimensional contacts on each fingertip, and lifts the spoon approximately
`0.262 m` with approximately `1.77 mm` relative motion.

The 300-bean scene is computationally expensive because the exact 1 ms timestep
and high-accuracy contact solver are intentionally retained.

## Known issues

- `validate_safe_position_control.py` fails on the legacy `robot.xml` scenes
  (`final_scene.xml`, `final_scene_100_beans.xml`, `final_scene_300_beans.xml`),
  which have no position actuators. The validator globs every `final_scene*.xml`
  rather than only the four scenes the launcher can reach. This reproduces
  identically on the upstream branch at `09e2f89` and was **not** introduced by
  this port. The scenes participants actually run are unaffected.
- Teleoperation, grasping, and a full four-stage run have not yet been verified
  by an EBiM maintainer on this branch; [STATUS.md](../STATUS.md) reflects that.

## Differences from upstream

Only three changes were made to upstream files; everything else is
byte-identical to `09e2f89`.

1. **`run_simulation.py`** gained a `check_large_assets()` preflight that fails
   with an actionable message when the externally-hosted assets are absent.
   Without it MuJoCo aborts model compilation with a bare
   `Error opening file 'assets/robot/...obj'`. The check is skipped under
   `--dry-run`.
2. **Fourteen unreferenced texture files were dropped** (~5.9 MB): source-format
   `.jpg`/`.jpeg`/`.exr` duplicates of `.png` textures that no scene XML loads,
   plus `3d66Model-19059402-files-024.{jpg,png}`, which nothing references.
   Verified by parsing every `<mesh>` and `<texture>` element across all scene
   XML; all 109 referenced assets resolve.
3. **`teleop_keyboard.py`'s on-screen `HELP` text was corrected** (text only, no
   behaviour change). It described arm translation as "always TCP-local", but
   `--arm-frame` defaults to `base` — the very confusion the upstream README
   warns about, where `U`/`PageUp` moves the gripper *down* under the TCP frame.
   The help now describes the base-frame default, notes what `--arm-frame tcp`
   changes, and states that rotation is TCP-local regardless.

New files added by this integration: `scripts/download_large_assets.sh` and
`scripts/large_assets.txt`.
