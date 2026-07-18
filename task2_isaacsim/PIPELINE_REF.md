# Task 2 — Pipeline Reference

Technical reference for the Task 2 Isaac Sim teleoperation bridge and the
demonstration-recording pipeline. For a high-level overview and quickstarts,
see the [README](README.md).

## Container / process topology

| Process | Where it runs | Repo mount | Role |
|---|---|---|---|
| Scene script + bridge (`scripts/scene_room.py` / `scene_barebone.py` + `isaacsim_fr3duo_teleop_bridge_core.py`) | `isaac-sim-5-1-0-workshop` container, via `/isaac-sim/python.sh` (launched by `scripts/run_isaacsim_teleop.sh`) | `/workspace/EBiM_Challenge` | Simulation, teleop runtime, joint state/command ROS node; with `--record` also `/isaac/clock` (bridge node), the camera OmniGraphs (`scripts/recording/camera_publishers.py`), and ground-truth publishers (`scripts/recording/scene_capture.py`) |
| Helper stack (`ros_republisher`, `position_controller`, `teleop_adapters`, `browser_controller`) | `task2_*` containers from [docker-compose.yml](docker-compose.yml), all `ros:jazzy-ros-base`-based | `../task1_isaacsim` at `/workspace` | Task 1 scripts reused verbatim: remap `/bridge/*` commands onto `/isaac/*`, adapt device topics, serve the browser UI (port 8090) |
| Recorder (`services/recording/record_task2.py`) | `task2_lerobot_recorder` container (compose profile `record`), launched by `scripts/run_recorder.sh` | whole repo at `/repo`, working dir `/repo/task2_isaacsim` | Subscribes to the recording topics and writes the LeRobot dataset + `task2_extras/` sidecar |
| Device publishers (keyboard / GELLO / pedal) | host, from the [`teleoperation`](https://github.com/EBiM-Benchmark/teleoperation) repo | — | Publish `/keyboard/state`, `/{left,right}/gello/joint_states`, pedal state |

Everything is `network_mode: host` + FastDDS over UDPv4, so topics flow
between the containers and the host without any broker configuration.

## Topic contract

All Task 2 topic names live in **[`config/topics.yaml`](config/topics.yaml)**
and are loaded through
[`scripts/topics.py`](scripts/topics.py) by the bridge
core, the sim-side recording publishers, and the recorder. The loader **fails
hard** when the file or a required key is missing — no process falls back to
baked-in names, so a rename is either picked up everywhere or rejected loudly
at startup. `topics.py` is stdlib + PyYAML only and safe to import outside
Isaac Sim; the recorder imports it through
`sys.path.insert(<task2>/scripts)`.

### Teleop command flow

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/keyboard/state`, `/{left,right}/gello/joint_states`, gripper width topics | various | host device publishers → teleop adapters |
| `/pedal/state` | `std_msgs/String` | teleop adapters (`keyboard_to_base.py`) or host pedal publisher → bridge (swerve base) |
| `/bridge/{left,right}_joint_commands`, `/bridge/{left,right}_robotiq_joint_commands` | `sensor_msgs/JointState` | adapters / browser UI → republisher + position controller *(task1-side names, not in the contract)* |
| `/isaac/{left,right}_joint_commands` | `sensor_msgs/JointState` | position controller → bridge |
| `/isaac/{left,right}_robotiq_joint_commands` | `sensor_msgs/JointState` | republisher (gripper calibration) → bridge |
| `/isaac/browser/{left,right}_joint_commands`, `/isaac/browser/{left,right}_robotiq_joint_commands` | `sensor_msgs/JointState` | browser controller → bridge (disabled with `--disable-browser-command-topics`) |

### Bridge state (60 Hz)

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/isaac/{left,right}_joint_states` | `sensor_msgs/JointState` | bridge → helper stack / UI |
| `/isaac/{left,right}_robotiq_joint_states` | `sensor_msgs/JointState` | bridge → helper stack / UI |

### Recording streams (bridge, only with `--record` / `--publish-recording-topics`)

| Topic | Type | Content |
|---|---|---|
| `/isaac/applied_joint_commands` | `sensor_msgs/JointState` | post-arbitration joint position targets (action source) |
| `/isaac/joint_states_full` | `sensor_msgs/JointState` | full articulation state |
| `/isaac/odom` | `nav_msgs/Odometry` | base odometry |
| `/isaac/cmd_vel_applied` | `geometry_msgs/Twist` | applied base twist (body frame) |
| `/isaac/{left,right}_ee_pose` | `geometry_msgs/PoseStamped` | link8 end-effector poses |

### Ground truth (`scripts/recording/scene_capture.py`, only with `--record`)

| Topic | Type | Content |
|---|---|---|
| `/isaac/task2/object_poses` | `std_msgs/String` (JSON) | `{"sim_time", "objects": {name: [x,y,z,qw,qx,qy,qz]}}` |
| `/isaac/task2/pad_points` | `std_msgs/Float32MultiArray` | `[sim_time, n_points, x0,y0,z0,...]` deformed pad vertices (~10 Hz) |
| `/isaac/task2/scene_reset` | `std_msgs/String` (JSON) | reset/randomize event, published after the reset completes |
| `/isaac/task2/scene_reset_request` | `std_msgs/String` | any message triggers a scene reset (recorder menu keys `1` reset+record and `5` reset; same effect as the sim-window `5` hotkey) |

### Clock and cameras

| Topic | Type | Producer |
|---|---|---|
| `/isaac/clock` | `rosgraph_msgs/Clock` | bridge node, from `world.current_time` (recorder paces on sim time; rebases to 0 on scene reset) |
| `<namespace>/image_raw`, `<namespace>/camera_info`, `<namespace>/depth` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | per-camera OmniGraph; namespaces `/isaac/head_camera` (1280×720), `/isaac/{left,right}_wrist_camera` (848×480); depth only with `--robot-camera-depth` |
| `/isaac/eval_camera/{image_raw,depth,camera_info}` | `sensor_msgs/*` | scene camera (1280×720 static top-down camera) from `config/cameras_<scene>.yaml`, both scenes with `--enable-scene-cameras` (implied by `--record`) |
| `/isaac/eval_camera/{bbox_2d_tight,semantic_labels,semantic_segmentation}` | `vision_msgs/Detection2DArray`, `std_msgs/String`, `sensor_msgs/Image` | scene camera `publish_bbox`/`publish_semantic` flags; feeds the recorder's `--suggest-success` IoU suggestion |

### Coupled values (renaming in topics.yaml is NOT enough)

- **Command topics + `/pedal/state`** are rebuilt from `--bridge-prefix` /
  `--isaac-prefix` inside the Task 1 services reused via mount
  (`task1_isaacsim/scripts/controllers/ros_joint_republisher.py`, the browser
  controller, the teleop adapters). The bridge logs its loaded contract at
  startup — compare against `ros2 topic list` when in doubt.
- **Robot camera namespaces/resolutions** are duplicated in
  `assets/embodiments/fr3duo_mobile_task2/camera_sensors.yaml` (overridable
  via `--camera-sensors-yaml`); `camera_publishers.py` cross-checks the two
  at graph-build time and raises on any mismatch.
- **`/isaac/eval_camera/*`** must match the scene camera configs
  (`config/cameras_room.yaml` / `config/cameras_barebone.yaml` — the room
  values mirror the hardcoded setup in
  `scripts/scenes/scene_robot_room_keyboard.py`, which stays authoritative
  there) and the Task 2 evaluation stack (`scripts/evaluation/task2/`).

## Mapping to Task 1 counterparts

| Task 2 | Task 1 counterpart | Relationship |
|---|---|---|
| `scripts/scene_barebone.py`, `scripts/scene_room.py`, `scripts/isaacsim_fr3duo_teleop_bridge_core.py` | `scripts/isaaclab_fr3duo_newton_bridge.py` | Reimplementation for plain Isaac Sim 5.1.0 / PhysX (Isaac Lab + Newton cannot run the deformable pad). Same topics, joint names, defaults; ports task1's swerve-base math, spine keyboard control, and articulation-root fix. Imports task1's `isaac_bridge_constants.py` directly. |
| `scripts/run_isaacsim_teleop.sh` | `scripts/run_isaaclab_newton_teleop.sh` | Same flag conventions; simpler (expects the Isaac Sim container to be already running; adds `--scene room\|barebone`). |
| `docker-compose.yml` (containers `task2_*`) | Same-named services in `task1_isaacsim/docker-compose.yml` (containers `task1_*`) | Same images, commands, env, profiles — only the volume differs: task2 mounts `../task1_isaacsim` at `/workspace`, so the containers execute the Task 1 scripts unmodified. |
| *(no copy — reused via mount)* | `scripts/adapters/keyboard_to_base.py`, `scripts/adapters/gello_to_bridge.py` | Pure topic remappers, task-agnostic. |
| *(no copy — reused via mount)* | `scripts/controllers/ros_joint_republisher.py`, `scripts/controllers/joint_position_controller.py` | Isaac-agnostic rclpy nodes. |
| *(no copy — reused via mount)* | `services/teleop_adapters/`, `services/browser_controller/` | Adapter launcher + web UI (port 8090). |
| *(imported directly)* | `scripts/isaac_bridge_constants.py` | Joint name lists, Robotiq driver/coupled-joint constants, topic layout. |
| `.env.example` | `task1_isaacsim/.env.example` | Same variables and defaults. |
| Robot USD `task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd` | (same file) | Shared asset. Under PhysX the bridge additionally deactivates the OmniGraph graphs embedded in this USD (they crash plain Isaac Sim) and relies on the USD-authored `PhysxMimicJointAPI` for the gripper linkage. |

Task-2-only pieces with no Task 1 counterpart: the scene composition
(`assets/task2_objects/` deformable thermal pad + RAM boards; the robot room
via `scripts/scenes/scene_robot_room_keyboard.py`), PhysX GPU-dynamics
setup, the `/isaac/eval_camera/*` publishers for the Task 2 evaluation
stack, and the whole recording pipeline (`scripts/recording/`,
`services/recording/`, `config/topics.yaml`).

## Configuration

### `config/topics.yaml`

Topic contract, see above. Resolved relative to `topics.py`
(`<task2_isaacsim>/config/topics.yaml`), which works under both container
mounts. Missing file or key → immediate startup error naming the path/keys.

### Camera configs

- `assets/embodiments/fr3duo_mobile_task2/camera_sensors.yaml` — the robot
  cameras (head + wrists). `camera_publishers.py` consumes `namespace`,
  `frame_id`, `render_resolution`, and `prim_path_tokens` (substrings that
  locate the Camera prim authored in the robot USD) per entry; select
  another file with `--camera-sensors-yaml`.
- `config/cameras_room.yaml` / `config/cameras_barebone.yaml` — the scene
  cameras (`eval_camera` plus any user-added ones), built by
  `scripts/recording/scene_cameras.py` when `--enable-scene-cameras`
  (implied by `--record`) is set: the Camera prim is created when missing,
  the pose always comes from the yaml, and a `/ROS2_CameraGraphs/<name>`
  graph already built by the scene (the room's eval camera) is left
  untouched. Select another file with `--scene-cameras-config`; see the
  recipes below for the per-entry schema.

### `services/recording/recording.yaml`

Recorder defaults; keys mirror the `record_task2.py` CLI flags. Precedence:

```
argparse defaults  <  recording.yaml (--config)  <  explicit CLI flags
```

- `--config` selects another file (default: the committed one next to the
  script). Through compose, `RECORDER_CONFIG` sets the container-side path;
  `scripts/run_recorder.sh record --config <host path>` translates a host
  path for you (the file must be inside the repo — it is read via the
  `/repo` mount).
- Individual flags still go through `RECORDER_ARGS` (compose) or after `--`
  (`run_recorder.sh`) and win over the YAML.
- Unknown keys, CLI-only keys (`resume`, `resume_version`, `config`), or
  type mismatches (booleans especially) are hard errors.
- `null` means "use the built-in default" (e.g. `eval_module_dir`).

### `.env`

Helper-stack knobs (gripper calibration, adapter selection, controller
mode) — see [.env.example](.env.example). `HOST_UID`/`HOST_GID` make the
recorder write datasets with your ownership; `run_recorder.sh` exports them
automatically (export them yourself first if invoking
`docker compose --profile record` directly).

## Dataset schema

Datasets follow
[`assets/embodiments/fr3duo_mobile_task2/data_contract_recording.yaml`](assets/embodiments/fr3duo_mobile_task2/data_contract_recording.yaml)
(a 20-dim-action extension of the Task 1 `fr3duo_mobile` contract; the spine
target is appended at index 19). The recorder's schema constants
(`ACTION_NAMES`/`STATE_NAMES`, joint names, `GRIPPER_CLOSED_RAD`) currently
mirror that contract in code rather than loading it — keep them in sync when
touching either. Full on-disk layout (LeRobot v3.0 directory structure, meta
files, per-index feature tables, extras sidecar):
[`services/recording/DATASET.md`](services/recording/DATASET.md).

| Stream | Layout |
|---|---|
| `action` (20, float32) | base twist vx,vy,wz (0–2) · absolute arm joint targets left+right (3–16) · gripper open-fraction targets (17–18) · spine height target (19). Post-arbitration applied targets from PhysX, so keyboard-RMPflow, GELLO/ROS, and spine-key commands are captured uniformly. |
| `observation.state` (37, float32) | link8 EE poses left+right, xyz+quat (0–13) · arm joints (14–27) · spine height (28) · gripper open-fractions (29–30) · base odom x,y,yaw (31–33) · base velocity vx,vy,wz (34–36) |
| `observation.images.{head,wrist_left,wrist_right,eval_camera}` | RGB video at the shapes in `config/topics.yaml` |
| `task2_extras/episode_*.npz` | per-frame object world poses, deformed pad vertices (~10 Hz), optional float16 depth, sim/wall timestamps |
| `task2_extras/episodes_task2.jsonl` | per-episode success label (operator-confirmed, IoU auto-suggestion), scene-randomization offsets, frame counts |

Mechanics:

- **repo_id / versioning** — `<hub_namespace>/<repo_name>_vN` (namespace
  set by `hub_namespace`, default `ebim`), written to
  `<output_dir>/<repo_name>_vN/`; every launch starts the next free version,
  `--resume` / `--resume_version N` append instead (refused when `fps` or
  `cameras` differ from the dataset's `meta/info.json`).
- **Pacing** — frames are sampled on `/isaac/clock` simulation time; the deformable
  scene runs below real time, so wall-clock pacing would sample
  non-uniformly.
- **Scratch / crash recovery** — during recording a sibling
  `<dataset>.tmp/` holds pre-encoding PNGs and a per-episode stream of all
  non-image data; deleted on clean exit, kept for salvage after a crash.
- **Validation** — `services/recording/validate_task2_dataset.py <dataset>`
  (host, numpy only) checks schema, timing, and the extras sidecar.

### Visualizing episodes

```bash
# inside the recorder container (run_recorder.sh shell) or any lerobot env;
# --mode distant serves the Rerun web viewer at http://localhost:9090,
# drop it for a local viewer window outside the container:
python -m lerobot.scripts.lerobot_dataset_viz \
    --root task2_isaacsim/dataset/task2_thermalpad_v1 \
    --repo-id ebim/task2_thermalpad_v1 --episode-index 0 --mode distant
```

Quick sync check: the gripper dip in `action[17]` should line up with the
grasp visible in the wrist video, and base twist with odometry motion.

### Recording caveats

- Modalities are latest-message sampled; per-topic transport skew of a sim
  tick or two is inherent to the ROS-topic recorder design.
- Four RGB streams at the 60 Hz render rate are heavy on DDS; if
  `ros2 topic hz` shows drops, raise `--robot-camera-frame-skip` on the sim
  side or lower the recorder `fps`.
- `record_depth` buffers depth in recorder RAM until episode save — keep
  episodes short or reduce `depth_every` / the camera subset.

## Recording service internals

`services/recording/` is the self-contained recorder service:

- `Dockerfile` — `ros:jazzy-ros-base` + ffmpeg + `vision_msgs`/
  `rosgraph_msgs` + `lerobot[dataset,viz]` (CPU torch). Code is **not**
  baked in; the container runs it from the `/repo` bind mount.
- `recording.yaml` — default recorder config (above).
- `record_task2.py` — the recorder node (interactive stdin episode control).
- `validate_task2_dataset.py` — offline dataset checker.

`scripts/run_recorder.sh` wraps the compose service: `record` (default,
foreground + TTY), `build`, `shell`; `--config`, `--resume`,
`--resume-version N`, `--build`; args after `--` are appended to
`RECORDER_ARGS`. The container runs as `HOST_UID:HOST_GID` with `HOME=/tmp`
(the host uid has no passwd entry inside, and HF/rerun need a writable
cache dir).

## Recipes

**Rename / add a topic** — edit `config/topics.yaml`; if the key is new, add
it to `_REQUIRED_KEYS` in `scripts/topics.py` and wire the
consumer. For command topics, also update the Task 1 republisher/browser
services (see Coupled values) — the bridge startup log + `ros2 topic list`
verify the result.

**Add a robot camera** — add the camera to
`assets/embodiments/fr3duo_mobile_task2/camera_sensors.yaml` (namespace,
frame_id, render_resolution, `prim_path_tokens` matching the Camera prim
authored in the robot USD) and a `cameras.robot.<recorder_key>` entry
(namespace, `sensors_key`, shape) in `config/topics.yaml`; the startup
cross-check catches disagreements, and cameras without a contract entry
publish with a warning but are not recorded. The recorder picks contracted
cameras up via `--cameras`/`cameras:` automatically.

**Add a scene camera** — append an entry to `config/cameras_<scene>.yaml`
(prim_path, namespace, frame_id, translation, rotation_xyz_deg,
render_resolution; optional optics and `publish_depth`/`publish_semantic`/
`publish_bbox` flags). The prim is created if the scene did not author it,
the pose always comes from the yaml, and an existing
`/ROS2_CameraGraphs/<name>` graph (e.g. the room's eval camera) is left
untouched. Add a `cameras.*` entry in `config/topics.yaml` plus a
`contract:` key only if the recorder should record it.

**Change recording defaults** — edit `services/recording/recording.yaml`
(one-off runs: pass flags after `--` instead). For a personal variant, copy
the file inside the repo and point `run_recorder.sh record --config` at it.
