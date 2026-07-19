# Task 2 dataset structure

Reference for the datasets produced by [`record_task2.py`](record_task2.py),
using default values and `dataset/task2_thermalpad_v1/meta/info.json` as the
worked example. Datasets follow the **LeRobot v3.0** on-disk format, extended
with a Task-2-specific `task2_extras/` sidecar directory.

The authoritative schema definition is the embodiment data contract,
[`assets/embodiments/fr3duo_mobile_task2/data_contract_recording.yaml`](../../assets/embodiments/fr3duo_mobile_task2/data_contract_recording.yaml);
the name lists below are hand-mirrored into `record_task2.py` (`ACTION_NAMES` /
`STATE_NAMES`) and must be kept in sync with it.

## Overview

| Field (from `meta/info.json`) | Value |
|---|---|
| `codebase_version` | `v3.0` (LeRobot dataset format) |
| `robot_type` | `fr3duo_mobile_task2` — dual FR3 arms on a mobile base with a vertical spine lift |
| `fps` | 30 (paced on sim time from `/isaac/clock`, not wall time) |
| `total_episodes` / `total_frames` / `total_tasks` | (grows as episodes are recorded) |
| `splits` | `{"train": "0:n"}` — training and validation splits |
| Video codec | AV1 (`libsvtav1` by default), `yuv420p`, 30 fps, `g=2`, `crf=30`, `preset=12`, decoded with pyav |

## Directory layout

```
task2_thermalpad_v1/
├── data/                                  # frame-aligned tabular data
│   └── chunk-000/
│       ├── file-000.parquet
│       └── file-001.parquet
├── videos/                                # one directory per camera key
│   ├── observation.images.head/chunk-000/file-000.mp4 …
│   ├── observation.images.wrist_left/chunk-000/…
│   ├── observation.images.wrist_right/chunk-000/…
│   └── observation.images.eval_camera/chunk-000/…
├── meta/
│   ├── info.json                          # dataset-level schema + counts (this doc's reference)
│   ├── stats.json                         # global per-feature statistics
│   ├── tasks.parquet                      # task_index -> task string
│   └── episodes/chunk-000/file-00N.parquet  # per-episode index + stats
└── task2_extras/                          # NOT part of the LeRobot spec (see below)
    ├── episode_000000.npz
    ├── episode_000001.npz
    └── episodes_task2.jsonl
```

Paths are generated from the `info.json` templates
`data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet` and
`videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4`. Unlike
LeRobot v2.x there is no one-file-per-episode guarantee: files are rolled by
size (`data_files_size_in_mb: 100`, `video_files_size_in_mb: 200`) and chunks by
count (`chunks_size: 1000`), so a file may contain several episodes. Episode
boundaries live in `meta/episodes/`, not in file names. (In this small example
dataset the mapping happens to be one episode per file.) v3.0 also replaces the
v2.x `episodes.jsonl` / `tasks.jsonl` / `episodes_stats.jsonl` with parquet
metadata.

## `meta/` files

- **`info.json`** — the dataset header: format version, fps, robot type, feature
  schema (dtype / shape / per-dimension names for every column and camera),
  totals, file-rolling limits, and path templates. Read this first when
  consuming a dataset programmatically.
- **`episodes/chunk-NNN/file-NNN.parquet`** — one row per episode: `episode_index`,
  `tasks`, `length`, the `data/` and per-video `chunk_index`/`file_index` it is
  stored in, `dataset_from_index`/`dataset_to_index` (global frame range), the
  `from_timestamp`/`to_timestamp` range of the episode inside each camera's mp4,
  and an embedded `stats/<feature>/{min,max,mean,std,count,q01…q99}` block per
  feature (135 columns total).
- **`tasks.parquet`** — `task_index` → natural-language task. This dataset has a
  single task 0: *"Pick up the thermal pad and place it on the target RAM board."*
- **`stats.json`** — dataset-wide statistics per feature (min/max/mean/std/count
  and quantiles); image stats are per-channel. Used by LeRobot for normalization.

## Frame features (parquet columns)

Each row of a `data/**.parquet` file is one 30 Hz frame:

| Column | dtype | Shape | Meaning |
|---|---|---|---|
| `action` | float32 | (20,) | Applied control targets (below) |
| `observation.state` | float32 | (37,) | Measured robot state (below) |
| `timestamp` | float32 | (1,) | Seconds since episode start (sim time) |
| `frame_index` | int64 | (1,) | Frame number within the episode |
| `episode_index` | int64 | (1,) | Episode number |
| `index` | int64 | (1,) | Global frame number across the dataset |
| `task_index` | int64 | (1,) | Row in `meta/tasks.parquet` |

Camera frames are not parquet columns — they live in `videos/` and are joined by
`timestamp` via the per-episode video timestamp ranges in `meta/episodes/`.

### `action` (20, float32)

Post-arbitration targets actually applied to PhysX, so keyboard/RMPflow, GELLO
and spine-key commands are captured uniformly:

| Index | Name | Meaning |
|---|---|---|
| 0–2 | `base.vx`, `base.vy`, `base.wz` | Base twist command, body frame (m/s, m/s, rad/s) |
| 3–9 | `left_fr3v2_joint1..7.target` | Left arm absolute joint position targets (rad) |
| 10–16 | `right_fr3v2_joint1..7.target` | Right arm absolute joint position targets (rad) |
| 17 | `left_gripper.open_fraction.target` | Left gripper open fraction (0 = closed, 1 = open) |
| 18 | `right_gripper.open_fraction.target` | Right gripper open fraction |
| 19 | `spine.height.target` | Spine lift height target (m) |

### `observation.state` (37, float32)

| Index | Name | Meaning |
|---|---|---|
| 0–6 | `left_ee.{x,y,z,qx,qy,qz,qw}` | Left end-effector pose (`left_fr3v2_link8`), world frame |
| 7–13 | `right_ee.{x,y,z,qx,qy,qz,qw}` | Right end-effector pose (`right_fr3v2_link8`), world frame |
| 14–20 | `left_fr3v2_joint1..7.pos` | Left arm measured joint positions (rad) |
| 21–27 | `right_fr3v2_joint1..7.pos` | Right arm measured joint positions (rad) |
| 28 | `spine.height` | Measured spine height (m) |
| 29–30 | `left/right_gripper.open_fraction` | Measured gripper open fractions |
| 31–33 | `base.odom.{x,y,yaw}` | Base odometry, world frame (m, m, rad) |
| 34–36 | `base.vel.{vx,vy,wz}` | Base velocity, body frame |

## Cameras

Four `dtype: "video"` features, all AV1 / yuv420p / 30 fps:

| Key | Resolution (H×W) | Source |
|---|---|---|
| `observation.images.head` | 720×1280 | Head ZED Mini |
| `observation.images.wrist_left` | 480×848 | Left wrist D405 |
| `observation.images.wrist_right` | 480×848 | Right wrist D405 |
| `observation.images.eval_camera` | 720×1280 | Static scene evaluation camera |

The `shape` in `info.json` is `[height, width, channels]`; decoded frames are
RGB. With `--rgb-vcodec` the recorder can write H.264/HEVC (e.g. `h264_nvenc`)
instead of AV1 — the actual codec is always recorded in each feature's
`info` block, and mixed-codec datasets are refused on `--resume`.

## `task2_extras/` sidecar

Task-2-specific ground truth that does not fit the LeRobot schema. Standard
LeRobot tooling ignores this directory.

**`episode_{episode_index:06d}.npz`** (per episode, `T` = frame count):

| Key | Shape / dtype | Meaning |
|---|---|---|
| `sim_time` | (T,) float64 | Sim-clock stamp of each frame |
| `wall_time_ns` | (T,) int64 | Wall-clock stamp of each frame |
| `object_poses` | (T, 6, 7) float32 | World pose (x, y, z, qx, qy, qz, qw) of each tracked object per frame |
| `object_names` | (6,) str | Row order of `object_poses`: `board_0`, `board_1`, `board_2`, `board_target`, `thermalpad`, `thermalpad_base` |
| `pad_points` | (S, 2004, 3) float32 | Deformed thermal-pad mesh vertices, snapshotted at ~10 Hz (`S` snapshots) |
| `pad_sim_time` | (S,) float64 | Sim-clock stamp of each pad snapshot |

**`episodes_task2.jsonl`** — one JSON record per episode:

| Field | Meaning |
|---|---|
| `episode_index`, `frames`, `task` | Mirror of the LeRobot metadata |
| `success` | Operator-confirmed success flag (chosen at save time) |
| `success_suggestion` | Automatic hint: `iou_thermalpad_vs_target_current`, `is_orientation_correct`, `orientation_case` |
| `dropped_stale_frames` | Frames skipped because a camera/state message was stale |
| `encoder_dropped_frames` | Streaming-encoder drops (`null` when streaming encoding is off) |
| `fps_sim` | Recording rate in sim time (30) |
| `sim_time_start`, `sim_time_end`, `wall_time_saved` | Episode timing |
| `scene_reset_events` | Scene resets during the episode, with `sim_time`, `randomized`, and randomization `offsets` |
| `extras_file` | The npz file for this episode |
| `pad_snapshots` | Number of pad mesh snapshots (`S`) |
| `depth_frames` | Per-camera depth frame counts (empty when depth recording is off) |

## Reading the data

```python
import pyarrow.parquet as pq
import numpy as np

root = "task2_isaacsim/dataset/task2_thermalpad_v1"
frames = pq.read_table(f"{root}/data/chunk-000/file-000.parquet").to_pandas()
action = np.stack(frames["action"])            # (T, 20)
state = np.stack(frames["observation.state"])  # (T, 37)

extras = np.load(f"{root}/task2_extras/episode_000000.npz", allow_pickle=True)
```

Or via LeRobot (handles video decoding and episode indexing):

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset("ebim/task2_thermalpad", root=root)
```

Sanity-check a dataset offline (host, numpy only) with
[`validate_task2_dataset.py`](validate_task2_dataset.py), and see
[`../../PIPELINE_REF.md`](../../PIPELINE_REF.md) for recording mechanics
(versioning, crash recovery, sim-time pacing).
