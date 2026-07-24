# Data collection, training, and policy rollout

This adds a LeRobotDataset recording pipeline and a policy-rollout input
mode on top of the base teleop package, so an operator can record
demonstrations while driving the robot normally, train a policy on the
result with LeRobot's own training scripts, and run that policy back in
the same simulation.

## What gets recorded

Per configured tick, one frame with:

- **`observation.state`** (50-dim): base `x, y, theta, vx, vy, omega`
  (world pose + local-frame velocity), then per arm (left, right)
  `position_rad x7, velocity_rad_s x7, effort_nm x7` (block layout), then
  left/right gripper open-fraction (`0`=closed, `1`=open).
- **`action`** (19-dim, the contract's own explicit order): base target
  `vx, vy, omega`, then left/right arm target joint angles (`x7` each,
  radians), then left/right gripper target open-fraction.
- **`observation.images.<camera_name>`** — one RGB frame per camera in
  `--record-camera`.

This layout matches `task1_isaacsim`'s own
`assets/embodiments/fr3duo_mobile/data_contract.yaml`
(`fr3_duo_mobile_data_contract` v1.0.0), so a recording made here is
structurally comparable to the Isaac Sim side of Task 1 rather than a
MuJoCo-specific raw `qpos`/`ctrl` dump. See `teleop/data_record.py`'s
`build_contract_state`/`build_contract_action` for the exact
implementation.

### Cameras

Default `--record-camera`/`--policy-camera` value is
`head_cam,left_wrist_cam,right_wrist_cam` — the same three cameras named
in `camera_sensors.yaml` (`head_camera`, `left_wrist_camera`,
`right_wrist_camera`). Each camera records at that real device's own
resolution (not a shared default), matching the exact `render_resolution`
`camera_sensors.yaml` specifies:

| camera | device | resolution | vertical FOV |
|---|---|---|---|
| `left_wrist_cam` / `right_wrist_cam` | Intel RealSense D405 | 848x480 | 58 deg |
| `head_cam` | Stereolabs ZED Mini | 1280x720 | 60 deg |

Cameras with no real-hardware entry (`mnet_overhead`, `main`) fall back to
`--record-width`/`--record-height` (`--policy-width`/`--policy-height` for
rollout), default 320x240.

## Recording a dataset

```bash
python main.py --input keyboard --record-dataset /path/to/dataset
```

Drive the robot normally (any input method: keyboard, gamepad, VR). Type
`record` into the **terminal** (not the viewer window) to start an
episode, `record` again to stop and save it. Repeat for as many episodes
as needed; exiting the program finalizes the dataset index.

Needs the `lerobot` package (`pip install lerobot`) — deliberately not in
`environment.yml`, since the base teleop/practice/eval paths never import
it and it pulls a large `torch`/`torchvision` install. Install it
manually in whichever conda env runs `main.py`.

Verifying a recording:

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset(repo_id="local/duo_fr3_cable_management", root="/path/to/dataset")
print(len(ds), ds.meta.total_episodes, ds.meta.features.keys())
```

## Reading state / action / RGB from a recording

`LeRobotDataset` is a standard PyTorch `Dataset` - index it directly, or
wrap it in a `DataLoader` for batched/multi-worker loading. Video frames
are decoded and time-aligned with the tabular state/action data
automatically:

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset(repo_id="local/duo_fr3_cable_management", root="/path/to/dataset")

frame = ds[0]
state = frame["observation.state"]                       # (50,) tensor
action = frame["action"]                                  # (19,) tensor
head_rgb = frame["observation.images.head_cam"]           # (3, 720, 1280) tensor
left_rgb = frame["observation.images.left_wrist_cam"]     # (3, 480, 848) tensor
right_rgb = frame["observation.images.right_wrist_cam"]   # (3, 480, 848) tensor

# batched loading:
from torch.utils.data import DataLoader
loader = DataLoader(ds, batch_size=32, shuffle=True)
batch = next(iter(loader))
```

No dependency on `lerobot`'s own training code is required to read a
dataset this way - only the `LeRobotDataset` class itself. The
underlying storage is also just standard parquet (`data/`) and video
files (`videos/`), readable directly with `pandas`/`pyarrow` and any
video decoder (`opencv-python`, `PyAV`, ...) if a project needs to avoid
the `lerobot` dependency entirely - `meta/info.json` documents the exact
per-feature shapes and dtypes.

## Scripted / non-interactive recording

`teleop/data_record.EpisodeRecorder` doesn't care what drives the robot —
it only needs `maybe_record(data, dt)` called once per physics tick. A
scripted trajectory generator, a leader-arm reader, or any other
programmatic controller can drive the session directly and reuse the same
recorder. See `demo_data_collection.py` for a minimal working example.

## Training

Any LeRobot policy type works directly against a dataset recorded here
(`--policy.type=act`, `diffusion`, `pi0`, `smolvla`, ... — see
`python -m lerobot.scripts.lerobot_train --help` for the full list):

```bash
python -m lerobot.scripts.lerobot_train \
  --dataset.root=/path/to/dataset \
  --dataset.repo_id=local/duo_fr3_cable_management \
  --policy.type=act --policy.push_to_hub=false \
  --output_dir=/path/to/checkpoints --job_name=my_run \
  --batch_size=8 --steps=20000 --wandb.enable=false
```

On Windows, the training script's end-of-run "checkpoints/last" symlink
step can fail with `OSError: [WinError 1314]` unless Developer Mode is
enabled (or the shell runs elevated) - the checkpoint itself
(`checkpoints/<step>/pretrained_model`) is unaffected; reference it by
its numeric step directory instead of `checkpoints/last`.

## Policy rollout

```bash
python main.py --input policy \
  --policy-path /path/to/checkpoints/<step>/pretrained_model \
  --policy-dataset-root /path/to/dataset
```

Or bring your own model (not necessarily trained via LeRobot at all):

```bash
python main.py --input policy --policy-adapter my_policy.py:MyAdapter
```

where `MyAdapter` implements `predict(state, images) -> action` — see
`teleop/run_policy.py`'s module docstring for the exact shapes.

## Alternative data formats

The recorder writes `LeRobotDataset` only (parquet + encoded video, via
the `lerobot` package) - there's no built-in option to write a different
format directly. A completed dataset can be converted afterward (it's
standard parquet + video files, both broadly supported), or
`EpisodeRecorder`'s storage calls can be replaced with a different
backend if a project needs to write a different format from the start.
