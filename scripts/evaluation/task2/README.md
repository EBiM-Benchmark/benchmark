# Task 2 Evaluation Module

Evaluates thermal-pad placement in the task2 scene by computing a
**bounding-box IoU** between the pad (liner / thermalpad) and the target, plus an
orientation check, from the Isaac Sim eval-camera ROS2 streams.

The main `docker/` stack (Isaac Sim/Lab) ships **without ROS2**, so this module
runs in its **own, self-contained ROS2 container** (`ros:jazzy-ros-base`). It is
fully isolated from the Isaac stack and started independently.

## Quick start

```bash
# 0. (in the Isaac Sim container) launch the task2 scene WITH the ROS2 bridge
python scripts/scenes/scene_robot_room_keyboard.py --task task2 --ros2-bridge fastdds
# — or, to actually drive the robot, launch the teleoperable room scene from the
#   host instead (also publishes the /isaac/eval_camera/* topics):
#   bash task2_isaacsim/scripts/run_isaacsim_teleop.sh --scene room
#   (see task2_isaacsim/README.md)

# 1. one-time: create the persistent volume + .env (UID/GID baked in)
bash scripts/evaluation/task2/setup.sh

# 2. build + start the eval container
bash scripts/evaluation/task2/run.sh up
bash scripts/evaluation/task2/run.sh status      # service + container health

# 3. evaluate the current frame (repeat any time; stateless)
bash scripts/evaluation/task2/run.sh evaluate

# 4. inspect artifacts on the host
ls ~/docker/iros-workshop/eval-task2/evaluate/

# stop
bash scripts/evaluation/task2/run.sh down
```

`run.sh evaluate` simply calls the ROS2 service:

```bash
ros2 service call /isaac/eval_camera/evaluate std_srvs/srv/Trigger '{}'
```

You can also trigger it with the helper client (from inside the container):

```bash
docker exec -it eval_task2 bash -lc \
  "source /opt/ros/jazzy/setup.bash && python3 /workspace/scripts/evaluation/task2/client.py"
```

## Layout

| File | Purpose |
|------|---------|
| `main.py` | Entrypoint: load config → start the ROS2 node. |
| `config.py` | Defaults, `SEMANTIC_RAW_ID_NAME_HINTS`, YAML + CLI config loader. |
| `image_utils.py` | Pure ROS-Image ↔ ndarray conversions and bbox helpers. |
| `evaluation.py` | Pure IoU + orientation logic (unit-testable, no ROS). |
| `node.py` | ROS2 node: subscriptions + `Trigger` service orchestration. |
| `client.py` | Thin client to trigger the service from inside the container. |
| `config.yaml` | Topic names, labels, output dir. |
| `Dockerfile`, `docker-compose.yml`, `.env.example` | Container definition. |
| `setup.sh`, `run.sh` | Provision the persistent volume / lifecycle wrapper. |

## Persistent artifacts (volume)

Artifacts persist to a **host bind mount** under `${ISAAC_DOCKER_ROOT}` — the same
convention the main containers use for their caches/data:

```
${ISAAC_DOCKER_ROOT}/eval-task2/evaluate/    # default: ~/docker/iros-workshop/eval-task2/evaluate/
```

Inside the container this is mounted at `/output`, and the service writes to
`/output/evaluate/`. `setup.sh` creates the directory and a `.env` file.

## Evaluation metric

Bounding-box **IoU** between the active pad and the target. Orientation is decided
by which pad surface is visible:

| Liner bbox | Thermalpad bbox | `orientation_case` | `is_orientation_correct` |
|-----------|-----------------|--------------------|--------------------------|
| ✓ | ✗ | `liner_only` | `True` |
| ✗ | ✓ | `thermalpad_only` | `False` |
| ✓ | ✓ | resolved by semantic-mask pixel ratio (below) | depends |
| ✗ | ✗ | `neither_pad_present` | `False` |
| — | — (no target) | `no_target_label` / `no_target_bbox` | `False` |

**Both pads visible** — count pixels in the raw int32 semantic mask using
`SEMANTIC_RAW_ID_NAME_HINTS` and compare:
- `liner_ratio > 0.9` → `both_liner_dominant` (correct)
- `thermalpad_ratio > 0.9` → `both_thermalpad_dominant` (wrong)
- otherwise → `sideways` (wrong, IoU = 0)

### Semantic raw-ID map

The raw semantic-segmentation image is single-channel int32 where each pixel is a class ID. That ID scheme differs from the `semantic_labels` topic (which starts at 0 and omits `unlabeled`), so a fixed hint map is used. For the **current** task2 scene (set in `config.py`):

```python
SEMANTIC_RAW_ID_NAME_HINTS = {
    1: "unlabeled",
    2: "board",
    3: "thermalpad",
    4: "target",
    5: "liner",
}
```

If you change the scene's semantic labeling, update this map — it only affects the both-pads-visible tie-break, and a wrong map silently flips that decision.

## Output artifacts

Per `evaluate` call, written to `/output/evaluate/` (timestamped):

- `eval_camera_rgb_<ts>.jpg`
- `eval_camera_depth_<ts>.npy` / `.png`
- `eval_camera_semantic_segmentation_<ts>.npy` (raw int32) / `.png` (colorized)
- `eval_camera_semantic_labels_<ts>.txt`
- `eval_camera_iou_<ts>.json` — primary result (`iou_thermalpad_vs_target_current`,
  `is_orientation_correct`, `orientation_case`, areas, `pad_bbox`, `target_bbox`, …)
- `eval_camera_bbox2d_tight_<ts>.json`
- `eval_camera_rgb_bbox2d_tight_<ts>.jpg` (bbox overlay)

## ROS2 Topics

Published by the scene's ROS2 bridge graph:

- `image_raw`
- `depth`
- `semantic_segmentation`
- `semantic_labels`
- `bbox_2d_tight`
- `camera_info`

If `ros2 topic list` shows the labels topic as `semantic_segmentation_labels`, rather than `semantic_labels`, override it in `config.yaml` or via CLI args to `main.py`.

## Unit Tests

Pure evaluation logic can be unit-tested without ROS (needs only `numpy`):

```bash
python3 scripts/evaluation/task2/tests/test_evaluation.py
```
