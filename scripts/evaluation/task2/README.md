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
ls ~/docker/ebim-challenge/eval-task2/evaluate/

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
${ISAAC_DOCKER_ROOT}/eval-task2/evaluate/    # default: ~/docker/ebim-challenge/eval-task2/evaluate/
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

**Both pads visible** — count pixels in the raw int32 semantic mask using the
live raw-ID map (below) and compare:
- `liner_ratio > 0.9` → `both_liner_dominant` (correct)
- `thermalpad_ratio > 0.9` → `both_thermalpad_dominant` (wrong)
- otherwise → `sideways` (wrong, IoU = 0)

### Three label topics, three ID schemes

Isaac Sim's `ROS2CameraHelper` publishes each annotator's **own** `idToLabels`
map, on its own topic. The three schemes differ by construction and are
mutually incompatible — resolving one stream's IDs through another's map
silently picks the wrong class (the bridge's shared default used to
interleave two of them on one topic before this was split out):

- `semantic_labels` — the segmentation annotator's map. IDs are the raw mask
  pixel values, with `0=BACKGROUND` / `1=UNLABELLED` reserved and scene
  classes from 2. **The class order is assigned per session** (verified: it
  permutes between runs), so this map is parsed live
  (`hints_from_label_payload` in `evaluation.py`) to resolve mask pixels.
- `bbox_2d_tight_labels` — the tight-bbox annotator's map. Classes counted
  from 0 over the classes **visible in the frame** (e.g.
  `{0: liner, 1: thermalpad, 2: board, 3: target}`; a fully occluded class
  is simply absent). Resolves the numeric `class_id` strings in
  `bbox_2d_tight` detections.
- `bbox_2d_loose_labels` — the loose-bbox annotator's map. Classes counted
  from 0 over **all** labelled classes, regardless of visibility. Resolves
  `class_id` strings in `bbox_2d_loose` detections.

Tight bboxes are cropped to an object's visible pixels and dropped entirely
once it's fully occluded; loose bboxes are published regardless of occlusion.
Quoting the [authoritative source](https://forums.developer.nvidia.com/t/occluded-objects-bounding-boxes/222506):
"Loose will always be annotated, regardless of occlusions. Tight should
always be cropped, or removed if fully occluded."

That distinction is why the target is resolved through the loose stream: a **correctly placed pad occludes the target completely** — under
the tight stream alone, that scored `no_target_label`, IoU 0.0, on every
correct placement. Therefore the evaluation service takes the
**target** from `bbox_2d_loose` / `bbox_2d_loose_labels`, while the **pad**,
its **orientation**, and its own bbox still come from the tight stream —
"which pad face is visible" is exactly the occlusion-derived signal that
distinguishes a correctly placed pad (liner up) from an inverted one, so it
must stay occlusion-aware.

A scene predating the loose helper publishes no `bbox_2d_loose` /
`bbox_2d_loose_labels` topics; the service then falls back to resolving the
target through the tight stream too, restoring the old occlusion-degenerate
behaviour above. Scene and service must come from the same checkout.

`SEMANTIC_RAW_ID_NAME_HINTS` in `config.py` is only a **fallback** for when no
`semantic_labels` payload has been received. It only affects the
both-pads-visible tie-break, and a wrong map silently flips that decision.

## Output artifacts

Per `evaluate` call, written to `/output/evaluate/` (timestamped):

- `eval_camera_rgb_<ts>.jpg`
- `eval_camera_depth_<ts>.npy` / `.png`
- `eval_camera_semantic_segmentation_<ts>.npy` (raw int32) / `.png` (colorized)
- `eval_camera_semantic_labels_<ts>.txt` (segmentation annotator's raw-mask ID map)
- `eval_camera_bbox_tight_labels_<ts>.txt` (tight annotator's class-ID map)
- `eval_camera_bbox_loose_labels_<ts>.txt` (loose annotator's class-ID map)
- `eval_camera_iou_<ts>.json` — primary result (`iou_thermalpad_vs_target_current`,
  `is_orientation_correct`, `orientation_case`, areas, `pad_bbox`, `target_bbox`, …)
- `eval_camera_bbox2d_tight_<ts>.json`
- `eval_camera_rgb_bbox2d_tight_<ts>.jpg` (bbox overlay)
- `eval_camera_bbox2d_loose_<ts>.json` / `eval_camera_rgb_bbox2d_loose_<ts>.jpg`
  (bbox overlay) — written only when the scene publishes `bbox_2d_loose`

## ROS2 Topics

Published by the scene's ROS2 bridge graph:

- `image_raw`
- `depth`
- `semantic_segmentation`
- `semantic_labels` (segmentation annotator's raw-mask ID map)
- `bbox_2d_tight`
- `bbox_2d_tight_labels` (tight-bbox annotator's class-ID map)
- `bbox_2d_loose` (loose-bbox annotator's detections, published regardless
  of occlusion; feeds the target bbox)
- `bbox_2d_loose_labels` (loose-bbox annotator's class-ID map)
- `camera_info`

If the scene predates the loose-bbox helper, `bbox_2d_loose` /
`bbox_2d_loose_labels` are absent and the service falls back to resolving
the target through the tight stream too, restoring the old
occlusion-degenerate behavior (see above). If it predates the
labels-topic split, `bbox_2d_tight_labels` is absent and the service falls
back to resolving bbox class IDs via `semantic_labels` — that topic then
interleaves both annotators' maps, so evaluations are unreliable (pre-split
behavior). Run scene and service from the same checkout. Topic names can be
overridden in `config.yaml` or via CLI args to `main.py`.

## Unit Tests

Pure evaluation logic can be unit-tested without ROS (needs only `numpy`):

```bash
python3 scripts/evaluation/task2/tests/test_evaluation.py
```
