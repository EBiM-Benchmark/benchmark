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

## Frame synchronization

The service evaluates only **stamp-coherent snapshots**: every eval-camera
stream is buffered with its own sim-time stamp (`header.stamp` for the
image/depth/camera_info/bbox streams; the `*_labels` topics are plain
`std_msgs/String` with no ROS header at all, so their stamp comes from the
`time_stamp` field embedded in the JSON payload itself). On each `evaluate`
call, selection anchors on the newest stamp for which every required stream
has an item within `sync_tolerance_s` of that anchor, and binds each label
table to the frame it actually describes — never a "whatever arrived most
recently" latch. See `stream_sync.py` (`EvalStreamSync`) for the selection
algorithm.

Scene resets rebase sim time back to (approximately) `0`. A backward jump on
any one stream's stamp is treated as evidence the whole session just
rebased, so it flushes every stream's buffer at once — pre-reset and
post-reset data can never be paired into the same selection.

If no coherent set appears within `sync_timeout_s`, `evaluate` returns
`success=False` with a per-stream summary (last stamp seen, buffer depth)
and writes only `eval_camera_sync_failure_<ts>.json` — no other artifact is
produced for that call. The service is stateless, so callers just retry.

| Config key | Default | Meaning |
|------------|---------|---------|
| `sync_tolerance_s` | `0.0167` | Max stamp distance from the anchor (~one render period @ 60 Hz). |
| `sync_timeout_s` | `5.0` | How long `evaluate` waits for a coherent set before refusing. |
| `sync_max_age_s` | `0.5` | How far behind the newest required-stream stamp an anchor may sit. |
| `sync_rebase_epsilon_s` | `0.05` | Backward-jump slack before a stamp drop counts as a reset rebase. |
| `sync_image_buffer_len` | `12` | Ring-buffer depth for `image_raw` / `depth` / `semantic_segmentation`. |
| `sync_buffer_len` | `120` | Ring-buffer depth for the label/bbox streams. |

Two degraded modes, both recorded in the result rather than hidden:
- A `*_labels` stream whose payload has never carried a parseable
  `time_stamp` (an older bridge) is attached as the newest arrival instead
  of stamp-matched: `sync.status` becomes `"ok_unstamped_labels"`, and for
  the semantic table specifically, `label_provenance.semantic_table_binding`
  becomes `"arrival_order"`.
- When no usable `semantic_labels` payload is available for the selected
  frame at all — e.g. the brief window right after a scene-reset flush,
  before the first post-reset labels message has arrived — evaluation
  falls back to the static `SEMANTIC_RAW_ID_NAME_HINTS` table and
  `label_provenance.semantic_table_source` becomes `"static_hints"`.

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

Each `*_labels` payload also embeds its own `time_stamp`
(`{"sec": S, "nanosec": N, ...}`) alongside the class map, which is what
[frame synchronization](#frame-synchronization) binds a table to a frame
with — never ROS receive time. The `*_labels` topics are plain
`std_msgs/String` with no ROS header to begin with, and the annotator
republishes its full table on every gated render frame regardless of
whether the mapping actually changed, so "the message that arrived most
recently" is not evidence of which sim tick it describes. Binding by the
embedded stamp instead of receive time is what stops a fresh mask from
silently pairing with a stale mapping.

## Output artifacts

Per `evaluate` call, written to `/output/evaluate/` (timestamped):

- `eval_camera_rgb_<ts>.jpg`
- `eval_camera_depth_<ts>.npy` / `.png`
- `eval_camera_semantic_segmentation_<ts>.npy` (raw int32) / `.png` (colorized)
- `eval_camera_semantic_labels_<ts>.txt` (segmentation annotator's raw-mask ID map)
- `eval_camera_bbox_tight_labels_<ts>.txt` (tight annotator's class-ID map)
- `eval_camera_bbox_loose_labels_<ts>.txt` (loose annotator's class-ID map)
- `eval_camera_iou_<ts>.json` — primary result (`iou_thermalpad_vs_target_current`,
  `is_orientation_correct`, `orientation_case`, areas, `pad_bbox`, `target_bbox`, …).
  Newer fields:
  - `liner_pixels` / `thermalpad_pixels` — raw semantic-mask pixel counts for
    each label; only set in the both-pads-visible case, `null` otherwise.
  - `liner_pixel_ratio` / `thermalpad_pixel_ratio` — those counts as a
    fraction of their sum.
  - `orientation_confidence` — `1.0` when only one pad bbox is visible;
    otherwise the winning (higher) of the two pixel ratios above, or `0.0`
    when no mask was available to compute one.
  - Orientation/placement decoupling: `orientation_case` `sideways` and
    `both_present_no_mask` now carry a real IoU against the **union** of
    the liner and thermalpad bboxes instead of a forced zero.
    `is_orientation_correct` is still `False` for both, so success gating
    (`is_orientation_correct AND iou > min`) is unaffected — orientation
    and placement IoU are measured independently.
  - `sync` — this call's frame-sync diagnostics: `status` (`ok` /
    `ok_unstamped_labels`), `anchor_stamp`, `tolerance_s`, `max_stamp_delta`,
    and `stamps` (all nine stream names from `stream_sync.ALL_STREAMS` →
    their selected stamp or `null`). See
    [Frame synchronization](#frame-synchronization).
  - `label_provenance` — `semantic_table_source` (`dynamic` / `static_hints`),
    `semantic_table_binding` (`stamp` / `arrival_order` / `static`),
    `semantic_table_stamp`, `tight_table_stamp`, `loose_table_stamp`,
    `bbox_table_source` (`tight` / `legacy_semantic_shared`), and
    `semantic_raw_ids` (the resolved liner/thermalpad/target raw mask IDs).
  - `evaluator_version` — short git sha (or an `EVAL_TASK2_VERSION` /
    `EVAL_TASK2_VERSION_BUILD` override, or `"unknown"`) identifying the
    evaluator build that produced this result.
  - `physical_audit` — the ground-truth coverage/spill audit for this
    instant (see [Physical coverage audit](#physical-coverage-audit)), or
    `null`. When `null`, `physical_audit_unavailable_reason` explains why:
    `audit_disabled`, `no_object_poses`, `no_pad_points`,
    `board_target_missing`, `stale_skew`, or `audit_error`.
- `eval_camera_bbox2d_tight_<ts>.json`
- `eval_camera_rgb_bbox2d_tight_<ts>.jpg` (bbox overlay)
- `eval_camera_bbox2d_loose_<ts>.json` / `eval_camera_rgb_bbox2d_loose_<ts>.jpg`
  (bbox overlay) — written only when the scene publishes `bbox_2d_loose`
- `eval_camera_sync_failure_<ts>.json` — written **instead of** every other
  artifact when `evaluate` times out waiting for a coherent set: `wall_time`,
  `sync_config` (the four sync tuning values in effect), and `streams`
  (per-stream `ever_received` / `ever_stamped` / `buffered` / `newest_stamp`).
  See [Frame synchronization](#frame-synchronization).

## Physical coverage audit

An **additive, ground-truth-only** audit alongside the official bbox-IoU
score: `coverage_metrics.py` rasterizes the thermal pad's deformed-mesh
world-space vertices (from `/isaac/task2/pad_points`) onto the target
rectangle expressed in the *board's own local frame* at the moment of the
sample (from `/isaac/task2/object_poses`, `board_target`) — so board
translation or yaw since the scene reset never misaligns the reference, the
way a fixed reset-time rectangle would. It reports `coverage` (footprint ∩
target / target area), `spill` (footprint \ target / footprint area),
`center_error_m`, `yaw_error_deg`, `z_span_m`, `points_in_band` /
`points_total`, and `off_board`.

**It never affects the official score** — `iou_thermalpad_vs_target_current`,
`is_orientation_correct`, and the fields around them are computed exactly as
before; this is a separate, best-effort measurement bolted on for analysis.

Two ways to get it:

- **Live**, in every `evaluate` result's `physical_audit` field (see
  [Output artifacts](#output-artifacts)) — requires the Isaac Sim scene to
  be running with `--record` (that is what publishes
  `/isaac/task2/object_poses` and `/isaac/task2/pad_points`) and the two
  ground-truth samples to agree within `audit_max_skew_s` of each other;
  otherwise `physical_audit` is `null` with a reason (see above).
- **Offline**, over an already-recorded dataset's `task2_extras/`
  ground-truth sidecars — evaluates each episode's *last* pad sample
  against the nearest-in-time `board_target` pose:

  ```bash
  conda run -n pt2 python scripts/evaluation/task2/coverage_from_extras.py \
    --dataset task2_isaacsim/dataset/<name> --out coverage_audit.jsonl
  ```

  Numpy-only, no ROS; writes one JSON record per episode (joined with
  `success` / the recorded IoU suggestion from `episodes_task2.jsonl` when
  present) plus a coverage summary on stdout.

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

Three suites, each self-contained (no ROS needed) and runnable directly from
the repo root with any Python that has `numpy` — the host `pt2` conda env,
or inside the running eval container:

```bash
python3 scripts/evaluation/task2/tests/test_stream_sync.py
python3 scripts/evaluation/task2/tests/test_evaluation.py
python3 scripts/evaluation/task2/tests/test_coverage_metrics.py
```

- `test_stream_sync.py` — coherent-set selection, tolerance/max-age
  rejection, rebase flushing, and the unstamped/static-hints degraded modes
  (pure stdlib, no numpy).
- `test_evaluation.py` — IoU + orientation logic, the sync/provenance
  diagnostics passthrough, and the label-scheme split (numpy only).
- `test_coverage_metrics.py` — the physical coverage/spill audit formulas
  against synthetic golden cases, plus the `task2_extras/` episode loader
  (numpy only).

From inside the running container:

```bash
docker exec -it eval_task2 bash -lc \
  "python3 /workspace/scripts/evaluation/task2/tests/test_evaluation.py"
```
