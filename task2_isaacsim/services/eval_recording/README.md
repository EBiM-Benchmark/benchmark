# Task 2 evaluation audit recorder

Organizer-side passive audit of Task 2 evaluation sessions. Records, per
evaluation episode: one MP4 per camera (`head`, `wrist_left`,
`wrist_right`, `eval_camera`), per-frame timestamp JSONLs, low-rate
ground-truth/joint dumps, official evaluator calls (response +
artifacts), and a manifest with provenance. Output lands in
`services/eval_recording/eval_raw_out/<eval-name>/ep_NNN/` (gitignored;
move the root with `EVAL_RAW_OUT` in `.env`).

Episodes are **segmented automatically at `/isaac/task2/scene_reset`
events** (the policy under evaluation may reset the scene itself; the
event is published *after* the reset completes, so it is a clean
boundary), and a **scene-reset *request* automatically triggers the
official evaluation** on the ending episode's final scene state — the
request arrives seconds before the actual reset wipes the scene, so the
score reflects the last frame of the attempt. The interactive console is
the manual fallback/override.

## Pieces

| File | Role |
|---|---|
| `record_eval_task2.py` | audit recorder node, runs as the `eval_recording` compose service |
| `eval_recording.yaml` | recorder defaults (argparse < YAML < CLI, like `recording.yaml`) |
| `Dockerfile` | recorder image: ros:jazzy + ffmpeg + numpy/yaml + PyAV (libx264) |
| `tests/mock_sim.py` | synthetic contract publisher for Isaac-free testing |
| `tests/smoke_test.sh` | scripted end-to-end smoke test against the mock sim |
| `../../scripts/run_eval_recorder.sh` | launcher: record / build / shell / mock-sim + session provenance |

## Per-session workflow

```bash
# 0. One-time: build the recorder image
task2_isaacsim/scripts/run_eval_recorder.sh build

# 1. Launch the scene with recording topics enabled (repo root)
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh --scene room \
    --no-browser --no-republisher --controller-mode none -- --record

# 2. Bring up the official evaluator (for [e]/auto-evaluate capture)
bash scripts/evaluation/task2/run.sh up

# 3. Attach an audit session (interactive console)
task2_isaacsim/scripts/run_eval_recorder.sh record --eval-name sub_16

# 4. Reset the scene ([1] in the console, or the policy resets it) ->
#    episode starts; run the policy. The next reset request (or [e])
#    scores the attempt; [q] when done (the last attempt, which never
#    sees another reset request, is auto-scored on quit).
```

Console keys: `[1]` publish scene-reset request, `[2]` force-start an
episode, `[3]` stop, `[0]` stop + mark discarded (data kept), `[e]`
official evaluation into the current/last episode, `[s]` status panel,
`[q]` quit. In `-- extra args`: `--manual` disables auto-segmentation,
`--no-auto-evaluate` disables evaluate-on-reset-request,
`--no-evaluate-on-quit` skips the final-state evaluation on quit,
`--record-preamble` also records from session start to the first reset.
Without a TTY (piped stdin) keys are read line-wise and EOF quits, so
sessions can be scripted.

The status line/panel shows sim latch (clock + per-camera liveness/fps),
scene status (sim time, RTF, object count), and reset status (last
event, pending request). The same data is rewritten to
`eval_raw_out/<eval-name>/.status.json` every tick.

## Episode directory

```
ep_001/
├── manifest.json        # started_by/ended_by, sim/wall times, camera
│                        #   stats (encoded/dropped/nonmonotonic/missing),
│                        #   scene_reset_events, discarded flag
├── provenance.json      # tree + git state + config sha256s + image ids
├── videos/<cam>.mp4     # fragmented MP4, PTS = sim time (ms, rebased)
├── frames/<cam>.jsonl   # {i, pts_ms, stamp, recv_wall, clock} per frame
├── dumps/*.jsonl        # object_poses + joint_states (every 6th msg)
└── evaluator/call_NNN/  # evaluator_result.json (trigger: manual |
                         #   reset_request | quit | signal)
                         #   + artifacts/ + calls.jsonl
```

Videos are playable even after a hard kill (fragmented MP4, per-packet
flush + a 1 s `frag_duration` bound — `frag_keyframe` alone would buffer
tens of seconds at render-tied frame rates). A missing `manifest.json`
means the session died mid-episode: the MP4s/JSONLs up to the kill are
still valid (salvage state).

## Notes / failure modes

- **Auto-evaluate ordering.** The evaluation fires on
  `/isaac/task2/scene_reset_request` (from any publisher, including the
  console's own `[1]`), targeting the episode that was recording at
  request time — the subsequent `scene_reset` event may rotate episodes
  while the evaluation is still archiving; the result still lands in the
  correct episode's `evaluator/` dir. A call completing after the
  manifest is written is only counted in `calls.jsonl` (authoritative),
  not `evaluator_calls_at_close`. One evaluation runs at a time;
  concurrent triggers are skipped with a console note.
- **Last episode / quit.** A session's final attempt never sees another
  reset request, so `[q]`/SIGTERM first joins any in-flight evaluation
  (bounded by `evaluate_timeout_s` + 5 s) and then, with
  `evaluate_on_quit` (default on, requires `auto_evaluate`), scores a
  still-recording episode that has no evaluator calls yet. Episodes
  ended with `[3]`/`[0]` before quitting are deliberate stops and are
  not auto-scored. `evaluate_timeout_s` bounds each single Trigger call
  (unrelated to `stream_timeout_s`, the silent-stream warning).
- **Evaluator capture.** `[e]`/auto-evaluate call the same
  `/isaac/eval_camera/evaluate` Trigger service as the official
  `run.sh evaluate` and diff the evaluator output dir (the handler
  writes artifacts before returning, so the diff is race-free). The dir
  is `${ISAAC_DOCKER_ROOT}/eval-task2`, mounted read-only at
  `/eval_out`; all new files are archived. With `eval_out_dir` unset the
  Trigger response is still recorded, only artifact archiving is
  skipped.
- **Provenance.** `.session_provenance.json` (git state, contract
  sha256s, container image ids) is written by `run_eval_recorder.sh`
  *before* the session and copied into each episode as
  `provenance.json` — sessions launched via bare `docker compose run`
  record episodes without it.
- **Missing cameras never block.** A camera silent on the wire is warned
  about and marked `missing` in the manifest (encoders open lazily, so
  no empty MP4s); `optional_cameras` in `eval_recording.yaml`
  additionally allows contract-absent keys to be dropped with a warning.
- **Encoding load.** 720p-class streams at `preset ultrafast`. If the
  sim slows (compare `/isaac/clock` rate with the recorder off), raise
  `frame_stride` in `eval_recording.yaml`.
- **Codec fallback.** The image build asserts PyAV ships `libx264`; if
  that ever fails, set `vcodec: libopenh264` or `libsvtav1`, or pipe
  rawvideo to the apt `ffmpeg`.
- **Offline test rig.** `tests/mock_sim.py` publishes the contract
  synthetically (cameras, clock, resets; mirrors reset_request →
  scene_reset): `run_eval_recorder.sh mock-sim -- --reset-every 25`.
  `tests/smoke_test.sh` runs the full Isaac-free end-to-end check.
