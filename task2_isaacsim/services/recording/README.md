# Task 2 recording service

Self-contained LeRobot demonstration recorder for Task 2, run as the
`lerobot_recorder` docker compose service (profile `record`) defined in
[../../docker-compose.yml](../../docker-compose.yml).

| File | Purpose |
|---|---|
| `record_task2.py` | Interactive recorder node — subscribes to the Isaac Sim recording topics and writes a LeRobot dataset + `task2_extras/` sidecar |
| `DATASET.md` | Structure of the recorded dataset (directory layout, feature tables, extras sidecar) |
| `recording.yaml` | Default recorder settings (repo name, fps, cameras, …); precedence: argparse defaults < this file < CLI flags |
| `validate_task2_dataset.py` | Offline dataset sanity checker (host, numpy only) |
| `Dockerfile` | `ros:jazzy-ros-base` + ffmpeg + `lerobot[dataset,viz]`; code is not baked in — the container runs it from the `/repo` bind mount |

Launch through [../../scripts/run_recorder.sh](../../scripts/run_recorder.sh):

```bash
task2_isaacsim/scripts/run_recorder.sh                       # defaults
task2_isaacsim/scripts/run_recorder.sh record --resume       # append to latest version
task2_isaacsim/scripts/run_recorder.sh record -- --fps 20    # one-off overrides
```

Controls are single keypresses (no Enter; line-buffered fallback without a
TTY). Keybinds are:
| Keys | Action |
|---|---|
| `1` | reset/randomize the scene, then start recording |
| `2` | start recording without reset (episode starts at the current sim time; use after manually reposing the scene) |
| `4` | visualize a saved episode |
| `5` | reset/randomize the scene (same key as the sim window's reset hotkey) |
| `q` | quit (discards the episode) |

While recording:
| Keys | Action |
|---|---|
| `3` | stop + save (confirms the success label, showing the IoU suggestion) |
| `0` | stop + discard |
| `q` | quit (discards the episode) |


**Streaming encoding** (opt-in): Set `streaming_encoding: true` in `recording.yaml` or using `--streaming-encoding` on the CLI.
```bash
run_recorder.sh record -- --streaming-encoding
```

This encodes camera frames on the fly instead of dumping PNGs and
batch-encoding at save, making episode save near-instant. In-progress
temp videos are written as fragmented MP4, so a crashed recording leaves
playable files — swept into `<dataset>.tmp/streaming_leftover/` on the next
start. If the encoder cannot keep up it drops frames (video/state desync);
the recorder warns and asks before saving such an episode.
Raise `--encoder-queue-maxsize` or switch to hardware encoding with `--rgb-vcodec auto`.
To enable GPU access, the current implementation provides an optional NVENC
override [../../docker-compose.nvenc.yml](../../docker-compose.nvenc.yml).
The `run_recorder.sh` adds automatically when a compatible driver is detected
on the host (systems without GPU still record with the CPU default). Keep
`--rgb-vcodec` consistent for the lifetime of a dataset; mismatched resumes
are refused.

Topic names come from the shared contract
[../../config/topics.yaml](../../config/topics.yaml). Recording quickstart:
[../../README.md](../../README.md#demonstration-recording-imitation-learning);
full pipeline reference: [../../PIPELINE_REF.md](../../PIPELINE_REF.md).
