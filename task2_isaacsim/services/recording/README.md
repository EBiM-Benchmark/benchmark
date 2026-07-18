# Task 2 recording service

Self-contained LeRobot demonstration recorder for Task 2, run as the
`lerobot_recorder` docker compose service (profile `record`) defined in
[../../docker-compose.yml](../../docker-compose.yml).

| File | Purpose |
|---|---|
| `record_task2.py` | Interactive recorder node — subscribes to the Isaac Sim recording topics and writes a LeRobot dataset + `task2_extras/` sidecar |
| `recording.yaml` | Default recorder settings (repo name, fps, cameras, …); precedence: argparse defaults < this file < CLI flags |
| `validate_task2_dataset.py` | Offline dataset sanity checker (host, numpy only) |
| `Dockerfile` | `ros:jazzy-ros-base` + ffmpeg + `lerobot[dataset,viz]`; code is not baked in — the container runs it from the `/repo` bind mount |

Launch through [../../scripts/run_recorder.sh](../../scripts/run_recorder.sh):

```bash
task2_isaacsim/scripts/run_recorder.sh                       # defaults
task2_isaacsim/scripts/run_recorder.sh record --resume       # append to latest version
task2_isaacsim/scripts/run_recorder.sh record -- --fps 20    # one-off overrides
```

Topic names come from the shared contract
[../../config/topics.yaml](../../config/topics.yaml). Recording quickstart:
[../../README.md](../../README.md#demonstration-recording-imitation-learning);
full pipeline reference: [../../PIPELINE_REF.md](../../PIPELINE_REF.md).
