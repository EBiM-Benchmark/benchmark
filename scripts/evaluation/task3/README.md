# Task 3 Evaluation Helpers

This directory contains deterministic development-time grading for the four
Task 3 stages. It is not the official competition scorer and is not yet wired
into the live Task 3 teleoperation loop.

## Pure grading tests

Run from the repository root without Isaac Sim:

```bash
python -B scripts/evaluation/task3/tests/test_grading.py
```

Pass `stage1`, `stage2`, `stage3`, or `stage4` to run one stage.

## Isaac Sim integration validation

The integration runner builds the real room scene and moves scene objects
through deterministic validation motions before calling the pure scoring
helpers. Run it inside the repository's Isaac Lab container:

```bash
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-lab-2.3.2 up -d isaac-lab-2-3-2

docker exec isaac-lab-2-3-2-workshop bash -lc \
  'cd /workspace/EBiM_Challenge && \
   python -B scripts/evaluation/task3/integration_test.py --headless all'
```

Omit `--headless` to keep the Isaac Sim GUI open for inspection. A stage emits
one `STAGE_RESULT` JSON line containing its score and measurements.

## Files

| Path | Purpose |
|---|---|
| `grading.py` | Pure geometry, hold-time, bean-recovery, and cleanup scoring |
| `tests/test_grading.py` | Direct tests without Isaac Sim |
| `integration_test.py` | Live USD/physics validation in Isaac Sim |

Official scoring follows the
[competition rules](https://ebim-benchmark.github.io/competition.html#tasks).
