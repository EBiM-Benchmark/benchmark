# Simulation Development Status

Last updated: 2026-07-07 — updated with each release; every checkmark is verifiable in this repository's history.

## Legend

Capabilities tracked per task/engine:

1. Scene assets complete
2. Robot asset in scene
3. Teleoperation
   - 3.1 keyboard: gripper
   - 3.2 keyboard: base
   - 3.3 GELLO: gripper
   - 3.4 VR: gripper
   - 3.5 foot pedal: base
   - 3.6 keyboard: arm lift
4. Grasping works within contact-force limits
5. Full task run completable via teleoperation
6. Baseline model
7. Real-world dataset (200 episodes, GELLO + keyboard)

✅ = verified working in the current release. This matrix covers what is built and verified; the competition page lists all committed engines per task (e.g., Genesis for Task 2), which may not yet appear here.

Note: evaluation code in this repository (e.g., the Task 2 scoring module and the vendored ManipulationNet client) is a development facilitator; official scoring follows the competition Rulebook v1.2 as published on the website.

## Capability × track matrix

| Capability | Task 1 Isaac Sim | Task 1 MuJoCo | Task 2 Isaac Sim | Task 3 Isaac Sim | Task 3 MuJoCo\* |
|---|:---:|:---:|:---:|:---:|:---:|
| 1. Scene assets complete | — | ✅ | ✅ | ✅ | — |
| 2. Robot asset in scene | ✅ | ✅ | ✅ | ✅ | — |
| 3.1 Teleop — keyboard: gripper | — | ✅ | ✅ | — | — |
| 3.2 Teleop — keyboard: base | — | ✅ | — | — | — |
| 3.3 Teleop — GELLO: gripper | ✅ | — | — | — | — |
| 3.4 Teleop — VR: gripper | — | — | — | — | — |
| 3.5 Teleop — foot pedal: base | ✅ | — | — | — | — |
| 3.6 Teleop — keyboard: arm lift | — | — | — | — | — |
| 4. Grasping within contact-force limits | ✅ | ✅ | ✅ | tracked in [#13](https://github.com/EBiM-Benchmark/benchmark/issues/13) | — |
| 5. Full run completable via teleop | tracked in [#15](https://github.com/EBiM-Benchmark/benchmark/issues/15) | ✅ | ✅ | tracked in [#13](https://github.com/EBiM-Benchmark/benchmark/issues/13) | — |
| 6. Baseline model — tracked in [#16](https://github.com/EBiM-Benchmark/benchmark/issues/16) | — | — | — | — | — |
| 7. Real-world dataset (200 ep) — tracked in [#17](https://github.com/EBiM-Benchmark/benchmark/issues/17) | — | — | — | — | — |

\* Task 3 MuJoCo: Committed engine — environment bring-up in progress; release will be announced on Discord and in this file. Tracked in [#14](https://github.com/EBiM-Benchmark/benchmark/issues/14).

## What can I develop against today?

Task 1 (MuJoCo) and Task 2 (Isaac Sim) are fully usable end-to-end; Task 1 (Isaac Sim) is partially operational (GELLO/pedal teleoperation and grasping verified; end-to-end run pending). Task 3 environments are under active development — the Isaac Sim scene and robot are in place; MuJoCo is a committed engine with environment bring-up in progress, and its release will be announced on Discord and in this file. Baselines and the real-world dataset (200 episodes) release incrementally.
