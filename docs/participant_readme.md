<!--
  🚧 DRAFT SKELETON — NOT YET PUBLISHED. Tracking issue: #8 (Participant-grade README for launch).
  Purpose: a participant-facing guide skeleton. Verifiable sections are filled from existing
  repo docs + the competition page. Per-task run commands are EXTRACTED from repo sources and
  marked "(extracted from <file:line> — verify)" — each task owner verifies/corrects the command
  for the code they wrote. (Task 3's end-to-end run is coming soon — no participant command yet; see #13.)
  Prerequisites, repo structure, integration, and final assembly/placement
  (including whether this graduates to the top-level README.md) are @Ju6276's as steward.
  NO run command in this file was invented; anything not extractable is left as an owner-tagged
  <!-- @handle: question --> stub. Do not merge until the stubs are resolved.
-->

# EBiM Benchmark — Participant Guide

> **🚧 Draft skeleton (issue #8).** This is the participant-facing quickstart. Sections below marked
> `(extracted from … — verify)` carry a command pulled from an existing repo file that the task owner
> still needs to confirm; sections with `<!-- @handle: … -->` comments are open questions for that owner.

## What this is

The **EBiM Benchmark** is a globally coordinated benchmark for real-world **embodied bimanual
manipulation**: an open simulation phase combined with cross-continent real-robot validation on
identical **Mobile FR3 Duo** platforms (dual Franka FR3 arms + Robotiq 2F-85 grippers on a
steer-drive mobile base).
<!-- Overview text extracted from https://ebim-benchmark.github.io/competition.html — @Ju6276 verify the one-line framing you want participants to see. -->

This repository is where you **develop and practice** against the competition tasks in simulation.
For the exact capability status of every task/engine before you build, see **[STATUS.md](../STATUS.md)**.
Full rules and official scoring live on the **[competition page](https://ebim-benchmark.github.io/competition.html#tasks)**.

## Competition tasks

<!-- Reused verbatim from the current stopgap table on `main` (README.md:5-13, PR #19). -->

| Task | Engines | Where in this repo | Status |
|---|---|---|---|
| Task 1 — Cable Routing & Plugging | Isaac Sim, MuJoCo | [`task1_isaacsim/`](../task1_isaacsim/), [`task1_mujoco/`](../task1_mujoco/) | see [STATUS.md](../STATUS.md) |
| Task 2 — Deformable Material Handling (Thermal Pad Placement) | Isaac Sim (Genesis committed) | [`task2_isaacsim/`](../task2_isaacsim/), [`assets/task2_objects/`](../assets/task2_objects/), [`scripts/evaluation/task2/`](../scripts/evaluation/task2/) | see [STATUS.md](../STATUS.md) |
| Task 3 — Assisted Living & Feeding | Isaac Sim (MuJoCo committed) | Isaac Sim: [`scripts/scenes/scene_robot_room_keyboard.py`](../scripts/scenes/scene_robot_room_keyboard.py), [`assets/robot_room.usd`](../assets/robot_room.usd); MuJoCo: in development — see [STATUS.md](../STATUS.md) | see [STATUS.md](../STATUS.md) |

Full rules and official scoring are on the competition page:
<https://ebim-benchmark.github.io/competition.html#tasks> . The evaluation code in this repository is a
development facilitator; official scoring follows the rules published there.

## Prerequisites

Requirements differ per task and engine; each task's linked README below carries the authoritative,
task-specific setup. In general:

- **Isaac Sim / Isaac Lab tasks (Task 1 Isaac Sim, Task 2, Task 3)** — Linux host with a supported
  NVIDIA GPU; Docker Engine + Docker Compose v2; NVIDIA Container Toolkit; X11 for GUI; NVIDIA NGC
  access for `nvcr.io/nvidia/isaac-sim` / `isaac-lab` images.
  <!-- (extracted from docs/developer_setup.md:9-14 — verify) -->
- **Task 1 MuJoCo** — Miniconda for native practice (the launcher bootstraps its own env); Docker
  Engine + Compose v2 for the scored evaluation. Scored runs need native Linux (WSL2 cannot reach the
  evidence camera's 25 fps). <!-- (extracted from task1_mujoco/README.md:97-113 — verify) -->

Note: Task 1 (Isaac Sim) uses a **Newton-enabled Isaac Lab overlay**, not the repo's
`docker/isaac-lab-2.3.2` profile — see its README for the container it needs.

<!-- @Ju6276: is there a single "get started" prerequisite path you want participants to land on first
     (e.g. "start with Task 1 MuJoCo — least setup"), or should prerequisites stay per-task as above?
     Also: any registration / competition-signup prerequisite that belongs here before the task setup? -->

## Per-task quickstart

> Each runnable block below starts from a command **extracted from a repo file** (Task 3 is coming
> soon — no participant command yet). The task owner confirms it is the right participant entry point
> and trims/expands as needed. Full setup for every task is in the task's own README, linked at the top
> of each block.

### Task 1 — Cable Routing & Plugging (Isaac Sim)

Full setup + run: **[`task1_isaacsim/README.md`](../task1_isaacsim/README.md)** (one-time asset download,
Newton-enabled Isaac Lab overlay, teleop device layer).

```bash
# Quick start from the repo root, after the one-time setup:
EMBODIMENT=fr3duo_mobile bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --usd-path assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd \
  --controller-mode position --with-keyboard-teleop
```
<!-- (extracted from README.md:33-37; fuller variants at task1_isaacsim/README.md:194-224 — verify) -->
<!-- @QGSQ + @2houyuhang: confirm this is the participant entry command for Task 1 (Isaac Sim), and
     which variant to lead with — keyboard-base + browser-arms (no hardware) vs. the tested GELLO+pedal
     config. Point participants at the minimal path first. -->

### Task 1 — Cable Routing & Plugging (MuJoCo)

Full setup + run: **[`task1_mujoco/README.md`](../task1_mujoco/README.md)** (native practice vs. Docker
scored eval; input modes; controls; troubleshooting).

```bash
cd task1_mujoco
./start.sh              # native teleoperation practice (Windows: double-click start.bat)
./eval.sh sim           # scored ManipulationNet evaluation (Docker), terminal 1
./eval.sh client        # terminal 2: official mnet client
```
<!-- (extracted from README.md:50-55; step-by-step scored walkthrough at task1_mujoco/README.md:126-209 — verify) -->
<!-- @2houyuhang: confirm the two-terminal scored-eval sequence is the participant path, and whether the
     native `./start.sh` practice step should come first in this guide. -->

### Task 2 — Deformable Material Handling / Thermal Pad Placement (Isaac Sim)

Full setup + run: **[`task2_isaacsim/README.md`](../task2_isaacsim/README.md)** (prerequisites,
scenes, input devices, architecture); scoring is in
**[`scripts/evaluation/task2/README.md`](../scripts/evaluation/task2/README.md)**
(eval container lifecycle, IoU metric, artifacts).

```bash
# From the repo root, with the Isaac Sim 5.1.0 container running and the robot
# USD downloaded: launch the teleoperable room scene (publishes the eval-camera
# topics) — keyboard base + browser arms, no special hardware:
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
  --scene room \
  --with-keyboard-teleop

# Drive the pad placement (see task2_isaacsim/README.md for GELLO + pedal), then
# score it — one-time setup, build + start the eval container, evaluate:
bash scripts/evaluation/task2/setup.sh
bash scripts/evaluation/task2/run.sh up
bash scripts/evaluation/task2/run.sh evaluate
```
<!-- (extracted from task2_isaacsim/README.md Quickstart + scripts/evaluation/task2/README.md:13-32) -->
<!-- Resolved on feature/task2_teleop: the teleop path between scene-launch and `run.sh evaluate`
     is task2_isaacsim/ (keyboard/browser or GELLO + foot pedal). -->


### Task 3 — Assisted Living & Feeding (Isaac Sim)

Task overview: four stages — Table Setup → Feed → Bean Recovery → Clean Up (see
[README.md:313-333](../README.md)).

> **🕒 Coming soon.** Task 3 (Isaac Sim): scene composition is available; a teleoperable end-to-end run
> is in development (tracked in [#13](https://github.com/EBiM-Benchmark/benchmark/issues/13)). MuJoCo
> engine in development (tracked in [#14](https://github.com/EBiM-Benchmark/benchmark/issues/14)).
<!-- @leochien1110: is the coming-soon framing correct, or is there a partial runnable path worth documenting now? -->

## Local scoring

The evaluation code in this repository (the Task 2 scoring module and the vendored ManipulationNet
client) is a **development facilitator**; official scoring follows the official rules and scoring
published on the **[competition page](https://ebim-benchmark.github.io/competition.html#tasks)**.
<!-- Facilitator note condensed (near-verbatim) from STATUS.md:25 + README.md:13. -->

## Submitting

Official submission is via the **[EBiM-Benchmark/submissions](https://github.com/EBiM-Benchmark/submissions)**
repository — open a **New Issue** with the *Repository Submission* form:

**➡ [Submit your work](https://github.com/EBiM-Benchmark/submissions/issues/new/choose)**

Your submission is a link to a **public GitHub repo** containing a **Dockerfile** and a **README**
explaining how to run it (source code is not required). Submissions are **open now**; the submission deadline will be extended and is announced on the [competition page](https://ebim-benchmark.github.io/competition.html) and in [Discord](https://discord.gg/pGwRbMRjuH).

> **One clarification for Task 1 developers:** the ManipulationNet performance-submission client used in
> the Task 1 development loop (`eval.sh client`) is **not** the official competition submission — official
> entries go through the EBiM-Benchmark/submissions form above.
<!-- Distinction requested by @Ju6276 on issue #8; framing per EBiM-Benchmark/submissions#1. -->

## Reporting issues & getting help

- **Questions / feature requests / help before submitting:** the EBiM Benchmark **Discord** —
  <https://discord.gg/pGwRbMRjuH>
- **Bugs in this repository:** open an issue with the
  [Bug report form](https://github.com/EBiM-Benchmark/benchmark/issues/new/choose).
- **Rules, timeline, FAQ:** <https://ebim-benchmark.github.io/faq.html>
<!-- Channels reused from .github/ISSUE_TEMPLATE/config.yml and the submissions repo config.yml. -->

## License & citation

This repository is licensed under the **Apache License 2.0** — see [`LICENSE`](../LICENSE) and
[`NOTICE`](../NOTICE) for the full attribution. Original work is © 2026 The EBiM Organizing Committee;
the repository also incorporates robot_lab (Ziqi Fan), Isaac Lab, the franka_isaacSim prototype
(Task 1 Isaac Sim), and the vendored ManipulationNet client (Task 1 MuJoCo).
<!-- Attribution summary aligned with NOTICE on main (post PR #21/#24); NOTICE is authoritative. -->

If you use the EBiM Benchmark in your work, please cite:

```bibtex
@misc{ebim2026,
  title  = {The EBiM Benchmark for Embodied Bimanual Manipulation},
  author = {The EBiM Organizing Committee},
  year   = {2026},
  url    = {https://ebim-benchmark.github.io/}
}
```
