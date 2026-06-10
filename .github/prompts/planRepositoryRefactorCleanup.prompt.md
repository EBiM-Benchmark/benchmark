## Plan: Repository Refactor Cleanup

Refactor the repo into a workshop-focused layout: replace the current Isaac-Lab-template Docker setup with a custom Isaac Sim Docker stack, prepare `franka_description` for submodule use, remove imported legacy `source/` and non-workshop `scripts/` trees, move runnable root scripts into `scripts/`, and rewrite repo-owned docs/comments into English.

**Steps**
1. **Phase 1 — Lock scope**
   - Keep only workshop-facing code.
   - Remove legacy imported areas:
     - `source/robot_lab`
     - `scripts/reinforcement_learning`
     - `scripts/tools`
   - Keep and migrate current root workshop scripts:
     - `scene_robot_keyboard.py`
     - `scene_robot_tables.py`
     - `scene_11_tables.py`
     - `scene_with_table.py`
     - `keyboard_control.py`
     - `tmr_base_control.py`
     - `compose_scene_usd.py`
     - `create_wall_room.py`
     - `inspect_usd.py`
     - `test_table_cutlery.py`
     - `test_table_letter.py`

2. **Phase 2 — Define target layout**
   - Recommended target:
     - `/home/leochien/workspace/IROS_Workshop/assets`
     - `/home/leochien/workspace/IROS_Workshop/docker`
     - `/home/leochien/workspace/IROS_Workshop/docs`
     - `/home/leochien/workspace/IROS_Workshop/scripts/...`
     - `/home/leochien/workspace/IROS_Workshop/third_party/franka_description`
   - Split moved scripts into:
     - `scripts/scenes`
     - `scripts/tools`
     - `scripts/manual_tests`
     - `scripts/robot_learning`
   - First introduce a shared path helper so scripts stop depending on being at repo root.

3. **Phase 3 — Redesign Docker**
   - Replace current Docker design in:
     - `/home/leochien/workspace/IROS_Workshop/docker/Dockerfile`
     - `/home/leochien/workspace/IROS_Workshop/docker/docker-compose.yaml`
     - `/home/leochien/workspace/IROS_Workshop/docker/.env.base`
   - Remove dependency on `source/robot_lab` from the image build.
   - Support:
     - Isaac Sim `nvcr.io/nvidia/isaac-sim:5.1.0` (https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html)
     - Isaac Sim `nvcr.io/nvidia/isaac-sim:6.0.0-dev2` (https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_container.html)
     - Isaac Lab `nvcr.io/nvidia/isaac-lab:2.3.2` (https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html)
   - Ensure:
     - full workspace bind mount into container
     - NVIDIA GPU access
     - host networking where needed
     - persistent cache/config/data/log mounts
     - X11 forwarding for GUI
     - tag the container with workshop-specific `name:tags` e.g. `isaac-sim-5.1.0:iros2026-ebim`
   - Keep tags parameterized in env files.

4. **Phase 4 — Prepare `franka_description` for submodule**
   - Move it behind a stable vendor boundary:
     - `/home/leochien/workspace/IROS_Workshop/third_party/franka_description`
   - Update all hardcoded references to use shared path resolution.
   - Known affected scripts:
     - `/home/leochien/workspace/IROS_Workshop/scene_robot_tables.py`
     - `/home/leochien/workspace/IROS_Workshop/compose_scene_usd.py`
     - `/home/leochien/workspace/IROS_Workshop/keyboard_control.py`
     - `/home/leochien/workspace/IROS_Workshop/scene_robot_keyboard.py`
   - Add clone/init instructions for submodules in docs.
   - Leave the actual remote URL undecided for now.

5. **Phase 5 — Remove legacy package/template content**
   - Delete:
     - `/home/leochien/workspace/IROS_Workshop/source/robot_lab`
     - old non-workshop `scripts/` content
   - Update `/home/leochien/workspace/IROS_Workshop/pyproject.toml` so tooling no longer expects `source` or old script trees.
   - Clean references in `.dockerignore`, docs, and dev configs.

6. **Phase 6 — Move root scripts into `scripts/`**
   - Relocate all runnable workshop scripts into the new `scripts/` structure.
   - Fix imports, especially `tmr_base_control`.
   - Update any `__file__`-relative asset logic.
   - Preserve script names where possible to reduce doc churn.

7. **Phase 7 — Rewrite documentation in English**
   - Rewrite `/home/leochien/workspace/IROS_Workshop/README.md` in English.
   - Cover:
     - project overview
     - repo layout
     - prerequisites
     - Docker quick start for Isaac Sim `5.1.0` and `6.0.0-dev2`
     - X11 notes
     - submodule initialization
     - script usage
   - Rewrite only repo-owned docs/comments.
   - Do not rewrite vendor contents under `franka_description`.

8. **Phase 8 — Validate**
   - Verify compose config for each runtime target.
   - Verify workspace mount works in container.
   - Verify GUI launch through X11 for both Isaac Sim versions.
   - Smoke-test main workshop scripts from new locations.
   - Search for stale references to removed paths.

**Relevant files**
- `/home/leochien/workspace/IROS_Workshop/docker/docker-compose.yaml` — redesign services/profiles around Isaac Sim versions, GPU, host networking, X11, workspace mount.
- `/home/leochien/workspace/IROS_Workshop/docker/Dockerfile` — remove `source/robot_lab` installation logic.
- `/home/leochien/workspace/IROS_Workshop/docker/.env.base` — parameterize image tags and runtime paths.
- `/home/leochien/workspace/IROS_Workshop/README.md` — full English rewrite.
- `/home/leochien/workspace/IROS_Workshop/pyproject.toml` — remove tooling references to deleted trees.
- `/home/leochien/workspace/IROS_Workshop/scene_robot_keyboard.py` — main migration reference.
- `/home/leochien/workspace/IROS_Workshop/scene_robot_tables.py` — update path handling.
- `/home/leochien/workspace/IROS_Workshop/keyboard_control.py` — update imports and asset paths.
- `/home/leochien/workspace/IROS_Workshop/compose_scene_usd.py` — update robot USD path.
- `/home/leochien/workspace/IROS_Workshop/tmr_base_control.py` — preserve importability after move.
- `/home/leochien/workspace/IROS_Workshop/franka_description` — current vendor directory to isolate before submodule conversion.

**Verification**
1. Resolve and inspect compose config for both Isaac Sim targets.
2. Start each container target and confirm repo mount is present.
3. Launch Isaac Sim GUI through X11 and confirm display works.
4. Run smoke tests for the main workshop scripts from `scripts/`.
5. Search for stale references to:
   - `source/robot_lab`
   - `scripts/reinforcement_learning`
   - `scripts/tools`
   - old root script paths
6. Review `README.md` for path/command accuracy.

**Decisions**
- Primary Docker strategy: custom image built on Isaac Sim.
- Isaac Sim targets: `5.1.0` and `6.0.0-dev2`.
- Repo becomes workshop-focused; imported template/package areas are removed.
- Only repo-owned docs/comments are rewritten into English.
- `franka_description` submodule URL remains unspecified for now.

**Further Considerations**
1. Biggest migration risk is path breakage during script moves; path abstraction should happen first.
2. `6.0.0-dev2` should be documented as a temporary documented tag if NVIDIA later publishes a GA tag.
3. One version should likely be presented as the default workflow in the README, with the other as an alternate profile.

The plan is saved in `/memories/session/plan.md`.

If approved, this is ready for handoff to implementation.
