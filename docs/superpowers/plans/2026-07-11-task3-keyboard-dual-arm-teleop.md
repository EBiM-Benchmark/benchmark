# Task 3 Keyboard Dual-Arm Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add keyboard Cartesian control for both Task 3 FR3 arms, grippers, and spine while preserving mobile-base control and a device-independent boundary for future ROS 2/GELLO adapters.

**Architecture:** Pure Python modules define commands, keyboard mapping, pose tracking, and selective target composition. An Isaac Sim-only adapter owns two Lula solvers, while the existing Isaac Lab scene loop applies the composed targets to the articulation.

**Tech Stack:** Python 3.11, dataclasses, NumPy, PyTorch, Isaac Sim Lula motion generation, Isaac Lab articulation APIs, pytest.

---

## File Structure

- Create `scripts/common/teleop_commands.py`: immutable command and pose-delta values plus freshness handling.
- Create `scripts/common/keyboard_arm_teleop.py`: control modes, held-key mapping, rates, and generated help text.
- Create `scripts/common/teleop_targets.py`: bounded Cartesian targets, quaternion math, joint partitioning, and selective position-target composition.
- Create `scripts/common/dual_arm_lula.py`: lazy Isaac Sim imports and independent left/right Lula solver adapter.
- Modify `scripts/scenes/scene_robot_room_keyboard.py`: instantiate components and apply composed targets in the existing loop.
- Create `scripts/tests/test_teleop_commands.py`, `test_keyboard_arm_teleop.py`, `test_teleop_targets.py`, and `test_dual_arm_lula.py`.
- Modify `scripts/tests/test_scene_robot_room_keyboard.py`: regression tests for integration helpers and non-reset behavior.
- Modify `README.md`: document controls, architecture, limitations, and future `teleoperation` adapter boundary.

### Task 1: Unified Commands and Keyboard Mapping

**Files:**
- Create: `scripts/common/teleop_commands.py`
- Create: `scripts/common/keyboard_arm_teleop.py`
- Create: `scripts/tests/test_teleop_commands.py`
- Create: `scripts/tests/test_keyboard_arm_teleop.py`

- [ ] **Step 1: Write failing command-model tests**

Test immutable zero-valued `PoseDelta`/`TeleopCommand`, source and timestamp retention, and `safe_command(command, now, timeout)` returning a stopped command when inactive or stale while preserving a fresh active command.

```python
def test_stale_command_becomes_safe_stop():
    command = TeleopCommand(timestamp=1.0, source="keyboard", active=True,
                            base_twist=(0.5, 0.0, 0.2))
    safe = safe_command(command, now=1.6, timeout=0.5)
    assert safe.base_twist == (0.0, 0.0, 0.0)
    assert safe.left_pose == PoseDelta.zero()
    assert safe.active is False
```

- [ ] **Step 2: Run command tests and verify RED**

Run: `pytest -q scripts/tests/test_teleop_commands.py`
Expected: collection fails because `teleop_commands` does not exist.

- [ ] **Step 3: Implement the minimal command model**

Define frozen `PoseDelta` and `TeleopCommand` dataclasses with tuple defaults, `PoseDelta.zero()`, `TeleopCommand.stop()`, and `safe_command`. Do not import simulator, ROS, NumPy, or PyTorch packages.

- [ ] **Step 4: Run command tests and verify GREEN**

Run: `pytest -q scripts/tests/test_teleop_commands.py`
Expected: all command tests pass.

- [ ] **Step 5: Write failing keyboard-mapper tests**

Cover base mode preserving WASD/QE behavior, mode keys selecting left/right/base, selected-arm translation and rotation, opposing keys cancelling, gripper/spine keys, `dt` rate scaling, and help text containing every declared binding.

```python
def test_left_mode_maps_held_key_to_dt_scaled_left_translation():
    mapper = KeyboardTeleopMapper(mode=ControlMode.LEFT_ARM)
    command = mapper.map_keys({"w"}, timestamp=2.0, dt=0.1)
    assert command.left_pose.translation == (0.03, 0.0, 0.0)
    assert command.right_pose == PoseDelta.zero()
```

- [ ] **Step 6: Run keyboard tests and verify RED**

Run: `pytest -q scripts/tests/test_keyboard_arm_teleop.py`
Expected: collection fails because `keyboard_arm_teleop` does not exist.

- [ ] **Step 7: Implement keyboard mapping**

Define `ControlMode`, binding constants, rates, `KeyboardTeleopMapper`, edge-triggered mode selection, selected-arm pose deltas, gripper/spine deltas, and `control_help()`. Mode changes must not themselves generate motion.

- [ ] **Step 8: Run Task 1 tests and commit**

Run: `pytest -q scripts/tests/test_teleop_commands.py scripts/tests/test_keyboard_arm_teleop.py`
Expected: all tests pass.

```bash
git add scripts/common/teleop_commands.py scripts/common/keyboard_arm_teleop.py scripts/tests/test_teleop_commands.py scripts/tests/test_keyboard_arm_teleop.py
git commit -m "feat: add device-independent keyboard teleop commands"
```

### Task 2: Pose Tracking and Selective Joint Targets

**Files:**
- Create: `scripts/common/teleop_targets.py`
- Create: `scripts/tests/test_teleop_targets.py`

- [ ] **Step 1: Write failing pose-target tests**

Test initialization, local-frame translation, incremental Euler rotation with normalized wxyz quaternion output, workspace clamping, gripper/spine clamping, independent arm updates, and no motion for stopped commands.

```python
def test_pose_tracker_clamps_translation_and_normalizes_rotation():
    tracker = CartesianTargetTracker(initial_targets(), limits=tight_limits())
    targets = tracker.apply(command_with_large_left_delta())
    assert targets.left.position == (0.6, 0.2, 0.8)
    assert math.isclose(sum(v * v for v in targets.left.orientation_wxyz), 1.0)
```

- [ ] **Step 2: Run pose tests and verify RED**

Run: `pytest -q scripts/tests/test_teleop_targets.py -k pose`
Expected: collection fails because `teleop_targets` does not exist.

- [ ] **Step 3: Implement pose tracking and bounds**

Define pure `Pose`, `TeleopTargets`, `TargetLimits`, `CartesianTargetTracker`, quaternion multiply/normalize helpers, and clamp functions. Keep this module free of Isaac Sim and Isaac Lab imports.

- [ ] **Step 4: Run pose tests and verify GREEN**

Run: `pytest -q scripts/tests/test_teleop_targets.py -k pose`
Expected: pose tests pass.

- [ ] **Step 5: Write failing joint partition/composition tests**

Test exact discovery of left/right seven-joint groups, grippers, spine, steering, and drive IDs; diagnostic failure for missing/duplicate joints; selective replacement of only supplied joint groups; and preservation of unrelated current targets.

```python
def test_composer_preserves_unowned_joint_targets():
    groups = discover_joint_groups(FR3_DUO_JOINT_NAMES)
    current = torch.arange(len(FR3_DUO_JOINT_NAMES)).reshape(1, -1).float()
    result = compose_position_targets(current, groups, left_arm=[0.1] * 7)
    assert result[0, groups.right_arm].tolist() == current[0, groups.right_arm].tolist()
```

- [ ] **Step 6: Run composition tests and verify RED**

Run: `pytest -q scripts/tests/test_teleop_targets.py -k 'joint or compose'`
Expected: fails because discovery/composition functions are missing.

- [ ] **Step 7: Implement partitioning and composition**

Define frozen `JointGroups`, `discover_joint_groups(joint_names)`, and `compose_position_targets(current, groups, ...)`. Clone the incoming tensor once and update only explicitly supplied groups.

- [ ] **Step 8: Run Task 2 tests and commit**

Run: `pytest -q scripts/tests/test_teleop_targets.py`
Expected: all target tests pass.

```bash
git add scripts/common/teleop_targets.py scripts/tests/test_teleop_targets.py
git commit -m "feat: add bounded teleop targets and joint composition"
```

### Task 3: Independent Dual-Arm Lula Adapter

**Files:**
- Create: `scripts/common/dual_arm_lula.py`
- Create: `scripts/tests/test_dual_arm_lula.py`
- Copy/adapt configuration from `/tmp/benchmark-archive.git`, branch
  `Robotiq_DEMO`: `DEMO/robot_description/left_arm_description.yaml` and
  `DEMO/robot_description/right_arm_description.yaml`
- Create: `scripts/config/task3_teleop/left_arm_description.yaml`
- Create: `scripts/config/task3_teleop/right_arm_description.yaml`

- [ ] **Step 1: Write failing solver-boundary tests**

Use injected fake per-arm solvers to test independent success/failure, joint-name-indexed output, retention of the last valid result for one failed arm, and project-relative configuration resolution. The tests must import without Isaac Sim installed.

```python
def test_one_arm_failure_does_not_discard_other_arm_solution():
    adapter = DualArmIkAdapter(left_solver=failing_solver(),
                               right_solver=solver_returning([0.2] * 7),
                               joint_names=joint_names())
    result = adapter.solve(left_pose(), right_pose())
    assert result.left == adapter.last_valid.left
    assert result.right == tuple([0.2] * 7)
```

- [ ] **Step 2: Run adapter tests and verify RED**

Run: `pytest -q scripts/tests/test_dual_arm_lula.py`
Expected: collection fails because `dual_arm_lula` does not exist.

- [ ] **Step 3: Implement the injectable adapter**

Define `ArmIkResult`, a small solver protocol, and `DualArmIkAdapter`. Put imports of `omni.isaac.motion_generation`/current Isaac Sim equivalents inside a `create_lula_adapter(...)` factory. Convert internal wxyz quaternions to the order required by the installed solver at the boundary.

- [ ] **Step 4: Add portable solver configuration**

Adapt the archived left/right cspace and fixed-joint descriptions to the current joint/link names. Resolve the YAML and current mobile FR3 Duo URDF/USD-related description paths relative to `Path(__file__)`; reject missing files with explicit paths.

- [ ] **Step 5: Run Task 3 tests and commit**

Run: `pytest -q scripts/tests/test_dual_arm_lula.py`
Expected: all adapter tests pass without importing Kit.

```bash
git add scripts/common/dual_arm_lula.py scripts/config/task3_teleop scripts/tests/test_dual_arm_lula.py
git commit -m "feat: add portable dual-arm Lula IK adapter"
```

### Task 4: Integrate Task 3 Runtime and Documentation

**Files:**
- Modify: `scripts/scenes/scene_robot_room_keyboard.py:1377-1550`
- Modify: `scripts/tests/test_scene_robot_room_keyboard.py`
- Modify: `README.md:314-345`

- [ ] **Step 1: Write failing runtime-integration tests**

Add pure tests for constructing current position targets from measured/default state once, composing left/right/gripper/spine updates without resetting unrelated joints, deriving help from the mapper, and retaining the existing base joint path.

```python
def test_runtime_target_update_does_not_restore_default_arm_pose():
    current = torch.tensor([[0.4, 0.5, 0.6]])
    default = torch.zeros_like(current)
    result = scene_keyboard.prepare_position_targets(current, default,
                                                      initialized=True)
    assert torch.equal(result, current)
```

- [ ] **Step 2: Run integration tests and verify RED**

Run: `pytest -q scripts/tests/test_scene_robot_room_keyboard.py`
Expected: fails because runtime target helpers are missing.

- [ ] **Step 3: Integrate command, target, IK, and composer components**

After scene reset, discover joint groups, initialize persistent position targets and end-effector poses, create the mapper/tracker/Lula adapter, and update the loop in this order: snapshot keys, update mode, map command, enforce freshness, update Cartesian/gripper/spine targets, solve both arms, compose position targets, apply existing base steering/wheel targets, step simulation. Remove the per-frame `default_joint_pos.clone()` reset.

- [ ] **Step 4: Add actionable runtime diagnostics**

Print active mode and generated controls, fail initialization with missing joint/link/config details, and rate-limit per-arm IK warnings. Preserve Escape, `Ctrl+C`, viewer-only mode, bridge setup, heading compensation, and dynamic-bean behavior.

- [ ] **Step 5: Update README**

Document exact controls, Isaac Sim versus Isaac Lab responsibilities, configuration paths, keyboard-only scope, IK failure behavior, and the future adapter contract for `/keyboard/state`, namespaced GELLO joint states, grippers, and pedal state.

- [ ] **Step 6: Run focused and regression verification**

Run:

```bash
pytest -q scripts/tests/test_teleop_commands.py scripts/tests/test_keyboard_arm_teleop.py scripts/tests/test_teleop_targets.py scripts/tests/test_dual_arm_lula.py scripts/tests/test_scene_robot_room_keyboard.py
python -B scripts/evaluation/task3/tests/test_grading.py
```

Expected: all pytest tests and all Task 3 grading groups pass.

- [ ] **Step 7: Run available Isaac Sim integration smoke test**

Run the repository-documented Task 3 headless integration command when the configured Isaac Sim container/runtime is available. Expected: scene and robot initialize, required joint groups and end-effectors resolve, and no position-target overwrite occurs. If unavailable, record the exact environment limitation rather than claiming runtime validation.

- [ ] **Step 8: Commit runtime integration**

```bash
git add scripts/scenes/scene_robot_room_keyboard.py scripts/tests/test_scene_robot_room_keyboard.py README.md
git commit -m "feat: integrate keyboard dual-arm IK into task3"
```

### Task 5: Final Review and Verification

**Files:**
- Review all files changed by Tasks 1-4.

- [ ] **Step 1: Run formatting and static checks used by the repository**

Run the applicable configured Ruff/format commands discovered from repository configuration. Expected: no new errors in changed Python files.

- [ ] **Step 2: Run the complete relevant test set freshly**

Run all commands from Task 4 Step 6 plus any repository test command covering `scripts/common` and the scene launcher. Expected: zero failures.

- [ ] **Step 3: Audit requirements against the design**

Confirm every acceptance criterion in `docs/superpowers/specs/2026-07-11-task3-keyboard-dual-arm-teleop-design.md`, inspect `git diff --check`, and ensure no ROS 2/GELLO runtime dependency was introduced.

- [ ] **Step 4: Request final code review**

Provide reviewers the design, this plan, base SHA, head SHA, and verification output. Fix every Critical or Important issue and repeat the affected tests before completion.
