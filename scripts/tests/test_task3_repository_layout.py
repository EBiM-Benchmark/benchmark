# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Repository-boundary checks for the participant-facing Task 3 layout."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

EXPECTED_TASK3_PATHS = (
    "task3_isaacsim/README.md",
    "task3_isaacsim/.env.example",
    "task3_isaacsim/docker-compose.yml",
    "task3_isaacsim/assets/lula/mobile_fr3_duo/left_arm_description.yaml",
    "task3_isaacsim/assets/lula/mobile_fr3_duo/right_arm_description.yaml",
    "task3_isaacsim/assets/lula/mobile_fr3_duo/left_arm_rmpflow_config.yaml",
    "task3_isaacsim/assets/lula/mobile_fr3_duo/right_arm_rmpflow_config.yaml",
    (
        "task3_isaacsim/assets/lula/mobile_fr3_duo/"
        "mobile_fr3_duo_v0_2_franka_hand.urdf"
    ),
    "task3_isaacsim/scripts/gripper_profiles.py",
    "task3_isaacsim/scripts/run_helper_containers.sh",
    "task3_isaacsim/scripts/run_isaacsim_teleop.sh",
    "task3_isaacsim/scripts/scene_room.py",
    "task3_isaacsim/scripts/scene_robot_room_rmpflow.py",
    "task3_isaacsim/scripts/common/dual_arm_lula.py",
    "task3_isaacsim/scripts/common/keyboard_arm_teleop.py",
    "task3_isaacsim/scripts/common/teleop_commands.py",
    "task3_isaacsim/scripts/common/teleop_targets.py",
    "task3_isaacsim/tests/test_dual_arm_lula.py",
    "task3_isaacsim/tests/test_gripper_profiles.py",
    "task3_isaacsim/tests/test_keyboard_arm_teleop.py",
    "task3_isaacsim/tests/test_launcher.py",
    "task3_isaacsim/tests/test_scene_robot_room_rmpflow.py",
    "task3_isaacsim/tests/test_teleop_commands.py",
    "task3_isaacsim/tests/test_teleop_targets.py",
    "scripts/evaluation/task3/README.md",
)

OLD_TASK3_ONLY_PATHS = (
    "scripts/scenes/scene_robot_room_rmpflow.py",
    "scripts/common/dual_arm_lula.py",
    "scripts/common/keyboard_arm_teleop.py",
    "scripts/common/teleop_commands.py",
    "scripts/common/teleop_targets.py",
    "task3_isaacsim/config/task3_rmpflow/left_arm_rmpflow_config.yaml",
    "task3_isaacsim/config/task3_rmpflow/right_arm_rmpflow_config.yaml",
    "task3_isaacsim/config/task3_teleop/left_arm_description.yaml",
    "task3_isaacsim/config/task3_teleop/right_arm_description.yaml",
    "task3_isaacsim/config/task3_teleop/mobile_fr3_duo_v0_2_franka_hand.urdf",
    "scripts/tests/test_dual_arm_lula.py",
    "scripts/tests/test_keyboard_arm_teleop.py",
    "scripts/tests/test_scene_robot_room_rmpflow.py",
    "scripts/tests/test_teleop_commands.py",
    "scripts/tests/test_teleop_targets.py",
)


def test_task3_participant_files_live_under_task_folder():
    missing = [
        path
        for path in EXPECTED_TASK3_PATHS
        if not (REPO_ROOT / path).is_file()
    ]
    assert not missing, f"Missing Task 3 participant files: {missing}"


def test_old_task3_only_paths_are_removed():
    remaining = [
        path for path in OLD_TASK3_ONLY_PATHS if (REPO_ROOT / path).exists()
    ]
    assert not remaining, (
        f"Task-3-only files remain outside task folder: {remaining}"
    )


def test_shared_room_builder_and_asset_remain_repository_wide():
    shared_scene = REPO_ROOT / "scripts/scenes/scene_robot_room_keyboard.py"
    assert shared_scene.is_file()
    assert (REPO_ROOT / "assets/robot_room.usd").is_file()
