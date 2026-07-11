import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))

from keyboard_arm_teleop import (
    ANGULAR_SPEED_RADPS,
    BINDINGS,
    LINEAR_SPEED_MPS,
    ROTATION_RATE_RADPS,
    TRANSLATION_RATE_MPS,
    KeyboardTeleopMapper,
    control_help,
)
from teleop_commands import PoseDelta


def test_both_arms_use_the_rmpflow_translation_key_map():
    command = KeyboardTeleopMapper().map_keys(
        {"w", "d", "q", "o", ";", "p"}, timestamp=1.0, dt=0.1
    )

    assert command.base_twist == (0.0, 0.0, 0.0)
    assert command.left_pose.translation == (
        TRANSLATION_RATE_MPS * 0.1,
        -TRANSLATION_RATE_MPS * 0.1,
        -TRANSLATION_RATE_MPS * 0.1,
    )
    assert command.right_pose.translation == (
        TRANSLATION_RATE_MPS * 0.1,
        -TRANSLATION_RATE_MPS * 0.1,
        TRANSLATION_RATE_MPS * 0.1,
    )


def test_both_arms_use_the_rmpflow_rotation_key_map():
    command = KeyboardTeleopMapper().map_keys(
        {"z", "g", "v", "n", "j", "."}, timestamp=1.0, dt=0.1
    )

    expected = ROTATION_RATE_RADPS * 0.1
    assert command.left_pose.rotation_rpy == (expected, -expected, -expected)
    assert command.right_pose.rotation_rpy == (expected, -expected, -expected)


def test_shift_selects_base_controls_and_suppresses_overlapping_arm_keys():
    command = KeyboardTeleopMapper().map_keys(
        {"shift", "h", "b", "g", "n", "m", "j"},
        timestamp=1.0,
        dt=0.1,
    )

    assert command.base_twist == (0.0, 0.0, 0.0)
    assert command.left_pose == PoseDelta.zero()
    assert command.right_pose == PoseDelta.zero()

    command = KeyboardTeleopMapper().map_keys(
        {"shift", "h", "b", "g"}, timestamp=1.0, dt=0.1
    )
    assert command.base_twist == (
        LINEAR_SPEED_MPS,
        LINEAR_SPEED_MPS,
        ANGULAR_SPEED_RADPS,
    )
    assert command.left_pose == PoseDelta.zero()
    assert command.right_pose == PoseDelta.zero()


def test_base_controls_require_shift():
    command = KeyboardTeleopMapper().map_keys(
        {"h", "b"}, timestamp=1.0, dt=0.1
    )

    assert command.base_twist == (0.0, 0.0, 0.0)


def test_r_resets_arm_targets_once_per_key_press():
    mapper = KeyboardTeleopMapper()

    assert mapper.map_keys({"r"}, timestamp=1.0, dt=0.1).reset_arms
    assert not mapper.map_keys({"r"}, timestamp=1.1, dt=0.1).reset_arms
    mapper.map_keys(set(), timestamp=1.2, dt=0.1)
    assert mapper.map_keys({"r"}, timestamp=1.3, dt=0.1).reset_arms


def test_f_and_apostrophe_toggle_the_matching_grippers_once_per_press():
    mapper = KeyboardTeleopMapper()

    command = mapper.map_keys({"f", "'"}, timestamp=1.0, dt=0.1)
    assert command.toggle_left_gripper
    assert command.toggle_right_gripper
    held = mapper.map_keys({"f", "'"}, timestamp=1.1, dt=0.1)
    assert not held.toggle_left_gripper
    assert not held.toggle_right_gripper


def test_base_speed_defaults_match_requested_mobile_robot_rates():
    assert LINEAR_SPEED_MPS == pytest.approx(0.25)
    assert ANGULAR_SPEED_RADPS == pytest.approx(0.75)


def test_command_metadata_identifies_active_keyboard_source():
    command = KeyboardTeleopMapper().map_keys(set(), timestamp=4.0, dt=0.1)

    assert command.timestamp == 4.0
    assert command.source == "keyboard"
    assert command.active is True


@pytest.mark.parametrize("dt", [-0.1, float("nan"), float("inf")])
def test_invalid_dt_is_rejected(dt):
    with pytest.raises(ValueError, match="dt"):
        KeyboardTeleopMapper().map_keys({"w"}, timestamp=4.0, dt=dt)


def test_help_is_generated_from_every_declared_binding():
    help_text = control_help()

    for binding in BINDINGS:
        assert binding.key.upper() in help_text
        assert binding.description in help_text


def test_help_includes_shared_terminal_control_panel_layout():
    help_text = control_help()

    assert "+---------------- TASK 3 KEYBOARD CONTROL PANEL" in help_text
    assert "LEFT ARM" in help_text
    assert "RIGHT ARM" in help_text
    assert "[H/N] Forward/Backward" in help_text
    assert "Hold [SHIFT]" in help_text
