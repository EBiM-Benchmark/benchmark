# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))

from keyboard_arm_teleop import (
    ANGULAR_SPEED_RADPS,
    BINDINGS,
    GRIPPER_SPEED_PER_SECOND,
    LINEAR_SPEED_MPS,
    ROTATION_RATE_RADPS,
    SPINE_SPEED_MPS,
    TRANSLATION_RATE_MPS,
    ControlMode,
    KeyboardTeleopMapper,
    control_help,
)
from teleop_commands import PoseDelta


def test_base_mode_preserves_mobile_wasd_qe_and_arrow_mappings():
    mapper = KeyboardTeleopMapper()

    command = mapper.map_keys({"w", "a", "q", "left"}, timestamp=1.0, dt=0.1)

    assert command.base_twist == (
        LINEAR_SPEED_MPS,
        LINEAR_SPEED_MPS,
        ANGULAR_SPEED_RADPS,
    )
    assert command.left_pose == PoseDelta.zero()
    assert command.right_pose == PoseDelta.zero()
    assert command.left_joint_positions is None
    assert command.right_joint_positions is None


def test_right_arrow_is_an_alias_without_doubling_rotation_speed():
    mapper = KeyboardTeleopMapper()

    command = mapper.map_keys({"e", "right"}, timestamp=1.0, dt=0.1)

    assert command.base_twist == (0.0, 0.0, -ANGULAR_SPEED_RADPS)


def test_opposing_base_keys_cancel():
    command = KeyboardTeleopMapper().map_keys(
        {"w", "s", "a", "d", "q", "e"}, timestamp=1.0, dt=0.1
    )

    assert command.base_twist == (0.0, 0.0, 0.0)


def test_mode_keys_select_modes_on_press_edges():
    mapper = KeyboardTeleopMapper()

    mapper.map_keys({"2"}, timestamp=1.0, dt=0.1)
    assert mapper.mode is ControlMode.LEFT_ARM
    mapper.map_keys(set(), timestamp=1.1, dt=0.1)
    mapper.map_keys({"3"}, timestamp=1.2, dt=0.1)
    assert mapper.mode is ControlMode.RIGHT_ARM
    mapper.map_keys(set(), timestamp=1.3, dt=0.1)
    mapper.map_keys({"1"}, timestamp=1.4, dt=0.1)
    assert mapper.mode is ControlMode.BASE


def test_held_mode_key_does_not_retrigger_after_external_mode_change():
    mapper = KeyboardTeleopMapper()

    mapper.map_keys({"2"}, timestamp=1.0, dt=0.1)
    mapper.mode = ControlMode.BASE
    mapper.map_keys({"2"}, timestamp=1.1, dt=0.1)

    assert mapper.mode is ControlMode.BASE


def test_simultaneous_new_mode_keys_stop_without_delayed_mode_selection():
    mapper = KeyboardTeleopMapper()

    ambiguous = mapper.map_keys({"2", "3", "w"}, timestamp=1.0, dt=0.1)
    held_chord = mapper.map_keys({"2", "w"}, timestamp=1.1, dt=0.1)

    assert mapper.mode is ControlMode.BASE
    assert ambiguous.base_twist == (0.0, 0.0, 0.0)
    assert ambiguous.left_pose == PoseDelta.zero()
    assert ambiguous.right_pose == PoseDelta.zero()
    assert held_chord.base_twist == (LINEAR_SPEED_MPS, 0.0, 0.0)


def test_mode_change_frame_does_not_generate_motion():
    mapper = KeyboardTeleopMapper()

    transition = mapper.map_keys({"2", "w"}, timestamp=1.0, dt=0.1)
    motion = mapper.map_keys({"w"}, timestamp=1.1, dt=0.1)

    assert transition.left_pose == PoseDelta.zero()
    assert motion.left_pose.translation == (
        TRANSLATION_RATE_MPS * 0.1,
        0.0,
        0.0,
    )


def test_left_mode_maps_dt_scaled_motion_only_to_left_arm():
    mapper = KeyboardTeleopMapper(mode=ControlMode.LEFT_ARM)

    command = mapper.map_keys(
        {"w", "d", "q", "i", "l", "u"}, timestamp=2.0, dt=0.1
    )

    assert command.base_twist == (0.0, 0.0, 0.0)
    assert command.left_pose.translation == (
        TRANSLATION_RATE_MPS * 0.1,
        -TRANSLATION_RATE_MPS * 0.1,
        TRANSLATION_RATE_MPS * 0.1,
    )
    assert command.left_pose.rotation_rpy == (
        ROTATION_RATE_RADPS * 0.1,
        -ROTATION_RATE_RADPS * 0.1,
        ROTATION_RATE_RADPS * 0.1,
    )
    assert command.right_pose == PoseDelta.zero()


def test_right_mode_maps_dt_scaled_motion_only_to_right_arm():
    mapper = KeyboardTeleopMapper(mode=ControlMode.RIGHT_ARM)

    command = mapper.map_keys({"s", "j", "o"}, timestamp=2.0, dt=0.2)

    assert command.left_pose == PoseDelta.zero()
    assert command.right_pose.translation == (
        -TRANSLATION_RATE_MPS * 0.2,
        0.0,
        0.0,
    )
    assert command.right_pose.rotation_rpy == (
        0.0,
        ROTATION_RATE_RADPS * 0.2,
        -ROTATION_RATE_RADPS * 0.2,
    )


def test_opposing_arm_keys_cancel():
    mapper = KeyboardTeleopMapper(mode=ControlMode.LEFT_ARM)

    command = mapper.map_keys(
        {"w", "s", "a", "d", "q", "e", "i", "k", "j", "l", "u", "o"},
        timestamp=2.0,
        dt=0.1,
    )

    assert command.left_pose == PoseDelta.zero()


@pytest.mark.parametrize(
    ("key", "left", "right"),
    [
        ("z", 1.0, 0.0),
        ("x", -1.0, 0.0),
        ("c", 0.0, 1.0),
        ("v", 0.0, -1.0),
    ],
)
def test_each_gripper_direction_maps_independently(key, left, right):
    command = KeyboardTeleopMapper().map_keys({key}, timestamp=3.0, dt=0.2)

    scale = GRIPPER_SPEED_PER_SECOND * 0.2
    assert command.left_gripper_delta == left * scale
    assert command.right_gripper_delta == right * scale


@pytest.mark.parametrize("key, direction", [("r", 1.0), ("f", -1.0)])
def test_each_spine_direction_maps_independently(key, direction):
    command = KeyboardTeleopMapper().map_keys({key}, timestamp=3.0, dt=0.2)

    assert command.spine_delta == direction * SPINE_SPEED_MPS * 0.2


def test_gripper_and_spine_bindings_are_dt_scaled_and_independent():
    command = KeyboardTeleopMapper().map_keys(
        {"z", "v", "r"}, timestamp=3.0, dt=0.2
    )

    assert command.left_gripper_delta == GRIPPER_SPEED_PER_SECOND * 0.2
    assert command.right_gripper_delta == -GRIPPER_SPEED_PER_SECOND * 0.2
    assert command.spine_delta == SPINE_SPEED_MPS * 0.2


def test_opposing_gripper_and_spine_keys_cancel():
    command = KeyboardTeleopMapper().map_keys(
        {"z", "x", "c", "v", "r", "f"}, timestamp=3.0, dt=0.2
    )

    assert command.left_gripper_delta == 0.0
    assert command.right_gripper_delta == 0.0
    assert command.spine_delta == 0.0


def test_command_metadata_identifies_active_keyboard_source():
    command = KeyboardTeleopMapper().map_keys(set(), timestamp=4.0, dt=0.1)

    assert command.timestamp == 4.0
    assert command.source == "keyboard"
    assert command.active is True


@pytest.mark.parametrize("dt", [-0.1, float("nan"), float("inf")])
def test_invalid_dt_is_rejected_before_mode_state_changes(dt):
    mapper = KeyboardTeleopMapper()

    with pytest.raises(ValueError, match="dt"):
        mapper.map_keys({"2"}, timestamp=4.0, dt=dt)

    assert mapper.mode is ControlMode.BASE


def test_help_is_generated_from_every_declared_binding():
    help_text = control_help()

    for binding in BINDINGS:
        assert binding.key.upper() in help_text
        assert binding.description in help_text
