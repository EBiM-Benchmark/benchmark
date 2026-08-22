# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

import sys
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "scripts" / "common")
)

from teleop_commands import PoseDelta, TeleopCommand, safe_command


def test_pose_delta_zero_has_no_translation_or_rotation():
    assert PoseDelta.zero() == PoseDelta(
        translation=(0.0, 0.0, 0.0),
        rotation_rpy=(0.0, 0.0, 0.0),
    )


def test_command_defaults_to_zero_motion_and_retains_metadata():
    command = TeleopCommand(timestamp=2.5, source="keyboard", active=True)

    assert command.base_twist == (0.0, 0.0, 0.0)
    assert command.left_pose == PoseDelta.zero()
    assert command.right_pose == PoseDelta.zero()
    assert command.left_gripper_delta == 0.0
    assert command.right_gripper_delta == 0.0
    assert command.spine_delta == 0.0
    assert command.left_joint_positions is None
    assert command.right_joint_positions is None
    assert command.timestamp == 2.5
    assert command.source == "keyboard"
    assert command.active is True


def test_command_values_are_immutable():
    command = TeleopCommand(timestamp=1.0, source="keyboard", active=True)

    with pytest.raises(FrozenInstanceError):
        command.active = False


def test_fresh_active_command_is_preserved():
    command = TeleopCommand(
        timestamp=1.0,
        source="keyboard",
        active=True,
        base_twist=(0.5, 0.0, 0.2),
    )

    assert safe_command(command, now=1.4, timeout=0.5) is command


@pytest.mark.parametrize(
    ("active", "now"),
    [(False, 1.1), (True, 1.6), (True, 0.9)],
)
def test_inactive_stale_or_future_command_becomes_safe_stop(active, now):
    command = TeleopCommand(
        timestamp=1.0,
        source="keyboard",
        active=active,
        base_twist=(0.5, 0.0, 0.2),
        left_pose=PoseDelta(translation=(0.1, 0.0, 0.0)),
        left_gripper_delta=0.1,
        spine_delta=0.1,
        left_joint_positions=(0.1,) * 7,
        right_joint_positions=(0.2,) * 7,
    )

    safe = safe_command(command, now=now, timeout=0.5)

    assert safe == TeleopCommand.stop(
        timestamp=command.timestamp,
        source=command.source,
    )
    assert safe.active is False
    assert safe.left_joint_positions is None
    assert safe.right_joint_positions is None


def test_absolute_arm_joint_positions_are_canonical_immutable_tuples():
    command = TeleopCommand(
        timestamp=1.0,
        source="gello",
        active=True,
        left_joint_positions=[0, 1, 2, 3, 4, 5, 6],
    )

    assert command.left_joint_positions == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert isinstance(command.left_joint_positions, tuple)


@pytest.mark.parametrize(
    "positions",
    [(0.0,) * 6, (0.0,) * 8, (0.0, 0.0, 0.0, float("nan"), 0.0, 0.0, 0.0)],
)
def test_absolute_arm_joint_positions_require_seven_finite_values(positions):
    with pytest.raises(ValueError, match="seven finite"):
        TeleopCommand(
            timestamp=1.0,
            source="gello",
            active=True,
            right_joint_positions=positions,
        )


@pytest.mark.parametrize("timeout", [-0.1, float("nan"), float("inf")])
def test_invalid_timeout_is_rejected(timeout):
    command = TeleopCommand(timestamp=1.0, source="keyboard", active=True)

    with pytest.raises(ValueError, match="timeout"):
        safe_command(command, now=1.0, timeout=timeout)
