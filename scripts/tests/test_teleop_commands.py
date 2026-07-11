from dataclasses import FrozenInstanceError
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))

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
    )

    safe = safe_command(command, now=now, timeout=0.5)

    assert safe == TeleopCommand.stop(
        timestamp=command.timestamp,
        source=command.source,
    )
    assert safe.active is False


@pytest.mark.parametrize("timeout", [-0.1, float("nan"), float("inf")])
def test_invalid_timeout_is_rejected(timeout):
    command = TeleopCommand(timestamp=1.0, source="keyboard", active=True)

    with pytest.raises(ValueError, match="timeout"):
        safe_command(command, now=1.0, timeout=timeout)
