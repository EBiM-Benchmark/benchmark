# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Held-key mapping for mobile-base and dual-arm teleoperation."""

from dataclasses import dataclass
from enum import Enum
import math

from teleop_commands import PoseDelta, TeleopCommand


LINEAR_SPEED_MPS = 0.5
ANGULAR_SPEED_RADPS = 1.2
TRANSLATION_RATE_MPS = 0.3
ROTATION_RATE_RADPS = 0.8
GRIPPER_SPEED_PER_SECOND = 0.5
SPINE_SPEED_MPS = 0.1


class ControlMode(str, Enum):
    BASE = "base"
    LEFT_ARM = "left_arm"
    RIGHT_ARM = "right_arm"


@dataclass(frozen=True)
class KeyBinding:
    key: str
    action: str
    description: str
    modes: tuple[ControlMode, ...] = ()
    value: float = 0.0


_ARM_MODES = (ControlMode.LEFT_ARM, ControlMode.RIGHT_ARM)
BINDINGS = (
    KeyBinding("1", "mode_base", "select base mode"),
    KeyBinding("2", "mode_left_arm", "select left-arm mode"),
    KeyBinding("3", "mode_right_arm", "select right-arm mode"),
    KeyBinding("w", "x", "base forward", (ControlMode.BASE,), 1.0),
    KeyBinding("s", "x", "base backward", (ControlMode.BASE,), -1.0),
    KeyBinding("a", "y", "base left", (ControlMode.BASE,), 1.0),
    KeyBinding("d", "y", "base right", (ControlMode.BASE,), -1.0),
    KeyBinding("q", "yaw", "base rotate left", (ControlMode.BASE,), 1.0),
    KeyBinding("e", "yaw", "base rotate right", (ControlMode.BASE,), -1.0),
    KeyBinding("left", "yaw", "base rotate left", (ControlMode.BASE,), 1.0),
    KeyBinding("right", "yaw", "base rotate right", (ControlMode.BASE,), -1.0),
    KeyBinding("w", "tx", "end effector +X", _ARM_MODES, 1.0),
    KeyBinding("s", "tx", "end effector -X", _ARM_MODES, -1.0),
    KeyBinding("a", "ty", "end effector +Y", _ARM_MODES, 1.0),
    KeyBinding("d", "ty", "end effector -Y", _ARM_MODES, -1.0),
    KeyBinding("q", "tz", "end effector +Z", _ARM_MODES, 1.0),
    KeyBinding("e", "tz", "end effector -Z", _ARM_MODES, -1.0),
    KeyBinding("i", "roll", "end effector +roll", _ARM_MODES, 1.0),
    KeyBinding("k", "roll", "end effector -roll", _ARM_MODES, -1.0),
    KeyBinding("j", "pitch", "end effector +pitch", _ARM_MODES, 1.0),
    KeyBinding("l", "pitch", "end effector -pitch", _ARM_MODES, -1.0),
    KeyBinding("u", "yaw_rotation", "end effector +yaw", _ARM_MODES, 1.0),
    KeyBinding("o", "yaw_rotation", "end effector -yaw", _ARM_MODES, -1.0),
    KeyBinding("z", "left_gripper", "open left gripper", value=1.0),
    KeyBinding("x", "left_gripper", "close left gripper", value=-1.0),
    KeyBinding("c", "right_gripper", "open right gripper", value=1.0),
    KeyBinding("v", "right_gripper", "close right gripper", value=-1.0),
    KeyBinding("r", "spine", "raise spine", value=1.0),
    KeyBinding("f", "spine", "lower spine", value=-1.0),
)

_MODE_ACTIONS = {
    "mode_base": ControlMode.BASE,
    "mode_left_arm": ControlMode.LEFT_ARM,
    "mode_right_arm": ControlMode.RIGHT_ARM,
}


class KeyboardTeleopMapper:
    """Convert a held-key snapshot into one time-scaled command."""

    def __init__(self, mode: ControlMode = ControlMode.BASE):
        self.mode = mode
        self._previous_keys: set[str] = set()

    def map_keys(
        self,
        pressed_keys: set[str],
        *,
        timestamp: float,
        dt: float,
    ) -> TeleopCommand:
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError("dt must be finite and non-negative")

        keys = {str(key).lower() for key in pressed_keys}
        pressed_edges = keys - self._previous_keys

        selected_modes = [
            _MODE_ACTIONS[binding.action]
            for binding in BINDINGS
            if binding.action in _MODE_ACTIONS
            and binding.key in pressed_edges
        ]
        if len(selected_modes) > 1:
            self._previous_keys = keys
            return _command(timestamp=timestamp)

        self._previous_keys = keys
        mode_changed = bool(selected_modes)
        if mode_changed:
            self.mode = selected_modes[0]

        if mode_changed:
            return _command(timestamp=timestamp)

        actions: dict[str, float] = {}
        for binding in BINDINGS:
            if binding.key not in keys or binding.action in _MODE_ACTIONS:
                continue
            if binding.modes and self.mode not in binding.modes:
                continue
            actions[binding.action] = (
                actions.get(binding.action, 0.0) + binding.value
            )

        base_twist = (0.0, 0.0, 0.0)
        left_pose = PoseDelta.zero()
        right_pose = PoseDelta.zero()
        if self.mode is ControlMode.BASE:
            base_twist = (
                _unit(actions.get("x", 0.0)) * LINEAR_SPEED_MPS,
                _unit(actions.get("y", 0.0)) * LINEAR_SPEED_MPS,
                _unit(actions.get("yaw", 0.0)) * ANGULAR_SPEED_RADPS,
            )
        else:
            pose_delta = PoseDelta(
                translation=(
                    actions.get("tx", 0.0) * TRANSLATION_RATE_MPS * dt,
                    actions.get("ty", 0.0) * TRANSLATION_RATE_MPS * dt,
                    actions.get("tz", 0.0) * TRANSLATION_RATE_MPS * dt,
                ),
                rotation_rpy=(
                    actions.get("roll", 0.0) * ROTATION_RATE_RADPS * dt,
                    actions.get("pitch", 0.0) * ROTATION_RATE_RADPS * dt,
                    actions.get("yaw_rotation", 0.0)
                    * ROTATION_RATE_RADPS
                    * dt,
                ),
            )
            if self.mode is ControlMode.LEFT_ARM:
                left_pose = pose_delta
            else:
                right_pose = pose_delta

        return _command(
            timestamp=timestamp,
            base_twist=base_twist,
            left_pose=left_pose,
            right_pose=right_pose,
            left_gripper_delta=actions.get("left_gripper", 0.0)
            * GRIPPER_SPEED_PER_SECOND
            * dt,
            right_gripper_delta=actions.get("right_gripper", 0.0)
            * GRIPPER_SPEED_PER_SECOND
            * dt,
            spine_delta=actions.get("spine", 0.0) * SPINE_SPEED_MPS * dt,
        )


def control_help() -> str:
    """Return help generated directly from the active binding table."""
    lines = ["Task 3 keyboard controls:"]
    lines.extend(
        f"  {binding.key.upper()}: {binding.description}"
        for binding in BINDINGS
    )
    return "\n".join(lines)


def _command(timestamp: float, **motion) -> TeleopCommand:
    return TeleopCommand(
        timestamp=timestamp,
        source="keyboard",
        active=True,
        **motion,
    )


def _unit(value: float) -> float:
    return max(-1.0, min(1.0, value))
