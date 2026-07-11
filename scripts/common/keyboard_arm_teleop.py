# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Shared direct keyboard mapping for Task 3 mobile-base and dual-arm teleop."""

from dataclasses import dataclass
import math

from teleop_commands import PoseDelta, TeleopCommand


LINEAR_SPEED_MPS = 0.25
ANGULAR_SPEED_RADPS = 0.75
TRANSLATION_RATE_MPS = 0.3
ROTATION_RATE_RADPS = 0.8


@dataclass(frozen=True)
class KeyBinding:
    key: str
    description: str


BINDINGS = (
    KeyBinding("w/s", "left arm X+/-"),
    KeyBinding("a/d", "left arm Y+/-"),
    KeyBinding("q/e", "left arm Z-/+"),
    KeyBinding("z/x", "left arm roll +/-"),
    KeyBinding("t/g", "left arm pitch +/-"),
    KeyBinding("c/v", "left arm yaw +/-"),
    KeyBinding("f", "toggle left gripper"),
    KeyBinding("o/l", "right arm X+/-"),
    KeyBinding("k/;", "right arm Y+/-"),
    KeyBinding("i/p", "right arm Z-/+"),
    KeyBinding("n/m", "right arm roll +/-"),
    KeyBinding("u/j", "right arm pitch +/-"),
    KeyBinding(",/.", "right arm yaw +/-"),
    KeyBinding("'", "toggle right gripper"),
    KeyBinding("r", "reset both arm targets"),
    KeyBinding("shift + h/n", "base forward/backward"),
    KeyBinding("shift + b/m", "base left/right"),
    KeyBinding("shift + g/j", "base rotate CCW/CW"),
)


class KeyboardTeleopMapper:
    """Convert held keys into the same Task 3 commands as plain Isaac Sim."""

    def __init__(self) -> None:
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

        keys = {_normalize_key(key) for key in pressed_keys}
        pressed_edges = keys - self._previous_keys
        self._previous_keys = keys
        base_active = "shift" in keys

        left_pose = _pose_delta(
            keys,
            dt,
            translation_keys=("w", "s", "a", "d", "e", "q"),
            rotation_keys=("z", "x", "t", "g", "c", "v"),
            suppress=("g",) if base_active else (),
        )
        right_pose = _pose_delta(
            keys,
            dt,
            translation_keys=("o", "l", "k", ";", "p", "i"),
            rotation_keys=("n", "m", "u", "j", ",", "."),
            suppress=("n", "m", "j") if base_active else (),
        )
        base_twist = _base_twist(keys) if base_active else (0.0, 0.0, 0.0)

        return _command(
            timestamp=timestamp,
            base_twist=base_twist,
            left_pose=left_pose,
            right_pose=right_pose,
            reset_arms="r" in pressed_edges,
            toggle_left_gripper="f" in pressed_edges,
            toggle_right_gripper="'" in pressed_edges,
        )


def control_help() -> str:
    """Return the exact direct-key layout shared with the RMPflow launcher."""
    lines = [
        "+---------------- TASK 3 KEYBOARD CONTROL PANEL ----------------+",
        "| LEFT ARM:  [W/S] X+/- [A/D] Y+/- [Q/E] Z-/+                  |",
        "|            [Z/X] Roll+/- [T/G] Pitch+/- [C/V] Yaw+/- [F] Grip |",
        "| RIGHT ARM: [O/L] X+/- [K/;] Y+/- [I/P] Z-/+                  |",
        "|            [N/M] Roll+/- [U/J] Pitch+/- [,/.] Yaw+/- ['] Grip |",
        "|                                                                |",
        "| Hold [SHIFT] for the mobile base (overlapping arm keys off):  |",
        "| [H/N] Forward/Backward  [B/M] Left/Right  [G/J] Rotate CCW/CW |",
        "| [R] Reset both arm targets                                    |",
        "+----------------------------------------------------------------+",
        "Binding reference:",
    ]
    lines.extend(f"  {binding.key.upper()}: {binding.description}" for binding in BINDINGS)
    return "\n".join(lines)


def _pose_delta(
    keys: set[str],
    dt: float,
    *,
    translation_keys: tuple[str, str, str, str, str, str],
    rotation_keys: tuple[str, str, str, str, str, str],
    suppress: tuple[str, ...],
) -> PoseDelta:
    def axis(positive: str, negative: str) -> float:
        if positive in suppress:
            positive_active = False
        else:
            positive_active = positive in keys
        if negative in suppress:
            negative_active = False
        else:
            negative_active = negative in keys
        return float(positive_active) - float(negative_active)

    return PoseDelta(
        translation=tuple(
            axis(translation_keys[index], translation_keys[index + 1])
            * TRANSLATION_RATE_MPS
            * dt
            for index in range(0, 6, 2)
        ),
        rotation_rpy=tuple(
            axis(rotation_keys[index], rotation_keys[index + 1])
            * ROTATION_RATE_RADPS
            * dt
            for index in range(0, 6, 2)
        ),
    )


def _base_twist(keys: set[str]) -> tuple[float, float, float]:
    return (
        _axis(keys, "h", "n") * LINEAR_SPEED_MPS,
        _axis(keys, "b", "m") * LINEAR_SPEED_MPS,
        _axis(keys, "g", "j") * ANGULAR_SPEED_RADPS,
    )


def _axis(keys: set[str], positive: str, negative: str) -> float:
    return float(positive in keys) - float(negative in keys)


def _normalize_key(key: object) -> str:
    aliases = {
        "left_shift": "shift",
        "right_shift": "shift",
        "shift_l": "shift",
        "shift_r": "shift",
    }
    normalized = str(key).lower()
    return aliases.get(normalized, normalized)


def _command(timestamp: float, **motion) -> TeleopCommand:
    return TeleopCommand(
        timestamp=timestamp,
        source="keyboard",
        active=True,
        **motion,
    )
