# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Simulator-independent command values for Task 3 teleoperation."""

from dataclasses import dataclass, field
import math


Vector3 = tuple[float, float, float]
ZERO_VECTOR: Vector3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class PoseDelta:
    """Incremental Cartesian translation and roll-pitch-yaw rotation."""

    translation: Vector3 = ZERO_VECTOR
    rotation_rpy: Vector3 = ZERO_VECTOR

    @classmethod
    def zero(cls) -> "PoseDelta":
        return cls()


@dataclass(frozen=True)
class TeleopCommand:
    """One input source's device-independent teleoperation command."""

    timestamp: float
    source: str
    active: bool
    base_twist: Vector3 = ZERO_VECTOR
    left_pose: PoseDelta = field(default_factory=PoseDelta.zero)
    right_pose: PoseDelta = field(default_factory=PoseDelta.zero)
    left_gripper_delta: float = 0.0
    right_gripper_delta: float = 0.0
    spine_delta: float = 0.0

    @classmethod
    def stop(cls, *, timestamp: float, source: str) -> "TeleopCommand":
        """Create an inactive command containing no requested motion."""
        return cls(timestamp=timestamp, source=source, active=False)


def safe_command(
    command: TeleopCommand,
    *,
    now: float,
    timeout: float,
) -> TeleopCommand:
    """Return an inactive stop when a command is inactive or stale."""
    if not math.isfinite(timeout) or timeout < 0.0:
        raise ValueError("timeout must be finite and non-negative")

    age = now - command.timestamp
    if command.active and 0.0 <= age <= timeout:
        return command
    return TeleopCommand.stop(
        timestamp=command.timestamp,
        source=command.source,
    )
