# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Bounded teleoperation targets and mobile FR3 Duo joint composition."""

from dataclasses import dataclass
import math
from typing import Sequence

from teleop_commands import PoseDelta, TeleopCommand, Vector3


QuaternionWxyz = tuple[float, float, float, float]


@dataclass(frozen=True)
class Pose:
    """Cartesian pose with a scalar-first quaternion."""

    position: Vector3
    orientation_wxyz: QuaternionWxyz

    def __post_init__(self) -> None:
        _validate_finite_vector("position", self.position, length=3)
        _validate_finite_vector(
            "orientation_wxyz", self.orientation_wxyz, length=4
        )


@dataclass(frozen=True)
class TeleopTargets:
    """Persistent targets owned by the teleoperation controller."""

    left: Pose
    right: Pose
    left_gripper: float
    right_gripper: float
    spine: float


@dataclass(frozen=True)
class TargetLimits:
    """Shared Cartesian workspace and mechanism position bounds."""

    position_min: Vector3
    position_max: Vector3
    gripper_min: float = 0.0
    gripper_max: float = 0.04
    spine_min: float = 0.0
    spine_max: float = 0.85

    def __post_init__(self) -> None:
        _validate_finite_vector("position_min", self.position_min, length=3)
        _validate_finite_vector("position_max", self.position_max, length=3)
        _validate_bounds("position", self.position_min, self.position_max)
        _validate_bounds(
            "gripper", (self.gripper_min,), (self.gripper_max,)
        )
        _validate_bounds("spine", (self.spine_min,), (self.spine_max,))


class CartesianTargetTracker:
    """Accumulate bounded command deltas into independent arm targets."""

    def __init__(
        self,
        initial: TeleopTargets,
        *,
        limits: TargetLimits,
    ) -> None:
        _validate_finite_scalar("left_gripper", initial.left_gripper)
        _validate_finite_scalar("right_gripper", initial.right_gripper)
        _validate_finite_scalar("spine", initial.spine)
        self._limits = limits
        self._targets = TeleopTargets(
            left=_bounded_pose(initial.left, limits),
            right=_bounded_pose(initial.right, limits),
            left_gripper=_clamp(
                initial.left_gripper,
                limits.gripper_min,
                limits.gripper_max,
            ),
            right_gripper=_clamp(
                initial.right_gripper,
                limits.gripper_min,
                limits.gripper_max,
            ),
            spine=_clamp(initial.spine, limits.spine_min, limits.spine_max),
        )

    @property
    def targets(self) -> TeleopTargets:
        return self._targets

    def apply(self, command: TeleopCommand) -> TeleopTargets:
        """Apply one active command; inactive commands retain all targets."""
        _validate_command(command)
        if not command.active:
            return self._targets

        limits = self._limits
        current = self._targets
        self._targets = TeleopTargets(
            left=_apply_pose_delta(current.left, command.left_pose, limits),
            right=_apply_pose_delta(
                current.right, command.right_pose, limits
            ),
            left_gripper=_clamp(
                current.left_gripper + command.left_gripper_delta,
                limits.gripper_min,
                limits.gripper_max,
            ),
            right_gripper=_clamp(
                current.right_gripper + command.right_gripper_delta,
                limits.gripper_min,
                limits.gripper_max,
            ),
            spine=_clamp(
                current.spine + command.spine_delta,
                limits.spine_min,
                limits.spine_max,
            ),
        )
        return self._targets


LEFT_ARM_JOINTS = tuple(f"left_fr3v2_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"right_fr3v2_joint{i}" for i in range(1, 8))
LEFT_GRIPPER_JOINTS = (
    "left_fr3v2_finger_joint1",
    "left_fr3v2_finger_joint2",
)
RIGHT_GRIPPER_JOINTS = (
    "right_fr3v2_finger_joint1",
    "right_fr3v2_finger_joint2",
)
SPINE_JOINTS = ("franka_spine_vertical_joint",)
STEERING_JOINTS = ("tmrv0_2_joint_0", "tmrv0_2_joint_2")
DRIVE_JOINTS = ("tmrv0_2_joint_1", "tmrv0_2_joint_3")


@dataclass(frozen=True)
class JointGroups:
    """Articulation indices for independently owned robot joint groups."""

    left_arm: tuple[int, ...]
    right_arm: tuple[int, ...]
    left_gripper: tuple[int, ...]
    right_gripper: tuple[int, ...]
    spine: tuple[int, ...]
    steering: tuple[int, ...]
    drive: tuple[int, ...]


def discover_joint_groups(joint_names: Sequence[str]) -> JointGroups:
    """Discover required mobile FR3 Duo joints and reject ambiguity."""
    required_groups = (
        LEFT_ARM_JOINTS,
        RIGHT_ARM_JOINTS,
        LEFT_GRIPPER_JOINTS,
        RIGHT_GRIPPER_JOINTS,
        SPINE_JOINTS,
        STEERING_JOINTS,
        DRIVE_JOINTS,
    )
    required = tuple(name for group in required_groups for name in group)
    indices_by_name: dict[str, list[int]] = {name: [] for name in required}
    for index, name in enumerate(joint_names):
        if name in indices_by_name:
            indices_by_name[name].append(index)

    missing = [name for name in required if not indices_by_name[name]]
    duplicates = [
        name for name in required if len(indices_by_name[name]) > 1
    ]
    if missing or duplicates:
        diagnostics = []
        if missing:
            diagnostics.append(
                f"Missing required mobile FR3 Duo joints: {missing}"
            )
        if duplicates:
            diagnostics.append(
                f"Duplicate required mobile FR3 Duo joints: {duplicates}"
            )
        raise RuntimeError("; ".join(diagnostics))

    def ids(names: tuple[str, ...]) -> tuple[int, ...]:
        return tuple(indices_by_name[name][0] for name in names)

    return JointGroups(
        left_arm=ids(LEFT_ARM_JOINTS),
        right_arm=ids(RIGHT_ARM_JOINTS),
        left_gripper=ids(LEFT_GRIPPER_JOINTS),
        right_gripper=ids(RIGHT_GRIPPER_JOINTS),
        spine=ids(SPINE_JOINTS),
        steering=ids(STEERING_JOINTS),
        drive=ids(DRIVE_JOINTS),
    )


def compose_position_targets(
    current,
    groups: JointGroups,
    *,
    left_arm=None,
    right_arm=None,
    left_gripper=None,
    right_gripper=None,
    spine=None,
):
    """Clone targets and update only explicitly supplied position groups."""
    import torch

    if not isinstance(current, torch.Tensor):
        raise TypeError("current must be a torch.Tensor")
    if not torch.is_floating_point(current):
        raise TypeError("current must be a floating-point torch.Tensor")
    if current.ndim not in (1, 2):
        raise ValueError("current must have shape (joints,) or (envs, joints)")
    if not bool(torch.isfinite(current).all()):
        raise ValueError("current position targets must be finite")

    updates = (
        (groups.left_arm, left_arm, False),
        (groups.right_arm, right_arm, False),
        (groups.left_gripper, left_gripper, True),
        (groups.right_gripper, right_gripper, True),
        (groups.spine, spine, True),
    )
    for _, values, _ in updates:
        if values is None:
            continue
        values_tensor = torch.as_tensor(
            values,
            dtype=current.dtype,
            device=current.device,
        )
        if not bool(torch.isfinite(values_tensor).all()):
            raise ValueError("position target updates must be finite")

    result = current.clone()
    for joint_ids, values, per_environment_scalar in updates:
        if values is not None:
            _assign_group(
                result,
                joint_ids,
                values,
                torch,
                per_environment_scalar=per_environment_scalar,
            )
    return result


def _apply_pose_delta(
    pose: Pose,
    delta: PoseDelta,
    limits: TargetLimits,
) -> Pose:
    position = tuple(
        _clamp(value + change, lower, upper)
        for value, change, lower, upper in zip(
            pose.position,
            delta.translation,
            limits.position_min,
            limits.position_max,
        )
    )
    delta_quaternion = _quaternion_from_rpy(*delta.rotation_rpy)
    orientation = _normalize_quaternion(
        _quaternion_multiply(delta_quaternion, pose.orientation_wxyz)
    )
    return Pose(position=position, orientation_wxyz=orientation)


def _bounded_pose(pose: Pose, limits: TargetLimits) -> Pose:
    return Pose(
        position=tuple(
            _clamp(value, lower, upper)
            for value, lower, upper in zip(
                pose.position, limits.position_min, limits.position_max
            )
        ),
        orientation_wxyz=_normalize_quaternion(pose.orientation_wxyz),
    )


def _quaternion_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> QuaternionWxyz:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _quaternion_multiply(
    first: QuaternionWxyz,
    second: QuaternionWxyz,
) -> QuaternionWxyz:
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _normalize_quaternion(
    quaternion: QuaternionWxyz,
) -> QuaternionWxyz:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("orientation quaternion must have non-zero finite norm")
    return tuple(value / norm for value in quaternion)


def _assign_group(
    result,
    joint_ids,
    values,
    torch_module,
    *,
    per_environment_scalar: bool,
) -> None:
    values_tensor = torch_module.as_tensor(
        values,
        dtype=result.dtype,
        device=result.device,
    )
    count = len(joint_ids)
    if result.ndim == 1:
        if values_tensor.ndim == 0:
            values_tensor = values_tensor.expand(count)
        if tuple(values_tensor.shape) != (count,):
            raise ValueError(
                f"target values must contain {count} joint values"
            )
        result[list(joint_ids)] = values_tensor
        return

    environments = result.shape[0]
    if values_tensor.ndim == 0:
        values_tensor = values_tensor.expand(environments, count)
    elif (
        per_environment_scalar
        and tuple(values_tensor.shape) == (environments,)
    ):
        values_tensor = values_tensor.unsqueeze(1).expand(
            environments, count
        )
    elif tuple(values_tensor.shape) == (count,):
        if count == 1 and environments == 1:
            values_tensor = values_tensor.reshape(1, 1)
        elif count == 1 and environments == values_tensor.shape[0]:
            values_tensor = values_tensor.reshape(environments, 1)
        else:
            values_tensor = values_tensor.unsqueeze(0).expand(
                environments, count
            )
    elif tuple(values_tensor.shape) != (environments, count):
        raise ValueError(
            "target values must be scalar, per-joint, per-environment, "
            f"or have shape ({environments}, {count})"
        )
    result[:, list(joint_ids)] = values_tensor


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _validate_bounds(
    label: str,
    lower: Sequence[float],
    upper: Sequence[float],
) -> None:
    if len(lower) != len(upper) or not all(
        math.isfinite(low) and math.isfinite(high) and low <= high
        for low, high in zip(lower, upper)
    ):
        raise ValueError(f"{label} limits must be finite ordered bounds")


def _validate_finite_vector(
    label: str,
    values: Sequence[float],
    *,
    length: int,
) -> None:
    try:
        actual_length = len(values)
    except TypeError as error:
        raise ValueError(f"{label} must contain exactly {length} values") from error
    if actual_length != length:
        raise ValueError(f"{label} must contain exactly {length} values")
    try:
        finite = all(math.isfinite(value) for value in values)
    except TypeError as error:
        raise ValueError(f"{label} values must be finite numbers") from error
    if not finite:
        raise ValueError(f"{label} values must be finite")


def _validate_finite_scalar(label: str, value: float) -> None:
    try:
        finite = math.isfinite(value)
    except TypeError as error:
        raise ValueError(f"{label} must be finite") from error
    if not finite:
        raise ValueError(f"{label} must be finite")


def _validate_command(command: TeleopCommand) -> None:
    for side, pose_delta in (
        ("left", command.left_pose),
        ("right", command.right_pose),
    ):
        _validate_finite_vector(
            f"command {side} translation",
            pose_delta.translation,
            length=3,
        )
        _validate_finite_vector(
            f"command {side} rotation_rpy",
            pose_delta.rotation_rpy,
            length=3,
        )
    for label, value in (
        ("left_gripper_delta", command.left_gripper_delta),
        ("right_gripper_delta", command.right_gripper_delta),
        ("spine_delta", command.spine_delta),
    ):
        _validate_finite_scalar(f"command {label}", value)
