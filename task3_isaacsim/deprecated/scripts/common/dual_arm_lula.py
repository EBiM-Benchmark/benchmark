# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Independent dual-arm Lula IK adapter for the mobile FR3 Duo.

The two-solver construction follows EBiM benchmark-archive ``Robotiq_DEMO``
commit 78c28ea.  Isaac Sim imports are intentionally delayed until the runtime
factory is called so this module remains importable in ordinary Python tests.
"""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from types import MappingProxyType
from typing import Any

import numpy as np

LOGGER = logging.getLogger(__name__)

LEFT_ARM_JOINTS = tuple(f"left_fr3v2_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"right_fr3v2_joint{i}" for i in range(1, 8))
LEFT_END_EFFECTOR = "left_fr3v2_hand_tcp"
RIGHT_END_EFFECTOR = "right_fr3v2_hand_tcp"


@dataclass(frozen=True)
class DualArmIKResult:
    """Latest valid targets and current-step success for each arm."""

    left: Mapping[str, float]
    right: Mapping[str, float]
    left_succeeded: bool
    right_succeeded: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "left", MappingProxyType(dict(self.left)))
        object.__setattr__(self, "right", MappingProxyType(dict(self.right)))

    @property
    def combined(self) -> dict[str, float]:
        return {**self.left, **self.right}


@dataclass(frozen=True)
class _RawIKAction:
    joint_positions: np.ndarray
    joint_indices: np.ndarray


class RawLulaArmSolver:
    """Adapt raw Lula FK/IK to one arm using CPU NumPy warm starts."""

    def __init__(
        self,
        lula_solver: Any,
        frame_name: str,
        joint_names: Sequence[str],
        arm_joint_names: Sequence[str],
        current_joint_positions: Callable[[], Any],
    ) -> None:
        self._lula_solver = lula_solver
        self._frame_name = frame_name
        self._joint_names = tuple(joint_names)
        self._arm_indices = np.array(
            [self._joint_names.index(name) for name in arm_joint_names],
            dtype=np.int64,
        )
        self._current_joint_positions = current_joint_positions

    def _warm_start(self) -> np.ndarray:
        full = _finite_vector(
            self._current_joint_positions(),
            len(self._joint_names),
            "current joint positions",
        )
        return np.array(full[self._arm_indices], dtype=np.float64, copy=True)

    def compute_inverse_kinematics(
        self, position: Sequence[float], orientation: Sequence[float] | None
    ) -> tuple[_RawIKAction, bool]:
        target_position = _finite_vector(position, 3, "target position")
        target_orientation = (
            None
            if orientation is None
            else _normalized_quaternion(orientation, "target orientation")
        )
        positions, succeeded = self._lula_solver.compute_inverse_kinematics(
            self._frame_name,
            target_position,
            target_orientation,
            self._warm_start(),
        )
        return (
            _RawIKAction(
                np.asarray(positions, dtype=np.float64),
                self._arm_indices.copy(),
            ),
            bool(succeeded),
        )

    def compute_end_effector_pose(self) -> tuple[np.ndarray, np.ndarray]:
        return self._lula_solver.compute_forward_kinematics(
            self._frame_name, self._warm_start()
        )


class DualArmLulaIK:
    """Solve each arm independently and retain its last valid solution."""

    def __init__(
        self,
        left_solver: Any,
        right_solver: Any,
        joint_names: Sequence[str],
        *,
        left_lula_solver: Any | None = None,
        right_lula_solver: Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        warning_interval: float = 1.0,
    ) -> None:
        self._left_solver = left_solver
        self._right_solver = right_solver
        self._left_lula_solver = left_lula_solver
        self._right_lula_solver = right_lula_solver
        self._joint_names = tuple(joint_names)
        if len(set(self._joint_names)) != len(self._joint_names):
            raise ValueError("Articulation joint names must be unique")
        if not math.isfinite(warning_interval) or warning_interval < 0.0:
            raise ValueError(
                "warning_interval must be finite and non-negative"
            )
        self._monotonic = monotonic
        self._warning_interval = warning_interval
        self._last_warning = {"left": -math.inf, "right": -math.inf}
        self._last_left: dict[str, float] = {}
        self._last_right: dict[str, float] = {}

    def current_end_effector_poses(
        self,
        base_position: Sequence[float] = (0.0, 0.0, 0.0),
        base_orientation_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
        spine_position: float = 0.0,
    ) -> tuple[
        tuple[np.ndarray, np.ndarray],
        tuple[np.ndarray, np.ndarray],
    ]:
        """Read both current world poses as position and normalized wxyz."""
        base_position_array = _finite_vector(base_position, 3, "base_position")
        base_orientation = _normalized_quaternion(
            base_orientation_wxyz, "base_orientation_wxyz"
        )
        spine_position = float(spine_position)
        if not math.isfinite(spine_position):
            raise ValueError("spine_position must be finite")
        self._set_robot_base_poses(base_position_array, base_orientation)
        spine_offset = _rotated_vertical_offset(
            base_orientation, spine_position
        )
        left_position, left_orientation = _current_end_effector_pose(
            self._left_solver
        )
        right_position, right_orientation = _current_end_effector_pose(
            self._right_solver
        )
        return (
            (left_position + spine_offset, left_orientation),
            (right_position + spine_offset, right_orientation),
        )

    def solve(
        self,
        left_position: Sequence[float],
        right_position: Sequence[float],
        left_orientation: Sequence[float] | None = None,
        right_orientation: Sequence[float] | None = None,
        spine_position: float = 0.0,
        base_position: Sequence[float] = (0.0, 0.0, 0.0),
        base_orientation_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    ) -> DualArmIKResult:
        """Compute both IK solutions even when one solver fails.

        ``spine_position`` is measured along the robot base's local vertical
        axis. Its world-space vector is removed from each world target because
        the Lula YAML fixes the vertical spine joint at zero.
        """
        try:
            spine_position = float(spine_position)
        except (TypeError, ValueError):
            raise ValueError("spine_position must be finite") from None
        if not math.isfinite(spine_position):
            raise ValueError("spine_position must be finite")
        base_orientation = _normalized_quaternion(
            base_orientation_wxyz, "base_orientation_wxyz"
        )
        base_position_array = _finite_vector(base_position, 3, "base_position")
        self._set_robot_base_poses(base_position_array, base_orientation)
        spine_offset_world = _rotated_vertical_offset(
            base_orientation, spine_position
        )
        left, left_succeeded = self._solve_arm(
            "left",
            self._left_solver,
            left_position,
            left_orientation,
            LEFT_ARM_JOINTS,
            self._last_left,
            spine_offset_world,
        )
        right, right_succeeded = self._solve_arm(
            "right",
            self._right_solver,
            right_position,
            right_orientation,
            RIGHT_ARM_JOINTS,
            self._last_right,
            spine_offset_world,
        )
        if left_succeeded:
            self._last_left = left
        if right_succeeded:
            self._last_right = right
        return DualArmIKResult(
            left=dict(self._last_left),
            right=dict(self._last_right),
            left_succeeded=left_succeeded,
            right_succeeded=right_succeeded,
        )

    def _set_robot_base_poses(
        self, base_position: np.ndarray, base_orientation: np.ndarray
    ) -> None:
        updated: set[int] = set()
        for solver in (self._left_lula_solver, self._right_lula_solver):
            if solver is None or id(solver) in updated:
                continue
            solver.set_robot_base_pose(
                base_position.copy(), base_orientation.copy()
            )
            updated.add(id(solver))

    def _solve_arm(
        self,
        side: str,
        solver: Any,
        position: Sequence[float],
        orientation: Sequence[float] | None,
        expected_joints: tuple[str, ...],
        previous: Mapping[str, float],
        spine_offset_world: np.ndarray,
    ) -> tuple[dict[str, float], bool]:
        try:
            compensated_position = (
                _finite_vector(position, 3, "target position")
                - spine_offset_world
            )
            normalized_orientation = (
                None
                if orientation is None
                else _normalized_quaternion(orientation, "target orientation")
            )
            action, succeeded = solver.compute_inverse_kinematics(
                compensated_position, normalized_orientation
            )
            if not succeeded:
                raise ValueError("solver reported no solution")
            return self._validated_targets(action, expected_joints), True
        except Exception as error:
            self._warn_failure(side, error)
            return dict(previous), False

    def _warn_failure(self, side: str, error: Exception) -> None:
        now = self._monotonic()
        last = self._last_warning[side]
        if now < last or now - last >= self._warning_interval:
            LOGGER.warning("%s arm IK failed: %s", side.capitalize(), error)
            self._last_warning[side] = now

    def _validated_targets(
        self, action: Any, expected_joints: tuple[str, ...]
    ) -> dict[str, float]:
        positions = getattr(action, "joint_positions", None)
        indices = getattr(action, "joint_indices", None)
        if positions is None or indices is None:
            raise ValueError("IK action has no joint positions or indices")

        positions = list(positions)
        indices = [int(index) for index in indices]
        if len(positions) != 7 or len(indices) != 7:
            raise ValueError("IK action must contain exactly seven joints")
        if len(set(indices)) != 7:
            raise ValueError("IK action contains duplicate joint indices")
        if any(
            index < 0 or index >= len(self._joint_names) for index in indices
        ):
            raise ValueError("IK action contains an out-of-range joint index")
        if any(not math.isfinite(float(value)) for value in positions):
            raise ValueError("IK action contains a non-finite joint position")

        targets = {
            self._joint_names[index]: float(value)
            for index, value in zip(indices, positions)
        }
        if set(targets) != set(expected_joints):
            target_names = sorted(targets)
            raise ValueError(f"IK joint mismatch: {target_names}")
        return {name: targets[name] for name in expected_joints}


def create_dual_arm_lula(
    articulation: Any,
    *,
    joint_names: Sequence[str] | None = None,
    project_root: str | Path | None = None,
    lula_solver_cls: type | None = None,
    articulation_solver_cls: type | None = None,
) -> DualArmLulaIK:
    """Create two Isaac Sim 5 Lula stacks for an Isaac Sim Core articulation.

    This factory does not accept an Isaac Lab ``Articulation`` directly. The
    Task 3 runtime must construct or retrieve the corresponding Isaac Sim Core
    articulation before calling it.
    """
    root = (
        Path(project_root).resolve()
        if project_root is not None
        else Path(__file__).resolve().parents[2]
    )
    config_dir = root / "assets" / "lula" / "mobile_fr3_duo"
    left_description = config_dir / "left_arm_description.yaml"
    right_description = config_dir / "right_arm_description.yaml"
    urdf = config_dir / "mobile_fr3_duo_v0_2_franka_hand.urdf"
    missing = [
        path
        for path in (left_description, right_description, urdf)
        if not path.is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "Missing Task 3 Lula files: " + ", ".join(map(str, missing))
        )

    if lula_solver_cls is None or articulation_solver_cls is None:
        default_lula, default_articulation = _load_isaac_solver_classes()
        lula_solver_cls = lula_solver_cls or default_lula
        articulation_solver_cls = (
            articulation_solver_cls or default_articulation
        )

    left_lula = lula_solver_cls(str(left_description), str(urdf))
    right_lula = lula_solver_cls(str(right_description), str(urdf))
    left_solver = articulation_solver_cls(
        articulation, left_lula, LEFT_END_EFFECTOR
    )
    right_solver = articulation_solver_cls(
        articulation, right_lula, RIGHT_END_EFFECTOR
    )
    names = tuple(joint_names or _articulation_joint_names(articulation))
    return DualArmLulaIK(
        left_solver,
        right_solver,
        names,
        left_lula_solver=left_lula,
        right_lula_solver=right_lula,
    )


def create_raw_dual_arm_lula(
    joint_names: Sequence[str],
    current_joint_positions: Callable[[], Any],
    *,
    project_root: str | Path | None = None,
    lula_solver_cls: type | None = None,
) -> DualArmLulaIK:
    """Create raw Lula arm solvers with CPU joint-state warm starts."""
    root = (
        Path(project_root).resolve()
        if project_root is not None
        else Path(__file__).resolve().parents[2]
    )
    config_dir = root / "assets" / "lula" / "mobile_fr3_duo"
    left_description = config_dir / "left_arm_description.yaml"
    right_description = config_dir / "right_arm_description.yaml"
    urdf = config_dir / "mobile_fr3_duo_v0_2_franka_hand.urdf"
    missing = [
        path
        for path in (left_description, right_description, urdf)
        if not path.is_file()
    ]
    if missing:
        raise FileNotFoundError(
            "Missing Task 3 Lula files: " + ", ".join(map(str, missing))
        )
    if lula_solver_cls is None:
        lula_solver_cls = _load_lula_solver_class()
    names = tuple(joint_names)
    left_lula = lula_solver_cls(str(left_description), str(urdf))
    right_lula = lula_solver_cls(str(right_description), str(urdf))
    return DualArmLulaIK(
        RawLulaArmSolver(
            left_lula,
            LEFT_END_EFFECTOR,
            names,
            LEFT_ARM_JOINTS,
            current_joint_positions,
        ),
        RawLulaArmSolver(
            right_lula,
            RIGHT_END_EFFECTOR,
            names,
            RIGHT_ARM_JOINTS,
            current_joint_positions,
        ),
        names,
        left_lula_solver=left_lula,
        right_lula_solver=right_lula,
    )


def _load_isaac_solver_classes() -> tuple[type, type]:
    """Load Isaac Sim 5 exports, including the archive-layout fallback."""
    motion_generation = import_module(
        "isaacsim.robot_motion.motion_generation"
    )
    try:
        return (
            motion_generation.LulaKinematicsSolver,
            motion_generation.ArticulationKinematicsSolver,
        )
    except AttributeError:
        # Robotiq_DEMO 78c28ea imports these submodules directly; retain that
        # layout only for Isaac Sim builds that do not re-export the classes.
        lula_module = import_module(
            "isaacsim.robot_motion.motion_generation.lula.kinematics"
        )
        articulation_module = import_module(
            "isaacsim.robot_motion.motion_generation."
            "articulation_kinematics_solver"
        )
        return (
            lula_module.LulaKinematicsSolver,
            articulation_module.ArticulationKinematicsSolver,
        )


def _load_lula_solver_class() -> type:
    motion_generation = import_module(
        "isaacsim.robot_motion.motion_generation"
    )
    try:
        return motion_generation.LulaKinematicsSolver
    except AttributeError:
        return import_module(
            "isaacsim.robot_motion.motion_generation.lula.kinematics"
        ).LulaKinematicsSolver


def _articulation_joint_names(articulation: Any) -> Sequence[str]:
    for attribute in ("dof_names", "joint_names"):
        names = getattr(articulation, attribute, None)
        if names is not None:
            return names
    raise ValueError(
        "joint_names must be supplied when articulation exposes no joint names"
    )


def _finite_vector(
    values: Sequence[float], length: int, label: str
) -> np.ndarray:
    try:
        result = np.array(values, dtype=np.float64, copy=True)
    except (TypeError, ValueError):
        raise ValueError(
            f"{label} must contain {length} finite values"
        ) from None
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{label} must contain {length} finite values")
    return result


def _normalized_quaternion(values: Sequence[float], label: str) -> np.ndarray:
    quaternion = _finite_vector(values, 4, label)
    norm = float(np.linalg.norm(quaternion))
    if norm == 0.0:
        raise ValueError(f"{label} must be nonzero")
    return quaternion / norm


def _rotated_vertical_offset(
    orientation_wxyz: np.ndarray, distance: float
) -> np.ndarray:
    w, x, y, z = orientation_wxyz
    local_z_in_world = np.array(
        (
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ),
        dtype=np.float64,
    )
    return local_z_in_world * distance


def _current_end_effector_pose(
    solver: Any,
) -> tuple[np.ndarray, np.ndarray]:
    position, rotation = solver.compute_end_effector_pose()
    return (
        _finite_vector(position, 3, "end-effector position"),
        _rotation_matrix_to_quaternion(rotation),
    )


def _rotation_matrix_to_quaternion(rotation: Any) -> np.ndarray:
    try:
        matrix = np.array(rotation, dtype=np.float64, copy=True)
    except (TypeError, ValueError):
        raise ValueError(
            "end-effector rotation must be a finite 3x3 matrix"
        ) from None
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("end-effector rotation must be a finite 3x3 matrix")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            (
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            )
        )
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = (
                math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
                * 2.0
            )
            quaternion = np.array(
                (
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                )
            )
        elif axis == 1:
            scale = (
                math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
                * 2.0
            )
            quaternion = np.array(
                (
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                )
            )
        else:
            scale = (
                math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
                * 2.0
            )
            quaternion = np.array(
                (
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                )
            )
    return _normalized_quaternion(quaternion, "end-effector orientation")
