# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np
import warp as wp
import yaml

import newton


@dataclass(frozen=True)
class PoseConfig:
    position_m: tuple[float, float, float]
    rotation_euler_xyz_deg: tuple[float, float, float]
    rotation: wp.quat


@dataclass(frozen=True)
class FingerConfig:
    density: float
    friction: float


@dataclass(frozen=True)
class GapConfig:
    initial_m: float
    target_m: float
    min_m: float
    max_m: float


@dataclass(frozen=True)
class GripperControlConfig:
    mode: str
    drive_force: float
    stiffness: float
    damping: float


@dataclass(frozen=True)
class GripperTeleopConfig:
    enabled: bool
    frame: str
    linear_speed_mps: float
    linear_speed_xy_mps: float
    linear_speed_z_mps: float
    angular_speed_radps: float
    gap_speed_mps: float
    require_ctrl: bool
    modifier: str


@dataclass(frozen=True)
class GravityCompensationConfig:
    enabled: bool
    bodies: str


@dataclass(frozen=True)
class SraGripperConfig:
    enabled: bool
    label: str
    profile: str
    asset_variant: str
    pose: PoseConfig
    finger: FingerConfig
    gap: GapConfig
    control: GripperControlConfig
    teleop: GripperTeleopConfig
    gravity_compensation: GravityCompensationConfig


@dataclass(frozen=True)
class SraGripperBuildResult:
    root_body_id: int
    finger_body_ids: tuple[int, int]
    finger_body_masses: tuple[float, float]
    root_shape_ids: tuple[int, ...]
    finger_shape_ids: tuple[tuple[int, ...], tuple[int, ...]]
    finger_collision_shape_ids: tuple[tuple[int, ...], tuple[int, ...]]
    world_joint_id: int
    prismatic_joint_ids: tuple[int, int]
    prismatic_dof_starts: tuple[int, int]


@dataclass(frozen=True)
class GraspPoseBindConfig:
    enabled: bool
    candidate_body_labels: tuple[str, ...]
    confirm_steps: int
    release_gap_m: float
    normal_alignment_min_cos: float
    opposing_normal_min_cos: float
    max_position_error_m: float
    max_rotation_error_rad: float
    candidate_body_label_prefixes: tuple[str, ...] = ()
    activation_radius_m: float = 0.0


def _normalize_quat_np(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise ValueError("Quaternion norm must be positive.")
    return np.asarray(q, dtype=np.float64) / norm


def _quat_conjugate_np(q: np.ndarray) -> np.ndarray:
    qn = _normalize_quat_np(q)
    return np.asarray((-qn[0], -qn[1], -qn[2], qn[3]), dtype=np.float64)


def _quat_multiply_np(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = _normalize_quat_np(a)
    bx, by, bz, bw = _normalize_quat_np(b)
    return _normalize_quat_np(
        np.asarray(
            (
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ),
            dtype=np.float64,
        )
    )


def rotate_vector_by_quat(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    qn = _normalize_quat_np(q)
    q_vec = qn[:3]
    q_w = float(qn[3])
    v = np.asarray(vector, dtype=np.float64)
    t = 2.0 * np.cross(q_vec, v)
    return v + q_w * t + np.cross(q_vec, t)


def _relative_pose_np(root_pose: np.ndarray, object_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    root_p = np.asarray(root_pose[:3], dtype=np.float64)
    root_q_inv = _quat_conjugate_np(np.asarray(root_pose[3:7], dtype=np.float64))
    object_p = np.asarray(object_pose[:3], dtype=np.float64)
    object_q = np.asarray(object_pose[3:7], dtype=np.float64)
    relative_p = rotate_vector_by_quat(root_q_inv, object_p - root_p)
    relative_q = _quat_multiply_np(root_q_inv, object_q)
    return relative_p, relative_q


def _compose_pose_np(root_pose: np.ndarray, relative_p: np.ndarray, relative_q: np.ndarray) -> np.ndarray:
    root_p = np.asarray(root_pose[:3], dtype=np.float64)
    root_q = np.asarray(root_pose[3:7], dtype=np.float64)
    object_p = root_p + rotate_vector_by_quat(root_q, relative_p)
    object_q = _quat_multiply_np(root_q, relative_q)
    return np.asarray((*object_p, *object_q), dtype=np.float64)


def _quat_angle_np(a: np.ndarray, b: np.ndarray) -> float:
    aq = _normalize_quat_np(a)
    bq = _normalize_quat_np(b)
    dot = min(max(abs(float(np.dot(aq, bq))), 0.0), 1.0)
    return 2.0 * math.acos(dot)


def _axis_angle_quat_np(axis: tuple[float, float, float], angle_rad: float) -> np.ndarray:
    axis_np = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(axis_np))
    if norm <= 0.0:
        raise ValueError("Rotation axis norm must be positive.")
    axis_np = axis_np / norm
    half_angle = 0.5 * float(angle_rad)
    sin_half = math.sin(half_angle)
    return np.asarray(
        (
            axis_np[0] * sin_half,
            axis_np[1] * sin_half,
            axis_np[2] * sin_half,
            math.cos(half_angle),
        ),
        dtype=np.float64,
    )


def _quat_np_to_wp(q: np.ndarray) -> wp.quat:
    qn = _normalize_quat_np(q)
    return wp.quat(float(qn[0]), float(qn[1]), float(qn[2]), float(qn[3]))


def _wp_quat_to_tuple(q: wp.quat) -> tuple[float, float, float, float]:
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _quat_to_euler_xyz_rad(q: tuple[float, float, float, float] | list[float]) -> tuple[float, float, float]:
    x, y, z, w = _normalize_quat_np(np.asarray(q, dtype=np.float64))

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(min(max(sinp, -1.0), 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


class GraspPoseBindController:
    def __init__(
        self,
        config: GraspPoseBindConfig,
        build_result: SraGripperBuildResult,
        model: newton.Model,
        candidate_body_ids: tuple[int, ...],
    ):
        if config.confirm_steps <= 0:
            raise ValueError("Grasp pose bind confirm_steps must be positive.")
        if config.release_gap_m < 0.0:
            raise ValueError("Grasp pose bind release_gap_m must be non-negative.")
        if config.max_position_error_m < 0.0:
            raise ValueError("Grasp pose bind max_position_error_m must be non-negative.")
        if config.max_rotation_error_rad < 0.0:
            raise ValueError("Grasp pose bind max_rotation_error_rad must be non-negative.")
        if config.activation_radius_m < 0.0:
            raise ValueError("Grasp pose bind activation_radius_m must be non-negative.")
        if len(candidate_body_ids) == 0:
            raise ValueError("Grasp pose bind requires at least one candidate body id.")

        self.config = config
        self.build_result = build_result
        self.model = model
        self.candidate_body_ids = tuple(int(body_id) for body_id in candidate_body_ids)
        self._candidate_body_id_set = frozenset(self.candidate_body_ids)
        self._gripper_body_ids = (
            int(build_result.root_body_id),
            int(build_result.finger_body_ids[0]),
            int(build_result.finger_body_ids[1]),
        )
        self.state_name = "idle"
        self.grasped_body_id: int | None = None
        self._confirm_body_id: int | None = None
        self._confirm_steps = 0
        self._relative_position: np.ndarray | None = None
        self._relative_rotation: np.ndarray | None = None
        self._shape_body = model.shape_body.numpy()
        self._left_finger_shapes = frozenset(int(shape_id) for shape_id in build_result.finger_collision_shape_ids[0])
        self._right_finger_shapes = frozenset(int(shape_id) for shape_id in build_result.finger_collision_shape_ids[1])

    def release(self) -> None:
        self.state_name = "idle"
        self.grasped_body_id = None
        self._confirm_body_id = None
        self._confirm_steps = 0
        self._relative_position = None
        self._relative_rotation = None

    def update_from_contacts(self, state: newton.State, contacts: Any, target_gap_m: float) -> None:
        if not self.config.enabled:
            return
        if target_gap_m > self.config.release_gap_m:
            self.release()
            return
        if self.state_name == "grasped":
            if self.grasped_body_id is None:
                raise ValueError("Grasp pose bind state is missing grasped body id.")
            # Once a cable body is pinched, keep the pose binding until the
            # gripper opens or the bound body drifts too far. Requiring bilateral
            # contact every step makes the grasp flicker because contact pairs can
            # disappear for a frame after the body is snapped to the gripper root.
            if self._grasp_error_exceeds_limit(state):
                self.release()
                return
            self.apply_pose_binding(state)
            return
        if not self._candidate_inside_activation_radius(state):
            self.release()
            return

        candidate_body_id = self._detect_attach_candidate(state, contacts)
        if candidate_body_id is None:
            self.release()
            return
        if self._confirm_body_id == candidate_body_id:
            self._confirm_steps += 1
        else:
            self._confirm_body_id = candidate_body_id
            self._confirm_steps = 1
        self.state_name = "confirming"
        if self._confirm_steps >= self.config.confirm_steps:
            self._enter_grasped(state, candidate_body_id)
            self.apply_pose_binding(state)

    def apply_pose_binding(self, state: newton.State) -> None:
        if self.state_name != "grasped":
            return
        if self.grasped_body_id is None or self._relative_position is None or self._relative_rotation is None:
            raise ValueError("Grasp pose bind state is missing grasp data.")

        body_q = state.body_q.numpy()
        root_pose = body_q[self.build_result.root_body_id]
        target_pose = _compose_pose_np(root_pose, self._relative_position, self._relative_rotation)
        body_q[self.grasped_body_id, :] = target_pose.astype(body_q.dtype)
        state.body_q.assign(body_q)

        body_qd = state.body_qd.numpy()
        body_qd[self.grasped_body_id, :] = 0.0
        state.body_qd.assign(body_qd)

    def _enter_grasped(self, state: newton.State, body_id: int) -> None:
        body_q = state.body_q.numpy()
        relative_position, relative_rotation = _relative_pose_np(
            body_q[self.build_result.root_body_id],
            body_q[body_id],
        )
        self.state_name = "grasped"
        self.grasped_body_id = int(body_id)
        self._relative_position = relative_position
        self._relative_rotation = relative_rotation

    def _grasp_error_exceeds_limit(self, state: newton.State) -> bool:
        if self.grasped_body_id is None or self._relative_position is None or self._relative_rotation is None:
            raise ValueError("Grasp pose bind state is missing grasp data.")
        body_q = state.body_q.numpy()
        target_pose = _compose_pose_np(
            body_q[self.build_result.root_body_id],
            self._relative_position,
            self._relative_rotation,
        )
        current_pose = body_q[self.grasped_body_id]
        position_error = float(np.linalg.norm(np.asarray(current_pose[:3], dtype=np.float64) - target_pose[:3]))
        rotation_error = _quat_angle_np(np.asarray(current_pose[3:7], dtype=np.float64), target_pose[3:7])
        return position_error > self.config.max_position_error_m or rotation_error > self.config.max_rotation_error_rad

    def _detect_attach_candidate(self, state: newton.State, contacts: Any) -> int | None:
        contact_count = int(contacts.rigid_contact_count.numpy()[0])
        shape0 = contacts.rigid_contact_shape0.numpy()[:contact_count]
        shape1 = contacts.rigid_contact_shape1.numpy()[:contact_count]
        normals = contacts.rigid_contact_normal.numpy()[:contact_count]
        root_q = state.body_q.numpy()[self.build_result.root_body_id, 3:7]
        grasp_axis = rotate_vector_by_quat(root_q, np.asarray((0.0, 1.0, 0.0), dtype=np.float64))

        left_normals: dict[int, np.ndarray] = {}
        left_alignments: dict[int, float] = {}
        right_normals: dict[int, np.ndarray] = {}
        right_alignments: dict[int, float] = {}
        for shape_a_raw, shape_b_raw, normal_raw in zip(shape0, shape1, normals, strict=True):
            shape_a = int(shape_a_raw)
            shape_b = int(shape_b_raw)
            normal = np.asarray(normal_raw, dtype=np.float64)
            self._record_contact_normal(shape_a, shape_b, normal, grasp_axis, left_normals, left_alignments, right_normals, right_alignments)
            self._record_contact_normal(shape_b, shape_a, -normal, grasp_axis, left_normals, left_alignments, right_normals, right_alignments)

        for body_id in self.candidate_body_ids:
            if body_id in left_normals and body_id in right_normals:
                opposing_alignment = float(np.dot(left_normals[body_id], -right_normals[body_id]))
                if opposing_alignment >= self.config.opposing_normal_min_cos:
                    return body_id
        return None

    def _has_bilateral_contact_with_body(self, contacts: Any, body_id: int) -> bool:
        contact_count = int(contacts.rigid_contact_count.numpy()[0])
        shape0 = contacts.rigid_contact_shape0.numpy()[:contact_count]
        shape1 = contacts.rigid_contact_shape1.numpy()[:contact_count]
        has_left_contact = False
        has_right_contact = False
        for shape_a_raw, shape_b_raw in zip(shape0, shape1, strict=True):
            shape_a = int(shape_a_raw)
            shape_b = int(shape_b_raw)
            has_left_contact = has_left_contact or self._finger_shape_contacts_body(
                shape_a,
                shape_b,
                self._left_finger_shapes,
                body_id,
            )
            has_left_contact = has_left_contact or self._finger_shape_contacts_body(
                shape_b,
                shape_a,
                self._left_finger_shapes,
                body_id,
            )
            has_right_contact = has_right_contact or self._finger_shape_contacts_body(
                shape_a,
                shape_b,
                self._right_finger_shapes,
                body_id,
            )
            has_right_contact = has_right_contact or self._finger_shape_contacts_body(
                shape_b,
                shape_a,
                self._right_finger_shapes,
                body_id,
            )
            if has_left_contact and has_right_contact:
                return True
        return False

    def _finger_shape_contacts_body(
        self,
        finger_shape: int,
        object_shape: int,
        finger_shapes: frozenset[int],
        body_id: int,
    ) -> bool:
        return finger_shape in finger_shapes and int(self._shape_body[object_shape]) == body_id

    def _candidate_inside_activation_radius(self, state: newton.State) -> bool:
        if self.config.activation_radius_m == 0.0:
            return True
        body_q = state.body_q.numpy()
        gripper_positions = np.asarray(body_q[list(self._gripper_body_ids), :3], dtype=np.float64)
        candidate_positions = np.asarray(body_q[list(self.candidate_body_ids), :3], dtype=np.float64)
        deltas = candidate_positions[:, None, :] - gripper_positions[None, :, :]
        min_distance_sq = float(np.min(np.sum(deltas * deltas, axis=2)))
        activation_radius_sq = float(self.config.activation_radius_m * self.config.activation_radius_m)
        return min_distance_sq <= activation_radius_sq

    def _record_contact_normal(
        self,
        finger_shape: int,
        object_shape: int,
        finger_to_object_normal: np.ndarray,
        grasp_axis: np.ndarray,
        left_normals: dict[int, np.ndarray],
        left_alignments: dict[int, float],
        right_normals: dict[int, np.ndarray],
        right_alignments: dict[int, float],
    ) -> None:
        object_body = int(self._shape_body[object_shape])
        if object_body not in self._candidate_body_id_set:
            return
        normal_length = float(np.linalg.norm(finger_to_object_normal))
        if normal_length <= 0.0:
            return
        normal = finger_to_object_normal / normal_length
        if finger_shape in self._left_finger_shapes:
            alignment = float(np.dot(normal, -grasp_axis))
            if alignment >= self.config.normal_alignment_min_cos:
                if object_body not in left_alignments or alignment > left_alignments[object_body]:
                    left_alignments[object_body] = alignment
                    left_normals[object_body] = normal
        elif finger_shape in self._right_finger_shapes:
            alignment = float(np.dot(normal, grasp_axis))
            if alignment >= self.config.normal_alignment_min_cos:
                if object_body not in right_alignments or alignment > right_alignments[object_body]:
                    right_alignments[object_body] = alignment
                    right_normals[object_body] = normal


class RobotiqFingerPoseBindController:
    """Bind a pinched cable body to the midpoint between a pair of kinematic finger bodies."""

    def __init__(
        self,
        config: GraspPoseBindConfig,
        model: newton.Model,
        candidate_body_ids: tuple[int, ...],
        finger_body_ids: tuple[int, ...],
        finger_shape_ids: tuple[int, ...],
        finger_pairs: tuple[tuple[int, int], ...] = ((0, 1), (2, 3)),
    ):
        if config.confirm_steps <= 0:
            raise ValueError("Robotiq grasp pose bind confirm_steps must be positive.")
        if config.release_gap_m < 0.0:
            raise ValueError("Robotiq grasp pose bind release_gap_m must be non-negative.")
        if config.max_position_error_m < 0.0:
            raise ValueError("Robotiq grasp pose bind max_position_error_m must be non-negative.")
        if config.max_rotation_error_rad < 0.0:
            raise ValueError("Robotiq grasp pose bind max_rotation_error_rad must be non-negative.")
        if config.activation_radius_m < 0.0:
            raise ValueError("Robotiq grasp pose bind activation_radius_m must be non-negative.")
        if len(candidate_body_ids) == 0:
            raise ValueError("Robotiq grasp pose bind requires at least one candidate body id.")

        self.config = config
        self.model = model
        self.candidate_body_ids = tuple(int(body_id) for body_id in candidate_body_ids)
        self._candidate_body_id_set = frozenset(self.candidate_body_ids)
        self.finger_body_ids = tuple(int(body_id) for body_id in finger_body_ids)
        self.finger_shape_ids = tuple(int(shape_id) for shape_id in finger_shape_ids)
        self.finger_pairs = tuple((int(a), int(b)) for a, b in finger_pairs)
        self._shape_body = model.shape_body.numpy()
        self._states = [
            {
                "state_name": "idle",
                "grasped_body_id": None,
                "confirm_body_id": None,
                "confirm_steps": 0,
                "relative_position": None,
                "relative_rotation": None,
            }
            for _ in self.finger_pairs
        ]

    def release(self, pair_index: int | None = None) -> None:
        states = self._states if pair_index is None else (self._states[pair_index],)
        for state in states:
            state["state_name"] = "idle"
            state["grasped_body_id"] = None
            state["confirm_body_id"] = None
            state["confirm_steps"] = 0
            state["relative_position"] = None
            state["relative_rotation"] = None

    def update_from_contacts(self, state: newton.State, contacts: Any) -> None:
        if not self.config.enabled:
            return
        for pair_index, pair in enumerate(self.finger_pairs):
            self._update_pair_from_contacts(pair_index, pair, state, contacts)

    def apply_pose_binding(self, state: newton.State) -> None:
        body_q = state.body_q.numpy()
        body_qd = state.body_qd.numpy()
        updated = False
        for pair_index, pair in enumerate(self.finger_pairs):
            pair_state = self._states[pair_index]
            if pair_state["state_name"] != "grasped":
                continue
            body_id = pair_state["grasped_body_id"]
            rel_pos = pair_state["relative_position"]
            rel_rot = pair_state["relative_rotation"]
            if body_id is None or rel_pos is None or rel_rot is None:
                raise ValueError("Robotiq grasp pose bind state is missing grasp data.")
            root_pose = self._pair_pose_from_body_q(body_q, pair)
            target_pose = _compose_pose_np(root_pose, rel_pos, rel_rot)
            body_q[int(body_id), :] = target_pose.astype(body_q.dtype)
            body_qd[int(body_id), :] = 0.0
            updated = True
        if updated:
            state.body_q.assign(body_q)
            state.body_qd.assign(body_qd)

    def _update_pair_from_contacts(self, pair_index: int, pair: tuple[int, int], state: newton.State, contacts: Any) -> None:
        pair_state = self._states[pair_index]
        if self._pair_gap_m(state, pair) > self.config.release_gap_m:
            self.release(pair_index)
            return
        if pair_state["state_name"] == "grasped":
            if self._grasp_error_exceeds_limit(pair_index, pair, state):
                self.release(pair_index)
                return
            self.apply_pose_binding(state)
            return
        if not self._candidate_inside_activation_radius(state, pair):
            self.release(pair_index)
            return

        candidate_body_id = self._detect_attach_candidate(state, contacts, pair)
        if candidate_body_id is None:
            self.release(pair_index)
            return
        if pair_state["confirm_body_id"] == candidate_body_id:
            pair_state["confirm_steps"] = int(pair_state["confirm_steps"]) + 1
        else:
            pair_state["confirm_body_id"] = candidate_body_id
            pair_state["confirm_steps"] = 1
        pair_state["state_name"] = "confirming"
        if int(pair_state["confirm_steps"]) >= self.config.confirm_steps:
            self._enter_grasped(pair_index, pair, state, candidate_body_id)
            self.apply_pose_binding(state)

    def _enter_grasped(self, pair_index: int, pair: tuple[int, int], state: newton.State, body_id: int) -> None:
        body_q = state.body_q.numpy()
        root_pose = self._pair_pose_from_body_q(body_q, pair)
        relative_position, relative_rotation = _relative_pose_np(root_pose, body_q[body_id])
        pair_state = self._states[pair_index]
        pair_state["state_name"] = "grasped"
        pair_state["grasped_body_id"] = int(body_id)
        pair_state["relative_position"] = relative_position
        pair_state["relative_rotation"] = relative_rotation

    def _grasp_error_exceeds_limit(self, pair_index: int, pair: tuple[int, int], state: newton.State) -> bool:
        pair_state = self._states[pair_index]
        body_id = pair_state["grasped_body_id"]
        rel_pos = pair_state["relative_position"]
        rel_rot = pair_state["relative_rotation"]
        if body_id is None or rel_pos is None or rel_rot is None:
            raise ValueError("Robotiq grasp pose bind state is missing grasp data.")
        body_q = state.body_q.numpy()
        target_pose = _compose_pose_np(self._pair_pose_from_body_q(body_q, pair), rel_pos, rel_rot)
        current_pose = body_q[int(body_id)]
        position_error = float(np.linalg.norm(np.asarray(current_pose[:3], dtype=np.float64) - target_pose[:3]))
        rotation_error = _quat_angle_np(np.asarray(current_pose[3:7], dtype=np.float64), target_pose[3:7])
        return position_error > self.config.max_position_error_m or rotation_error > self.config.max_rotation_error_rad

    def _detect_attach_candidate(self, state: newton.State, contacts: Any, pair: tuple[int, int]) -> int | None:
        contact_count = int(contacts.rigid_contact_count.numpy()[0])
        shape0 = contacts.rigid_contact_shape0.numpy()[:contact_count]
        shape1 = contacts.rigid_contact_shape1.numpy()[:contact_count]
        normals = contacts.rigid_contact_normal.numpy()[:contact_count]
        root_pose = self._pair_pose_from_body_q(state.body_q.numpy(), pair)
        grasp_axis = rotate_vector_by_quat(root_pose[3:7], np.asarray((0.0, 1.0, 0.0), dtype=np.float64))

        first_shape = self.finger_shape_ids[pair[0]]
        second_shape = self.finger_shape_ids[pair[1]]
        first_normals: dict[int, np.ndarray] = {}
        first_alignments: dict[int, float] = {}
        second_normals: dict[int, np.ndarray] = {}
        second_alignments: dict[int, float] = {}
        for shape_a_raw, shape_b_raw, normal_raw in zip(shape0, shape1, normals, strict=True):
            shape_a = int(shape_a_raw)
            shape_b = int(shape_b_raw)
            normal = np.asarray(normal_raw, dtype=np.float64)
            self._record_contact_normal(
                shape_a, shape_b, normal, grasp_axis, first_shape, second_shape,
                first_normals, first_alignments, second_normals, second_alignments
            )
            self._record_contact_normal(
                shape_b, shape_a, -normal, grasp_axis, first_shape, second_shape,
                first_normals, first_alignments, second_normals, second_alignments
            )

        for body_id in self.candidate_body_ids:
            if body_id in first_normals and body_id in second_normals:
                opposing_alignment = float(np.dot(first_normals[body_id], -second_normals[body_id]))
                if opposing_alignment >= self.config.opposing_normal_min_cos:
                    return body_id
        return None

    def _record_contact_normal(
        self,
        finger_shape: int,
        object_shape: int,
        finger_to_object_normal: np.ndarray,
        grasp_axis: np.ndarray,
        first_shape: int,
        second_shape: int,
        first_normals: dict[int, np.ndarray],
        first_alignments: dict[int, float],
        second_normals: dict[int, np.ndarray],
        second_alignments: dict[int, float],
    ) -> None:
        object_body = int(self._shape_body[object_shape])
        if object_body not in self._candidate_body_id_set:
            return
        normal_length = float(np.linalg.norm(finger_to_object_normal))
        if normal_length <= 0.0:
            return
        normal = finger_to_object_normal / normal_length
        # Robotiq target boxes inherit orientation from visual finger frames, whose
        # local axes are not guaranteed to match the pinch direction. For this path
        # the robust signal is bilateral contact plus opposing contact normals.
        alignment = 1.0
        if finger_shape == first_shape:
            if object_body not in first_alignments or alignment > first_alignments[object_body]:
                first_alignments[object_body] = alignment
                first_normals[object_body] = normal
        elif finger_shape == second_shape:
            if object_body not in second_alignments or alignment > second_alignments[object_body]:
                second_alignments[object_body] = alignment
                second_normals[object_body] = normal

    def _candidate_inside_activation_radius(self, state: newton.State, pair: tuple[int, int]) -> bool:
        if self.config.activation_radius_m == 0.0:
            return True
        body_q = state.body_q.numpy()
        root_position = self._pair_pose_from_body_q(body_q, pair)[:3]
        candidate_positions = np.asarray(body_q[list(self.candidate_body_ids), :3], dtype=np.float64)
        deltas = candidate_positions - root_position[None, :]
        min_distance_sq = float(np.min(np.sum(deltas * deltas, axis=1)))
        activation_radius_sq = float(self.config.activation_radius_m * self.config.activation_radius_m)
        return min_distance_sq <= activation_radius_sq

    def _pair_gap_m(self, state: newton.State, pair: tuple[int, int]) -> float:
        body_q = state.body_q.numpy()
        first = np.asarray(body_q[self.finger_body_ids[pair[0]], :3], dtype=np.float64)
        second = np.asarray(body_q[self.finger_body_ids[pair[1]], :3], dtype=np.float64)
        return float(np.linalg.norm(second - first))

    def _pair_pose_from_body_q(self, body_q: np.ndarray, pair: tuple[int, int]) -> np.ndarray:
        first_pose = np.asarray(body_q[self.finger_body_ids[pair[0]]], dtype=np.float64)
        second_pose = np.asarray(body_q[self.finger_body_ids[pair[1]]], dtype=np.float64)
        position = 0.5 * (first_pose[:3] + second_pose[:3])
        quat = _normalize_quat_np(first_pose[3:7])
        return np.asarray((position[0], position[1], position[2], quat[0], quat[1], quat[2], quat[3]), dtype=np.float64)


@dataclass
class SraGripperTeleopState:
    position_m: list[float]
    rotation_quat_xyzw: list[float]
    target_gap_m: float
    translation_frame: str

    @classmethod
    def from_config(cls, config: SraGripperConfig) -> "SraGripperTeleopState":
        return cls(
            position_m=list(config.pose.position_m),
            rotation_quat_xyzw=list(_wp_quat_to_tuple(config.pose.rotation)),
            target_gap_m=float(config.gap.target_m),
            translation_frame=str(config.teleop.frame),
        )

    def update_from_viewer(self, viewer: Any, config: SraGripperConfig, dt: float) -> None:
        if not config.teleop.enabled:
            return
        if config.teleop.modifier != "none" and not _viewer_key_down(viewer, config.teleop.modifier):
            return

        translation_delta = np.zeros(3, dtype=np.float64)
        step_xy = float(config.teleop.linear_speed_xy_mps) * float(dt)
        step_z = float(config.teleop.linear_speed_z_mps) * float(dt)
        if _viewer_key_down(viewer, "w"):
            translation_delta[1] += step_xy
        if _viewer_key_down(viewer, "s"):
            translation_delta[1] -= step_xy
        if _viewer_key_down(viewer, "d"):
            translation_delta[0] += step_xy
        if _viewer_key_down(viewer, "a"):
            translation_delta[0] -= step_xy
        if _viewer_key_down(viewer, "q"):
            translation_delta[2] += step_z
        if _viewer_key_down(viewer, "e"):
            translation_delta[2] -= step_z

        rotation = _normalize_quat_np(np.asarray(self.rotation_quat_xyzw, dtype=np.float64))
        if self.translation_frame == "world":
            world_delta = translation_delta
        elif self.translation_frame == "eeframe":
            world_delta = rotate_vector_by_quat(rotation, translation_delta)
        else:
            raise ValueError(
                f"Unsupported gripper translation frame '{self.translation_frame}'. Expected 'world' or 'eeframe'."
            )
        for i in range(3):
            self.position_m[i] += float(world_delta[i])

        angle_step = float(config.teleop.angular_speed_radps) * float(dt)
        for key_positive, key_negative, axis in (
            ("c", "v", (1.0, 0.0, 0.0)),
            ("z", "x", (0.0, 1.0, 0.0)),
            ("t", "g", (0.0, 0.0, 1.0)),
        ):
            signed_step = 0.0
            if _viewer_key_down(viewer, key_positive):
                signed_step += angle_step
            if _viewer_key_down(viewer, key_negative):
                signed_step -= angle_step
            if signed_step != 0.0:
                rotation = _quat_multiply_np(rotation, _axis_angle_quat_np(axis, signed_step))
        self.rotation_quat_xyzw = [float(v) for v in rotation.tolist()]

        gap_step = float(config.teleop.gap_speed_mps) * float(dt)
        if _viewer_key_down(viewer, "n"):
            self.target_gap_m -= gap_step
        if _viewer_key_down(viewer, "m"):
            self.target_gap_m += gap_step
        self.target_gap_m = min(max(self.target_gap_m, config.gap.min_m), config.gap.max_m)

    def root_transform(self) -> wp.transform:
        return wp.transform(
            wp.vec3(float(self.position_m[0]), float(self.position_m[1]), float(self.position_m[2])),
            _quat_np_to_wp(np.asarray(self.rotation_quat_xyzw, dtype=np.float64)),
        )


@dataclass(frozen=True)
class GraspObjectBoxConfig:
    position_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    density: float
    friction: float


@dataclass(frozen=True)
class GraspObjectCapsuleConfig:
    position_m: tuple[float, float, float]
    radius_m: float
    half_height_m: float
    density: float
    friction: float


@dataclass(frozen=True)
class GraspControlSolverConfig:
    iterations: int
    friction_epsilon: float
    rigid_contact_k_start: float


@dataclass(frozen=True)
class GraspControlContactConfig:
    rigid_contact_margin_m: float
    rigid_gap_m: float


@dataclass(frozen=True)
class GraspControlSceneConfig:
    gripper_config_path: Path
    solver: GraspControlSolverConfig
    contact: GraspControlContactConfig
    cuboid: GraspObjectBoxConfig
    capsule: GraspObjectCapsuleConfig
    ground: GraspObjectBoxConfig


@dataclass(frozen=True)
class GraspControlBuildResult:
    gripper: SraGripperBuildResult
    object_body_ids: tuple[int, int]
    object_shape_ids: tuple[int, int]
    ground_shape_id: int


def _viewer_key_down(viewer: Any, key: str) -> bool:
    if not hasattr(viewer, "is_key_down"):
        return False
    if key == "ctrl":
        if bool(viewer.is_key_down("ctrl")):
            return True
        try:
            import pyglet
        except Exception:
            return False
        return bool(
            viewer.is_key_down(pyglet.window.key.LCTRL)
            or viewer.is_key_down(pyglet.window.key.RCTRL)
        )
    if key == "shift":
        if bool(viewer.is_key_down("shift")):
            return True
        try:
            import pyglet
        except Exception:
            return False
        return bool(
            viewer.is_key_down(pyglet.window.key.LSHIFT)
            or viewer.is_key_down(pyglet.window.key.RSHIFT)
        )
    return bool(viewer.is_key_down(key))


def is_gripper_teleop_modifier_down(viewer: Any, modifier: str = "ctrl") -> bool:
    if modifier == "none":
        return False
    return _viewer_key_down(viewer, modifier)


def _require_mapping(data: Any, key_path: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError(f"Config section '{key_path}' must be a mapping.")
    return data


def _require_keys(data: dict[str, Any], keys: tuple[str, ...], key_path: str) -> None:
    for key in keys:
        if key not in data:
            raise KeyError(f"Config section '{key_path}' is missing required key '{key}'.")


def _require_bool(data: dict[str, Any], key: str, key_path: str) -> bool:
    value = data[key]
    if not isinstance(value, bool):
        raise ValueError(f"Config key '{key_path}.{key}' must be boolean.")
    return bool(value)


def _require_str(data: dict[str, Any], key: str, key_path: str) -> str:
    value = data[key]
    if not isinstance(value, str):
        raise ValueError(f"Config key '{key_path}.{key}' must be a string.")
    if value == "":
        raise ValueError(f"Config key '{key_path}.{key}' must not be empty.")
    return value


def _require_float(data: dict[str, Any], key: str, key_path: str) -> float:
    value = data[key]
    if not isinstance(value, int | float):
        raise ValueError(f"Config key '{key_path}.{key}' must be numeric.")
    return float(value)


def _require_vec(data: dict[str, Any], key: str, key_path: str, length: int) -> tuple[float, ...]:
    value = data[key]
    if not isinstance(value, list | tuple):
        raise ValueError(f"Config key '{key_path}.{key}' must be a {length}-element sequence.")
    if len(value) != length:
        raise ValueError(f"Config key '{key_path}.{key}' must contain exactly {length} values.")
    out: list[float] = []
    for item in value:
        if not isinstance(item, int | float):
            raise ValueError(f"Config key '{key_path}.{key}' must contain only numeric values.")
        out.append(float(item))
    return tuple(out)


def _require_positive_vec3(data: dict[str, Any], key: str, key_path: str) -> tuple[float, float, float]:
    vec = _require_vec(data, key, key_path, 3)
    if min(vec) <= 0.0:
        raise ValueError(f"Config key '{key_path}.{key}' must contain positive values.")
    return (vec[0], vec[1], vec[2])


def _load_yaml_mapping(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return _require_mapping(data, str(config_path))


def load_gripper_config(config_path: str | Path) -> SraGripperConfig:
    config_file = Path(config_path).resolve()
    if not config_file.is_file():
        raise ValueError(f"Config file does not exist: {config_file}")

    data = _load_yaml_mapping(config_file)
    _require_keys(
        data,
        (
            "enabled",
            "label",
            "profile",
            "asset_variant",
            "pose",
            "finger",
            "gap",
            "control",
            "teleop",
            "gravity_compensation",
        ),
        "root",
    )

    pose_data = _require_mapping(data["pose"], "pose")
    finger_data = _require_mapping(data["finger"], "finger")
    gap_data = _require_mapping(data["gap"], "gap")
    control_data = _require_mapping(data["control"], "control")
    teleop_data = _require_mapping(data["teleop"], "teleop")
    gravity_compensation_data = _require_mapping(data["gravity_compensation"], "gravity_compensation")
    _require_keys(pose_data, ("position", "rotation_euler_xyz_deg"), "pose")
    _require_keys(finger_data, ("density", "friction"), "finger")
    _require_keys(gap_data, ("initial", "target", "min", "max"), "gap")
    _require_keys(control_data, ("mode", "drive_force", "stiffness", "damping"), "control")
    _require_keys(teleop_data, ("enabled", "frame", "angular_speed_deg", "gap_speed", "require_ctrl"), "teleop")
    _require_keys(gravity_compensation_data, ("enabled", "bodies"), "gravity_compensation")

    gap = GapConfig(
        initial_m=_require_float(gap_data, "initial", "gap"),
        target_m=_require_float(gap_data, "target", "gap"),
        min_m=_require_float(gap_data, "min", "gap"),
        max_m=_require_float(gap_data, "max", "gap"),
    )
    if gap.min_m > gap.max_m:
        raise ValueError("Config key 'gap.min' must be <= 'gap.max'.")
    if gap.initial_m < gap.min_m or gap.initial_m > gap.max_m:
        raise ValueError("Config key 'gap.initial' must be within [gap.min, gap.max].")
    if gap.target_m < gap.min_m or gap.target_m > gap.max_m:
        raise ValueError("Config key 'gap.target' must be within [gap.min, gap.max].")

    mode = _require_str(control_data, "mode", "control")
    if mode != "position":
        raise ValueError("Config key 'control.mode' must be 'position'.")
    frame = _require_str(teleop_data, "frame", "teleop")
    if frame not in ("world", "eeframe"):
        raise ValueError("Config key 'teleop.frame' must be 'world' or 'eeframe'.")
    require_ctrl = _require_bool(teleop_data, "require_ctrl", "teleop")
    modifier = str(teleop_data.get("modifier", "ctrl" if require_ctrl else "none"))
    if modifier not in ("ctrl", "shift", "none"):
        raise ValueError("Config key 'teleop.modifier' must be 'ctrl', 'shift', or 'none'.")
    profile = _require_str(data, "profile", "root")
    if profile != "franka":
        raise ValueError("Config key 'profile' must be 'franka'.")
    asset_variant = _require_str(data, "asset_variant", "root")
    if asset_variant not in ("white", "black"):
        raise ValueError("Config key 'asset_variant' must be 'white' or 'black'.")
    rotation_euler_xyz_deg = _require_vec(pose_data, "rotation_euler_xyz_deg", "pose", 3)
    gravity_compensation_bodies = _require_str(gravity_compensation_data, "bodies", "gravity_compensation")
    if gravity_compensation_bodies != "fingers":
        raise ValueError("Config key 'gravity_compensation.bodies' must be 'fingers'.")
    linear_speed_mps = _require_float(teleop_data, "linear_speed", "teleop") if "linear_speed" in teleop_data else 0.05
    linear_speed_xy_mps = (
        _require_float(teleop_data, "linear_speed_xy", "teleop")
        if "linear_speed_xy" in teleop_data
        else linear_speed_mps
    )
    linear_speed_z_mps = (
        _require_float(teleop_data, "linear_speed_z", "teleop")
        if "linear_speed_z" in teleop_data
        else linear_speed_mps
    )

    return SraGripperConfig(
        enabled=_require_bool(data, "enabled", "root"),
        label=_require_str(data, "label", "root"),
        profile=profile,
        asset_variant=asset_variant,
        pose=PoseConfig(
            position_m=_require_vec(pose_data, "position", "pose", 3),  # type: ignore[arg-type]
            rotation_euler_xyz_deg=rotation_euler_xyz_deg,  # type: ignore[arg-type]
            rotation=_euler_xyz_deg_to_quat(rotation_euler_xyz_deg),
        ),
        finger=FingerConfig(
            density=_require_float(finger_data, "density", "finger"),
            friction=_require_float(finger_data, "friction", "finger"),
        ),
        gap=gap,
        control=GripperControlConfig(
            mode=mode,
            drive_force=_require_float(control_data, "drive_force", "control"),
            stiffness=_require_float(control_data, "stiffness", "control"),
            damping=_require_float(control_data, "damping", "control"),
        ),
        teleop=GripperTeleopConfig(
            enabled=_require_bool(teleop_data, "enabled", "teleop"),
            frame=frame,
            linear_speed_mps=linear_speed_mps,
            linear_speed_xy_mps=linear_speed_xy_mps,
            linear_speed_z_mps=linear_speed_z_mps,
            angular_speed_radps=math.radians(_require_float(teleop_data, "angular_speed_deg", "teleop")),
            gap_speed_mps=_require_float(teleop_data, "gap_speed", "teleop"),
            require_ctrl=require_ctrl,
            modifier=modifier,
        ),
        gravity_compensation=GravityCompensationConfig(
            enabled=_require_bool(gravity_compensation_data, "enabled", "gravity_compensation"),
            bodies=gravity_compensation_bodies,
        ),
    )


def _make_shape_cfg(
    base_cfg: newton.ModelBuilder.ShapeConfig,
    density: float,
    friction: float,
    visible: bool = True,
) -> newton.ModelBuilder.ShapeConfig:
    cfg = base_cfg.copy()
    cfg.density = density
    cfg.mu = friction
    cfg.is_visible = visible
    return cfg


def _euler_xyz_deg_to_quat(value: tuple[float, float, float]) -> wp.quat:
    return _euler_xyz_to_quat([math.radians(float(v)) for v in value])


def _joint_target_for_gap(config: SraGripperConfig, gap_m: float) -> float:
    return 0.5 * float(gap_m)


FRANKA_FINGER_JOINT_Z_M = 0.0584
FRANKA_FINGER_JOINT_LIMIT_MIN_M = 0.0
FRANKA_FINGER_JOINT_LIMIT_MAX_M = 0.04
FRANKA_FINGER_COLLISION_BOXES = (
    ((0.0, 18.5e-3, 11e-3), (0.0, 0.0, 0.0), (22e-3, 15e-3, 20e-3)),
    ((0.0, 6.8e-3, 2.2e-3), (0.0, 0.0, 0.0), (22e-3, 8.8e-3, 3.8e-3)),
    ((0.0, 15.9e-3, 28.35e-3), (0.5235987755982988, 0.0, 0.0), (17.5e-3, 7e-3, 23.5e-3)),
    ((0.0, 7.58e-3, 45.25e-3), (0.0, 0.0, 0.0), (17.5e-3, 15.2e-3, 18.5e-3)),
)
FRANKA_RIGHT_FINGER_COLLISION_RPY = (
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (-0.5235987755982988, 0.0, math.pi),
    (0.0, 0.0, 0.0),
)


def _franka_asset_dir(config: SraGripperConfig) -> Path:
    return (
        Path(newton.utils.download_asset("franka_emika_panda"))
        / "meshes"
        / "robot_ee"
        / f"franka_hand_{config.asset_variant}"
    )


def _try_load_mesh(path: Path, *, label: str) -> Optional[newton.Mesh]:
    try:
        return newton.Mesh.create_from_file(str(path), compute_inertia=False)
    except Exception as exc:  # noqa: BLE001 - optional visual mesh fallback
        print(f"[sra_gripper] Warning: failed to load {label} mesh {path}: {exc}")
        return None


def _make_visual_shape_cfg(base_cfg: newton.ModelBuilder.ShapeConfig) -> newton.ModelBuilder.ShapeConfig:
    cfg = base_cfg.copy()
    cfg.density = 0.0
    cfg.has_shape_collision = False
    cfg.has_particle_collision = False
    return cfg


def _rotate_vector(q: wp.quat, vector_m: tuple[float, float, float]) -> np.ndarray:
    rotation = np.asarray(wp.quat_to_matrix(q), dtype=np.float64).reshape(3, 3)
    return rotation @ np.asarray(vector_m, dtype=np.float64)


def _world_position(
    root_position_m: tuple[float, float, float],
    root_rot: wp.quat,
    local_position_m: tuple[float, float, float],
) -> wp.vec3:
    position = np.asarray(root_position_m, dtype=np.float64) + _rotate_vector(root_rot, local_position_m)
    return wp.vec3(float(position[0]), float(position[1]), float(position[2]))


def _add_franka_finger_shapes(
    builder: newton.ModelBuilder,
    body: int,
    config: SraGripperConfig,
    finger_mesh: Optional[newton.Mesh],
    side: str,
) -> tuple[tuple[int, ...], tuple[int, ...]]:
    if side not in ("left", "right"):
        raise ValueError("Franka finger side must be 'left' or 'right'.")
    visual_cfg = _make_visual_shape_cfg(builder.default_shape_cfg)
    collision_cfg = _make_shape_cfg(
        builder.default_shape_cfg,
        density=config.finger.density,
        friction=config.finger.friction,
        visible=False,
    )
    shape_ids: list[int] = []
    if finger_mesh is not None:
        shape_ids.append(
            builder.add_shape_mesh(
                body=body,
                mesh=finger_mesh,
                cfg=visual_cfg,
                label=f"{config.label}:{side}_finger_visual",
            )
        )
    collision_shape_ids: list[int] = []
    for i, (position_m, left_rpy_rad, size_m) in enumerate(FRANKA_FINGER_COLLISION_BOXES):
        if side == "right":
            rpy_rad = FRANKA_RIGHT_FINGER_COLLISION_RPY[i]
        else:
            rpy_rad = left_rpy_rad
        shape_id = builder.add_shape_box(
            body=body,
            xform=wp.transform(wp.vec3(*position_m), wp.quat_rpy(*rpy_rad)),
            hx=size_m[0] * 0.5,
            hy=size_m[1] * 0.5,
            hz=size_m[2] * 0.5,
            cfg=collision_cfg,
            label=f"{config.label}:{side}_finger_collision_{i}",
        )
        shape_ids.append(shape_id)
        collision_shape_ids.append(shape_id)
    return tuple(shape_ids), tuple(collision_shape_ids)


def add_gripper_from_config(builder: newton.ModelBuilder, config: SraGripperConfig) -> SraGripperBuildResult:
    if config.gap.max_m * 0.5 > FRANKA_FINGER_JOINT_LIMIT_MAX_M:
        raise ValueError("Config key 'gap.max' exceeds the Franka finger joint limit.")
    if config.gap.min_m * 0.5 < FRANKA_FINGER_JOINT_LIMIT_MIN_M:
        raise ValueError("Config key 'gap.min' is below the Franka finger joint limit.")

    asset_dir = _franka_asset_dir(config)
    hand_visual_mesh = _try_load_mesh(asset_dir / "visual" / "hand.dae", label="Franka hand visual")
    hand_collision_mesh = newton.Mesh.create_from_file(str(asset_dir / "collision" / "hand.stl"), compute_inertia=False)
    finger_mesh = _try_load_mesh(asset_dir / "visual" / "finger.dae", label="Franka finger visual")

    root_rot = config.pose.rotation
    root_xform = wp.transform(wp.vec3(*config.pose.position_m), root_rot)
    root_body = builder.add_link(
        xform=root_xform,
        mass=0.0,
        is_kinematic=True,
        label=f"{config.label}:root",
    )
    root_visual_cfg = _make_visual_shape_cfg(builder.default_shape_cfg)
    root_collision_cfg = _make_shape_cfg(
        builder.default_shape_cfg,
        density=0.0,
        friction=config.finger.friction,
        visible=False,
    )
    root_shape_ids: list[int] = []
    if hand_visual_mesh is not None:
        root_shape_ids.append(
            builder.add_shape_mesh(
                body=root_body,
                mesh=hand_visual_mesh,
                cfg=root_visual_cfg,
                label=f"{config.label}:root_visual",
            )
        )
    root_collision_shape = builder.add_shape_mesh(
        body=root_body,
        mesh=hand_collision_mesh,
        cfg=root_collision_cfg,
        label=f"{config.label}:root_collision",
    )
    root_shape_ids.append(root_collision_shape)

    initial_opening = _joint_target_for_gap(config, config.gap.initial_m)
    target_opening = _joint_target_for_gap(config, config.gap.target_m)
    right_finger_rot = root_rot * wp.quat_rpy(0.0, 0.0, math.pi)

    left_body = builder.add_link(
        xform=wp.transform(
            _world_position(config.pose.position_m, root_rot, (0.0, initial_opening, FRANKA_FINGER_JOINT_Z_M)),
            root_rot,
        ),
        mass=0.0,
        label=f"{config.label}:left_finger",
    )
    right_body = builder.add_link(
        xform=wp.transform(
            _world_position(config.pose.position_m, root_rot, (0.0, -initial_opening, FRANKA_FINGER_JOINT_Z_M)),
            right_finger_rot,
        ),
        mass=0.0,
        label=f"{config.label}:right_finger",
    )
    left_shape_ids, left_collision_shape_ids = _add_franka_finger_shapes(builder, left_body, config, finger_mesh, "left")
    right_shape_ids, right_collision_shape_ids = _add_franka_finger_shapes(builder, right_body, config, finger_mesh, "right")

    limit_lower = _joint_target_for_gap(config, config.gap.min_m)
    limit_upper = _joint_target_for_gap(config, config.gap.max_m)
    world_joint = builder.add_joint_fixed(
        parent=-1,
        child=root_body,
        parent_xform=root_xform,
        label=f"{config.label}:world",
    )
    left_joint = builder.add_joint_prismatic(
        parent=root_body,
        child=left_body,
        parent_xform=wp.transform(wp.vec3(0.0, 0.0, FRANKA_FINGER_JOINT_Z_M), wp.quat_identity()),
        axis=newton.Axis.Y,
        target_pos=target_opening,
        target_ke=config.control.stiffness,
        target_kd=config.control.damping,
        effort_limit=config.control.drive_force,
        limit_lower=limit_lower,
        limit_upper=limit_upper,
        actuator_mode=newton.JointTargetMode.POSITION,
        label=f"{config.label}:left_slider",
    )
    right_joint = builder.add_joint_prismatic(
        parent=root_body,
        child=right_body,
        parent_xform=wp.transform(wp.vec3(0.0, 0.0, FRANKA_FINGER_JOINT_Z_M), wp.quat_rpy(0.0, 0.0, math.pi)),
        axis=newton.Axis.Y,
        target_pos=target_opening,
        target_ke=config.control.stiffness,
        target_kd=config.control.damping,
        effort_limit=config.control.drive_force,
        limit_lower=limit_lower,
        limit_upper=limit_upper,
        actuator_mode=newton.JointTargetMode.POSITION,
        label=f"{config.label}:right_slider",
    )
    builder.joint_q[builder.joint_q_start[left_joint]] = initial_opening
    builder.joint_q[builder.joint_q_start[right_joint]] = initial_opening
    builder.add_articulation([world_joint, left_joint, right_joint], label=config.label)

    return SraGripperBuildResult(
        root_body_id=root_body,
        finger_body_ids=(left_body, right_body),
        finger_body_masses=(float(builder.body_mass[left_body]), float(builder.body_mass[right_body])),
        root_shape_ids=tuple(root_shape_ids),
        finger_shape_ids=(left_shape_ids, right_shape_ids),
        finger_collision_shape_ids=(left_collision_shape_ids, right_collision_shape_ids),
        world_joint_id=world_joint,
        prismatic_joint_ids=(left_joint, right_joint),
        prismatic_dof_starts=(builder.joint_qd_start[left_joint], builder.joint_qd_start[right_joint]),
    )


class SraGripperController:
    def __init__(self, config: SraGripperConfig, build_result: SraGripperBuildResult):
        self.config = config
        self.build_result = build_result
        self.teleop_state = SraGripperTeleopState.from_config(config)
        self.position_offset_enabled = False
        self.position_offset_m = [0.0, 0.0, 0.0]
        self.eeframe_a_push_enabled = False
        self.eeframe_a_push_m = 0.0

    def update_from_viewer(self, viewer: Any, dt: float) -> None:
        self.teleop_state.update_from_viewer(viewer, self.config, dt)

    def command_position(self) -> tuple[float, float, float]:
        return (
            float(self.teleop_state.position_m[0]),
            float(self.teleop_state.position_m[1]),
            float(self.teleop_state.position_m[2]),
        )

    def command_euler_xyz_rad(self) -> tuple[float, float, float]:
        return _quat_to_euler_xyz_rad(self.teleop_state.rotation_quat_xyzw)

    def command_gap_m(self) -> float:
        return float(self.teleop_state.target_gap_m)

    def command_position_offset_enabled(self) -> bool:
        return bool(self.position_offset_enabled)

    def command_position_offset_m(self) -> tuple[float, float, float]:
        return (
            float(self.position_offset_m[0]),
            float(self.position_offset_m[1]),
            float(self.position_offset_m[2]),
        )

    def set_position_offset_enabled(self, enabled: bool) -> None:
        self.position_offset_enabled = bool(enabled)

    def set_position_offset_m(self, offset_m: tuple[float, float, float] | list[float]) -> None:
        if len(offset_m) != 3:
            raise ValueError("Gripper position offset must contain exactly three values.")
        self.position_offset_m = [float(offset_m[0]), float(offset_m[1]), float(offset_m[2])]

    def command_eeframe_a_push_enabled(self) -> bool:
        return bool(self.eeframe_a_push_enabled)

    def command_eeframe_a_push_m(self) -> float:
        return float(self.eeframe_a_push_m)

    def set_eeframe_a_push_enabled(self, enabled: bool) -> None:
        self.eeframe_a_push_enabled = bool(enabled)

    def set_eeframe_a_push_m(self, distance_m: float) -> None:
        self.eeframe_a_push_m = float(distance_m)

    def command_translation_frame(self) -> str:
        return str(self.teleop_state.translation_frame)

    def set_translation_frame(self, frame: str) -> None:
        if frame not in ("world", "eeframe"):
            raise ValueError(f"Unsupported gripper translation frame '{frame}'. Expected 'world' or 'eeframe'.")
        self.teleop_state.translation_frame = str(frame)

    def set_command(
        self,
        position_m: tuple[float, float, float],
        euler_xyz_rad: tuple[float, float, float],
        target_gap_m: float,
    ) -> None:
        self.teleop_state.position_m = [float(position_m[0]), float(position_m[1]), float(position_m[2])]
        self.teleop_state.rotation_quat_xyzw = list(_wp_quat_to_tuple(_euler_xyz_to_quat(list(euler_xyz_rad))))
        self.teleop_state.target_gap_m = min(max(float(target_gap_m), self.config.gap.min_m), self.config.gap.max_m)

    def apply(
        self,
        state: newton.State,
        control: newton.Control,
        gravity: tuple[float, float, float] | None = None,
    ) -> None:
        body_q = state.body_q.numpy()
        root_xform = self.teleop_state.root_transform()
        position_m = np.asarray(self.teleop_state.position_m, dtype=np.float32)
        if self.position_offset_enabled:
            position_m = position_m + np.asarray(self.position_offset_m, dtype=np.float32)
        if self.eeframe_a_push_enabled and self.eeframe_a_push_m != 0.0:
            root_q = np.asarray([root_xform.q[0], root_xform.q[1], root_xform.q[2], root_xform.q[3]], dtype=np.float64)
            a_push_world = rotate_vector_by_quat(
                root_q,
                np.asarray((-float(self.eeframe_a_push_m), 0.0, 0.0), dtype=np.float64),
            )
            position_m = position_m + a_push_world.astype(np.float32)
        body_q[self.build_result.root_body_id, :3] = position_m
        q = root_xform.q
        body_q[self.build_result.root_body_id, 3:] = np.array([q[0], q[1], q[2], q[3]], dtype=np.float32)

        target = _joint_target_for_gap(self.config, self.teleop_state.target_gap_m)
        root_q = np.asarray([q[0], q[1], q[2], q[3]], dtype=np.float64)
        left_position_m = position_m.astype(np.float64) + rotate_vector_by_quat(
            root_q,
            np.asarray((0.0, target, FRANKA_FINGER_JOINT_Z_M), dtype=np.float64),
        )
        right_position_m = position_m.astype(np.float64) + rotate_vector_by_quat(
            root_q,
            np.asarray((0.0, -target, FRANKA_FINGER_JOINT_Z_M), dtype=np.float64),
        )
        right_q = q * wp.quat_rpy(0.0, 0.0, math.pi)
        body_q[self.build_result.finger_body_ids[0], :3] = left_position_m.astype(np.float32)
        body_q[self.build_result.finger_body_ids[0], 3:] = np.array([q[0], q[1], q[2], q[3]], dtype=np.float32)
        body_q[self.build_result.finger_body_ids[1], :3] = right_position_m.astype(np.float32)
        body_q[self.build_result.finger_body_ids[1], 3:] = np.array(
            [right_q[0], right_q[1], right_q[2], right_q[3]],
            dtype=np.float32,
        )
        state.body_q.assign(body_q)

        body_qd = state.body_qd.numpy()
        body_qd[self.build_result.root_body_id, :] = 0.0
        body_qd[self.build_result.finger_body_ids[0], :] = 0.0
        body_qd[self.build_result.finger_body_ids[1], :] = 0.0
        state.body_qd.assign(body_qd)

        target_pos = control.joint_target_pos.numpy()
        for dof_start in self.build_result.prismatic_dof_starts:
            target_pos[dof_start] = target
        control.joint_target_pos.assign(target_pos)

        if self.config.gravity_compensation.enabled:
            if gravity is None:
                raise ValueError("Gravity compensation requires a gravity vector.")
            gravity_vec = np.asarray(gravity, dtype=np.float32)
            if gravity_vec.shape != (3,):
                raise ValueError("Gravity compensation requires a 3D gravity vector.")
            body_f = state.body_f.numpy()
            for body_id, body_mass in zip(
                self.build_result.finger_body_ids,
                self.build_result.finger_body_masses,
                strict=True,
            ):
                body_f[body_id, :3] += -float(body_mass) * gravity_vec
            state.body_f.assign(body_f)


def _euler_xyz_to_quat(euler_xyz_rad: list[float]) -> wp.quat:
    roll, pitch, yaw = euler_xyz_rad
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return wp.normalize(
        wp.quat(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def _load_box_config(data: dict[str, Any], key_path: str) -> GraspObjectBoxConfig:
    _require_keys(data, ("position", "size", "density", "friction"), key_path)
    return GraspObjectBoxConfig(
        position_m=_require_vec(data, "position", key_path, 3),  # type: ignore[arg-type]
        size_m=_require_positive_vec3(data, "size", key_path),
        density=_require_float(data, "density", key_path),
        friction=_require_float(data, "friction", key_path),
    )


def _load_capsule_config(data: dict[str, Any], key_path: str) -> GraspObjectCapsuleConfig:
    _require_keys(data, ("position", "radius", "half_height", "density", "friction"), key_path)
    radius = _require_float(data, "radius", key_path)
    half_height = _require_float(data, "half_height", key_path)
    if radius <= 0.0:
        raise ValueError(f"Config key '{key_path}.radius' must be positive.")
    if half_height <= 0.0:
        raise ValueError(f"Config key '{key_path}.half_height' must be positive.")
    return GraspObjectCapsuleConfig(
        position_m=_require_vec(data, "position", key_path, 3),  # type: ignore[arg-type]
        radius_m=radius,
        half_height_m=half_height,
        density=_require_float(data, "density", key_path),
        friction=_require_float(data, "friction", key_path),
    )


def _load_solver_config(data: dict[str, Any], key_path: str) -> GraspControlSolverConfig:
    _require_keys(data, ("iterations", "friction_epsilon", "rigid_contact_k_start"), key_path)
    iterations = _require_float(data, "iterations", key_path)
    if not float(iterations).is_integer():
        raise ValueError(f"Config key '{key_path}.iterations' must be an integer.")
    if iterations <= 0.0:
        raise ValueError(f"Config key '{key_path}.iterations' must be positive.")
    friction_epsilon = _require_float(data, "friction_epsilon", key_path)
    if friction_epsilon <= 0.0:
        raise ValueError(f"Config key '{key_path}.friction_epsilon' must be positive.")
    rigid_contact_k_start = _require_float(data, "rigid_contact_k_start", key_path)
    if rigid_contact_k_start <= 0.0:
        raise ValueError(f"Config key '{key_path}.rigid_contact_k_start' must be positive.")
    return GraspControlSolverConfig(
        iterations=int(iterations),
        friction_epsilon=friction_epsilon,
        rigid_contact_k_start=rigid_contact_k_start,
    )


def _load_contact_config(data: dict[str, Any], key_path: str) -> GraspControlContactConfig:
    _require_keys(data, ("rigid_contact_margin", "rigid_gap"), key_path)
    margin = _require_float(data, "rigid_contact_margin", key_path)
    if margin < 0.0:
        raise ValueError(f"Config key '{key_path}.rigid_contact_margin' must be non-negative.")
    gap = _require_float(data, "rigid_gap", key_path)
    if gap < 0.0:
        raise ValueError(f"Config key '{key_path}.rigid_gap' must be non-negative.")
    return GraspControlContactConfig(rigid_contact_margin_m=margin, rigid_gap_m=gap)


def load_grasp_control_config(config_path: str | Path) -> GraspControlSceneConfig:
    config_file = Path(config_path).resolve()
    if not config_file.is_file():
        raise ValueError(f"Config file does not exist: {config_file}")
    data = _load_yaml_mapping(config_file)
    _require_keys(data, ("gripper_config_path", "solver", "contact", "object", "ground"), "root")
    object_data = _require_mapping(data["object"], "object")
    _require_keys(object_data, ("cuboid", "capsule"), "object")
    gripper_path = Path(_require_str(data, "gripper_config_path", "root"))
    return GraspControlSceneConfig(
        gripper_config_path=gripper_path,
        solver=_load_solver_config(_require_mapping(data["solver"], "solver"), "solver"),
        contact=_load_contact_config(_require_mapping(data["contact"], "contact"), "contact"),
        cuboid=_load_box_config(_require_mapping(object_data["cuboid"], "object.cuboid"), "object.cuboid"),
        capsule=_load_capsule_config(_require_mapping(object_data["capsule"], "object.capsule"), "object.capsule"),
        ground=_load_box_config(_require_mapping(data["ground"], "ground"), "ground"),
    )


def build_grasp_control_scene(
    builder: newton.ModelBuilder,
    scene_config: GraspControlSceneConfig,
    gripper_config: SraGripperConfig,
) -> GraspControlBuildResult:
    ground_cfg = _make_shape_cfg(builder.default_shape_cfg, density=0.0, friction=scene_config.ground.friction)
    ground_shape = builder.add_shape_box(
        body=-1,
        xform=wp.transform(wp.vec3(*scene_config.ground.position_m), wp.quat_identity()),
        hx=scene_config.ground.size_m[0] * 0.5,
        hy=scene_config.ground.size_m[1] * 0.5,
        hz=scene_config.ground.size_m[2] * 0.5,
        cfg=ground_cfg,
        label="grasp_control:ground",
    )

    cuboid_cfg = _make_shape_cfg(builder.default_shape_cfg, scene_config.cuboid.density, scene_config.cuboid.friction)
    cuboid_body = builder.add_link(
        xform=wp.transform(wp.vec3(*scene_config.cuboid.position_m), wp.quat_identity()),
        mass=0.0,
        label="grasp_control:cuboid",
    )
    cuboid_shape = builder.add_shape_box(
        body=cuboid_body,
        hx=scene_config.cuboid.size_m[0] * 0.5,
        hy=scene_config.cuboid.size_m[1] * 0.5,
        hz=scene_config.cuboid.size_m[2] * 0.5,
        cfg=cuboid_cfg,
        label="grasp_control:cuboid_shape",
    )
    capsule_cfg = _make_shape_cfg(builder.default_shape_cfg, scene_config.capsule.density, scene_config.capsule.friction)
    capsule_body = builder.add_link(
        xform=wp.transform(wp.vec3(*scene_config.capsule.position_m), wp.quat_identity()),
        mass=0.0,
        label="grasp_control:capsule",
    )
    capsule_shape = builder.add_shape_capsule(
        body=capsule_body,
        radius=scene_config.capsule.radius_m,
        half_height=scene_config.capsule.half_height_m,
        cfg=capsule_cfg,
        label="grasp_control:capsule_shape",
    )
    gripper = add_gripper_from_config(builder, gripper_config)
    return GraspControlBuildResult(
        gripper=gripper,
        object_body_ids=(cuboid_body, capsule_body),
        object_shape_ids=(cuboid_shape, capsule_shape),
        ground_shape_id=ground_shape,
    )
