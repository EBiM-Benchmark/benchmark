# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""USD BasisCurves cable importer with explicit PhysicsFixedJoint heads.

This importer parses a BasisCurves cable description and builds Newton rigid cable segments
with optional mesh heads connected by fixed joints (scheme 2):

1. Cable segments are created with ``ModelBuilder.add_rod_graph`` (capsule segments + cable joints).
2. Head parts are authored by explicit ``PhysicsFixedJoint`` prims.
3. Each head body is connected to the cable by the authored fixed-joint frame.
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass

# Work around an OpenUSD thread-safety crash in UsdPhysics.LoadUsdPhysicsFromRange
# when parsing rigid bodies with many mesh colliders. This must be set before any
# pxr module initializes its thread pool.
os.environ.setdefault("PXR_WORK_THREAD_LIMIT", "1")

import numpy as np
import warp as wp

import newton

try:
    from pxr import Usd, UsdGeom  # type: ignore
except ImportError as exc:
    raise ImportError("This importer requires USD Python bindings (`pxr`).") from exc


HEAD_SHAPE_MODES = ("mesh", "convex_hull", "visual_mesh_with_convex_proxy", "visual_mesh_with_sdf_proxy")
HEAD_SDF_MAX_RESOLUTION = 64


@dataclass
class CableCurveImportResult:
    cable_body_ids: list[int]
    cable_joint_ids: list[int]
    head_body_ids: list[int]
    head_body_prim_paths: list[str]
    head_fixed_joint_ids: list[int]
    fixed_body_ids: list[int]
    source_points_m: np.ndarray
    edges: list[tuple[int, int]]
    radius_m: float
    curve_prim_path: str


@dataclass
class _AuthoredFixedJoint:
    prim_path: str
    body0_path: str
    body1_path: str
    local_pos0_m: wp.vec3
    local_pos1_m: wp.vec3
    local_rot0: wp.quat
    local_rot1: wp.quat


def _uses_direct_cable_joint_stiffness() -> bool:
    """Newton 1.2.0 stores add_rod_graph stiffness directly on cable joint DOFs."""
    return getattr(newton, "__version__", "") == "1.2.0"


def _restore_segment_length_scaled_cable_stiffness(
    builder: newton.ModelBuilder,
    *,
    cable_body_ids: list[int],
    cable_joint_ids: list[int],
    edge_lengths: list[float],
    stretch_stiffness: float,
    bend_stiffness: float,
) -> None:
    body_to_edge_idx = {int(body_id): edge_idx for edge_idx, body_id in enumerate(cable_body_ids)}

    for joint_id in cable_joint_ids:
        parent_body = int(builder.joint_parent[joint_id])
        child_body = int(builder.joint_child[joint_id])
        edge_indices = [
            edge_idx
            for edge_idx in (body_to_edge_idx.get(parent_body), body_to_edge_idx.get(child_body))
            if edge_idx is not None
        ]
        if len(edge_indices) == 0:
            raise RuntimeError(f"Cable joint {joint_id} does not reference a generated cable segment body.")

        # Match the dev add_rod_graph effective stiffness: ke is scaled by the
        # average length of the two adjacent cable segments that form the joint.
        segment_length = float(sum(edge_lengths[edge_idx] for edge_idx in edge_indices) / len(edge_indices))
        if segment_length <= 0.0:
            raise RuntimeError(f"Cable joint {joint_id} has invalid segment length {segment_length}.")

        dof_start = int(builder.joint_qd_start[joint_id])
        builder.joint_target_ke[dof_start] = float(stretch_stiffness) / segment_length
        builder.joint_target_ke[dof_start + 1] = float(bend_stiffness) / segment_length


def _as_matrix_world(prim: "Usd.Prim") -> np.ndarray:  # type: ignore[name-defined]
    xformable = UsdGeom.Xformable(prim)
    mat = np.asarray(xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=np.float64).reshape(4, 4)
    return mat


def _transform_points_rowvec(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    pts_h = np.concatenate([pts, np.ones((pts.shape[0], 1), dtype=np.float64)], axis=1)
    pts_out = pts_h @ np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    return pts_out[:, :3].astype(np.float32, copy=False)


def _fan_triangulate_faces(face_counts: np.ndarray, face_indices: np.ndarray) -> np.ndarray:
    counts = np.asarray(face_counts, dtype=np.int32).reshape(-1)
    indices = np.asarray(face_indices, dtype=np.int32).reshape(-1)
    if counts.size == 0:
        return np.zeros((0,), dtype=np.int32)
    if int(np.sum(counts)) != int(indices.shape[0]):
        raise ValueError("Invalid face data: sum(face_counts) does not match face index count.")

    tris: list[int] = []
    offset = 0
    for n in counts:
        n = int(n)
        if n < 3:
            offset += n
            continue
        base = int(indices[offset])
        for i in range(n - 2):
            i1 = int(indices[offset + i + 1])
            i2 = int(indices[offset + i + 2])
            tris.extend([base, i1, i2])
        offset += n
    return np.asarray(tris, dtype=np.int32)


def _find_mesh_prims(root_prim: "Usd.Prim") -> list["Usd.Prim"]:  # type: ignore[name-defined]
    mesh_prims: list["Usd.Prim"] = []
    stack = [root_prim]
    while stack:
        prim = stack.pop()
        if prim.GetTypeName() == "Mesh":
            mesh_prims.append(prim)
        children = list(prim.GetChildren())
        stack.extend(reversed(children))
    return mesh_prims


def _meters_per_unit(stage: "Usd.Stage") -> float:  # type: ignore[name-defined]
    if UsdGeom.StageHasAuthoredMetersPerUnit(stage):
        return float(UsdGeom.GetStageMetersPerUnit(stage))
    return 1.0


def _get_world_transform_m(prim: "Usd.Prim", meters_per_unit: float) -> wp.transform:  # type: ignore[name-defined]
    mat = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=np.float32)
    pos, rot, _ = wp.transform_decompose(wp.mat44(mat.T))
    pos_m = wp.vec3(
        float(pos[0]) * meters_per_unit,
        float(pos[1]) * meters_per_unit,
        float(pos[2]) * meters_per_unit,
    )
    return wp.transform(pos_m, wp.normalize(rot))


def _get_single_relationship_target(prim: "Usd.Prim", rel_name: str) -> str:  # type: ignore[name-defined]
    rel = prim.GetRelationship(rel_name)
    if not rel:
        raise ValueError(f"Joint '{prim.GetPath()}' is missing relationship '{rel_name}'.")
    targets = rel.GetTargets()
    if len(targets) != 1:
        raise ValueError(
            f"Joint '{prim.GetPath()}' relationship '{rel_name}' must contain exactly one target, got {len(targets)}."
        )
    return str(targets[0])


def _get_required_vec3_attr_m(prim: "Usd.Prim", attr_name: str, meters_per_unit: float) -> wp.vec3:  # type: ignore[name-defined]
    attr = prim.GetAttribute(attr_name)
    if not attr or not attr.HasAuthoredValue():
        raise ValueError(f"Joint '{prim.GetPath()}' is missing required attribute '{attr_name}'.")
    val = attr.Get()
    vec = np.asarray(val, dtype=np.float64).reshape(3)
    if not np.isfinite(vec).all():
        raise ValueError(f"Joint '{prim.GetPath()}' attribute '{attr_name}' contains non-finite values: {val}.")
    return wp.vec3(
        float(vec[0]) * meters_per_unit,
        float(vec[1]) * meters_per_unit,
        float(vec[2]) * meters_per_unit,
    )


def _get_required_quat_attr(prim: "Usd.Prim", attr_name: str) -> wp.quat:  # type: ignore[name-defined]
    attr = prim.GetAttribute(attr_name)
    if not attr or not attr.HasAuthoredValue():
        raise ValueError(f"Joint '{prim.GetPath()}' is missing required attribute '{attr_name}'.")
    val = attr.Get()
    q = wp.quat(float(val.imaginary[0]), float(val.imaginary[1]), float(val.imaginary[2]), float(val.real))
    qn = wp.normalize(q)
    if float(wp.length(qn)) <= 0.0:
        raise ValueError(f"Joint '{prim.GetPath()}' attribute '{attr_name}' is invalid quaternion: {val}.")
    return qn


def _collect_authored_fixed_joints_for_curve(
    stage: "Usd.Stage",  # type: ignore[name-defined]
    curve_parent_path: str,
    meters_per_unit: float,
) -> list[_AuthoredFixedJoint]:
    joints: list[_AuthoredFixedJoint] = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "PhysicsFixedJoint":
            continue

        body0_path = _get_single_relationship_target(prim, "physics:body0")
        if body0_path != curve_parent_path:
            continue

        body1_path = _get_single_relationship_target(prim, "physics:body1")
        local_pos0_m = _get_required_vec3_attr_m(prim, "physics:localPos0", meters_per_unit)
        local_pos1_m = _get_required_vec3_attr_m(prim, "physics:localPos1", meters_per_unit)
        local_rot0 = _get_required_quat_attr(prim, "physics:localRot0")
        local_rot1 = _get_required_quat_attr(prim, "physics:localRot1")

        joints.append(
            _AuthoredFixedJoint(
                prim_path=str(prim.GetPath()),
                body0_path=body0_path,
                body1_path=body1_path,
                local_pos0_m=local_pos0_m,
                local_pos1_m=local_pos1_m,
                local_rot0=local_rot0,
                local_rot1=local_rot1,
            )
        )
    return joints


def _load_mesh_from_body_prim(
    stage: "Usd.Stage",  # type: ignore[name-defined]
    body_prim_path: str,
    meters_per_unit: float,
) -> tuple[newton.Mesh, "Usd.Prim"]:  # type: ignore[name-defined]
    body_prim = stage.GetPrimAtPath(body_prim_path)
    if not body_prim or not body_prim.IsValid():
        raise ValueError(f"Invalid body prim path '{body_prim_path}' for fixed-joint head attachment.")

    mesh_prims = _find_mesh_prims(body_prim)
    if len(mesh_prims) == 0:
        raise ValueError(f"No Mesh prim found under body prim '{body_prim_path}'.")

    collision_mesh_prims = [prim for prim in mesh_prims if "/Collisions/" in str(prim.GetPath())]
    selected_mesh_prims = collision_mesh_prims if len(collision_mesh_prims) > 0 else mesh_prims

    body_world = _as_matrix_world(body_prim)
    world_body = np.linalg.inv(body_world)
    merged_points_local: list[np.ndarray] = []
    merged_tri_indices: list[np.ndarray] = []
    vertex_offset = 0

    for mesh_prim in selected_mesh_prims:
        mesh = UsdGeom.Mesh(mesh_prim)
        points_raw = mesh.GetPointsAttr().Get()
        face_indices_raw = mesh.GetFaceVertexIndicesAttr().Get()
        face_counts_raw = mesh.GetFaceVertexCountsAttr().Get()
        if points_raw is None or face_indices_raw is None or face_counts_raw is None:
            raise ValueError(f"Mesh '{mesh_prim.GetPath()}' is missing points/faces data.")

        mesh_world = _as_matrix_world(mesh_prim)
        mesh_body = mesh_world @ world_body

        points_local = _transform_points_rowvec(np.asarray(points_raw, dtype=np.float32), mesh_body)
        tri_indices = _fan_triangulate_faces(np.asarray(face_counts_raw), np.asarray(face_indices_raw))
        if tri_indices.shape[0] == 0:
            raise ValueError(f"Mesh '{mesh_prim.GetPath()}' has no triangulated faces.")

        merged_points_local.append(points_local)
        merged_tri_indices.append(tri_indices + vertex_offset)
        vertex_offset += int(points_local.shape[0])

    points_local_merged = np.concatenate(merged_points_local, axis=0)
    tri_indices_merged = np.concatenate(merged_tri_indices, axis=0)
    points_local_m = (points_local_merged * meters_per_unit).astype(np.float32, copy=False)
    return newton.Mesh(points_local_m, tri_indices_merged), body_prim


def load_usd_mesh_prim_relative_to_body(
    source_usd_path: str,
    *,
    mesh_prim_path: str,
    body_prim_path: str,
) -> newton.Mesh:
    """Load a USD Mesh prim as a Newton mesh in the local frame of a body prim."""
    stage = Usd.Stage.Open(source_usd_path)
    if stage is None:
        raise ValueError(f"Failed to open USD file '{source_usd_path}'.")

    body_prim = stage.GetPrimAtPath(body_prim_path)
    if not body_prim or not body_prim.IsValid():
        raise ValueError(f"Invalid body prim path '{body_prim_path}' in USD file '{source_usd_path}'.")

    mesh_prim = stage.GetPrimAtPath(mesh_prim_path)
    if not mesh_prim or not mesh_prim.IsValid():
        raise ValueError(f"Invalid mesh prim path '{mesh_prim_path}' in USD file '{source_usd_path}'.")
    if mesh_prim.GetTypeName() != "Mesh":
        raise ValueError(f"USD prim '{mesh_prim_path}' must be a Mesh, got '{mesh_prim.GetTypeName()}'.")

    mesh = UsdGeom.Mesh(mesh_prim)
    points_raw = mesh.GetPointsAttr().Get()
    face_indices_raw = mesh.GetFaceVertexIndicesAttr().Get()
    face_counts_raw = mesh.GetFaceVertexCountsAttr().Get()
    if points_raw is None or face_indices_raw is None or face_counts_raw is None:
        raise ValueError(f"Mesh '{mesh_prim_path}' is missing points/faces data.")

    meters_per_unit = _meters_per_unit(stage)
    body_world = _as_matrix_world(body_prim)
    world_body = np.linalg.inv(body_world)
    mesh_world = _as_matrix_world(mesh_prim)
    mesh_body = mesh_world @ world_body

    points_local = _transform_points_rowvec(np.asarray(points_raw, dtype=np.float32), mesh_body)
    tri_indices = _fan_triangulate_faces(np.asarray(face_counts_raw), np.asarray(face_indices_raw))
    if tri_indices.shape[0] == 0:
        raise ValueError(f"Mesh '{mesh_prim_path}' has no triangulated faces.")

    points_local_m = (points_local * float(meters_per_unit)).astype(np.float32, copy=False)
    return newton.Mesh(points_local_m, tri_indices)


def _match_anchor_to_edge(
    anchor_world_m: np.ndarray,
    points_m: np.ndarray,
    edges: list[tuple[int, int]],
) -> tuple[int, float, float]:
    best_edge_idx = -1
    best_t = 0.0
    best_dist = float("inf")

    for e_idx, (u, v) in enumerate(edges):
        p0 = points_m[u]
        p1 = points_m[v]
        d = p1 - p0
        d2 = float(np.dot(d, d))
        if d2 <= 0.0:
            raise ValueError(f"Edge ({u}, {v}) has zero length while matching fixed-joint anchor.")

        t = float(np.dot(anchor_world_m - p0, d) / d2)
        if t < 0.0:
            t = 0.0
        if t > 1.0:
            t = 1.0
        proj = p0 + t * d
        dist = float(np.linalg.norm(anchor_world_m - proj))
        if dist < best_dist:
            best_dist = dist
            best_t = t
            best_edge_idx = e_idx

    if best_edge_idx < 0:
        raise RuntimeError("Internal error: failed to match fixed-joint anchor to any cable edge.")
    return best_edge_idx, best_t, best_dist


def _quat_from_segment_direction(direction: np.ndarray) -> wp.quat:
    d = np.asarray(direction, dtype=np.float64).reshape(3)
    n = np.linalg.norm(d)
    if n <= 0.0:
        raise ValueError("Cannot build orientation from zero-length direction vector.")
    d = d / n
    q = wp.quat_between_vectors(wp.vec3(0.0, 0.0, 1.0), wp.vec3(float(d[0]), float(d[1]), float(d[2])))
    return wp.normalize(q)


def add_cable_from_usd_curve(
    builder: newton.ModelBuilder,
    source_usd_path: str,
    curve_prim_path: str = "/World/cable/curve_0",
    *,
    cable_label: str | None = None,
    cable_cfg: newton.ShapeConfig | None = None,
    stretch_stiffness: float = 1.0e9,
    stretch_damping: float = 0.0,
    bend_stiffness: float = 0.0,
    bend_damping: float = 0.0,
    wrap_in_articulation: bool = True,
    head_shape_mode: str = "mesh",
    head_cfg: newton.ShapeConfig | None = None,
    head_mass: float = 0.0,
) -> CableCurveImportResult:
    """Import a BasisCurves cable with optional rigid mesh heads.

    Args:
        builder: Target model builder to mutate in-place.
        source_usd_path: USD file containing the cable BasisCurves prim.
        curve_prim_path: Prim path to the BasisCurves object.
        cable_label: Label prefix for cable bodies/joints.
        cable_cfg: Shape config for cable capsule segments.
        stretch_stiffness: Cable stretch stiffness [N/m].
        stretch_damping: Cable stretch damping.
        bend_stiffness: Cable bend stiffness [N*m].
        bend_damping: Cable bend damping.
        wrap_in_articulation: Whether to wrap cable joints in articulation(s).
        head_shape_mode: Either ``"mesh"``, ``"convex_hull"``,
            ``"visual_mesh_with_convex_proxy"``, or ``"visual_mesh_with_sdf_proxy"`` for head shapes.
        head_cfg: Shape config for head meshes.
        head_mass: Initial mass [kg] for each head body before shape mass contribution.

    Returns:
        Imported cable/head body and joint indices.
    """
    if head_shape_mode not in HEAD_SHAPE_MODES:
        raise ValueError(f"Unsupported head_shape_mode '{head_shape_mode}'. Expected one of {HEAD_SHAPE_MODES}.")

    stage = Usd.Stage.Open(source_usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open curve USD stage: '{source_usd_path}'.")

    curve_prim = stage.GetPrimAtPath(curve_prim_path)
    if not curve_prim or not curve_prim.IsValid():
        raise ValueError(f"Curve prim '{curve_prim_path}' is not valid in stage '{source_usd_path}'.")
    if curve_prim.GetTypeName() != "BasisCurves":
        raise ValueError(f"Prim '{curve_prim_path}' is type '{curve_prim.GetTypeName()}', expected 'BasisCurves'.")

    points_attr = curve_prim.GetAttribute("points")
    points_raw = points_attr.Get() if points_attr else None
    if points_raw is None:
        raise ValueError(f"BasisCurves '{curve_prim_path}' is missing 'points'.")

    widths_attr = curve_prim.GetAttribute("widths")
    widths_raw = widths_attr.Get() if widths_attr else None
    if widths_raw is None:
        raise ValueError(f"BasisCurves '{curve_prim_path}' is missing 'widths'.")

    connections_attr = curve_prim.GetAttribute("connections")
    connections_raw = connections_attr.Get() if connections_attr else None
    fixed_points_attr = curve_prim.GetAttribute("fixed_points")
    fixed_points_raw = fixed_points_attr.Get() if fixed_points_attr else None

    points = np.asarray(points_raw, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] < 2:
        raise ValueError(f"BasisCurves '{curve_prim_path}' must contain at least 2 points.")

    mpu = _meters_per_unit(stage)
    points_m = points * float(mpu)

    widths = np.asarray(widths_raw, dtype=np.float64).reshape(-1)
    if widths.shape[0] == 0:
        raise ValueError(f"BasisCurves '{curve_prim_path}' has empty 'widths'.")
    if float(np.min(widths)) <= 0.0:
        raise ValueError(f"BasisCurves '{curve_prim_path}' has non-positive width values.")
    radius_m = float(widths[0]) * 0.5 * float(mpu)

    edges: list[tuple[int, int]] = []
    if connections_raw is None:
        for i in range(points_m.shape[0] - 1):
            edges.append((i, i + 1))
    else:
        for pair in connections_raw:
            u = int(pair[0])
            v = int(pair[1])
            if u < 0 or u >= points_m.shape[0] or v < 0 or v >= points_m.shape[0]:
                raise ValueError(f"Connection ({u}, {v}) is out of point range [0, {points_m.shape[0] - 1}].")
            if u == v:
                raise ValueError(f"Connection ({u}, {v}) is invalid (self-edge).")
            edges.append((u, v))
    if len(edges) == 0:
        raise ValueError(f"BasisCurves '{curve_prim_path}' has no valid connections.")

    fixed_point_indices: list[int] = []
    if fixed_points_raw is not None:
        fixed_points = np.asarray(fixed_points_raw, dtype=np.int64).reshape(-1)
        for idx_raw in fixed_points:
            idx = int(idx_raw)
            if idx < 0 or idx >= points_m.shape[0]:
                raise ValueError(
                    f"Fixed point index {idx} in '{curve_prim_path}' is out of point range [0, {points_m.shape[0] - 1}]."
                )
            fixed_point_indices.append(idx)

    edge_lengths: list[float] = []
    for (u, v) in edges:
        seg = points_m[v] - points_m[u]
        length = float(np.linalg.norm(seg))
        if length <= 0.0:
            raise ValueError(f"Edge ({u}, {v}) has zero length.")
        edge_lengths.append(length)

    node_positions_wp = [wp.vec3(float(p[0]), float(p[1]), float(p[2])) for p in points_m]
    rod_cfg = builder.default_shape_cfg if cable_cfg is None else cable_cfg
    cable_body_ids, cable_joint_ids = builder.add_rod_graph(
        node_positions=node_positions_wp,
        edges=edges,
        radius=radius_m,
        cfg=rod_cfg,
        stretch_stiffness=stretch_stiffness,
        stretch_damping=stretch_damping,
        bend_stiffness=bend_stiffness,
        bend_damping=bend_damping,
        label=cable_label,
        wrap_in_articulation=wrap_in_articulation,
    )
    cable_body_ids = [int(v) for v in cable_body_ids]
    cable_joint_ids = [int(v) for v in cable_joint_ids]
    if _uses_direct_cable_joint_stiffness():
        _restore_segment_length_scaled_cable_stiffness(
            builder,
            cable_body_ids=cable_body_ids,
            cable_joint_ids=cable_joint_ids,
            edge_lengths=edge_lengths,
            stretch_stiffness=stretch_stiffness,
            bend_stiffness=bend_stiffness,
        )

    fixed_body_ids: list[int] = []
    if len(fixed_point_indices) > 0:
        fixed_points_set = set(fixed_point_indices)
        fixed_body_seen: set[int] = set()
        for edge_idx, (u, v) in enumerate(edges):
            if u in fixed_points_set or v in fixed_points_set:
                body_id = int(cable_body_ids[edge_idx])
                if body_id not in fixed_body_seen:
                    fixed_body_seen.add(body_id)
                    fixed_body_ids.append(body_id)

    head_body_ids: list[int] = []
    head_body_prim_paths: list[str] = []
    head_fixed_joint_ids: list[int] = []

    curve_parent_path = str(curve_prim.GetParent().GetPath())
    authored_fixed_joints = _collect_authored_fixed_joints_for_curve(stage, curve_parent_path, float(mpu))

    head_models_attr = curve_prim.GetAttribute("cable_head_models")
    has_legacy_head_schema = bool(head_models_attr and head_models_attr.HasAuthoredValue())
    if has_legacy_head_schema and len(authored_fixed_joints) == 0:
        raise ValueError(
            "Legacy head schema 'cable_head_models/cable_head_ranges_i' is no longer supported. "
            "Please author explicit PhysicsFixedJoint entries with physics:body0/body1/localPos/localRot."
        )

    head_shape_cfg = builder.default_shape_cfg if head_cfg is None else head_cfg
    curve_base_label = cable_label if cable_label is not None else str(curve_prim.GetPath())

    attach_tolerance_m = max(2.0 * radius_m, 1.0e-4)
    body1_to_head_body: dict[str, int] = {}

    for authored_idx, authored in enumerate(authored_fixed_joints):
        head_body = body1_to_head_body.get(authored.body1_path)
        if head_body is None:
            head_mesh, body1_prim = _load_mesh_from_body_prim(stage, authored.body1_path, float(mpu))
            head_xform = _get_world_transform_m(body1_prim, float(mpu))
            head_label = f"{curve_base_label}:authored_head_{authored_idx}:{authored.body1_path}"
            head_body = builder.add_link(xform=head_xform, mass=head_mass, label=head_label)
            head_body_ids.append(head_body)
            head_body_prim_paths.append(authored.body1_path)
            body1_to_head_body[authored.body1_path] = head_body

            head_shape_label = f"{head_label}:shape"
            if head_shape_mode == "mesh":
                builder.add_shape_mesh(
                    body=head_body,
                    xform=wp.transform(),
                    mesh=head_mesh,
                    scale=wp.vec3(1.0, 1.0, 1.0),
                    cfg=head_shape_cfg,
                    label=head_shape_label,
                )
            elif head_shape_mode == "convex_hull":
                builder.add_shape_convex_hull(
                    body=head_body,
                    xform=wp.transform(),
                    mesh=head_mesh,
                    scale=wp.vec3(1.0, 1.0, 1.0),
                    cfg=head_shape_cfg,
                    label=head_shape_label,
                )
            elif head_shape_mode == "visual_mesh_with_convex_proxy":
                visual_cfg = head_shape_cfg.copy()
                visual_cfg.density = 0.0
                visual_cfg.has_shape_collision = False
                visual_cfg.has_particle_collision = False
                visual_cfg.is_visible = True
                collision_proxy_cfg = head_shape_cfg.copy()
                collision_proxy_cfg.is_visible = False
                collision_proxy_cfg.has_shape_collision = True
                builder.add_shape_mesh(
                    body=head_body,
                    xform=wp.transform(),
                    mesh=head_mesh,
                    scale=wp.vec3(1.0, 1.0, 1.0),
                    cfg=visual_cfg,
                    label=f"{head_label}:visual_mesh",
                )
                builder.add_shape_convex_hull(
                    body=head_body,
                    xform=wp.transform(),
                    mesh=head_mesh,
                    scale=wp.vec3(1.0, 1.0, 1.0),
                    cfg=collision_proxy_cfg,
                    label=f"{head_label}:collision_proxy",
                )
            else:
                visual_cfg = head_shape_cfg.copy()
                visual_cfg.density = 0.0
                visual_cfg.has_shape_collision = False
                visual_cfg.has_particle_collision = False
                visual_cfg.is_visible = True
                collision_proxy_cfg = head_shape_cfg.copy()
                collision_proxy_cfg.is_visible = False
                collision_proxy_cfg.has_shape_collision = True
                head_mesh.build_sdf(max_resolution=HEAD_SDF_MAX_RESOLUTION)
                builder.add_shape_mesh(
                    body=head_body,
                    xform=wp.transform(),
                    mesh=head_mesh,
                    scale=wp.vec3(1.0, 1.0, 1.0),
                    cfg=visual_cfg,
                    label=f"{head_label}:visual_mesh",
                )
                builder.add_shape_mesh(
                    body=head_body,
                    xform=wp.transform(),
                    mesh=head_mesh,
                    scale=wp.vec3(1.0, 1.0, 1.0),
                    cfg=collision_proxy_cfg,
                    label=f"{head_label}:sdf_collision_proxy",
                )

        body0_prim = stage.GetPrimAtPath(authored.body0_path)
        if not body0_prim or not body0_prim.IsValid():
            raise ValueError(f"Fixed joint '{authored.prim_path}' references invalid body0 prim '{authored.body0_path}'.")
        body0_world = _get_world_transform_m(body0_prim, float(mpu))
        anchor_world_p = wp.transform_point(body0_world, authored.local_pos0_m)
        anchor_world_q = wp.normalize(wp.transform_get_rotation(body0_world) * authored.local_rot0)
        anchor_world_np = np.asarray(
            [float(anchor_world_p[0]), float(anchor_world_p[1]), float(anchor_world_p[2])], dtype=np.float64
        )

        matched_edge_idx, edge_t, edge_dist = _match_anchor_to_edge(anchor_world_np, points_m, edges)
        if edge_dist > attach_tolerance_m:
            raise ValueError(
                f"Fixed joint '{authored.prim_path}' anchor is too far from cable edges: "
                f"distance={edge_dist:.6e} m, tolerance={attach_tolerance_m:.6e} m."
            )

        edge_u, edge_v = edges[matched_edge_idx]
        seg_vec = points_m[edge_v] - points_m[edge_u]
        seg_q = _quat_from_segment_direction(seg_vec)
        parent_q = wp.normalize(wp.quat_inverse(seg_q) * anchor_world_q)
        parent_z = float(edge_t) * float(edge_lengths[matched_edge_idx])
        parent_xform = wp.transform(wp.vec3(0.0, 0.0, parent_z), parent_q)
        child_xform = wp.transform(authored.local_pos1_m, authored.local_rot1)

        fixed_label = f"{curve_base_label}:fixed:{authored.prim_path}"
        fixed_joint = builder.add_joint_fixed(
            parent=int(cable_body_ids[matched_edge_idx]),
            child=head_body,
            parent_xform=parent_xform,
            child_xform=child_xform,
            label=fixed_label,
            collision_filter_parent=True,
            enabled=True,
        )
        head_fixed_joint_ids.append(fixed_joint)

    return CableCurveImportResult(
        cable_body_ids=[int(v) for v in cable_body_ids],
        cable_joint_ids=[int(v) for v in cable_joint_ids],
        head_body_ids=head_body_ids,
        head_body_prim_paths=head_body_prim_paths,
        head_fixed_joint_ids=head_fixed_joint_ids,
        fixed_body_ids=fixed_body_ids,
        source_points_m=points_m.astype(np.float32, copy=True),
        edges=list(edges),
        radius_m=radius_m,
        curve_prim_path=curve_prim_path,
    )


def _default_curve_usd_path() -> str:
    return os.path.realpath(
        os.path.join(os.path.dirname(__file__), "assets", "version3", "SRA_curve", "cable_SRA_curve02.usda")
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse BasisCurves cable USD and build Newton cable + rigid heads.")
    parser.add_argument("--usd-path", type=str, default=_default_curve_usd_path())
    parser.add_argument("--curve-prim-path", type=str, default="/World/cable/curve_0")
    parser.add_argument("--head-shape-mode", choices=HEAD_SHAPE_MODES, default="mesh")
    parser.add_argument("--wrap-in-articulation", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    builder = newton.ModelBuilder()
    result = add_cable_from_usd_curve(
        builder,
        source_usd_path=args.usd_path,
        curve_prim_path=args.curve_prim_path,
        cable_label="curve_cable",
        wrap_in_articulation=bool(args.wrap_in_articulation),
        head_shape_mode=args.head_shape_mode,
    )

    print(
        "[usd_curve_import] summary: "
        f"usd={args.usd_path} curve={args.curve_prim_path} "
        f"cable_bodies={len(result.cable_body_ids)} "
        f"cable_joints={len(result.cable_joint_ids)} "
        f"head_bodies={len(result.head_body_ids)} "
        f"head_fixed_joints={len(result.head_fixed_joint_ids)} "
        f"fixed_bodies={len(result.fixed_body_ids)}"
    )
