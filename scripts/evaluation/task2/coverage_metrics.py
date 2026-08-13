#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Pure numpy ground-truth coverage/spill audit for task2 pad placement.

Computes how well the thermal pad's *physical* footprint (the deformed
mesh's world-space vertices, published on ``/isaac/task2/pad_points``
and recorded into ``task2_extras/episode_XXXXXX.npz``) overlaps the
target rectangle painted on the RAM board -- independent of the
camera-based bbox/semantic-mask IoU computed in ``evaluation.py``.
This module is additive and ground-truth only: nothing here feeds the
official score, and it has no ROS import, so it is usable from both
the eval container and plain offline analysis scripts.

Episode sidecar format (``task2_isaacsim/dataset/<name>/task2_extras/
episode_XXXXXX.npz``, written by
``task2_isaacsim/services/recording/record_task2.py``): ``sim_time``
``(T,) f64``, ``wall_time_ns`` ``(T,) i64``, ``object_poses``
``(T, K, 7) f32`` rows ``[x, y, z, qw, qx, qy, qz]`` (world frame),
``object_names`` ``(K,)`` (includes ``board_target``), ``pad_points``
``(P, 2004, 3) f32`` (world frame), ``pad_sim_time`` ``(P,) f64`` (~10
Hz). A fallback layout, ``pad_points_flat`` + ``pad_points_counts
(P,)``, exists for the rare topology-changing episode where the P
samples do not all share the same point count; see
``coverage_from_extras.load_episode`` for how it is reconstructed.
Points contain duplicated mesh vertices, and node ordering does not
reliably identify the top vs. bottom face.

Live topics (present only when the scene runs with ``--record``; see
``task2_isaacsim/scripts/recording/scene_capture.py``):
``/isaac/task2/object_poses`` (``std_msgs/String`` JSON
``{"sim_time": <float>, "objects": {"<name>": [x, y, z, qw, qx, qy,
qz], ...}}``) and ``/isaac/task2/pad_points`` (``std_msgs/
Float32MultiArray``, ``data = [sim_time, n_points, x0, y0, z0, x1,
...]``, world frame). Both publish at default QoS (RELIABLE, depth
10). ``parse_pad_points_payload``/``parse_object_poses_payload`` below
parse these for the live audit wired into ``node.py``.

Target geometry: from ``assets/task2_objects/Ram_Board_Target.usda``
(the Target cube), scale ``(0.12, 0.02, 0.0001)``, translate ``(0, 0,
0.0006)`` in the ``board_target`` root frame, so in board-local
coordinates the target rectangle is x in [-0.06, 0.06], y in [-0.01,
0.01] on the plane z ~= 0.0006 (``TARGET_HALF_EXTENTS_M`` /
``TARGET_LOCAL_OFFSET_M`` below). The pad mesh node pitch is ~5-6 mm
and the pad itself is ~3 mm thick.

Why rasterize-and-close instead of a convex hull: the ~5-6 mm node
pitch leaves real gaps between neighboring points that a hull would
not need to bridge but a point-in-polygon test on the raw points
would -- ``footprint_occupancy``'s morphological closing bridges
exactly this. A curled or folded pad's true footprint is non-convex,
which a hull cannot represent. The raw vertex set contains duplicated
vertices, which the boolean occupancy grid silently absorbs (a
per-point area estimate would not). And node ordering does not
reliably separate the top face from the bottom one, so both faces are
rasterized together -- harmless because the ~3 mm pad thickness is
well under one closing radius in plan view, so both faces collapse
into the same closed footprint rather than distorting it.
"""

import json
import math
from typing import Any

import numpy as np

TARGET_HALF_EXTENTS_M = (0.06, 0.01)
TARGET_LOCAL_OFFSET_M = (0.0, 0.0, 0.0006)

# Below this fraction of points landing inside the z-band, the pad is
# considered not to be resting on the board at all (picked up, still
# in flight, or otherwise not a real placement attempt) -- see
# coverage_metrics.
_MIN_IN_BAND_FRACTION = 0.3
# Two in-band principal-axis eigenvalues within this relative
# tolerance of each other are treated as isotropic: no reliable axis
# to report a yaw error against.
_YAW_DEGENERACY_TOL = 0.05


def quat_wxyz_to_matrix(
    qw: float, qx: float, qy: float, qz: float
) -> np.ndarray:
    """Rotation matrix for a ``(qw, qx, qy, qz)`` quaternion.

    Normalizes the input first, so a slightly denormalized live pose
    (e.g. a float32 round-trip) still yields an orthonormal matrix.
    Returns the identity for a (degenerate) zero-norm quaternion.
    """
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm == 0.0:
        return np.eye(3)
    w, x, y, z = q / norm
    return np.array(
        [
            [
                1 - 2 * (y * y + z * z),
                2 * (x * y - z * w),
                2 * (x * z + y * w),
            ],
            [
                2 * (x * y + z * w),
                1 - 2 * (x * x + z * z),
                2 * (y * z - x * w),
            ],
            [
                2 * (x * z - y * w),
                2 * (y * z + x * w),
                1 - 2 * (x * x + y * y),
            ],
        ]
    )


def board_local_points(points_world: np.ndarray, board_pose7) -> np.ndarray:
    """World-frame points expressed in the ``board_target`` local frame.

    ``board_pose7`` is ``[x, y, z, qw, qx, qy, qz]`` (world frame, as
    stored in ``object_poses`` / published on ``object_poses`` for the
    ``board_target`` prim). ``local = R.T @ (p - t)``, then
    ``TARGET_LOCAL_OFFSET_M`` is subtracted so the returned z is
    measured from the target plane (z ~= 0 on the plane), not from the
    ``board_target`` prim's own origin.
    """
    pose = np.asarray(board_pose7, dtype=np.float64)
    translation = pose[0:3]
    rotation = quat_wxyz_to_matrix(*pose[3:7])
    points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    # Row-vector batch form of R.T @ (p - t): (R.T @ v)_i == (v_row @ R)_i.
    local = (points - translation) @ rotation
    return local - np.asarray(TARGET_LOCAL_OFFSET_M, dtype=np.float64)


def parse_pad_points_payload(
    data,
) -> tuple[float | None, np.ndarray | None]:
    """Parse a ``pad_points`` ``Float32MultiArray.data`` payload.

    ``data = [sim_time, n_points, x0, y0, z0, x1, ...]`` (see the
    module docstring). Returns ``(sim_time, (N, 3) f32 points)``, or
    ``(None, None)`` for any truncated/malformed payload: fewer than
    2 elements, a non-positive point count, or a length that does not
    equal ``2 + 3 * n_points``.
    """
    values = list(data)
    if len(values) < 2:
        return None, None
    try:
        sim_time = float(values[0])
        n_points = int(round(values[1]))
    except (TypeError, ValueError):
        return None, None
    if n_points <= 0:
        return None, None
    if len(values) != 2 + 3 * n_points:
        return None, None
    points = np.asarray(values[2:], dtype=np.float32).reshape(n_points, 3)
    return sim_time, points


def parse_object_poses_payload(
    payload: str,
) -> tuple[float | None, dict[str, list[float]] | None]:
    """Parse an ``object_poses`` ``String.data`` JSON payload.

    Expects ``{"sim_time": <float>, "objects": {"<name>": [x, y, z,
    qw, qx, qy, qz], ...}}`` (see the module docstring). Returns
    ``(None, None)`` on any parse failure: malformed JSON, a missing
    ``sim_time``/``objects`` key, or an ``objects`` value that is not
    a JSON object.
    """
    try:
        obj = json.loads(payload)
        sim_time = float(obj["sim_time"])
        objects = obj["objects"]
    except (TypeError, ValueError, KeyError, json.JSONDecodeError):
        return None, None
    if not isinstance(objects, dict):
        return None, None
    return sim_time, objects


def _shift(grid: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """``grid`` translated by ``(dx, dy)`` cells; out-of-bounds reads False."""
    out = np.zeros_like(grid)
    n_x, n_y = grid.shape
    src_x0, src_x1 = max(0, -dx), min(n_x, n_x - dx)
    src_y0, src_y1 = max(0, -dy), min(n_y, n_y - dy)
    if src_x0 >= src_x1 or src_y0 >= src_y1:
        return out
    dst_x0, dst_x1 = src_x0 + dx, src_x1 + dx
    dst_y0, dst_y1 = src_y0 + dy, src_y1 + dy
    out[dst_x0:dst_x1, dst_y0:dst_y1] = grid[src_x0:src_x1, src_y0:src_y1]
    return out


def _disk_offsets(radius_m: float, grid_res_m: float) -> list[tuple[int, int]]:
    """Integer ``(dx, dy)`` cell offsets covering a disk of ``radius_m``."""
    if radius_m <= 0.0:
        return [(0, 0)]
    r_cells = int(math.ceil(radius_m / grid_res_m))
    radius_sq = radius_m * radius_m
    return [
        (dx, dy)
        for dx in range(-r_cells, r_cells + 1)
        for dy in range(-r_cells, r_cells + 1)
        if (dx * grid_res_m) ** 2 + (dy * grid_res_m) ** 2 <= radius_sq
    ]


def _morphological_close(
    grid: np.ndarray, radius_m: float, grid_res_m: float
) -> np.ndarray:
    """Binary closing (dilate then erode) by a disk, numpy-only.

    Both steps reuse the same symmetric disk offset list: dilation is
    the union (OR) of ``grid`` shifted by every offset; erosion is the
    intersection (AND) of the dilated grid shifted by every offset.
    Cells shifted in from outside the grid read as False for both
    steps, so a shape touching the grid boundary can only shrink
    there, never wrap around or fabricate coverage.
    """
    offsets = _disk_offsets(radius_m, grid_res_m)
    dilated = np.zeros_like(grid)
    for dx, dy in offsets:
        dilated |= _shift(grid, dx, dy)
    eroded = np.ones_like(dilated)
    for dx, dy in offsets:
        eroded &= _shift(dilated, dx, dy)
    return eroded


def footprint_occupancy(
    local_xy: np.ndarray,
    *,
    grid_res_m: float = 0.001,
    close_radius_m: float = 0.004,
    bounds_margin_m: float = 0.04,
) -> tuple[np.ndarray, float, float]:
    """Rasterize ``local_xy`` into a closed boolean occupancy grid.

    The grid spans the target rectangle plus ``bounds_margin_m`` on
    every side (~200x100 cells at the 1 mm default -- modest enough
    for plain numpy). Points are splat into cells via vectorized index
    assignment (no per-point Python loop), then closed with a disk of
    radius ``close_radius_m`` (see ``_morphological_close``) to bridge
    the pad mesh's ~5-6 mm node pitch. Points outside the margin box
    are silently dropped (they cannot be represented on this grid);
    ``coverage_metrics`` falls back to the raw point centroid in the
    (rare) case this empties the footprint entirely.

    Returns ``(grid, x0, y0)`` where ``grid[i, j]`` is True iff cell
    ``(i, j)`` is occupied, and ``(x0, y0)`` is the coordinate of cell
    ``(0, 0)``'s center, in the same (board-local) frame as
    ``local_xy``.
    """
    half_x, half_y = TARGET_HALF_EXTENTS_M
    x_min = -half_x - bounds_margin_m
    y_min = -half_y - bounds_margin_m
    n_x = int(round(2.0 * (half_x + bounds_margin_m) / grid_res_m))
    n_y = int(round(2.0 * (half_y + bounds_margin_m) / grid_res_m))
    x0 = x_min + grid_res_m / 2.0
    y0 = y_min + grid_res_m / 2.0

    grid = np.zeros((n_x, n_y), dtype=bool)
    xy = np.asarray(local_xy, dtype=np.float64).reshape(-1, 2)
    if xy.shape[0] > 0:
        ix = np.floor((xy[:, 0] - x_min) / grid_res_m).astype(np.int64)
        iy = np.floor((xy[:, 1] - y_min) / grid_res_m).astype(np.int64)
        valid = (ix >= 0) & (ix < n_x) & (iy >= 0) & (iy < n_y)
        grid[ix[valid], iy[valid]] = True

    grid = _morphological_close(grid, close_radius_m, grid_res_m)
    return grid, x0, y0


def _footprint_vs_target(
    grid: np.ndarray,
    x0: float,
    y0: float,
    grid_res_m: float,
    in_band_xy: np.ndarray,
) -> tuple[float, float, float]:
    """``(coverage, spill, center_error_m)`` of a closed footprint grid."""
    half_x, half_y = TARGET_HALF_EXTENTS_M
    n_x, n_y = grid.shape
    xs = x0 + np.arange(n_x) * grid_res_m
    ys = y0 + np.arange(n_y) * grid_res_m
    rect_mask = np.outer(
        (xs >= -half_x) & (xs <= half_x), (ys >= -half_y) & (ys <= half_y)
    )

    footprint_count = int(np.count_nonzero(grid))
    intersection_count = int(np.count_nonzero(grid & rect_mask))
    target_area = (2.0 * half_x) * (2.0 * half_y)
    cell_area = grid_res_m * grid_res_m

    coverage = (
        (intersection_count * cell_area) / target_area
        if target_area > 0.0
        else 0.0
    )
    spill = (
        1.0 - intersection_count / footprint_count
        if footprint_count > 0
        else 0.0
    )

    if footprint_count > 0:
        xx, yy = np.meshgrid(xs, ys, indexing="ij")
        centroid = (float(xx[grid].mean()), float(yy[grid].mean()))
    else:
        # The closed grid has nothing in it even though in-band points
        # exist (they all fell outside the +/- bounds_margin_m box, or
        # closing eroded an isolated point away): fall back to the raw
        # point centroid so a badly mislocated pad is not reported as
        # perfectly centered.
        centroid = (
            float(in_band_xy[:, 0].mean()),
            float(in_band_xy[:, 1].mean()),
        )
    center_error_m = float(math.hypot(centroid[0], centroid[1]))
    return coverage, spill, center_error_m


def _principal_axis_yaw_error(xy: np.ndarray) -> float:
    """Angle (degrees) between the in-band points' principal axis and
    board-local X, folded to ``[0, 90]`` (an axis, not a directed
    vector, is 180-degree periodic). Falls back to 0.0 when the two
    2nd-moment eigenvalues sit within ``_YAW_DEGENERACY_TOL`` of each
    other -- too isotropic a point cloud to pick a reliable axis from.
    """
    centered = xy - xy.mean(axis=0)
    cov = (centered.T @ centered) / xy.shape[0]
    eigvals, eigvecs = np.linalg.eigh(cov)
    lam_small, lam_large = float(eigvals[0]), float(eigvals[1])
    if (
        lam_large <= 0.0
        or (lam_large - lam_small) <= _YAW_DEGENERACY_TOL * lam_large
    ):
        return 0.0
    principal = eigvecs[:, 1]
    angle = math.degrees(math.atan2(principal[1], principal[0])) % 180.0
    return float(180.0 - angle if angle > 90.0 else angle)


def coverage_metrics(
    pad_points_world: np.ndarray,
    board_pose7,
    *,
    z_band_m: tuple[float, float] = (-0.003, 0.008),
    grid_res_m: float = 0.001,
    close_radius_m: float = 0.004,
) -> dict[str, Any]:
    """Physical coverage/spill/placement audit of one pad point cloud.

    Ground-truth-only counterpart to the camera-based bbox IoU in
    ``evaluation.py``: ``pad_points_world`` is one ``pad_points``
    sample (``(N, 3)`` world-frame mesh vertices, both faces, per the
    module docstring) and ``board_pose7`` is the matching
    ``board_target`` world pose. Computed over ALL points inside
    ``z_band_m`` of the target plane (both faces -- the pad is ~3 mm
    thick, comfortably inside one ``close_radius_m`` in plan view, so
    rasterizing both together does not distort the footprint; see the
    module docstring for why rasterize-and-close was chosen over a
    convex hull).

    Returns exactly: ``coverage`` (``|footprint (intersect) target
    rect| / |target rect|``), ``spill`` (``|footprint - target rect| /
    |footprint|``, 0.0 for an empty footprint), ``center_error_m``
    (footprint centroid to rect-center distance in the board plane),
    ``yaw_error_deg`` (principal-axis angle vs. board-local X, folded
    to [0, 90]), ``z_span_m`` (max - min z of in-band points, 0.0 if
    none), ``points_in_band``/``points_total`` (int counts), and
    ``off_board`` (bool).

    An empty in-band point set (``points_in_band == 0``) zeroes every
    numeric field and sets ``off_board`` True. Otherwise ``off_board``
    is ``points_in_band / points_total < _MIN_IN_BAND_FRACTION`` (too
    little of the pad sits near the target plane at all to be a real
    placement attempt); when that trips, only ``coverage``/``spill``
    are forced to 0.0 -- ``center_error_m``/``yaw_error_deg``/
    ``z_span_m`` still describe whatever few in-band points there are,
    for debugging.
    """
    points_total = int(np.asarray(pad_points_world).shape[0])

    if points_total > 0:
        local = board_local_points(pad_points_world, board_pose7)
        band_mask = (local[:, 2] >= z_band_m[0]) & (local[:, 2] <= z_band_m[1])
        in_band_xy = local[band_mask, 0:2]
        in_band_z = local[band_mask, 2]
    else:
        in_band_xy = np.zeros((0, 2), dtype=np.float64)
        in_band_z = np.zeros((0,), dtype=np.float64)
    points_in_band = int(in_band_xy.shape[0])

    if points_in_band == 0:
        return {
            "coverage": 0.0,
            "spill": 0.0,
            "center_error_m": 0.0,
            "yaw_error_deg": 0.0,
            "z_span_m": 0.0,
            "points_in_band": 0,
            "points_total": points_total,
            "off_board": True,
        }

    grid, x0, y0 = footprint_occupancy(
        in_band_xy, grid_res_m=grid_res_m, close_radius_m=close_radius_m
    )
    coverage, spill, center_error_m = _footprint_vs_target(
        grid, x0, y0, grid_res_m, in_band_xy
    )
    yaw_error_deg = _principal_axis_yaw_error(in_band_xy)
    z_span_m = float(in_band_z.max() - in_band_z.min())

    off_board = (points_in_band / points_total) < _MIN_IN_BAND_FRACTION
    if off_board:
        coverage = 0.0
        spill = 0.0

    return {
        "coverage": float(coverage),
        "spill": float(spill),
        "center_error_m": center_error_m,
        "yaw_error_deg": yaw_error_deg,
        "z_span_m": z_span_m,
        "points_in_band": points_in_band,
        "points_total": points_total,
        "off_board": bool(off_board),
    }
