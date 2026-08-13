#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for coverage_metrics/coverage_from_extras (no ROS required).

Run: python3 scripts/evaluation/task2/tests/test_coverage_metrics.py
Only depends on numpy (+ stdlib tempfile/json for the loader test) --
matches the modules under test.
"""

import json
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

# Make the flat eval modules importable when run directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coverage_from_extras import evaluate_episode, load_episode  # noqa: E402
from coverage_metrics import (  # noqa: E402
    TARGET_HALF_EXTENTS_M,
    coverage_metrics,
    parse_object_poses_payload,
    parse_pad_points_payload,
    quat_wxyz_to_matrix,
)


def expect(name, cond):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}")
    if not cond:
        raise AssertionError(name)


# --------------------------------------------------------------------------- #
# Synthetic golden-case helpers
# --------------------------------------------------------------------------- #
def _yaw_quat_wxyz(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


# Board pose used by every golden case: translated + yawed 30 degrees, per
# the brief's worked example, so the golden numbers only hold if
# board_local_points' rotate-and-translate math is actually correct (an
# axis-aligned/untranslated board pose could hide a wrong-frame bug).
BOARD_POSE = (0.3, -0.2, 0.8) + _yaw_quat_wxyz(30.0)


def _rect_grid_xy(
    half_x: float, half_y: float, spacing_m: float
) -> np.ndarray:
    n_x = int(round(2 * half_x / spacing_m)) + 1
    n_y = int(round(2 * half_y / spacing_m)) + 1
    xs = np.linspace(-half_x, half_x, n_x)
    ys = np.linspace(-half_y, half_y, n_y)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([xx.ravel(), yy.ravel()], axis=1)


def _pad_local_points(
    *,
    offset_xy=(0.0, 0.0),
    yaw_deg=0.0,
    z_layers=(0.001, 0.003),
    spacing_m=0.0015,
):
    """Synthetic pad point cloud in the ``board_target`` root frame (the
    frame ``_transform_local_to_world`` below expects -- i.e. BEFORE
    ``coverage_metrics.board_local_points``' ``TARGET_LOCAL_OFFSET_M``
    subtraction).

    A dense grid exactly the size of the target rectangle
    (``TARGET_HALF_EXTENTS_M``), optionally yawed about the rect center
    and/or shifted, replicated at each z in ``z_layers`` (two, by
    default, to mimic the pad's top/bottom faces).

    The 1.5 mm default spacing is deliberately denser than the real
    pad mesh's ~5-6 mm node pitch: closing (see
    ``footprint_occupancy``) only fully recovers a lattice's own
    convex boundary once neighboring boundary points sit closer than
    roughly half the closing radius apart (a genuine, expected
    property of dilate-then-erode, not a bug -- the real ~5-6 mm pitch
    against the default 4 mm close_radius_m measurably undershoots the
    golden thresholds below, verified empirically). These golden cases
    exist to pin down the *formula*, decoupled from real-mesh-density
    adequacy; the CLI's read-only run against real recorded episodes
    (see the task report) is what sanity-checks the latter.
    """
    from coverage_metrics import TARGET_LOCAL_OFFSET_M

    half_x, half_y = TARGET_HALF_EXTENTS_M
    xy = _rect_grid_xy(half_x, half_y, spacing_m)
    if yaw_deg:
        theta = math.radians(yaw_deg)
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, -s], [s, c]])
        xy = xy @ rotation.T
    xy = xy + np.asarray(offset_xy)
    layers = [
        np.concatenate([xy, np.full((xy.shape[0], 1), z)], axis=1)
        for z in z_layers
    ]
    local_target_frame = np.concatenate(layers, axis=0)
    return local_target_frame + np.asarray(TARGET_LOCAL_OFFSET_M)


def _transform_local_to_world(local_points, board_pose7):
    """Inverse of ``board_local_points`` (minus the offset subtraction):
    ``world = local_before_offset_subtraction @ R.T + t``.
    """
    pose = np.asarray(board_pose7, dtype=np.float64)
    translation = pose[0:3]
    rotation = quat_wxyz_to_matrix(*pose[3:7])
    return local_points @ rotation.T + translation


def _perfect_world_points(**kwargs):
    return _transform_local_to_world(_pad_local_points(**kwargs), BOARD_POSE)


# --------------------------------------------------------------------------- #
# Golden coverage_metrics cases
# --------------------------------------------------------------------------- #
def test_coverage_golden_perfect():
    world = _perfect_world_points()
    result = coverage_metrics(world, BOARD_POSE)
    expect("perfect: coverage >= 0.98", result["coverage"] >= 0.98)
    expect("perfect: spill <= 0.05", result["spill"] <= 0.05)
    expect("perfect: center_error < 2mm", result["center_error_m"] < 0.002)
    expect("perfect: yaw_error < 2deg", result["yaw_error_deg"] < 2.0)
    expect("perfect: off_board False", result["off_board"] is False)


def test_coverage_golden_shifted():
    world = _perfect_world_points(offset_xy=(0.0, 0.006))
    result = coverage_metrics(world, BOARD_POSE)
    expect(
        "shifted: coverage ~0.7 +/- 0.03",
        abs(result["coverage"] - 0.7) <= 0.03,
    )
    expect("shifted: spill ~0.3 +/- 0.03", abs(result["spill"] - 0.3) <= 0.03)
    expect(
        "shifted: center_error ~6mm +/- 1mm",
        abs(result["center_error_m"] - 0.006) <= 0.001,
    )


def test_coverage_golden_rotated():
    world = _perfect_world_points(yaw_deg=90.0)
    result = coverage_metrics(world, BOARD_POSE)
    expect(
        "rotated: coverage ~1/6 +/- 0.03",
        abs(result["coverage"] - (1.0 / 6.0)) <= 0.03,
    )
    expect(
        "rotated: yaw_error ~90deg +/- 2",
        abs(result["yaw_error_deg"] - 90.0) <= 2.0,
    )


def test_off_board():
    world = _perfect_world_points(z_layers=(0.05, 0.052))
    result = coverage_metrics(world, BOARD_POSE)
    expect("off_board: flag True", result["off_board"] is True)
    expect("off_board: coverage 0.0", result["coverage"] == 0.0)
    expect("off_board: spill 0.0", result["spill"] == 0.0)
    expect("off_board: center_error 0.0", result["center_error_m"] == 0.0)
    expect("off_board: yaw_error 0.0", result["yaw_error_deg"] == 0.0)
    expect("off_board: z_span 0.0", result["z_span_m"] == 0.0)
    expect("off_board: points_in_band 0", result["points_in_band"] == 0)


def test_duplicate_points_harmless():
    world = _perfect_world_points(offset_xy=(0.0, 0.006))
    result = coverage_metrics(world, BOARD_POSE)
    world_dup = np.concatenate([world, world], axis=0)
    result_dup = coverage_metrics(world_dup, BOARD_POSE)
    for key in (
        "coverage",
        "spill",
        "center_error_m",
        "yaw_error_deg",
        "z_span_m",
        "off_board",
    ):
        expect(f"dup: {key} identical", result[key] == result_dup[key])
    expect(
        "dup: points_total doubled",
        result_dup["points_total"] == 2 * result["points_total"],
    )
    expect(
        "dup: points_in_band doubled",
        result_dup["points_in_band"] == 2 * result["points_in_band"],
    )


# --------------------------------------------------------------------------- #
# Payload parsers
# --------------------------------------------------------------------------- #
def test_parse_pad_points_payload():
    expected = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    payload = [0.5, 2, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    sim_time, points = parse_pad_points_payload(payload)
    expect("pad payload: sim_time", sim_time == 0.5)
    expect("pad payload: shape", points.shape == (2, 3))
    expect("pad payload: values", np.allclose(points, expected))
    expect("pad payload: dtype", points.dtype == np.float32)

    truncated = [0.5, 2, 1.0, 2.0, 3.0]  # missing the second point
    expect(
        "pad payload: truncated -> (None, None)",
        parse_pad_points_payload(truncated) == (None, None),
    )

    expect(
        "pad payload: empty -> (None, None)",
        parse_pad_points_payload([]) == (None, None),
    )

    expect(
        "pad payload: n<=0 -> (None, None)",
        parse_pad_points_payload([0.5, 0]) == (None, None),
    )

    expect(
        "pad payload: single element -> (None, None)",
        parse_pad_points_payload([0.5]) == (None, None),
    )


def test_parse_object_poses_payload():
    payload = json.dumps(
        {
            "sim_time": 1.25,
            "objects": {
                "board_target": [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
                "thermalpad": [0.4, 0.5, 0.6, 1.0, 0.0, 0.0, 0.0],
            },
        }
    )
    sim_time, objects = parse_object_poses_payload(payload)
    expect("poses payload: sim_time", sim_time == 1.25)
    expect(
        "poses payload: board_target",
        objects["board_target"] == [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
    )
    expect("poses payload: two objects", len(objects) == 2)

    expect(
        "poses payload: malformed JSON -> (None, None)",
        parse_object_poses_payload("{not json") == (None, None),
    )

    missing_objects = json.dumps({"sim_time": 1.0})
    expect(
        "poses payload: missing 'objects' -> (None, None)",
        parse_object_poses_payload(missing_objects) == (None, None),
    )

    missing_sim_time = json.dumps({"objects": {}})
    expect(
        "poses payload: missing 'sim_time' -> (None, None)",
        parse_object_poses_payload(missing_sim_time) == (None, None),
    )

    not_a_dict = json.dumps({"sim_time": 1.0, "objects": [1, 2, 3]})
    expect(
        "poses payload: objects not a dict -> (None, None)",
        parse_object_poses_payload(not_a_dict) == (None, None),
    )


# --------------------------------------------------------------------------- #
# coverage_from_extras loader / evaluate_episode
# --------------------------------------------------------------------------- #
def _write_regular_npz(
    path, *, sim_time, object_names, object_poses, pad_samples, pad_sim_time
):
    np.savez_compressed(
        path,
        sim_time=sim_time,
        object_poses=object_poses,
        object_names=np.array(object_names),
        pad_points=np.stack(pad_samples),
        pad_sim_time=pad_sim_time,
    )


def _write_flat_npz(
    path, *, sim_time, object_names, object_poses, pad_samples, pad_sim_time
):
    # Mirrors record_task2.py's ExtrasBuffer.save() "topology changed"
    # branch exactly: a 1-D concatenation of each sample's own (n_i, 3)
    # reshaped to (n_i * 3,), NOT a pre-stacked (sum_counts, 3) array.
    flat = np.concatenate([sample.reshape(-1) for sample in pad_samples])
    counts = np.array(
        [sample.shape[0] for sample in pad_samples], dtype=np.int64
    )
    np.savez_compressed(
        path,
        sim_time=sim_time,
        object_poses=object_poses,
        object_names=np.array(object_names),
        pad_points_flat=flat,
        pad_points_counts=counts,
        pad_sim_time=pad_sim_time,
    )


def test_extras_loader():
    object_names = ["board_target", "thermalpad"]
    sim_time = np.array([0.0, 1.0, 2.0], dtype=np.float64)
    object_poses = np.zeros((3, 2, 7), dtype=np.float32)
    object_poses[:, 0] = BOARD_POSE  # board_target row, every frame
    pad_sim_time = np.array([1.5, 2.0], dtype=np.float64)
    common_kwargs = dict(
        grid_res_m=0.001,
        close_radius_m=0.004,
        z_band_m=(-0.003, 0.008),
        settle_eps_m=0.001,
    )

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)

        # Regular (P, N, 3) layout: both samples identical -> settled.
        perfect = _perfect_world_points().astype(np.float32)
        regular_path = tmp_path / "episode_000000.npz"
        _write_regular_npz(
            regular_path,
            sim_time=sim_time,
            object_names=object_names,
            object_poses=object_poses,
            pad_samples=[perfect, perfect],
            pad_sim_time=pad_sim_time,
        )

        loaded = load_episode(regular_path)
        expect(
            "loader: regular pad_points sample count",
            len(loaded["pad_points"]) == 2,
        )
        expect(
            "loader: regular last-sample shape",
            loaded["pad_points"][-1].shape == perfect.shape,
        )
        record = evaluate_episode(0, loaded, **common_kwargs)
        expect(
            "loader: regular metrics present", record["metrics"] is not None
        )
        expect(
            "loader: regular coverage high (known-perfect placement)",
            record["metrics"]["coverage"] >= 0.98,
        )
        expect("loader: regular settled True", record["settled"] is True)
        expect(
            "loader: regular pad_sim_time is the last sample",
            record["pad_sim_time"] == 2.0,
        )

        # Flat fallback layout (topology-changing episode): two samples
        # with DIFFERENT point counts.
        sample_a = perfect
        sample_b = sample_a[:-5]  # fewer points -> topology changed
        flat_path = tmp_path / "episode_000001.npz"
        _write_flat_npz(
            flat_path,
            sim_time=sim_time,
            object_names=object_names,
            object_poses=object_poses,
            pad_samples=[sample_a, sample_b],
            pad_sim_time=pad_sim_time,
        )

        loaded_flat = load_episode(flat_path)
        expect(
            "loader: flat pad_points sample count",
            len(loaded_flat["pad_points"]) == 2,
        )
        expect(
            "loader: flat sample shapes reconstructed",
            loaded_flat["pad_points"][0].shape == sample_a.shape
            and loaded_flat["pad_points"][1].shape == sample_b.shape,
        )
        record_flat = evaluate_episode(1, loaded_flat, **common_kwargs)
        expect(
            "loader: flat metrics present", record_flat["metrics"] is not None
        )
        expect(
            "loader: flat settled False (topology changed)",
            record_flat["settled"] is False,
        )

        # Missing pad data entirely -> metrics null with a reason, no crash.
        no_pad_path = tmp_path / "episode_000002.npz"
        np.savez_compressed(
            no_pad_path,
            sim_time=sim_time,
            object_poses=object_poses,
            object_names=np.array(object_names),
        )
        loaded_no_pad = load_episode(no_pad_path)
        record_no_pad = evaluate_episode(2, loaded_no_pad, **common_kwargs)
        expect(
            "loader: no pad data -> metrics null",
            record_no_pad["metrics"] is None,
        )
        expect(
            "loader: no pad data -> reason set",
            record_no_pad["reason"] == "no_pad_points",
        )

        # board_target missing from object_names -> metrics null with a
        # reason, but settled/pad_sim_time still computed (pad data is
        # fine; only the board pose lookup fails).
        no_board_path = tmp_path / "episode_000003.npz"
        _write_regular_npz(
            no_board_path,
            sim_time=sim_time,
            object_names=["thermalpad"],
            object_poses=object_poses[:, 1:2],
            pad_samples=[perfect, perfect],
            pad_sim_time=pad_sim_time,
        )
        loaded_no_board = load_episode(no_board_path)
        record_no_board = evaluate_episode(3, loaded_no_board, **common_kwargs)
        expect(
            "loader: no board_target -> metrics null",
            record_no_board["metrics"] is None,
        )
        expect(
            "loader: no board_target -> reason set",
            record_no_board["reason"] == "board_target_missing",
        )
        expect(
            "loader: no board_target -> settled still computed",
            record_no_board["settled"] is True,
        )


def main():
    tests = [
        test_coverage_golden_perfect,
        test_coverage_golden_shifted,
        test_coverage_golden_rotated,
        test_off_board,
        test_duplicate_points_harmless,
        test_parse_pad_points_payload,
        test_parse_object_poses_payload,
        test_extras_loader,
    ]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} coverage_metrics tests passed.")


if __name__ == "__main__":
    main()
