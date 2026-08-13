# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Isaac-free tests for the pure helpers in integration_test.py."""

import importlib.util
import math
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / "integration_test.py"
SPEC = importlib.util.spec_from_file_location("task3_integration", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)

Point3D = runner.Point3D
Bounds3D = runner.Bounds3D


def test_spoon_poses_start_retract_and_insertion_offsets():
    head_feed = Point3D(1.0, 2.0, 3.0)
    start, insertion, retract = runner.stage2_spoon_poses(head_feed)
    assert start.y == pytest.approx(head_feed.y - 0.20)
    assert insertion.y == pytest.approx(head_feed.y - 0.08)
    assert retract == start, "retract must return to the start pose"
    for pose in (start, insertion, retract):
        assert pose.x == pytest.approx(head_feed.x)
        assert pose.z == pytest.approx(head_feed.z)


def test_reported_insertion_distance_matches_the_travelled_distance():
    # Pins result["insertion_distance_m"] == 0.12 against the poses that
    # produce it, instead of restating the constant.
    start, insertion, _ = runner.stage2_spoon_poses(Point3D(1.0, 2.0, 3.0))
    travelled = math.dist(
        (start.x, start.y, start.z), (insertion.x, insertion.y, insertion.z)
    )
    assert travelled == pytest.approx(0.12)


def test_closest_head_offset_is_a_y_component_not_a_distance():
    # stage2_feed_pose lifts the feed point 0.17 m above the head origin, so
    # the spoon is 0.188 m from the head at insertion, not 0.08 m. This test
    # documents the discrepancy result["closest_head_offset_m"] = 0.08 hides.
    head_origin = Point3D(1.0, 2.0, 3.0)
    feed = Point3D(head_origin.x, head_origin.y, head_origin.z + 0.17)
    _, insertion, _ = runner.stage2_spoon_poses(feed)
    y_offset = abs(insertion.y - head_origin.y)
    true_distance = math.dist(
        (insertion.x, insertion.y, insertion.z),
        (head_origin.x, head_origin.y, head_origin.z),
    )
    assert y_offset == pytest.approx(0.08)
    assert true_distance == pytest.approx(math.hypot(0.08, 0.17))
    assert true_distance > y_offset


def test_container_translation_aligns_centres_in_xy_and_floors_in_z():
    source = Bounds3D(
        x_min=0.0, y_min=0.0, z_min=0.0, x_max=2.0, y_max=2.0, z_max=1.0
    )
    target = Bounds3D(
        x_min=10.0, y_min=20.0, z_min=5.0, x_max=12.0, y_max=24.0, z_max=6.0
    )
    moved = runner.translate_points_between_containers(
        [Point3D(1.0, 1.0, 0.5)], source, target
    )
    # x/y follow centre-to-centre; z follows z_min-to-z_min. The asymmetry is
    # deliberate — beans keep their height above the container floor.
    assert moved[0].x == pytest.approx(11.0)
    assert moved[0].y == pytest.approx(22.0)
    assert moved[0].z == pytest.approx(5.5)


def test_container_translation_preserves_the_bean_packing_layout():
    # The PR claims it "preserves the original bean geometry, density, packing
    # layout, and Z offsets". A rigid translation must leave every pairwise
    # distance unchanged.
    source = Bounds3D(
        x_min=0.0, y_min=0.0, z_min=0.0, x_max=2.0, y_max=2.0, z_max=1.0
    )
    target = Bounds3D(
        x_min=10.0, y_min=20.0, z_min=5.0, x_max=12.0, y_max=24.0, z_max=6.0
    )
    points = [
        Point3D(0.9, 1.0, 0.10),
        Point3D(1.1, 1.0, 0.10),
        Point3D(1.0, 1.2, 0.35),
    ]
    moved = runner.translate_points_between_containers(points, source, target)
    assert len(moved) == len(points)
    for a, b in ((0, 1), (0, 2), (1, 2)):
        before = math.dist(
            (points[a].x, points[a].y, points[a].z),
            (points[b].x, points[b].y, points[b].z),
        )
        after = math.dist(
            (moved[a].x, moved[a].y, moved[a].z),
            (moved[b].x, moved[b].y, moved[b].z),
        )
        assert after == pytest.approx(before)


def test_container_translation_of_an_empty_layout_is_empty():
    bounds = Bounds3D(
        x_min=0.0, y_min=0.0, z_min=0.0, x_max=1.0, y_max=1.0, z_max=1.0
    )
    assert runner.translate_points_between_containers([], bounds, bounds) == []
