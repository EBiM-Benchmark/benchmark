#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for pure task3 grading logic.

Run: python3 scripts/evaluation/task3/tests/test_grading.py
No Isaac Sim runtime is required.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from grading import (  # noqa: E402
    DEFAULT_STAGE1_OBJECTS,
    DEFAULT_UTENSIL_OBJECTS,
    TASK3_BEAN_RECOVERY_REGION,
    TASK3_KITCHEN_AREA,
    TASK3_SINK_REGION,
    Area2D,
    Bounds2D,
    FeedHoldState,
    Point3D,
    bean_recovery_score,
    classify_table_area,
    count_points_in_sphere,
    feed_score,
    movement_is_smooth,
    score_stage1_table_setup,
    score_stage4_cleanup,
    update_feed_hold,
)


def expect(name, cond):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}")
    if not cond:
        raise AssertionError(name)


def test_kitchen_and_dining_area_classification():
    expect(
        "kitchen contains original tray pose",
        classify_table_area((-4.28, -1.59), TASK3_KITCHEN_AREA) == "inside",
    )
    expect(
        "dining starts beyond kitchen y border",
        classify_table_area((-2.85, TASK3_KITCHEN_AREA.y_max + 0.001))
        == "dining",
    )
    expect(
        "overlap border remains kitchen",
        classify_table_area((-2.85, TASK3_KITCHEN_AREA.y_max)) == "kitchen",
    )
    expect(
        "outside both regions",
        classify_table_area((1.0, 1.0)) == "outside",
    )


def test_dining_classification_respects_dining_lower_y_bound():
    kitchen = Area2D(center_x=0.0, center_y=0.0, scale_x=2.0, scale_y=2.0)
    dining = Area2D(center_x=0.0, center_y=5.0, scale_x=2.0, scale_y=2.0)

    expect(
        "non-overlapping gap is outside",
        classify_table_area(
            (0.0, 2.0), kitchen_area=kitchen, dining_area=dining
        )
        == "outside",
    )
    expect(
        "non-overlapping dining region still works",
        classify_table_area(
            (0.0, 4.5), kitchen_area=kitchen, dining_area=dining
        )
        == "dining",
    )


def test_stage1_counts_objects_moved_to_dining_area():
    poses = {
        "simple_tray": Point3D(-2.85, 1.90, 0.80),
        "bowl2": Point3D(-2.70, 1.80, 0.80),
        "spoon2": Point3D(-2.60, 1.70, 0.80),
        "plate2": Point3D(-4.30, -1.60, 0.80),
        "cup": Point3D(-4.20, -1.70, 0.80),
    }

    result = score_stage1_table_setup(poses)

    expect("stage1 score counts dining objects", result.score == 3)
    expect("stage1 uses default stage1 object list", result.max_score == 5)
    expect(
        "stage1 records moved objects",
        result.passed == ["simple_tray", "bowl2", "spoon2"],
    )
    expect(
        "stage1 default objects include tray",
        "simple_tray" in DEFAULT_STAGE1_OBJECTS,
    )


def test_feed_score_requires_smooth_three_second_hold():
    smooth_positions = [
        Point3D(-4.34, -1.65, 0.78),
        Point3D(-3.90, -0.80, 0.82),
        Point3D(-3.30, 0.50, 0.85),
        Point3D(-2.80, 1.70, 0.90),
    ]
    jump_positions = [
        Point3D(-4.34, -1.65, 0.78),
        Point3D(-2.80, 1.70, 0.90),
    ]

    expect(
        "smooth feed path accepted",
        movement_is_smooth(smooth_positions, max_step=1.5),
    )
    expect(
        "jump feed path rejected",
        not movement_is_smooth(jump_positions, max_step=1.5),
    )
    expect(
        "feed caps score at four",
        feed_score(beans_left=6, hold_seconds=3.0, smooth=True) == 4,
    )
    expect(
        "feed counts remaining beans under cap",
        feed_score(beans_left=2, hold_seconds=3.5, smooth=True) == 2,
    )
    expect(
        "feed rejects short hold",
        feed_score(beans_left=4, hold_seconds=2.99, smooth=True) == 0,
    )
    expect(
        "feed rejects non-smooth insertion",
        feed_score(beans_left=4, hold_seconds=3.2, smooth=False) == 0,
    )


def test_feed_hold_accumulates_continuously_and_resets_on_break():
    state = FeedHoldState()
    state = update_feed_hold(
        state,
        bean_count=6,
        in_feed_zone=True,
        dt=1.5,
    )
    state = update_feed_hold(
        state,
        bean_count=6,
        in_feed_zone=True,
        dt=1.5,
    )

    expect("feed hold passes at exactly three seconds", state.completed)
    expect(
        "feed hold tracks elapsed time",
        abs(state.hold_seconds - 3.0) < 1e-9,
    )

    interrupted = update_feed_hold(
        state,
        bean_count=6,
        in_feed_zone=False,
        dt=0.1,
    )
    expect(
        "feed hold resets when zone condition breaks",
        not interrupted.completed,
    )
    expect(
        "feed hold reset clears elapsed time",
        interrupted.hold_seconds == 0.0,
    )

    no_beans = update_feed_hold(
        FeedHoldState(),
        bean_count=0,
        in_feed_zone=True,
        dt=3.0,
    )
    expect("feed hold requires beans on spoon", not no_beans.completed)


def test_bean_recovery_counts_sphere_volume_and_thresholds():
    center = TASK3_BEAN_RECOVERY_REGION.center
    inside = [center, Point3D(center.x + 0.19, center.y, center.z)]
    outside = [Point3D(center.x + 0.21, center.y, center.z)]

    expect(
        "sphere includes boundary points",
        count_points_in_sphere(
            [inside[0], Point3D(center.x + 0.2, center.y, center.z)],
            TASK3_BEAN_RECOVERY_REGION,
        )
        == 2,
    )
    expect(
        "sphere excludes outside points",
        count_points_in_sphere([*inside, *outside], TASK3_BEAN_RECOVERY_REGION)
        == 2,
    )
    expect("bean recovery 100 percent", bean_recovery_score(10, 10) == 4)
    expect("bean recovery 90 percent", bean_recovery_score(9, 10) == 3)
    expect("bean recovery 80 percent", bean_recovery_score(8, 10) == 2)
    expect("bean recovery below 80 percent", bean_recovery_score(7, 10) == 0)
    expect(
        "bean recovery handles zero beans",
        bean_recovery_score(0, 0) == 0,
    )


def test_stage4_scores_utensils_overlapping_sink_above_tabletop():
    sink = TASK3_SINK_REGION.bounds
    poses = {
        "simple_tray": Bounds2D(
            sink.x_min, sink.y_min, sink.x_max, sink.y_max
        ),
        "bowl2": Bounds2D(
            sink.x_min - 0.05,
            sink.y_min,
            sink.x_min + 0.05,
            sink.y_max,
        ),
        "spoon2": Bounds2D(
            sink.x_max + 0.01,
            sink.y_min,
            sink.x_max + 0.10,
            sink.y_max,
        ),
        "plate2": Bounds2D(sink.x_min, sink.y_min, sink.x_max, sink.y_max),
        "cup": Bounds2D(sink.x_min, sink.y_min, sink.x_max, sink.y_max),
    }
    z_values = {
        "simple_tray": 0.80,
        "bowl2": 0.80,
        "spoon2": 0.80,
        "plate2": 0.70,
        "cup": TASK3_SINK_REGION.tabletop_z,
    }

    result = score_stage4_cleanup(poses, z_values)

    expect("stage4 counts overlap and tabletop z", result.score == 3)
    expect(
        "stage4 uses default utensil list",
        result.max_score == len(DEFAULT_UTENSIL_OBJECTS),
    )
    expect(
        "stage4 records returned objects",
        result.passed == ["simple_tray", "bowl2", "cup"],
    )


TEST_GROUPS = {
    "stage1": [
        test_kitchen_and_dining_area_classification,
        test_dining_classification_respects_dining_lower_y_bound,
        test_stage1_counts_objects_moved_to_dining_area,
    ],
    "stage2": [
        test_feed_score_requires_smooth_three_second_hold,
        test_feed_hold_accumulates_continuously_and_resets_on_break,
    ],
    "stage3": [
        test_bean_recovery_counts_sphere_volume_and_thresholds,
    ],
    "stage4": [
        test_stage4_scores_utensils_overlapping_sink_above_tabletop,
    ],
}
TEST_GROUPS["all"] = [
    test
    for group_name in ("stage1", "stage2", "stage3", "stage4")
    for test in TEST_GROUPS[group_name]
]


def main():
    group_name = sys.argv[1].lower() if len(sys.argv) > 1 else "all"
    if group_name not in TEST_GROUPS:
        allowed = ", ".join(TEST_GROUPS)
        raise SystemExit(f"Unknown test group '{group_name}'. Use: {allowed}")

    tests = TEST_GROUPS[group_name]
    for test in tests:
        test()
    print(f"\n{len(tests)} task3 {group_name} grading tests passed.")


if __name__ == "__main__":
    main()
