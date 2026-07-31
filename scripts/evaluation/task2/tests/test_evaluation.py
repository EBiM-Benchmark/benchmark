#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the pure task2 evaluation logic (no ROS required).

Run: python3 scripts/evaluation/task2/tests/test_evaluation.py
Only depends on numpy. Builds duck-typed stubs mimicking vision_msgs
detections.
"""

import json
import os
import sys
from types import SimpleNamespace as NS

import numpy as np

# Make the flat eval modules importable when run directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import SEMANTIC_RAW_ID_NAME_HINTS  # noqa: E402
from evaluation import (  # noqa: E402
    evaluate_thermalpad_target_iou,
    hints_from_label_payload,
)

# semantic_labels topic mapping (starts at 0, no 'unlabeled') --
# distinct from the raw int32 mask scheme in SEMANTIC_RAW_ID_NAME_HINTS,
# on purpose.
LABELS_PAYLOAD = json.dumps(
    {
        "0": {"class": "liner"},
        "1": {"class": "thermalpad"},
        "2": {"class": "board"},
        "3": {"class": "target"},
    }
)
LABELS_NO_TARGET = json.dumps(
    {
        "0": {"class": "liner"},
        "1": {"class": "thermalpad"},
        "2": {"class": "board"},
    }
)

CLASS_ID = {"liner": "0", "thermalpad": "1", "board": "2", "target": "3"}

# The loose annotator's own map. Deliberately a DIFFERENT scheme from
# CLASS_ID above: "0" is liner in the tight map but board here, "1" is
# thermalpad there but target here. Resolving one stream through the
# other's map therefore picks the wrong class, which is the bug this
# split exists to prevent.
LOOSE_LABELS_PAYLOAD = json.dumps(
    {
        "0": {"class": "board"},
        "1": {"class": "target"},
        "2": {"class": "liner"},
        "3": {"class": "thermalpad"},
    }
)
LOOSE_CLASS_ID = {
    "board": "0",
    "target": "1",
    "liner": "2",
    "thermalpad": "3",
}


def make_detection(label: str, x1, y1, x2, y2, score=1.0, class_ids=None):
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    bbox = NS(
        center=NS(position=NS(x=cx, y=cy)), size_x=x2 - x1, size_y=y2 - y1
    )
    ids = CLASS_ID if class_ids is None else class_ids
    hypothesis = NS(class_id=ids[label], score=score)
    return NS(bbox=bbox, results=[NS(hypothesis=hypothesis)])


def make_bbox_msg(detections):
    return NS(
        detections=detections,
        header=NS(frame_id="eval_camera", stamp=NS(sec=1, nanosec=0)),
    )


def mask(liner_px: int, thermalpad_px: int) -> np.ndarray:
    """Build a flat int32 mask with given pixel counts (raw-ID scheme)."""
    liner_id = next(
        k for k, v in SEMANTIC_RAW_ID_NAME_HINTS.items() if v == "liner"
    )
    therm_id = next(
        k for k, v in SEMANTIC_RAW_ID_NAME_HINTS.items() if v == "thermalpad"
    )
    arr = np.zeros(liner_px + thermalpad_px + 1, dtype=np.int32)
    arr[:liner_px] = liner_id
    arr[liner_px : liner_px + thermalpad_px] = therm_id
    return arr


def run_eval(
    bbox_msg,
    payload=LABELS_PAYLOAD,
    label_array=None,
    target_bbox_msg=None,
    target_labels_payload=None,
):
    return evaluate_thermalpad_target_iou(
        bbox_msg,
        payload,
        thermalpad_label="thermalpad",
        liner_label="liner",
        target_label="target",
        semantic_hints=SEMANTIC_RAW_ID_NAME_HINTS,
        label_array=label_array,
        target_bbox_msg=target_bbox_msg,
        target_labels_payload=target_labels_payload,
    )


# Target box used across tests.
TARGET = ("target", 100, 100, 200, 200)  # area 10000


def expect(name, cond):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}")
    if not cond:
        raise AssertionError(name)


def test_liner_only():
    # liner overlaps target by half horizontally -> intersection 50*100=5000.
    msg = make_bbox_msg(
        [make_detection("liner", 150, 100, 250, 200), make_detection(*TARGET)]
    )
    r = run_eval(msg)
    expect("liner_only case", r["orientation_case"] == "liner_only")
    expect("liner_only correct", r["is_orientation_correct"] is True)
    expect("liner_only pad", r["pad_source_label"] == "liner")
    # IoU = 5000 / (10000 + 10000 - 5000) = 1/3.
    expect(
        "liner_only iou",
        abs(r["iou_thermalpad_vs_target_current"] - (5000 / 15000)) < 1e-6,
    )
    expect("liner_only coverage", abs(r["coverage_on_target"] - 0.5) < 1e-6)


def test_thermalpad_only():
    msg = make_bbox_msg(
        [
            make_detection("thermalpad", 100, 100, 200, 200),
            make_detection(*TARGET),
        ]
    )
    r = run_eval(msg)
    expect("thermalpad_only case", r["orientation_case"] == "thermalpad_only")
    expect("thermalpad_only wrong", r["is_orientation_correct"] is False)
    expect(
        "thermalpad_only iou==1",
        abs(r["iou_thermalpad_vs_target_current"] - 1.0) < 1e-6,
    )


def test_both_liner_dominant():
    msg = make_bbox_msg(
        [
            make_detection("liner", 150, 100, 250, 200),
            make_detection("thermalpad", 0, 0, 10, 10),
            make_detection(*TARGET),
        ]
    )
    r = run_eval(msg, label_array=mask(liner_px=95, thermalpad_px=5))
    expect("both_liner case", r["orientation_case"] == "both_liner_dominant")
    expect("both_liner correct", r["is_orientation_correct"] is True)
    expect("both_liner pad", r["pad_source_label"] == "liner")


def test_both_thermalpad_dominant():
    msg = make_bbox_msg(
        [
            make_detection("liner", 0, 0, 10, 10),
            make_detection("thermalpad", 100, 100, 200, 200),
            make_detection(*TARGET),
        ]
    )
    r = run_eval(msg, label_array=mask(liner_px=5, thermalpad_px=95))
    expect(
        "both_thermalpad case",
        r["orientation_case"] == "both_thermalpad_dominant",
    )
    expect("both_thermalpad wrong", r["is_orientation_correct"] is False)
    expect(
        "both_thermalpad iou==1",
        abs(r["iou_thermalpad_vs_target_current"] - 1.0) < 1e-6,
    )


def test_sideways():
    msg = make_bbox_msg(
        [
            make_detection("liner", 150, 100, 250, 200),
            make_detection("thermalpad", 100, 100, 200, 200),
            make_detection(*TARGET),
        ]
    )
    r = run_eval(msg, label_array=mask(liner_px=50, thermalpad_px=50))
    expect("sideways case", r["orientation_case"] == "sideways")
    expect("sideways wrong", r["is_orientation_correct"] is False)
    expect("sideways iou==0", r["iou_thermalpad_vs_target_current"] == 0.0)
    expect("sideways pad null", r["pad_bbox"] is None)


def test_neither_pad_present():
    msg = make_bbox_msg([make_detection(*TARGET)])
    r = run_eval(msg)
    expect("neither case", r["orientation_case"] == "neither_pad_present")
    expect("neither iou==0", r["iou_thermalpad_vs_target_current"] == 0.0)
    expect("neither target bbox set", r["target_bbox"] is not None)


def test_no_target_label():
    msg = make_bbox_msg([make_detection("liner", 150, 100, 250, 200)])
    r = run_eval(msg, payload=LABELS_NO_TARGET)
    expect("no_target_label case", r["orientation_case"] == "no_target_label")
    expect(
        "no_target_label iou==0", r["iou_thermalpad_vs_target_current"] == 0.0
    )


def test_no_target_bbox():
    # target is in the label map but no target detection present.
    msg = make_bbox_msg([make_detection("liner", 150, 100, 250, 200)])
    r = run_eval(msg)
    expect("no_target_bbox case", r["orientation_case"] == "no_target_bbox")
    expect(
        "no_target_bbox iou==0", r["iou_thermalpad_vs_target_current"] == 0.0
    )


def test_hints_from_label_payload():
    # Segmentation-annotator payload: raw mask IDs with reserved 0/1 and a
    # session-specific class order (as captured live from Isaac Sim 5.1).
    payload = json.dumps(
        {
            "0": {"class": "BACKGROUND"},
            "1": {"class": "UNLABELLED"},
            "2": {"class": "target"},
            "3": {"class": "thermalpad"},
            "4": {"class": "liner"},
            "5": {"class": "board"},
            "time_stamp": {"sec": 53, "nanosec": 866669476},
        }
    )
    hints = hints_from_label_payload(payload)
    expect(
        "hints seg scheme",
        hints
        == {
            0: "background",
            1: "unlabelled",
            2: "target",
            3: "thermalpad",
            4: "liner",
            5: "board",
        },
    )
    # Duplicate names keep the first ID (mirrors parse_semantic_label_map).
    dup = json.dumps({"2": {"class": "liner"}, "4": {"class": "liner"}})
    expect(
        "hints dup first-win", hints_from_label_payload(dup) == {2: "liner"}
    )
    expect("hints bad json", hints_from_label_payload("not json") is None)
    expect(
        "hints no classes",
        hints_from_label_payload(json.dumps({"time_stamp": {}})) is None,
    )


def test_target_from_loose_stream():
    # The pad covers the target exactly (same 0.12 x 0.02 footprint), so
    # the tight annotator dropped the target from both its detections and
    # its label map. The loose annotator still reports it.
    tight = make_bbox_msg([make_detection("liner", 100, 100, 200, 200)])
    loose = make_bbox_msg(
        [
            make_detection(
                "target", 100, 100, 200, 200, class_ids=LOOSE_CLASS_ID
            ),
            make_detection(
                "liner", 100, 100, 200, 200, class_ids=LOOSE_CLASS_ID
            ),
        ]
    )
    r = run_eval(
        tight,
        payload=LABELS_NO_TARGET,
        target_bbox_msg=loose,
        target_labels_payload=LOOSE_LABELS_PAYLOAD,
    )
    expect("loose target case", r["orientation_case"] == "liner_only")
    expect("loose target correct", r["is_orientation_correct"] is True)
    expect(
        "loose target iou==1",
        abs(r["iou_thermalpad_vs_target_current"] - 1.0) < 1e-6,
    )
    # target id from the LOOSE map, liner id from the TIGHT map.
    expect("loose target id", r["target_label_id"] == 1)
    expect("tight liner id", r["liner_label_id"] == 0)


def test_tight_and_loose_schemes_are_independent():
    # Pad geometry must come from the tight stream and the target from the
    # loose one; swapping either would resolve an ID through the wrong map.
    tight = make_bbox_msg([make_detection("liner", 120, 100, 220, 200)])
    loose = make_bbox_msg(
        [
            make_detection(
                "target", 100, 100, 200, 200, class_ids=LOOSE_CLASS_ID
            ),
            make_detection("board", 0, 0, 10, 10, class_ids=LOOSE_CLASS_ID),
        ]
    )
    r = run_eval(
        tight,
        payload=LABELS_NO_TARGET,
        target_bbox_msg=loose,
        target_labels_payload=LOOSE_LABELS_PAYLOAD,
    )
    expect("independent case", r["orientation_case"] == "liner_only")
    expect(
        "independent target bbox",
        r["target_bbox"]
        == {"x1": 100.0, "y1": 100.0, "x2": 200.0, "y2": 200.0},
    )
    expect(
        "independent pad bbox",
        r["pad_bbox"] == {"x1": 120.0, "y1": 100.0, "x2": 220.0, "y2": 200.0},
    )
    # intersection 80*100=8000, union 10000+10000-8000=12000.
    expect(
        "independent iou",
        abs(r["iou_thermalpad_vs_target_current"] - (8000 / 12000)) < 1e-6,
    )


def test_loose_absent_falls_back_to_tight():
    # A scene predating the loose helper: both loose parameters are None
    # and the result must be identical to the tight-only behaviour.
    msg = make_bbox_msg(
        [make_detection("liner", 150, 100, 250, 200), make_detection(*TARGET)]
    )
    baseline = run_eval(msg)
    with_none = run_eval(msg, target_bbox_msg=None, target_labels_payload=None)
    expect("loose fallback identical", baseline == with_none)
    expect(
        "loose fallback case",
        with_none["orientation_case"] == "liner_only",
    )


def test_partial_loose_arguments_fall_back_to_tight():
    # The two loose parameters come from independent ROS subscriptions, so
    # one can arrive without the other. Loose class IDs are meaningless
    # without the loose map, so either half alone must fall back to the
    # tight stream rather than half-resolve.
    msg = make_bbox_msg(
        [make_detection("liner", 150, 100, 250, 200), make_detection(*TARGET)]
    )
    loose = make_bbox_msg(
        [make_detection("target", 0, 0, 10, 10, class_ids=LOOSE_CLASS_ID)]
    )
    baseline = run_eval(msg)
    bbox_only = run_eval(msg, target_bbox_msg=loose)
    payload_only = run_eval(msg, target_labels_payload=LOOSE_LABELS_PAYLOAD)
    expect("partial loose: bbox only falls back", bbox_only == baseline)
    expect("partial loose: payload only falls back", payload_only == baseline)
    # The loose target box (0,0,10,10) must NOT have been used.
    expect(
        "partial loose: target bbox is the tight one",
        bbox_only["target_bbox"]
        == {"x1": 100.0, "y1": 100.0, "x2": 200.0, "y2": 200.0},
    )


def main():
    tests = [
        test_liner_only,
        test_thermalpad_only,
        test_both_liner_dominant,
        test_both_thermalpad_dominant,
        test_sideways,
        test_neither_pad_present,
        test_no_target_label,
        test_no_target_bbox,
        test_hints_from_label_payload,
        test_target_from_loose_stream,
        test_tight_and_loose_schemes_are_independent,
        test_loose_absent_falls_back_to_tight,
        test_partial_loose_arguments_fall_back_to_tight,
    ]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} evaluation tests passed.")


if __name__ == "__main__":
    main()
