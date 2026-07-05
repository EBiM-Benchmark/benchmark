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
from evaluation import evaluate_thermalpad_target_iou  # noqa: E402

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


def make_detection(label: str, x1, y1, x2, y2, score=1.0):
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    bbox = NS(
        center=NS(position=NS(x=cx, y=cy)), size_x=x2 - x1, size_y=y2 - y1
    )
    hypothesis = NS(class_id=CLASS_ID[label], score=score)
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


def run_eval(bbox_msg, payload=LABELS_PAYLOAD, label_array=None):
    return evaluate_thermalpad_target_iou(
        bbox_msg,
        payload,
        thermalpad_label="thermalpad",
        liner_label="liner",
        target_label="target",
        semantic_hints=SEMANTIC_RAW_ID_NAME_HINTS,
        label_array=label_array,
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
    ]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} evaluation tests passed.")


if __name__ == "__main__":
    main()
