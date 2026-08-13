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
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

# Make the flat eval modules importable when run directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import (  # noqa: E402
    SEMANTIC_RAW_ID_NAME_HINTS,
    resolve_evaluator_version,
)
from evaluation import (  # noqa: E402
    evaluate_thermalpad_target_iou,
    hints_from_label_payload,
    resolve_semantic_raw_ids,
)
from stream_sync import ALL_STREAMS  # noqa: E402

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


def test_sideways_keeps_iou():
    # 50/50 pixel split: neither pad dominates, but IoU stays real -- the
    # orientation verdict (False) and the placement IoU are decoupled.
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
    expect("sideways pad source", r["pad_source_label"] == "liner+thermalpad")
    # Union of liner (150,100,250,200) and thermalpad (100,100,200,200).
    union_bbox = {"x1": 100.0, "y1": 100.0, "x2": 250.0, "y2": 200.0}
    expect("sideways pad bbox is union", r["pad_bbox"] == union_bbox)
    # union area 150*100=15000; intersection with target (100,100,200,200)
    # is 100*100=10000; iou = 10000 / (15000 + 10000 - 10000) = 2/3.
    expect(
        "sideways iou nonzero",
        abs(r["iou_thermalpad_vs_target_current"] - (10000 / 15000)) < 1e-6,
    )
    expect(
        "sideways confidence is max ratio",
        abs(r["orientation_confidence"] - 0.5) < 1e-9,
    )


def test_sideways_threshold_edge():
    # Same bbox for liner and thermalpad, so the union bbox equals either
    # single bbox: only the pixel ratio moves across the 0.9 dominance
    # boundary. Before the decoupling, ratio 0.899 forced IoU to 0.0
    # while 0.901 kept the real IoU -- a measurement cliff right at the
    # boundary. Now both sides report the identical nonzero IoU.
    def eval_at(liner_px, thermalpad_px):
        msg = make_bbox_msg(
            [
                make_detection("liner", 150, 100, 250, 200),
                make_detection("thermalpad", 150, 100, 250, 200),
                make_detection(*TARGET),
            ]
        )
        return run_eval(
            msg,
            label_array=mask(liner_px=liner_px, thermalpad_px=thermalpad_px),
        )

    below = eval_at(899, 101)  # ratio 0.899, below the 0.9 threshold
    above = eval_at(901, 99)  # ratio 0.901, above the 0.9 threshold

    expect("edge below case", below["orientation_case"] == "sideways")
    expect("edge below wrong", below["is_orientation_correct"] is False)
    expect(
        "edge above case",
        above["orientation_case"] == "both_liner_dominant",
    )
    expect("edge above correct", above["is_orientation_correct"] is True)

    expect(
        "edge iou equal across the boundary",
        below["iou_thermalpad_vs_target_current"]
        == above["iou_thermalpad_vs_target_current"],
    )
    expect(
        "edge iou nonzero",
        below["iou_thermalpad_vs_target_current"] > 0.0,
    )
    expect("edge bboxes equal", below["pad_bbox"] == above["pad_bbox"])


def test_both_present_no_mask_case():
    msg = make_bbox_msg(
        [
            make_detection("liner", 150, 100, 250, 200),
            make_detection("thermalpad", 100, 100, 200, 200),
            make_detection(*TARGET),
        ]
    )
    union_bbox = {"x1": 100.0, "y1": 100.0, "x2": 250.0, "y2": 200.0}

    no_mask = run_eval(msg, label_array=None)
    expect(
        "no_mask case",
        no_mask["orientation_case"] == "both_present_no_mask",
    )
    expect("no_mask wrong", no_mask["is_orientation_correct"] is False)
    expect(
        "no_mask pad source",
        no_mask["pad_source_label"] == "liner+thermalpad",
    )
    expect("no_mask pad bbox is union", no_mask["pad_bbox"] == union_bbox)
    expect(
        "no_mask iou nonzero",
        no_mask["iou_thermalpad_vs_target_current"] > 0.0,
    )
    expect("no_mask confidence", no_mask["orientation_confidence"] == 0.0)
    expect("no_mask pixels none", no_mask["liner_pixels"] is None)
    expect("no_mask ratio none", no_mask["liner_pixel_ratio"] is None)

    # A mask that yields zero pixels for both labels lands in the same
    # case: the ratio would be 0/0, undefined, not zero.
    zero_mask = run_eval(msg, label_array=mask(liner_px=0, thermalpad_px=0))
    expect(
        "zero_mask case",
        zero_mask["orientation_case"] == "both_present_no_mask",
    )
    expect("zero_mask pad bbox is union", zero_mask["pad_bbox"] == union_bbox)
    expect(
        "zero_mask iou matches no_mask iou",
        zero_mask["iou_thermalpad_vs_target_current"]
        == no_mask["iou_thermalpad_vs_target_current"],
    )
    expect("zero_mask pixels none", zero_mask["liner_pixels"] is None)


def test_ratio_fields():
    msg = make_bbox_msg(
        [
            make_detection("liner", 150, 100, 250, 200),
            make_detection("thermalpad", 100, 100, 200, 200),
            make_detection(*TARGET),
        ]
    )
    r = run_eval(msg, label_array=mask(liner_px=60, thermalpad_px=40))
    expect("ratio fields liner pixels", r["liner_pixels"] == 60)
    expect("ratio fields thermalpad pixels", r["thermalpad_pixels"] == 40)
    expect(
        "ratio fields liner ratio value",
        abs(r["liner_pixel_ratio"] - 0.6) < 1e-9,
    )
    expect(
        "ratio fields thermalpad ratio value",
        abs(r["thermalpad_pixel_ratio"] - 0.4) < 1e-9,
    )
    expect(
        "ratio fields sum to one",
        abs(r["liner_pixel_ratio"] + r["thermalpad_pixel_ratio"] - 1.0) < 1e-9,
    )

    liner_msg = make_bbox_msg(
        [
            make_detection("liner", 150, 100, 250, 200),
            make_detection(*TARGET),
        ]
    )
    lr = run_eval(liner_msg)
    expect("liner_only pixels none", lr["liner_pixels"] is None)
    expect(
        "liner_only thermalpad pixels none",
        lr["thermalpad_pixels"] is None,
    )
    expect("liner_only ratio none", lr["liner_pixel_ratio"] is None)
    expect(
        "liner_only thermalpad ratio none",
        lr["thermalpad_pixel_ratio"] is None,
    )
    expect(
        "liner_only confidence is 1.0",
        lr["orientation_confidence"] == 1.0,
    )


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


def test_zero_results_carry_new_fields():
    msg = make_bbox_msg([make_detection("liner", 150, 100, 250, 200)])
    r = run_eval(msg)
    expect("zero case", r["orientation_case"] == "no_target_bbox")
    expect("zero confidence", r["orientation_confidence"] == 0.0)
    expect("zero liner pixels none", r["liner_pixels"] is None)
    expect("zero thermalpad pixels none", r["thermalpad_pixels"] is None)
    expect("zero liner ratio none", r["liner_pixel_ratio"] is None)
    expect("zero thermalpad ratio none", r["thermalpad_pixel_ratio"] is None)


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


def test_diagnostics_passthrough():
    # Every diagnostics field gets a distinct, recognizable value --
    # this is pure passthrough, so nothing may be dropped or altered.
    msg = make_bbox_msg(
        [make_detection("liner", 150, 100, 250, 200), make_detection(*TARGET)]
    )
    stream_stamps = {
        "image_raw": 12.5,
        "semantic_segmentation": 12.48,
        "bbox_2d_tight": 12.51,
    }
    label_provenance = {"semantic_labels_source": "live"}
    r = evaluate_thermalpad_target_iou(
        msg,
        LABELS_PAYLOAD,
        thermalpad_label="thermalpad",
        liner_label="liner",
        target_label="target",
        semantic_hints=SEMANTIC_RAW_ID_NAME_HINTS,
        stream_stamps=stream_stamps,
        sync_status="ok",
        sync_tolerance_s=0.0167,
        sync_anchor_stamp=12.5,
        max_stamp_delta=0.03,
        label_provenance=label_provenance,
        evaluator_version="abc1234",
    )
    expect("passthrough sync status", r["sync"]["status"] == "ok")
    expect("passthrough sync anchor", r["sync"]["anchor_stamp"] == 12.5)
    expect("passthrough sync tolerance", r["sync"]["tolerance_s"] == 0.0167)
    expect("passthrough sync max delta", r["sync"]["max_stamp_delta"] == 0.03)
    expect(
        "passthrough stamps has all nine keys",
        set(r["sync"]["stamps"]) == set(ALL_STREAMS),
    )
    for stream in ALL_STREAMS:
        expect(
            f"passthrough stamp {stream}",
            r["sync"]["stamps"][stream] == stream_stamps.get(stream),
        )
    expect(
        "passthrough label_provenance merged",
        r["label_provenance"]
        == {
            "semantic_labels_source": "live",
            "semantic_raw_ids": {
                "liner": 5,
                "thermalpad": 3,
                "target": 4,
            },
        },
    )
    expect(
        "passthrough evaluator_version", r["evaluator_version"] == "abc1234"
    )


def test_diagnostics_defaults():
    # Existing recorder call shape: none of the new kwargs supplied.
    msg = make_bbox_msg(
        [make_detection("liner", 150, 100, 250, 200), make_detection(*TARGET)]
    )
    r = run_eval(msg)
    expect("defaults sync status empty", r["sync"]["status"] == "")
    expect("defaults sync anchor none", r["sync"]["anchor_stamp"] is None)
    expect("defaults sync tolerance none", r["sync"]["tolerance_s"] is None)
    expect(
        "defaults sync max delta none",
        r["sync"]["max_stamp_delta"] is None,
    )
    expect(
        "defaults stamps has all nine keys",
        set(r["sync"]["stamps"]) == set(ALL_STREAMS),
    )
    expect(
        "defaults stamps all none",
        all(v is None for v in r["sync"]["stamps"].values()),
    )
    expect(
        "defaults label_provenance",
        r["label_provenance"]
        == {"semantic_raw_ids": {"liner": 5, "thermalpad": 3, "target": 4}},
    )
    expect("defaults evaluator_version", r["evaluator_version"] == "")

    # _zero_result path (no_target_bbox) carries the same three keys.
    no_bbox_msg = make_bbox_msg([make_detection("liner", 150, 100, 250, 200)])
    zr = run_eval(no_bbox_msg)
    expect("defaults zero case", zr["orientation_case"] == "no_target_bbox")
    expect("defaults zero sync status", zr["sync"]["status"] == "")
    expect(
        "defaults zero stamps all none",
        all(v is None for v in zr["sync"]["stamps"].values()),
    )
    expect(
        "defaults zero label_provenance",
        zr["label_provenance"]
        == {"semantic_raw_ids": {"liner": 5, "thermalpad": 3, "target": 4}},
    )
    expect("defaults zero evaluator_version", zr["evaluator_version"] == "")


def test_semantic_raw_ids_resolution():
    hints = {5: "liner", 3: "thermalpad", 2: "target"}
    resolved = resolve_semantic_raw_ids(
        hints,
        liner_label="liner",
        thermalpad_label="thermalpad",
        target_label="target",
    )
    expect(
        "semantic raw ids full resolution",
        resolved == {"liner": 5, "thermalpad": 3, "target": 2},
    )

    missing_target = {5: "liner", 3: "thermalpad"}
    resolved_missing = resolve_semantic_raw_ids(
        missing_target,
        liner_label="liner",
        thermalpad_label="thermalpad",
        target_label="target",
    )
    expect(
        "semantic raw ids missing target is None",
        resolved_missing == {"liner": 5, "thermalpad": 3, "target": None},
    )


def test_resolve_evaluator_version():
    env_keys = ("EVAL_TASK2_VERSION", "EVAL_TASK2_VERSION_BUILD")
    saved = {k: os.environ.get(k) for k in env_keys}
    try:
        # (a) EVAL_TASK2_VERSION wins over everything else, including a
        # build-time env var that is also set.
        os.environ["EVAL_TASK2_VERSION"] = "env-version"
        os.environ["EVAL_TASK2_VERSION_BUILD"] = "build-version"
        expect(
            "version: env var wins",
            resolve_evaluator_version(Path("/nonexistent-repo"))
            == "env-version",
        )
        os.environ.pop("EVAL_TASK2_VERSION", None)
        os.environ.pop("EVAL_TASK2_VERSION_BUILD", None)

        sha = "abcdef1234567890abcdef1234567890abcdef12"

        # (b) .git/HEAD -> "ref: <path>" -> loose ref file holding the sha.
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            git_dir = repo / ".git"
            git_dir.mkdir()
            (git_dir / "HEAD").write_text(
                "ref: refs/heads/x\n", encoding="utf-8"
            )
            refs_dir = git_dir / "refs" / "heads"
            refs_dir.mkdir(parents=True)
            (refs_dir / "x").write_text(sha + "\n", encoding="utf-8")
            expect(
                "version: loose ref resolves 7-char sha",
                resolve_evaluator_version(repo) == sha[:7],
            )

        # (c) detached HEAD: HEAD itself holds a bare 40-hex sha.
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            git_dir = repo / ".git"
            git_dir.mkdir()
            (git_dir / "HEAD").write_text(sha + "\n", encoding="utf-8")
            expect(
                "version: detached head resolves 7-char sha",
                resolve_evaluator_version(repo) == sha[:7],
            )

        # (d) packed-refs fallback: no loose ref file, sha lives only in
        # packed-refs (comment and peel lines must be skipped).
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            git_dir = repo / ".git"
            git_dir.mkdir()
            (git_dir / "HEAD").write_text(
                "ref: refs/heads/x\n", encoding="utf-8"
            )
            (git_dir / "packed-refs").write_text(
                "# pack-refs with: peeled fully-peeled sorted\n"
                f"{sha} refs/heads/x\n"
                "^1111111111111111111111111111111111111111\n",
                encoding="utf-8",
            )
            expect(
                "version: packed-refs fallback resolves 7-char sha",
                resolve_evaluator_version(repo) == sha[:7],
            )

        # Worktree gitdir pointer: .git is a *file* pointing elsewhere.
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp) / "worktree"
            repo.mkdir()
            real_git_dir = Path(tmp) / "real.git"
            real_git_dir.mkdir()
            (real_git_dir / "HEAD").write_text(sha + "\n", encoding="utf-8")
            (repo / ".git").write_text(
                f"gitdir: {real_git_dir}\n", encoding="utf-8"
            )
            expect(
                "version: worktree gitdir file resolves 7-char sha",
                resolve_evaluator_version(repo) == sha[:7],
            )

        # (e) no usable .git at all -> falls through to the build-time
        # env var baked into the image.
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            os.environ["EVAL_TASK2_VERSION_BUILD"] = "build-fallback"
            expect(
                "version: build env var fallback",
                resolve_evaluator_version(repo) == "build-fallback",
            )
            os.environ.pop("EVAL_TASK2_VERSION_BUILD", None)

            # (f) nothing resolves at all -> "unknown".
            expect(
                "version: unknown fallback",
                resolve_evaluator_version(repo) == "unknown",
            )
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def main():
    tests = [
        test_liner_only,
        test_thermalpad_only,
        test_both_liner_dominant,
        test_both_thermalpad_dominant,
        test_sideways_keeps_iou,
        test_sideways_threshold_edge,
        test_both_present_no_mask_case,
        test_ratio_fields,
        test_neither_pad_present,
        test_no_target_label,
        test_no_target_bbox,
        test_zero_results_carry_new_fields,
        test_hints_from_label_payload,
        test_target_from_loose_stream,
        test_tight_and_loose_schemes_are_independent,
        test_loose_absent_falls_back_to_tight,
        test_partial_loose_arguments_fall_back_to_tight,
        test_diagnostics_passthrough,
        test_diagnostics_defaults,
        test_semantic_raw_ids_resolution,
        test_resolve_evaluator_version,
    ]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} evaluation tests passed.")


if __name__ == "__main__":
    main()
