#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Pure IoU + orientation evaluation logic for task2.

Nothing here touches ROS message parsing: the node parses the semantic mask and
stamps and passes plain values in, so this module is fully unit-testable with
lightweight stubs.
"""

import json
from typing import Any

import numpy as np
from image_utils import bbox_from_detection, iter_detection_classifications

BBox = tuple[float, float, float, float]


# --------------------------------------------------------------------------- #
# Geometry
# --------------------------------------------------------------------------- #
def bbox_area(bbox: BBox) -> float:
    x1, y1, x2, y2 = bbox
    w = max(0.0, float(x2) - float(x1))
    h = max(0.0, float(y2) - float(y1))
    return w * h


def bbox_intersection_area(a: BBox, b: BBox) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    x_left = max(float(ax1), float(bx1))
    y_top = max(float(ay1), float(by1))
    x_right = min(float(ax2), float(bx2))
    y_bottom = min(float(ay2), float(by2))
    return max(0.0, x_right - x_left) * max(0.0, y_bottom - y_top)


def bbox_to_dict(bbox: BBox) -> dict[str, float]:
    x1, y1, x2, y2 = bbox
    return {"x1": float(x1), "y1": float(y1), "x2": float(x2), "y2": float(y2)}


# --------------------------------------------------------------------------- #
# Detection / label helpers
# --------------------------------------------------------------------------- #
def detection_best_score(detection) -> float:
    best = float("-inf")
    for _, score in iter_detection_classifications(detection):
        if score is None:
            continue
        best = max(best, score)
    return 0.0 if best == float("-inf") else best


def detection_matches_label(
    detection, target_label: str, target_id: int | None
) -> bool:
    # Isaac Sim's BBox2D bridge encodes class_id as either the label name
    # or its integer ID as a string, depending on the bridge version;
    # check all three forms.
    for class_id, _ in iter_detection_classifications(detection):
        class_id_raw = class_id.strip()
        if not class_id_raw:
            continue
        if class_id_raw.lower() == target_label:
            return True
        if target_id is not None:
            if class_id_raw == str(target_id):
                return True
            try:
                if int(class_id_raw) == int(target_id):
                    return True
            except (TypeError, ValueError):
                pass
    return False


def select_best_bbox_for_label(
    bbox_msg, target_label: str, target_id: int
) -> BBox | None:
    best_bbox: BBox | None = None
    best_score = float("-inf")
    for det in getattr(bbox_msg, "detections", []) or []:
        if not detection_matches_label(det, target_label, target_id):
            continue
        bbox_coords = bbox_from_detection(det)
        if bbox_coords is None:
            continue
        det_score = detection_best_score(det)
        if det_score > best_score:
            best_score = det_score
            best_bbox = bbox_coords
    return best_bbox


def parse_semantic_label_map(payload: str) -> dict[str, int]:
    try:
        obj = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f"Failed to parse semantic labels payload: {exc}"
        ) from exc

    if not isinstance(obj, dict):
        raise ValueError("Semantic labels payload is not a JSON object")

    label_to_id: dict[str, int] = {}
    for raw_key, raw_value in obj.items():
        try:
            label_id = int(raw_key)
        except (TypeError, ValueError):
            continue

        label_name = None
        if isinstance(raw_value, dict):
            for key in ("class", "label", "name"):
                candidate = raw_value.get(key)
                if isinstance(candidate, str) and candidate.strip():
                    label_name = candidate.strip().lower()
                    break
        elif isinstance(raw_value, str) and raw_value.strip():
            label_name = raw_value.strip().lower()

        if label_name and label_name not in label_to_id:
            label_to_id[label_name] = label_id

    if not label_to_id:
        raise ValueError(
            "Semantic labels payload did not contain any class label entries"
        )

    return label_to_id


def count_pixels_for_hint_label(
    label_array: np.ndarray, label_name: str, semantic_hints: dict[int, str]
) -> int:
    for raw_id, name in semantic_hints.items():
        if name == label_name:
            return int(np.sum(label_array == raw_id))
    return 0


# --------------------------------------------------------------------------- #
# Main evaluation
# --------------------------------------------------------------------------- #
def evaluate_thermalpad_target_iou(
    bbox_msg,
    semantic_labels_payload: str,
    *,
    thermalpad_label: str,
    liner_label: str,
    target_label: str,
    semantic_hints: dict[int, str],
    label_array: np.ndarray | None = None,
    current_frame_stamp: str = "",
    bbox_frame_stamp: str = "",
) -> dict[str, Any]:
    """Compute bbox IoU between the active pad (liner/thermalpad) and target.

    ``label_array`` is the parsed int32 semantic mask, required only to resolve
    the case where both liner and thermalpad bboxes are present.
    """
    if bbox_msg is None:
        raise ValueError(
            "BBox message is required for bbox-based IoU evaluation"
        )

    label_to_id = parse_semantic_label_map(semantic_labels_payload)
    thermalpad_id = label_to_id.get(thermalpad_label)
    liner_id = label_to_id.get(liner_label)
    target_id = label_to_id.get(target_label)

    base: dict[str, Any] = {
        "thermalpad_label": thermalpad_label,
        "liner_label": liner_label,
        "target_label": target_label,
        "thermalpad_label_id": int(thermalpad_id)
        if thermalpad_id is not None
        else None,
        "liner_label_id": int(liner_id) if liner_id is not None else None,
        "target_label_id": int(target_id) if target_id is not None else None,
        "current_frame_stamp": current_frame_stamp,
        "bbox_frame_stamp": bbox_frame_stamp,
    }

    def _zero_result(
        orientation_case: str, target_bbox_val=None
    ) -> dict[str, Any]:
        target_area_val = (
            float(bbox_area(target_bbox_val))
            if target_bbox_val is not None
            else 0.0
        )
        return {
            "metric": "iou_pad_vs_target_current",
            "iou_thermalpad_vs_target_current": 0.0,
            "is_orientation_correct": False,
            "orientation_case": orientation_case,
            "pad_source_label": "",
            "intersection_area_pixels": 0.0,
            "union_area_pixels": 0.0,
            "pad_area_pixels": 0.0,
            "target_area_pixels": target_area_val,
            "coverage_on_target": 0.0,
            "precision_on_pad": 0.0,
            "pad_bbox": None,
            "target_bbox": bbox_to_dict(target_bbox_val)
            if target_bbox_val is not None
            else None,
            **base,
        }

    # Target must be present.
    if target_id is None:
        return _zero_result("no_target_label")
    target_bbox = select_best_bbox_for_label(
        bbox_msg, target_label, int(target_id)
    )
    if target_bbox is None:
        return _zero_result("no_target_bbox")

    thermalpad_bbox = (
        select_best_bbox_for_label(
            bbox_msg, thermalpad_label, int(thermalpad_id)
        )
        if thermalpad_id is not None
        else None
    )
    liner_bbox = (
        select_best_bbox_for_label(bbox_msg, liner_label, int(liner_id))
        if liner_id is not None
        else None
    )

    has_thermalpad = thermalpad_bbox is not None
    has_liner = liner_bbox is not None

    if not has_thermalpad and not has_liner:
        return _zero_result("neither_pad_present", target_bbox)
    elif has_liner and not has_thermalpad:
        pad_bbox = liner_bbox
        pad_source_label = liner_label
        is_orientation_correct = True
        orientation_case = "liner_only"
    elif has_thermalpad and not has_liner:
        pad_bbox = thermalpad_bbox
        pad_source_label = thermalpad_label
        is_orientation_correct = False
        orientation_case = "thermalpad_only"
    else:
        # Both present -- resolve via pixel counts from the semantic mask.
        pad_bbox = None
        pad_source_label = ""
        is_orientation_correct = False
        orientation_case = "sideways"
        if label_array is not None:
            thermalpad_px = count_pixels_for_hint_label(
                label_array, thermalpad_label, semantic_hints
            )
            liner_px = count_pixels_for_hint_label(
                label_array, liner_label, semantic_hints
            )
            total_px = thermalpad_px + liner_px
            if total_px > 0:
                liner_ratio = liner_px / total_px
                thermalpad_ratio = thermalpad_px / total_px
                # 90 % dominance threshold: below this, the pad is visibly
                # sideways.
                if liner_ratio > 0.9:
                    pad_bbox = liner_bbox
                    pad_source_label = liner_label
                    is_orientation_correct = True
                    orientation_case = "both_liner_dominant"
                elif thermalpad_ratio > 0.9:
                    pad_bbox = thermalpad_bbox
                    pad_source_label = thermalpad_label
                    is_orientation_correct = False
                    orientation_case = "both_thermalpad_dominant"
                # else: sideways, pad_bbox stays None
        if pad_bbox is None:
            return _zero_result(orientation_case, target_bbox)

    intersection = bbox_intersection_area(pad_bbox, target_bbox)
    pad_area = bbox_area(pad_bbox)
    target_area = bbox_area(target_bbox)
    union = float(pad_area + target_area - intersection)
    iou = float(intersection / union) if union > 0.0 else 0.0
    coverage_on_target = (
        float(intersection / target_area) if target_area > 0.0 else 0.0
    )
    precision_on_pad = (
        float(intersection / pad_area) if pad_area > 0.0 else 0.0
    )

    return {
        "metric": "iou_pad_vs_target_current",
        "iou_thermalpad_vs_target_current": iou,
        "is_orientation_correct": is_orientation_correct,
        "orientation_case": orientation_case,
        "pad_source_label": pad_source_label,
        "intersection_area_pixels": float(intersection),
        "union_area_pixels": float(union),
        "pad_area_pixels": float(pad_area),
        "target_area_pixels": float(target_area),
        "coverage_on_target": coverage_on_target,
        "precision_on_pad": precision_on_pad,
        "pad_bbox": bbox_to_dict(pad_bbox),
        "target_bbox": bbox_to_dict(target_bbox),
        **base,
    }
