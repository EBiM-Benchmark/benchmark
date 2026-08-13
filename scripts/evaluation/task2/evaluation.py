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
from stream_sync import ALL_STREAMS

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


def bbox_union(a: BBox, b: BBox) -> BBox:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    return (
        min(float(ax1), float(bx1)),
        min(float(ay1), float(by1)),
        max(float(ax2), float(bx2)),
        max(float(ay2), float(by2)),
    )


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


def hints_from_label_payload(payload: str) -> dict[int, str] | None:
    """Build a ``{raw_id: label_name}`` map from a semantic-labels payload.

    Inverse of :func:`parse_semantic_label_map`, for payloads published by
    the *segmentation* annotator, whose IDs are the raw mask pixel values.
    Isaac Sim assigns those IDs per session, so a live payload must win over
    the static ``SEMANTIC_RAW_ID_NAME_HINTS`` fallback. Returns ``None`` when
    the payload cannot be parsed (caller keeps its fallback).
    """
    try:
        label_to_id = parse_semantic_label_map(payload)
    except ValueError:
        return None
    hints: dict[int, str] = {}
    for name, raw_id in label_to_id.items():
        # parse_semantic_label_map is first-win on names; keep the same
        # policy per ID.
        if raw_id not in hints:
            hints[raw_id] = name
    return hints


def count_pixels_for_hint_label(
    label_array: np.ndarray, label_name: str, semantic_hints: dict[int, str]
) -> int:
    for raw_id, name in semantic_hints.items():
        if name == label_name:
            return int(np.sum(label_array == raw_id))
    return 0


def pixel_ratios(
    label_array: np.ndarray,
    liner_label: str,
    thermalpad_label: str,
    semantic_hints: dict[int, str],
) -> tuple[int, int]:
    """Return ``(liner_px, thermalpad_px)`` pixel counts (not ratios).

    Named for the ratio math callers derive from these counts; dividing
    by their sum is left to the caller once it has confirmed the sum is
    greater than zero.
    """
    liner_px = count_pixels_for_hint_label(
        label_array, liner_label, semantic_hints
    )
    thermalpad_px = count_pixels_for_hint_label(
        label_array, thermalpad_label, semantic_hints
    )
    return liner_px, thermalpad_px


# --------------------------------------------------------------------------- #
# Diagnostics passthrough
# --------------------------------------------------------------------------- #
def resolve_stream_stamps(
    stream_stamps: dict[str, float | None] | None,
) -> dict[str, float | None]:
    """Project ``stream_stamps`` onto all nine ``ALL_STREAMS`` keys.

    ``stream_stamps=None`` (or empty) returns all nine values as
    ``None``. A partial dict leaves the streams it does not mention as
    ``None``; keys outside ``ALL_STREAMS`` are ignored.
    """
    stamps: dict[str, float | None] = dict.fromkeys(ALL_STREAMS)
    if stream_stamps:
        for stream in ALL_STREAMS:
            if stream in stream_stamps:
                stamps[stream] = stream_stamps[stream]
    return stamps


def resolve_semantic_raw_ids(
    semantic_hints: dict[int, str],
    *,
    liner_label: str,
    thermalpad_label: str,
    target_label: str,
) -> dict[str, int | None]:
    """Resolve raw semantic-mask IDs for liner/thermalpad/target.

    Same first-win policy as :func:`hints_from_label_payload`: for
    each of the three label names, returns the first ``raw_id`` in
    ``semantic_hints`` whose value matches it, or ``None`` when no
    entry matches.
    """
    wanted = {
        "liner": liner_label,
        "thermalpad": thermalpad_label,
        "target": target_label,
    }
    resolved: dict[str, int | None] = dict.fromkeys(wanted, None)
    for raw_id, name in semantic_hints.items():
        for key, label_name in wanted.items():
            if resolved[key] is None and name == label_name:
                resolved[key] = raw_id
    return resolved


# --------------------------------------------------------------------------- #
# Main evaluation
# --------------------------------------------------------------------------- #
def evaluate_thermalpad_target_iou(
    bbox_msg,
    bbox_labels_payload: str,
    *,
    thermalpad_label: str,
    liner_label: str,
    target_label: str,
    semantic_hints: dict[int, str],
    label_array: np.ndarray | None = None,
    current_frame_stamp: str = "",
    bbox_frame_stamp: str = "",
    target_bbox_msg=None,
    target_labels_payload: str | None = None,
    stream_stamps: dict[str, float | None] | None = None,
    sync_status: str = "",
    sync_tolerance_s: float | None = None,
    sync_anchor_stamp: float | None = None,
    max_stamp_delta: float | None = None,
    label_provenance: dict[str, Any] | None = None,
    evaluator_version: str = "",
) -> dict[str, Any]:
    """Compute bbox IoU between the active pad (liner/thermalpad) and target.

    ``bbox_labels_payload`` is the *tight* annotator's id->label map; it
    resolves the liner/thermalpad class IDs in ``bbox_msg``.

    ``target_bbox_msg`` / ``target_labels_payload`` are the *loose*
    annotator's detections and map. Tight bboxes are occlusion-aware and
    drop a fully occluded object, and a correctly placed pad occludes the
    target exactly (identical 0.12 x 0.02 footprints), so the target is
    resolved through the loose stream. Both default to ``None``, in which
    case the tight stream is used for the target too — the behaviour of a
    scene built before the loose helper existed.

    ``label_array`` is the parsed int32 semantic mask, required only to
    resolve the case where both liner and thermalpad bboxes are present.

    Orientation (which pad dominates the mask, if either) and placement
    (pad-vs-target IoU) are independent measurements. When both bboxes
    are present but neither pixel-dominates (``sideways``), or no mask
    is available to resolve them (``both_present_no_mask``),
    ``is_orientation_correct`` is False but the IoU is still computed
    against the union of the two bboxes rather than forced to zero --
    there is no measurement cliff between "just barely sideways" and
    "just barely dominant". Success gating (e.g. orientation correct
    AND iou above a minimum) is decided by the caller, not here.

    ``stream_stamps`` / ``sync_status`` / ``sync_tolerance_s`` /
    ``sync_anchor_stamp`` / ``max_stamp_delta`` are pure passthrough
    diagnostics describing the caller's stream-sync selection (see
    ``stream_sync.EvalStreamSync``) -- this module never computes sync
    itself, only reshapes what it is given. ``stream_stamps`` may be
    partial or ``None``; the result always carries all nine
    ``stream_sync.ALL_STREAMS`` keys, missing ones as ``None``.
    ``label_provenance`` is caller-supplied metadata merged with a
    ``semantic_raw_ids`` map that this function resolves itself from
    ``semantic_hints``. ``evaluator_version`` is an opaque passthrough
    string. All default to empty/``None`` so callers using the
    pre-diagnostics call shape still get null-but-present structure,
    never missing keys.
    """
    if bbox_msg is None:
        raise ValueError(
            "BBox message is required for bbox-based IoU evaluation"
        )

    label_to_id = parse_semantic_label_map(bbox_labels_payload)
    thermalpad_id = label_to_id.get(thermalpad_label)
    liner_id = label_to_id.get(liner_label)
    if target_bbox_msg is not None and target_labels_payload is not None:
        target_msg = target_bbox_msg
        target_label_to_id = parse_semantic_label_map(target_labels_payload)
    else:
        target_msg = bbox_msg
        target_label_to_id = label_to_id
    target_id = target_label_to_id.get(target_label)

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
        # Built here (not duplicated per return path) so _zero_result
        # and the full result below cannot drift out of sync.
        "sync": {
            "status": sync_status,
            "anchor_stamp": sync_anchor_stamp,
            "tolerance_s": sync_tolerance_s,
            "max_stamp_delta": max_stamp_delta,
            "stamps": resolve_stream_stamps(stream_stamps),
        },
        "label_provenance": {
            **(label_provenance or {}),
            "semantic_raw_ids": resolve_semantic_raw_ids(
                semantic_hints,
                liner_label=liner_label,
                thermalpad_label=thermalpad_label,
                target_label=target_label,
            ),
        },
        "evaluator_version": evaluator_version,
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
            "orientation_confidence": 0.0,
            "pad_source_label": "",
            "liner_pixels": None,
            "thermalpad_pixels": None,
            "liner_pixel_ratio": None,
            "thermalpad_pixel_ratio": None,
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
        target_msg, target_label, int(target_id)
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

    liner_px: int | None = None
    thermalpad_px: int | None = None
    liner_ratio: float | None = None
    thermalpad_ratio: float | None = None

    if not has_thermalpad and not has_liner:
        return _zero_result("neither_pad_present", target_bbox)
    elif has_liner and not has_thermalpad:
        pad_bbox = liner_bbox
        pad_source_label = liner_label
        is_orientation_correct = True
        orientation_case = "liner_only"
        orientation_confidence = 1.0
    elif has_thermalpad and not has_liner:
        pad_bbox = thermalpad_bbox
        pad_source_label = thermalpad_label
        is_orientation_correct = False
        orientation_case = "thermalpad_only"
        orientation_confidence = 1.0
    else:
        # Both present -- resolve via pixel counts from the semantic
        # mask. Orientation (dominance) and placement (IoU) are
        # independent: even when neither pad dominates, or no mask is
        # available to tell, evaluation still falls through to the
        # normal IoU computation below, against the union of both
        # bboxes.
        pad_source_label = f"{liner_label}+{thermalpad_label}"
        if label_array is not None:
            raw_liner_px, raw_thermalpad_px = pixel_ratios(
                label_array, liner_label, thermalpad_label, semantic_hints
            )
            total_px = raw_liner_px + raw_thermalpad_px
            if total_px > 0:
                liner_px, thermalpad_px = raw_liner_px, raw_thermalpad_px
                liner_ratio = liner_px / total_px
                thermalpad_ratio = thermalpad_px / total_px

        # 90 % dominance threshold: below this, the pad is visibly
        # sideways.
        if liner_ratio is not None and liner_ratio > 0.9:
            pad_bbox = liner_bbox
            pad_source_label = liner_label
            is_orientation_correct = True
            orientation_case = "both_liner_dominant"
            orientation_confidence = max(liner_ratio, thermalpad_ratio)
        elif thermalpad_ratio is not None and thermalpad_ratio > 0.9:
            pad_bbox = thermalpad_bbox
            pad_source_label = thermalpad_label
            is_orientation_correct = False
            orientation_case = "both_thermalpad_dominant"
            orientation_confidence = max(liner_ratio, thermalpad_ratio)
        elif liner_ratio is not None:
            pad_bbox = bbox_union(liner_bbox, thermalpad_bbox)
            is_orientation_correct = False
            orientation_case = "sideways"
            orientation_confidence = max(liner_ratio, thermalpad_ratio)
        else:
            # No mask, or the mask had zero pixels for both labels.
            pad_bbox = bbox_union(liner_bbox, thermalpad_bbox)
            is_orientation_correct = False
            orientation_case = "both_present_no_mask"
            orientation_confidence = 0.0

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
        "orientation_confidence": orientation_confidence,
        "pad_source_label": pad_source_label,
        "liner_pixels": liner_px,
        "thermalpad_pixels": thermalpad_px,
        "liner_pixel_ratio": liner_ratio,
        "thermalpad_pixel_ratio": thermalpad_ratio,
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
