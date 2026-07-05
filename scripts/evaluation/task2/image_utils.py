#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Pure image / bounding-box helpers for the task2 eval service.

These functions operate on duck-typed ROS messages (accessed via ``getattr``)
and numpy arrays only -- they hold no node state and import no ROS runtime, so
they can be unit-tested with plain stubs.
"""

import cv2
import numpy as np

BBox = tuple[float, float, float, float]


# --------------------------------------------------------------------------- #
# Encoding helpers
# --------------------------------------------------------------------------- #
def channels_from_encoding(encoding: str) -> int:
    if encoding in {"bgr8", "rgb8"}:
        return 3
    if encoding in {"bgra8", "rgba8"}:
        return 4
    if encoding == "mono8":
        return 1
    raise ValueError(
        f"Unsupported image encoding: {encoding}. "
        "Supported: bgr8, rgb8, bgra8, rgba8, mono8"
    )


def ros_image_to_bgr(msg) -> np.ndarray:
    if msg.height == 0 or msg.width == 0:
        raise ValueError("Received empty image dimensions")

    channels = channels_from_encoding(msg.encoding)
    expected_size = msg.height * msg.width * channels
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if raw.size < expected_size:
        raise ValueError(
            f"Image buffer too small for encoding '{msg.encoding}': "
            f"have {raw.size}, expected at least {expected_size}"
        )

    array = raw[:expected_size].reshape((msg.height, msg.width, channels))

    if msg.encoding == "bgr8":
        return array
    if msg.encoding == "rgb8":
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgra8":
        return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    if msg.encoding == "rgba8":
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    if msg.encoding == "mono8":
        gray = array[:, :, 0]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    raise ValueError(
        f"Unsupported image encoding: {msg.encoding}. "
        "Supported: bgr8, rgb8, bgra8, rgba8, mono8"
    )


def ros_image_to_label_array(msg) -> np.ndarray:
    if msg.height == 0 or msg.width == 0:
        raise ValueError("Received empty semantic label image dimensions")
    if msg.encoding != "32SC1":
        raise ValueError(
            f"Unsupported semantic label encoding: {msg.encoding}"
        )

    expected_size = msg.height * msg.width
    labels = np.frombuffer(msg.data, dtype=np.int32)
    if labels.size < expected_size:
        raise ValueError(
            f"Semantic label buffer too small for encoding '{msg.encoding}': "
            f"have {labels.size}, expected at least {expected_size}"
        )
    return labels[:expected_size].reshape((msg.height, msg.width))


def ros_image_to_depth_array(msg) -> np.ndarray:
    if msg.height == 0 or msg.width == 0:
        raise ValueError("Received empty depth image dimensions")

    if msg.encoding in {"16UC1", "mono16"}:
        dtype = np.uint16
    elif msg.encoding == "32FC1":
        dtype = np.float32
    else:
        raise ValueError(
            f"Unsupported depth encoding: {msg.encoding}. "
            "Supported: 16UC1, mono16, 32FC1"
        )

    expected_size = msg.height * msg.width
    depth = np.frombuffer(msg.data, dtype=dtype)
    if depth.size < expected_size:
        raise ValueError(
            f"Depth buffer too small for encoding '{msg.encoding}': "
            f"have {depth.size}, expected at least {expected_size}"
        )
    return depth[:expected_size].reshape((msg.height, msg.width))


def depth_to_visual(depth: np.ndarray) -> np.ndarray:
    """Normalize a depth map to an 8-bit grayscale visualization."""
    depth_f = depth.astype(np.float32)
    valid = np.isfinite(depth_f) & (depth_f > 0.0)
    if not np.any(valid):
        return np.zeros_like(depth_f, dtype=np.uint8)

    valid_values = depth_f[valid]
    min_v = float(np.min(valid_values))
    max_v = float(np.max(valid_values))
    if max_v <= min_v:
        return np.zeros_like(depth_f, dtype=np.uint8)

    norm = np.zeros_like(depth_f, dtype=np.float32)
    norm[valid] = (depth_f[valid] - min_v) / (max_v - min_v)
    return (norm * 255.0).astype(np.uint8)


def _build_label_color_palette() -> np.ndarray:
    palette = np.zeros((256, 3), dtype=np.uint8)
    indices = np.arange(256, dtype=np.uint16)
    palette[:, 0] = (indices * 37) % 256
    palette[:, 1] = (indices * 67) % 256
    palette[:, 2] = (indices * 97) % 256
    return palette


# Fixed pseudo-color lookup table, computed once at import.
_LABEL_COLOR_PALETTE = _build_label_color_palette()


def label_map_to_color(labels: np.ndarray) -> np.ndarray:
    """Pseudo-color an int32 label map for human-readable visualization."""
    return _LABEL_COLOR_PALETTE[np.mod(labels, 256).astype(np.uint8)]


# --------------------------------------------------------------------------- #
# Bounding-box helpers
# --------------------------------------------------------------------------- #
def bbox_from_detection(detection) -> BBox | None:
    bbox = getattr(detection, "bbox", None)
    center = getattr(bbox, "center", None) if bbox is not None else None
    if center is None:
        return None

    # vision_msgs ≥ 4.x nests xy under center.position; older versions expose
    # x/y directly on center.
    center_pos = getattr(center, "position", center)
    cx = float(getattr(center_pos, "x", 0.0))
    cy = float(getattr(center_pos, "y", 0.0))
    size_x = float(getattr(bbox, "size_x", 0.0)) if bbox is not None else 0.0
    size_y = float(getattr(bbox, "size_y", 0.0)) if bbox is not None else 0.0
    return (
        cx - size_x / 2.0,
        cy - size_y / 2.0,
        cx + size_x / 2.0,
        cy + size_y / 2.0,
    )


def iter_detection_classifications(detection):
    """Yield ``(class_id, score)`` tuples; score may be ``None``."""
    for result in getattr(detection, "results", None) or []:
        hypothesis = getattr(result, "hypothesis", result)
        class_id = str(getattr(hypothesis, "class_id", ""))
        score = getattr(hypothesis, "score", None)
        if score is None:
            # ObjectHypothesisWithPose stores score on .hypothesis;
            # older ObjectHypothesis on the result.
            score = getattr(result, "score", None)
        yield class_id, float(score) if score is not None else None


def label_from_detection(detection) -> str:
    first = next(iter_detection_classifications(detection), None)
    if first is None:
        return ""
    class_id, score = first
    if score is None:
        return str(class_id)
    return f"{class_id}:{score:.2f}"


def draw_bbox_overlay(image: np.ndarray, bbox_msg) -> np.ndarray:
    for det in getattr(bbox_msg, "detections", []):
        bbox_coords = bbox_from_detection(det)
        if bbox_coords is None:
            continue

        x1, y1, x2, y2 = bbox_coords
        x1 = int(max(0.0, x1))
        y1 = int(max(0.0, y1))
        x2 = int(min(image.shape[1] - 1.0, x2))
        y2 = int(min(image.shape[0] - 1.0, y2))

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        label = label_from_detection(det)
        if label:
            cv2.putText(
                image,
                label,
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
    return image


def bbox_2d_array_to_dict(
    bbox_msg, only_top_per_class: bool = False
) -> dict[str, object]:
    detections_with_meta = []
    for det in getattr(bbox_msg, "detections", []):
        bbox_coords = bbox_from_detection(det)
        if bbox_coords is None:
            continue
        x1, y1, x2, y2 = bbox_coords
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        det_dict = {
            "bbox": {
                "center": {"x": cx, "y": cy},
                "size_x": x2 - x1,
                "size_y": y2 - y1,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
            }
        }

        results_out = []
        for class_id, score in iter_detection_classifications(det):
            result_out = {"class_id": class_id}
            if score is not None:
                result_out["score"] = score
            results_out.append(result_out)
        det_dict["results"] = results_out

        primary_class_id = ""
        primary_score = float("-inf")
        if results_out:
            primary_class_id = str(results_out[0].get("class_id", ""))
            if "score" in results_out[0]:
                primary_score = float(results_out[0]["score"])

        detections_with_meta.append(
            {
                "detection": det_dict,
                "primary_class_id": primary_class_id,
                "primary_score": primary_score,
            }
        )

    if only_top_per_class:
        best_by_class: dict[str, dict[str, object]] = {}
        for item in detections_with_meta:
            class_id = str(item["primary_class_id"]) or "__unlabeled__"
            current_best = best_by_class.get(class_id)
            if current_best is None or float(item["primary_score"]) > float(
                current_best["primary_score"]
            ):
                best_by_class[class_id] = item
        detections = [item["detection"] for item in best_by_class.values()]
    else:
        detections = [item["detection"] for item in detections_with_meta]

    header = getattr(bbox_msg, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None

    return {
        "export_mode": "top_per_class_only"
        if only_top_per_class
        else "all_detections",
        "frame_id": str(getattr(header, "frame_id", ""))
        if header is not None
        else "",
        "stamp": {
            "sec": int(getattr(stamp, "sec", 0)) if stamp is not None else 0,
            "nanosec": int(getattr(stamp, "nanosec", 0))
            if stamp is not None
            else 0,
        },
        "source_detection_count": len(detections_with_meta),
        "detection_count": len(detections),
        "detections": detections,
    }


def write_image(path, image: np.ndarray, jpeg_quality: int) -> bool:
    return cv2.imwrite(
        str(path), image, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
    )


def write_png(path, image: np.ndarray) -> bool:
    return cv2.imwrite(str(path), image)
