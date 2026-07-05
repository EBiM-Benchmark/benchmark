#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""ROS2 node for the task2 eval camera service.

Subscribes to the Isaac Sim eval-camera topics, and on each ``Trigger``
call snapshots the latest frame, saves all modalities, and computes
pad-vs-target IoU. Heavy lifting is delegated to ``image_utils``
(conversions) and ``evaluation`` (IoU + orientation), keeping this
module focused on ROS plumbing and IO.
"""

import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from vision_msgs.msg import Detection2DArray
except Exception:  # pragma: no cover - depends on runtime image
    Detection2DArray = None

import image_utils
from config import SEMANTIC_RAW_ID_NAME_HINTS, coerce_bool
from evaluation import evaluate_thermalpad_target_iou


def _stamp_to_string(msg) -> str:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is None:
        return ""
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    return f"{sec}.{nanosec:09d}"


def _artifact_path(out: Path, kind: str, ts: str, ext: str) -> Path:
    return out / f"eval_camera_{kind}_{ts}.{ext}"


@dataclass
class _Snapshot:
    """Latest message of each modality captured atomically under the lock."""

    image: Any = None
    depth: Any = None
    semantic: Any = None
    labels: Any = None
    bbox: Any = None
    camera_info: Any = None


class EvalCameraCaptureService(Node):
    """ROS2 node that snapshots eval-camera streams and evaluates
    pad placement."""

    def __init__(self, config: dict[str, Any]):
        super().__init__("eval_camera_capture_service")

        self._image_topic = str(config["image_topic"])
        self._base_output_dir = Path(str(config["output_dir"]))
        self._evaluate_output_dir = self._base_output_dir / "evaluate"
        self._evaluate_output_dir.mkdir(parents=True, exist_ok=True)
        self._jpeg_quality = int(config["jpeg_quality"])
        self._thermalpad_label = (
            str(config["thermalpad_label"]).strip().lower()
        )
        self._liner_label = str(config["liner_label"]).strip().lower()
        self._target_label = str(config["target_label"]).strip().lower()
        self._bbox_json_top_per_class_only = coerce_bool(
            config["bbox_json_top_per_class_only"]
        )

        # Subscription callbacks and the service handler can run on separate
        # threads with a MultiThreadedExecutor; all _latest_* writes go
        # through this lock.
        self._lock = Lock()
        self._latest_image = None
        self._latest_depth = None
        self._latest_semantic_segmentation = None
        self._latest_semantic_labels = None
        self._latest_bbox_2d_tight = None
        self._latest_camera_info = None

        # Isaac Sim's ROS2 bridge publishes with BEST_EFFORT reliability;
        # qos_profile_sensor_data matches that. A RELIABLE subscriber
        # silently receives nothing.
        self.create_subscription(
            Image, self._image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            str(config["camera_info_topic"]),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(config["depth_topic"]),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(config["semantic_segmentation_topic"]),
            self._on_semantic_segmentation,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(config["semantic_labels_topic"]),
            self._on_semantic_labels,
            qos_profile_sensor_data,
        )
        if Detection2DArray is not None:
            self.create_subscription(
                Detection2DArray,
                str(config["bbox_2d_tight_topic"]),
                self._on_bbox_2d_tight,
                qos_profile_sensor_data,
            )
        else:
            self.get_logger().warn(
                "vision_msgs is not available; bbox overlays/evaluation "
                "are disabled. "
                "Install ros-jazzy-vision-msgs in the runtime image."
            )

        self.create_service(
            Trigger,
            str(config["evaluate_service_name"]),
            self._on_save_request,
        )

        self.get_logger().info(f"Subscribed image topic: {self._image_topic}")
        self.get_logger().info(
            f"Evaluate service ready: {config['evaluate_service_name']}"
        )
        self.get_logger().info(
            f"Output directory: {self._evaluate_output_dir.resolve()}"
        )
        self.get_logger().info(
            f"IoU labels: thermalpad='{self._thermalpad_label}', "
            f"liner='{self._liner_label}', target='{self._target_label}'"
        )

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #
    def _on_image(self, msg):
        with self._lock:
            self._latest_image = msg

    def _on_camera_info(self, msg):
        with self._lock:
            self._latest_camera_info = msg

    def _on_depth(self, msg):
        with self._lock:
            self._latest_depth = msg

    def _on_semantic_segmentation(self, msg):
        with self._lock:
            self._latest_semantic_segmentation = msg

    def _on_semantic_labels(self, msg):
        with self._lock:
            self._latest_semantic_labels = msg

    def _on_bbox_2d_tight(self, msg):
        with self._lock:
            self._latest_bbox_2d_tight = msg

    # ------------------------------------------------------------------ #
    # Service handler
    # ------------------------------------------------------------------ #
    def _capture_latest(self) -> _Snapshot:
        with self._lock:
            return _Snapshot(
                image=self._latest_image,
                depth=self._latest_depth,
                semantic=self._latest_semantic_segmentation,
                labels=self._latest_semantic_labels,
                bbox=self._latest_bbox_2d_tight,
                camera_info=self._latest_camera_info,
            )

    def _on_save_request(self, _request, response):
        snap = self._capture_latest()
        if snap.image is None:
            response.success = False
            response.message = f"No image received yet on {self._image_topic}"
            return response

        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        out = self._evaluate_output_dir
        saved: list[Path] = []
        missing: list[str] = []

        try:
            rgb_bgr = self._save_rgb(out, ts, snap.image, saved)
            self._save_depth(out, ts, snap.depth, saved, missing)
            label_array = self._save_semantic(
                out, ts, snap.semantic, saved, missing
            )
            self._save_labels(out, ts, snap.labels, saved, missing)
            # Modality snapshots above are written regardless; only
            # evaluation requires both.
            if snap.labels is None or snap.bbox is None:
                raise ValueError(
                    "Evaluation requires semantic_labels and bbox_2d_tight"
                )
            eval_result = self._save_eval(out, ts, snap, label_array, saved)
            self._save_bbox_artifacts(out, ts, snap.bbox, rgb_bgr, saved)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response

        info = ""
        if snap.camera_info is not None:
            info = f" frame_id={snap.camera_info.header.frame_id}"
        if missing:
            info += f" missing={','.join(missing)}"
        is_correct = eval_result["is_orientation_correct"]
        eval_msg = (
            f" eval_iou={eval_result['iou_thermalpad_vs_target_current']:.4f}"
            f" orientation={'correct' if is_correct else 'wrong'}"
            f"[{eval_result['orientation_case']}]"
        )

        summary = ", ".join(str(p) for p in saved)
        response.success = True
        response.message = f"Saved [{summary}]{info}{eval_msg}"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ #
    # Per-modality save helpers (raise ValueError on hard failure)
    # ------------------------------------------------------------------ #
    def _save_rgb(self, out, ts, image_msg, saved: list[Path]) -> np.ndarray:
        rgb_bgr = image_utils.ros_image_to_bgr(image_msg)
        rgb_path = _artifact_path(out, "rgb", ts, "jpg")
        if not image_utils.write_image(rgb_path, rgb_bgr, self._jpeg_quality):
            raise ValueError(f"Failed to write JPEG: {rgb_path}")
        saved.append(rgb_path)
        return rgb_bgr

    def _save_depth(
        self, out, ts, depth_msg, saved: list[Path], missing: list[str]
    ) -> None:
        if depth_msg is None:
            missing.append("depth")
            return
        depth_array = image_utils.ros_image_to_depth_array(depth_msg)
        depth_npy = _artifact_path(out, "depth", ts, "npy")
        np.save(str(depth_npy), depth_array)
        saved.append(depth_npy)
        depth_png = _artifact_path(out, "depth", ts, "png")
        if image_utils.write_png(
            depth_png, image_utils.depth_to_visual(depth_array)
        ):
            saved.append(depth_png)

    def _save_semantic(
        self, out, ts, seg_msg, saved: list[Path], missing: list[str]
    ) -> np.ndarray | None:
        if seg_msg is None:
            missing.append("semantic_segmentation")
            return None
        # Parse the int32 mask once; derive both the colorized .png and
        # raw .npy from it.
        label_array = image_utils.ros_image_to_label_array(seg_msg)
        seg_png = _artifact_path(out, "semantic_segmentation", ts, "png")
        if not image_utils.write_png(
            seg_png, image_utils.label_map_to_color(label_array)
        ):
            raise ValueError(
                f"Failed to write semantic segmentation PNG: {seg_png}"
            )
        saved.append(seg_png)
        seg_npy = _artifact_path(out, "semantic_segmentation", ts, "npy")
        np.save(str(seg_npy), label_array)
        saved.append(seg_npy)
        return label_array

    @staticmethod
    def _save_labels(
        out, ts, labels_msg, saved: list[Path], missing: list[str]
    ) -> None:
        if labels_msg is None:
            missing.append("semantic_labels")
            return
        labels_path = _artifact_path(out, "semantic_labels", ts, "txt")
        labels_path.write_text(labels_msg.data, encoding="utf-8")
        saved.append(labels_path)

    def _save_eval(
        self, out, ts, snap: _Snapshot, label_array, saved: list[Path]
    ) -> dict[str, Any]:
        try:
            eval_result = evaluate_thermalpad_target_iou(
                snap.bbox,
                snap.labels.data,
                thermalpad_label=self._thermalpad_label,
                liner_label=self._liner_label,
                target_label=self._target_label,
                semantic_hints=SEMANTIC_RAW_ID_NAME_HINTS,
                label_array=label_array,
                current_frame_stamp=_stamp_to_string(snap.semantic)
                if snap.semantic is not None
                else "",
                bbox_frame_stamp=_stamp_to_string(snap.bbox),
            )
        except ValueError as exc:
            raise ValueError(f"Evaluation failed: {exc}") from exc
        eval_path = _artifact_path(out, "iou", ts, "json")
        eval_path.write_text(
            json.dumps(eval_result, indent=2), encoding="utf-8"
        )
        saved.append(eval_path)
        return eval_result

    def _save_bbox_artifacts(
        self, out, ts, bbox_msg, rgb_bgr, saved: list[Path]
    ) -> None:
        bbox_json = _artifact_path(out, "bbox2d_tight", ts, "json")
        bbox_payload = image_utils.bbox_2d_array_to_dict(
            bbox_msg, only_top_per_class=self._bbox_json_top_per_class_only
        )
        bbox_json.write_text(
            json.dumps(bbox_payload, indent=2), encoding="utf-8"
        )
        saved.append(bbox_json)

        overlay = image_utils.draw_bbox_overlay(rgb_bgr.copy(), bbox_msg)
        overlay_path = _artifact_path(out, "rgb_bbox2d_tight", ts, "jpg")
        if not image_utils.write_image(
            overlay_path, overlay, self._jpeg_quality
        ):
            raise ValueError(
                f"Failed to write bbox overlay JPEG: {overlay_path}"
            )
        saved.append(overlay_path)


def run(config: dict[str, Any], args=None) -> None:
    rclpy.init(args=args)
    node = EvalCameraCaptureService(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
