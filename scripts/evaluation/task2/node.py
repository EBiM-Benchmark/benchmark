#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""ROS2 node for the task2 eval camera service.

Subscribes to the Isaac Sim eval-camera topics, and on each
``Trigger`` call selects a stamp-coherent set of the buffered
per-stream messages (via ``stream_sync.EvalStreamSync``), saves all
modalities, and computes pad-vs-target IoU. Heavy lifting is
delegated to ``image_utils`` (conversions), ``stream_sync``
(coherent-set selection), and ``evaluation`` (IoU + orientation),
keeping this module focused on ROS plumbing and IO.
"""

import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Any

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

try:
    from vision_msgs.msg import Detection2DArray
except Exception:  # pragma: no cover - depends on runtime image
    Detection2DArray = None

import image_utils
from config import SEMANTIC_RAW_ID_NAME_HINTS, coerce_bool
from coverage_metrics import (
    coverage_metrics,
    parse_object_poses_payload,
    parse_pad_points_payload,
)
from evaluation import (
    evaluate_thermalpad_target_iou,
    hints_from_label_payload,
)
from stream_sync import (
    ALL_STREAMS,
    STREAM_BBOX_LOOSE,
    STREAM_BBOX_LOOSE_LABELS,
    STREAM_BBOX_TIGHT,
    STREAM_BBOX_TIGHT_LABELS,
    STREAM_CAMERA_INFO,
    STREAM_DEPTH,
    STREAM_IMAGE,
    STREAM_SEMANTIC,
    STREAM_SEMANTIC_LABELS,
    EvalStreamSync,
    parse_label_payload_ok,
    parse_label_stamp,
    stamp_to_seconds,
)

# The three streams EvalStreamSync always anchors selection on; reused
# below to put required streams first in the sync-failure message.
_REQUIRED_CORE_STREAMS = (STREAM_IMAGE, STREAM_SEMANTIC, STREAM_BBOX_TIGHT)
_STREAM_STATUS_ORDER = _REQUIRED_CORE_STREAMS + tuple(
    stream for stream in ALL_STREAMS if stream not in _REQUIRED_CORE_STREAMS
)


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


def _stream_status_text(stream: str, info: dict) -> str:
    # A stream that has never received anything has nothing else worth
    # reporting. One that has received messages but never produced a
    # usable stamp also reports NEVER, but keeps its buffer depth.
    if not info["ever_received"]:
        return f"{stream} last=NEVER"
    newest_stamp = info["newest_stamp"]
    last = f"{newest_stamp:.2f}" if newest_stamp is not None else "NEVER"
    return f"{stream} last={last} buffered={info['buffered']}"


@dataclass
class _Snapshot:
    """Per-modality messages selected from one coherent SyncSelection."""

    image: Any = None
    depth: Any = None
    semantic: Any = None
    labels: Any = None
    bbox_tight_labels: Any = None
    bbox_loose_labels: Any = None
    bbox_loose: Any = None
    bbox: Any = None
    camera_info: Any = None


class EvalCameraCaptureService(Node):
    """ROS2 node that selects synchronized eval-camera streams and
    evaluates pad placement."""

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
        self._sync_tolerance_s = float(config["sync_tolerance_s"])
        self._sync_timeout_s = float(config["sync_timeout_s"])
        self._sync_max_age_s = float(config["sync_max_age_s"])
        self._sync_rebase_epsilon_s = float(config["sync_rebase_epsilon_s"])
        self._evaluator_version = str(config.get("evaluator_version", ""))

        # Best-effort ground-truth physical coverage/spill audit (see
        # coverage_metrics.py). GT sim_time is the bridge main-loop
        # clock sampled off the render tick -- an independent notion
        # of "now" from the render-derived header.stamp values
        # EvalStreamSync anchors on -- so the two latest-parsed GT
        # payloads are cached under their own small lock rather than
        # folded into stream sync.
        self._audit_enabled = coerce_bool(config["audit_enabled"])
        self._audit_object_poses_topic = str(
            config["audit_object_poses_topic"]
        )
        self._audit_pad_points_topic = str(config["audit_pad_points_topic"])
        self._audit_max_skew_s = float(config["audit_max_skew_s"])
        self._audit_lock = Lock()
        self._audit_latest_object_poses = (None, None)
        self._audit_latest_pad_points = (None, None)

        # Subscriptions and the service run in separate
        # MutuallyExclusiveCallbackGroups on a MultiThreadedExecutor
        # (see run()) -- the service handler can then block in
        # wait_for_selection() while subscription callbacks keep
        # draining incoming messages on another thread.
        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._srv_group = MutuallyExclusiveCallbackGroup()
        self._sync = EvalStreamSync(
            required_core=_REQUIRED_CORE_STREAMS,
            tolerance_s=self._sync_tolerance_s,
            max_age_s=self._sync_max_age_s,
            rebase_epsilon_s=self._sync_rebase_epsilon_s,
            image_buffer_len=int(config["sync_image_buffer_len"]),
            buffer_len=int(config["sync_buffer_len"]),
        )

        # Isaac Sim 5.1's bridge publishes RELIABLE by default;
        # qos_profile_sensor_data (BEST_EFFORT) remains a compatible
        # subscriber, and it also tolerates older bridges that still
        # publish BEST_EFFORT.
        self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            CameraInfo,
            str(config["camera_info_topic"]),
            self._on_camera_info,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Image,
            str(config["depth_topic"]),
            self._on_depth,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Image,
            str(config["semantic_segmentation_topic"]),
            self._on_semantic_segmentation,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            String,
            str(config["semantic_labels_topic"]),
            self._on_semantic_labels,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        # The bbox annotator's own id->label map; its ID scheme differs
        # from the raw mask IDs carried by semantic_labels.
        self.create_subscription(
            String,
            str(config["bbox_tight_labels_topic"]),
            self._on_bbox_tight_labels,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        # The loose annotator's own id->label map; its ID scheme differs
        # from both the tight map and the raw mask IDs.
        self.create_subscription(
            String,
            str(config["bbox_loose_labels_topic"]),
            self._on_bbox_loose_labels,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        if Detection2DArray is not None:
            self.create_subscription(
                Detection2DArray,
                str(config["bbox_2d_tight_topic"]),
                self._on_bbox_2d_tight,
                qos_profile_sensor_data,
                callback_group=self._sub_group,
            )
            self.create_subscription(
                Detection2DArray,
                str(config["bbox_2d_loose_topic"]),
                self._on_bbox_2d_loose,
                qos_profile_sensor_data,
                callback_group=self._sub_group,
            )
        else:
            self.get_logger().warn(
                "vision_msgs is not available; bbox overlays/evaluation "
                "are disabled. "
                "Install ros-jazzy-vision-msgs in the runtime image."
            )

        # Ground-truth audit topics exist only when the Isaac Sim scene
        # runs with --record; publishers are RELIABLE default (not
        # BEST_EFFORT), so this uses default QoS depth 10, not
        # qos_profile_sensor_data.
        if self._audit_enabled:
            self.create_subscription(
                String,
                self._audit_object_poses_topic,
                self._on_audit_object_poses,
                10,
                callback_group=self._sub_group,
            )
            self.create_subscription(
                Float32MultiArray,
                self._audit_pad_points_topic,
                self._on_audit_pad_points,
                10,
                callback_group=self._sub_group,
            )

        self.create_service(
            Trigger,
            str(config["evaluate_service_name"]),
            self._on_save_request,
            callback_group=self._srv_group,
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
        self._sync.observe(
            STREAM_IMAGE,
            msg,
            stamp=stamp_to_seconds(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            ),
        )

    def _on_camera_info(self, msg):
        self._sync.observe(
            STREAM_CAMERA_INFO,
            msg,
            stamp=stamp_to_seconds(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            ),
        )

    def _on_depth(self, msg):
        self._sync.observe(
            STREAM_DEPTH,
            msg,
            stamp=stamp_to_seconds(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            ),
        )

    def _on_semantic_segmentation(self, msg):
        self._sync.observe(
            STREAM_SEMANTIC,
            msg,
            stamp=stamp_to_seconds(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            ),
        )

    def _on_semantic_labels(self, msg):
        self._sync.observe(
            STREAM_SEMANTIC_LABELS,
            msg,
            stamp=parse_label_stamp(msg.data),
            parsed_ok=parse_label_payload_ok(msg.data),
        )

    def _on_bbox_tight_labels(self, msg):
        self._sync.observe(
            STREAM_BBOX_TIGHT_LABELS,
            msg,
            stamp=parse_label_stamp(msg.data),
            parsed_ok=parse_label_payload_ok(msg.data),
        )

    def _on_bbox_loose_labels(self, msg):
        self._sync.observe(
            STREAM_BBOX_LOOSE_LABELS,
            msg,
            stamp=parse_label_stamp(msg.data),
            parsed_ok=parse_label_payload_ok(msg.data),
        )

    def _on_bbox_2d_tight(self, msg):
        self._sync.observe(
            STREAM_BBOX_TIGHT,
            msg,
            stamp=stamp_to_seconds(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            ),
        )

    def _on_bbox_2d_loose(self, msg):
        self._sync.observe(
            STREAM_BBOX_LOOSE,
            msg,
            stamp=stamp_to_seconds(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            ),
        )

    def _on_audit_object_poses(self, msg):
        # Not part of stream sync -- see the audit-state comment in
        # __init__. Every message overwrites the cached latest, parse
        # failure included: a garbled single message degrades the
        # audit for one evaluation rather than serving stale data.
        parsed = parse_object_poses_payload(msg.data)
        with self._audit_lock:
            self._audit_latest_object_poses = parsed

    def _on_audit_pad_points(self, msg):
        parsed = parse_pad_points_payload(msg.data)
        with self._audit_lock:
            self._audit_latest_pad_points = parsed

    # ------------------------------------------------------------------ #
    # Service handler
    # ------------------------------------------------------------------ #
    def _on_save_request(self, _request, response):
        selection = self._sync.wait_for_selection(self._sync_timeout_s)
        if selection is None:
            return self._on_sync_failure(response)

        snap = _Snapshot(
            image=selection.items[STREAM_IMAGE],
            depth=selection.items[STREAM_DEPTH],
            semantic=selection.items[STREAM_SEMANTIC],
            labels=selection.items[STREAM_SEMANTIC_LABELS],
            bbox_tight_labels=selection.items[STREAM_BBOX_TIGHT_LABELS],
            bbox_loose_labels=selection.items[STREAM_BBOX_LOOSE_LABELS],
            bbox_loose=selection.items[STREAM_BBOX_LOOSE],
            bbox=selection.items[STREAM_BBOX_TIGHT],
            camera_info=selection.items[STREAM_CAMERA_INFO],
        )

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
            self._save_labels(
                out, ts, "semantic_labels", snap.labels, saved, missing
            )
            self._save_labels(
                out,
                ts,
                "bbox_tight_labels",
                snap.bbox_tight_labels,
                saved,
                missing,
            )
            self._save_labels(
                out,
                ts,
                "bbox_loose_labels",
                snap.bbox_loose_labels,
                saved,
                missing,
            )
            # Modality snapshots above are written regardless; only
            # evaluation requires a label map and the bbox stream.
            if (
                snap.labels is None and snap.bbox_tight_labels is None
            ) or snap.bbox is None:
                raise ValueError(
                    "Evaluation requires bbox_2d_tight and a label map "
                    "(bbox_2d_tight_labels or semantic_labels)"
                )
            eval_result = self._save_eval(
                out, ts, snap, label_array, selection, saved
            )
            self._save_bbox_artifacts(
                out, ts, "bbox2d_tight", snap.bbox, rgb_bgr, saved
            )
            if snap.bbox_loose is not None:
                self._save_bbox_artifacts(
                    out, ts, "bbox2d_loose", snap.bbox_loose, rgb_bgr, saved
                )
            else:
                missing.append("bbox_2d_loose")
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
        semantic_table_source = eval_result["label_provenance"][
            "semantic_table_source"
        ]
        labels_display = (
            "dynamic" if semantic_table_source == "dynamic" else "static"
        )
        eval_msg = (
            f" eval_iou={eval_result['iou_thermalpad_vs_target_current']:.4f}"
            f" orientation={'correct' if is_correct else 'wrong'}"
            f"[{eval_result['orientation_case']}]"
            f" sync={selection.status}"
            f" anchor={selection.anchor_stamp:.4f}"
            f" max_delta={selection.max_stamp_delta:.4f}"
            f" tol={self._sync_tolerance_s:.4f}"
            f" labels={labels_display}"
            f" version={self._evaluator_version}"
        )

        summary = ", ".join(str(p) for p in saved)
        response.success = True
        response.message = f"Saved [{summary}]{info}{eval_msg}"
        self.get_logger().info(response.message)
        return response

    def _on_sync_failure(self, response):
        report = self._sync.stream_report()
        now = datetime.now()
        ts = now.strftime("%Y%m%d_%H%M%S_%f")
        payload = {
            "wall_time": now.isoformat(),
            "sync_config": {
                "tolerance_s": self._sync_tolerance_s,
                "timeout_s": self._sync_timeout_s,
                "max_age_s": self._sync_max_age_s,
                "rebase_epsilon_s": self._sync_rebase_epsilon_s,
            },
            "streams": report,
        }
        failure_path = _artifact_path(
            self._evaluate_output_dir, "sync_failure", ts, "json"
        )
        try:
            failure_path.write_text(
                json.dumps(payload, indent=2), encoding="utf-8"
            )
            artifact_note = f"See {failure_path}"
        except OSError as exc:
            # Graceful degrade: a disk-full/permission/missing-dir
            # failure writing the diagnostic artifact must not
            # prevent a controlled (success=False) response -- that
            # would defeat the point of this whole failure path.
            artifact_note = (
                f"Failed to write diagnostic artifact {failure_path}: {exc}"
            )

        stream_texts = (
            _stream_status_text(stream, report[stream])
            for stream in _STREAM_STATUS_ORDER
        )
        response.success = False
        response.message = (
            f"Sync failed after {self._sync_timeout_s:.1f}s "
            f"(tolerance {self._sync_tolerance_s:.3f}s): "
            f"{'; '.join(stream_texts)}. {artifact_note}"
        )
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
        out, ts, name: str, labels_msg, saved: list[Path], missing: list[str]
    ) -> None:
        if labels_msg is None:
            missing.append(name)
            return
        labels_path = _artifact_path(out, name, ts, "txt")
        labels_path.write_text(labels_msg.data, encoding="utf-8")
        saved.append(labels_path)

    def _save_eval(
        self,
        out,
        ts,
        snap: _Snapshot,
        label_array,
        selection,
        saved: list[Path],
    ) -> dict[str, Any]:
        # bbox class_ids resolve through the bbox annotator's own label
        # map; fall back to the legacy shared topic when the scene
        # predates the split (racy, but no worse than before).
        bbox_label_map = (
            snap.bbox_tight_labels
            if snap.bbox_tight_labels is not None
            else snap.labels
        )
        # The raw mask IDs are assigned per session; derive them from
        # the selected segmentation label map. hints must never end
        # up None here -- evaluate_thermalpad_target_iou requires
        # semantic_hints to be a dict -- so an absent or unparsable
        # live payload always falls back to the static hints; there
        # is no "trust the live table is just late" grace window.
        hints = None
        if snap.labels is not None:
            hints = hints_from_label_payload(snap.labels.data)
        semantic_table_stamp = selection.stamps[STREAM_SEMANTIC_LABELS]
        if hints is not None:
            semantic_source = "dynamic"
            semantic_table_binding = (
                "stamp"
                if semantic_table_stamp is not None
                else "arrival_order"
            )
        else:
            hints = SEMANTIC_RAW_ID_NAME_HINTS
            semantic_source = "static_hints"
            semantic_table_binding = "static"
        # The target resolves through the loose annotator's own stream and
        # map when the scene publishes them; otherwise evaluate() falls
        # back to the tight stream (pre-loose scene).
        target_bbox_msg = snap.bbox_loose
        target_labels_payload = (
            snap.bbox_loose_labels.data
            if snap.bbox_loose_labels is not None
            else None
        )

        label_provenance = {
            "semantic_table_source": semantic_source,
            "semantic_table_binding": semantic_table_binding,
            "semantic_table_stamp": semantic_table_stamp,
            "tight_table_stamp": selection.stamps[STREAM_BBOX_TIGHT_LABELS],
            "loose_table_stamp": selection.stamps[STREAM_BBOX_LOOSE_LABELS],
            "bbox_table_source": "tight"
            if snap.bbox_tight_labels is not None
            else "legacy_semantic_shared",
        }

        try:
            eval_result = evaluate_thermalpad_target_iou(
                snap.bbox,
                bbox_label_map.data,
                thermalpad_label=self._thermalpad_label,
                liner_label=self._liner_label,
                target_label=self._target_label,
                semantic_hints=hints,
                label_array=label_array,
                current_frame_stamp=_stamp_to_string(snap.semantic)
                if snap.semantic is not None
                else "",
                bbox_frame_stamp=_stamp_to_string(snap.bbox),
                target_bbox_msg=target_bbox_msg,
                target_labels_payload=target_labels_payload,
                stream_stamps=selection.stamps,
                sync_status=selection.status,
                sync_tolerance_s=self._sync_tolerance_s,
                sync_anchor_stamp=selection.anchor_stamp,
                max_stamp_delta=selection.max_stamp_delta,
                label_provenance=label_provenance,
                evaluator_version=self._evaluator_version,
            )
        except ValueError as exc:
            raise ValueError(f"Evaluation failed: {exc}") from exc
        # Best-effort ground-truth audit: never affects eval_result's
        # official fields, the response, or the failure path above (it
        # only runs once evaluation has already succeeded).
        self._apply_physical_audit(eval_result)
        eval_path = _artifact_path(out, "iou", ts, "json")
        eval_path.write_text(
            json.dumps(eval_result, indent=2), encoding="utf-8"
        )
        saved.append(eval_path)
        return eval_result

    def _apply_physical_audit(self, eval_result: dict[str, Any]) -> None:
        """Fill in ``eval_result["physical_audit"]`` (or a reason why not).

        Purely additive and best-effort: wraps the whole computation
        in a broad try/except so an unexpected failure here can never
        propagate into ``_save_eval``/``_on_save_request`` and affect
        the official fields, the response, or the failure path --
        it only ever mutates ``eval_result`` in place, adding
        ``physical_audit`` (a ``coverage_metrics()`` dict plus the two
        GT sim_time stamps used, on success) and, when unavailable,
        ``physical_audit_unavailable_reason`` (one of
        ``audit_disabled``, ``no_object_poses``, ``no_pad_points``,
        ``board_target_missing``, ``stale_skew``, ``audit_error``).
        """
        try:
            if not self._audit_enabled:
                eval_result["physical_audit"] = None
                eval_result["physical_audit_unavailable_reason"] = (
                    "audit_disabled"
                )
                return

            with self._audit_lock:
                t_poses, poses = self._audit_latest_object_poses
                t_pad, pad_points = self._audit_latest_pad_points

            if t_poses is None or poses is None:
                eval_result["physical_audit"] = None
                eval_result["physical_audit_unavailable_reason"] = (
                    "no_object_poses"
                )
                return
            if t_pad is None or pad_points is None:
                eval_result["physical_audit"] = None
                eval_result["physical_audit_unavailable_reason"] = (
                    "no_pad_points"
                )
                return
            if "board_target" not in poses:
                eval_result["physical_audit"] = None
                eval_result["physical_audit_unavailable_reason"] = (
                    "board_target_missing"
                )
                return
            if abs(t_poses - t_pad) > self._audit_max_skew_s:
                eval_result["physical_audit"] = None
                eval_result["physical_audit_unavailable_reason"] = "stale_skew"
                return

            metrics = coverage_metrics(pad_points, poses["board_target"])
            eval_result["physical_audit"] = {
                **metrics,
                "object_poses_sim_time": t_poses,
                "pad_points_sim_time": t_pad,
            }
        except Exception as exc:  # noqa: BLE001 - audit is best-effort only
            eval_result["physical_audit"] = None
            eval_result["physical_audit_unavailable_reason"] = "audit_error"
            self.get_logger().warn(f"Physical audit failed: {exc}")

    def _save_bbox_artifacts(
        self, out, ts, name: str, bbox_msg, rgb_bgr, saved: list[Path]
    ) -> None:
        bbox_json = _artifact_path(out, name, ts, "json")
        bbox_payload = image_utils.bbox_2d_array_to_dict(
            bbox_msg, only_top_per_class=self._bbox_json_top_per_class_only
        )
        bbox_json.write_text(
            json.dumps(bbox_payload, indent=2), encoding="utf-8"
        )
        saved.append(bbox_json)

        overlay = image_utils.draw_bbox_overlay(rgb_bgr.copy(), bbox_msg)
        overlay_path = _artifact_path(out, f"rgb_{name}", ts, "jpg")
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
