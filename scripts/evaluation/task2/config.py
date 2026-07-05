#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Runtime configuration for the task2 eval camera service.

Defaults live in ``APP_DEFAULTS`` and ``config.yaml``; both can be
overridden on the command line. ``load_runtime_config`` merges (in
increasing priority): APP_DEFAULTS < config.yaml < CLI args.
"""

import argparse
from pathlib import Path
from typing import Any

import yaml

APP_DEFAULTS: dict[str, Any] = {
    "evaluate_service_name": "/isaac/eval_camera/evaluate",
    "image_topic": "/isaac/eval_camera/image_raw",
    "depth_topic": "/isaac/eval_camera/depth",
    "semantic_segmentation_topic": "/isaac/eval_camera/semantic_segmentation",
    "semantic_labels_topic": "/isaac/eval_camera/semantic_labels",
    "bbox_2d_tight_topic": "/isaac/eval_camera/bbox_2d_tight",
    "camera_info_topic": "/isaac/eval_camera/camera_info",
    "thermalpad_label": "thermalpad",
    "liner_label": "liner",
    "target_label": "target",
    # Default output dir is the persistent volume mounted into the container.
    "output_dir": "/output",
    "jpeg_quality": 95,
    "bbox_json_top_per_class_only": False,
}

# Raw int32 semantic-mask pixel value -> class name, for the current
# task2 scene. It is only used to resolve the both-pads-visible case via
# pixel ratios; an incorrect mapping silently breaks the
# both_liner_dominant / both_thermalpad_dominant / sideways decision.
SEMANTIC_RAW_ID_NAME_HINTS: dict[int, str] = {
    1: "unlabeled",
    2: "board",
    3: "thermalpad",
    4: "target",
    5: "liner",
}


def coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "on"}:
            return True
        if text in {"0", "false", "no", "off"}:
            return False
    raise ValueError(f"Cannot parse boolean value: {value}")


def _default_config_path() -> Path:
    return Path(__file__).with_name("config.yaml")


def _load_yaml_config(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        return {}
    with config_path.open("r", encoding="utf-8") as file_obj:
        loaded = yaml.safe_load(file_obj) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Config file must be a YAML mapping: {config_path}")
    # Support an optional nested 'eval_task2' section.
    if "eval_task2" in loaded:
        nested = loaded["eval_task2"]
        if not isinstance(nested, dict):
            raise ValueError(
                "'eval_task2' section must be a mapping in config YAML"
            )
        return dict(nested)
    return dict(loaded)


def _build_arg_parser(defaults: dict[str, Any]) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Eval task2 camera capture and IoU evaluation service"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(_default_config_path()),
        help=(
            "Path to YAML config file (default: this directory's config.yaml)"
        ),
    )
    parser.add_argument(
        "--image-topic", type=str, default=str(defaults["image_topic"])
    )
    parser.add_argument(
        "--depth-topic", type=str, default=str(defaults["depth_topic"])
    )
    parser.add_argument(
        "--semantic-segmentation-topic",
        type=str,
        default=str(defaults["semantic_segmentation_topic"]),
    )
    parser.add_argument(
        "--semantic-labels-topic",
        type=str,
        default=str(defaults["semantic_labels_topic"]),
    )
    parser.add_argument(
        "--bbox-2d-tight-topic",
        type=str,
        default=str(defaults["bbox_2d_tight_topic"]),
    )
    parser.add_argument(
        "--camera-info-topic",
        type=str,
        default=str(defaults["camera_info_topic"]),
    )
    parser.add_argument(
        "--evaluate-service-name",
        type=str,
        default=str(defaults["evaluate_service_name"]),
    )
    parser.add_argument(
        "--output-dir", type=str, default=str(defaults["output_dir"])
    )
    parser.add_argument(
        "--jpeg-quality", type=int, default=int(defaults["jpeg_quality"])
    )
    parser.add_argument(
        "--thermalpad-label",
        type=str,
        default=str(defaults["thermalpad_label"]),
    )
    parser.add_argument(
        "--liner-label", type=str, default=str(defaults["liner_label"])
    )
    parser.add_argument(
        "--target-label", type=str, default=str(defaults["target_label"])
    )
    parser.add_argument(
        "--bbox-json-top-per-class-only",
        type=coerce_bool,
        default=coerce_bool(defaults["bbox_json_top_per_class_only"]),
    )
    return parser


def load_runtime_config(args=None) -> dict[str, Any]:
    # First pass: discover the config path so YAML can feed argparse defaults.
    bootstrap_parser = argparse.ArgumentParser(add_help=False)
    bootstrap_parser.add_argument(
        "--config", type=str, default=str(_default_config_path())
    )
    bootstrap_args, _ = bootstrap_parser.parse_known_args(args=args)

    config_path = Path(bootstrap_args.config)
    yaml_defaults = _load_yaml_config(config_path)
    merged_defaults = dict(APP_DEFAULTS)
    merged_defaults.update(yaml_defaults)

    parser = _build_arg_parser(merged_defaults)
    parsed = parser.parse_args(args=args)

    return {
        "image_topic": parsed.image_topic,
        "depth_topic": parsed.depth_topic,
        "semantic_segmentation_topic": parsed.semantic_segmentation_topic,
        "semantic_labels_topic": parsed.semantic_labels_topic,
        "bbox_2d_tight_topic": parsed.bbox_2d_tight_topic,
        "camera_info_topic": parsed.camera_info_topic,
        "evaluate_service_name": parsed.evaluate_service_name,
        "output_dir": parsed.output_dir,
        "jpeg_quality": int(parsed.jpeg_quality),
        "thermalpad_label": parsed.thermalpad_label,
        "liner_label": parsed.liner_label,
        "target_label": parsed.target_label,
        "bbox_json_top_per_class_only": coerce_bool(
            parsed.bbox_json_top_per_class_only
        ),
    }
