"""Shared configuration helpers for Isaac camera sensors.

DEPRECATED: This module has been moved to services/isaac_camera_service/

New location: services/isaac_camera_service/camera_config.py

This file is kept for backward compatibility only. New code should import
from isaac_camera_service instead:

    from isaac_camera_service import (
        load_camera_sensor_config,
        normalize_camera_specs,
        camera_feed_configs,
        validate_camera_contract_alignment,
    )

Camera parameters are defined in embodiment folders:
    assets/embodiments/<embodiment>/camera_sensors.yaml
"""

from __future__ import annotations

import math
import os

import yaml

from fr3_duo_data_contract import DEFAULT_EMBODIMENT_KEY, load_data_contract
from stack_config import REPO_ROOT


DEFAULT_CAMERA_CONFIG_PATH = os.path.join(
    REPO_ROOT,
    "assets",
    "isaac_assets",
    "config",
    "fr3_duo_camera_sensors.yaml",
)


def resolve_camera_config_path(config_path: str | None = None) -> str:
    raw_path = config_path or DEFAULT_CAMERA_CONFIG_PATH
    expanded = os.path.expanduser(str(raw_path))
    if expanded.startswith("/workspace/"):
        expanded = os.path.join(REPO_ROOT, expanded[len("/workspace/") :])
    return os.path.abspath(expanded)


def load_camera_sensor_config(config_path: str | None = None) -> dict:
    resolved_path = resolve_camera_config_path(config_path)
    if not os.path.exists(resolved_path):
        raise FileNotFoundError(f"Camera config file not found: {resolved_path}")
    with open(resolved_path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Camera config root must be a mapping: {resolved_path}")
    loaded["_config_path"] = resolved_path
    return loaded


def _normalized_resolution(camera_config: dict) -> tuple[int, int]:
    render_resolution = camera_config.get("render_resolution", {})
    native_resolution = camera_config.get("native_resolution", {})
    width = int(
        render_resolution.get(
            "width",
            native_resolution.get("width", camera_config.get("width", 640)),
        )
    )
    height = int(
        render_resolution.get(
            "height",
            native_resolution.get("height", camera_config.get("height", 480)),
        )
    )
    return max(width, 1), max(height, 1)


def _normalized_aliases(key: str, camera_config: dict) -> tuple[str, ...]:
    aliases = []
    for value in camera_config.get("aliases", []):
        normalized = str(value or "").strip().lower()
        if normalized:
            aliases.append(normalized)
    for default_alias in (
        str(key).strip().lower(),
        str(camera_config.get("contract_video_key", "")).strip().lower(),
        str(camera_config.get("device_model", "")).strip().lower(),
    ):
        if default_alias:
            aliases.append(default_alias)
    deduped = []
    seen = set()
    for alias in aliases:
        if alias in seen:
            continue
        deduped.append(alias)
        seen.add(alias)
    return tuple(deduped)


def focal_length_mm_from_hfov(sensor_width_mm: float, horizontal_fov_deg: float) -> float:
    fov_rad = math.radians(max(min(float(horizontal_fov_deg), 179.0), 1.0))
    return float(sensor_width_mm) / (2.0 * math.tan(fov_rad / 2.0))


def frame_skip_count_for_rate(*, publish_hz: float, render_hz: float) -> int:
    publish_hz = max(float(publish_hz), 0.1)
    render_hz = max(float(render_hz), publish_hz)
    step_ratio = max(int(round(render_hz / publish_hz)), 1)
    return max(step_ratio - 1, 0)


def normalize_camera_specs(config: dict) -> list[dict]:
    camera_entries = config.get("cameras", {})
    if not isinstance(camera_entries, dict):
        raise ValueError("cameras must be a mapping")

    normalized = []
    for key, camera_config in camera_entries.items():
        if not isinstance(camera_config, dict):
            raise ValueError(f"camera entry must be a mapping: {key}")

        width, height = _normalized_resolution(camera_config)
        sensor_width_mm = float(camera_config.get("sensor_width_mm", 6.0))
        sensor_height_mm = float(
            camera_config.get("sensor_height_mm", sensor_width_mm * (height / width))
        )
        horizontal_fov_deg = float(camera_config.get("horizontal_fov_deg", 69.0))
        vertical_fov_deg = float(camera_config.get("vertical_fov_deg", 42.0))
        focal_length_mm = float(
            camera_config.get(
                "focal_length_mm",
                focal_length_mm_from_hfov(sensor_width_mm, horizontal_fov_deg),
            )
        )
        namespace = str(camera_config.get("namespace", f"/isaac/{key}")).rstrip("/")
        image_topic = str(camera_config.get("image_topic", namespace + "/image_raw"))
        camera_info_topic = str(camera_config.get("camera_info_topic", namespace + "/camera_info"))
        clip_range_m = camera_config.get("clip_range_m", {})
        local_translation = list(camera_config.get("local_translation_m", [0.0, 0.0, 0.0]))
        local_rotation = list(camera_config.get("local_rotation_deg", [-90.0, 0.0, -90.0]))
        if len(local_translation) != 3:
            raise ValueError(f"local_translation_m must contain 3 values: {key}")
        if len(local_rotation) != 3:
            raise ValueError(f"local_rotation_deg must contain 3 values: {key}")

        normalized.append(
            {
                "key": str(key),
                "label": str(camera_config.get("label", key.replace("_", " ").title())),
                "device_model": str(camera_config.get("device_model", "")),
                "contract_video_key": str(camera_config.get("contract_video_key", "")),
                "attachment_frame_name": str(camera_config.get("attachment_frame_name", "")),
                "attachment_frame_path": str(camera_config.get("attachment_frame_path", "")),
                "frame_id": str(camera_config.get("frame_id", "")),
                "camera_prim_name": str(camera_config.get("camera_prim_name", "camera_sensor")),
                "node_namespace": namespace,
                "image_topic": image_topic,
                "camera_info_topic": camera_info_topic,
                "width": int(width),
                "height": int(height),
                "native_width": int(camera_config.get("native_resolution", {}).get("width", width)),
                "native_height": int(camera_config.get("native_resolution", {}).get("height", height)),
                "publish_hz": float(camera_config.get("publish_hz", 10.0)),
                "sensor_width_mm": sensor_width_mm,
                "sensor_height_mm": sensor_height_mm,
                "horizontal_fov_deg": horizontal_fov_deg,
                "vertical_fov_deg": vertical_fov_deg,
                "focal_length_mm": focal_length_mm,
                "focus_distance_m": float(camera_config.get("focus_distance_m", 1.0)),
                "f_stop": float(camera_config.get("f_stop", 2.8)),
                "near_clip_m": float(clip_range_m.get("near", 0.05)),
                "far_clip_m": float(clip_range_m.get("far", 25.0)),
                "local_translation_m": [float(value) for value in local_translation],
                "local_rotation_deg": [float(value) for value in local_rotation],
                "aliases": _normalized_aliases(str(key), camera_config),
                "contract_dtype": str(camera_config.get("contract_dtype", "uint8")),
                "contract_layout": str(camera_config.get("contract_layout", "HWC")),
                "contract_color_space": str(camera_config.get("contract_color_space", "RGB")),
                "parameter_source": dict(camera_config.get("parameter_source", {})),
                "notes": str(camera_config.get("notes", "")),
            }
        )
    return normalized


def camera_feed_configs(config_path: str | None = None) -> list[dict]:
    config = load_camera_sensor_config(config_path)
    feeds = []
    for spec in normalize_camera_specs(config):
        feeds.append(
            {
                "key": spec["key"],
                "label": spec["label"],
                "aliases": spec["aliases"],
                "topic_name": spec["image_topic"],
                "camera_info_topic": spec["camera_info_topic"],
                "contract_video_key": spec["contract_video_key"],
                "contract_dtype": spec["contract_dtype"],
                "contract_layout": spec["contract_layout"],
                "contract_color_space": spec["contract_color_space"],
                "width": spec["width"],
                "height": spec["height"],
            }
        )
    return feeds


def expected_video_keys(
    contract_path: str | None = None,
    *,
    embodiment_key: str = DEFAULT_EMBODIMENT_KEY,
) -> list[str]:
    contract = load_data_contract(contract_path)
    embodiments = contract.get("data_generation", {}).get("embodiments", {})
    embodiment = embodiments.get(embodiment_key, {})
    video = embodiment.get("modalities", {}).get("video", {})
    keys = video.get("keys", [])
    return [str(value) for value in keys if str(value or "").strip()]


def validate_camera_contract_alignment(
    camera_specs: list[dict],
    contract_path: str | None = None,
    *,
    embodiment_key: str = DEFAULT_EMBODIMENT_KEY,
) -> dict:
    expected_keys = expected_video_keys(contract_path, embodiment_key=embodiment_key)
    seen_keys = []
    errors = []
    warnings = []

    for spec in camera_specs:
        contract_video_key = str(spec.get("contract_video_key", "")).strip()
        if not contract_video_key:
            errors.append(f"{spec.get('key')} is missing contract_video_key")
            continue
        seen_keys.append(contract_video_key)
        if contract_video_key not in expected_keys:
            errors.append(
                f"{spec.get('key')} maps to unexpected contract video key {contract_video_key}"
            )
        if str(spec.get("contract_dtype", "")).lower() != "uint8":
            errors.append(f"{spec.get('key')} must expose uint8 video for the contract")
        if str(spec.get("contract_layout", "")).upper() != "HWC":
            errors.append(f"{spec.get('key')} must expose HWC video for the contract")
        if str(spec.get("contract_color_space", "")).upper() != "RGB":
            errors.append(f"{spec.get('key')} must expose RGB video for the contract")
        if not str(spec.get("frame_id", "")).strip():
            errors.append(f"{spec.get('key')} is missing frame_id")
        if not str(spec.get("attachment_frame_name", "")).strip() and not str(
            spec.get("attachment_frame_path", "")
        ).strip():
            errors.append(f"{spec.get('key')} is missing attachment frame information")

    for expected_key in expected_keys:
        if expected_key not in seen_keys:
            errors.append(f"missing camera mapping for contract video key {expected_key}")
    for seen_key in sorted(set(seen_keys)):
        if seen_keys.count(seen_key) > 1:
            errors.append(f"duplicate camera mapping for contract video key {seen_key}")

    if not expected_keys:
        warnings.append("selected contract embodiment does not define any video keys")

    return {
        "compliant": not errors,
        "errors": errors,
        "warnings": warnings,
        "expected_video_keys": expected_keys,
        "configured_video_keys": seen_keys,
    }
