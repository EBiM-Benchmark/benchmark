# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 publishers for scene-level (non-robot) cameras.

Builds cameras described in a per-scene yaml (config/cameras_<scene>.yaml):
creates the Camera prim when the scene has not authored one, applies the
pose from the yaml (the yaml wins over the USD), and builds the same
RenderProduct -> ROS2CameraHelper OmniGraph as the robot cameras via
camera_publishers._setup_single_camera_graph — unless the scene already
built /ROS2_CameraGraphs/<name> (the room scene's eval_camera), which is
then left untouched.
"""

from __future__ import annotations

import sys
from pathlib import Path

from pxr import Gf, UsdGeom

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML ships with Isaac Sim
    yaml = None

from topics import load_topics

from .camera_publishers import CAMERA_GRAPH_ROOT, _setup_single_camera_graph

_REQUIRED_FIELDS = (
    "prim_path",
    "namespace",
    "frame_id",
    "translation",
    "rotation_xyz_deg",
    "render_resolution",
)


def load_scene_camera_configs(config_path: Path) -> dict:
    """Camera entries from a scene camera yaml."""
    if yaml is None:
        raise RuntimeError("PyYAML is required to read scene camera configs")
    config_path = Path(config_path)
    if not config_path.is_file():
        raise FileNotFoundError(
            f"Scene camera config not found: {config_path}"
        )
    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cameras = data.get("cameras") or {}
    if not cameras:
        raise RuntimeError(f"{config_path} defines no cameras")
    problems = []
    for camera_key, config in cameras.items():
        for field in _REQUIRED_FIELDS:
            if not config.get(field):
                problems.append(f"{camera_key}: missing '{field}'")
        resolution = config.get("render_resolution") or {}
        if isinstance(resolution, dict) and not (
            resolution.get("width") and resolution.get("height")
        ):
            problems.append(
                f"{camera_key}: render_resolution needs width and height"
            )
    if problems:
        raise RuntimeError(
            f"Invalid camera entries in {config_path}: " + "; ".join(problems)
        )
    return cameras


def _check_contract(camera_key: str, config: dict) -> None:
    """Validate a camera against config/topics.yaml when it declares a
    contract key; cameras without one publish but are not recorded."""
    contract_key = config.get("contract")
    if contract_key is None:
        print(
            f"Warning: scene camera '{camera_key}' has no contract entry in "
            "config/topics.yaml; it will publish but the recorder will not "
            "record it",
            file=sys.stderr,
        )
        return
    contract = load_topics()["cameras"].get(contract_key)
    if not isinstance(contract, dict) or "namespace" not in contract:
        raise RuntimeError(
            f"{camera_key}: unknown contract '{contract_key}' "
            "(no such cameras entry in config/topics.yaml)"
        )
    mismatches = []
    namespace = str(config["namespace"])
    if namespace != contract["namespace"]:
        mismatches.append(
            f"{camera_key}: namespace {namespace!r} != contract "
            f"{contract['namespace']!r}"
        )
    resolution = config["render_resolution"]
    height, width = int(contract["shape"][0]), int(contract["shape"][1])
    if (int(resolution["height"]), int(resolution["width"])) != (
        height,
        width,
    ):
        mismatches.append(
            f"{camera_key}: render_resolution "
            f"{resolution['width']}x{resolution['height']} != contract "
            f"shape {width}x{height}"
        )
    if mismatches:
        raise RuntimeError(
            "Scene camera config disagrees with config/topics.yaml: "
            + "; ".join(mismatches)
        )


def _ensure_camera_prim(stage, camera_key: str, config: dict) -> None:
    """Create the Camera prim if the scene has not authored one, then apply
    the configured pose (the yaml pose wins over the USD)."""
    prim_path = str(config["prim_path"])
    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        if not prim.IsA(UsdGeom.Camera):
            raise RuntimeError(
                f"{camera_key}: prim {prim_path} exists but is not a Camera "
                f"({prim.GetTypeName()})"
            )
        camera = UsdGeom.Camera(prim)
        print(
            f"Recording: adopting existing camera prim {prim_path} "
            f"for '{camera_key}'",
            flush=True,
        )
    else:
        camera = UsdGeom.Camera.Define(stage, prim_path)
        camera.GetFocalLengthAttr().Set(float(config.get("focal_length", 20)))
        camera.GetFocusDistanceAttr().Set(
            float(config.get("focus_distance", 400))
        )
        camera.GetProjectionAttr().Set(
            str(config.get("projection", "perspective"))
        )
    xform_api = UsdGeom.XformCommonAPI(camera)
    xform_api.SetTranslate(Gf.Vec3d(*(float(v) for v in config["translation"])))
    xform_api.SetRotate(
        Gf.Vec3f(*(float(v) for v in config["rotation_xyz_deg"])),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )


def setup_scene_camera_graphs(stage, config_path: Path) -> dict[str, str]:
    """Create prim + camera graph per configured scene camera.

    Returns {camera_key: camera_prim_path} for the graphs this call built
    (cameras whose graph the scene already built are excluded).
    """
    configs = load_scene_camera_configs(config_path)
    built: dict[str, str] = {}
    for camera_key, config in configs.items():
        _check_contract(camera_key, config)
        _ensure_camera_prim(stage, camera_key, config)
        graph_path = f"{CAMERA_GRAPH_ROOT}/{camera_key}"
        if stage.GetPrimAtPath(graph_path).IsValid():
            print(
                f"Recording: camera graph {graph_path} already exists — "
                "skipping (built by the scene)",
                flush=True,
            )
            continue
        _setup_single_camera_graph(
            camera_key,
            str(config["prim_path"]),
            config,
            publish_depth=bool(config.get("publish_depth", False)),
            publish_semantic=bool(config.get("publish_semantic", False)),
            publish_bbox=bool(config.get("publish_bbox", False)),
        )
        built[camera_key] = str(config["prim_path"])
    return built


def print_setup_failure(exc: Exception) -> None:
    print(
        f"Warning: scene camera publishers unavailable: {exc}",
        file=sys.stderr,
    )
