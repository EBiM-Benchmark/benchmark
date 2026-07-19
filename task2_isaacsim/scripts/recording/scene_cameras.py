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

from topics import load_topics

from pxr import Gf, UsdGeom

from .camera_publishers import (
    CAMERA_GRAPH_ROOT,
    _setup_single_camera_graph,
    contract_mismatches,
    load_camera_configs,
)

_REQUIRED_FIELDS = (
    "prim_path",
    "namespace",
    "frame_id",
    "translation",
    "rotation_xyz_deg",
    "render_resolution",
)


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
    mismatches = contract_mismatches(camera_key, config, contract)
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
    xform_api.SetTranslate(
        Gf.Vec3d(*(float(v) for v in config["translation"]))
    )
    xform_api.SetRotate(
        Gf.Vec3f(*(float(v) for v in config["rotation_xyz_deg"])),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )


def setup_scene_camera_graphs(stage, config_path: Path) -> dict[str, str]:
    """Create prim + camera graph per configured scene camera.

    Returns {camera_key: camera_prim_path} for the graphs this call built
    (cameras whose graph the scene already built are excluded).
    """
    configs = load_camera_configs(
        config_path,
        required_fields=_REQUIRED_FIELDS,
        what="scene camera config",
    )
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
