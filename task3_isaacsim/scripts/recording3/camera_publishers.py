# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS 2 publishers for the robot cameras.

Builds one RenderProduct -> ROS2CameraHelper OmniGraph per camera listed in
the embodiment's camera_sensors.yaml. The sim clock publisher lives on the
bridge node (isaacsim_fr3duo_teleop_bridge_core.py, world.current_time) — an
OmniGraph IsaacReadSimulationTime clock produces jumping values after a scene
reset (world.stop clears its time samples), so it must not be used.

The robot USD already authors the Camera prims with the correct optics, so
the graphs attach render products to those existing prims; the render
resolution, ROS topic layout, and the prim_path_tokens used to locate each
prim all come from camera_sensors.yaml. Scene-level (non-robot) cameras are
handled by scene_cameras.py, which creates prims from config and reuses this
module's graph builder.
"""

from __future__ import annotations

import sys
from pathlib import Path

from pxr import Usd, UsdGeom

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML ships with Isaac Sim
    yaml = None

from topics import load_topics

CAMERA_GRAPH_ROOT = "/ROS2_CameraGraphs"

# Fields each camera_sensors.yaml entry must provide for the graph builder.
_REQUIRED_SENSOR_FIELDS = (
    "namespace",
    "frame_id",
    "render_resolution",
    "prim_path_tokens",
)


def load_camera_configs(
    path: Path,
    *,
    required_fields=_REQUIRED_SENSOR_FIELDS,
    what: str = "camera_sensors.yaml",
) -> dict:
    """Validated camera entries from a camera yaml (robot or scene)."""
    if yaml is None:
        raise RuntimeError(f"PyYAML is required to read {what}")
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"{what} not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cameras = data.get("cameras") or {}
    if not cameras:
        raise RuntimeError(f"{path} defines no cameras")
    problems = []
    for camera_key, config in cameras.items():
        for field in required_fields:
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
            f"Invalid camera entries in {path}: " + "; ".join(problems)
        )
    return cameras


def contract_mismatches(
    camera_key: str, config: dict, contract: dict
) -> list[str]:
    """Namespace/resolution mismatches between a camera entry and its
    config/topics.yaml contract."""
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
    return mismatches


def _check_topic_contract(configs: dict) -> None:
    """Fail loudly when camera_sensors.yaml drifts from config/topics.yaml.

    The recorder builds its subscriptions purely from the topic contract, so
    a namespace or resolution mismatch would otherwise surface as silently
    missing frames instead of a startup error.
    """
    contract = load_topics()["cameras"]["robot"]
    mismatches = []
    for recorder_key, entry in contract.items():
        config = configs.get(entry["sensors_key"])
        if config is None:
            mismatches.append(
                f"{recorder_key}: camera_sensors.yaml has no entry "
                f"'{entry['sensors_key']}'"
            )
            continue
        mismatches.extend(contract_mismatches(recorder_key, config, entry))
    if mismatches:
        raise RuntimeError(
            "camera_sensors.yaml disagrees with config/topics.yaml: "
            + "; ".join(mismatches)
        )
    contracted = {entry["sensors_key"] for entry in contract.values()}
    for camera_key in sorted(set(configs) - contracted):
        print(
            f"Warning: camera '{camera_key}' has no cameras.robot entry in "
            "config/topics.yaml; it will publish but the recorder will not "
            "record it",
            file=sys.stderr,
        )


def find_robot_camera_prims(
    stage, robot_prim_path: str, configs: dict
) -> dict[str, str]:
    """Locate the authored Camera prims under the robot for each camera key.

    Fails loudly with the list of Camera prims found so a robot USD change
    surfaces immediately instead of recording black frames.
    """
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {robot_prim_path}")

    camera_paths = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(robot_prim)
        if prim.IsA(UsdGeom.Camera)
    ]
    resolved: dict[str, str] = {}
    for camera_key, config in configs.items():
        tokens = tuple(
            str(token).lower() for token in config["prim_path_tokens"]
        )
        matches = [
            path
            for path in camera_paths
            if all(token in path.lower() for token in tokens)
        ]
        if len(matches) == 1:
            resolved[camera_key] = matches[0]
        else:
            raise RuntimeError(
                f"Expected exactly one Camera prim for '{camera_key}' "
                f"(path tokens {tokens}), found {matches or 'none'}. "
                f"Camera prims under {robot_prim_path}: {camera_paths}"
            )
    return resolved


def _setup_single_camera_graph(
    camera_key: str,
    camera_prim_path: str,
    config: dict,
    *,
    publish_depth: bool,
    publish_semantic: bool = False,
    publish_bbox: bool = False,
) -> None:
    import omni.graph.core as og

    graph_path = f"{CAMERA_GRAPH_ROOT}/{camera_key}"
    namespace = str(config["namespace"])
    frame_id = str(config["frame_id"])
    render_resolution = config["render_resolution"]
    width = int(render_resolution["width"])
    height = int(render_resolution["height"])
    subtopics = load_topics()["cameras"]["subtopics"]

    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ]
    connections = [
        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
        ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
        (
            "RenderProduct.outputs:execOut",
            "CameraInfoPublish.inputs:execIn",
        ),
        (
            "RenderProduct.outputs:renderProductPath",
            "CameraInfoPublish.inputs:renderProductPath",
        ),
        ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
    ]
    set_values = [
        ("RenderProduct.inputs:cameraPrim", camera_prim_path),
        ("RenderProduct.inputs:width", width),
        ("RenderProduct.inputs:height", height),
        ("CameraInfoPublish.inputs:topicName", subtopics["camera_info"]),
        ("CameraInfoPublish.inputs:frameId", frame_id),
        ("CameraInfoPublish.inputs:nodeNamespace", namespace),
        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
    ]
    # (node name, helper type, topic, enabled, needs semantic labels)
    helper_streams = [
        ("RGBPublish", "rgb", subtopics["image"], True, False),
        ("DepthPublish", "depth", subtopics["depth"], publish_depth, False),
        (
            "SemanticPublish",
            "semantic_segmentation",
            "semantic_segmentation",
            publish_semantic,
            True,
        ),
        (
            "Bbox2dTightPublish",
            "bbox_2d_tight",
            "bbox_2d_tight",
            publish_bbox,
            True,
        ),
    ]
    for name, helper_type, topic, enabled, semantic_labels in helper_streams:
        if not enabled:
            continue
        create_nodes.append((name, "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connections.extend(
            [
                ("RenderProduct.outputs:execOut", f"{name}.inputs:execIn"),
                (
                    "RenderProduct.outputs:renderProductPath",
                    f"{name}.inputs:renderProductPath",
                ),
                ("Context.outputs:context", f"{name}.inputs:context"),
            ]
        )
        set_values.extend(
            [
                (f"{name}.inputs:type", helper_type),
                (f"{name}.inputs:topicName", topic),
                (f"{name}.inputs:frameId", frame_id),
                (f"{name}.inputs:nodeNamespace", namespace),
                (f"{name}.inputs:resetSimulationTimeOnStop", True),
            ]
        )
        if semantic_labels:
            set_values.append((f"{name}.inputs:enableSemanticLabels", True))

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: create_nodes,
            keys.CONNECT: connections,
            keys.SET_VALUES: set_values,
        },
    )
    extras = "".join(
        f", +{name}"
        for name, enabled in (
            ("depth", publish_depth),
            ("semantic", publish_semantic),
            ("bbox", publish_bbox),
        )
        if enabled
    )
    print(
        f"Recording: camera graph {graph_path} -> "
        f"{namespace}/{subtopics['image']} "
        f"({width}x{height}, prim {camera_prim_path}{extras})",
        flush=True,
    )


def _set_frame_skip(camera_keys, frame_skip: int) -> None:
    """Throttle camera helpers via frameSkipCount where the node supports
    it (publish every frame_skip+1 render frames)."""
    if frame_skip <= 0:
        return
    import contextlib

    import omni.graph.core as og

    for camera_key in camera_keys:
        for helper in ("RGBPublish", "DepthPublish"):
            attr_path = (
                f"{CAMERA_GRAPH_ROOT}/{camera_key}/{helper}"
                ".inputs:frameSkipCount"
            )
            # Helper node absent or an older bridge without the attribute.
            with contextlib.suppress(Exception):
                og.Controller.attribute(attr_path).set(int(frame_skip))


def setup_robot_camera_graphs(
    stage,
    robot_prim_path: str,
    sensors_path: Path,
    *,
    publish_depth: bool = False,
    frame_skip: int = 0,
) -> dict[str, str]:
    """Create one camera graph per robot camera.

    Returns {camera_key: camera_prim_path}.
    """
    configs = load_camera_configs(sensors_path)
    _check_topic_contract(configs)
    prim_paths = find_robot_camera_prims(stage, robot_prim_path, configs)
    for camera_key, camera_prim_path in prim_paths.items():
        _setup_single_camera_graph(
            camera_key,
            camera_prim_path,
            configs[camera_key],
            publish_depth=publish_depth,
        )
    _set_frame_skip(prim_paths.keys(), frame_skip)
    if frame_skip > 0:
        print(
            f"Recording: camera publishers skip {frame_skip} render "
            "frame(s) between messages",
            flush=True,
        )
    return prim_paths
