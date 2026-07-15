"""Dynamic Isaac camera attachment and ROS2 publisher setup.

DEPRECATED: This module has been moved to services/isaac_camera_service/

New location: services/isaac_camera_service/camera_service.py

This file is kept for backward compatibility only. New code should use
IsaacCameraService class from isaac_camera_service:

    from isaac_camera_service import IsaacCameraService
    
    camera_service = IsaacCameraService()
    summary = camera_service.attach_configured_cameras(
        stage=stage,
        config_path=config_path,
        render_hz=render_hz
    )

The old attach_configured_cameras function is still available below but
will be removed in a future version.
"""

from __future__ import annotations

from isaac_camera_config import (
    frame_skip_count_for_rate,
    load_camera_sensor_config,
    normalize_camera_specs,
    resolve_camera_config_path,
    validate_camera_contract_alignment,
)


def _resolve_attachment_prim(stage, spec: dict):
    attachment_frame_path = str(spec.get("attachment_frame_path", "")).strip()
    if attachment_frame_path:
        try:
            prim = stage.GetPrimAtPath(attachment_frame_path)
        except Exception:
            prim = None
        if prim and prim.IsValid():
            return prim

    attachment_frame_name = str(spec.get("attachment_frame_name", "")).strip()
    if not attachment_frame_name:
        return None

    exact_match = None
    suffix_match = None
    try:
        for prim in stage.Traverse():
            prim_name = prim.GetName()
            if prim_name == attachment_frame_name:
                exact_match = prim
                break
            if prim_name.endswith(attachment_frame_name):
                suffix_match = prim
    except Exception:
        return None
    return exact_match or suffix_match


def _camera_graph_path(spec: dict) -> str:
    return f"/ROS_CameraGraphs/{spec['key']}"


def _camera_prim_path(attachment_prim, spec: dict) -> str:
    return f"{attachment_prim.GetPath()}/{spec['camera_prim_name']}"


def _create_camera_prim(stage, attachment_prim, spec: dict) -> str:
    from pxr import Gf, UsdGeom

    camera_prim_path = _camera_prim_path(attachment_prim, spec)
    camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
    xform_api = UsdGeom.XformCommonAPI(camera_prim)
    xform_api.SetTranslate(Gf.Vec3d(*spec["local_translation_m"]))
    xform_api.SetRotate(
        tuple(spec["local_rotation_deg"]),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )
    camera_prim.GetProjectionAttr().Set("perspective")
    camera_prim.GetHorizontalApertureAttr().Set(float(spec["sensor_width_mm"]))
    camera_prim.GetVerticalApertureAttr().Set(float(spec["sensor_height_mm"]))
    camera_prim.GetFocalLengthAttr().Set(float(spec["focal_length_mm"]))
    camera_prim.GetFocusDistanceAttr().Set(float(spec["focus_distance_m"]))
    camera_prim.GetClippingRangeAttr().Set(
        Gf.Vec2f(float(spec["near_clip_m"]), float(spec["far_clip_m"]))
    )
    return camera_prim_path


def _create_ros_camera_graph(camera_prim_path: str, spec: dict, *, render_hz: float) -> str:
    import omni.graph.core as og
    import usdrt.Sdf

    graph_path = _camera_graph_path(spec)
    frame_skip_count = frame_skip_count_for_rate(
        publish_hz=spec["publish_hz"],
        render_hz=render_hz,
    )
    image_topic_name = spec["image_topic"].rsplit("/", 1)[-1]
    camera_info_topic_name = spec["camera_info_topic"].rsplit("/", 1)[-1]

    keys = og.Controller.Keys
    try:
        graph_handle, _, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                    ("CreateRenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                    (
                        "CreateRenderProduct.outputs:renderProductPath",
                        "CameraInfoPublish.inputs:renderProductPath",
                    ),
                ],
                keys.SET_VALUES: [
                    ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                    ("CreateRenderProduct.inputs:width", int(spec["width"])),
                    ("CreateRenderProduct.inputs:height", int(spec["height"])),
                    ("RGBPublish.inputs:type", "rgb"),
                    ("RGBPublish.inputs:frameId", spec["frame_id"]),
                    ("RGBPublish.inputs:nodeNamespace", spec["node_namespace"]),
                    ("RGBPublish.inputs:topicName", image_topic_name),
                    ("RGBPublish.inputs:frameSkipCount", frame_skip_count),
                    ("CameraInfoPublish.inputs:frameId", spec["frame_id"]),
                    ("CameraInfoPublish.inputs:nodeNamespace", spec["node_namespace"]),
                    ("CameraInfoPublish.inputs:topicName", camera_info_topic_name),
                    ("CameraInfoPublish.inputs:frameSkipCount", frame_skip_count),
                ],
            },
        )
        try:
            og.Controller.evaluate_sync(graph_handle)
        except Exception:
            pass
    except Exception as exc:
        # Graph already exists in the USD (e.g. baked into the stage file).
        # The existing graph will handle publishing — treat as success.
        print(f"Note: Camera graph at {graph_path} already exists, reusing: {exc}")
    return graph_path


def attach_configured_cameras(
    stage,
    *,
    config_path: str | None,
    render_hz: float,
) -> dict:
    if stage is None or not config_path:
        return {
            "config_path": None,
            "attached_count": 0,
            "missing_frames": [],
            "cameras": [],
            "contract_validation": {
                "compliant": True,
                "errors": [],
                "warnings": [],
                "expected_video_keys": [],
                "configured_video_keys": [],
            },
        }

    config = load_camera_sensor_config(config_path)
    specs = normalize_camera_specs(config)
    contract_validation = validate_camera_contract_alignment(specs)
    attached_cameras = []
    missing_frames = []

    for spec in specs:
        attachment_prim = _resolve_attachment_prim(stage, spec)
        if attachment_prim is None or not attachment_prim.IsValid():
            missing_frames.append(spec["attachment_frame_name"] or spec["attachment_frame_path"])
            continue

        camera_prim_path = _create_camera_prim(stage, attachment_prim, spec)
        graph_path = _create_ros_camera_graph(
            camera_prim_path,
            spec,
            render_hz=render_hz,
        )
        attached_cameras.append(
            {
                "key": spec["key"],
                "attachment_prim_path": str(attachment_prim.GetPath()),
                "camera_prim_path": camera_prim_path,
                "graph_path": graph_path,
                "image_topic": spec["image_topic"],
                "camera_info_topic": spec["camera_info_topic"],
                "frame_id": spec["frame_id"],
                "resolution": {
                    "width": spec["width"],
                    "height": spec["height"],
                },
                "contract_video_key": spec["contract_video_key"],
                "publish_hz": spec["publish_hz"],
            }
        )

    return {
        "config_path": resolve_camera_config_path(config_path),
        "attached_count": len(attached_cameras),
        "missing_frames": missing_frames,
        "cameras": attached_cameras,
        "contract_validation": contract_validation,
    }
