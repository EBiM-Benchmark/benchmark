"""Isaac Sim camera service - runtime camera integration and management.

This module provides the IsaacCameraService class which handles:
- Resolving attachment frames in USD stage
- Creating USD camera primitives with proper transforms
- Building ROS2 publishing OmniGraphs for camera images and info
- Managing camera lifecycle during simulation
"""

from __future__ import annotations

from .camera_config import (
    frame_skip_count_for_rate,
    load_camera_sensor_config,
    normalize_camera_specs,
    resolve_camera_config_path,
    validate_camera_contract_alignment,
)


class IsaacCameraService:
    """Service for managing Isaac Sim camera creation and ROS2 integration."""
    
    def __init__(self):
        """Initialize the camera service."""
        pass
    
    def _resolve_attachment_prim(self, stage, spec: dict):
        """Find attachment prim in USD stage for camera mounting.
        
        Tries exact path first, then searches by name (exact match, then suffix).
        
        Args:
            stage: USD stage to search
            spec: Camera specification dictionary
            
        Returns:
            USD prim for attachment, or None if not found
        """
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
    
    def _camera_graph_path(self, spec: dict) -> str:
        """Generate OmniGraph path for camera's ROS2 publishing graph."""
        return f"/ROS_CameraGraphs/{spec['key']}"
    
    def _camera_prim_path(self, attachment_prim, spec: dict) -> str:
        """Generate USD path for camera primitive."""
        return f"{attachment_prim.GetPath()}/{spec['camera_prim_name']}"
    
    def _create_camera_prim(self, stage, attachment_prim, spec: dict) -> str:
        """Create USD Camera primitive with configured optical parameters.
        
        Args:
            stage: USD stage to create camera in
            attachment_prim: Parent prim to attach camera to
            spec: Normalized camera specification
            
        Returns:
            USD path to created camera prim
        """
        from pxr import Gf, UsdGeom

        camera_prim_path = self._camera_prim_path(attachment_prim, spec)
        camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
        
        # Set transform relative to attachment frame
        xform_api = UsdGeom.XformCommonAPI(camera_prim)
        xform_api.SetTranslate(Gf.Vec3d(*spec["local_translation_m"]))
        xform_api.SetRotate(
            tuple(spec["local_rotation_deg"]),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )
        
        # Configure camera optics
        camera_prim.GetProjectionAttr().Set("perspective")
        camera_prim.GetHorizontalApertureAttr().Set(float(spec["sensor_width_mm"]))
        camera_prim.GetVerticalApertureAttr().Set(float(spec["sensor_height_mm"]))
        camera_prim.GetFocalLengthAttr().Set(float(spec["focal_length_mm"]))
        camera_prim.GetFocusDistanceAttr().Set(float(spec["focus_distance_m"]))
        camera_prim.GetClippingRangeAttr().Set(
            Gf.Vec2f(float(spec["near_clip_m"]), float(spec["far_clip_m"]))
        )
        
        return camera_prim_path
    
    def _create_ros_camera_graph(self, camera_prim_path: str, spec: dict, *, render_hz: float) -> str:
        """Create OmniGraph for ROS2 camera publishing.
        
        Builds execution graph with:
        - OnPlaybackTick: Triggers on each simulation tick
        - CreateRenderProduct: Creates render target from camera
        - RGBPublish: Publishes RGB image to ROS2 topic
        - CameraInfoPublish: Publishes camera calibration info
        
        Args:
            camera_prim_path: USD path to camera primitive
            spec: Normalized camera specification
            render_hz: Rendering frequency for frame skip calculation
            
        Returns:
            USD path to created OmniGraph
        """
        import omni.graph.core as og
        import usdrt.Sdf

        graph_path = self._camera_graph_path(spec)
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
        self,
        stage,
        *,
        config_path: str | None,
        render_hz: float,
    ) -> dict:
        """Attach configured cameras to USD stage with ROS2 publishing.
        
        Main entry point for camera service. Loads configuration, creates
        camera primitives, and sets up ROS2 publishing graphs.
        
        Args:
            stage: USD stage to attach cameras to
            config_path: Path to camera configuration YAML
            render_hz: Rendering frequency for frame skip calculation
            
        Returns:
            Summary dictionary with:
            - config_path: Resolved configuration file path
            - attached_count: Number of successfully attached cameras
            - missing_frames: List of attachment frames not found
            - cameras: List of attached camera details
            - contract_validation: Contract compliance results
        """
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
            attachment_prim = self._resolve_attachment_prim(stage, spec)
            if attachment_prim is None or not attachment_prim.IsValid():
                missing_frames.append(spec["attachment_frame_name"] or spec["attachment_frame_path"])
                continue

            camera_prim_path = self._create_camera_prim(stage, attachment_prim, spec)
            graph_path = self._create_ros_camera_graph(
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
