# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Sim-side helpers for Task 2 demonstration recording.

camera_publishers -- robot camera OmniGraph ROS publishers driven by the
                     embodiment's camera_sensors.yaml (the sim clock topic
                     is published by the bridge node itself).
scene_cameras     -- scene-level cameras (e.g. eval_camera) driven by
                     config/cameras_<scene>.yaml: creates/adopts the Camera
                     prim, applies the configured pose, reuses the
                     camera_publishers graph builder.
scene_capture     -- ground-truth object/pad publishers and the scene
                     reset/randomize hotkey, run as run_teleop_loop tick
                     callbacks.

All modules import Isaac Sim modules at import time; import them only after
SimulationApp has been created and isaacsim.ros2.bridge is enabled.
Keep this __init__ free of Isaac imports (the helpers below import the
submodules lazily for the same reason).
"""

from __future__ import annotations

import sys
from pathlib import Path

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"


def setup_recording_cameras(
    stage, args_cli, robot_prim_path: str, default_scene_config: str
) -> None:
    """Build the robot/scene camera publishers a scene script asked for.

    Failures only warn: recording is optional for teleop."""
    if args_cli.enable_robot_cameras:
        from . import camera_publishers  # noqa: PLC0415

        try:
            camera_publishers.setup_robot_camera_graphs(
                stage,
                robot_prim_path,
                args_cli.camera_sensors_yaml,
                publish_depth=args_cli.robot_camera_depth,
                frame_skip=args_cli.robot_camera_frame_skip,
            )
        except Exception as exc:  # noqa: BLE001 - recording is optional
            print(
                f"Warning: robot camera publishers unavailable: {exc}",
                file=sys.stderr,
            )

    if args_cli.enable_scene_cameras:
        from . import scene_cameras  # noqa: PLC0415

        config = args_cli.scene_cameras_config or (
            _CONFIG_DIR / default_scene_config
        )
        try:
            scene_cameras.setup_scene_camera_graphs(stage, config)
        except Exception as exc:  # noqa: BLE001 - recording is optional
            print(
                f"Warning: scene camera publishers unavailable: {exc}",
                file=sys.stderr,
            )


def build_recording_tick_callbacks(
    world,
    robot,
    stage,
    args_cli,
    objects_root: str,
    *,
    spine_controller,
    arm_teleop,
) -> list:
    """Ground-truth publisher + scene-reset hotkey tick callbacks.

    Failures only warn: recording is optional for teleop."""
    callbacks = []
    if args_cli.publish_ground_truth:
        from .scene_capture import GroundTruthPublisher  # noqa: PLC0415

        try:
            callbacks.append(
                GroundTruthPublisher(
                    stage,
                    objects_root,
                    pad_points_every=args_cli.ground_truth_pad_every,
                )
            )
        except Exception as exc:  # noqa: BLE001 - recording is optional
            print(f"Warning: ground-truth publisher unavailable: {exc}")
    if args_cli.scene_reset_hotkey:
        from .scene_capture import SceneResetController  # noqa: PLC0415

        try:
            callbacks.append(
                SceneResetController(
                    world,
                    robot,
                    stage,
                    objects_root,
                    spine_controller=spine_controller,
                    arm_teleop=arm_teleop,
                    randomize=args_cli.randomize_objects,
                    xy_jitter_m=args_cli.randomize_xy_cm / 100.0,
                    yaw_jitter_deg=args_cli.randomize_yaw_deg,
                )
            )
        except Exception as exc:  # noqa: BLE001 - recording is optional
            print(f"Warning: scene reset hotkey unavailable: {exc}")
    return callbacks
