#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Isaac Sim 5.1.0 ROS bridge for Task 2 teleoperation in the robot room scene.

Builds the same stage as scripts/scenes/scene_robot_room_keyboard.py --task
task2 (robot room + mobile FR3 + Task 2 objects on the table + the
/isaac/eval_camera/* OmniGraph publishers) by reusing that script's
build_stage(), and drives it with the same ROS teleop bridge as the barebone
scene_barebone.py (shared isaacsim_fr3duo_teleop_bridge_core module).

Runs inside the plain Isaac Sim 5.1.0 container with /isaac-sim/python.sh; see
scene_barebone.py for the environment requirements and
task2_isaacsim/README.md for the full workflow.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_SCENES_DIR = _REPO_ROOT / "scripts" / "scenes"
if str(_SCENES_DIR) not in sys.path:
    sys.path.insert(0, str(_SCENES_DIR))

# Import-safe before SimulationApp: pxr/omni imports live inside its functions.
import scene_robot_room_keyboard as room_scene  # noqa: E402
from isaacsim_fr3duo_teleop_bridge_args import (  # noqa: E402
    add_common_bridge_args,
    resolve_recording_flags,
)

DEFAULT_ROBOT_USD = (
    _REPO_ROOT
    / "task1_isaacsim"
    / "assets"
    / "Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd"
)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--room-usd",
        type=Path,
        default=room_scene.asset_path("robot_room.usd"),
        help="Room USD to reference.",
    )
    parser.add_argument(
        "--robot-usd",
        type=Path,
        default=DEFAULT_ROBOT_USD,
        help="Robot USD to reference.",
    )
    parser.add_argument(
        "--task",
        choices=tuple(room_scene.TASK_ROBOT_POSES),
        default="task2",
        help="Task preset used for the robot spawn position and scene "
        "content.",
    )
    parser.add_argument(
        "--robot-x",
        type=float,
        default=None,
        help="Override the preset robot X position.",
    )
    parser.add_argument(
        "--robot-y",
        type=float,
        default=None,
        help="Override the preset robot Y position.",
    )
    parser.add_argument(
        "--robot-z",
        type=float,
        default=None,
        help="Override the preset robot Z position.",
    )
    parser.add_argument(
        "--robot-yaw",
        type=float,
        default=None,
        help="Override the preset robot yaw in degrees.",
    )
    parser.add_argument(
        "--head-placement",
        type=room_scene.head_placement_arg,
        default="random",
        help="Task3 head placement: A-I, or random (task3 only).",
    )
    add_common_bridge_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
resolve_recording_flags(args_cli)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": args_cli.headless, "width": 1280, "height": 720}
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

# Must be enabled before build_stage(): the task2 eval camera OmniGraph uses
# isaacsim.ros2.bridge node types.
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import isaacsim_fr3duo_teleop_bridge_core as core  # noqa: E402

import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402

ROBOT_PRIM_PATH = "/World/Robot"
TASK_OBJECTS_ROOT = "/World/Scene/task_objects"
TASK2_VIEW_EYE = (1.0, 2.5, 1.35)


def main():
    room_path = Path(args_cli.room_usd).expanduser()
    robot_path = Path(args_cli.robot_usd).expanduser()
    franka_root = Path(args_cli.franka_root).expanduser()
    if not room_path.is_file():
        raise FileNotFoundError(f"Room USD not found: {room_path}")
    if not robot_path.is_file():
        raise FileNotFoundError(f"Robot USD not found: {robot_path}")

    groups = core._load_joint_groups(
        franka_root,
        args_cli.embodiment,
        include_browser_commands=not args_cli.disable_browser_command_topics,
    )

    robot_position = room_scene.resolve_robot_position(args_cli)
    robot_yaw = room_scene.resolve_robot_yaw(args_cli)

    app = omni.kit.app.get_app()
    room_scene.build_stage(
        app,
        room_path=room_path,
        robot_path=robot_path,
        task=args_cli.task,
        robot_position=robot_position,
        robot_rotation=room_scene.yaw_to_quat(robot_yaw),
        robot_yaw=robot_yaw,
        head_placement=args_cli.head_placement,
    )
    if args_cli.task == "task2" and not args_cli.headless:
        # Override build_stage's room overview with a view of the task2 table.
        set_camera_view(
            eye=list(TASK2_VIEW_EYE),
            target=list(room_scene.TASK2_TABLE_POSITION),
            camera_prim_path="/OmniverseKit_Persp",
        )

    stage = omni.usd.get_context().get_stage()
    if args_cli.enable_robot_cameras:
        from recording import camera_publishers  # noqa: PLC0415

        try:
            camera_publishers.setup_robot_camera_graphs(
                stage,
                ROBOT_PRIM_PATH,
                franka_root,
                args_cli.embodiment,
                publish_depth=args_cli.robot_camera_depth,
                frame_skip=args_cli.robot_camera_frame_skip,
            )
        except Exception as exc:  # noqa: BLE001 - recording is optional
            camera_publishers.print_setup_failure(exc)

    # Adopt the room's authored PhysicsScene rather than creating a second one.
    physics_scene_path = core._find_physics_scene_path() or "/physicsScene"
    world = World(
        physics_prim_path=physics_scene_path,
        stage_units_in_meters=1.0,
        physics_dt=1.0 / args_cli.physics_hz,
        rendering_dt=1.0 / args_cli.render_hz,
    )
    physics_context = world.get_physics_context()
    # GPU dynamics is required by the thermal pad (PhysxDeformableBodyAPI).
    physics_context.enable_gpu_dynamics(True)
    physics_context.set_broadphase_type("GPU")

    core.prepare_robot_prim(ROBOT_PRIM_PATH, args_cli)

    articulation_root_path = core._find_articulation_root_path(ROBOT_PRIM_PATH)
    robot = SingleArticulation(prim_path=articulation_root_path, name="robot")
    world.scene.add(robot)
    world.reset()

    print("Isaac Sim fr3duo Task 2 room bridge started")
    print("Physics scene:", physics_scene_path)
    print("Articulation root:", articulation_root_path)
    (
        group_indices,
        coupled_indices,
        steering_ids,
        drive_ids,
        spine_keyboard_controller,
        arm_keyboard_teleop,
    ) = core.setup_robot_control(robot, groups, args_cli)

    tick_callbacks = []
    if args_cli.publish_ground_truth:
        from recording.scene_capture import (
            GroundTruthPublisher,  # noqa: PLC0415
        )

        try:
            tick_callbacks.append(
                GroundTruthPublisher(
                    stage,
                    TASK_OBJECTS_ROOT,
                    pad_points_every=args_cli.ground_truth_pad_every,
                )
            )
        except Exception as exc:  # noqa: BLE001 - recording is optional
            print(f"Warning: ground-truth publisher unavailable: {exc}")
    if args_cli.scene_reset_hotkey:
        from recording.scene_capture import (
            SceneResetController,  # noqa: PLC0415
        )

        try:
            tick_callbacks.append(
                SceneResetController(
                    world,
                    robot,
                    stage,
                    TASK_OBJECTS_ROOT,
                    spine_controller=spine_keyboard_controller,
                    arm_teleop=arm_keyboard_teleop,
                    randomize=args_cli.randomize_objects,
                    xy_jitter_m=args_cli.randomize_xy_cm / 100.0,
                    yaw_jitter_deg=args_cli.randomize_yaw_deg,
                )
            )
        except Exception as exc:  # noqa: BLE001 - recording is optional
            print(f"Warning: scene reset hotkey unavailable: {exc}")

    core.run_teleop_loop(
        simulation_app,
        world,
        robot,
        groups,
        group_indices,
        coupled_indices,
        steering_ids,
        drive_ids,
        spine_keyboard_controller,
        arm_keyboard_teleop,
        args_cli,
        # Keep rendering in headless sessions so the task2 eval camera
        # OmniGraph still publishes /isaac/eval_camera/*.
        force_render=True,
        tick_callbacks=tick_callbacks,
    )


if __name__ == "__main__":
    main()
