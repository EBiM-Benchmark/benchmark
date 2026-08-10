#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Task 3 robot-room ROS bridge for plain Isaac Sim 5.1.0."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SHARED_SCENES_DIR = REPO_ROOT / "scripts" / "scenes"
TASK2_SCRIPTS_DIR = REPO_ROOT / "task2_isaacsim" / "scripts"
for module_dir in (SHARED_SCENES_DIR, TASK2_SCRIPTS_DIR):
    if str(module_dir) not in sys.path:
        sys.path.insert(0, str(module_dir))

import scene_robot_room_keyboard as room_scene  # noqa: E402
from gripper_profiles import (  # noqa: E402
    DEFAULT_GRIPPER,
    GRIPPER_PROFILE_NAMES,
    get_gripper_profile,
    get_profile_drive_gains,
)
from isaacsim_fr3duo_teleop_bridge_args import (  # noqa: E402
    add_common_bridge_args,
    resolve_recording_flags,
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--gripper",
        choices=GRIPPER_PROFILE_NAMES,
        default=DEFAULT_GRIPPER,
        help="Robot end-effector profile (default: robotiq).",
    )
    parser.add_argument(
        "--room-usd",
        type=Path,
        default=room_scene.asset_path("robot_room.usd"),
        help="Room USD to reference.",
    )
    parser.add_argument(
        "--robot-usd",
        type=Path,
        default=None,
        help="Override the robot USD selected by --gripper.",
    )
    parser.add_argument("--robot-x", type=float, default=-4.6)
    parser.add_argument("--robot-y", type=float, default=2.7)
    parser.add_argument("--robot-z", type=float, default=0.0)
    parser.add_argument("--robot-yaw", type=float, default=-90.0)
    parser.add_argument(
        "--head-placement",
        type=room_scene.head_placement_arg,
        default="random",
        help="Task 3 head placement: A-I or random.",
    )
    parser.add_argument(
        "--dynamic-beans",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable rigid-body physics for the Task 3 coffee beans.",
    )
    add_common_bridge_args(parser)
    parser.set_defaults(
        arm_teleop_gripper_open=None,
        arm_teleop_gripper_closed=None,
    )
    return parser


def resolve_profile_defaults(args: argparse.Namespace):
    profile = get_gripper_profile(args.gripper)
    if args.robot_usd is None:
        args.robot_usd = profile.robot_usd
    if args.arm_teleop_gripper_open is None:
        args.arm_teleop_gripper_open = profile.keyboard_positions[0]
    if args.arm_teleop_gripper_closed is None:
        args.arm_teleop_gripper_closed = profile.keyboard_positions[1]
    return profile


args_cli = build_arg_parser().parse_args()
profile_cli = resolve_profile_defaults(args_cli)
resolve_recording_flags(args_cli)


from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": args_cli.headless, "width": 1280, "height": 720}
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
if not args_cli.headless:
    enable_extension("omni.physx.ui")
simulation_app.update()

import isaacsim_fr3duo_teleop_bridge_core as core  # noqa: E402

import omni.kit.app  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402

ROBOT_PRIM_PATH = "/World/Robot"
TASK_OBJECTS_ROOT = "/World/Environment/RobotRoom/Asset/{}"
OBJECT_NAMES = ["head", "bowl2", "plate2", "spoon2", "cup", "simple_tray"]
OBJECT_PRIM_PATHS = [TASK_OBJECTS_ROOT.format(name) for name in OBJECT_NAMES]

def main() -> None:


    room_path = Path(args_cli.room_usd).expanduser()
    robot_path = Path(args_cli.robot_usd).expanduser()
    franka_root = Path(args_cli.franka_root).expanduser()
    if not room_path.is_file():
        raise FileNotFoundError(f"Room USD not found: {room_path}")
    if not robot_path.is_file():
        hint = ""
        if profile_cli.name == "robotiq":
            hint = (
                " Run task1_isaacsim/scripts/download_large_assets.sh first."
            )
        raise FileNotFoundError(f"Robot USD not found: {robot_path}.{hint}")

    groups = core._load_joint_groups(
        franka_root,
        args_cli.embodiment,
        include_browser_commands=not args_cli.disable_browser_command_topics,
    )
    # groups = core._load_joint_groups(
    #     include_browser_commands=not args_cli.disable_browser_command_topics,
    # )
    args_cli.task = "task3"
    args_cli.robot_x = -4.0
    args_cli.robot_y = -1.5
    args_cli.robot_z = 0.0
    args_cli.robot_yaw = -180.0

    robot_position = room_scene.resolve_robot_position(args_cli)
    robot_yaw = room_scene.resolve_robot_yaw(args_cli)


    room_scene.build_stage(
        omni.kit.app.get_app(),
        room_path=room_path,
        robot_path=robot_path,
        task="task3",
        robot_position=robot_position,
        robot_rotation=room_scene.yaw_to_quat(robot_yaw),
        robot_yaw=robot_yaw,
        head_placement=args_cli.head_placement,
        dynamic_beans=args_cli.dynamic_beans,
    )
    #Task 2 recording copy
    import recording3 as recording  # noqa: PLC0415

    # build_stage already created the eval camera prim + graph; the scene
    # camera config pass adopts them (pose from yaml) and only builds
    # graphs for cameras the scene did not author.




    physics_scene_path = core._find_physics_scene_path() or "/physicsScene"
    world = World(
        physics_prim_path=physics_scene_path,
        stage_units_in_meters=1.0,
        physics_dt=1.0 / args_cli.physics_hz,
        rendering_dt=1.0 / args_cli.render_hz,
        sim_params={"use_fabric": True},
    )

    world.get_physics_context().enable_fabric(True)
    #world.set_gpu_dynamics_enabled(True)
    stage = omni.usd.get_context().get_stage()

    recording.setup_recording_cameras(
        stage, args_cli, ROBOT_PRIM_PATH, "cameras_room.yaml"
    )
    core.prepare_robot_prim(ROBOT_PRIM_PATH, args_cli)
    core._configure_drives(
        ROBOT_PRIM_PATH,
        lambda joint_name: get_profile_drive_gains(
            profile_cli.name, joint_name
        ),
    )
    articulation_root_path = core._find_articulation_root_path(ROBOT_PRIM_PATH)
    robot = SingleArticulation(
        prim_path=articulation_root_path,
        name="task3_robot",
    )
    world.scene.add(robot)
    world.reset()

    print(f"Task 3 ROS bridge started (gripper={profile_cli.name})")
    print("Robot USD:", robot_path)
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

    tick_callbacks = recording.build_recording_tick_callbacks(
        world,
        robot,
        stage,
        args_cli,
        OBJECT_PRIM_PATHS,
        spine_controller=spine_keyboard_controller,
        arm_teleop=arm_keyboard_teleop,
    )

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
