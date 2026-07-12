#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Isaac Sim 5.1.0 ROS bridge for Task 2 (thermal pad placement) teleoperation.

Barebone scene: robot + Task 2 objects + ground plane. For the full robot-room
scene (scripts/scenes/scene_robot_room_keyboard.py) use scene_room.py instead.

This script runs inside the plain Isaac Sim 5.1.0 container
(isaac-sim-5-1-0-workshop) with /isaac-sim/python.sh, while the Task 2 ROS
helper containers (task2_isaacsim/docker-compose.yml, reusing the Task 1
adapter/controller scripts) keep running on the host network.  It
publishes/subscribes the same `/isaac/*` topics as the Task 1 bridge so
`ros_republisher`, `position_controller`, `browser_controller`, and the
`teleop_adapters` (keyboard/GELLO) can be reused.

Task 2 uses plain PhysX instead of Isaac Lab + Newton/MJWarp because the
thermal pad asset relies on PhysxDeformableBodyAPI (GPU deformables).

rclpy comes from the ROS 2 libraries bundled with the isaacsim.ros2.bridge
extension; the launcher must export ROS_DISTRO=jazzy and LD_LIBRARY_PATH
pointing at the bundled jazzy/lib before starting this process.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaacsim_fr3duo_teleop_bridge_args import add_common_bridge_args


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd-path",
        default="/workspace/EBiM_Challenge/task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd",
        help="Robot USD file to load into the scene.",
    )
    parser.add_argument(
        "--objects-usd-path",
        default="/workspace/EBiM_Challenge/assets/task2_objects/"
        "task2_objects.usda",
        help="Task 2 objects USD (RAM boards, target, deformable "
        "thermal pad).",
    )
    parser.add_argument(
        "--objects-position",
        type=float,
        nargs=3,
        default=(0.8, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="World translation applied to the Task 2 objects root prim.",
    )
    parser.add_argument(
        "--objects-yaw-deg",
        type=float,
        default=0.0,
        help="World yaw rotation in degrees applied to the Task 2 objects "
        "root prim.",
    )
    parser.add_argument("--robot-prim-path", default="/World/Robot")
    parser.add_argument("--objects-prim-path", default="/World/Task2Objects")
    parser.add_argument(
        "--camera-position",
        type=float,
        nargs=3,
        default=(5.0, 0.0, 3.0),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--camera-target",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
    )
    add_common_bridge_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": args_cli.headless, "width": 1280, "height": 720}
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import isaacsim_fr3duo_teleop_bridge_core as core  # noqa: E402

import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402


def main():
    usd_path = Path(args_cli.usd_path).expanduser()
    objects_usd_path = Path(args_cli.objects_usd_path).expanduser()
    franka_root = Path(args_cli.franka_root).expanduser()
    if not usd_path.exists():
        raise FileNotFoundError(f"USD path does not exist: {usd_path}")
    if not objects_usd_path.exists():
        raise FileNotFoundError(
            f"Objects USD path does not exist: {objects_usd_path}"
        )

    groups = core._load_joint_groups(
        franka_root,
        args_cli.embodiment,
        include_browser_commands=not args_cli.disable_browser_command_topics,
    )

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / args_cli.physics_hz,
        rendering_dt=1.0 / args_cli.render_hz,
    )
    physics_context = world.get_physics_context()
    # GPU dynamics is required by the thermal pad (PhysxDeformableBodyAPI).
    physics_context.enable_gpu_dynamics(True)
    physics_context.set_broadphase_type("GPU")

    stage = omni.usd.get_context().get_stage()
    add_reference_to_stage(str(usd_path), args_cli.robot_prim_path)
    core.prepare_robot_prim(args_cli.robot_prim_path, args_cli)

    add_reference_to_stage(str(objects_usd_path), args_cli.objects_prim_path)
    core._place_objects(
        stage,
        args_cli.objects_prim_path,
        args_cli.objects_position,
        args_cli.objects_yaw_deg,
    )

    world.scene.add_default_ground_plane()
    core._add_dome_light(stage)
    if not args_cli.headless:
        set_camera_view(
            eye=list(args_cli.camera_position),
            target=list(args_cli.camera_target),
        )

    articulation_root_path = core._find_articulation_root_path(
        args_cli.robot_prim_path
    )
    robot = SingleArticulation(prim_path=articulation_root_path, name="robot")
    world.scene.add(robot)
    world.reset()

    print("Isaac Sim fr3duo Task 2 bridge started")
    print("Articulation root:", articulation_root_path)
    (
        group_indices,
        coupled_indices,
        steering_ids,
        drive_ids,
        spine_keyboard_controller,
        arm_keyboard_teleop,
    ) = core.setup_robot_control(robot, groups, args_cli)

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
    )


if __name__ == "__main__":
    main()
