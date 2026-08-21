#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Isaac Sim 5.1.0 ROS bridge for Task 2 teleoperation in the robot room scene.

Builds the same stage as scripts/scenes/scene_robot_room_keyboard.py --task
task2 (robot room + mobile FR3 + Task 2 objects on the table + the
/isaac/eval_camera/* OmniGraph publishers) by reusing that script's
build_stage(), and drives it with the same ROS teleop bridge as the barebone
scene_barebone.py (shared isaacsim_fr3duo_teleop_bridge_core module).

--room-usd defaults to robot_room.usd. A NuRec .usdz instead composes that
volume with --mesh under one joint xform (--xyz-deg / --xyz / --scale).

Runs inside the plain Isaac Sim 5.1.0 container with /isaac-sim/python.sh; see
scene_barebone.py for the environment requirements and
task2_isaacsim/README.md for the full workflow.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

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

ROBOT_PRIM_PATH = "/World/Robot"
TASK_OBJECTS_ROOT = "/World/Scene/task_objects"
TASK2_VIEW_EYE = (1.0, 2.5, 1.35)
ENV_ROOT_PATH = "/World/Environment/Room"
ENV_VOLUME_PATH = f"{ENV_ROOT_PATH}/Volume"
ENV_MESH_PATH = f"{ENV_ROOT_PATH}/Mesh"
TABLE_PATH = "/World/Scene/Table"
# GLB node is already Rx(+90°); cancel it so mesh and volume share --xyz-deg.
MESH_RX_OFFSET_DEG = -90.0
# Crop in the volume's local (pre-Rx) frame; drops far Gaussian floaters.
NUREC_CROP_MIN = (-1.73, -1.24, -1.82)
NUREC_CROP_MAX = (1.51, 1.09, 4.02)


def _is_nurec_asset(path: Path) -> bool:
    return path.suffix.lower() in {".usdz", ".nurec"}


def _as_scale(values: Any) -> tuple[float, float, float]:
    if not values:
        return (1.0, 1.0, 1.0)
    vals = [float(v) for v in values]
    if len(vals) == 1:
        return (vals[0], vals[0], vals[0])
    if len(vals) == 3:
        return (vals[0], vals[1], vals[2])
    raise ValueError("--scale expects 1 or 3 floats")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--room-usd",
        type=Path,
        default=room_scene.asset_path("robot_room.usd"),
        help="Room USD to reference (default: robot_room.usd). A NuRec "
        ".usdz is composed with --mesh instead of the original room.",
    )
    parser.add_argument(
        "--robot-usd",
        type=Path,
        default=DEFAULT_ROBOT_USD,
        help="Robot USD to reference.",
    )
    parser.add_argument(
        "--mesh",
        type=Path,
        default=None,
        help="Collider mesh (GLB/USD) for a NuRec .usdz room. Required "
        "when --room-usd is a .usdz; ignored otherwise.",
    )
    parser.add_argument(
        "--xyz-deg",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("RX", "RY", "RZ"),
        help="Euler XYZ rotation (degrees) of the NuRec volume and mesh "
        "together. Ignored for a regular room USD.",
    )
    parser.add_argument(
        "--xyz",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="Translate (metres) of the NuRec volume and mesh together. "
        "Ignored for a regular room USD.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        nargs="+",
        default=[1.0],
        metavar="S",
        help="Scale of the NuRec volume and mesh together: one uniform "
        "value or X Y Z. Ignored for a regular room USD.",
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

_nurec_mode = _is_nurec_asset(Path(args_cli.room_usd).expanduser())
if _nurec_mode and args_cli.mesh is None:
    raise SystemExit("NuRec --room-usd requires --mesh PATH (GLB/USD collider).")

# WebRTC livestream (same idea as Isaac Lab --livestream 1 / PUBLIC_IP).
# Match /isaac-sim/standalone_examples/api/isaacsim.simulation_app/livestream.py
# — bare headless+webrtc often connects but shows a black viewport.
if args_cli.livestream:
    import os

    args_cli.headless = True
    _public_ip = os.environ.get("PUBLIC_IP", "").strip()
    sys.argv.append("--/app/livestream/port=49100")
    # Pin the WebRTC media/host UDP port so it is firewallable. Without this,
    # streamsdk grabs a random ephemeral UDP port each run, which can never
    # match a fixed Security Group rule -> signaling connects but media is
    # dropped and the client shows a gray/black viewport. 47998 matches the
    # UDP rule documented in setup.md.
    _media_port = os.environ.get("LIVESTREAM_MEDIA_PORT", "47998").strip()
    sys.argv.append(f"--/app/livestream/fixedHostPort={_media_port}")
    sys.argv.append(f"--/app/livestream/minHostPort={_media_port}")
    sys.argv.append(f"--/app/livestream/maxHostPort={_media_port}")
    if _public_ip:
        sys.argv.append(f"--/app/livestream/publicEndpointAddress={_public_ip}")

# Isaac Sim 5.1 NuRec volumes need UJITSO before Kit starts.
if _nurec_mode:
    sys.argv.append("--/UJITSO/geometry=true")
    sys.argv.append("--/renderer/multiGpu/enabled=false")
    sys.argv.append("--/rtx/rtpt/gaussian/skipTonemapping/enabled=false")

from isaacsim import SimulationApp  # noqa: E402

_app_config: dict = {"headless": args_cli.headless, "width": 1280, "height": 720}
if args_cli.livestream:
    _app_config.update(
        {
            "window_width": 1920,
            "window_height": 1080,
            "hide_ui": False,  # required — otherwise WebRTC gets a black frame
            "renderer": "RaytracedLighting",
            "display_options": 3286,
        }
    )
if _nurec_mode:
    _app_config["multi_gpu"] = False
    _app_config["renderer"] = "RaytracedLighting"
simulation_app = SimulationApp(launch_config=_app_config)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

# Must be enabled before build_stage(): the task2 eval camera OmniGraph uses
# isaacsim.ros2.bridge node types.
enable_extension("isaacsim.ros2.bridge")
if args_cli.livestream:
    enable_extension("omni.kit.livestream.webrtc")
    simulation_app.set_setting("/app/window/drawMouse", True)
simulation_app.update()

import isaacsim_fr3duo_teleop_bridge_core as core  # noqa: E402

import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402


def _crop_nurec_volumes(stage: Any, root_path: str) -> int:
    from pxr import Gf, Usd

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0
    cropped = 0
    for prim in Usd.PrimRange(root):
        flag = prim.GetAttribute("omni:nurec:isNuRecVolume")
        if not (flag and flag.IsValid() and flag.Get()):
            continue
        prim.GetAttribute("omni:nurec:crop:minBounds").Set(Gf.Vec3f(*NUREC_CROP_MIN))
        prim.GetAttribute("omni:nurec:crop:maxBounds").Set(Gf.Vec3f(*NUREC_CROP_MAX))
        cropped += 1
    return cropped


def _author_mesh_collision(stage: Any, root_path: str) -> int:
    from pxr import Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0
    authored = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(
            "meshSimplification"
        )
        authored += 1
    return authored


def compose_nurec_environment(
    stage: Any,
    *,
    volume_path: Path,
    mesh_path: Path,
    xyz: tuple[float, float, float],
    xyz_deg: tuple[float, float, float],
    scale: tuple[float, float, float],
) -> None:
    """Volume + mesh under one joint xform; mesh only adds the GLB Rx offset."""
    from pxr import Gf, UsdGeom

    UsdGeom.Scope.Define(stage, "/World/Environment")
    root = UsdGeom.Xform.Define(stage, ENV_ROOT_PATH).GetPrim()
    room_scene.set_xform(root, xyz, room_scene.euler_xyz_to_quat(xyz_deg))
    UsdGeom.Xformable(root).AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*scale)
    )

    volume = UsdGeom.Xform.Define(stage, ENV_VOLUME_PATH).GetPrim()
    volume.GetReferences().AddReference(str(volume_path.resolve()))
    n_crop = _crop_nurec_volumes(stage, ENV_VOLUME_PATH)
    print(f"NuRec crop applied on {n_crop} volume prim(s)")

    mesh = UsdGeom.Xform.Define(stage, ENV_MESH_PATH).GetPrim()
    room_scene.set_xform(
        mesh,
        (0.0, 0.0, 0.0),
        room_scene.euler_xyz_to_quat((MESH_RX_OFFSET_DEG, 0.0, 0.0)),
    )
    mesh_asset = UsdGeom.Xform.Define(stage, f"{ENV_MESH_PATH}/Asset").GetPrim()
    mesh_asset.GetReferences().AddReference(str(mesh_path.resolve()))
    n_col = _author_mesh_collision(stage, ENV_MESH_PATH)
    UsdGeom.Imageable(mesh).MakeInvisible()
    print(f"Mesh collision on {n_col} Mesh prim(s) (hidden; volume stays visible)")
    print("Joint xyz:", xyz, "Rxyz deg:", xyz_deg, "scale:", scale)
    print("Volume:", volume_path)
    print("Mesh:", mesh_path)


def build_nurec_stage(
    app: Any,
    *,
    volume_path: Path,
    mesh_path: Path,
    robot_path: Path,
    robot_position: tuple[float, float, float],
    robot_rotation: tuple[float, float, float, float],
    task: str,
    xyz: tuple[float, float, float],
    xyz_deg: tuple[float, float, float],
    scale: tuple[float, float, float],
) -> Any:
    from pxr import UsdGeom

    context = omni.usd.get_context()
    context.new_stage()
    for _ in range(10):
        app.update()
    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Could not create an Isaac Sim stage.")

    stage.SetFramesPerSecond(60.0)
    stage.SetTimeCodesPerSecond(60.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    compose_nurec_environment(
        stage,
        volume_path=volume_path,
        mesh_path=mesh_path,
        xyz=xyz,
        xyz_deg=xyz_deg,
        scale=scale,
    )
    room_scene.reference_usd(
        stage, ROBOT_PRIM_PATH, robot_path, robot_position, robot_rotation
    )
    if task == "task2":
        table_usd = room_scene.asset_path("table_edit.usd")
        if not table_usd.is_file():
            raise FileNotFoundError(f"Task 2 table USD missing: {table_usd}")
        table_xy = room_scene.TASK2_TABLE_POSITION
        room_scene.reference_usd(
            stage,
            TABLE_PATH,
            table_usd,
            (table_xy[0], table_xy[1], 0.0),
            (1.0, 0.0, 0.0, 0.0),
        )
        _author_mesh_collision(stage, TABLE_PATH)
        room_scene.load_deformable_assets(stage)
        room_scene.setup_deformable_camera(stage)

    for _ in range(10):
        app.update()
    return stage


def main():
    room_path = Path(args_cli.room_usd).expanduser()
    robot_path = Path(args_cli.robot_usd).expanduser()
    franka_root = Path(args_cli.franka_root).expanduser()
    if not room_path.is_file():
        raise FileNotFoundError(f"Room USD not found: {room_path}")
    if not robot_path.is_file():
        raise FileNotFoundError(f"Robot USD not found: {robot_path}")

    nurec_mode = _is_nurec_asset(room_path)
    mesh_path = (
        Path(args_cli.mesh).expanduser() if args_cli.mesh is not None else None
    )
    if nurec_mode and (mesh_path is None or not mesh_path.is_file()):
        raise FileNotFoundError(f"NuRec room requires --mesh (missing: {mesh_path})")

    groups = core._load_joint_groups(
        franka_root,
        args_cli.embodiment,
        include_browser_commands=not args_cli.disable_browser_command_topics,
    )

    robot_position = room_scene.resolve_robot_position(args_cli)
    robot_yaw = room_scene.resolve_robot_yaw(args_cli)

    app = omni.kit.app.get_app()
    if nurec_mode:
        build_nurec_stage(
            app,
            volume_path=room_path,
            mesh_path=mesh_path,
            robot_path=robot_path,
            robot_position=robot_position,
            robot_rotation=room_scene.yaw_to_quat(robot_yaw),
            task=args_cli.task,
            xyz=tuple(args_cli.xyz),
            xyz_deg=tuple(args_cli.xyz_deg),
            scale=_as_scale(args_cli.scale),
        )
    else:
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
    if args_cli.task == "task2":
        # Override build_stage's room overview with a view of the task2 table.
        # Also required for --livestream (headless Kit, WebRTC streams this
        # perspective camera).
        set_camera_view(
            eye=list(TASK2_VIEW_EYE),
            target=list(room_scene.TASK2_TABLE_POSITION),
            camera_prim_path="/OmniverseKit_Persp",
        )

    stage = omni.usd.get_context().get_stage()
    import recording  # noqa: PLC0415

    # build_stage already created the eval camera prim + graph; the scene
    # camera config pass adopts them (pose from yaml) and only builds
    # graphs for cameras the scene did not author.
    recording.setup_recording_cameras(
        stage, args_cli, ROBOT_PRIM_PATH, "cameras_room.yaml"
    )

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
    if nurec_mode:
        world.scene.add_default_ground_plane()

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

    tick_callbacks = recording.build_recording_tick_callbacks(
        world,
        robot,
        stage,
        args_cli,
        TASK_OBJECTS_ROOT,
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
