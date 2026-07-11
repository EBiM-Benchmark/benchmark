#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Launch the robot room in Isaac Sim with the mobile FR3 placed."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

COMMON_DIR = Path(__file__).resolve().parents[1] / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import asset_path, franka_urdf_path

ISAACSIM_EXPERIENCES = {
    "base": "/isaac-sim/apps/isaacsim.exp.base.kit",
    "full": "/isaac-sim/apps/isaacsim.exp.full.kit",
}
ROS2_BRIDGE_ROOT = Path("/isaac-sim/exts/isaacsim.ros2.bridge")
ROS2_ENV_READY_VAR = "EBIM_ROS2_BRIDGE_ENV_READY"
INSIDE_KIT_ENV_VAR = "EBIM_SCENE_LAUNCH_INSIDE_KIT"
INNER_ARGV_ENV_VAR = "EBIM_SCENE_LAUNCH_ARGV"
ISAACSIM_LAUNCHER = Path("/isaac-sim/isaac-sim.sh")
DEFAULT_BEAN_COLOR = (0.20, 0.12, 0.07)
DEFAULT_BEAN_COUNT = 300
DEFAULT_BEAN_DENSITY = 850.0
BOWL_USD = asset_path("bowl2.usd")
TASK3_BOWL_POSITION = (-4.3, -1.5, 0.74659)
TASK3_HEAD_PLACEMENTS = {
    "A": ((-2.8, 1.7, 0.74659), (0.0, 0.0, 270.0)),
    "B": ((-2.4, 1.7, 0.74659), (0.0, 0.0, 270.0)),
    "C": ((-2.0, 1.7, 0.74659), (0.0, 0.0, 270.0)),
    "D": ((-1.6, 1.7, 0.74659), (0.0, 0.0, 270.0)),
    "E": ((-1.35, 1.95, 0.74659), (0.0, 0.0, 0.0)),
    "F": ((-1.6, 2.2, 0.74659), (0.0, 0.0, 90.0)),
    "G": ((-2.0, 2.2, 0.74659), (0.0, 0.0, 90.0)),
    "H": ((-2.4, 2.2, 0.74659), (0.0, 0.0, 90.0)),
    "I": ((-2.8, 2.2, 0.74659), (0.0, 0.0, 90.0)),
}
INITIAL_VIEW_POSE = (
    (-8.12589, -3.29067, 2.79653),
    (73.13762, 0.0, -50.88313),
)
BEAN_PHYSICS = {
    "radius": 0.0025,
    "half_height": 0.0016,
    "spawn_height": 0.02,
    "spawn_wall_thickness": 0.016,
    "spawn_spacing_scale": 1.2,
    "friction": 0.55,
    "restitution": 0.02,
}
TASK_ROBOT_POSES = {
    "task1": {"position": (4.4, -2.5, 0.0), "yaw": 90.0},
    "task2": {"position": (4.4, 2.6, 0.0), "yaw": -90.0},
    "task3": {"position": (-4.6, 2.7, 0.0), "yaw": -90.0},
}
INITIAL_ROBOT_JOINT_POS = {
    "left_fr3v2_joint1": 0.0,
    "left_fr3v2_joint2": -1.5,
    "left_fr3v2_joint3": 0.0,
    "left_fr3v2_joint4": -2.2,
    "left_fr3v2_joint5": 0.0,
    "left_fr3v2_joint6": 1.5,
    "left_fr3v2_joint7": 0.785,
    "right_fr3v2_joint1": 0.0,
    "right_fr3v2_joint2": -1.5,
    "right_fr3v2_joint3": 0.0,
    "right_fr3v2_joint4": -2.2,
    "right_fr3v2_joint5": 0.0,
    "right_fr3v2_joint6": 1.5,
    "right_fr3v2_joint7": 0.785,
}

# Task 2 Specific
TASK2_TABLE_POSITION = (2.05, 1.95, 0.75)
TASK2_CAMERA_POSITION = (2.087, 1.885, 2.7)
TASK2_OBJECT_SPAWN_CONFIG = {  # relative to table origin
    "thermalpad": {  # 2 deformable meshes + attachment
        "asset_path": "task2_objects/Ram_ThermalPad_Res20_Top.usda",
        "position": (-0.3, 0.0, 0.1),
        "rotation": (0.70711, 0.0, 0.0, 0.70711),
    },
    "thermalpad_base": {  # 1 rigid kinematic mesh
        "asset_path": "task2_objects/sticker_base.usda",
        "position": (-0.31, -0.04, 0.017),
        "rotation": (1.0, 0.0, 0.0, 0.0),
    },
    "board_target": {  # 1 rigid body
        "asset_path": "task2_objects/Ram_Board_Target.usda",
        "position": (0.1, 0.0, 0.0),
        "rotation": (0.70711, 0.0, 0.0, 0.70711),
    },
    "boards": {  # 3 rigid bodies
        "asset_path": "task2_objects/Ram_Board.usda",
        "spawns": [
            {
                "position": (-0.1, 0.0, 0.0),
                "rotation": (0.70711, 0.0, 0.0, 0.70711),
            },
            {
                "position": (0.0, 0.0, 0.0),
                "rotation": (0.70711, 0.0, 0.0, 0.70711),
            },
            {
                "position": (0.2, 0.0, 0.0),
                "rotation": (0.70711, 0.0, 0.0, 0.70711),
            },
        ],
    },
}


def parse_args() -> argparse.Namespace:
    argv = None
    if os.environ.get(INSIDE_KIT_ENV_VAR) == "1":
        raw_argv = os.environ.get(INNER_ARGV_ENV_VAR)
        if raw_argv:
            argv = json.loads(raw_argv)

    parser = argparse.ArgumentParser(
        description=(
            "Launch robot_room.usd with the mobile FR3 in Isaac Sim."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--room-usd",
        type=Path,
        default="robot_room.usd",
        help=("Room USD to reference under asset folder."),
    )
    parser.add_argument(
        "--robot-usd",
        type=Path,
        default=None,
        help="Robot USD to reference. Defaults to the Franka mobile FR3 USD.",
    )
    parser.add_argument(
        "--task",
        choices=tuple(TASK_ROBOT_POSES),
        default="task3",
        help="Task preset used for the robot spawn position.",
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
        type=head_placement_arg,
        default="random",
        help=("Task3 head placement: A-I, or random. Lowercase is accepted."),
    )
    parser.add_argument(
        "--num-envs",
        "--num_envs",
        dest="num_envs",
        type=int,
        default=1,
        help="Number of Isaac Lab environments for keyboard control.",
    )
    parser.add_argument(
        "--device",
        default="cuda:0",
        help="Isaac Lab simulation device used for keyboard control.",
    )
    parser.add_argument(
        "--stabilization-steps",
        type=int,
        default=0,
        help="Initial physics steps before enabling keyboard control.",
    )
    parser.add_argument(
        "--dynamic-beans",
        action="store_true",
        help="Enable rigid-body physics for task3 beans in keyboard mode.",
    )
    keyboard_group = parser.add_mutually_exclusive_group()
    keyboard_group.add_argument(
        "--keyboard-control",
        dest="keyboard_control",
        action="store_true",
        default=None,
        help="Run the robot with live WASD/QE keyboard base control.",
    )
    keyboard_group.add_argument(
        "--no-keyboard-control",
        dest="keyboard_control",
        action="store_false",
        help="Load the scene as a passive Isaac Sim viewer.",
    )
    parser.add_argument(
        "--autoplay",
        action="store_true",
        help="Start the Isaac Sim timeline immediately after loading.",
    )
    parser.add_argument(
        "--experience",
        choices=tuple(ISAACSIM_EXPERIENCES),
        default="base",
        help="Isaac Sim Kit experience to launch.",
    )
    parser.add_argument(
        "--ros2-bridge",
        choices=("disabled", "fastdds", "cyclonedds"),
        default="disabled",
        help="Enable the Isaac Sim ROS2 bridge with the selected RMW.",
    )
    parser.add_argument(
        "--ros-distro",
        choices=("jazzy", "humble"),
        default="jazzy",
        help="Bundled ROS2 bridge distro to use when ROS2 is enabled.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Launch Isaac Sim without a GUI window.",
    )
    parser.add_argument(
        "--inside-kit",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    return parser.parse_args(argv)


def should_enable_keyboard_control(args: argparse.Namespace) -> bool:
    if args.keyboard_control is not None:
        return bool(args.keyboard_control)
    return args.task == "task3"


def robot_actuator_cfg_specs() -> dict[str, dict[str, Any]]:
    return {
        "steering_joints": {
            "joint_names_expr": ["tmrv0_2_joint_0", "tmrv0_2_joint_2"],
            "stiffness": 500.0,
            "damping": 50.0,
            "effort_limit_sim": 200.0,
        },
        "drive_joints": {
            "joint_names_expr": ["tmrv0_2_joint_1", "tmrv0_2_joint_3"],
            "stiffness": 0.0,
            "damping": 5.0,
            "effort_limit_sim": 500.0,
            "velocity_limit_sim": 20.0,
        },
        "passive_base_joints": {
            "joint_names_expr": [".*caster.*", "rocker_arm_joint"],
            "stiffness": 0.0,
            "damping": 0.0,
        },
        "spine": {
            "joint_names_expr": ["franka_spine_vertical_joint"],
            "stiffness": 5000.0,
            "damping": 500.0,
            "effort_limit_sim": 200.0,
        },
        "arms": {
            "joint_names_expr": [".*fr3v2_joint[1-7]"],
            "stiffness": 5000.0,
            "damping": 500.0,
            "effort_limit_sim": 200.0,
        },
        "grippers": {
            "joint_names_expr": [".*finger.*"],
            "stiffness": 200.0,
            "damping": 20.0,
            "effort_limit_sim": 50.0,
        },
    }


def resolve_usd_path(selection: Path | None, default_path: Path) -> Path:
    if selection is None:
        return default_path

    candidate = selection.expanduser()
    if candidate.is_absolute() or candidate.is_file():
        return candidate
    return asset_path(*candidate.parts)


def yaw_to_quat(yaw_degrees: float) -> tuple[float, float, float, float]:
    half_yaw = math.radians(yaw_degrees) * 0.5
    return (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))


def euler_xyz_to_quat(
    rotation_degrees: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    roll, pitch, yaw = (math.radians(angle) for angle in rotation_degrees)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def multiply_quats(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    left_w, left_x, left_y, left_z = left
    right_w, right_x, right_y, right_z = right
    return (
        left_w * right_w
        - left_x * right_x
        - left_y * right_y
        - left_z * right_z,
        left_w * right_x
        + left_x * right_w
        + left_y * right_z
        - left_z * right_y,
        left_w * right_y
        - left_x * right_z
        + left_y * right_w
        + left_z * right_x,
        left_w * right_z
        + left_x * right_y
        - left_y * right_x
        + left_z * right_w,
    )


def axis_angle_to_quat(
    axis: str,
    angle_degrees: float,
) -> tuple[float, float, float, float]:
    half_angle = math.radians(angle_degrees) * 0.5
    real = math.cos(half_angle)
    imaginary = math.sin(half_angle)
    if axis == "x":
        return (real, imaginary, 0.0, 0.0)
    if axis == "y":
        return (real, 0.0, imaginary, 0.0)
    return (real, 0.0, 0.0, imaginary)


def usd_rotate_xyz_to_quat(
    rotation_degrees: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    x_rotation = axis_angle_to_quat("x", rotation_degrees[0])
    y_rotation = axis_angle_to_quat("y", rotation_degrees[1])
    z_rotation = axis_angle_to_quat("z", rotation_degrees[2])
    return multiply_quats(multiply_quats(x_rotation, y_rotation), z_rotation)


def resolve_robot_position(
    args: argparse.Namespace,
) -> tuple[float, float, float]:
    preset_x, preset_y, preset_z = TASK_ROBOT_POSES[args.task]["position"]
    return (
        preset_x if args.robot_x is None else args.robot_x,
        preset_y if args.robot_y is None else args.robot_y,
        preset_z if args.robot_z is None else args.robot_z,
    )


def resolve_robot_yaw(args: argparse.Namespace) -> float:
    if args.robot_yaw is not None:
        return args.robot_yaw
    return TASK_ROBOT_POSES[args.task]["yaw"]


def normalize_head_placement_name(selection: str) -> str:
    normalized = selection.strip().upper()
    if normalized == "RANDOM":
        return "random"
    if normalized in TASK3_HEAD_PLACEMENTS:
        return normalized
    allowed = ", ".join((*TASK3_HEAD_PLACEMENTS, "random"))
    raise ValueError(f"Unknown head placement '{selection}'. Use: {allowed}")


def head_placement_arg(selection: str) -> str:
    try:
        return normalize_head_placement_name(selection)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def resolve_head_placement(
    selection: str,
) -> tuple[str, tuple[float, float, float], tuple[float, float, float, float]]:
    normalized = normalize_head_placement_name(selection)
    if normalized == "random":
        normalized = random.choice(tuple(TASK3_HEAD_PLACEMENTS))

    position, rotation_degrees = TASK3_HEAD_PLACEMENTS[normalized]
    return normalized, position, usd_rotate_xyz_to_quat(rotation_degrees)


def set_head_xform_orient(
    prim: Any,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> None:
    from pxr import Gf as pxr_gf
    from pxr import UsdGeom as pxr_usd_geom

    Gf: Any = pxr_gf
    UsdGeom: Any = pxr_usd_geom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    for rotate_attr_name in (
        "xformOp:rotateXYZ",
        "xformOp:rotateX",
        "xformOp:rotateY",
        "xformOp:rotateZ",
    ):
        rotate_attr = prim.GetAttribute(rotate_attr_name)
        if rotate_attr:
            rotate_attr.Block()

    translate_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
    translate_op.Set(Gf.Vec3d(*position))
    orient_op.Set(
        Gf.Quatf(
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3],
        )
    )
    xform.SetXformOpOrder([translate_op, orient_op], True)


def configure_ros2_bridge_env(args: argparse.Namespace) -> None:
    if args.ros2_bridge == "disabled":
        return

    bridge_lib = ROS2_BRIDGE_ROOT / args.ros_distro / "lib"
    if not bridge_lib.is_dir():
        raise FileNotFoundError(
            f"ROS2 bridge library path not found: {bridge_lib}"
        )

    rmw_by_bridge = {
        "fastdds": "rmw_fastrtps_cpp",
        "cyclonedds": "rmw_cyclonedds_cpp",
    }
    os.environ["ROS_DISTRO"] = args.ros_distro
    os.environ["RMW_IMPLEMENTATION"] = rmw_by_bridge[args.ros2_bridge]
    os.environ.setdefault("ROS_LOG_DIR", "/isaac-sim/kit/logs/ros")
    existing_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_paths = [str(bridge_lib)]
    if existing_ld_path:
        ld_paths.append(existing_ld_path)
    os.environ["LD_LIBRARY_PATH"] = ":".join(ld_paths)

    if os.environ.get(ROS2_ENV_READY_VAR) != "1":
        env = os.environ.copy()
        env[ROS2_ENV_READY_VAR] = "1"
        os.execvpe(sys.executable, [sys.executable, *sys.argv], env)


def enable_ros2_bridge(app: Any, args: argparse.Namespace) -> None:
    if args.ros2_bridge == "disabled":
        return

    import omni.kit.app

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_manager.set_extension_enabled_immediate(
        "isaacsim.ros2.bridge",
        True,
    )
    for _ in range(10):
        app.update()
    print(
        f"ROS2 bridge: {args.ros_distro} / {os.environ['RMW_IMPLEMENTATION']}"
    )


def launch_isaac_sim(args: argparse.Namespace) -> None:
    if not ISAACSIM_LAUNCHER.is_file():
        raise FileNotFoundError(
            f"Isaac Sim launcher not found: {ISAACSIM_LAUNCHER}"
        )

    configure_ros2_bridge_env(args)

    command = [
        str(ISAACSIM_LAUNCHER),
        ISAACSIM_EXPERIENCES[args.experience],
    ]
    if args.headless:
        command.append("--no-window")
    command.extend(["--exec", str(Path(__file__).resolve()), "--inside-kit"])
    env = os.environ.copy()
    env[INSIDE_KIT_ENV_VAR] = "1"
    env[INNER_ARGV_ENV_VAR] = json.dumps(
        [arg for arg in sys.argv[1:] if arg != "--inside-kit"]
    )
    os.chdir("/isaac-sim")
    os.execvpe(command[0], command, env)


def set_xform(
    prim: Any,
    position: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> None:
    from pxr import Gf as pxr_gf
    from pxr import UsdGeom as pxr_usd_geom

    Gf: Any = pxr_gf
    UsdGeom: Any = pxr_usd_geom

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*position)
    )
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(rotation[0], rotation[1], rotation[2], rotation[3])
    )


def reference_usd(
    stage: Any,
    prim_path: str,
    usd_path: Path,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    reset_asset_xform: bool = False,
) -> Any:
    from pxr import UsdGeom as pxr_usd_geom

    UsdGeom: Any = pxr_usd_geom

    parent_prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
    set_xform(parent_prim, position, rotation)

    asset_prim = UsdGeom.Xform.Define(stage, f"{prim_path}/Asset").GetPrim()
    asset_prim.GetReferences().AddReference(str(usd_path.resolve()))
    if reset_asset_xform:
        set_xform(asset_prim, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    return asset_prim


def remove_embedded_physics_scenes(stage: Any, root_prim: Any) -> list[str]:
    from pxr import Usd as pxr_usd
    from pxr import UsdPhysics as pxr_usd_physics

    Usd: Any = pxr_usd
    UsdPhysics: Any = pxr_usd_physics

    paths_to_remove = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(root_prim)
        if prim.IsA(UsdPhysics.Scene)
    ]
    for prim_path in paths_to_remove:
        stage.OverridePrim(prim_path).SetActive(False)
    return paths_to_remove


def move_task3_head(
    stage: Any,
    room_asset_path: str,
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
) -> str:
    candidate_paths = (
        f"{room_asset_path}/head",
        f"{room_asset_path}/root/head",
        "/root/head",
    )
    for prim_path in candidate_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if prim and prim.IsValid():
            set_head_xform_orient(prim, position, orientation)
            return prim_path

    raise RuntimeError(
        "Could not find task3 head prim. Tried: " + ", ".join(candidate_paths)
    )


def create_preview_material(
    stage: Any,
    path: str,
    diffuse_color: tuple[float, float, float],
    metallic: float = 0.0,
    roughness: float = 0.5,
) -> Any:
    from pxr import Gf as pxr_gf
    from pxr import Sdf as pxr_sdf
    from pxr import UsdShade as pxr_usd_shade

    Gf: Any = pxr_gf
    Sdf: Any = pxr_sdf
    UsdShade: Any = pxr_usd_shade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*diffuse_color)
    )
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    surface_output = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surface_output)
    return material


def apply_physics_material(
    material: Any,
    friction: float,
    restitution: float,
) -> None:
    from pxr import UsdPhysics as pxr_usd_physics

    UsdPhysics: Any = pxr_usd_physics

    physics_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_api.CreateStaticFrictionAttr(friction)
    physics_api.CreateDynamicFrictionAttr(friction)
    physics_api.CreateRestitutionAttr(restitution)


def usd_world_bounds(
    path: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    from pxr import Usd as pxr_usd
    from pxr import UsdGeom as pxr_usd_geom

    Usd: Any = pxr_usd
    UsdGeom: Any = pxr_usd_geom

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise ValueError(f"Could not open USD stage: {path}")

    purposes = [
        UsdGeom.Tokens.default_,
        UsdGeom.Tokens.render,
        UsdGeom.Tokens.proxy,
    ]
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    bound_range = bbox_cache.ComputeWorldBound(
        stage.GetPseudoRoot()
    ).ComputeAlignedRange()
    bound_min = bound_range.GetMin()
    bound_max = bound_range.GetMax()
    return tuple(bound_min), tuple(bound_max)


def bean_spawn_positions(
    count: int,
    bowl_position: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
    bowl_min_local, bowl_max_local = usd_world_bounds(BOWL_USD)
    container_min = tuple(
        bowl_min_local[index] + bowl_position[index] for index in range(3)
    )
    container_max = tuple(
        bowl_max_local[index] + bowl_position[index] for index in range(3)
    )
    container_center_xy = (
        0.5 * (container_min[0] + container_max[0]),
        0.5 * (container_min[1] + container_max[1]),
    )
    container_inner_radius = 0.5 * min(
        container_max[0] - container_min[0],
        container_max[1] - container_min[1],
    )
    bean_radius = BEAN_PHYSICS["radius"]
    bean_half_height = BEAN_PHYSICS["half_height"]
    bean_length = 2.0 * (bean_half_height + bean_radius)
    radial_margin = max(1.25 * bean_radius, 0.60 * bean_half_height)
    usable_radius = max(
        bean_radius,
        container_inner_radius
        - BEAN_PHYSICS["spawn_wall_thickness"]
        - radial_margin,
    )
    layer_height = max(2.4 * bean_radius, 0.9 * bean_length)
    spawn_bottom_z = bowl_position[2] + BEAN_PHYSICS["spawn_height"]
    ring_spacing = BEAN_PHYSICS["spawn_spacing_scale"] * max(
        2.8 * bean_radius,
        0.92 * bean_length,
    )
    angular_spacing = BEAN_PHYSICS["spawn_spacing_scale"] * max(
        2.6 * bean_radius,
        0.8 * bean_length,
    )

    positions = []
    layer_index = 0
    while len(positions) < count:
        z = spawn_bottom_z + layer_index * layer_height
        ring_phase = 0.5 * math.pi * (layer_index % 4)

        positions.append((container_center_xy[0], container_center_xy[1], z))
        if len(positions) >= count:
            break

        ring_radius = ring_spacing
        while ring_radius <= usable_radius and len(positions) < count:
            circumference = 2.0 * math.pi * ring_radius
            count_on_ring = max(6, int(circumference / angular_spacing))
            angle_step = 2.0 * math.pi / count_on_ring
            for ring_index in range(count_on_ring):
                angle = ring_phase + ring_index * angle_step
                radial_jitter = random.uniform(
                    -0.08 * ring_spacing,
                    0.08 * ring_spacing,
                )
                theta_jitter = random.uniform(-0.08, 0.08) * angle_step
                current_radius = min(
                    usable_radius,
                    max(bean_radius, ring_radius + radial_jitter),
                )
                x = current_radius * math.cos(angle + theta_jitter)
                y = current_radius * math.sin(angle + theta_jitter)
                if x * x + y * y > usable_radius * usable_radius:
                    continue
                positions.append(
                    (
                        container_center_xy[0] + x,
                        container_center_xy[1] + y,
                        z
                        + random.uniform(
                            -0.08 * bean_radius,
                            0.08 * bean_radius,
                        ),
                    )
                )
                if len(positions) >= count:
                    break
            ring_radius += ring_spacing
        layer_index += 1
    return positions[:count]


def add_coffee_beans(
    stage: Any,
    count: int,
    color: tuple[float, float, float],
    density: float,
    bowl_position: tuple[float, float, float],
    dynamic: bool = True,
) -> None:
    if count <= 0:
        return

    from pxr import UsdGeom as pxr_usd_geom
    from pxr import UsdPhysics as pxr_usd_physics
    from pxr import UsdShade as pxr_usd_shade

    UsdGeom: Any = pxr_usd_geom
    UsdPhysics: Any = pxr_usd_physics
    UsdShade: Any = pxr_usd_shade

    UsdGeom.Scope.Define(stage, "/World/Scene")
    UsdGeom.Scope.Define(stage, "/World/Scene/CoffeeBeans")
    UsdGeom.Scope.Define(stage, "/World/Looks")
    material = create_preview_material(
        stage,
        "/World/Looks/CoffeeBean",
        diffuse_color=color,
        metallic=0.0,
        roughness=0.8,
    )
    apply_physics_material(
        material,
        friction=BEAN_PHYSICS["friction"],
        restitution=BEAN_PHYSICS["restitution"],
    )

    radius = BEAN_PHYSICS["radius"]
    half_height = BEAN_PHYSICS["half_height"]

    positions = bean_spawn_positions(count, bowl_position)
    for index, position in enumerate(positions):
        bean_prim_path = f"/World/Scene/CoffeeBeans/Bean_{index:04d}"
        bean = UsdGeom.Capsule.Define(stage, bean_prim_path)
        bean.CreateRadiusAttr(radius)
        bean.CreateHeightAttr(2.0 * half_height)
        bean.CreateAxisAttr("X")
        bean_prim = bean.GetPrim()

        yaw = random.uniform(0.0, 2.0 * math.pi)
        set_xform(bean_prim, position, yaw_to_quat(math.degrees(yaw)))

        UsdPhysics.CollisionAPI.Apply(bean_prim)
        if dynamic:
            UsdPhysics.RigidBodyAPI.Apply(bean_prim)
            mass_api = UsdPhysics.MassAPI.Apply(bean_prim)
            mass_api.CreateDensityAttr(density)
        UsdShade.MaterialBindingAPI.Apply(bean_prim).Bind(material)


def load_deformable_assets(
    stage: Any,
) -> None:
    root_position = TASK2_TABLE_POSITION
    asset_root_path = "/World/Scene/task_objects"

    for asset_key, asset_config in TASK2_OBJECT_SPAWN_CONFIG.items():
        if asset_key in ("boards",):
            for i, board_spawn in enumerate(asset_config["spawns"]):
                reference_usd(
                    stage,
                    f"{asset_root_path}/board_{i}",
                    asset_path(asset_config["asset_path"]),
                    position=tuple(
                        root_position[index] + board_spawn["position"][index]
                        for index in range(3)
                    ),
                    rotation=board_spawn["rotation"],
                )
            continue
        reference_usd(
            stage,
            f"{asset_root_path}/{asset_key}",
            asset_path(asset_config["asset_path"]),
            position=tuple(
                root_position[index] + asset_config["position"][index]
                for index in range(3)
            ),
            rotation=asset_config["rotation"],
        )


def setup_deformable_camera(
    stage: Any,
) -> None:
    import omni.graph.core as og
    from pxr import Gf as pxr_gf
    from pxr import UsdGeom as pxr_usd_geom

    Gf: Any = pxr_gf
    UsdGeom: Any = pxr_usd_geom

    # Creating a Camera Prim
    camera_prim_path = "/World/Scene/eval_camera"
    camera_prim = UsdGeom.Camera.Define(stage, camera_prim_path)
    xform_api = UsdGeom.XformCommonAPI(camera_prim)
    xform_api.SetTranslate(Gf.Vec3d(*TASK2_CAMERA_POSITION))
    xform_api.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_prim.GetFocalLengthAttr().Set(20)
    camera_prim.GetFocusDistanceAttr().Set(400)
    camera_prim.GetProjectionAttr().Set("perspective")
    # camera_prim.GetHorizontalApertureAttr().Set(21)
    # camera_prim.GetVerticalApertureAttr().Set(16)

    # ROS2 helper
    ROS_TOPIC_NAMESPACE = "/isaac/eval_camera"
    ROS_TOPIC_FRAMEID = "eval_camera"

    keys = og.Controller.Keys
    (ros_camera_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS2_CameraGraphs/eval_camera",
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                (
                    "CameraInfoPublish",
                    "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
                ),
                (
                    "RenderProduct",
                    "isaacsim.core.nodes.IsaacCreateRenderProduct",
                ),
                (
                    "RunOnce",
                    "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame",
                ),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("SemanticPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (
                    "Bbox2dTightPublish",
                    "isaacsim.ros2.bridge.ROS2CameraHelper",
                ),
            ],
            keys.CONNECT: [
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
                (
                    "Context.outputs:context",
                    "CameraInfoPublish.inputs:context",
                ),
                ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "RGBPublish.inputs:renderProductPath",
                ),
                (
                    "RenderProduct.outputs:execOut",
                    "DepthPublish.inputs:execIn",
                ),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "DepthPublish.inputs:renderProductPath",
                ),
                (
                    "RenderProduct.outputs:execOut",
                    "SemanticPublish.inputs:execIn",
                ),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "SemanticPublish.inputs:renderProductPath",
                ),
                ("Context.outputs:context", "SemanticPublish.inputs:context"),
                (
                    "RenderProduct.outputs:execOut",
                    "Bbox2dTightPublish.inputs:execIn",
                ),
                (
                    "RenderProduct.outputs:renderProductPath",
                    "Bbox2dTightPublish.inputs:renderProductPath",
                ),
                (
                    "Context.outputs:context",
                    "Bbox2dTightPublish.inputs:context",
                ),
            ],
            keys.SET_VALUES: [
                # Render Product
                ("RenderProduct.inputs:cameraPrim", camera_prim_path),
                ("RenderProduct.inputs:height", 720),
                ("RenderProduct.inputs:width", 1280),
                # Publisher: Camera Info
                ("CameraInfoPublish.inputs:topicName", "camera_info"),
                ("CameraInfoPublish.inputs:frameId", ROS_TOPIC_FRAMEID),
                (
                    "CameraInfoPublish.inputs:nodeNamespace",
                    ROS_TOPIC_NAMESPACE,
                ),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                # Publisher: RGB
                ("RGBPublish.inputs:type", "rgb"),
                ("RGBPublish.inputs:nodeNamespace", ROS_TOPIC_NAMESPACE),
                ("RGBPublish.inputs:topicName", "image_raw"),
                ("RGBPublish.inputs:frameId", ROS_TOPIC_FRAMEID),
                ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
                # Publisher: Depth
                ("DepthPublish.inputs:type", "depth"),
                ("DepthPublish.inputs:nodeNamespace", ROS_TOPIC_NAMESPACE),
                ("DepthPublish.inputs:topicName", "depth"),
                ("DepthPublish.inputs:frameId", ROS_TOPIC_FRAMEID),
                ("DepthPublish.inputs:resetSimulationTimeOnStop", True),
                # Publisher: Semantic Segmentation
                ("SemanticPublish.inputs:topicName", "semantic_segmentation"),
                ("SemanticPublish.inputs:type", "semantic_segmentation"),
                ("SemanticPublish.inputs:frameId", ROS_TOPIC_FRAMEID),
                ("SemanticPublish.inputs:nodeNamespace", ROS_TOPIC_NAMESPACE),
                ("SemanticPublish.inputs:enableSemanticLabels", True),
                ("SemanticPublish.inputs:resetSimulationTimeOnStop", True),
                # Publisher: 2D Bounding Box Tight
                ("Bbox2dTightPublish.inputs:topicName", "bbox_2d_tight"),
                ("Bbox2dTightPublish.inputs:type", "bbox_2d_tight"),
                ("Bbox2dTightPublish.inputs:resetSimulationTimeOnStop", True),
                ("Bbox2dTightPublish.inputs:frameId", ROS_TOPIC_FRAMEID),
                (
                    "Bbox2dTightPublish.inputs:nodeNamespace",
                    ROS_TOPIC_NAMESPACE,
                ),
                ("Bbox2dTightPublish.inputs:enableSemanticLabels", True),
            ],
        },
    )


def set_initial_perspective_view(app: Any) -> None:
    if not INITIAL_VIEW_POSE:
        return

    position, rotation = INITIAL_VIEW_POSE
    rotation_quat = euler_xyz_to_quat(rotation)
    camera_path = "/OmniverseKit_Persp"

    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        from pxr import Gf as pxr_gf

        Gf: Any = pxr_gf
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = camera_path
            try:
                camera_state = ViewportCameraState(camera_path, viewport)
            except TypeError:
                camera_state = ViewportCameraState(camera_path)
            camera_state.set_position_world(Gf.Vec3d(*position), True)
            camera_state.set_rotation_world(Gf.Quatd(*rotation_quat), True)
            app.update()
            return
    except Exception as exc:
        print(f"Viewport pose API unavailable: {exc}")


def configure_robot_room_stage(
    app: Any,
    stage: Any,
    room_path: Path,
    task: str,
    head_placement: str,
    *,
    robot_path: Path | None = None,
    robot_position: tuple[float, float, float] | None = None,
    robot_rotation: tuple[float, float, float, float] | None = None,
    robot_yaw: float | None = None,
    dynamic_beans: bool = True,
) -> Any:
    from pxr import UsdGeom as pxr_usd_geom
    from pxr import UsdLux as pxr_usd_lux

    UsdGeom: Any = pxr_usd_geom
    UsdLux: Any = pxr_usd_lux

    stage.SetFramesPerSecond(60.0)
    stage.SetTimeCodesPerSecond(60.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, "/World/Environment")

    room_asset_prim = reference_usd(
        stage,
        "/World/Environment/RobotRoom",
        room_path,
        reset_asset_xform=True,
    )
    removed_physics_scenes = remove_embedded_physics_scenes(
        stage,
        room_asset_prim,
    )
    if removed_physics_scenes:
        print(
            "Disabled embedded room physics scenes: "
            + ", ".join(removed_physics_scenes),
            flush=True,
        )
    if (
        robot_path is not None
        and robot_position is not None
        and robot_rotation is not None
    ):
        reference_usd(
            stage,
            "/World/Robot",
            robot_path,
            robot_position,
            robot_rotation,
        )

    resolved_head_placement = None
    head_prim_path = None
    if task == "task1":
        pass
    elif task == "task2":
        load_deformable_assets(stage)
        setup_deformable_camera(stage)
    elif task == "task3":
        (
            resolved_head_placement,
            head_position,
            head_orientation,
        ) = resolve_head_placement(head_placement)
        head_prim_path = move_task3_head(
            stage,
            str(room_asset_prim.GetPath()),
            head_position,
            head_orientation,
        )
        add_coffee_beans(
            stage,
            count=DEFAULT_BEAN_COUNT,
            color=DEFAULT_BEAN_COLOR,
            density=DEFAULT_BEAN_DENSITY,
            bowl_position=TASK3_BOWL_POSITION,
            dynamic=dynamic_beans,
        )

    dome = UsdLux.DomeLight.Define(stage, "/World/Light")
    dome.CreateIntensityAttr(3000.0)

    for _ in range(10):
        app.update()

    set_initial_perspective_view(app)

    print("=" * 80)
    print("Robot room loaded in Isaac Sim")
    print("=" * 80)
    print(f"Room USD: {room_path}")
    if robot_path is not None:
        print(f"Robot USD: {robot_path}")
    if robot_position is not None:
        print(
            "Robot start: "
            f"({robot_position[0]:.3f}, {robot_position[1]:.3f}, "
            f"{robot_position[2]:.3f})"
        )
    if robot_yaw is not None:
        print(f"Robot yaw: {robot_yaw:.1f} deg")
    bean_mode = "dynamic" if dynamic_beans else "static"
    bean_count = DEFAULT_BEAN_COUNT if task == "task3" else 0
    print(f"Coffee beans: {bean_count} ({bean_mode})")
    if resolved_head_placement and head_prim_path:
        print(f"Head placement: {resolved_head_placement}")
        print(f"Head prim: {head_prim_path}")
    return stage


def build_stage(
    app: Any,
    room_path: Path,
    robot_path: Path,
    task: str,
    robot_position: tuple[float, float, float],
    robot_rotation: tuple[float, float, float, float],
    robot_yaw: float,
    head_placement: str,
) -> Any:
    import omni.usd

    context = omni.usd.get_context()
    context.new_stage()
    for _ in range(10):
        app.update()

    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Could not create an Isaac Sim stage.")

    return configure_robot_room_stage(
        app,
        stage,
        room_path=room_path,
        task=task,
        head_placement=head_placement,
        robot_path=robot_path,
        robot_position=robot_position,
        robot_rotation=robot_rotation,
        robot_yaw=robot_yaw,
    )


def make_robot_actuator_cfgs(implicit_actuator_cfg: Any) -> dict[str, Any]:
    return {
        name: implicit_actuator_cfg(**spec)
        for name, spec in robot_actuator_cfg_specs().items()
    }


def make_control_scene_cfg(
    *,
    num_envs: int,
    robot_path: Path,
    robot_position: tuple[float, float, float],
    robot_rotation: tuple[float, float, float, float],
) -> Any:
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.scene import InteractiveSceneCfg

    scene_cfg = InteractiveSceneCfg(num_envs=num_envs, env_spacing=10.0)
    scene_cfg.robot = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=str(robot_path)),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=robot_position,
            rot=robot_rotation,
            joint_pos=INITIAL_ROBOT_JOINT_POS,
        ),
        actuators=make_robot_actuator_cfgs(ImplicitActuatorCfg),
    )
    return scene_cfg


def disable_robot_external_wrenches(robot: Any) -> None:
    """Keep Isaac Lab from applying unused external link wrenches."""
    for composer_name in (
        "instantaneous_wrench_composer",
        "permanent_wrench_composer",
    ):
        composer = getattr(robot, composer_name, None)
        if composer is not None:
            composer.reset()


def require_single_teleop_environment(num_envs: int) -> None:
    """Lula's Core articulation wrapper controls one Task 3 robot."""
    if num_envs != 1:
        raise RuntimeError(
            "Keyboard dual-arm IK requires exactly one environment; "
            f"received num_envs={num_envs}."
        )


MOTION_GENERATION_EXTENSION = "isaacsim.robot_motion.motion_generation"


def enable_motion_generation_extension(extension_manager: Any) -> None:
    """Enable the Lula extension before importing its Python package."""
    if extension_manager.is_extension_enabled(MOTION_GENERATION_EXTENSION):
        return
    enabled = extension_manager.set_extension_enabled_immediate(
        MOTION_GENERATION_EXTENSION, True
    )
    if enabled is False:
        raise RuntimeError(
            "Could not enable Isaac Sim motion-generation extension: "
            f"{MOTION_GENERATION_EXTENSION}"
        )


def measured_position_targets(robot: Any) -> Any:
    """Snapshot measured joints once as persistent position targets."""
    return robot.data.joint_pos.detach().clone()


def robot_root_world_pose(
    robot: Any,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Read the first environment's root pose in Isaac Lab wxyz order."""
    position = robot.data.root_pos_w[0].detach().cpu().tolist()
    orientation = robot.data.root_quat_w[0].detach().cpu().tolist()
    return tuple(position), tuple(orientation)


def clamp_direct_joint_command(command: Any, robot: Any, groups: Any) -> Any:
    """Clamp present direct targets to the articulation's soft limits."""
    if (
        command.left_joint_positions is None
        and command.right_joint_positions is None
    ):
        return command
    limits = getattr(robot.data, "soft_joint_pos_limits", None)
    if limits is None:
        raise RuntimeError(
            "Direct arm commands require robot.data.soft_joint_pos_limits"
        )
    required_ids = groups.left_arm + groups.right_arm
    if (
        limits.ndim != 3
        or limits.shape[0] < 1
        or limits.shape[2] != 2
        or not required_ids
        or limits.shape[1] <= max(required_ids)
    ):
        raise RuntimeError(
            "soft_joint_pos_limits must have shape (envs, joints, 2)"
        )
    from teleop_targets import clamp_arm_joint_positions

    updates = {}
    for side, values, joint_ids in (
        ("left", command.left_joint_positions, groups.left_arm),
        ("right", command.right_joint_positions, groups.right_arm),
    ):
        if values is None:
            continue
        lower = limits[0, list(joint_ids), 0].detach().cpu().tolist()
        upper = limits[0, list(joint_ids), 1].detach().cpu().tolist()
        updates[f"{side}_joint_positions"] = clamp_arm_joint_positions(
            values, lower, upper
        )
    return replace(command, **updates)


def configure_keyboard_control_stage(
    configure: Any,
    app: Any,
    stage: Any,
    **kwargs: Any,
) -> Any:
    """Configure room props only; InteractiveScene owns the sole robot."""
    return configure(app, stage, robot_path=None, **kwargs)


def run_with_app_cleanup(app: Any, callback: Any) -> Any:
    try:
        return callback()
    finally:
        app.close()


class PynputKeyboardTeleop:
    def __init__(self, keyboard_module: Any) -> None:
        self._keyboard = keyboard_module
        self.pressed: set[str] = set()
        self.stop_requested = False
        self._listener: Any | None = None

    def start(self) -> None:
        self._listener = self._keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
            suppress=False,
        )
        self._listener.daemon = True
        self._listener.start()

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()

    def _on_press(self, key: Any) -> bool | None:
        self._update_pressed(key, add=True)
        if key == self._keyboard.Key.esc:
            self.stop_requested = True
            return False
        return None

    def _on_release(self, key: Any) -> bool | None:
        self._update_pressed(key, add=False)
        if key == self._keyboard.Key.esc:
            self.stop_requested = True
            return False
        return None

    def _update_pressed(self, key: Any, *, add: bool) -> None:
        key_name = None
        if hasattr(key, "char") and key.char:
            key_name = key.char.lower()
        elif hasattr(key, "name"):
            key_name = key.name

        if key_name is None:
            return
        if add:
            self.pressed.add(key_name)
        else:
            self.pressed.discard(key_name)


class KitKeyboardTeleop:
    def __init__(self, carb_input: Any, appwindow: Any) -> None:
        self._carb_input = carb_input
        self._keyboard = appwindow.get_default_app_window().get_keyboard()
        self._input = carb_input.acquire_input_interface()
        self._subscription: Any | None = None
        self.pressed: set[str] = set()
        self.stop_requested = False

    def start(self) -> None:
        self._subscription = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            self._on_keyboard_event,
        )

    def stop(self) -> None:
        if self._subscription is None:
            return
        unsubscribe = getattr(
            self._input,
            "unsubscribe_to_keyboard_events",
            None,
        )
        if unsubscribe is not None:
            unsubscribe(self._keyboard, self._subscription)
        self._subscription = None

    def _on_keyboard_event(self, event: Any, *_args: Any) -> bool:
        key_name = normalize_keyboard_event_input(event.input)
        if key_name is None:
            return True

        event_type = event.type
        if event_type in (
            self._carb_input.KeyboardEventType.KEY_PRESS,
            self._carb_input.KeyboardEventType.KEY_REPEAT,
        ):
            self.pressed.add(key_name)
            if key_name == "esc":
                self.stop_requested = True
        elif event_type == self._carb_input.KeyboardEventType.KEY_RELEASE:
            self.pressed.discard(key_name)
            if key_name == "esc":
                self.stop_requested = True
        return True


def normalize_keyboard_event_input(key_input: Any) -> str | None:
    raw_name = getattr(key_input, "name", None)
    if raw_name is None:
        raw_name = str(key_input).rsplit(".", maxsplit=1)[-1]
    key_name = str(raw_name).lower()
    aliases = {
        "escape": "esc",
        "left_arrow": "left",
        "right_arrow": "right",
        "arrow_left": "left",
        "arrow_right": "right",
        "key_1": "1",
        "key_2": "2",
        "key_3": "3",
    }
    return aliases.get(key_name, key_name)


def create_keyboard_teleop() -> Any:
    try:
        import carb.input
        import omni.appwindow

        return KitKeyboardTeleop(carb.input, omni.appwindow)
    except Exception:
        pass

    try:
        from pynput import keyboard

        return PynputKeyboardTeleop(keyboard)
    except ImportError:
        import carb.input
        import omni.appwindow

        return KitKeyboardTeleop(carb.input, omni.appwindow)


def print_keyboard_control_help(control_help: str, mode: Any) -> None:
    print("\n" + "=" * 80)
    print(f"Keyboard robot control enabled (mode: {mode.value})")
    print("=" * 80)
    print(control_help)
    print("  ESC     stop keyboard listener and exit")
    print("  Ctrl+C  exit")
    print(
        "Tip: the listener is global, so the viewport does not need focus.\n"
    )


def run_keyboard_control(
    args: argparse.Namespace,
    *,
    room_path: Path,
    robot_path: Path,
    robot_position: tuple[float, float, float],
    robot_rotation: tuple[float, float, float, float],
    robot_yaw: float,
) -> None:
    configure_ros2_bridge_env(args)
    require_single_teleop_environment(args.num_envs)
    try:
        from isaaclab.app import AppLauncher
    except ImportError as exc:
        raise RuntimeError(
            "Keyboard robot control requires the Isaac Lab runtime. "
            "Run this in the isaac-lab Docker profile, or pass "
            "--no-keyboard-control to use the passive Isaac Sim viewer."
        ) from exc
    app_launcher = AppLauncher({"headless": args.headless})
    simulation_app = app_launcher.app
    run_with_app_cleanup(
        simulation_app,
        lambda: _run_keyboard_control_app(
            args,
            simulation_app=simulation_app,
            room_path=room_path,
            robot_path=robot_path,
            robot_position=robot_position,
            robot_rotation=robot_rotation,
            robot_yaw=robot_yaw,
        ),
    )


def _run_keyboard_control_app(
    args: argparse.Namespace,
    *,
    simulation_app: Any,
    room_path: Path,
    robot_path: Path,
    robot_position: tuple[float, float, float],
    robot_rotation: tuple[float, float, float, float],
    robot_yaw: float,
) -> None:
    from dual_arm_lula import (
        LEFT_ARM_JOINTS,
        RIGHT_ARM_JOINTS,
        create_raw_dual_arm_lula,
    )
    from keyboard_arm_teleop import KeyboardTeleopMapper, control_help
    from teleop_commands import safe_command
    from teleop_targets import (
        CartesianTargetTracker,
        DirectJointTargetLatch,
        Pose,
        TargetLimits,
        TeleopTargets,
        compose_position_targets,
        discover_joint_groups,
        pose_base_to_world,
        pose_world_to_base,
    )
    from tmr_base_control import (
        compensate_yaw_rate,
        compute_drive_targets,
        find_drive_joint_ids,
        get_root_yaw,
    )

    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene
    from isaaclab.sim import SimulationContext

    enable_ros2_bridge(simulation_app, args)

    sim_cfg = sim_utils.SimulationCfg(
        dt=0.005,
        device=args.device,
        gravity=(0.0, 0.0, -9.81),
    )
    print("Creating SimulationContext...", flush=True)
    sim = SimulationContext(sim_cfg)
    print("SimulationContext ready.", flush=True)
    print("Configuring robot room stage...", flush=True)
    configure_keyboard_control_stage(
        configure_robot_room_stage,
        simulation_app,
        sim.stage,
        room_path=room_path,
        task=args.task,
        head_placement=args.head_placement,
        robot_position=robot_position,
        robot_yaw=robot_yaw,
        dynamic_beans=args.dynamic_beans,
    )
    print("Robot room stage configured.", flush=True)
    sim.set_camera_view(
        eye=[robot_position[0] + 3.5, robot_position[1] + 3.5, 2.5],
        target=[robot_position[0], robot_position[1], 0.5],
    )

    print("Creating Isaac Lab InteractiveScene...", flush=True)
    scene_cfg = make_control_scene_cfg(
        num_envs=args.num_envs,
        robot_path=robot_path,
        robot_position=robot_position,
        robot_rotation=robot_rotation,
    )
    scene = InteractiveScene(scene_cfg)
    print("InteractiveScene ready.", flush=True)
    print("Resetting simulation...", flush=True)
    sim.reset()
    print("Simulation reset complete.", flush=True)
    print("Resetting scene...", flush=True)
    scene.reset()
    print("Scene reset complete.", flush=True)

    robot = scene["robot"]
    print(
        f"Robot joints ({len(robot.joint_names)}): {robot.joint_names}",
        flush=True,
    )
    stabilization_steps = max(0, args.stabilization_steps)
    stabilization_targets = robot.data.default_joint_pos.clone()
    for index in range(stabilization_steps):
        robot.set_joint_position_target(stabilization_targets)
        disable_robot_external_wrenches(robot)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.cfg.dt)
        if index == 0 or (index + 1) % 50 == 0:
            print(
                f"Stabilizing robot... {index + 1}/{stabilization_steps}",
                flush=True,
            )

    print("Finding drive joint ids...", flush=True)
    steering_indices, drive_indices = find_drive_joint_ids(robot.joint_names)
    print("Drive joint ids ready.", flush=True)
    joint_groups = discover_joint_groups(robot.joint_names)
    position_targets = measured_position_targets(robot)

    print("Enabling Isaac Sim motion-generation extension...", flush=True)
    import omni.kit.app

    enable_motion_generation_extension(
        omni.kit.app.get_app().get_extension_manager()
    )
    print("Isaac Sim motion-generation extension ready.", flush=True)
    print("Creating raw Lula NumPy joint-state bridge...", flush=True)
    dual_arm_ik = create_raw_dual_arm_lula(
        robot.joint_names,
        lambda: robot.data.joint_pos[0].detach().cpu().numpy(),
    )
    root_position, root_orientation = robot_root_world_pose(robot)
    initial_spine = float(position_targets[0, joint_groups.spine[0]].item())
    left_world_values, right_world_values = (
        dual_arm_ik.current_end_effector_poses(
            root_position,
            root_orientation,
            initial_spine,
        )
    )
    left_relative = pose_world_to_base(
        Pose(tuple(left_world_values[0]), tuple(left_world_values[1])),
        root_position,
        root_orientation,
    )
    right_relative = pose_world_to_base(
        Pose(tuple(right_world_values[0]), tuple(right_world_values[1])),
        root_position,
        root_orientation,
    )
    initial_left_gripper = float(
        position_targets[0, joint_groups.left_gripper[0]].item()
    )
    initial_right_gripper = float(
        position_targets[0, joint_groups.right_gripper[0]].item()
    )
    tracker = CartesianTargetTracker(
        TeleopTargets(
            left=left_relative,
            right=right_relative,
            left_gripper=initial_left_gripper,
            right_gripper=initial_right_gripper,
            spine=initial_spine,
        ),
        limits=TargetLimits(
            position_min=(-1.5, -1.5, -0.5),
            position_max=(1.5, 1.5, 2.5),
            gripper_min=0.0,
            gripper_max=0.04,
            spine_min=0.0,
            spine_max=0.85,
        ),
    )
    mapper = KeyboardTeleopMapper()
    direct_joint_latch = DirectJointTargetLatch()
    displayed_mode = mapper.mode
    print("Dual-arm Lula controller ready.", flush=True)
    print("Reading root yaw...", flush=True)
    heading_hold_yaw = get_root_yaw(robot)
    print("Root yaw ready.", flush=True)
    print("Creating keyboard teleop backend...", flush=True)
    teleop = create_keyboard_teleop()
    print("Keyboard teleop backend ready.", flush=True)
    print(f"Active steering joints: {steering_indices}", flush=True)
    print(f"Active drive joints: {drive_indices}", flush=True)
    print_keyboard_control_help(control_help(), mapper.mode)

    count = 0
    listener_started = False
    try:
        teleop.start()
        listener_started = True
        print("Keyboard teleop listener started.", flush=True)
        while simulation_app.is_running() and not teleop.stop_requested:
            now = time.monotonic()
            command = mapper.map_keys(
                set(teleop.pressed), timestamp=now, dt=sim.cfg.dt
            )
            command = safe_command(command, now=now, timeout=0.25)
            command = clamp_direct_joint_command(command, robot, joint_groups)
            if mapper.mode is not displayed_mode:
                displayed_mode = mapper.mode
                print(f"Control mode: {displayed_mode.value}", flush=True)

            vx, vy, wz_cmd = command.base_twist
            wz, heading_hold_yaw = compensate_yaw_rate(
                robot,
                vx,
                vy,
                wz_cmd,
                heading_hold_yaw,
                manual_rotation=abs(wz_cmd) > 1.0e-4,
            )

            targets = tracker.apply(command)
            root_position, root_orientation = robot_root_world_pose(robot)
            left_world = pose_base_to_world(
                targets.left, root_position, root_orientation
            )
            right_world = pose_base_to_world(
                targets.right, root_position, root_orientation
            )
            ik_result = dual_arm_ik.solve(
                left_world.position,
                right_world.position,
                left_world.orientation_wxyz,
                right_world.orientation_wxyz,
                spine_position=targets.spine,
                base_position=root_position,
                base_orientation_wxyz=root_orientation,
            )
            left_arm_targets, right_arm_targets = direct_joint_latch.select(
                command,
                ik_result,
                LEFT_ARM_JOINTS,
                RIGHT_ARM_JOINTS,
            )
            position_targets = compose_position_targets(
                position_targets,
                joint_groups,
                left_arm=left_arm_targets,
                right_arm=right_arm_targets,
                left_gripper=targets.left_gripper,
                right_gripper=targets.right_gripper,
                spine=targets.spine,
            )
            robot.set_joint_position_target(position_targets)

            steering_pos_targets, drive_vel_targets = compute_drive_targets(
                robot,
                steering_indices,
                vx,
                vy,
                wz,
                num_envs=args.num_envs,
                device=sim.device,
            )
            robot.set_joint_position_target(
                steering_pos_targets,
                joint_ids=steering_indices,
            )
            robot.set_joint_velocity_target(
                drive_vel_targets,
                joint_ids=drive_indices,
            )

            disable_robot_external_wrenches(robot)
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.cfg.dt)

            count += 1
            if count % 400 == 0 and (vx != 0.0 or vy != 0.0 or wz != 0.0):
                print(
                    f"step={count} vx={vx:+.2f} vy={vy:+.2f} "
                    f"wz={wz:+.2f} keys={sorted(teleop.pressed)}"
                )
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if listener_started:
            teleop.stop()


def main() -> None:
    args = parse_args()
    room_path = resolve_usd_path(
        args.room_usd,
        asset_path("robot_room.usd"),
    )
    robot_path = resolve_usd_path(
        args.robot_usd,
        franka_urdf_path("mobile_fr3_duo_v0_2_franka_hand.usd"),
    )

    if not room_path.is_file():
        raise FileNotFoundError(f"Room USD not found: {room_path}")
    if not robot_path.is_file():
        raise FileNotFoundError(f"Robot USD not found: {robot_path}")

    robot_position = resolve_robot_position(args)
    robot_yaw = resolve_robot_yaw(args)
    robot_rotation = yaw_to_quat(robot_yaw)

    if should_enable_keyboard_control(args):
        run_keyboard_control(
            args,
            room_path=room_path,
            robot_path=robot_path,
            robot_position=robot_position,
            robot_rotation=robot_rotation,
            robot_yaw=robot_yaw,
        )
        return

    if not args.inside_kit and os.environ.get(INSIDE_KIT_ENV_VAR) != "1":
        launch_isaac_sim(args)
        return

    import omni.kit.app

    app = omni.kit.app.get_app()
    try:
        build_stage(
            app,
            room_path=room_path,
            robot_path=robot_path,
            task=args.task,
            robot_position=robot_position,
            robot_rotation=robot_rotation,
            robot_yaw=robot_yaw,
            head_placement=args.head_placement,
        )
        enable_ros2_bridge(app, args)

        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if args.autoplay:
            timeline.play()
            print("Timeline: playing")
        else:
            timeline.stop()
            print(
                "Timeline: paused. Click Play in the Isaac Sim GUI to start."
            )
        print("Close the Isaac Sim GUI window to exit.")
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
