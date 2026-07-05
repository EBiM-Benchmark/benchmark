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
    from pxr import UsdGeom as pxr_usd_geom
    from pxr import UsdLux as pxr_usd_lux

    UsdGeom: Any = pxr_usd_geom
    UsdLux: Any = pxr_usd_lux

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
    UsdGeom.Scope.Define(stage, "/World/Environment")

    room_asset_prim = reference_usd(
        stage,
        "/World/Environment/RobotRoom",
        room_path,
        reset_asset_xform=True,
    )
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
    print(f"Robot USD: {robot_path}")
    print(
        "Robot start: "
        f"({robot_position[0]:.3f}, {robot_position[1]:.3f}, "
        f"{robot_position[2]:.3f})"
    )
    print(f"Robot yaw: {robot_yaw:.1f} deg")
    print(f"Coffee beans: {DEFAULT_BEAN_COUNT if task == 'task3' else 0}")
    if resolved_head_placement and head_prim_path:
        print(f"Head placement: {resolved_head_placement}")
        print(f"Head prim: {head_prim_path}")
    return stage


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
            robot_rotation=yaw_to_quat(robot_yaw),
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
