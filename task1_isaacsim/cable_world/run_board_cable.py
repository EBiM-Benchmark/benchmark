# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# Keep this before importing pxr/newton, matching the SRA runtime workaround.
os.environ.setdefault("PXR_WORK_THREAD_LIMIT", "1")

import numpy as np
import warp as wp
import yaml
from pxr import Usd, UsdGeom

import newton
import newton.examples


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_USD_PATH = PACKAGE_DIR / "assets" / "cable.usda"
DEFAULT_BOARD_USD_PATH = PACKAGE_DIR / "assets" / "table_board_fixture" / "table_board_fixture.usd"
DEFAULT_CURVE_PRIM_PATH = "/cable/curve_0"
DEFAULT_BOARD_ROOT_PATH = "/World"
DEFAULT_CONFIG_PATH = PACKAGE_DIR / "configs" / "table_board_fixture_cable.yaml"
DEFAULT_GRIPPER_CONFIG_PATH = PACKAGE_DIR / "configs" / "gripper.yaml"

sys.path.insert(0, str(PACKAGE_DIR))

from usd_cable_curve_import import add_cable_from_usd_curve  # noqa: E402
from sra_gripper import (  # noqa: E402
    FingerConfig,
    GapConfig,
    GraspPoseBindConfig,
    GraspPoseBindController,
    GravityCompensationConfig,
    GripperControlConfig,
    GripperTeleopConfig,
    PoseConfig,
    RobotiqFingerPoseBindController,
    SraGripperConfig,
    SraGripperController,
    _euler_xyz_deg_to_quat,
    add_gripper_from_config,
    is_gripper_teleop_modifier_down,
    load_gripper_config,
)


@dataclass(frozen=True)
class BoardFloorConstraint:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    floor_z: float
    zero_downward_velocity: bool


@dataclass(frozen=True)
class BoardSupportPlane:
    shape_count: int
    center: tuple[float, float, float]
    width: float
    length: float
    height: float
    infinite: bool


_MISSING = object()


def _load_yaml_mapping(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"Config file '{config_path}' must contain a YAML mapping at the root.")
    return data


def _config_base_dir(config_path: Path) -> Path:
    if config_path.parent.name == "configs":
        return config_path.parent.parent
    return config_path.parent


def _resolve_cli_path(path_value: str | Path) -> Path:
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def _resolve_config_path(path_value: str | Path, config_base_dir: Path) -> Path:
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        path = config_base_dir / path
    return path.resolve()


def _nested_config_value(data: dict[str, Any], key_path: tuple[str, ...]) -> Any:
    current: Any = data
    for key in key_path:
        if not isinstance(current, dict) or key not in current:
            return _MISSING
        current = current[key]
    return current


def _first_config_value(
    data: dict[str, Any],
    key_paths: tuple[tuple[str, ...], ...],
    default: Any,
) -> Any:
    for key_path in key_paths:
        value = _nested_config_value(data, key_path)
        if value is not _MISSING and value is not None:
            return value
    return default


def _float_tuple(value: Any, length: int, key_path: str) -> tuple[float, ...]:
    if not isinstance(value, list | tuple):
        raise ValueError(f"Config key '{key_path}' must be a {length}-element sequence.")
    if len(value) != length:
        raise ValueError(f"Config key '{key_path}' must contain exactly {length} values.")
    out: list[float] = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, int | float):
            raise ValueError(f"Config key '{key_path}' must contain only numeric values.")
        out.append(float(item))
    return tuple(out)


def _str_tuple(value: Any, key_path: str) -> tuple[str, ...]:
    if isinstance(value, str):
        if value == "":
            raise ValueError(f"Config key '{key_path}' must not be empty.")
        return (value,)
    if not isinstance(value, list | tuple):
        raise ValueError(f"Config key '{key_path}' must be a string or sequence of strings.")
    if len(value) == 0:
        raise ValueError(f"Config key '{key_path}' must contain at least one path.")
    out: list[str] = []
    for item in value:
        if not isinstance(item, str) or item == "":
            raise ValueError(f"Config key '{key_path}' must contain only non-empty strings.")
        out.append(item)
    return tuple(out)


def _curve_prim_path_from_config(config_data: dict[str, Any]) -> str:
    curve_prim_path = _first_config_value(config_data, (("curve_prim_path",),), None)
    if curve_prim_path is not None:
        return str(curve_prim_path)

    curve_prim_paths = _first_config_value(config_data, (("curve_prim_paths",),), None)
    if curve_prim_paths is None:
        return DEFAULT_CURVE_PRIM_PATH
    if not isinstance(curve_prim_paths, list | tuple) or len(curve_prim_paths) == 0:
        raise ValueError("Config key 'curve_prim_paths' must contain at least one prim path.")
    first_path = curve_prim_paths[0]
    if not isinstance(first_path, str) or first_path == "":
        raise ValueError("Config key 'curve_prim_paths[0]' must be a non-empty string.")
    return first_path


def _load_runtime_configs() -> tuple[Path, dict[str, Any], Path, SraGripperConfig]:
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config-path", type=Path, default=DEFAULT_CONFIG_PATH)
    pre_parser.add_argument("--gripper-config-path", type=Path, default=None)
    pre_args, _ = pre_parser.parse_known_args()

    config_path = _resolve_cli_path(pre_args.config_path)
    if not config_path.is_file():
        raise FileNotFoundError(f"Missing board cable config YAML: {config_path}")
    config_data = _load_yaml_mapping(config_path)
    config_base = _config_base_dir(config_path)

    if pre_args.gripper_config_path is not None:
        gripper_config_path = _resolve_cli_path(pre_args.gripper_config_path)
    else:
        raw_gripper_config_path = _first_config_value(
            config_data,
            (("gripper_config_path",),),
            DEFAULT_GRIPPER_CONFIG_PATH,
        )
        gripper_config_path = _resolve_config_path(raw_gripper_config_path, config_base)
    gripper_config = load_gripper_config(gripper_config_path)

    return config_path, config_data, gripper_config_path, gripper_config


def _open_curve_stage(usd_path: Path, curve_prim_path: str) -> tuple[Usd.Stage, Usd.Prim]:
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {usd_path}")
    curve_prim = stage.GetPrimAtPath(curve_prim_path)
    if not curve_prim or not curve_prim.IsValid():
        raise ValueError(f"Curve prim '{curve_prim_path}' is not valid in stage '{usd_path}'.")
    if curve_prim.GetTypeName() != "BasisCurves":
        raise ValueError(f"Prim '{curve_prim_path}' is type '{curve_prim.GetTypeName()}', expected 'BasisCurves'.")
    return stage, curve_prim


def _curve_scalar_attr(curve_prim: Usd.Prim, attr_name: str, fallback: float) -> float:
    attr = curve_prim.GetAttribute(attr_name)
    if not attr:
        return float(fallback)
    value: Any = attr.Get()
    if value is None:
        return float(fallback)
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size == 0:
        return float(fallback)
    return float(arr[0])


def _override_or_curve_attr(
    override: float | None,
    curve_prim: Usd.Prim,
    attr_name: str,
    fallback: float,
) -> float:
    if override is not None:
        return float(override)
    return _curve_scalar_attr(curve_prim, attr_name, fallback)


def _default_gripper_position(points_m: np.ndarray) -> tuple[float, float, float]:
    if points_m.size == 0:
        return (0.5, 0.735, 0.12)
    center = np.mean(points_m, axis=0)
    max_z = float(np.max(points_m[:, 2]))
    return (float(center[0]), float(center[1]), max_z + 0.09)


def _compute_world_bbox(stage: Usd.Stage, prim_path: str) -> tuple[np.ndarray, np.ndarray]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise ValueError(f"Board bbox prim '{prim_path}' is not valid in stage '{stage.GetRootLayer().identifier}'.")
    purposes = [
        UsdGeom.Tokens.default_,
        UsdGeom.Tokens.render,
        UsdGeom.Tokens.proxy,
        UsdGeom.Tokens.guide,
    ]
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, useExtentsHint=True).ComputeWorldBound(prim)
    aligned = bbox.ComputeAlignedBox()
    if aligned.IsEmpty():
        raise ValueError(f"Board bbox prim '{prim_path}' produced an empty bounds.")
    min_pt = np.asarray(aligned.GetMin(), dtype=np.float64)
    max_pt = np.asarray(aligned.GetMax(), dtype=np.float64)
    return min_pt, max_pt


def _add_board_top_support_collision(
    builder: newton.ModelBuilder,
    board_usd_path: Path,
    args: argparse.Namespace,
    shape_ke: float,
    shape_kd: float,
) -> int:
    if not args.board_top_support_collision:
        return 0

    stage = Usd.Stage.Open(str(board_usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open board USD for top support bounds: {board_usd_path}")

    min_pt, max_pt = _compute_world_bbox(stage, str(args.board_top_support_prim_path))
    thickness = float(args.board_top_support_thickness)
    expand = float(args.board_top_support_expand)
    top_z = float(max_pt[2]) + float(args.board_top_support_offset)
    center = (
        0.5 * (float(min_pt[0]) + float(max_pt[0])),
        0.5 * (float(min_pt[1]) + float(max_pt[1])),
        top_z - 0.5 * thickness,
    )
    half_extents = (
        0.5 * max(float(max_pt[0] - min_pt[0]), 0.0) + expand,
        0.5 * max(float(max_pt[1] - min_pt[1]), 0.0) + expand,
        0.5 * thickness,
    )

    support_cfg = builder.default_shape_cfg.copy()
    support_cfg.density = 0.0
    support_cfg.ke = float(shape_ke)
    support_cfg.kd = float(shape_kd)
    support_cfg.mu = float(args.board_friction)
    support_cfg.margin = float(args.contact_margin)
    support_cfg.gap = float(args.rigid_gap)
    support_cfg.has_shape_collision = True
    support_cfg.has_particle_collision = False
    support_cfg.is_visible = bool(args.board_top_support_visible)

    builder.add_shape_box(
        body=-1,
        xform=wp.transform(wp.vec3(*center), wp.quat_identity()),
        hx=half_extents[0],
        hy=half_extents[1],
        hz=half_extents[2],
        cfg=support_cfg,
        label="board_top_support_collision",
    )
    print(
        "[board_cable] board top support collider added "
        f"center={center} half_extents={half_extents} top_z={top_z:.6g}"
    )
    return 1


def _make_board_floor_constraint(
    board_usd_path: Path,
    args: argparse.Namespace,
    cable_radius_m: float,
) -> BoardFloorConstraint | None:
    if not args.board_floor_constraint:
        return None

    stage = Usd.Stage.Open(str(board_usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open board USD for floor bounds: {board_usd_path}")

    min_pt, max_pt = _compute_world_bbox(stage, str(args.board_floor_prim_path))
    expand = float(args.board_floor_expand)
    floor_z = float(max_pt[2]) + float(cable_radius_m) + float(args.board_floor_clearance)
    constraint = BoardFloorConstraint(
        x_min=float(min_pt[0]) - expand,
        x_max=float(max_pt[0]) + expand,
        y_min=float(min_pt[1]) - expand,
        y_max=float(max_pt[1]) + expand,
        floor_z=floor_z,
        zero_downward_velocity=bool(args.board_floor_zero_downward_velocity),
    )
    print(
        "[board_cable] board floor constraint added "
        f"x=({constraint.x_min:.6g}, {constraint.x_max:.6g}) "
        f"y=({constraint.y_min:.6g}, {constraint.y_max:.6g}) "
        f"floor_z={constraint.floor_z:.6g}"
    )
    return constraint


def _add_board_support_plane(
    builder: newton.ModelBuilder,
    board_usd_path: Path,
    args: argparse.Namespace,
    shape_ke: float,
    shape_kd: float,
    cable_radius_m: float,
) -> BoardSupportPlane | None:
    if not args.board_support_plane:
        return None

    stage = Usd.Stage.Open(str(board_usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open board USD for support plane bounds: {board_usd_path}")

    prim_paths = tuple(str(v) for v in (args.board_support_plane_prim_paths or (args.board_support_plane_prim_path,)))
    expand = float(args.board_support_plane_expand)
    infinite = bool(args.board_support_plane_infinite)
    shape_count = 0
    last_center = (0.0, 0.0, 0.0)
    last_width = 0.0
    last_length = 0.0
    last_height = 0.0
    for prim_path in prim_paths:
        min_pt, max_pt = _compute_world_bbox(stage, prim_path)
        height = float(max_pt[2]) + float(cable_radius_m) + float(args.board_support_plane_clearance)
        center = (
            0.5 * (float(min_pt[0]) + float(max_pt[0])),
            0.5 * (float(min_pt[1]) + float(max_pt[1])),
            height,
        )
        width = 0.0 if infinite else max(float(max_pt[0] - min_pt[0]) + 2.0 * expand, 0.0)
        length = 0.0 if infinite else max(float(max_pt[1] - min_pt[1]) + 2.0 * expand, 0.0)

        plane_cfg = builder.default_shape_cfg.copy()
        plane_cfg.density = 0.0
        plane_cfg.ke = float(shape_ke)
        plane_cfg.kd = float(shape_kd)
        plane_cfg.mu = float(args.board_friction)
        plane_cfg.margin = float(args.contact_margin)
        plane_cfg.gap = float(args.rigid_gap)
        plane_cfg.has_shape_collision = True
        plane_cfg.has_particle_collision = False
        plane_cfg.is_visible = bool(args.board_support_plane_visible)

        builder.add_shape_plane(
            body=-1,
            plane=(0.0, 0.0, 1.0, -height) if infinite else None,
            xform=None if infinite else wp.transform(wp.vec3(*center), wp.quat_identity()),
            width=width,
            length=length,
            cfg=plane_cfg,
            label=f"board_support_plane:{prim_path}",
            color=(0.1, 0.25, 0.9),
        )
        print(
            "[board_cable] board support plane added "
            f"prim={prim_path} center={center} width={width:.6g} length={length:.6g} "
            f"height={height:.6g} infinite={infinite}"
        )
        shape_count += 1
        last_center = center
        last_width = width
        last_length = length
        last_height = height
    return BoardSupportPlane(
        shape_count=shape_count,
        center=last_center,
        width=last_width,
        length=last_length,
        height=last_height,
        infinite=infinite,
    )


def _apply_cable_nearby_self_collision_filter(
    builder: newton.ModelBuilder,
    cable_body_ids: list[int],
    neighbor_hops: int,
) -> int:
    if neighbor_hops <= 0:
        return 0

    filtered_pairs = 0
    body_to_shapes = builder.body_shapes
    for i, body_a in enumerate(cable_body_ids):
        shape_a_ids = tuple(int(shape_id) for shape_id in body_to_shapes.get(int(body_a), ()))
        if len(shape_a_ids) == 0:
            continue
        max_j = min(len(cable_body_ids), i + int(neighbor_hops) + 1)
        for j in range(i + 1, max_j):
            body_b = int(cable_body_ids[j])
            shape_b_ids = tuple(int(shape_id) for shape_id in body_to_shapes.get(body_b, ()))
            for shape_a in shape_a_ids:
                for shape_b in shape_b_ids:
                    builder.add_shape_collision_filter_pair(shape_a, shape_b)
                    filtered_pairs += 1
    return filtered_pairs


def _make_gripper_config(args: argparse.Namespace, points_m: np.ndarray) -> SraGripperConfig:
    position = tuple(float(v) for v in (args.gripper_position or _default_gripper_position(points_m)))
    rotation_deg = tuple(float(v) for v in args.gripper_rotation_euler_xyz_deg)
    modifier = str(args.gripper_teleop_modifier)
    linear_speed_xy = args.gripper_linear_speed_xy
    linear_speed_z = args.gripper_linear_speed_z
    if linear_speed_xy is None:
        linear_speed_xy = args.gripper_linear_speed
    if linear_speed_z is None:
        linear_speed_z = args.gripper_linear_speed
    return SraGripperConfig(
        enabled=bool(args.gripper),
        label=str(args.gripper_label),
        profile="franka",
        asset_variant=str(args.gripper_asset_variant),
        pose=PoseConfig(
            position_m=position,  # type: ignore[arg-type]
            rotation_euler_xyz_deg=rotation_deg,  # type: ignore[arg-type]
            rotation=_euler_xyz_deg_to_quat(rotation_deg),  # type: ignore[arg-type]
        ),
        finger=FingerConfig(
            density=float(args.gripper_finger_density),
            friction=float(args.gripper_finger_friction),
        ),
        gap=GapConfig(
            initial_m=float(args.gripper_initial_gap),
            target_m=float(args.gripper_target_gap),
            min_m=float(args.gripper_min_gap),
            max_m=float(args.gripper_max_gap),
        ),
        control=GripperControlConfig(
            mode="position",
            drive_force=float(args.gripper_drive_force),
            stiffness=float(args.gripper_stiffness),
            damping=float(args.gripper_damping),
        ),
        teleop=GripperTeleopConfig(
            enabled=bool(args.gripper_teleop),
            frame=str(args.gripper_teleop_frame),
            linear_speed_mps=float(args.gripper_linear_speed),
            linear_speed_xy_mps=float(linear_speed_xy),
            linear_speed_z_mps=float(linear_speed_z),
            angular_speed_radps=math.radians(float(args.gripper_angular_speed_deg)),
            gap_speed_mps=float(args.gripper_gap_speed),
            require_ctrl=modifier == "ctrl",
            modifier=modifier,
        ),
        gravity_compensation=GravityCompensationConfig(
            enabled=bool(args.gripper_gravity_compensation),
            bodies="fingers",
        ),
    )


def _make_parser(
    config_path: Path,
    config_data: dict[str, Any],
    gripper_config_path: Path,
    gripper_config: SraGripperConfig,
) -> argparse.ArgumentParser:
    config_base = _config_base_dir(config_path)
    gravity_default = _float_tuple(
        _first_config_value(config_data, (("simulation", "gravity"),), (0.0, 0.0, -9.81)),
        3,
        "simulation.gravity",
    )
    usd_path_default = _resolve_config_path(
        _first_config_value(config_data, (("cable_usd_path",), ("usd_path",)), DEFAULT_USD_PATH),
        config_base,
    )
    board_usd_path_default = _resolve_config_path(
        _first_config_value(config_data, (("scene_usd_path",), ("board_usd_path",)), DEFAULT_BOARD_USD_PATH),
        config_base,
    )
    board_support_plane_prim_paths = _first_config_value(
        config_data,
        (("board", "support_plane", "prim_paths"),),
        None,
    )
    board_support_plane_prim_paths_default = (
        _str_tuple(board_support_plane_prim_paths, "board.support_plane.prim_paths")
        if board_support_plane_prim_paths is not None
        else None
    )

    parser = newton.examples.create_parser()
    parser.add_argument("--config-path", type=Path, default=config_path)
    parser.add_argument("--gripper-config-path", type=Path, default=gripper_config_path)
    parser.add_argument("--usd-path", type=Path, default=usd_path_default)
    parser.add_argument("--curve-prim-path", default=_curve_prim_path_from_config(config_data))
    parser.add_argument("--board-usd-path", type=Path, default=board_usd_path_default)
    parser.add_argument(
        "--board-root-path",
        default=_first_config_value(config_data, (("board_root_path",), ("scene", "root_path")), DEFAULT_BOARD_ROOT_PATH),
    )
    parser.add_argument(
        "--load-board",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "load"), ("scene", "load_board")), True)),
    )
    parser.add_argument(
        "--board-load-visual-shapes",
        action=argparse.BooleanOptionalAction,
        default=bool(
            _first_config_value(
                config_data,
                (("board", "load_visual_shapes"), ("scene", "load_visual_shapes")),
                True,
            )
        ),
    )
    parser.add_argument(
        "--board-hide-collision-shapes",
        action=argparse.BooleanOptionalAction,
        default=bool(
            _first_config_value(
                config_data,
                (("board", "hide_collision_shapes"), ("scene", "hide_collision_shapes")),
                True,
            )
        ),
    )
    parser.add_argument(
        "--board-visual-shapes-as-colliders",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "visual_shapes_as_colliders"),), False)),
    )
    parser.add_argument(
        "--board-friction",
        type=float,
        default=float(_first_config_value(config_data, (("board", "friction"),), 3.0)),
    )
    parser.add_argument(
        "--board-support-plane",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "support_plane", "enabled"),), False)),
    )
    parser.add_argument(
        "--board-support-plane-infinite",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "support_plane", "infinite"),), True)),
    )
    parser.add_argument(
        "--board-support-plane-visible",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "support_plane", "visible"),), False)),
    )
    parser.add_argument(
        "--board-support-plane-prim-path",
        default=_first_config_value(
            config_data,
            (("board", "support_plane", "prim_path"),),
            "/World/Table",
        ),
    )
    parser.add_argument("--board-support-plane-prim-paths", nargs="+", default=board_support_plane_prim_paths_default)
    parser.add_argument(
        "--board-support-plane-clearance",
        type=float,
        default=float(_first_config_value(config_data, (("board", "support_plane", "clearance"),), 0.0002)),
    )
    parser.add_argument(
        "--board-support-plane-expand",
        type=float,
        default=float(_first_config_value(config_data, (("board", "support_plane", "expand"),), 0.0)),
    )
    parser.add_argument(
        "--board-top-support-collision",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "top_support", "enabled"),), False)),
    )
    parser.add_argument(
        "--board-top-support-visible",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "top_support", "visible"),), False)),
    )
    parser.add_argument(
        "--board-top-support-prim-path",
        default=_first_config_value(
            config_data,
            (("board", "top_support", "prim_path"),),
            "/World/board_segment/board_segment_bottom_left/Collisions/Collisions",
        ),
    )
    parser.add_argument(
        "--board-top-support-thickness",
        type=float,
        default=float(_first_config_value(config_data, (("board", "top_support", "thickness"),), 0.004)),
    )
    parser.add_argument(
        "--board-top-support-offset",
        type=float,
        default=float(_first_config_value(config_data, (("board", "top_support", "offset"),), 0.001)),
    )
    parser.add_argument(
        "--board-top-support-expand",
        type=float,
        default=float(_first_config_value(config_data, (("board", "top_support", "expand"),), 0.02)),
    )
    parser.add_argument(
        "--board-floor-constraint",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "floor_constraint", "enabled"),), False)),
    )
    parser.add_argument(
        "--board-floor-prim-path",
        default=_first_config_value(
            config_data,
            (("board", "floor_constraint", "prim_path"),),
            "/World/board_segment/board_segment_bottom_left/Collisions/Collisions",
        ),
    )
    parser.add_argument(
        "--board-floor-clearance",
        type=float,
        default=float(_first_config_value(config_data, (("board", "floor_constraint", "clearance"),), 0.0002)),
    )
    parser.add_argument(
        "--board-floor-expand",
        type=float,
        default=float(_first_config_value(config_data, (("board", "floor_constraint", "expand"),), 0.0)),
    )
    parser.add_argument(
        "--board-floor-zero-downward-velocity",
        action=argparse.BooleanOptionalAction,
        default=bool(
            _first_config_value(config_data, (("board", "floor_constraint", "zero_downward_velocity"),), True)
        ),
    )
    parser.add_argument(
        "--require-board-collision",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "require_collision"),), False)),
    )
    parser.add_argument(
        "--require-board-convex-collision",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("board", "require_convex_collision"),), False)),
    )
    parser.add_argument(
        "--label",
        default=str(_first_config_value(config_data, (("label",), ("asset_key",)), "board_cable")),
    )
    parser.add_argument("--gripper", action=argparse.BooleanOptionalAction, default=bool(gripper_config.enabled))
    parser.add_argument(
        "--proxy-gripper",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Add a lightweight kinematic box collider that can be driven from ROS gripper pose topics.",
    )
    parser.add_argument(
        "--proxy-gripper-size",
        type=float,
        nargs=3,
        default=(0.08, 0.025, 0.025),
        help="Proxy gripper box size in meters: x y z.",
    )
    parser.add_argument(
        "--proxy-gripper-friction",
        type=float,
        default=50.0,
        help="Friction coefficient for the proxy gripper collider.",
    )
    parser.add_argument("--gripper-label", default=gripper_config.label)
    parser.add_argument("--gripper-asset-variant", choices=("white", "black"), default=gripper_config.asset_variant)
    parser.add_argument("--gripper-position", type=float, nargs=3, default=gripper_config.pose.position_m)
    parser.add_argument(
        "--gripper-rotation-euler-xyz-deg",
        type=float,
        nargs=3,
        default=gripper_config.pose.rotation_euler_xyz_deg,
    )
    parser.add_argument("--gripper-initial-gap", type=float, default=gripper_config.gap.initial_m)
    parser.add_argument("--gripper-target-gap", type=float, default=gripper_config.gap.target_m)
    parser.add_argument("--gripper-min-gap", type=float, default=gripper_config.gap.min_m)
    parser.add_argument("--gripper-max-gap", type=float, default=gripper_config.gap.max_m)
    parser.add_argument("--gripper-finger-density", type=float, default=gripper_config.finger.density)
    parser.add_argument("--gripper-finger-friction", type=float, default=gripper_config.finger.friction)
    parser.add_argument("--gripper-drive-force", type=float, default=gripper_config.control.drive_force)
    parser.add_argument("--gripper-stiffness", type=float, default=gripper_config.control.stiffness)
    parser.add_argument("--gripper-damping", type=float, default=gripper_config.control.damping)
    parser.add_argument("--gripper-teleop", action=argparse.BooleanOptionalAction, default=gripper_config.teleop.enabled)
    parser.add_argument("--gripper-teleop-frame", choices=("world", "eeframe"), default=gripper_config.teleop.frame)
    parser.add_argument(
        "--gripper-teleop-modifier",
        choices=("ctrl", "shift", "none"),
        default=gripper_config.teleop.modifier,
    )
    parser.add_argument("--gripper-linear-speed", type=float, default=gripper_config.teleop.linear_speed_mps)
    parser.add_argument("--gripper-linear-speed-xy", type=float, default=gripper_config.teleop.linear_speed_xy_mps)
    parser.add_argument("--gripper-linear-speed-z", type=float, default=gripper_config.teleop.linear_speed_z_mps)
    parser.add_argument(
        "--gripper-angular-speed-deg",
        type=float,
        default=math.degrees(gripper_config.teleop.angular_speed_radps),
    )
    parser.add_argument("--gripper-gap-speed", type=float, default=gripper_config.teleop.gap_speed_mps)
    parser.add_argument(
        "--gripper-grasp-bind",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("gripper", "grasp_bind", "enabled"),), True)),
        help="Bind a bilaterally pinched cable body to the gripper root until the gripper opens.",
    )
    parser.add_argument(
        "--gripper-grasp-bind-confirm-steps",
        type=int,
        default=int(_first_config_value(config_data, (("gripper", "grasp_bind", "confirm_steps"),), 2)),
        help="Number of consecutive bilateral contact frames required before binding.",
    )
    parser.add_argument(
        "--gripper-grasp-bind-release-gap",
        type=float,
        default=float(_first_config_value(config_data, (("gripper", "grasp_bind", "release_gap"),), 0.065)),
        help="Release the bound cable body when commanded gripper gap exceeds this value in meters.",
    )
    parser.add_argument(
        "--gripper-grasp-bind-normal-alignment",
        type=float,
        default=float(_first_config_value(config_data, (("gripper", "grasp_bind", "normal_alignment_min_cos"),), 0.15)),
        help="Minimum contact-normal alignment cosine for each finger during bind detection.",
    )
    parser.add_argument(
        "--gripper-grasp-bind-opposing-normal",
        type=float,
        default=float(_first_config_value(config_data, (("gripper", "grasp_bind", "opposing_normal_min_cos"),), 0.1)),
        help="Minimum opposing-normal cosine between left and right finger contacts.",
    )
    parser.add_argument(
        "--gripper-grasp-bind-max-position-error",
        type=float,
        default=float(_first_config_value(config_data, (("gripper", "grasp_bind", "max_position_error"),), 0.08)),
        help="Release if bound cable body drifts farther than this from its target pose.",
    )
    parser.add_argument(
        "--gripper-grasp-bind-max-rotation-error",
        type=float,
        default=float(_first_config_value(config_data, (("gripper", "grasp_bind", "max_rotation_error"),), 3.14159)),
        help="Release if bound cable body rotates farther than this from its target pose.",
    )
    parser.add_argument(
        "--gripper-grasp-bind-activation-radius",
        type=float,
        default=float(_first_config_value(config_data, (("gripper", "grasp_bind", "activation_radius"),), 0.12)),
        help="Only search cable bodies within this distance from the gripper; 0 disables radius filtering.",
    )
    parser.add_argument(
        "--gripper-gravity-compensation",
        action=argparse.BooleanOptionalAction,
        default=gripper_config.gravity_compensation.enabled,
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=int(_first_config_value(config_data, (("simulation", "fps"),), 60)),
    )
    parser.add_argument(
        "--substeps",
        type=int,
        default=int(_first_config_value(config_data, (("simulation", "substeps"),), 5)),
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=int(_first_config_value(config_data, (("simulation", "iterations"),), 32)),
    )
    parser.add_argument("--gravity", type=float, nargs=3, default=gravity_default)
    parser.add_argument("--gravity-z", type=float, default=None)
    parser.add_argument(
        "--density",
        type=float,
        default=float(_first_config_value(config_data, (("shape", "density"),), 1000.0)),
    )
    parser.add_argument(
        "--shape-ke",
        type=float,
        default=float(_first_config_value(config_data, (("shape", "ke"),), 5.0e3)),
    )
    parser.add_argument(
        "--shape-kd",
        type=float,
        default=float(_first_config_value(config_data, (("shape", "kd"),), 0.01)),
    )
    parser.add_argument(
        "--friction",
        type=float,
        default=float(_first_config_value(config_data, (("cable", "friction"), ("shape", "mu")), 3.0)),
    )
    parser.add_argument(
        "--contact-margin",
        type=float,
        default=float(_first_config_value(config_data, (("contact", "rigid_contact_margin"),), 0.0005)),
    )
    parser.add_argument(
        "--rigid-gap",
        type=float,
        default=float(_first_config_value(config_data, (("contact", "rigid_gap"),), 0.0)),
    )
    parser.add_argument(
        "--rigid-contact-hard",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("solver", "rigid_contact_hard"),), False)),
    )
    parser.add_argument(
        "--rigid-contact-history",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("solver", "rigid_contact_history"),), False)),
    )
    parser.add_argument(
        "--stretch-stiffness",
        type=float,
        default=float(_first_config_value(config_data, (("cable", "stretch_stiffness"),), 100.0)),
    )
    parser.add_argument(
        "--stretch-damping",
        type=float,
        default=float(_first_config_value(config_data, (("cable", "stretch_damping"),), 0.05)),
    )
    parser.add_argument(
        "--bend-stiffness",
        type=float,
        default=float(_first_config_value(config_data, (("cable", "bend_stiffness"),), 0.001)),
    )
    parser.add_argument(
        "--bend-damping",
        type=float,
        default=float(_first_config_value(config_data, (("cable", "bend_damping"),), 0.05)),
    )
    parser.add_argument(
        "--cable-dahl-friction",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("cable", "dahl_friction", "enabled"),), False)),
    )
    parser.add_argument(
        "--cable-dahl-eps-max",
        type=float,
        default=float(_first_config_value(config_data, (("cable", "dahl_friction", "eps_max"),), 0.0)),
    )
    parser.add_argument(
        "--cable-dahl-tau",
        type=float,
        default=float(_first_config_value(config_data, (("cable", "dahl_friction", "tau"),), 1.0)),
    )
    parser.add_argument(
        "--cable-self-collision-filter-neighbor-hops",
        type=int,
        default=int(_first_config_value(config_data, (("cable", "self_collision_filter_neighbor_hops"),), 2)),
    )
    parser.add_argument(
        "--rigid-avbd-beta",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_avbd_beta"),), 0.0)),
    )
    parser.add_argument(
        "--rigid-avbd-gamma",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_avbd_gamma"),), 0.99)),
    )
    parser.add_argument(
        "--rigid-body-contact-buffer-size",
        type=int,
        default=int(_first_config_value(config_data, (("solver", "rigid_body_contact_buffer_size"),), 8192)),
    )
    parser.add_argument(
        "--rigid-body-particle-contact-buffer-size",
        type=int,
        default=int(_first_config_value(config_data, (("solver", "rigid_body_particle_contact_buffer_size"),), 256)),
    )
    parser.add_argument(
        "--rigid-joint-linear-ke",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_joint_linear_ke"),), 1.0e9)),
    )
    parser.add_argument(
        "--rigid-joint-angular-ke",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_joint_angular_ke"),), 1.0e9)),
    )
    parser.add_argument(
        "--rigid-joint-linear-k-start",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_joint_linear_k_start"),), 1.0e4)),
    )
    parser.add_argument(
        "--rigid-joint-angular-k-start",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_joint_angular_k_start"),), 1.0e1)),
    )
    parser.add_argument(
        "--rigid-joint-linear-kd",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_joint_linear_kd"),), 1.0e-2)),
    )
    parser.add_argument(
        "--rigid-joint-angular-kd",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "rigid_joint_angular_kd"),), 0.0)),
    )
    parser.add_argument(
        "--fixed-points-as-static",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("cable", "fixed_points_as_static"),), True)),
    )
    parser.add_argument(
        "--add-articulation",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("cable", "add_articulation"),), True)),
    )
    parser.add_argument(
        "--add-ground",
        action=argparse.BooleanOptionalAction,
        default=bool(_first_config_value(config_data, (("ground", "enabled"),), False)),
    )
    parser.add_argument(
        "--ground-height",
        type=float,
        default=float(_first_config_value(config_data, (("ground", "height"),), 0.0)),
    )
    parser.add_argument(
        "--friction-epsilon",
        type=float,
        default=float(_first_config_value(config_data, (("solver", "friction_epsilon"),), 0.01)),
    )
    parser.add_argument(
        "--rigid-contact-max",
        type=int,
        default=int(_first_config_value(config_data, (("solver", "rigid_contact_max"),), 65536)),
    )
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.fps <= 0:
        raise ValueError("--fps must be positive.")
    if args.substeps <= 0:
        raise ValueError("--substeps must be positive.")
    if args.iterations <= 0:
        raise ValueError("--iterations must be positive.")
    if args.density <= 0.0:
        raise ValueError("--density must be positive.")
    if args.friction_epsilon <= 0.0:
        raise ValueError("--friction-epsilon must be positive.")
    if args.rigid_contact_max <= 0:
        raise ValueError("--rigid-contact-max must be positive.")
    if args.rigid_body_contact_buffer_size <= 0:
        raise ValueError("--rigid-body-contact-buffer-size must be positive.")
    if args.rigid_body_particle_contact_buffer_size <= 0:
        raise ValueError("--rigid-body-particle-contact-buffer-size must be positive.")
    for key in (
        "rigid_joint_linear_ke",
        "rigid_joint_angular_ke",
    ):
        if float(getattr(args, key)) <= 0.0:
            raise ValueError(f"--{key.replace('_', '-')} must be positive.")
    for key in (
        "rigid_joint_linear_k_start",
        "rigid_joint_angular_k_start",
        "rigid_joint_linear_kd",
        "rigid_joint_angular_kd",
    ):
        if float(getattr(args, key)) < 0.0:
            raise ValueError(f"--{key.replace('_', '-')} must be non-negative.")
    if args.board_friction < 0.0:
        raise ValueError("--board-friction must be non-negative.")
    if args.board_support_plane_clearance < 0.0:
        raise ValueError("--board-support-plane-clearance must be non-negative.")
    if args.board_support_plane_expand < 0.0:
        raise ValueError("--board-support-plane-expand must be non-negative.")
    if args.board_top_support_thickness <= 0.0:
        raise ValueError("--board-top-support-thickness must be positive.")
    if args.board_top_support_offset < 0.0:
        raise ValueError("--board-top-support-offset must be non-negative.")
    if args.board_top_support_expand < 0.0:
        raise ValueError("--board-top-support-expand must be non-negative.")
    if args.board_floor_clearance < 0.0:
        raise ValueError("--board-floor-clearance must be non-negative.")
    if args.board_floor_expand < 0.0:
        raise ValueError("--board-floor-expand must be non-negative.")
    if args.friction is not None and args.friction < 0.0:
        raise ValueError("--friction must be non-negative.")
    if args.shape_ke is not None and args.shape_ke < 0.0:
        raise ValueError("--shape-ke must be non-negative.")
    if args.shape_kd is not None and args.shape_kd < 0.0:
        raise ValueError("--shape-kd must be non-negative.")
    if args.contact_margin < 0.0:
        raise ValueError("--contact-margin must be non-negative.")
    if args.rigid_gap < 0.0:
        raise ValueError("--rigid-gap must be non-negative.")
    if args.rigid_avbd_beta < 0.0:
        raise ValueError("--rigid-avbd-beta must be non-negative.")
    if args.rigid_avbd_gamma < 0.0 or args.rigid_avbd_gamma > 1.0:
        raise ValueError("--rigid-avbd-gamma must be in [0, 1].")
    if args.cable_dahl_eps_max < 0.0:
        raise ValueError("--cable-dahl-eps-max must be non-negative.")
    if args.cable_dahl_tau < 0.0:
        raise ValueError("--cable-dahl-tau must be non-negative.")
    if args.cable_self_collision_filter_neighbor_hops < 0:
        raise ValueError("--cable-self-collision-filter-neighbor-hops must be non-negative.")
    if getattr(args, "proxy_gripper_friction", 0.0) < 0.0:
        raise ValueError("--proxy-gripper-friction must be non-negative.")
    if any(float(v) <= 0.0 for v in getattr(args, "proxy_gripper_size", ())):
        raise ValueError("--proxy-gripper-size values must be positive.")
    if args.gripper_min_gap < 0.0:
        raise ValueError("--gripper-min-gap must be non-negative.")
    if args.gripper_min_gap > args.gripper_max_gap:
        raise ValueError("--gripper-min-gap must be <= --gripper-max-gap.")
    if args.gripper_grasp_bind_confirm_steps <= 0:
        raise ValueError("--gripper-grasp-bind-confirm-steps must be positive.")
    if args.gripper_grasp_bind_release_gap < 0.0:
        raise ValueError("--gripper-grasp-bind-release-gap must be non-negative.")
    for key in (
        "gripper_grasp_bind_max_position_error",
        "gripper_grasp_bind_max_rotation_error",
        "gripper_grasp_bind_activation_radius",
    ):
        if float(getattr(args, key)) < 0.0:
            raise ValueError(f"--{key.replace('_', '-')} must be non-negative.")
    for key in ("gripper_grasp_bind_normal_alignment", "gripper_grasp_bind_opposing_normal"):
        value = float(getattr(args, key))
        if value < -1.0 or value > 1.0:
            raise ValueError(f"--{key.replace('_', '-')} must be in [-1, 1].")
    for key in ("gripper_initial_gap", "gripper_target_gap"):
        value = float(getattr(args, key))
        if value < args.gripper_min_gap or value > args.gripper_max_gap:
            raise ValueError(f"--{key.replace('_', '-')} must be within the configured gripper gap range.")
    for key in (
        "gripper_finger_density",
        "gripper_finger_friction",
        "gripper_drive_force",
        "gripper_stiffness",
        "gripper_damping",
        "gripper_linear_speed",
        "gripper_linear_speed_xy",
        "gripper_linear_speed_z",
        "gripper_angular_speed_deg",
        "gripper_gap_speed",
    ):
        if float(getattr(args, key)) < 0.0:
            raise ValueError(f"--{key.replace('_', '-')} must be non-negative.")



class ProxyGripperController:
    """Small kinematic collider driven by an external pose topic."""

    def __init__(self, body_id: int, position_m, rotation):
        self.body_id = int(body_id)
        self.position_m = np.asarray(position_m, dtype=np.float32)
        self.rotation = rotation

    def set_command(self, position_m, euler_xyz_rad) -> None:
        self.position_m = np.asarray(position_m, dtype=np.float32)
        self.rotation = wp.quat_rpy(
            float(euler_xyz_rad[0]),
            float(euler_xyz_rad[1]),
            float(euler_xyz_rad[2]),
        )

    def command_position(self) -> tuple[float, float, float]:
        return (
            float(self.position_m[0]),
            float(self.position_m[1]),
            float(self.position_m[2]),
        )

    def apply(self, state: newton.State) -> None:
        body_q = state.body_q.numpy()
        body_q[self.body_id, :3] = self.position_m
        q = self.rotation
        body_q[self.body_id, 3:] = np.asarray([q[0], q[1], q[2], q[3]], dtype=np.float32)
        state.body_q.assign(body_q)

        body_qd = state.body_qd.numpy()
        body_qd[self.body_id, :] = 0.0
        state.body_qd.assign(body_qd)


class Example:
    def __init__(self, viewer, args: argparse.Namespace):
        _validate_args(args)
        usd_path = Path(args.usd_path).expanduser().resolve()
        if not usd_path.is_file():
            raise FileNotFoundError(f"Missing board cable USD: {usd_path}")

        self.viewer = viewer
        self.fps = int(args.fps)
        self.frame_dt = 1.0 / float(self.fps)
        self.sim_substeps = int(args.substeps)
        self.sim_dt = self.frame_dt / float(self.sim_substeps)
        self.sim_time = 0.0

        _, curve_prim = _open_curve_stage(usd_path, args.curve_prim_path)
        shape_ke = _override_or_curve_attr(args.shape_ke, curve_prim, "newton:ke", 1.0e5)
        shape_kd = _override_or_curve_attr(args.shape_kd, curve_prim, "newton:kd", 1.0)
        friction = _override_or_curve_attr(args.friction, curve_prim, "newton:mu", 0.5)
        stretch_stiffness = _override_or_curve_attr(
            args.stretch_stiffness, curve_prim, "newton:stretch_stiffness", 1.0e6
        )
        stretch_damping = _override_or_curve_attr(args.stretch_damping, curve_prim, "newton:stretch_damping", 0.0)
        bend_stiffness = _override_or_curve_attr(args.bend_stiffness, curve_prim, "newton:bend_stiffness", 0.01)
        bend_damping = _override_or_curve_attr(args.bend_damping, curve_prim, "newton:bend_damping", 0.0)
        cable_dahl_eps_max = _override_or_curve_attr(
            args.cable_dahl_eps_max, curve_prim, "newton:vbd:dahl_eps_max", 0.0
        )
        cable_dahl_tau = _override_or_curve_attr(args.cable_dahl_tau, curve_prim, "newton:vbd:dahl_tau", 1.0)
        cable_dahl_friction = bool(args.cable_dahl_friction) and cable_dahl_eps_max > 0.0 and cable_dahl_tau > 0.0

        builder = newton.ModelBuilder()
        if cable_dahl_friction:
            newton.solvers.SolverVBD.register_custom_attributes(builder)
        builder.num_rigid_contacts_per_world = int(args.rigid_contact_max)
        builder.rigid_contact_margin = float(args.contact_margin)
        builder.rigid_gap = float(args.rigid_gap)
        builder.default_shape_cfg.density = float(args.density)
        builder.default_shape_cfg.ke = float(shape_ke)
        builder.default_shape_cfg.kd = float(shape_kd)
        builder.default_shape_cfg.mu = float(friction)
        builder.default_shape_cfg.margin = float(args.contact_margin)
        builder.default_shape_cfg.gap = float(args.rigid_gap)

        self.board_body_count = 0
        self.board_shape_count = 0
        self.board_collision_shape_count = 0
        self.board_convex_collision_shape_count = 0
        self.board_top_support_shape_count = 0
        self.board_support_plane_shape_count = 0
        self.board_joint_count = 0
        self.board_floor_constraint: BoardFloorConstraint | None = None
        self.board_support_plane: BoardSupportPlane | None = None
        board_usd_path: Path | None = None
        if args.load_board:
            board_usd_path = Path(args.board_usd_path).expanduser().resolve()
            if not board_usd_path.is_file():
                raise FileNotFoundError(f"Missing board USD: {board_usd_path}")
            board_scene_result = builder.add_usd(
                str(board_usd_path),
                root_path=str(args.board_root_path),
                floating=False,
                load_sites=False,
                load_visual_shapes=bool(args.board_load_visual_shapes),
                hide_collision_shapes=bool(args.board_hide_collision_shapes),
                parse_mujoco_options=False,
                only_load_enabled_joints=True,
                only_load_enabled_rigid_bodies=False,
            )
            self.board_body_count = len(set(int(v) for v in board_scene_result["path_body_map"].values()))
            board_shape_ids = set(int(v) for v in board_scene_result["path_shape_map"].values())
            self.board_shape_count = len(board_shape_ids)
            self.board_joint_count = len(set(int(v) for v in board_scene_result["path_joint_map"].values()))
            if args.board_visual_shapes_as_colliders:
                for shape_id in board_shape_ids:
                    builder.shape_flags[shape_id] |= int(newton.ShapeFlags.COLLIDE_SHAPES)
            for shape_id in board_shape_ids:
                if builder.shape_flags[shape_id] & int(newton.ShapeFlags.COLLIDE_SHAPES):
                    builder.shape_material_mu[shape_id] = float(args.board_friction)
                    builder.shape_material_ke[shape_id] = float(shape_ke)
                    builder.shape_material_kd[shape_id] = float(shape_kd)
                    builder.shape_margin[shape_id] = float(args.contact_margin)
                    builder.shape_gap[shape_id] = float(args.rigid_gap)
            self.board_collision_shape_count = sum(
                1 for shape_id in board_shape_ids if builder.shape_flags[shape_id] & int(newton.ShapeFlags.COLLIDE_SHAPES)
            )
            self.board_convex_collision_shape_count = sum(
                1
                for shape_id in board_shape_ids
                if builder.shape_flags[shape_id] & int(newton.ShapeFlags.COLLIDE_SHAPES)
                and int(builder.shape_type[shape_id]) == int(newton.GeoType.CONVEX_MESH)
            )
            for body_id in set(int(v) for v in board_scene_result["path_body_map"].values()):
                builder.body_mass[body_id] = 0.0
                builder.body_inv_mass[body_id] = 0.0
                builder.body_inertia[body_id] = wp.mat33()
                builder.body_inv_inertia[body_id] = wp.mat33()
            self.board_top_support_shape_count = _add_board_top_support_collision(
                builder,
                board_usd_path,
                args,
                float(shape_ke),
                float(shape_kd),
            )
            self.board_shape_count += self.board_top_support_shape_count
            self.board_collision_shape_count += self.board_top_support_shape_count
            if args.require_board_collision and self.board_collision_shape_count == 0:
                raise ValueError(
                    f"Board USD '{board_usd_path}' produced no colliding shapes under root '{args.board_root_path}'."
                )
            if args.require_board_convex_collision and self.board_convex_collision_shape_count == 0:
                raise ValueError(
                    f"Board USD '{board_usd_path}' produced no convex hull collision shapes under root "
                    f"'{args.board_root_path}'."
                )
        elif args.require_board_collision or args.require_board_convex_collision:
            raise ValueError("Board collision was required, but --no-load-board was set.")

        result = add_cable_from_usd_curve(
            builder=builder,
            source_usd_path=str(usd_path),
            curve_prim_path=str(args.curve_prim_path),
            cable_label=str(args.label),
            cable_cfg=builder.default_shape_cfg.copy(),
            stretch_stiffness=float(stretch_stiffness),
            stretch_damping=float(stretch_damping),
            bend_stiffness=float(bend_stiffness),
            bend_damping=float(bend_damping),
            wrap_in_articulation=False,
        )
        self.import_result = result
        self.cable_self_collision_filter_pair_count = _apply_cable_nearby_self_collision_filter(
            builder,
            result.cable_body_ids,
            int(args.cable_self_collision_filter_neighbor_hops),
        )
        if args.load_board and board_usd_path is not None:
            self.board_support_plane = _add_board_support_plane(
                builder,
                board_usd_path,
                args,
                float(shape_ke),
                float(shape_kd),
                float(result.radius_m),
            )
            if self.board_support_plane is not None:
                self.board_support_plane_shape_count = self.board_support_plane.shape_count
                self.board_shape_count += self.board_support_plane_shape_count
                self.board_collision_shape_count += self.board_support_plane_shape_count
            self.board_floor_constraint = _make_board_floor_constraint(
                board_usd_path,
                args,
                float(result.radius_m),
            )

        articulation_joint_ids = [*result.cable_joint_ids, *result.head_fixed_joint_ids]
        if args.add_articulation and articulation_joint_ids:
            builder.add_articulation(articulation_joint_ids, label=f"{args.label}_articulation")

        if args.fixed_points_as_static:
            for body_id in result.fixed_body_ids:
                builder.body_mass[body_id] = 0.0
                builder.body_inv_mass[body_id] = 0.0
                builder.body_inertia[body_id] = wp.mat33()
                builder.body_inv_inertia[body_id] = wp.mat33()

        if args.add_ground:
            builder.add_ground_plane(height=float(args.ground_height), label="ground")

        self.gripper_controller: SraGripperController | None = None
        self.gripper_config: SraGripperConfig | None = None
        self.gripper_grasp_bind_controller: GraspPoseBindController | None = None
        gripper_build_result = None
        self.robotiq_finger_body_ids: tuple[int, ...] = ()
        self.robotiq_finger_shape_ids: tuple[int, ...] = ()
        self.robotiq_grasp_bind_controller: RobotiqFingerPoseBindController | None = None
        self.robotiq_finger_size_m: tuple[float, float, float] = tuple(
            float(v) for v in getattr(args, "robotiq_finger_size", (0.007, 0.010, 0.028))
        )
        self.proxy_gripper_controller: ProxyGripperController | None = None
        if args.proxy_gripper:
            proxy_config = _make_gripper_config(args, result.source_points_m)
            proxy_size = tuple(float(v) for v in args.proxy_gripper_size)
            proxy_shape_cfg = builder.default_shape_cfg.copy()
            proxy_shape_cfg.density = 0.0
            proxy_shape_cfg.mu = float(args.proxy_gripper_friction)
            proxy_shape_cfg.ke = float(shape_ke)
            proxy_shape_cfg.kd = float(shape_kd)
            proxy_shape_cfg.margin = float(args.contact_margin)
            proxy_shape_cfg.gap = float(args.rigid_gap)
            proxy_shape_cfg.has_shape_collision = True
            proxy_shape_cfg.has_particle_collision = False
            proxy_body = builder.add_link(
                xform=wp.transform(wp.vec3(*proxy_config.pose.position_m), proxy_config.pose.rotation),
                mass=0.0,
                is_kinematic=True,
                label=f"{args.label}:proxy_gripper",
            )
            builder.add_shape_box(
                body=proxy_body,
                xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity()),
                hx=0.5 * proxy_size[0],
                hy=0.5 * proxy_size[1],
                hz=0.5 * proxy_size[2],
                cfg=proxy_shape_cfg,
                label=f"{args.label}:proxy_gripper_collider",
                color=(0.8, 0.2, 0.1),
            )
            self.proxy_gripper_controller = ProxyGripperController(
                proxy_body,
                proxy_config.pose.position_m,
                proxy_config.pose.rotation,
            )
        if bool(getattr(args, "robotiq_finger_targets", False)):
            robotiq_shape_cfg = builder.default_shape_cfg.copy()
            robotiq_shape_cfg.density = 0.0
            robotiq_shape_cfg.mu = float(getattr(args, "robotiq_finger_friction", 0.8))
            robotiq_shape_cfg.ke = float(shape_ke)
            robotiq_shape_cfg.kd = float(shape_kd)
            robotiq_shape_cfg.margin = float(args.contact_margin)
            robotiq_shape_cfg.gap = float(args.rigid_gap)
            robotiq_shape_cfg.has_shape_collision = True
            robotiq_shape_cfg.has_particle_collision = False
            robotiq_body_ids: list[int] = []
            robotiq_shape_ids: list[int] = []
            sx, sy, sz = self.robotiq_finger_size_m
            for finger_id in range(4):
                body_id = builder.add_link(
                    xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity()),
                    mass=0.0,
                    is_kinematic=True,
                    label=f"{args.label}:robotiq_finger_target_{finger_id}",
                )
                shape_id = builder.add_shape_box(
                    body=body_id,
                    xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity()),
                    hx=0.5 * sx,
                    hy=0.5 * sy,
                    hz=0.5 * sz,
                    cfg=robotiq_shape_cfg,
                    label=f"{args.label}:robotiq_finger_collision_{finger_id}",
                    color=(0.9, 0.05, 0.02),
                )
                robotiq_body_ids.append(body_id)
                robotiq_shape_ids.append(shape_id)
            self.robotiq_finger_body_ids = tuple(robotiq_body_ids)
            self.robotiq_finger_shape_ids = tuple(robotiq_shape_ids)
        if args.gripper and not bool(getattr(args, "robotiq_finger_targets", False)):
            self.gripper_config = _make_gripper_config(args, result.source_points_m)
            gripper_build_result = add_gripper_from_config(builder, self.gripper_config)
            self.gripper_controller = SraGripperController(self.gripper_config, gripper_build_result)

        builder.color()
        sim_device = wp.get_device(args.device)
        self.model = builder.finalize(device=sim_device)
        if cable_dahl_friction and hasattr(self.model, "vbd"):
            self.model.vbd.dahl_eps_max.fill_(float(cable_dahl_eps_max))
            self.model.vbd.dahl_tau.fill_(float(cable_dahl_tau))
        fixed_body_ids = {int(body_id) for body_id in result.fixed_body_ids}
        candidate_body_ids = tuple(
            int(body_id) for body_id in result.cable_body_ids if int(body_id) not in fixed_body_ids
        )
        if len(candidate_body_ids) == 0:
            candidate_body_ids = tuple(int(body_id) for body_id in result.cable_body_ids)
        if self.gripper_controller is not None and gripper_build_result is not None and bool(args.gripper_grasp_bind):
            if len(candidate_body_ids) > 0:
                grasp_bind_config = GraspPoseBindConfig(
                    enabled=True,
                    candidate_body_labels=(),
                    confirm_steps=int(args.gripper_grasp_bind_confirm_steps),
                    release_gap_m=float(args.gripper_grasp_bind_release_gap),
                    normal_alignment_min_cos=float(args.gripper_grasp_bind_normal_alignment),
                    opposing_normal_min_cos=float(args.gripper_grasp_bind_opposing_normal),
                    max_position_error_m=float(args.gripper_grasp_bind_max_position_error),
                    max_rotation_error_rad=float(args.gripper_grasp_bind_max_rotation_error),
                    candidate_body_label_prefixes=(),
                    activation_radius_m=float(args.gripper_grasp_bind_activation_radius),
                )
                self.gripper_grasp_bind_controller = GraspPoseBindController(
                    grasp_bind_config,
                    gripper_build_result,
                    self.model,
                    candidate_body_ids,
                )
        if self.robotiq_finger_body_ids and self.robotiq_finger_shape_ids and bool(args.gripper_grasp_bind):
            if len(candidate_body_ids) > 0:
                robotiq_grasp_bind_config = GraspPoseBindConfig(
                    enabled=True,
                    candidate_body_labels=(),
                    confirm_steps=int(args.gripper_grasp_bind_confirm_steps),
                    release_gap_m=float(args.gripper_grasp_bind_release_gap),
                    normal_alignment_min_cos=float(args.gripper_grasp_bind_normal_alignment),
                    opposing_normal_min_cos=float(args.gripper_grasp_bind_opposing_normal),
                    max_position_error_m=float(args.gripper_grasp_bind_max_position_error),
                    max_rotation_error_rad=float(args.gripper_grasp_bind_max_rotation_error),
                    candidate_body_label_prefixes=(),
                    activation_radius_m=float(args.gripper_grasp_bind_activation_radius),
                )
                self.robotiq_grasp_bind_controller = RobotiqFingerPoseBindController(
                    robotiq_grasp_bind_config,
                    self.model,
                    candidate_body_ids,
                    self.robotiq_finger_body_ids,
                    self.robotiq_finger_shape_ids,
                )
                print(
                    "[board_cable] Robotiq grasp bind enabled: "
                    f"pairs=((0, 1), (2, 3)) candidates={len(candidate_body_ids)} "
                    f"release_gap={float(args.gripper_grasp_bind_release_gap):.6g}"
                )
        self.model.rigid_contact_max = int(args.rigid_contact_max)
        self.gravity = tuple(float(v) for v in args.gravity)
        if args.gravity_z is not None:
            self.gravity = (self.gravity[0], self.gravity[1], float(args.gravity_z))
        self.model.set_gravity(self.gravity)

        self.solver = newton.solvers.SolverVBD(
            self.model,
            iterations=int(args.iterations),
            friction_epsilon=float(args.friction_epsilon),
            rigid_avbd_beta=float(args.rigid_avbd_beta),
            rigid_avbd_gamma=float(args.rigid_avbd_gamma),
            rigid_contact_hard=bool(args.rigid_contact_hard),
            rigid_contact_history=bool(args.rigid_contact_history),
            rigid_body_contact_buffer_size=int(args.rigid_body_contact_buffer_size),
            rigid_body_particle_contact_buffer_size=int(args.rigid_body_particle_contact_buffer_size),
            rigid_joint_linear_ke=float(args.rigid_joint_linear_ke),
            rigid_joint_angular_ke=float(args.rigid_joint_angular_ke),
            rigid_joint_linear_k_start=float(args.rigid_joint_linear_k_start),
            rigid_joint_angular_k_start=float(args.rigid_joint_angular_k_start),
            rigid_joint_linear_kd=float(args.rigid_joint_linear_kd),
            rigid_joint_angular_kd=float(args.rigid_joint_angular_kd),
        )
        if hasattr(self.solver, "set_joint_constraint_mode"):
            for joint_id in range(self.model.joint_count):
                self.solver.set_joint_constraint_mode(joint_id, hard=False)
            if self.gripper_controller is not None:
                gripper_joint_ids = (
                    int(self.gripper_controller.build_result.world_joint_id),
                    *[int(joint_id) for joint_id in self.gripper_controller.build_result.prismatic_joint_ids],
                )
                for joint_id in gripper_joint_ids:
                    self.solver.set_joint_constraint_mode(joint_id, hard=True)

        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        if self.gripper_controller is not None:
            self.gripper_controller.apply(self.state_0, self.control, self.gravity)
            self.gripper_controller.apply(self.state_1, self.control, self.gravity)
        if self.proxy_gripper_controller is not None:
            self.proxy_gripper_controller.apply(self.state_0)
            self.proxy_gripper_controller.apply(self.state_1)
        if self.robotiq_finger_body_ids:
            for state in (self.state_0, self.state_1):
                body_q = state.body_q.numpy()
                body_qd = state.body_qd.numpy()
                for body_id in self.robotiq_finger_body_ids:
                    body_q[int(body_id), :3] = 0.0
                    body_q[int(body_id), 3:] = np.asarray((0.0, 0.0, 0.0, 1.0), dtype=np.float32)
                    body_qd[int(body_id), :] = 0.0
                state.body_q.assign(body_q)
                state.body_qd.assign(body_qd)
        pipeline = newton.CollisionPipeline(
            self.model,
            rigid_contact_max=int(args.rigid_contact_max),
            contact_matching="latest",
        )
        self.contacts = self.model.contacts(collision_pipeline=pipeline)

        self.viewer.set_model(self.model)
        self._install_gripper_camera_input_guard()
        self._set_camera_from_points(result.source_points_m)
        self._print_summary(
            usd_path=usd_path,
            curve_prim_path=str(args.curve_prim_path),
            shape_ke=shape_ke,
            shape_kd=shape_kd,
            friction=friction,
            board_friction=float(args.board_friction),
            contact_margin=float(args.contact_margin),
            rigid_gap=float(args.rigid_gap),
            rigid_contact_hard=bool(args.rigid_contact_hard),
            rigid_contact_history=bool(args.rigid_contact_history),
            rigid_avbd_beta=float(args.rigid_avbd_beta),
            rigid_avbd_gamma=float(args.rigid_avbd_gamma),
            board_floor_constraint="on" if self.board_floor_constraint is not None else "off",
            cable_self_collision_filter_pair_count=self.cable_self_collision_filter_pair_count,
            stretch_stiffness=stretch_stiffness,
            stretch_damping=stretch_damping,
            bend_stiffness=bend_stiffness,
            bend_damping=bend_damping,
            cable_dahl_friction=cable_dahl_friction,
            cable_dahl_eps_max=cable_dahl_eps_max,
            cable_dahl_tau=cable_dahl_tau,
        )
        if args.load_board and self.board_collision_shape_count == 0:
            print(
                "[board_cable] warning: board USD loaded through add_usd but produced 0 colliding shapes. "
                "The current board file may contain no Mesh/PhysicsCollision prims, so it will not collide."
            )
        if self.gripper_controller is not None:
            print(
                "[board_cable] teleop: hold Ctrl and use W/S/A/D/Q/E to move, "
                "C/V Z/X T/G to rotate, N/M to close/open the gripper."
            )

    def _set_camera_from_points(self, points_m: np.ndarray) -> None:
        if not hasattr(self.viewer, "set_camera") or points_m.size == 0:
            return
        center = np.mean(points_m, axis=0)
        span = np.max(points_m, axis=0) - np.min(points_m, axis=0)
        distance = max(float(np.linalg.norm(span)), 1.0)
        self.viewer.set_camera(
            pos=wp.vec3(float(center[0]), float(center[1] - distance), float(center[2] + 0.45 * distance)),
            pitch=-20.0,
            yaw=90.0,
        )

    def _print_summary(self, **kwargs: float | str | Path) -> None:
        result = self.import_result
        gripper_text = "off"
        if self.gripper_controller is not None:
            gripper_text = (
                f"on pos={self.gripper_controller.command_position()} "
                f"gap={self.gripper_controller.command_gap_m():.6g}"
            )
        elif self.proxy_gripper_controller is not None:
            gripper_text = f"proxy pos={self.proxy_gripper_controller.command_position()}"
        print(
            "[board_cable] parsed "
            f"usd={kwargs['usd_path']} curve={kwargs['curve_prim_path']} "
            f"points={len(result.source_points_m)} edges={len(result.edges)} radius_m={result.radius_m:.6g} "
            f"bodies={len(result.cable_body_ids)} joints={len(result.cable_joint_ids)} "
            f"fixed_bodies={len(result.fixed_body_ids)} "
            f"board=({self.board_body_count} bodies, {self.board_shape_count} shapes, "
            f"{self.board_collision_shape_count} colliders, "
            f"{self.board_convex_collision_shape_count} convex_colliders, "
            f"{self.board_top_support_shape_count} top_support, "
            f"{self.board_support_plane_shape_count} support_plane, {self.board_joint_count} joints) "
            f"gripper={gripper_text} "
            f"shape_ke={float(kwargs['shape_ke']):.6g} shape_kd={float(kwargs['shape_kd']):.6g} "
            f"mu={float(kwargs['friction']):.6g} board_mu={float(kwargs['board_friction']):.6g} "
            f"contact_margin={float(kwargs['contact_margin']):.6g} rigid_gap={float(kwargs['rigid_gap']):.6g} "
            f"hard_contact={bool(kwargs['rigid_contact_hard'])} contact_history={bool(kwargs['rigid_contact_history'])} "
            f"avbd_beta={float(kwargs['rigid_avbd_beta']):.6g} avbd_gamma={float(kwargs['rigid_avbd_gamma']):.6g} "
            f"board_floor={kwargs['board_floor_constraint']} "
            f"cable_self_filters={int(kwargs['cable_self_collision_filter_pair_count'])} "
            f"stretch=({float(kwargs['stretch_stiffness']):.6g}, {float(kwargs['stretch_damping']):.6g}) "
            f"bend=({float(kwargs['bend_stiffness']):.6g}, {float(kwargs['bend_damping']):.6g}) "
            f"dahl=({bool(kwargs['cable_dahl_friction'])}, "
            f"{float(kwargs['cable_dahl_eps_max']):.6g}, {float(kwargs['cable_dahl_tau']):.6g})"
        )

    def _install_gripper_camera_input_guard(self) -> None:
        if self.gripper_controller is None:
            return
        if self.gripper_config is None or not self.gripper_config.teleop.enabled:
            return
        if self.gripper_config.teleop.modifier == "none":
            return
        if not hasattr(self.viewer, "_update_camera"):
            return
        if getattr(self.viewer, "_board_cable_gripper_camera_guard_wrapped", False):
            return

        original_update_camera = self.viewer._update_camera

        def update_camera_with_gripper_guard(dt, original_update_camera=original_update_camera, example=self):
            if example.gripper_config is not None and is_gripper_teleop_modifier_down(
                example.viewer,
                example.gripper_config.teleop.modifier,
            ):
                cam_vel = getattr(example.viewer, "_cam_vel", None)
                if cam_vel is not None:
                    try:
                        cam_vel[:] = 0.0
                    except TypeError:
                        example.viewer._cam_vel = cam_vel * 0.0
                return
            original_update_camera(dt)

        self.viewer._update_camera = update_camera_with_gripper_guard
        self.viewer._board_cable_gripper_camera_guard_wrapped = True

    def _apply_board_floor_constraint(self) -> None:
        constraint = self.board_floor_constraint
        if constraint is None:
            return

        cable_body_ids = np.asarray(self.import_result.cable_body_ids, dtype=np.int64)
        if cable_body_ids.size == 0:
            return

        body_q = self.state_0.body_q.numpy()
        cable_pos = body_q[cable_body_ids, :3]
        inside_xy = (
            (cable_pos[:, 0] >= constraint.x_min)
            & (cable_pos[:, 0] <= constraint.x_max)
            & (cable_pos[:, 1] >= constraint.y_min)
            & (cable_pos[:, 1] <= constraint.y_max)
        )
        below_floor = inside_xy & (cable_pos[:, 2] < constraint.floor_z)
        if not np.any(below_floor):
            return

        constrained_body_ids = cable_body_ids[below_floor]
        body_q[constrained_body_ids, 2] = constraint.floor_z
        self.state_0.body_q.assign(body_q)

        if constraint.zero_downward_velocity:
            body_qd = self.state_0.body_qd.numpy()
            downward = body_qd[constrained_body_ids, 2] < 0.0
            if np.any(downward):
                body_qd[constrained_body_ids[downward], 2] = 0.0
                self.state_0.body_qd.assign(body_qd)

    def simulate(self) -> None:
        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)
            if self.gripper_controller is not None:
                self.gripper_controller.update_from_viewer(self.viewer, self.sim_dt)
                self.gripper_controller.apply(self.state_0, self.control, self.gravity)
            if self.proxy_gripper_controller is not None:
                self.proxy_gripper_controller.apply(self.state_0)
            self.model.collide(self.state_0, self.contacts)
            if self.gripper_grasp_bind_controller is not None:
                self.gripper_grasp_bind_controller.update_from_contacts(
                    self.state_0,
                    self.contacts,
                    self.gripper_controller.command_gap_m(),
                )
            if self.robotiq_grasp_bind_controller is not None:
                self.robotiq_grasp_bind_controller.update_from_contacts(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
            if self.gripper_grasp_bind_controller is not None:
                self.gripper_grasp_bind_controller.apply_pose_binding(self.state_0)
            if self.robotiq_grasp_bind_controller is not None:
                self.robotiq_grasp_bind_controller.apply_pose_binding(self.state_0)
            self._apply_board_floor_constraint()

    def step(self) -> None:
        self.simulate()
        self.sim_time += self.frame_dt

    def render(self) -> None:
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.end_frame()


def main() -> None:
    config_path, config_data, gripper_config_path, gripper_config = _load_runtime_configs()
    parser = _make_parser(config_path, config_data, gripper_config_path, gripper_config)
    viewer, args = newton.examples.init(parser)
    example = Example(viewer, args)
    newton.examples.run(example, args)


if __name__ == "__main__":
    main()
