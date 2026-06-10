#!/usr/bin/env python3
"""Create a simple USD wall room with configurable dimensions and materials."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

COMMON_DIR = Path(__file__).resolve().parents[1] / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import asset_path

Gf: Any = None
Sdf: Any = None
Usd: Any = None
UsdGeom: Any = None
UsdLux: Any = None
UsdPhysics: Any = None
UsdShade: Any = None
SIMULATION_APP: Any = None

ROOT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = asset_path("plain_white_room.usd")

MATERIAL_PRESETS = {
    "plain-white": {
        "diffuse_color": (1.0, 1.0, 1.0),
        "metallic": 0.0,
        "roughness": 0.65,
    },
    "matte-gray": {
        "diffuse_color": (0.65, 0.65, 0.65),
        "metallic": 0.0,
        "roughness": 0.85,
    },
    "warm-white": {
        "diffuse_color": (1.0, 0.96, 0.88),
        "metallic": 0.0,
        "roughness": 0.7,
    },
}

NATURAL_LIGHT_COLOR = (1.0, 0.97, 0.92)
ROOM_LIGHT_INTENSITY = 6000.0
AMBIENT_LIGHT_INTENSITY = 60.0
DEFAULT_LIGHT_PANEL_SIZE = 0.6
LIGHT_SIZE_PRESETS = {
    "square": (0.6, 0.6),
    "rectangle": (2.0, 0.28),
}
PARTITION_WIDTH = 5.0
PARTITION_HEIGHT = 2.25
PARTITION_THICKNESS = 0.005
DOOR_WIDTH = 1.0
DOOR_HEIGHT = 2.0
DOOR_POSITION = (0.0, 2.25, 1.0)


def initialize_usd_runtime() -> None:
    """Load pxr modules, starting Isaac Sim when required."""
    global Gf, Sdf, SIMULATION_APP, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    try:
        from pxr import Gf as pxr_gf
        from pxr import Sdf as pxr_sdf
        from pxr import Usd as pxr_usd
        from pxr import UsdGeom as pxr_usd_geom
        from pxr import UsdLux as pxr_usd_lux
        from pxr import UsdPhysics as pxr_usd_physics
        from pxr import UsdShade as pxr_usd_shade
    except ImportError:
        from isaacsim import SimulationApp

        SIMULATION_APP = SimulationApp({"headless": True})
        from pxr import Gf as pxr_gf
        from pxr import Sdf as pxr_sdf
        from pxr import Usd as pxr_usd
        from pxr import UsdGeom as pxr_usd_geom
        from pxr import UsdLux as pxr_usd_lux
        from pxr import UsdPhysics as pxr_usd_physics
        from pxr import UsdShade as pxr_usd_shade

    Gf = pxr_gf
    Sdf = pxr_sdf
    Usd = pxr_usd
    UsdGeom = pxr_usd_geom
    UsdLux = pxr_usd_lux
    UsdPhysics = pxr_usd_physics
    UsdShade = pxr_usd_shade


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a simple wall room USD asset.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Output USD path.",
    )
    parser.add_argument(
        "--length",
        type=float,
        default=30.0,
        help="Inside room length along Y in meters.",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=20.0,
        help="Inside room width along X in meters.",
    )
    parser.add_argument(
        "--height",
        type=float,
        default=3.0,
        help="Wall height in meters.",
    )
    parser.add_argument(
        "--wall-thickness",
        type=float,
        default=0.1,
        help="Wall thickness in meters.",
    )
    parser.add_argument(
        "--material-preset",
        choices=sorted(MATERIAL_PRESETS),
        default="plain-white",
        help="Material preset for floor and walls.",
    )
    parser.add_argument(
        "--floor-only",
        action="store_true",
        help="Create only the floor, without walls.",
    )
    parser.add_argument(
        "--ceiling",
        action="store_true",
        help="Add a ceiling panel to the room.",
    )
    parser.add_argument(
        "--light-density",
        type=float,
        default=1.8,
        help=(
            "Target spacing in meters between ceiling rect lights. "
            "Smaller values create more lights."
        ),
    )
    parser.add_argument(
        "--light-size",
        choices=sorted(LIGHT_SIZE_PRESETS),
        default="square",
        help="Fixed ceiling light panel size preset.",
    )
    parser.add_argument(
        "--partition",
        action="store_true",
        help="Add a 5m partition wall with a 1m x 2m door opening.",
    )
    return parser.parse_args()


def build_output_path(
    output_path: Path,
    width: float,
    length: float,
    height: float,
    partition: bool,
) -> Path:
    """Append integer room dimensions to the output filename stem."""
    dimension_suffix = f"{int(width)}_{int(length)}_{int(height)}"
    partition_suffix = "_partition" if partition else ""
    return output_path.with_name(
        f"{output_path.stem}_{dimension_suffix}{partition_suffix}{output_path.suffix}"
    )


def create_preview_material(
    stage: Any,
    path: str,
    preset: dict[str, Any],
) -> Any:
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*preset["diffuse_color"])
    )
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
        preset["metallic"]
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
        preset["roughness"]
    )
    surface_output = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surface_output)
    return material


def set_cube_transform(
    prim: Any,
    position: tuple[float, float, float],
    scale: tuple[float, float, float],
) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def add_box(
    stage: Any,
    path: str,
    position: tuple[float, float, float],
    scale: tuple[float, float, float],
    material: Any,
) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()
    set_cube_transform(prim, position, scale)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def create_partition_wall(stage: Any, material: Any) -> None:
    door_x, door_y, door_z = DOOR_POSITION
    side_width = (PARTITION_WIDTH - DOOR_WIDTH) / 2.0
    if side_width <= 0.0:
        raise ValueError("Partition width must be greater than door width.")

    wall_center_z = PARTITION_HEIGHT / 2.0
    half_door_width = DOOR_WIDTH / 2.0
    left_center_x = door_x - half_door_width - side_width / 2.0
    right_center_x = door_x + half_door_width + side_width / 2.0

    add_box(
        stage,
        "/Room/Geometry/Partition_Left",
        position=(left_center_x, door_y, wall_center_z),
        scale=(side_width, PARTITION_THICKNESS, PARTITION_HEIGHT),
        material=material,
    )
    add_box(
        stage,
        "/Room/Geometry/Partition_Right",
        position=(right_center_x, door_y, wall_center_z),
        scale=(side_width, PARTITION_THICKNESS, PARTITION_HEIGHT),
        material=material,
    )

    header_height = PARTITION_HEIGHT - (door_z + DOOR_HEIGHT / 2.0)
    if header_height > 0.0:
        header_center_z = PARTITION_HEIGHT - header_height / 2.0
        add_box(
            stage,
            "/Room/Geometry/Partition_Header",
            position=(door_x, door_y, header_center_z),
            scale=(DOOR_WIDTH, PARTITION_THICKNESS, header_height),
            material=material,
        )


def create_rect_lights(
    stage: Any,
    width: float,
    length: float,
    height: float,
    light_density: float,
    light_size: str,
) -> int:
    spacing = max(light_density, 0.5)
    z_position = max(height - 0.02, 0.1)
    light_width, light_height = LIGHT_SIZE_PRESETS[light_size]

    def compute_axis_positions(span: float, panel_size: float) -> list[float]:
        if span <= panel_size:
            return [0.0]

        light_count = max(
            1,
            math.floor((span + spacing) / (panel_size + spacing)),
        )
        occupied_span = panel_size * light_count
        min_gap_span = spacing * (light_count - 1)
        remaining_gap_span = max(span - occupied_span - min_gap_span, 0.0)
        edge_gap = remaining_gap_span / 2.0
        gap_size = spacing
        start = -span / 2.0 + gap_size + panel_size / 2.0
        start = -span / 2.0 + edge_gap + panel_size / 2.0
        step = panel_size + gap_size
        return [start + step * index for index in range(light_count)]

    x_positions = compute_axis_positions(width, light_width)
    y_positions = compute_axis_positions(length, light_height)

    for x_index, x_position in enumerate(x_positions):
        for y_index, y_position in enumerate(y_positions):
            light = UsdLux.RectLight.Define(
                stage,
                f"/Room/Lights/RectLight_{x_index}_{y_index}",
            )
            light.CreateWidthAttr(light_width)
            light.CreateHeightAttr(light_height)
            light.CreateDiffuseAttr(2.0)
            light.CreateIntensityAttr(ROOM_LIGHT_INTENSITY)
            light.CreateColorAttr(Gf.Vec3f(*NATURAL_LIGHT_COLOR))
            light.CreateExposureAttr(0.0)
            light_prim = light.GetPrim()
            light_prim.CreateAttribute(
                "visibleInPrimaryRay",
                Sdf.ValueTypeNames.Bool,
            ).Set(True)
            xform = UsdGeom.Xformable(light_prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(
                Gf.Vec3d(x_position, y_position, z_position)
            )
            # xform.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))

    return len(x_positions) * len(y_positions)


def create_ambient_light(stage: Any) -> None:
    dome_light = UsdLux.DomeLight.Define(stage, "/Room/Lights/AmbientLight")
    dome_light.CreateIntensityAttr(AMBIENT_LIGHT_INTENSITY)
    dome_light.CreateColorAttr(Gf.Vec3f(*NATURAL_LIGHT_COLOR))
    dome_light.CreateExposureAttr(0.0)


def create_room(
    output_path: Path,
    width: float,
    length: float,
    height: float,
    wall_thickness: float,
    material_preset: str,
    floor_only: bool,
    ceiling: bool,
    light_density: float,
    light_size: str,
    partition: bool,
) -> None:
    output_path = build_output_path(
        output_path.resolve(),
        width,
        length,
        height,
        partition,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    stage = Usd.Stage.CreateNew(str(output_path))
    stage.SetFramesPerSecond(60.0)
    stage.SetTimeCodesPerSecond(60.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    room = UsdGeom.Xform.Define(stage, "/Room")
    stage.SetDefaultPrim(room.GetPrim())
    UsdGeom.Scope.Define(stage, "/Room/Looks")
    UsdGeom.Scope.Define(stage, "/Room/Geometry")
    UsdGeom.Scope.Define(stage, "/Room/Lights")

    material = create_preview_material(
        stage,
        "/Room/Looks/RoomMaterial",
        MATERIAL_PRESETS[material_preset],
    )

    floor_thickness = wall_thickness
    add_box(
        stage,
        "/Room/Geometry/Floor",
        position=(0.0, 0.0, -floor_thickness / 2.0),
        scale=(width, length, floor_thickness),
        material=material,
    )

    if not floor_only:
        half_width = width / 2.0
        half_length = length / 2.0
        wall_z = height / 2.0
        add_box(
            stage,
            "/Room/Geometry/Wall_Positive_X",
            position=(half_width + wall_thickness / 2.0, 0.0, wall_z),
            scale=(wall_thickness, length + 2.0 * wall_thickness, height),
            material=material,
        )
        add_box(
            stage,
            "/Room/Geometry/Wall_Negative_X",
            position=(-half_width - wall_thickness / 2.0, 0.0, wall_z),
            scale=(wall_thickness, length + 2.0 * wall_thickness, height),
            material=material,
        )
        add_box(
            stage,
            "/Room/Geometry/Wall_Positive_Y",
            position=(0.0, half_length + wall_thickness / 2.0, wall_z),
            scale=(width, wall_thickness, height),
            material=material,
        )
        add_box(
            stage,
            "/Room/Geometry/Wall_Negative_Y",
            position=(0.0, -half_length - wall_thickness / 2.0, wall_z),
            scale=(width, wall_thickness, height),
            material=material,
        )

    if ceiling:
        add_box(
            stage,
            "/Room/Geometry/Ceiling",
            position=(0.0, 0.0, height + wall_thickness / 2.0),
            scale=(width, length, wall_thickness),
            material=material,
        )

    if partition:
        create_partition_wall(stage, material)

    light_count = create_rect_lights(
        stage,
        width=width,
        length=length,
        height=height,
        light_density=light_density,
        light_size=light_size,
    )
    # create_ambient_light(stage)

    stage.GetRootLayer().Save()
    print(f"Wrote: {output_path}")
    print(f"Dimensions: width={width}, length={length}, height={height}")
    print(f"Material preset: {material_preset}")
    print(f"Walls: {'no' if floor_only else 'yes'}")
    print(f"Ceiling: {'yes' if ceiling else 'no'}")
    print(f"Partition: {'yes' if partition else 'no'}")
    if partition:
        print(
            "Partition door: "
            f"width={DOOR_WIDTH}, height={DOOR_HEIGHT}, "
            f"thickness={PARTITION_THICKNESS}, position={DOOR_POSITION}"
        )
    light_width, light_height = LIGHT_SIZE_PRESETS[light_size]
    print(
        f"Light size preset: {light_size} ({light_width}m x {light_height}m)"
    )
    print(f"Ceiling rect lights: {light_count} (density={light_density}m)")


def main() -> None:
    args = parse_args()
    sys.argv = [sys.argv[0]]
    initialize_usd_runtime()
    try:
        create_room(
            output_path=args.output,
            width=args.width,
            length=args.length,
            height=args.height,
            wall_thickness=args.wall_thickness,
            material_preset=args.material_preset,
            floor_only=args.floor_only,
            ceiling=args.ceiling,
            light_density=args.light_density,
            light_size=args.light_size,
            partition=args.partition,
        )
    finally:
        if SIMULATION_APP:
            SIMULATION_APP.close()


if __name__ == "__main__":
    main()
