#!/usr/bin/env python3
"""Compose the 11-table workshop scene from assets using only USD/PXR."""

from __future__ import annotations

import argparse
import os
import random
import sys
from collections.abc import Iterable
from dataclasses import dataclass
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
UsdPhysics: Any = None
UsdShade: Any = None
SIMULATION_APP: Any = None

ROOT_DIR = Path(__file__).resolve().parent
ASSET_DIR = asset_path()
DEFAULT_OUTPUT = ASSET_DIR / "complete_scene_11_tables.usd"

LETTERS = tuple("ABCDEFGHI")
CUTLERY = ("bowl", "plate", "spoon")
IDENTITY_ROTATION = (1.0, 0.0, 0.0, 0.0)
LETTER_ROTATION = (0.70710678, -0.70710678, 0.0, 0.0)
LETTER_HEIGHT_OFFSET = -0.0097

TABLE_LAYOUT = {
    "Table_Left_1": (-2.0, 3.0, 0.0),
    "Table_Left_2": (-2.0, 1.5, 0.0),
    "Table_Left_3": (-2.0, 0.0, 0.0),
    "Table_Left_4": (-2.0, -1.5, 0.0),
    "Table_Left_5": (-2.0, -3.0, 0.0),
    "Table_Right_1": (2.0, 3.0, 0.0),
    "Table_Right_2": (2.0, 1.5, 0.0),
    "Table_Right_3": (2.0, 0.0, 0.0),
    "Table_Right_4": (2.0, -1.5, 0.0),
    "Table_Right_5": (2.0, -3.0, 0.0),
    "Table_Top_Center": (0.0, 4.5, 0.0),
    "Table_Bottom_Center": (0.0, -4.5, 0.0),
}

LETTER_TABLES = {
    "A": "Table_Left_2",
    "B": "Table_Right_2",
    "C": "Table_Left_3",
    "D": "Table_Right_3",
    "E": "Table_Left_4",
    "F": "Table_Right_4",
    "G": "Table_Left_5",
    "H": "Table_Right_5",
    "I": "Table_Bottom_Center",
}

CUTLERY_TABLE = "Table_Left_1"
CUTLERY_LAYOUT = {
    "bowl": {
        "offset": (0.0, 0.0, 0.77),
    },
    "plate": {
        "offset": (0.0, 0.0, 0.77),
    },
    "spoon": {
        "offset": (0.0, 0.0, 0.77),
    },
}

MATERIALS = {
    "Black": ((0.0, 0.0, 0.0), 0.0, 0.8),
}

REFERENCE_PRIM_PATHS: dict[Path, str | None] = {}


def environment_help_text() -> str:
    return (
        "Environment USD path. Use 'none' or provide a USD file path. "
        "If the file does not exist, no environment is added."
    )


def resolve_environment_usd(selection: str) -> Path | None:
    candidate = Path(selection).expanduser()
    if not candidate.is_absolute():
        candidate = (ROOT_DIR / candidate).resolve()
    if candidate.is_file():
        return candidate
    return None


def initialize_usd_runtime(preview: bool) -> None:
    """Load pxr modules, starting Isaac Sim first when required."""
    global Gf, Sdf, SIMULATION_APP, Usd, UsdGeom, UsdPhysics, UsdShade

    try:
        from pxr import Gf as pxr_gf
        from pxr import Sdf as pxr_sdf
        from pxr import Usd as pxr_usd
        from pxr import UsdGeom as pxr_usd_geom
        from pxr import UsdPhysics as pxr_usd_physics
        from pxr import UsdShade as pxr_usd_shade
    except ImportError:
        from isaacsim import SimulationApp

        SIMULATION_APP = SimulationApp({"headless": not preview})
        from pxr import Gf as pxr_gf
        from pxr import Sdf as pxr_sdf
        from pxr import Usd as pxr_usd
        from pxr import UsdGeom as pxr_usd_geom
        from pxr import UsdPhysics as pxr_usd_physics
        from pxr import UsdShade as pxr_usd_shade

    Gf = pxr_gf
    Sdf = pxr_sdf
    Usd = pxr_usd
    UsdGeom = pxr_usd_geom
    UsdPhysics = pxr_usd_physics
    UsdShade = pxr_usd_shade


@dataclass(frozen=True)
class AssetPlacement:
    """A USD reference plus its scene transform."""

    name: str
    usd_path: Path
    prim_path: str
    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float] = IDENTITY_ROTATION
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    material_name: str | None = None
    physics: str = "none"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview the 11-table scene in Isaac Sim."
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=(
            f"USD file to write when --save is set. Default: {DEFAULT_OUTPUT}"
        ),
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save the composed scene to --output.",
    )
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Do not keep the Isaac Sim preview window open after composing.",
    )
    parser.add_argument(
        "--asset-scale",
        type=float,
        default=1.0,
        help="Uniform scale applied to all referenced assets.",
    )
    parser.add_argument(
        "--table-z",
        type=float,
        default=0.0,
        help="Base Z coordinate for all table placements.",
    )
    parser.add_argument(
        "--tabletop-z-offset",
        type=float,
        default=0.76,
        help="Vertical item offset above each table base.",
    )
    parser.add_argument(
        "--env",
        default=None,
        help=environment_help_text(),
    )
    parser.add_argument(
        "--randomize-cutlery-color",
        action="store_true",
        help="Apply random preview colors to cutlery assets.",
    )
    return parser.parse_args()


def require_files(paths: Iterable[Path]) -> None:
    missing = [path for path in paths if not path.is_file()]
    if missing:
        missing_list = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"Missing USD asset(s):\n{missing_list}")


def relative_reference(asset_path: Path, output_path: Path | None) -> str:
    if output_path is None:
        return str(asset_path.resolve())
    return Path(
        os.path.relpath(asset_path.resolve(), output_path.resolve().parent)
    ).as_posix()


def reference_prim_path(asset_path: Path) -> str | None:
    resolved_path = asset_path.resolve()
    if resolved_path in REFERENCE_PRIM_PATHS:
        return REFERENCE_PRIM_PATHS[resolved_path]

    asset_stage = Usd.Stage.Open(str(resolved_path))
    if asset_stage is None:
        raise ValueError(f"Could not open USD asset: {resolved_path}")

    default_prim = asset_stage.GetDefaultPrim()
    if default_prim and default_prim.IsValid():
        prim_path = str(default_prim.GetPath())
    else:
        root_prims = [
            prim
            for prim in asset_stage.GetPseudoRoot().GetChildren()
            if prim.IsDefined()
        ]
        prim_path = None
        if len(root_prims) == 1:
            prim_path = str(root_prims[0].GetPath())

    REFERENCE_PRIM_PATHS[resolved_path] = prim_path
    return prim_path


def add_xyz(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def create_preview_material(
    stage: Any,
    path: str,
    diffuse_color: tuple[float, float, float],
    metallic: float,
    roughness: float,
) -> Any:
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


def create_materials(stage: Any) -> dict[str, Any]:
    UsdGeom.Scope.Define(stage, "/World/Looks")
    return {
        name: create_preview_material(
            stage,
            f"/World/Looks/{name}",
            diffuse_color=color,
            metallic=metallic,
            roughness=roughness,
        )
        for name, (color, metallic, roughness) in MATERIALS.items()
    }


def random_cutlery_material_name(item_name: str) -> str:
    return f"RandomCutlery_{item_name}"


def ensure_random_cutlery_materials(
    stage: Any,
    materials: dict[str, Any],
) -> None:
    for item_name in CUTLERY:
        material_name = random_cutlery_material_name(item_name)
        if material_name in materials:
            continue

        diffuse_color = (
            random.uniform(0.15, 1.0),
            random.uniform(0.15, 1.0),
            random.uniform(0.15, 1.0),
        )
        materials[material_name] = create_preview_material(
            stage,
            f"/World/Looks/{material_name}",
            diffuse_color=diffuse_color,
            metallic=0.2,
            roughness=0.4,
        )


def set_xform(
    prim: Any,
    position: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
    scale: tuple[float, float, float],
) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddOrientOp().Set(
        Gf.Quatf(rotation[0], rotation[1], rotation[2], rotation[3])
    )
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def bind_material_to_gprims(
    root_prim: Any,
    material: Any,
) -> None:
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Gprim):
            binding_api = UsdShade.MaterialBindingAPI.Apply(prim)
            binding_api.Bind(
                material,
                UsdShade.Tokens.strongerThanDescendants,
            )


def apply_physics(root_prim: Any, physics: str) -> None:
    """Apply collision and optional rigid body physics to an asset root."""
    if physics == "none":
        return

    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Gprim):
            UsdPhysics.CollisionAPI.Apply(prim)

    if physics == "rigid":
        UsdPhysics.RigidBodyAPI.Apply(root_prim)
        mass_api = UsdPhysics.MassAPI.Apply(root_prim)
        mass_api.CreateMassAttr(0.2)


def add_referenced_asset(
    stage: Any,
    output_path: Path | None,
    placement: AssetPlacement,
    materials: dict[str, Any],
) -> Any:
    prim = UsdGeom.Xform.Define(stage, placement.prim_path).GetPrim()
    asset_reference = relative_reference(placement.usd_path, output_path)
    asset_prim_path = reference_prim_path(placement.usd_path)

    if asset_prim_path:
        prim.GetReferences().AddReference(
            asset_reference,
            Sdf.Path(asset_prim_path),
        )
    else:
        prim.GetReferences().AddReference(asset_reference)

    set_xform(prim, placement.position, placement.rotation, placement.scale)

    if placement.material_name:
        bind_material_to_gprims(prim, materials[placement.material_name])

    apply_physics(prim, placement.physics)

    return prim


def table_layout(table_z: float) -> dict[str, tuple[float, float, float]]:
    return {
        name: (pos[0], pos[1], table_z) for name, pos in TABLE_LAYOUT.items()
    }


def table_placements(
    layout: dict[str, tuple[float, float, float]],
    asset_scale: tuple[float, float, float],
) -> list[AssetPlacement]:
    return [
        AssetPlacement(
            name=name,
            usd_path=ASSET_DIR / "table_edit.usd",
            prim_path=f"/World/Scene/{name}",
            position=position,
            scale=asset_scale,
            physics="none",
        )
        for name, position in layout.items()
    ]


def environment_placement(environment: str | None) -> AssetPlacement | None:
    if environment in (None, "none"):
        return None

    environment_usd = resolve_environment_usd(environment)
    if environment_usd is None:
        print(
            f"Environment USD not found: {environment}. "
            "Continuing without environment."
        )
        return None

    return AssetPlacement(
        name=environment_usd.stem,
        usd_path=environment_usd,
        prim_path=f"/World/Environment/{environment_usd.stem}",
        position=(0.0, 0.0, 0.0),
        physics="collision",
    )


def letter_placements(
    layout: dict[str, tuple[float, float, float]],
    offset: tuple[float, float, float],
    asset_scale: tuple[float, float, float],
) -> list[AssetPlacement]:
    return [
        AssetPlacement(
            name=f"Letter_{letter}",
            usd_path=ASSET_DIR / f"{letter}_edit.usd",
            prim_path=f"/World/Scene/Letter_{letter}",
            position=add_xyz(layout[table_name], offset),
            rotation=LETTER_ROTATION,
            scale=asset_scale,
            material_name="Black",
        )
        for letter, table_name in LETTER_TABLES.items()
    ]


def cutlery_placements(
    layout: dict[str, tuple[float, float, float]],
    tabletop_z_offset: float,
    asset_scale: tuple[float, float, float],
    randomize_color: bool,
) -> list[AssetPlacement]:
    placements = []
    base_position = layout[CUTLERY_TABLE]
    for item_name, item_cfg in CUTLERY_LAYOUT.items():
        x_offset, y_offset, z_offset = item_cfg["offset"]
        random_xy_offset = (
            random.uniform(-0.15, 0.15),
            random.uniform(-0.15, 0.15),
            0.0,
        )
        adjusted_offset = (
            x_offset + random_xy_offset[0],
            y_offset + random_xy_offset[1],
            tabletop_z_offset + (z_offset - 0.76),
        )
        placements.append(
            AssetPlacement(
                name=item_name.capitalize(),
                usd_path=ASSET_DIR / f"{item_name}.usd",
                prim_path=f"/World/Scene/{item_name.capitalize()}",
                position=add_xyz(base_position, adjusted_offset),
                scale=asset_scale,
                material_name=(
                    random_cutlery_material_name(item_name)
                    if randomize_color
                    else None
                ),
            )
        )
    return placements


def add_physics_scene(stage: Any) -> None:
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)


def compose_scene(
    output_path: Path,
    asset_scale: float,
    table_z: float,
    tabletop_z_offset: float,
    save: bool,
    preview: bool,
    environment: str | None,
    randomize_cutlery_color: bool,
) -> Any:
    if save:
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

    layout = table_layout(table_z)
    scale = (asset_scale, asset_scale, asset_scale)
    letter_offset = (0.0, 0.0, tabletop_z_offset + LETTER_HEIGHT_OFFSET)
    scene_placements = (
        table_placements(layout, scale)
        + letter_placements(layout, letter_offset, scale)
        + cutlery_placements(
            layout,
            tabletop_z_offset,
            scale,
            randomize_cutlery_color,
        )
    )
    env_placement = environment_placement(environment)
    placements = [*scene_placements]
    if env_placement:
        placements.insert(0, env_placement)
    require_files(placement.usd_path for placement in placements)

    if preview:
        import omni.usd

        context = omni.usd.get_context()
        context.new_stage()
        if SIMULATION_APP:
            SIMULATION_APP.update()
        stage = context.get_stage()
        reference_base = None
    elif save:
        stage = Usd.Stage.CreateNew(str(output_path))
        reference_base = output_path
    else:
        stage = Usd.Stage.CreateInMemory()
        reference_base = None

    if stage is None:
        raise RuntimeError("Could not create a USD stage for the scene.")

    stage.SetFramesPerSecond(60.0)
    stage.SetTimeCodesPerSecond(60.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, "/World/Environment")
    UsdGeom.Scope.Define(stage, "/World/Scene")

    materials = create_materials(stage)
    if randomize_cutlery_color:
        ensure_random_cutlery_materials(stage, materials)
    add_physics_scene(stage)

    for placement in placements:
        add_referenced_asset(stage, reference_base, placement, materials)

    if save and preview:
        stage.GetRootLayer().Export(str(output_path))
        print(f"Wrote: {output_path}")
    elif save:
        stage.GetRootLayer().Save()
        print(f"Wrote: {output_path}")
    else:
        print(
            "Preview scene composed in memory. Use --save to write a USD file."
        )
    print(f"Asset directory: {ASSET_DIR}")
    print(f"Asset scale: {scale}")
    print(f"Rotation applied to assets: {IDENTITY_ROTATION}")
    environment_path = env_placement.usd_path if env_placement else "none"
    print(f"Environment: {environment_path}")
    print(f"Tables: {len(TABLE_LAYOUT)}")
    print(f"Letters: {len(LETTER_TABLES)}")
    print(f"Cutlery: {len(CUTLERY_LAYOUT)}")
    return stage


def open_preview() -> None:
    print("Preview is open. Close the Isaac Sim window to exit.")
    while SIMULATION_APP and SIMULATION_APP.is_running():
        SIMULATION_APP.update()


def main() -> None:
    args = parse_args()
    sys.argv = [sys.argv[0]]
    preview = not args.no_preview
    initialize_usd_runtime(preview)
    compose_scene(
        output_path=args.output,
        asset_scale=args.asset_scale,
        table_z=args.table_z,
        tabletop_z_offset=args.tabletop_z_offset,
        save=args.save,
        preview=preview,
        environment=args.env,
        randomize_cutlery_color=args.randomize_cutlery_color,
    )
    if preview:
        open_preview()


if __name__ == "__main__":
    main()
