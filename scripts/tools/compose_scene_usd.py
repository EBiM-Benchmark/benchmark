#!/usr/bin/env python3
"""Compose the tabletop task scene as USD without importing Isaac Lab.

Run this with Isaac Sim's Python, or any Python that has Pixar USD's ``pxr``
module available. The default output is a static environment USD that Isaac
Lab can later reference as one asset while the robot remains a separate USD.
"""

from __future__ import annotations

import argparse
import math
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

from path_utils import asset_path, franka_urdf_path

Gf: Any = None
Sdf: Any = None
Usd: Any = None
UsdGeom: Any = None
UsdLux: Any = None
UsdPhysics: Any = None
UsdShade: Any = None
PhysxSchema: Any = None
SIMULATION_APP: Any = None


ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
ASSETS = asset_path()
DEFAULT_OUTPUT = ASSETS / "tabletop_task_scene.usd"

TABLE_USD = asset_path("table_edit.usd")
BOWL_USD = asset_path("bowl2.usd")
PLATE_USD = asset_path("plate2.usd")
SPOON_USD = asset_path("spoon2.usd")
HEAD_USD = asset_path("Collected_head/head.usd")
ROBOT_USD = franka_urdf_path("mobile_fr3_duo_v0_2_franka_hand.usd")

IDENTITY_ROT = (1.0, 0.0, 0.0, 0.0)
LETTER_ROT = (0.70710678, -0.70710678, 0.0, 0.0)
ROT_Z_90 = (0.70710678, 0.0, 0.0, 0.70710678)
ROT_Z_180 = (0.0, 0.0, 0.0, 1.0)
ASSET_SCALE = (1.0, 1.0, 1.0)
TABLETOP_Z_OFFSET = 0.76
HEAD_Y_OFFSET = -0.3
LETTER_HEIGHT_OFFSET = -0.0097
DEFAULT_BEAN_COLOR = (0.20, 0.12, 0.07)
DEFAULT_BEAN_COUNT = 150
DEFAULT_BEAN_DENSITY = 850.0
PHYSX_SCHEMA_WARNING_EMITTED = False

BEAN_PHYSICS = {
    "radius": 0.0025,
    "half_height": 0.0016,
    "spawn_height": 0.01,
    "spawn_wall_thickness": 0.016,
    "spawn_spacing_scale": 1.2,
    "particle_gap": 0.006,
    "friction": 0.55,
    "restitution": 0.02,
}


@dataclass(frozen=True)
class AssetPlacement:
    name: str
    usd_path: Path
    prim_path: str
    pos: tuple[float, float, float]
    rot: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    material_name: str | None = None
    as_payload: bool = False


@dataclass(frozen=True)
class PhysxSceneTuning:
    """Script-level PhysX scene defaults for dense bean simulation."""

    enable_gpu_dynamics: bool = True
    broadphase_type: str = "GPU"
    collision_system: str = "PCM"
    solver_type: str = "TGS"
    enable_ccd: bool = False
    enable_stabilization: bool = False
    enable_enhanced_determinism: bool = False
    enable_scene_query_support: bool = False
    gpu_max_rigid_contact_count: int = 2**23
    gpu_max_rigid_patch_count: int = 5 * 2**15
    gpu_found_lost_pairs_capacity: int = 2**21
    gpu_found_lost_aggregate_pairs_capacity: int = 2**25
    gpu_total_aggregate_pairs_capacity: int = 2**21
    gpu_collision_stack_size: int = 2**26
    gpu_heap_capacity: int = 2**26
    gpu_temp_buffer_capacity: int = 2**24
    gpu_max_num_partitions: int = 8

    def __post_init__(self) -> None:
        if self.gpu_max_num_partitions < 1:
            raise ValueError("gpu_max_num_partitions must be at least 1.")
        if self.gpu_max_num_partitions > 32:
            raise ValueError("gpu_max_num_partitions must be 32 or lower.")
        if self.gpu_max_num_partitions & (self.gpu_max_num_partitions - 1):
            raise ValueError("gpu_max_num_partitions must be a power of two.")


@dataclass(frozen=True)
class BeanSimulationTuning:
    """Script-level per-bean PhysX defaults for faster dense contacts."""

    solver_position_iterations: int = 4
    solver_velocity_iterations: int = 0
    contact_offset: float = 0.0005
    rest_offset: float = 0.0
    sleep_threshold: float = 1.0e-3
    linear_damping: float = 0.05
    angular_damping: float = 0.1
    max_depenetration_velocity: float = 1.0

    def __post_init__(self) -> None:
        if self.contact_offset < self.rest_offset:
            raise ValueError("contact_offset must be >= rest_offset.")
        if self.solver_position_iterations < 1:
            raise ValueError("solver_position_iterations must be at least 1.")
        if self.solver_velocity_iterations < 0:
            raise ValueError(
                "solver_velocity_iterations must be non-negative."
            )


TABLES = [
    ("Table_Left_1", (-2.0, 3.0, 0.0)),
    ("Table_Left_2", (-2.0, 1.5, 0.0)),
    ("Table_Left_3", (-2.0, 0.0, 0.0)),
    ("Table_Left_4", (-2.0, -1.5, 0.0)),
    ("Table_Left_5", (-2.0, -3.0, 0.0)),
    ("Table_Right_1", (2.0, 3.0, 0.0)),
    ("Table_Right_2", (2.0, 1.5, 0.0)),
    ("Table_Right_3", (2.0, 0.0, 0.0)),
    ("Table_Right_4", (2.0, -1.5, 0.0)),
    ("Table_Right_5", (2.0, -3.0, 0.0)),
    ("Table_Bottom_Center", (0.0, -4.5, 0.0)),
]

TOP_CENTER_TABLE = ("Table_Top_Center", (0.0, 4.5, 0.0))

LETTER_TABLE_POS = {
    "A": (-2.0, 1.5, 0.0),
    "B": (2.0, 1.5, 0.0),
    "C": (-2.0, 0.0, 0.0),
    "D": (2.0, 0.0, 0.0),
    "E": (-2.0, -1.5, 0.0),
    "F": (2.0, -1.5, 0.0),
    "G": (-2.0, -3.0, 0.0),
    "H": (2.0, -3.0, 0.0),
    "I": (0.0, -4.5, 0.0),
}

CUTLERY_TABLE_POS = (-2.0, 3.0, 0.0)
CUTLERY = {
    "bowl": {
        "usd_path": BOWL_USD,
        "offset": (0.0, 0.0, TABLETOP_Z_OFFSET),
        "fixed_pos": (-1.8, 2.75, 0.755),
        "fixed_rot": IDENTITY_ROT,
    },
    "plate": {
        "usd_path": PLATE_USD,
        "offset": (0.0, 0.0, TABLETOP_Z_OFFSET),
        "fixed_pos": (-1.8, 3.0, 0.755),
        "fixed_rot": IDENTITY_ROT,
    },
    "spoon": {
        "usd_path": SPOON_USD,
        "offset": (0.0, 0.0, TABLETOP_Z_OFFSET),
        "fixed_pos": (-1.8, 3.2, 0.765),
        "fixed_rot": ROT_Z_180,
    },
}

MATERIALS = {
    "Ground": ((0.48, 0.50, 0.52), 0.0, 0.75),
    "Black": ((0.0, 0.0, 0.0), 0.0, 0.8),
}

REFERENCE_PRIM_PATHS: dict[Path, str | None] = {}


def environment_help_text() -> str:
    return (
        "Environment USD path. Use 'none' or provide a USD file path. "
        "Relative paths are resolved from the workspace root."
    )


def resolve_environment_usd(selection: str) -> Path | None:
    candidate = Path(selection).expanduser()
    if not candidate.is_absolute():
        candidate = (WORKSPACE_ROOT / candidate).resolve()
    if candidate.is_file():
        return candidate
    return None


def initialize_usd_runtime(preview: bool) -> None:
    """Load pxr modules, starting Isaac Sim first when required."""
    global Gf, PhysxSchema, Sdf, SIMULATION_APP, Usd, UsdGeom, UsdLux
    global UsdPhysics, UsdShade

    if preview:
        try:
            from isaacsim import SimulationApp
        except ImportError as exc:
            raise SystemExit(
                "Could not import isaacsim for --preview. Run this with "
                "Python inside one of the Isaac Sim Docker runtimes."
            ) from exc

        SIMULATION_APP = SimulationApp({"headless": False})
        from pxr import Gf as pxr_gf
        from pxr import Sdf as pxr_sdf
        from pxr import Usd as pxr_usd
        from pxr import UsdGeom as pxr_usd_geom
        from pxr import UsdLux as pxr_usd_lux
        from pxr import UsdPhysics as pxr_usd_physics
        from pxr import UsdShade as pxr_usd_shade

        try:
            from pxr import PhysxSchema as pxr_physx_schema
        except ImportError:
            pxr_physx_schema = None

        Gf = pxr_gf
        PhysxSchema = pxr_physx_schema
        Sdf = pxr_sdf
        Usd = pxr_usd
        UsdGeom = pxr_usd_geom
        UsdLux = pxr_usd_lux
        UsdPhysics = pxr_usd_physics
        UsdShade = pxr_usd_shade
        return

    try:
        from pxr import Gf as pxr_gf
        from pxr import Sdf as pxr_sdf
        from pxr import Usd as pxr_usd
        from pxr import UsdGeom as pxr_usd_geom
        from pxr import UsdLux as pxr_usd_lux
        from pxr import UsdPhysics as pxr_usd_physics
        from pxr import UsdShade as pxr_usd_shade

        try:
            from pxr import PhysxSchema as pxr_physx_schema
        except ImportError:
            pxr_physx_schema = None
    except ImportError:
        try:
            from isaacsim import SimulationApp
        except ImportError as exc:
            raise SystemExit(
                "Could not import pxr or isaacsim. Run this with Python "
                "inside one of the Docker runtimes, for example:\n"
                "  python scripts/tools/compose_scene_usd.py"
            ) from exc

        SIMULATION_APP = SimulationApp({"headless": not preview})
        from pxr import Gf as pxr_gf
        from pxr import Sdf as pxr_sdf
        from pxr import Usd as pxr_usd
        from pxr import UsdGeom as pxr_usd_geom
        from pxr import UsdLux as pxr_usd_lux
        from pxr import UsdPhysics as pxr_usd_physics
        from pxr import UsdShade as pxr_usd_shade

        try:
            from pxr import PhysxSchema as pxr_physx_schema
        except ImportError:
            pxr_physx_schema = None

    Gf = pxr_gf
    PhysxSchema = pxr_physx_schema
    Sdf = pxr_sdf
    Usd = pxr_usd
    UsdGeom = pxr_usd_geom
    UsdLux = pxr_usd_lux
    UsdPhysics = pxr_usd_physics
    UsdShade = pxr_usd_shade


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compose the workshop table scene as USD without Isaac Lab."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="USD file to write when --save is set.",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save the composed scene to --output.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Visualize the composed scene in Isaac Sim.",
    )
    parser.add_argument(
        "--include-top-table",
        action="store_true",
        help=(
            "Add the top-center table at (0, 4.5, 0.7). Without this flag, "
            "that area is left open for the robot."
        ),
    )
    parser.add_argument(
        "--with-robot",
        action="store_true",
        help=(
            "Also reference the robot USD at /World/Robot for GUI validation. "
            "For Isaac Lab control, importing the robot separately is usually "
            "cleaner."
        ),
    )
    parser.add_argument(
        "--env",
        default="none",
        help=environment_help_text(),
    )
    parser.add_argument(
        "--randomize-cutlery-color",
        action="store_true",
        help="Apply random preview colors to cutlery assets.",
    )
    parser.add_argument(
        "--randomize-cutlery-placement",
        action="store_true",
        help=(
            "Randomize cutlery placement around the cutlery table. By "
            "default, fixed poses are used."
        ),
    )
    parser.add_argument(
        "--add-head",
        action="store_true",
        help="Add one head asset on top of each table.",
    )
    parser.add_argument(
        "--bean-count",
        type=int,
        default=DEFAULT_BEAN_COUNT,
        help="Number of coffee bean rigid bodies to place in the bowl.",
    )
    parser.add_argument(
        "--bean-color",
        type=float,
        nargs=3,
        default=DEFAULT_BEAN_COLOR,
        metavar=("R", "G", "B"),
        help="Coffee bean RGB color as three floats in [0, 1].",
    )
    parser.add_argument(
        "--bean-density",
        type=float,
        default=DEFAULT_BEAN_DENSITY,
        help="Coffee bean density used by USD physics mass properties.",
    )
    return parser.parse_args()


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
        prim_path = (
            str(root_prims[0].GetPath()) if len(root_prims) == 1 else None
        )

    REFERENCE_PRIM_PATHS[resolved_path] = prim_path
    return prim_path


def require_files(paths: Iterable[Path]) -> None:
    missing = [path for path in paths if not path.is_file()]
    if missing:
        missing_list = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"Missing USD asset(s):\n{missing_list}")


def create_preview_material(
    stage: Any,
    path: str,
    diffuse_color: tuple[float, float, float],
    metallic: float = 0.0,
    roughness: float = 0.5,
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


def apply_physics_material(
    material: Any,
    friction: float,
    restitution: float,
) -> None:
    physics_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_api.CreateStaticFrictionAttr(friction)
    physics_api.CreateDynamicFrictionAttr(friction)
    physics_api.CreateRestitutionAttr(restitution)


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
    pos: tuple[float, float, float],
    rot: tuple[float, float, float, float],
    scale: tuple[float, float, float],
) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(rot[0], rot[1], rot[2], rot[3])
    )
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))


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


def add_referenced_asset(
    stage: Any,
    output_path: Path | None,
    placement: AssetPlacement,
    materials: dict[str, Any],
) -> Any:
    prim = UsdGeom.Xform.Define(stage, placement.prim_path).GetPrim()
    set_xform(prim, placement.pos, placement.rot, placement.scale)

    asset_reference = relative_reference(placement.usd_path, output_path)
    asset_prim_path = reference_prim_path(placement.usd_path)
    if placement.as_payload:
        payloads = prim.GetPayloads()
        if asset_prim_path:
            payloads.AddPayload(asset_reference, Sdf.Path(asset_prim_path))
        else:
            payloads.AddPayload(asset_reference)
    else:
        references = prim.GetReferences()
        if asset_prim_path:
            references.AddReference(asset_reference, Sdf.Path(asset_prim_path))
        else:
            references.AddReference(asset_reference)

    if placement.material_name:
        bind_material_to_gprims(prim, materials[placement.material_name])

    return prim


def add_ground(stage: Any, materials: dict[str, Any]) -> None:
    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    ground_prim = ground.GetPrim()
    set_xform(
        ground_prim,
        pos=(0.0, 0.0, -0.01),
        rot=(1.0, 0.0, 0.0, 0.0),
        scale=(10.0, 10.0, 0.01),
    )
    UsdPhysics.CollisionAPI.Apply(ground_prim)
    UsdShade.MaterialBindingAPI.Apply(ground_prim).Bind(materials["Ground"])


def warn_physx_schema_missing() -> None:
    global PHYSX_SCHEMA_WARNING_EMITTED

    if PHYSX_SCHEMA_WARNING_EMITTED:
        return
    print(
        "Warning: pxr.PhysxSchema is unavailable; skipping extended PhysX "
        "performance tuning attributes."
    )
    PHYSX_SCHEMA_WARNING_EMITTED = True


def create_schema_attr(
    api: Any,
    prim: Any,
    create_method_name: str,
    attr_name: str,
    value: Any,
    value_type: Any,
) -> None:
    create_attr = getattr(api, create_method_name, None)
    if create_attr:
        create_attr(value)
        return

    prim.CreateAttribute(attr_name, value_type).Set(value)


def apply_physx_scene_tuning(prim: Any, tuning: PhysxSceneTuning) -> None:
    if PhysxSchema is None:
        warn_physx_schema_missing()
        return

    scene_api = PhysxSchema.PhysxSceneAPI.Apply(prim)
    attr_specs = (
        (
            "CreateEnableGPUDynamicsAttr",
            "physxScene:enableGPUDynamics",
            tuning.enable_gpu_dynamics,
            Sdf.ValueTypeNames.Bool,
        ),
        (
            "CreateBroadphaseTypeAttr",
            "physxScene:broadphaseType",
            tuning.broadphase_type,
            Sdf.ValueTypeNames.Token,
        ),
        (
            "CreateCollisionSystemAttr",
            "physxScene:collisionSystem",
            tuning.collision_system,
            Sdf.ValueTypeNames.Token,
        ),
        (
            "CreateSolverTypeAttr",
            "physxScene:solverType",
            tuning.solver_type,
            Sdf.ValueTypeNames.Token,
        ),
        (
            "CreateEnableCCDAttr",
            "physxScene:enableCCD",
            tuning.enable_ccd,
            Sdf.ValueTypeNames.Bool,
        ),
        (
            "CreateEnableStabilizationAttr",
            "physxScene:enableStabilization",
            tuning.enable_stabilization,
            Sdf.ValueTypeNames.Bool,
        ),
        (
            "CreateEnableEnhancedDeterminismAttr",
            "physxScene:enableEnhancedDeterminism",
            tuning.enable_enhanced_determinism,
            Sdf.ValueTypeNames.Bool,
        ),
        (
            "CreateEnableSceneQuerySupportAttr",
            "physxScene:enableSceneQuerySupport",
            tuning.enable_scene_query_support,
            Sdf.ValueTypeNames.Bool,
        ),
        (
            "CreateGpuMaxRigidContactCountAttr",
            "physxScene:gpuMaxRigidContactCount",
            tuning.gpu_max_rigid_contact_count,
            Sdf.ValueTypeNames.UInt,
        ),
        (
            "CreateGpuMaxRigidPatchCountAttr",
            "physxScene:gpuMaxRigidPatchCount",
            tuning.gpu_max_rigid_patch_count,
            Sdf.ValueTypeNames.UInt,
        ),
        (
            "CreateGpuFoundLostPairsCapacityAttr",
            "physxScene:gpuFoundLostPairsCapacity",
            tuning.gpu_found_lost_pairs_capacity,
            Sdf.ValueTypeNames.UInt,
        ),
        (
            "CreateGpuFoundLostAggregatePairsCapacityAttr",
            "physxScene:gpuFoundLostAggregatePairsCapacity",
            tuning.gpu_found_lost_aggregate_pairs_capacity,
            Sdf.ValueTypeNames.UInt,
        ),
        (
            "CreateGpuTotalAggregatePairsCapacityAttr",
            "physxScene:gpuTotalAggregatePairsCapacity",
            tuning.gpu_total_aggregate_pairs_capacity,
            Sdf.ValueTypeNames.UInt,
        ),
        (
            "CreateGpuCollisionStackSizeAttr",
            "physxScene:gpuCollisionStackSize",
            tuning.gpu_collision_stack_size,
            Sdf.ValueTypeNames.UInt64,
        ),
        (
            "CreateGpuHeapCapacityAttr",
            "physxScene:gpuHeapCapacity",
            tuning.gpu_heap_capacity,
            Sdf.ValueTypeNames.UInt64,
        ),
        (
            "CreateGpuTempBufferCapacityAttr",
            "physxScene:gpuTempBufferCapacity",
            tuning.gpu_temp_buffer_capacity,
            Sdf.ValueTypeNames.UInt64,
        ),
        (
            "CreateGpuMaxNumPartitionsAttr",
            "physxScene:gpuMaxNumPartitions",
            tuning.gpu_max_num_partitions,
            Sdf.ValueTypeNames.UInt,
        ),
    )
    for create_method_name, attr_name, value, value_type in attr_specs:
        create_schema_attr(
            scene_api,
            prim,
            create_method_name,
            attr_name,
            value,
            value_type,
        )


def apply_bean_physx_tuning(prim: Any, tuning: BeanSimulationTuning) -> None:
    if PhysxSchema is None:
        warn_physx_schema_missing()
        return

    rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    rigid_attr_specs = (
        (
            "CreateSolverPositionIterationCountAttr",
            "physxRigidBody:solverPositionIterationCount",
            tuning.solver_position_iterations,
            Sdf.ValueTypeNames.Int,
        ),
        (
            "CreateSolverVelocityIterationCountAttr",
            "physxRigidBody:solverVelocityIterationCount",
            tuning.solver_velocity_iterations,
            Sdf.ValueTypeNames.Int,
        ),
        (
            "CreateSleepThresholdAttr",
            "physxRigidBody:sleepThreshold",
            tuning.sleep_threshold,
            Sdf.ValueTypeNames.Float,
        ),
        (
            "CreateLinearDampingAttr",
            "physxRigidBody:linearDamping",
            tuning.linear_damping,
            Sdf.ValueTypeNames.Float,
        ),
        (
            "CreateAngularDampingAttr",
            "physxRigidBody:angularDamping",
            tuning.angular_damping,
            Sdf.ValueTypeNames.Float,
        ),
        (
            "CreateMaxDepenetrationVelocityAttr",
            "physxRigidBody:maxDepenetrationVelocity",
            tuning.max_depenetration_velocity,
            Sdf.ValueTypeNames.Float,
        ),
    )
    collision_attr_specs = (
        (
            "CreateContactOffsetAttr",
            "physxCollision:contactOffset",
            tuning.contact_offset,
            Sdf.ValueTypeNames.Float,
        ),
        (
            "CreateRestOffsetAttr",
            "physxCollision:restOffset",
            tuning.rest_offset,
            Sdf.ValueTypeNames.Float,
        ),
    )

    for create_method_name, attr_name, value, value_type in rigid_attr_specs:
        create_schema_attr(
            rigid_api,
            prim,
            create_method_name,
            attr_name,
            value,
            value_type,
        )
    for (
        create_method_name,
        attr_name,
        value,
        value_type,
    ) in collision_attr_specs:
        create_schema_attr(
            collision_api,
            prim,
            create_method_name,
            attr_name,
            value,
            value_type,
        )


def add_physics_scene(
    stage: Any,
    tuning: PhysxSceneTuning | None,
) -> None:
    physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    if tuning is not None:
        apply_physx_scene_tuning(physics_scene.GetPrim(), tuning)


def usd_world_bounds(
    path: Path,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
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
    bowl_pos: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
    bowl_min_local, bowl_max_local = usd_world_bounds(BOWL_USD)
    container_min = tuple(
        bowl_min_local[index] + bowl_pos[index] for index in range(3)
    )
    container_max = tuple(
        bowl_max_local[index] + bowl_pos[index] for index in range(3)
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
    spawn_bottom_z = bowl_pos[2] + BEAN_PHYSICS["spawn_height"]
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
    materials: dict[str, Any],
    count: int,
    color: tuple[float, float, float],
    density: float,
    bowl_pos: tuple[float, float, float],
    tuning: BeanSimulationTuning | None,
) -> None:
    if count <= 0:
        return

    UsdGeom.Scope.Define(stage, "/World/Scene/CoffeeBeans")
    material_name = "CoffeeBean"
    materials[material_name] = create_preview_material(
        stage,
        f"/World/Looks/{material_name}",
        diffuse_color=color,
        metallic=0.0,
        roughness=0.8,
    )
    apply_physics_material(
        materials[material_name],
        friction=BEAN_PHYSICS["friction"],
        restitution=BEAN_PHYSICS["restitution"],
    )

    radius = BEAN_PHYSICS["radius"]
    half_height = BEAN_PHYSICS["half_height"]

    for index, position in enumerate(bean_spawn_positions(count, bowl_pos)):
        bean_prim_path = f"/World/Scene/CoffeeBeans/Bean_{index:04d}"
        bean = UsdGeom.Capsule.Define(stage, bean_prim_path)
        bean.CreateRadiusAttr(radius)
        bean.CreateHeightAttr(2.0 * half_height)
        bean.CreateAxisAttr("X")
        bean_prim = bean.GetPrim()

        yaw = random.uniform(0.0, 2.0 * math.pi)
        rot = (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))
        set_xform(bean_prim, position, rot, ASSET_SCALE)

        UsdPhysics.CollisionAPI.Apply(bean_prim)
        UsdPhysics.RigidBodyAPI.Apply(bean_prim)
        if tuning is not None:
            apply_bean_physx_tuning(bean_prim, tuning)
        mass_api = UsdPhysics.MassAPI.Apply(bean_prim)
        mass_api.CreateDensityAttr(density)
        UsdShade.MaterialBindingAPI.Apply(bean_prim).Bind(
            materials[material_name]
        )


def add_lights(stage: Any) -> None:
    dome = UsdLux.DomeLight.Define(stage, "/World/Light")
    dome.CreateIntensityAttr(5000.0)
    dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
    distant.CreateIntensityAttr(3000.0)
    distant.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    distant.CreateAngleAttr(0.5)


def add_camera(stage: Any) -> None:
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera_prim = camera.GetPrim()
    set_xform(
        camera_prim,
        pos=(0.0, 8.0, 8.0),
        rot=(0.9239, -0.3827, 0.0, 0.0),
        scale=(1.0, 1.0, 1.0),
    )
    camera.CreateFocalLengthAttr(18.0)


def table_placements(include_top_table: bool) -> list[AssetPlacement]:
    tables = list(TABLES)
    if include_top_table:
        tables.append(TOP_CENTER_TABLE)

    return [
        AssetPlacement(
            name=name,
            usd_path=TABLE_USD,
            prim_path=f"/World/Scene/{name}",
            pos=pos,
            rot=IDENTITY_ROT,
            scale=ASSET_SCALE,
        )
        for name, pos in tables
    ]


def head_placements(
    tables: Iterable[AssetPlacement],
) -> list[AssetPlacement]:
    text_table_positions = set(LETTER_TABLE_POS.values())

    def head_position(table: AssetPlacement) -> tuple[float, float, float]:
        if table.name == "Table_Bottom_Center":
            return (
                table.pos[0] - HEAD_Y_OFFSET,
                table.pos[1],
                table.pos[2] + TABLETOP_Z_OFFSET,
            )
        if "_Right_" in table.name:
            return (
                table.pos[0],
                table.pos[1] - HEAD_Y_OFFSET,
                table.pos[2] + TABLETOP_Z_OFFSET,
            )
        return (
            table.pos[0],
            table.pos[1] + HEAD_Y_OFFSET,
            table.pos[2] + TABLETOP_Z_OFFSET,
        )

    def head_rotation(table_name: str) -> tuple[float, float, float, float]:
        if table_name == "Table_Bottom_Center":
            return ROT_Z_90
        if "_Right_" in table_name:
            return ROT_Z_180
        return IDENTITY_ROT

    return [
        AssetPlacement(
            name=f"Head_{table.name}",
            usd_path=HEAD_USD,
            prim_path=f"/World/Scene/Head_{table.name}",
            pos=head_position(table),
            rot=head_rotation(table.name),
            scale=ASSET_SCALE,
            as_payload=True,
        )
        for table in tables
        if table.pos in text_table_positions
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
        pos=(0.0, 0.0, 0.0),
    )


def letter_placements() -> list[AssetPlacement]:
    placements = []
    for letter, table_pos in LETTER_TABLE_POS.items():
        tx, ty, tz = table_pos
        placements.append(
            AssetPlacement(
                name=f"Letter_{letter}",
                usd_path=ASSETS / f"{letter}_edit.usd",
                prim_path=f"/World/Scene/Letter_{letter}",
                pos=(tx, ty, tz + TABLETOP_Z_OFFSET + LETTER_HEIGHT_OFFSET),
                rot=LETTER_ROT,
                scale=ASSET_SCALE,
                material_name="Black",
            )
        )
    return placements


def cutlery_placements(
    randomize_color: bool,
    randomize_placement: bool,
) -> list[AssetPlacement]:
    placements = []
    tx, ty, tz = CUTLERY_TABLE_POS
    for item_name, config in CUTLERY.items():
        ox, oy, oz = config["offset"]
        pos = config["fixed_pos"]
        rot = config["fixed_rot"]
        if randomize_placement:
            random_offset = (
                random.uniform(-0.15, 0.15),
                random.uniform(-0.15, 0.15),
                random.uniform(0.0, 0.2),
            )
            pos = (
                tx + ox + random_offset[0],
                ty + oy + random_offset[1],
                tz + oz + random_offset[2],
            )
            rot = IDENTITY_ROT
        placements.append(
            AssetPlacement(
                name=f"Ikea_{item_name.capitalize()}",
                usd_path=config["usd_path"],
                prim_path=f"/World/Scene/Ikea_{item_name.capitalize()}",
                pos=pos,
                rot=rot,
                scale=ASSET_SCALE,
                material_name=(
                    random_cutlery_material_name(item_name)
                    if randomize_color
                    else None
                ),
            )
        )
    return placements


def robot_placement() -> AssetPlacement:
    return AssetPlacement(
        name="Robot",
        usd_path=ROBOT_USD,
        prim_path="/World/Robot",
        pos=(0.0, 4.5, 0.0),
    )


def compose_scene(
    output_path: Path,
    include_top_table: bool,
    with_robot: bool,
    save: bool,
    preview: bool,
    environment: str | None,
    randomize_cutlery_color: bool,
    randomize_cutlery_placement: bool,
    add_head: bool,
    bean_count: int,
    bean_color: tuple[float, float, float],
    bean_density: float,
    physx_scene_tuning: PhysxSceneTuning | None,
    bean_simulation_tuning: BeanSimulationTuning | None,
) -> Any:
    if include_top_table and with_robot:
        raise ValueError(
            "--include-top-table and --with-robot place assets in the same "
            "top-center space. Use one of them, or move one asset after "
            "opening the USD."
        )

    if save:
        output_path = output_path.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)

    tables = table_placements(include_top_table)
    heads = head_placements(tables) if add_head else []
    scene_placements = (
        tables
        + heads
        + letter_placements()
        + cutlery_placements(
            randomize_cutlery_color,
            randomize_cutlery_placement,
        )
    )
    bowl_pos = next(
        placement.pos
        for placement in scene_placements
        if placement.name == "Ikea_Bowl"
    )
    env_placement = environment_placement(environment)
    placements = [*scene_placements]
    if env_placement:
        placements.insert(0, env_placement)
    required_assets = [placement.usd_path for placement in placements]
    if with_robot:
        required_assets.append(ROBOT_USD)
    require_files(required_assets)

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
    add_physics_scene(stage, physx_scene_tuning)
    add_ground(stage, materials)
    add_lights(stage)
    add_camera(stage)

    for placement in placements:
        add_referenced_asset(stage, reference_base, placement, materials)

    add_coffee_beans(
        stage,
        materials,
        count=bean_count,
        color=bean_color,
        density=bean_density,
        bowl_pos=bowl_pos,
        tuning=bean_simulation_tuning,
    )

    if with_robot:
        add_referenced_asset(
            stage, reference_base, robot_placement(), materials
        )

    if save and preview:
        stage.GetRootLayer().Export(str(output_path))
        print(f"Wrote: {output_path}")
    elif save:
        stage.GetRootLayer().Save()
        print(f"Wrote: {output_path}")
    else:
        print("Scene composed in memory. Use --save to write a USD file.")

    table_count = len(table_placements(include_top_table))
    environment_path = env_placement.usd_path if env_placement else "none"
    print(f"Environment: {environment_path}")
    print(f"Tables: {table_count}")
    print(f"Heads: {len(heads)}")
    print("Letters: 9")
    print("Cutlery: 3")
    print(f"Coffee beans: {bean_count}")
    print(
        f"Physics tuning: {'script defaults' if physx_scene_tuning else 'none'}"
    )
    print(f"Robot reference: {'yes' if with_robot else 'no'}")
    return stage


def open_preview() -> None:
    print("Preview is open. Close the Isaac Sim window to exit.")
    while SIMULATION_APP and SIMULATION_APP.is_running():
        SIMULATION_APP.update()


def main() -> None:
    args = parse_args()
    initialize_usd_runtime(args.preview)
    compose_scene(
        output_path=args.output,
        include_top_table=args.include_top_table,
        with_robot=args.with_robot,
        save=args.save,
        preview=args.preview,
        environment=args.env,
        randomize_cutlery_color=args.randomize_cutlery_color,
        randomize_cutlery_placement=args.randomize_cutlery_placement,
        add_head=args.add_head,
        bean_count=args.bean_count,
        bean_color=tuple(args.bean_color),
        bean_density=args.bean_density,
        physx_scene_tuning=PhysxSceneTuning(),
        bean_simulation_tuning=BeanSimulationTuning(),
    )
    if args.preview:
        open_preview()


if __name__ == "__main__":
    main()
