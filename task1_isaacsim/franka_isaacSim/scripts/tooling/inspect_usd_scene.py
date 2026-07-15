#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any

SIMULATION_APP = None
Gf = None
PhysxSchema = None
Sdf = None
Usd = None
UsdGeom = None
UsdPhysics = None
UsdRender = None


def _ensure_pxr_loaded() -> None:
    global Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdRender
    if Usd is not None:
        return
    from pxr import Gf as _Gf
    from pxr import PhysxSchema as _PhysxSchema
    from pxr import Sdf as _Sdf
    from pxr import Usd as _Usd
    from pxr import UsdGeom as _UsdGeom
    from pxr import UsdPhysics as _UsdPhysics
    from pxr import UsdRender as _UsdRender

    Gf = _Gf
    PhysxSchema = _PhysxSchema
    Sdf = _Sdf
    Usd = _Usd
    UsdGeom = _UsdGeom
    UsdPhysics = _UsdPhysics
    UsdRender = _UsdRender


def _safe_attr_value(attribute: Any) -> Any:
    value = attribute
    if hasattr(attribute, "Get"):
        try:
            value = attribute.Get()
        except Exception:
            return None
    if value is None:
        return None
    if Sdf is not None and isinstance(value, Sdf.Path):
        return str(value)
    if Gf is not None and isinstance(value, (Gf.Quatd, Gf.Quatf, Gf.Quath)):
        imaginary = value.GetImaginary()
        return [
            float(value.GetReal()),
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
        ]
    if isinstance(value, dict):
        return {str(key): _safe_attr_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_safe_attr_value(item) for item in value]
    if isinstance(value, (str, int, float, bool)):
        return value
    try:
        return [_safe_attr_value(item) for item in value]
    except TypeError:
        pass
    return str(value)


def _schema_names(prim) -> list[str]:
    names: list[str] = []
    try:
        for schema in prim.GetAppliedSchemas():
            names.append(str(schema))
    except Exception:
        pass
    return sorted(set(names))


def _inspect_physics_scene(prim) -> dict[str, Any]:
    scene = UsdPhysics.Scene(prim)
    physx_scene = PhysxSchema.PhysxSceneAPI(prim)
    return {
        "path": str(prim.GetPath()),
        "gravity_direction": _safe_attr_value(scene.GetGravityDirectionAttr()),
        "gravity_magnitude": _safe_attr_value(scene.GetGravityMagnitudeAttr()),
        "enable_ccd": _safe_attr_value(physx_scene.GetEnableCCDAttr()),
        "enable_stabilization": _safe_attr_value(
            physx_scene.GetEnableStabilizationAttr()
        ),
        "enable_gpu_dynamics": _safe_attr_value(
            physx_scene.GetEnableGPUDynamicsAttr()
        ),
        "broadphase_type": _safe_attr_value(physx_scene.GetBroadphaseTypeAttr()),
        "solver_type": _safe_attr_value(physx_scene.GetSolverTypeAttr()),
    }


def _inspect_camera(prim) -> dict[str, Any]:
    camera = UsdGeom.Camera(prim)
    return {
        "path": str(prim.GetPath()),
        "parent": str(prim.GetParent().GetPath()),
        "focal_length": _safe_attr_value(camera.GetFocalLengthAttr()),
        "horizontal_aperture": _safe_attr_value(camera.GetHorizontalApertureAttr()),
        "vertical_aperture": _safe_attr_value(camera.GetVerticalApertureAttr()),
        "clipping_range": _safe_attr_value(camera.GetClippingRangeAttr()),
    }


def _inspect_joint(prim) -> dict[str, Any]:
    joint = UsdPhysics.Joint(prim)
    relationships: dict[str, list[str]] = {}
    for rel_name in ("physics:body0", "physics:body1"):
        relationship = prim.GetRelationship(rel_name)
        if relationship:
            relationships[rel_name] = [
                str(target) for target in relationship.GetTargets()
            ]
    drives: dict[str, dict[str, Any]] = {}
    for drive_axis in ("angular", "linear"):
        drive_api = UsdPhysics.DriveAPI.Get(prim, drive_axis)
        if not drive_api:
            continue
        drives[drive_axis] = {
            "type": _safe_attr_value(drive_api.GetTypeAttr()),
            "stiffness": _safe_attr_value(drive_api.GetStiffnessAttr()),
            "damping": _safe_attr_value(drive_api.GetDampingAttr()),
            "max_force": _safe_attr_value(drive_api.GetMaxForceAttr()),
        }
    return {
        "path": str(prim.GetPath()),
        "type": prim.GetTypeName(),
        "parent": str(prim.GetParent().GetPath()),
        "enabled": _safe_attr_value(joint.GetJointEnabledAttr()),
        "lower_limit": _safe_attr_value(prim.GetAttribute("physics:lowerLimit")),
        "upper_limit": _safe_attr_value(prim.GetAttribute("physics:upperLimit")),
        "body_targets": relationships,
        "drives": drives,
    }


def _inspect_articulation(prim) -> dict[str, Any]:
    return {
        "path": str(prim.GetPath()),
        "type": prim.GetTypeName(),
        "parent": str(prim.GetParent().GetPath()),
        "schemas": _schema_names(prim),
    }


def _inspect_rigidbody(prim) -> dict[str, Any]:
    body = UsdPhysics.RigidBodyAPI(prim)
    mass_api = UsdPhysics.MassAPI(prim)
    return {
        "path": str(prim.GetPath()),
        "type": prim.GetTypeName(),
        "kinematic_enabled": _safe_attr_value(body.GetKinematicEnabledAttr()),
        "rigid_body_enabled": _safe_attr_value(body.GetRigidBodyEnabledAttr()),
        "mass": _safe_attr_value(mass_api.GetMassAttr()),
        "center_of_mass": _safe_attr_value(mass_api.GetCenterOfMassAttr()),
        "diagonal_inertia": _safe_attr_value(mass_api.GetDiagonalInertiaAttr()),
        "principal_axes": _safe_attr_value(mass_api.GetPrincipalAxesAttr()),
    }


def _inspect_collider(prim) -> dict[str, Any]:
    mesh_collision = UsdPhysics.MeshCollisionAPI(prim)
    collision = UsdPhysics.CollisionAPI(prim)
    return {
        "path": str(prim.GetPath()),
        "type": prim.GetTypeName(),
        "collision_enabled": _safe_attr_value(collision.GetCollisionEnabledAttr()),
        "approximation": _safe_attr_value(mesh_collision.GetApproximationAttr()),
        "parent": str(prim.GetParent().GetPath()),
    }


def _inspect_render_product(prim) -> dict[str, Any]:
    render_product = UsdRender.Product(prim)
    return {
        "path": str(prim.GetPath()),
        "camera_prim": _safe_attr_value(render_product.GetCameraRel().GetTargets()[0])
        if render_product.GetCameraRel().GetTargets()
        else None,
        "resolution": _safe_attr_value(render_product.GetResolutionAttr()),
    }


def inspect_stage(usd_path: Path) -> dict[str, Any]:
    _ensure_pxr_loaded()
    if SIMULATION_APP is not None:
        import omni.usd

        context = omni.usd.get_context()
        if not context.open_stage(str(usd_path)):
            raise RuntimeError(f"Failed to open USD stage in Isaac: {usd_path}")
        for _ in range(3):
            SIMULATION_APP.update()
        stage = context.get_stage()
    else:
        stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {usd_path}")

    prim_count = 0
    type_counter: Counter[str] = Counter()
    schema_counter: Counter[str] = Counter()
    articulation_roots: list[dict[str, Any]] = []
    joints: list[dict[str, Any]] = []
    fixed_joints: list[dict[str, Any]] = []
    rigid_bodies: list[dict[str, Any]] = []
    colliders: list[dict[str, Any]] = []
    collision_schema_prims: list[dict[str, Any]] = []
    cameras: list[dict[str, Any]] = []
    render_products: list[dict[str, Any]] = []
    physics_scenes: list[dict[str, Any]] = []
    xforms_with_camera_name: list[str] = []
    roots_with_references: list[dict[str, Any]] = []

    for prim in stage.Traverse():
        if not prim.IsActive():
            continue
        prim_count += 1
        type_name = prim.GetTypeName() or "untyped"
        type_counter[type_name] += 1
        schemas = _schema_names(prim)
        for schema_name in schemas:
            schema_counter[schema_name] += 1
        collision_schemas = [
            schema_name
            for schema_name in schemas
            if any(
                token in schema_name
                for token in (
                    "Collision",
                    "Contact",
                    "FilteredPairs",
                    "PhysxVehicle",
                )
            )
        ]
        if collision_schemas:
            collision_schema_prims.append(
                {
                    "path": str(prim.GetPath()),
                    "type": type_name,
                    "schemas": collision_schemas,
                }
            )

        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulation_roots.append(_inspect_articulation(prim))

        if prim.IsA(UsdPhysics.Joint):
            joint_info = _inspect_joint(prim)
            joints.append(joint_info)
            if prim.GetTypeName() == "FixedJoint":
                fixed_joints.append(joint_info)

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(_inspect_rigidbody(prim))

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append(_inspect_collider(prim))

        if prim.IsA(UsdGeom.Camera):
            cameras.append(_inspect_camera(prim))
        elif "camera" in prim.GetName().lower():
            xforms_with_camera_name.append(str(prim.GetPath()))

        if prim.IsA(UsdRender.Product):
            render_products.append(_inspect_render_product(prim))

        if prim.IsA(UsdPhysics.Scene):
            physics_scenes.append(_inspect_physics_scene(prim))

        references = prim.GetMetadata("references")
        if prim.GetParent().GetPath() == Sdf.Path.absoluteRootPath and references:
            assets = []
            for item in references.GetAddedOrExplicitItems():
                assets.append(
                    {
                        "asset_path": str(item.assetPath),
                        "prim_path": str(item.primPath) if item.primPath else None,
                    }
                )
            roots_with_references.append(
                {
                    "path": str(prim.GetPath()),
                    "assets": assets,
                }
            )

    collider_parent_counter = Counter(collider["parent"] for collider in colliders)
    joint_type_counter = Counter(joint["type"] for joint in joints)

    return {
        "usd_path": str(usd_path),
        "default_prim": str(stage.GetDefaultPrim().GetPath())
        if stage.GetDefaultPrim()
        else None,
        "start_time_code": stage.GetStartTimeCode(),
        "end_time_code": stage.GetEndTimeCode(),
        "up_axis": UsdGeom.GetStageUpAxis(stage),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "prim_count": prim_count,
        "type_counts": dict(type_counter.most_common()),
        "schema_counts": dict(schema_counter.most_common()),
        "articulation_roots": articulation_roots,
        "joint_counts": dict(joint_type_counter.most_common()),
        "joints": joints,
        "fixed_joints": fixed_joints,
        "rigid_body_count": len(rigid_bodies),
        "rigid_bodies": rigid_bodies,
        "collider_count": len(colliders),
        "colliders": colliders,
        "collider_parent_counts": dict(collider_parent_counter.most_common()),
        "collision_schema_prim_count": len(collision_schema_prims),
        "collision_schema_prims": collision_schema_prims,
        "physics_scenes": physics_scenes,
        "camera_count": len(cameras),
        "cameras": cameras,
        "camera_named_xforms": sorted(xforms_with_camera_name),
        "render_product_count": len(render_products),
        "render_products": render_products,
        "root_references": roots_with_references,
        "root_layer": stage.GetRootLayer().identifier,
        "sub_layers": list(stage.GetRootLayer().subLayerPaths),
    }


def _bootstrap_simulation_app(headless: bool = True) -> None:
    global SIMULATION_APP
    if SIMULATION_APP is not None:
        return
    from omni.isaac.kit import SimulationApp

    SIMULATION_APP = SimulationApp({"headless": headless})


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspect an Isaac USD stage for physics and camera structure."
    )
    parser.add_argument("usd_path", help="Path to the USD file to inspect.")
    parser.add_argument(
        "--json-out",
        help="Optional path to write the inspection JSON. Defaults to stdout.",
    )
    parser.add_argument(
        "--bootstrap-isaac",
        action="store_true",
        help="Start a headless Isaac SimulationApp first so schema plugins are loaded.",
    )
    args = parser.parse_args()

    if args.bootstrap_isaac:
        _bootstrap_simulation_app()
    _ensure_pxr_loaded()

    report = inspect_stage(Path(args.usd_path).expanduser().resolve())
    payload = json.dumps(report, indent=2, sort_keys=True)
    if args.json_out:
        Path(args.json_out).expanduser().resolve().write_text(payload + "\n")
    else:
        print(payload)
    if SIMULATION_APP is not None:
        SIMULATION_APP.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
