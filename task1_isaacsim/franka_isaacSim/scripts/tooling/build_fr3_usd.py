#!/usr/bin/env python3
"""Build digital_twin_fr3.usd — single FR3v2 arm + Robotiq 2F-85 gripper at world origin.

Extracts the left arm from digital_twin_fr3Duo.usd, strips the environment and right
arm, renames every prim (removes the 'left_' prefix), repositions fr3v2_link0 at
(0, 0, 0), and writes a clean articulation with fresh physics scaffolding.

Run inside the Isaac Sim container:

    docker exec isaac-sim bash -c "
      LD_LIBRARY_PATH=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311/bin \\
      /isaac-sim/python.sh /workspace/scripts/tooling/build_fr3_usd.py"

    # Custom paths:
    /isaac-sim/python.sh build_fr3_usd.py \\
        --src /workspace/assets/digital_twin_fr3Duo.usd \\
        --dst /workspace/assets/digital_twin_fr3.usd
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

DEFAULT_SRC = "/workspace/assets/digital_twin_fr3Duo.usd"
DEFAULT_DST = "/workspace/assets/digital_twin_fr3.usd"

# ── Rename helpers ────────────────────────────────────────────────────────────

def _rename_str(s: str) -> str:
    """Strip left-arm prefix tokens from a string (path segment or attribute value)."""
    return s.replace("left_fr3v2_", "fr3v2_").replace("left_robotiq_", "robotiq_")


def _rename_sdf(path: Sdf.Path) -> Sdf.Path:
    return Sdf.Path(_rename_str(str(path)))


# ── Source prim selection ─────────────────────────────────────────────────────

def _is_left_arm_prim(path: str) -> bool:
    """True for /ai_cell/<left_*> and /ai_cell/joints/<left_*> top-level prims."""
    parts = path.split("/")
    # Must be under /ai_cell
    if len(parts) < 3 or parts[1] != "ai_cell":
        return False
    segment = parts[2]
    # Top-level left_* link or joint prims directly under /ai_cell
    if segment.startswith("left_"):
        return True
    # Joints scope
    if segment == "joints" and len(parts) == 4 and parts[3].startswith("left_"):
        return True
    return False


# ── Reference rewriting ───────────────────────────────────────────────────────

def _rewrite_relationships(stage: Usd.Stage) -> None:
    """Update all physics:body0 / physics:body1 targets that still carry left_ names."""
    for prim in stage.Traverse():
        for rel_name in ("physics:body0", "physics:body1"):
            rel = prim.GetRelationship(rel_name)
            if not rel or not rel.HasAuthoredTargets():
                continue
            targets = rel.GetTargets()
            new_targets = [_rename_sdf(t) for t in targets]
            if new_targets != targets:
                rel.SetTargets(new_targets)


# ── Joint drive application ───────────────────────────────────────────────────

def _apply_drive(prim: Usd.Prim, stiffness: float, damping: float,
                 max_force: float) -> None:
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("acceleration")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(max_force)


def _apply_joint_drives(stage: Usd.Stage) -> None:
    """Apply joint drives per fr3/isaac_joint_drives.yaml values."""
    heavy = {f"/ai_cell/joints/fr3v2_joint{i}" for i in range(1, 5)}
    light = {f"/ai_cell/joints/fr3v2_joint{i}" for i in range(5, 8)}
    gripper = {
        f"/ai_cell/joints/fr3v2_robotiq_85_{s}"
        for s in (
            "left_knuckle_joint", "right_knuckle_joint",
            "left_inner_knuckle_joint", "right_inner_knuckle_joint",
            "left_finger_tip_joint", "right_finger_tip_joint",
        )
    }

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path in heavy:
            _apply_drive(prim, stiffness=625.0, damping=60.0, max_force=87.0)
        elif path in light:
            _apply_drive(prim, stiffness=625.0, damping=40.0, max_force=12.0)
        elif path in gripper:
            _apply_drive(prim, stiffness=1000.0, damping=100.0, max_force=200.0)


# ── Main build ────────────────────────────────────────────────────────────────

def build(src_path: str, dst_path: str) -> int:
    print(f"Source: {src_path}")
    print(f"Dest:   {dst_path}")

    src_stage = Usd.Stage.Open(src_path)
    if src_stage is None:
        print(f"ERROR: cannot open source USD: {src_path}", file=sys.stderr)
        return 1

    src_layer = src_stage.GetRootLayer()

    # ── New stage ─────────────────────────────────────────────────────────────
    Path(dst_path).parent.mkdir(parents=True, exist_ok=True)
    dst_stage = Usd.Stage.CreateNew(dst_path)
    dst_layer = dst_stage.GetRootLayer()

    UsdGeom.SetStageUpAxis(dst_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(dst_stage, 1.0)
    dst_stage.SetStartTimeCode(0)
    dst_stage.SetEndTimeCode(100)

    # ── /ai_cell root ─────────────────────────────────────────────────────────
    ai_cell = UsdGeom.Xform.Define(dst_stage, "/ai_cell")
    dst_stage.SetDefaultPrim(ai_cell.GetPrim())
    print("  Defined /ai_cell (defaultPrim)")

    # ── Physics scene ─────────────────────────────────────────────────────────
    physics_scene = UsdPhysics.Scene.Define(dst_stage, "/ai_cell/PhysicsScene")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    print("  Defined /ai_cell/PhysicsScene")

    # ── Minimal base rigid body (world anchor) ────────────────────────────────
    base_prim = UsdGeom.Xform.Define(dst_stage, "/ai_cell/base").GetPrim()
    rb = UsdPhysics.RigidBodyAPI.Apply(base_prim)
    rb.CreateKinematicEnabledAttr(True)
    print("  Defined /ai_cell/base (kinematic)")

    # ── root_joint: FixedJoint body0=world → body1=base + ArticulationRootAPI ─
    root_joint = UsdPhysics.FixedJoint.Define(dst_stage, "/ai_cell/root_joint")
    root_joint.GetPrim().GetRelationship("physics:body1").SetTargets(
        [Sdf.Path("/ai_cell/base")]
    )
    UsdPhysics.ArticulationRootAPI.Apply(root_joint.GetPrim())

    # Disable self-collision and set solver iterations
    physx_art = PhysxSchema.PhysxArticulationAPI.Apply(root_joint.GetPrim())
    physx_art.CreateEnabledSelfCollisionsAttr(False)
    physx_art.CreateSolverPositionIterationCountAttr(64)
    physx_art.CreateSolverVelocityIterationCountAttr(4)
    print("  Defined /ai_cell/root_joint (ArticulationRootAPI, 64/4 solver iters)")

    # ── Joints scope ──────────────────────────────────────────────────────────
    dst_stage.DefinePrim("/ai_cell/joints", "Scope")
    print("  Defined /ai_cell/joints scope")

    # ── Copy left arm prims from source ───────────────────────────────────────
    print("\n  Copying left arm prims from source:")
    copied = 0
    skipped = []

    for prim in src_stage.Traverse():
        src_path_str = str(prim.GetPath())
        if not _is_left_arm_prim(src_path_str):
            continue

        # Skip prims that are children of another left_ prim we already copied
        # (they arrive as part of their parent's CopySpec)
        parent_str = str(Sdf.Path(src_path_str).GetParentPath())
        if _is_left_arm_prim(parent_str):
            continue

        dst_path_str = _rename_str(src_path_str)
        src_sdf = Sdf.Path(src_path_str)
        dst_sdf = Sdf.Path(dst_path_str)

        # Ensure the parent scope exists in the destination layer
        parent_dst = dst_layer.GetPrimAtPath(dst_sdf.GetParentPath())
        if not parent_dst:
            skipped.append(src_path_str)
            print(f"    SKIP (no parent): {src_path_str}")
            continue

        Sdf.CopySpec(src_layer, src_sdf, dst_layer, dst_sdf)
        print(f"    {src_path_str}  →  {dst_path_str}")
        copied += 1

    print(f"\n  Copied {copied} top-level prims ({len(skipped)} skipped)")

    # ── Rewrite all left_ relationship targets ────────────────────────────────
    print("\n  Rewriting relationship targets...")
    _rewrite_relationships(dst_stage)

    # ── Reposition fr3v2_link0 at world origin ────────────────────────────────
    link0 = dst_stage.GetPrimAtPath("/ai_cell/fr3v2_link0")
    if link0.IsValid():
        xformable = UsdGeom.Xformable(link0)
        # Clear all xform ops and set a zero translate (arm upright at origin)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        print("  Repositioned fr3v2_link0 → (0, 0, 0)")
    else:
        print("  WARNING: /ai_cell/fr3v2_link0 not found — check source prim names")

    # ── joint0_fixed: base → fr3v2_link0 ─────────────────────────────────────
    # Look for an existing fixed joint that was copied and renamed
    j0_path = "/ai_cell/fr3v2_joint0_fixed"
    j0_prim = dst_stage.GetPrimAtPath(j0_path)
    if j0_prim.IsValid():
        # Update body refs to the new names (rewrite already handled this, but be explicit)
        j0_prim.GetRelationship("physics:body0").SetTargets([Sdf.Path("/ai_cell/base")])
        j0_prim.GetRelationship("physics:body1").SetTargets([Sdf.Path("/ai_cell/fr3v2_link0")])
        print(f"  Updated {j0_path} body refs")
    else:
        # Create it from scratch (source may not have exposed the fixed joint separately)
        j0 = UsdPhysics.FixedJoint.Define(dst_stage, j0_path)
        j0.GetPrim().GetRelationship("physics:body0").SetTargets([Sdf.Path("/ai_cell/base")])
        j0.GetPrim().GetRelationship("physics:body1").SetTargets([Sdf.Path("/ai_cell/fr3v2_link0")])
        print(f"  Created {j0_path} (base → fr3v2_link0)")

    # ── Apply joint drives ────────────────────────────────────────────────────
    print("\n  Applying joint drives...")
    _apply_joint_drives(dst_stage)

    # ── IsaacRobotAPI (auto-populates IsaacLinkAPI / IsaacJointAPI) ───────────
    try:
        ai_cell.GetPrim().ApplyAPI("IsaacRobotAPI")
        print("  Applied IsaacRobotAPI to /ai_cell")
    except Exception as exc:
        print(f"  WARNING: IsaacRobotAPI not available outside Isaac Sim: {exc}")
        print("    → Re-apply manually via the Stage panel after opening in Isaac Sim")

    # ── Save ──────────────────────────────────────────────────────────────────
    dst_layer.Save()
    print(f"\nSaved: {dst_path}")

    # ── Quick sanity report ───────────────────────────────────────────────────
    print("\n--- Sanity check ---")
    joint_count = 0
    link_count = 0
    for prim in dst_stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joint_count += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            link_count += 1
    print(f"  Revolute joints : {joint_count}  (expected 13: 7 arm + 6 gripper)")
    print(f"  Rigid bodies    : {link_count}   (expected 10: base + 9 arm links)")
    art_count = sum(
        1 for p in dst_stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)
    )
    print(f"  Articulation roots: {art_count}  (expected 1)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build digital_twin_fr3.usd from digital_twin_fr3Duo.usd."
    )
    parser.add_argument(
        "--src", default=DEFAULT_SRC,
        help=f"Source USD path (default: {DEFAULT_SRC})"
    )
    parser.add_argument(
        "--dst", default=DEFAULT_DST,
        help=f"Destination USD path (default: {DEFAULT_DST})"
    )
    args = parser.parse_args()
    return build(
        str(Path(args.src).expanduser().resolve()),
        str(Path(args.dst).expanduser().resolve()),
    )


if __name__ == "__main__":
    raise SystemExit(main())
