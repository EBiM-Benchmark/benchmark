#!/usr/bin/env python3
"""Restructure arm joint prims in the FR3 Duo USD to match USD_7DOF_Arm_Articulation_Guide.

Changes made:
  1. Move each arm revolute joint from the flat /ai_cell/joints/ scope to sit
     under its body0 parent link (e.g. joint1 → /ai_cell/left_fr3v2_link0/joint1).
  2. Apply IsaacRobotAPI to /ai_cell so Isaac auto-populates IsaacLinkAPI /
     IsaacJointAPI on all discovered rigid bodies and joints.
  3. Apply PhysxArticulationAPI on the ArticulationRootAPI prim and set
     solver_position_iterations=64, solver_velocity_iterations=4.

Run inside the Isaac Sim container:

    docker exec isaac-sim bash -c "
      LD_LIBRARY_PATH=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311/bin \\
      /isaac-sim/python.sh /workspace/scripts/tooling/restructure_usd_joints.py \\
        /workspace/assets/digital_twin_fr3Duo.usd"

    # Dry-run (print intended changes without writing):
    /isaac-sim/python.sh restructure_usd_joints.py <usd> --dry-run

    # Write to a separate output file:
    /isaac-sim/python.sh restructure_usd_joints.py <usd> --out-usd-path <out>
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from pxr import PhysxSchema, Sdf, Usd, UsdPhysics

# Arm joints to relocate
ARM_JOINT_NAMES = [
    "left_fr3v2_joint1",
    "left_fr3v2_joint2",
    "left_fr3v2_joint3",
    "left_fr3v2_joint4",
    "left_fr3v2_joint5",
    "left_fr3v2_joint6",
    "left_fr3v2_joint7",
    "right_fr3v2_joint1",
    "right_fr3v2_joint2",
    "right_fr3v2_joint3",
    "right_fr3v2_joint4",
    "right_fr3v2_joint5",
    "right_fr3v2_joint6",
    "right_fr3v2_joint7",
]

OLD_JOINT_SCOPE = "/ai_cell/joints"

# Articulation root prim containing ArticulationRootAPI (used for solver iterations)
ARTICULATION_ROOT_PATH = "/ai_cell/root_joint"


def _iter_prim_specs(prim_spec: Sdf.PrimSpec):
    """Depth-first traversal of prim specs in a layer."""
    yield prim_spec
    for child in prim_spec.nameChildren:
        yield from _iter_prim_specs(child)


def _get_body0_from_spec(prim_spec: Sdf.PrimSpec) -> str | None:
    """Read physics:body0 target from an Sdf prim spec."""
    rel_spec = prim_spec.relationships.get("physics:body0")
    if rel_spec is None:
        return None
    items = rel_spec.targetPathList.explicitItems
    return str(items[0]) if items else None


def _find_joint_spec(layer: Sdf.Layer, joint_name: str) -> Sdf.PrimSpec | None:
    """Find a joint prim spec, checking OLD_JOINT_SCOPE first then full traversal."""
    candidate_path = Sdf.Path(f"{OLD_JOINT_SCOPE}/{joint_name}")
    spec = layer.GetPrimAtPath(candidate_path)
    if spec:
        return spec
    # Fallback: full layer traversal
    root = layer.GetPrimAtPath(Sdf.Path.absoluteRootPath)
    if root is None:
        return None
    for spec in _iter_prim_specs(root):
        if spec.name == joint_name:
            return spec
    return None


def _move_prim(layer: Sdf.Layer, old_path_str: str, new_path_str: str) -> bool:
    """Move a prim spec atomically using BatchNamespaceEdit, falling back to CopySpec."""
    old_sdf = Sdf.Path(old_path_str)
    new_sdf = Sdf.Path(new_path_str)

    if not layer.GetPrimAtPath(old_sdf):
        print(f"  WARNING: {old_path_str} not found in root layer — skipping")
        return False

    # Primary: BatchNamespaceEdit (atomic move)
    try:
        edit = Sdf.BatchNamespaceEdit()
        edit.Add(Sdf.NamespaceEdit(old_sdf, new_sdf))
        if layer.Apply(edit):
            return True
        print(f"  WARNING: BatchNamespaceEdit returned False for {old_path_str}")
    except Exception as exc:
        print(f"  WARNING: BatchNamespaceEdit raised {exc}, trying CopySpec fallback")

    # Fallback: CopySpec + erase
    try:
        with Sdf.ChangeBlock():
            Sdf.CopySpec(layer, old_sdf, layer, new_sdf)
            parent_spec = layer.GetPrimAtPath(old_sdf.GetParentPath())
            parent_spec.nameChildren.erase(old_sdf.name)
        return True
    except Exception as exc:
        print(f"  ERROR: CopySpec fallback also failed: {exc}")
        return False


def _apply_api_to_layer_prim(layer: Sdf.Layer, prim_path_str: str, api_name: str) -> bool:
    """Add an API schema token to a prim spec, preserving all existing apiSchemas."""
    prim_spec = layer.GetPrimAtPath(Sdf.Path(prim_path_str))
    if prim_spec is None:
        print(f"  WARNING: {prim_path_str} not found in layer")
        return False
    existing = prim_spec.GetInfo("apiSchemas")
    # Collect all existing tokens across every list-op category
    all_existing: list[str] = []
    if existing is not None:
        all_existing = (
            list(existing.prependedItems)
            + list(existing.appendedItems)
            + list(existing.explicitItems)
        )
    if api_name in all_existing:
        print(f"  {api_name} already present on {prim_path_str}")
        return True
    # Prepend new token while keeping all existing prepended tokens
    prepended = list(existing.prependedItems) if existing else []
    prepended.append(api_name)
    new_op = Sdf.TokenListOp()
    new_op.prependedItems = prepended
    prim_spec.SetInfo("apiSchemas", new_op)
    print(f"  Applied {api_name} to {prim_path_str}")
    return True


def _set_solver_iterations_on_layer(
    layer: Sdf.Layer,
    articulation_path_str: str,
    position_iters: int = 64,
    velocity_iters: int = 4,
) -> bool:
    """Write PhysxArticulationAPI solver iteration attributes directly to the layer."""
    prim_spec = layer.GetPrimAtPath(Sdf.Path(articulation_path_str))
    if prim_spec is None:
        print(f"  WARNING: Articulation prim {articulation_path_str} not found")
        return False

    # Ensure PhysxArticulationAPI is in apiSchemas
    _apply_api_to_layer_prim(layer, articulation_path_str, "PhysxArticulationAPI")

    # Write solver iteration attributes
    for attr_name, value in [
        ("physxArticulation:solverPositionIterationCount", position_iters),
        ("physxArticulation:solverVelocityIterationCount", velocity_iters),
    ]:
        attr_spec = prim_spec.attributes.get(attr_name)
        if attr_spec is None:
            attr_spec = Sdf.AttributeSpec(prim_spec, attr_name, Sdf.ValueTypeNames.Int)
        attr_spec.default = value
        print(f"  Set {attr_name} = {value} on {articulation_path_str}")
    return True


def restructure(usd_path: str, out_path: str | None = None, dry_run: bool = False) -> int:
    # Open the root layer directly — avoids schema registry errors from composed stage
    layer = Sdf.Layer.FindOrOpen(usd_path)
    if layer is None:
        print(f"ERROR: Failed to open USD layer: {usd_path}", file=sys.stderr)
        return 1

    moves: list[tuple[str, str]] = []
    skipped: list[str] = []

    # ── 1. Collect joint moves ────────────────────────────────────────────────
    print("\n=== Step 1: Arm joint relocation ===")
    for joint_name in ARM_JOINT_NAMES:
        spec = _find_joint_spec(layer, joint_name)
        if spec is None:
            print(f"  SKIP {joint_name}: not found in layer")
            skipped.append(joint_name)
            continue

        old_path = str(spec.path)
        body0_path = _get_body0_from_spec(spec)
        if not body0_path:
            print(f"  SKIP {joint_name}: no physics:body0 target")
            skipped.append(joint_name)
            continue

        new_path = f"{body0_path}/{joint_name}"
        if old_path == new_path:
            print(f"  OK   {joint_name}: already at correct path")
            continue

        print(f"  MOVE {old_path}")
        print(f"       -> {new_path}  (body0={body0_path})")
        moves.append((old_path, new_path))

    if dry_run:
        print(
            f"\nDry run: {len(moves)} move(s) planned, {len(skipped)} skipped. "
            "No files written."
        )
        return 0

    # ── 2. Apply moves ────────────────────────────────────────────────────────
    failed = 0
    for old_path, new_path in moves:
        if not _move_prim(layer, old_path, new_path):
            failed += 1

    # ── 3. IsaacRobotAPI ─────────────────────────────────────────────────────
    # NOTE: IsaacRobotAPI must be applied from inside Isaac Sim's full runtime
    # (where the schema plugin is loaded), NOT via bare pxr / Sdf.Layer.
    # Applying it as a raw string token here registers an unknown schema that
    # causes prim.HasAPI() to throw during stage traversal, silently breaking
    # articulation discovery. Skip it here — apply manually via the Stage panel
    # in Isaac Sim if needed.
    print("\n=== Step 2: IsaacRobotAPI (skipped — requires Isaac Sim runtime) ===")
    print("  Apply IsaacRobotAPI to /ai_cell manually via the Isaac Sim Stage panel.")

    # ── 4. Solver iterations ─────────────────────────────────────────────────
    print("\n=== Step 3: Solver iterations (pos=64, vel=4) ===")
    _set_solver_iterations_on_layer(layer, ARTICULATION_ROOT_PATH)

    # ── 5. Save ───────────────────────────────────────────────────────────────
    resolved_out = Path(out_path or usd_path).expanduser().resolve()
    resolved_in = Path(usd_path).expanduser().resolve()

    if resolved_out == resolved_in:
        layer.Save()
        print(f"\nSaved in-place: {resolved_out}")
    else:
        resolved_out.parent.mkdir(parents=True, exist_ok=True)
        layer.Export(str(resolved_out))
        print(f"\nExported to: {resolved_out}")

    print(
        f"\nDone: {len(moves) - failed} joint(s) moved, "
        f"{failed} failed, {len(skipped)} skipped."
    )
    return 0 if failed == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Restructure FR3 Duo USD joint hierarchy per USD_7DOF_Arm_Articulation_Guide."
    )
    parser.add_argument("usd_path", help="Path to the input USD file.")
    parser.add_argument(
        "--out-usd-path",
        default=None,
        help="Output path. Defaults to in-place update.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned changes without writing anything.",
    )
    args = parser.parse_args()
    return restructure(
        str(Path(args.usd_path).expanduser().resolve()),
        out_path=args.out_usd_path,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
