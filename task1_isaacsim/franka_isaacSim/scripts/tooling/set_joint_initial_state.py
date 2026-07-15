#!/usr/bin/env python3
"""Bake home-pose joint positions into a USD stage as initial physics state.

Sets ``physics:state:angular:physics:position`` (degrees) on every
RevoluteJoint prim whose name matches a joint listed in the home-pose YAML.
This eliminates the one-frame "snap-to-zero" transient that occurs when
``world.reset()`` loads the USD initial state before
``teleport_to_home_pose()`` can run.

Requires the ``pxr`` library, which is available inside the Isaac Sim
container.  The USD file is updated **in-place** unless ``--out-usd-path``
is provided.

Usage
-----
    docker exec isaac-sim python3 /workspace/scripts/tooling/set_joint_initial_state.py \\
        /workspace/assets/ai_cell_handBoxing2.usd \\
        --home-pose /workspace/assets/isaac_assets/config/data_collection_home_pose.yaml

    # Write to a separate file instead of updating in-place:
    docker exec isaac-sim python3 /workspace/scripts/tooling/set_joint_initial_state.py \\
        /workspace/assets/ai_cell_handBoxing2.usd \\
        --home-pose /workspace/assets/isaac_assets/config/data_collection_home_pose.yaml \\
        --out-usd-path /workspace/assets/ai_cell_handBoxing2_fixed.usd
"""

from __future__ import annotations

"""Bake home-pose joint positions into a USD stage as initial physics state.

Sets ``physics:state:angular:physics:position`` (degrees) on every
RevoluteJoint prim whose name matches a joint listed in the home-pose YAML.
This eliminates the one-frame "snap-to-zero" transient that occurs when
``world.reset()`` loads the USD initial state before the bridge can send
position commands, and prevents arms from collapsing when simulation stops.

Uses ``Sdf.Layer`` directly instead of ``Usd.Stage.Open()`` to avoid
TfErrorException from unregistered PhysX vehicle/particle schemas referenced
by environment assets in the same USD.

Usage
-----
    docker exec isaac-sim bash -c "
      LD_LIBRARY_PATH=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311/bin \\
      /isaac-sim/python.sh /workspace/scripts/tooling/set_joint_initial_state.py \\
        /workspace/assets/digital_twin_fr3Duo.usd \\
        --home-pose /workspace/assets/isaac_assets/config/data_collection_home_pose.yaml"
"""

import argparse
import math
import sys
from pathlib import Path

import yaml
from pxr import Sdf

# Canonical arm joint order (left then right) matching data_collection_home_pose.yaml
_DEFAULT_LEFT_JOINTS = [
    "left_fr3v2_joint1",
    "left_fr3v2_joint2",
    "left_fr3v2_joint3",
    "left_fr3v2_joint4",
    "left_fr3v2_joint5",
    "left_fr3v2_joint6",
    "left_fr3v2_joint7",
]
_DEFAULT_RIGHT_JOINTS = [
    "right_fr3v2_joint1",
    "right_fr3v2_joint2",
    "right_fr3v2_joint3",
    "right_fr3v2_joint4",
    "right_fr3v2_joint5",
    "right_fr3v2_joint6",
    "right_fr3v2_joint7",
]


def _load_home_pose(yaml_path: str, joint_names_override: list[str] | None = None) -> dict[str, float]:
    """Return ``{joint_name: position_rad}`` from the home-pose YAML.

    The YAML may either provide ``joint_names`` lists per arm or rely on the
    default canonical ordering (_DEFAULT_*_JOINTS).
    """
    with open(yaml_path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)

    result: dict[str, float] = {}

    if joint_names_override:
        # Flat list of names provided via CLI; match positions in left/right order
        all_positions = []
        for arm_key in ("left_arm", "right_arm"):
            arm = data.get(arm_key, {})
            all_positions.extend(arm.get("positions", []))
        for name, pos in zip(joint_names_override, all_positions):
            result[str(name)] = float(pos)
        return result

    for arm_key, default_names in (
        ("left_arm", _DEFAULT_LEFT_JOINTS),
        ("right_arm", _DEFAULT_RIGHT_JOINTS),
    ):
        arm = data.get(arm_key, {})
        names = arm.get("joint_names", default_names)
        positions = arm.get("positions", [])
        for name, pos in zip(names, positions):
            result[str(name)] = float(pos)

    return result


def _iter_prim_specs(prim_spec: Sdf.PrimSpec):
    """Depth-first traversal over all prim specs in a layer."""
    yield prim_spec
    for child in prim_spec.nameChildren:
        yield from _iter_prim_specs(child)


def _set_initial_positions_via_layer(
    layer: Sdf.Layer, joint_positions: dict[str, float]
) -> list[str]:
    """Write ``physics:state:angular:physics:position`` directly on Sdf prim specs.

    Works without a composed stage — avoids TfErrorException from unknown
    PhysX schemas in environment asset references.
    Returns the list of prim paths that were updated.
    """
    updated: list[str] = []
    root = layer.GetPrimAtPath(Sdf.Path.absoluteRootPath)
    if root is None:
        return updated

    for spec in _iter_prim_specs(root):
        if spec.typeName != "PhysicsRevoluteJoint":
            continue
        joint_name = spec.name
        if joint_name not in joint_positions:
            continue
        rad = joint_positions[joint_name]
        deg = math.degrees(rad)
        attr = spec.attributes.get("physics:state:angular:physics:position")
        if attr is None:
            attr = Sdf.AttributeSpec(
                spec,
                "physics:state:angular:physics:position",
                Sdf.ValueTypeNames.Float,
            )
        attr.default = float(deg)
        updated.append(str(spec.path))

    return updated


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Bake home-pose joint positions into a USD stage as initial physics state."
    )
    parser.add_argument("usd_path", help="USD file to update")
    parser.add_argument(
        "--home-pose",
        required=True,
        help="Path to data_collection_home_pose.yaml",
    )
    parser.add_argument(
        "--joint-names",
        default=None,
        help="Comma-separated joint names (overrides YAML joint_names / default order).",
    )
    parser.add_argument(
        "--out-usd-path",
        default=None,
        help="Output USD path. Defaults to in-place update.",
    )
    args = parser.parse_args()

    names_override = [n.strip() for n in args.joint_names.split(",")] if args.joint_names else None
    joint_positions = _load_home_pose(args.home_pose, names_override)

    if not joint_positions:
        print("ERROR: No joints loaded from home-pose YAML.", file=sys.stderr)
        return 1

    print(f"Loaded {len(joint_positions)} joints from {args.home_pose}:")
    for name, rad in sorted(joint_positions.items()):
        print(f"  {name}: {rad:+.4f} rad  ({math.degrees(rad):+.2f} deg)")

    usd_path = str(Path(args.usd_path).expanduser().resolve())
    layer = Sdf.Layer.FindOrOpen(usd_path)
    if layer is None:
        print(f"ERROR: Failed to open USD layer: {usd_path}", file=sys.stderr)
        return 1

    updated = _set_initial_positions_via_layer(layer, joint_positions)
    if not updated:
        print(
            "WARNING: No matching PhysicsRevoluteJoint prims found.\n"
            "Check that joint names match the USD prim names exactly.",
            file=sys.stderr,
        )
        return 0

    out_path = args.out_usd_path or usd_path
    resolved_out = Path(out_path).expanduser().resolve()
    if resolved_out == Path(usd_path).resolve():
        layer.Save()
    else:
        resolved_out.parent.mkdir(parents=True, exist_ok=True)
        layer.Export(str(resolved_out))

    print(f"\nBaked initial state on {len(updated)} joint(s):")
    for path in updated:
        print(f"  {path}")
    print(f"Saved to {resolved_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
