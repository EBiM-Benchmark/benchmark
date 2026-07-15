from __future__ import annotations

"""Bake world-space link transforms for ALL !resetXformStack! prims via graph FK.

Seeds known world transforms from:
  - /ai_cell/base          → world origin (inherits from /ai_cell which is at origin)
  - /ai_cell/mount_link    → world origin (mount_joint has zero offsets from base)
  - left_fr3v2_link0       → left_FR3 scope × link0 local
  - right_fr3v2_link0      → right_FR3 scope × link0 local

Then BFS-propagates through every joint found under /ai_cell (including arm
joints nested under their parent links after restructuring), computing body1
world transforms via the PhysX FK formula and writing them for all prims that
carry !resetXformStack!.

Joint angles: home pose for arm joints 1–7; 0 rad (open) for gripper joints.

Run inside the Isaac Sim container:

    docker exec isaac-sim bash -c "
      BASE=/isaac-sim/extscache
      USDLIBS=\\$BASE/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
      PHYSXSCHEMA=\\$BASE/omni.usd.schema.physx-107.3.26+107.3.3.lx64.r.cp311.u353
      PYTHONPATH=\\$USDLIBS:\\$PHYSXSCHEMA LD_LIBRARY_PATH=\\$USDLIBS/bin:\\$PHYSXSCHEMA/bin \\
      /isaac-sim/kit/python/bin/python3.11 \\
        /workspace/scripts/tooling/bake_all_link_transforms.py \\
        /workspace/assets/digital_twin_fr3Duo.usd"
"""

import math
import sys
from collections import deque
from pathlib import Path
from pxr import Sdf, Gf

# ── Home pose angles (rad) — unlisted revolute joints default to 0 ─────────────
HOME_ANGLES: dict[str, float] = {
    "left_fr3v2_joint1":  0.0,
    "left_fr3v2_joint2":  -math.pi / 4,
    "left_fr3v2_joint3":  0.0,
    "left_fr3v2_joint4":  -3 * math.pi / 4,
    "left_fr3v2_joint5":  0.0,
    "left_fr3v2_joint6":  math.pi / 2,
    "left_fr3v2_joint7":  math.pi / 4,
    "right_fr3v2_joint1": 0.0,
    "right_fr3v2_joint2": -math.pi / 4,
    "right_fr3v2_joint3": 0.0,
    "right_fr3v2_joint4": -3 * math.pi / 4,
    "right_fr3v2_joint5": 0.0,
    "right_fr3v2_joint6": math.pi / 2,
    "right_fr3v2_joint7": math.pi / 4,
}

# ── Arm base anchors: (FR3 scope path, link0 path) ───────────────────────────
ARM_ANCHORS = [
    ("/ai_cell/left_FR3",  "/ai_cell/left_FR3/left_fr3v2_link0"),
    ("/ai_cell/right_FR3", "/ai_cell/right_FR3/right_fr3v2_link0"),
]

ZERO_POS     = Gf.Vec3d(0, 0, 0)
IDENTITY_ROT = Gf.Quatd(1, Gf.Vec3d(0, 0, 0))
JOINT_TYPES  = {"PhysicsRevoluteJoint", "PhysicsFixedJoint", "PhysicsPrismaticJoint"}


# ── Quaternion helpers ─────────────────────────────────────────────────────────

def q_mul(a: Gf.Quatd, b: Gf.Quatd) -> Gf.Quatd:
    return a * b


def q_inv(q: Gf.Quatd) -> Gf.Quatd:
    return q.GetInverse()


def q_rotate(q: Gf.Quatd, v: Gf.Vec3d) -> Gf.Vec3d:
    return (q * Gf.Quatd(0, v) * q.GetInverse()).GetImaginary()


def q_from_axis_angle(axis: Gf.Vec3d, angle_rad: float) -> Gf.Quatd:
    half = angle_rad / 2.0
    return Gf.Quatd(math.cos(half), axis * math.sin(half))


def q_from_raw(t) -> Gf.Quatd:
    """Accept Gf.Quatd/Quatf/Quath or (w,x,y,z) tuple."""
    if isinstance(t, (Gf.Quatd, Gf.Quatf, Gf.Quath)):
        img = t.GetImaginary()
        return Gf.Quatd(float(t.GetReal()),
                        Gf.Vec3d(float(img[0]), float(img[1]), float(img[2])))
    return Gf.Quatd(float(t[0]), Gf.Vec3d(float(t[1]), float(t[2]), float(t[3])))


def q_to_quatf(q: Gf.Quatd) -> Gf.Quatf:
    return Gf.Quatf(float(q.GetReal()),
                    Gf.Vec3f(*[float(x) for x in q.GetImaginary()]))


# ── Sdf helpers ───────────────────────────────────────────────────────────────

def read_pos(spec: Sdf.PrimSpec, name: str) -> Gf.Vec3d:
    a = spec.attributes.get(name)
    v = a.default if a else None
    if v is None:
        return Gf.Vec3d(0, 0, 0)
    return Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))


def read_rot(spec: Sdf.PrimSpec, name: str) -> Gf.Quatd:
    a = spec.attributes.get(name)
    v = a.default if a else None
    if v is None:
        return Gf.Quatd(1, Gf.Vec3d(0, 0, 0))
    return q_from_raw(v)


def uses_reset(spec: Sdf.PrimSpec) -> bool:
    order = spec.attributes.get("xformOpOrder")
    return bool(order and order.default and "!resetXformStack!" in list(order.default))


def write_xform(spec: Sdf.PrimSpec, pos: Gf.Vec3d, rot: Gf.Quatd) -> None:
    """Write translate + orient, matching existing attribute type (Quatd or Quatf)."""
    t_attr = spec.attributes.get("xformOp:translate")
    if t_attr is None:
        t_attr = Sdf.AttributeSpec(spec, "xformOp:translate", Sdf.ValueTypeNames.Double3)
    t_attr.default = Gf.Vec3d(pos[0], pos[1], pos[2])

    o_attr = spec.attributes.get("xformOp:orient")
    if o_attr is None:
        o_attr = Sdf.AttributeSpec(spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd)
    if "quatf" in str(o_attr.typeName).lower():
        o_attr.default = q_to_quatf(rot)
    else:
        o_attr.default = Gf.Quatd(float(rot.GetReal()),
                                   Gf.Vec3d(*[float(x) for x in rot.GetImaginary()]))

    order_attr = spec.attributes.get("xformOpOrder")
    if order_attr is None:
        order_attr = Sdf.AttributeSpec(spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray)
    needed = ["!resetXformStack!", "xformOp:translate", "xformOp:orient", "xformOp:scale"]
    if list(order_attr.default or []) != needed:
        order_attr.default = needed


# ── FK step (PhysX revolute/fixed joint convention) ───────────────────────────

def fk_step(
    body0_pos: Gf.Vec3d,
    body0_rot: Gf.Quatd,
    lp0: Gf.Vec3d,
    lr0: Gf.Quatd,
    angle_rad: float,
    lp1: Gf.Vec3d,
    lr1: Gf.Quatd,
) -> tuple[Gf.Vec3d, Gf.Quatd]:
    joint_pos = body0_pos + q_rotate(body0_rot, lp0)
    joint_rot = q_mul(q_mul(body0_rot, lr0),
                      q_from_axis_angle(Gf.Vec3d(0, 0, 1), angle_rad))
    body1_rot = q_mul(joint_rot, q_inv(lr1))
    body1_pos = joint_pos - q_rotate(body1_rot, lp1)
    return body1_pos, body1_rot


# ── Build full joint adjacency from the prim tree ─────────────────────────────

def iter_specs(spec: Sdf.PrimSpec):
    yield spec
    for child in spec.nameChildren:
        yield from iter_specs(child)


def build_adjacency(layer: Sdf.Layer) -> dict[str, list[tuple[Sdf.PrimSpec, str]]]:
    adjacency: dict[str, list] = {}
    root = layer.GetPrimAtPath(Sdf.Path("/ai_cell"))
    if root is None:
        return adjacency
    for spec in iter_specs(root):
        if spec.typeName not in JOINT_TYPES:
            continue
        b0_rel = spec.relationships.get("physics:body0")
        b1_rel = spec.relationships.get("physics:body1")
        if b0_rel is None or b1_rel is None:
            continue
        b0_items = b0_rel.targetPathList.explicitItems
        b1_items = b1_rel.targetPathList.explicitItems
        if not b0_items or not b1_items:
            continue
        b0, b1 = str(b0_items[0]), str(b1_items[0])
        adjacency.setdefault(b0, []).append((spec, b1))
    return adjacency


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    usd_path = sys.argv[1] if len(sys.argv) > 1 else "/workspace/assets/digital_twin_fr3Duo.usd"
    usd_path = str(Path(usd_path).expanduser().resolve())

    layer = Sdf.Layer.FindOrOpen(usd_path)
    if layer is None:
        print(f"ERROR: Could not open {usd_path}", file=sys.stderr)
        return 1

    # ── Seed world transforms ──────────────────────────────────────────────────
    # base and mount_link both at world origin (/ai_cell is at origin, both
    # inherit with zero local transforms; mount_joint has zero offsets)
    world: dict[str, tuple[Gf.Vec3d, Gf.Quatd]] = {
        "/ai_cell/base":       (ZERO_POS, IDENTITY_ROT),
        "/ai_cell/mount_link": (ZERO_POS, IDENTITY_ROT),
    }

    # link0 world = FR3_scope_local × link0_local (FR3 scope doesn't use
    # !resetXformStack!, so its authored xformOp IS local relative to /ai_cell;
    # since /ai_cell is at world origin, FR3 local == FR3 world)
    for fr3_path, link0_path in ARM_ANCHORS:
        fr3  = layer.GetPrimAtPath(Sdf.Path(fr3_path))
        lk0  = layer.GetPrimAtPath(Sdf.Path(link0_path))
        if fr3 is None or lk0 is None:
            print(f"WARNING: anchor not found: {fr3_path}")
            continue
        fr3_pos = read_pos(fr3, "xformOp:translate")
        fr3_rot = read_rot(fr3, "xformOp:orient")
        lk0_pos = read_pos(lk0, "xformOp:translate")
        lk0_rot = read_rot(lk0, "xformOp:orient")
        w_pos = fr3_pos + q_rotate(fr3_rot, lk0_pos)
        w_rot = q_mul(fr3_rot, lk0_rot)
        world[link0_path] = (w_pos, w_rot)
        name = link0_path.split("/")[-1]
        print(f"Seed {name}: ({w_pos[0]:.4f}, {w_pos[1]:.4f}, {w_pos[2]:.4f})")

    # ── Build joint graph ──────────────────────────────────────────────────────
    adjacency = build_adjacency(layer)
    n_joints = sum(len(v) for v in adjacency.values())
    print(f"\nJoint graph: {n_joints} connections from {len(adjacency)} parent bodies")

    # ── BFS ───────────────────────────────────────────────────────────────────
    queue: deque[str] = deque(world)
    visited_joints: set[str] = set()
    written = 0

    while queue:
        b0_path = queue.popleft()
        if b0_path not in world:
            continue
        b0_pos, b0_rot = world[b0_path]

        for j_spec, b1_path in adjacency.get(b0_path, []):
            j_key = str(j_spec.path)
            if j_key in visited_joints:
                continue
            visited_joints.add(j_key)

            # Seeds take priority; don't overwrite link0 computed above
            if b1_path in world:
                continue

            lp0 = read_pos(j_spec, "physics:localPos0")
            lr0 = read_rot(j_spec, "physics:localRot0")
            lp1 = read_pos(j_spec, "physics:localPos1")
            lr1 = read_rot(j_spec, "physics:localRot1")
            angle = (HOME_ANGLES.get(j_spec.name, 0.0)
                     if j_spec.typeName == "PhysicsRevoluteJoint" else 0.0)

            b1_pos, b1_rot = fk_step(b0_pos, b0_rot, lp0, lr0, angle, lp1, lr1)
            world[b1_path] = (b1_pos, b1_rot)

            b1_spec = layer.GetPrimAtPath(Sdf.Path(b1_path))
            if b1_spec and uses_reset(b1_spec):
                write_xform(b1_spec, b1_pos, b1_rot)
                written += 1
                name = b1_path.split("/")[-1]
                print(f"  {name:50s}  ({b1_pos[0]:+.4f}, {b1_pos[1]:+.4f}, {b1_pos[2]:+.4f})")

            queue.append(b1_path)

    layer.Save()
    print(f"\nWrote {written} link transforms. Saved in-place.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
