from __future__ import annotations

"""Compute home-pose link world transforms via FK and bake into the USD.

The FR3 Duo USD has a structural quirk: each arm's base link (link0) inherits
its world position from a parent scope (left_FR3 / right_FR3) that carries
the pedestal mount transform, but link1–link7 use !resetXformStack! so they
are authored at zero-config world positions assuming the base is at the world
origin.  When physics stops the stage resets and the links appear at those
inconsistent authored positions (disconnected, "collapsed").

This script:
1. Reads the arm base transform from the parent FR3 scope.
2. Runs forward kinematics through each joint using the home-pose angles and
   the joint localPos0/localRot0/localPos1/localRot1 frames from the USD.
3. Writes the resulting link world transforms back as xformOp:translate /
   xformOp:orient, preserving !resetXformStack! so the positions remain
   world-absolute (no change to the inheritance model).

Run inside the Isaac Sim Docker image:

    docker run --rm --entrypoint bash -e ACCEPT_EULA=Y \\
      -v $(pwd):/workspace nvcr.io/nvidia/isaac-sim:5.1.0 -c "
      PYBIN=/isaac-sim/kit/python/bin/python3.11
      USDLIBS=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
      PHYSXSCHEMA=/isaac-sim/extscache/omni.usd.schema.physx-107.3.26+107.3.3.lx64.r.cp311.u353
      PYTHONPATH=\\$USDLIBS:\\$PHYSXSCHEMA LD_LIBRARY_PATH=\\$USDLIBS/bin:\\$PHYSXSCHEMA/bin \\
      \\$PYBIN /workspace/scripts/tooling/bake_home_pose_link_transforms.py \\
        /workspace/assets/digital_twin_fr3Duo.usd"
"""

import math
import sys
from pathlib import Path
from pxr import Sdf, Gf

# ── Home pose joint angles (radians) ─────────────────────────────────────────
HOME = {
    "joint1": 0.0,
    "joint2": -math.pi / 4,        # -45°
    "joint3": 0.0,
    "joint4": -3 * math.pi / 4,    # -135°
    "joint5": 0.0,
    "joint6": math.pi / 2,         #  +90°
    "joint7": math.pi / 4,         #  +45°
}

# ── Arm definitions (paths derived from the dry-run output) ──────────────────
ARMS = [
    {
        "name": "left",
        "FR3_path": "/ai_cell/left_FR3",
        "link0_path": "/ai_cell/left_FR3/left_fr3v2_link0",
        "prefix": "left_fr3v2",
        "joint_paths": [
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_joint1",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_joint2",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_joint3",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_joint4",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_link4/left_fr3v2_joint5",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_link4/left_fr3v2_link5/left_fr3v2_joint6",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_link4/left_fr3v2_link5/left_fr3v2_link6/left_fr3v2_joint7",
        ],
        # link1 through link7 (link0 stays positioned via left_FR3, no change needed)
        "child_link_paths": [
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_link4",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_link4/left_fr3v2_link5",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_link4/left_fr3v2_link5/left_fr3v2_link6",
            "/ai_cell/left_FR3/left_fr3v2_link0/left_fr3v2_link1/left_fr3v2_link2/left_fr3v2_link3/left_fr3v2_link4/left_fr3v2_link5/left_fr3v2_link6/left_fr3v2_link7",
        ],
    },
    {
        "name": "right",
        "FR3_path": "/ai_cell/right_FR3",
        "link0_path": "/ai_cell/right_FR3/right_fr3v2_link0",
        "prefix": "right_fr3v2",
        "joint_paths": [
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_joint1",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_joint2",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_joint3",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_joint4",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_link4/right_fr3v2_joint5",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_link4/right_fr3v2_link5/right_fr3v2_joint6",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_link4/right_fr3v2_link5/right_fr3v2_link6/right_fr3v2_joint7",
        ],
        "child_link_paths": [
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_link4",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_link4/right_fr3v2_link5",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_link4/right_fr3v2_link5/right_fr3v2_link6",
            "/ai_cell/right_FR3/right_fr3v2_link0/right_fr3v2_link1/right_fr3v2_link2/right_fr3v2_link3/right_fr3v2_link4/right_fr3v2_link5/right_fr3v2_link6/right_fr3v2_link7",
        ],
    },
]


# ── Quaternion helpers (pxr convention: w first) ──────────────────────────────

def q_mul(a: Gf.Quatd, b: Gf.Quatd) -> Gf.Quatd:
    return a * b


def q_inv(q: Gf.Quatd) -> Gf.Quatd:
    return q.GetInverse()


def q_rotate(q: Gf.Quatd, v: Gf.Vec3d) -> Gf.Vec3d:
    """Rotate vector v by unit quaternion q."""
    qv = Gf.Quatd(0, v)
    rotated = q * qv * q.GetInverse()
    return rotated.GetImaginary()


def q_from_axis_angle(axis: Gf.Vec3d, angle_rad: float) -> Gf.Quatd:
    half = angle_rad / 2.0
    return Gf.Quatd(math.cos(half), axis * math.sin(half))


def q_from_tuple(t) -> Gf.Quatd:
    """(w, x, y, z) tuple or Gf.Quatf/Quatd → Gf.Quatd."""
    if isinstance(t, (Gf.Quatd, Gf.Quatf, Gf.Quath)):
        img = t.GetImaginary()
        return Gf.Quatd(float(t.GetReal()), Gf.Vec3d(float(img[0]), float(img[1]), float(img[2])))
    return Gf.Quatd(float(t[0]), Gf.Vec3d(float(t[1]), float(t[2]), float(t[3])))


def q_to_quatf(q: Gf.Quatd) -> Gf.Quatf:
    return Gf.Quatf(float(q.GetReal()),
                    Gf.Vec3f(*[float(x) for x in q.GetImaginary()]))


# ── Sdf helpers ───────────────────────────────────────────────────────────────

def read_attr(spec: Sdf.PrimSpec, name: str):
    a = spec.attributes.get(name)
    return a.default if a else None


def read_pos(spec, name: str) -> Gf.Vec3d:
    v = read_attr(spec, name)
    if v is None:
        return Gf.Vec3d(0, 0, 0)
    return Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))


def read_rot(spec, name: str) -> Gf.Quatd:
    v = read_attr(spec, name)
    if v is None:
        return Gf.Quatd(1, Gf.Vec3d(0, 0, 0))
    return q_from_tuple(v)


def write_xform(spec: Sdf.PrimSpec, pos: Gf.Vec3d, rot: Gf.Quatd) -> None:
    """Write xformOp:translate and xformOp:orient, preserving !resetXformStack!."""
    # Translate
    t_attr = spec.attributes.get("xformOp:translate")
    if t_attr is None:
        t_attr = Sdf.AttributeSpec(spec, "xformOp:translate", Sdf.ValueTypeNames.Double3)
    t_attr.default = Gf.Vec3d(pos[0], pos[1], pos[2])

    # Orient — match existing attribute type (Quatd or Quatf)
    o_attr = spec.attributes.get("xformOp:orient")
    if o_attr is None:
        o_attr = Sdf.AttributeSpec(spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd)
    if "quatf" in str(o_attr.typeName).lower():
        o_attr.default = q_to_quatf(rot)
    else:
        o_attr.default = Gf.Quatd(float(rot.GetReal()),
                                  Gf.Vec3d(*[float(x) for x in rot.GetImaginary()]))

    # Ensure xformOpOrder has !resetXformStack! (preserve world-absolute layout)
    order_attr = spec.attributes.get("xformOpOrder")
    if order_attr is None:
        order_attr = Sdf.AttributeSpec(spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray)
    existing = list(order_attr.default or [])
    needed = ["!resetXformStack!", "xformOp:translate", "xformOp:orient", "xformOp:scale"]
    if existing != needed:
        order_attr.default = needed


# ── Forward kinematics ────────────────────────────────────────────────────────

def fk_step(
    body0_pos: Gf.Vec3d,
    body0_rot: Gf.Quatd,
    lp0: Gf.Vec3d,
    lr0: Gf.Quatd,
    angle_rad: float,
    lp1: Gf.Vec3d,
    lr1: Gf.Quatd,
) -> tuple[Gf.Vec3d, Gf.Quatd]:
    """One FK step: body0 → body1 through a revolute joint around Z.

    PhysX joint convention:
      joint_world_pos = body0_world_pos + body0_world_R.Rotate(lp0)
      joint_world_R   = body0_world_R  × lr0
      After revolute rotation θ around joint Z:
        joint_world_R_after = joint_world_R × R(θ, Z)
      body1_world_R = joint_world_R_after × inv(lr1)
      body1_world_pos = joint_world_pos - body1_world_R.Rotate(lp1)
    """
    joint_pos = body0_pos + q_rotate(body0_rot, lp0)
    joint_rot = q_mul(body0_rot, lr0)

    # Revolute rotation around joint-frame Z axis
    joint_rot = q_mul(joint_rot, q_from_axis_angle(Gf.Vec3d(0, 0, 1), angle_rad))

    body1_rot = q_mul(joint_rot, q_inv(lr1))
    body1_pos = joint_pos - q_rotate(body1_rot, lp1)

    return body1_pos, body1_rot


def get_base_transform(layer: Sdf.Layer, arm: dict) -> tuple[Gf.Vec3d, Gf.Quatd]:
    """Compute link0 world transform = FR3_scope × link0_local."""
    fr3 = layer.GetPrimAtPath(Sdf.Path(arm["FR3_path"]))
    lk0 = layer.GetPrimAtPath(Sdf.Path(arm["link0_path"]))

    fr3_pos = read_pos(fr3, "xformOp:translate")
    fr3_rot = read_rot(fr3, "xformOp:orient")

    lk0_pos = read_pos(lk0, "xformOp:translate")
    lk0_rot = read_rot(lk0, "xformOp:orient")

    # link0 inherits FR3 (no !resetXformStack!): world = FR3 × link0_local
    world_pos = fr3_pos + q_rotate(fr3_rot, lk0_pos)
    world_rot = q_mul(fr3_rot, lk0_rot)
    return world_pos, world_rot


def run_fk(layer: Sdf.Layer, arm: dict) -> list[tuple[Gf.Vec3d, Gf.Quatd]]:
    """Return world (pos, rot) for link1 through link7 at home pose."""
    body_pos, body_rot = get_base_transform(layer, arm)
    results = []

    for i, joint_path in enumerate(arm["joint_paths"]):
        joint_name = joint_path.split("/")[-1]
        key = joint_name.replace(arm["prefix"] + "_", "")  # "joint1" … "joint7"
        angle = HOME.get(key, 0.0)

        jspec = layer.GetPrimAtPath(Sdf.Path(joint_path))
        if jspec is None:
            print(f"  WARNING: joint spec not found: {joint_path}")
            results.append((body_pos, body_rot))
            continue

        lp0 = read_pos(jspec, "physics:localPos0")
        lr0 = read_rot(jspec, "physics:localRot0")
        lp1 = read_pos(jspec, "physics:localPos1")
        lr1 = read_rot(jspec, "physics:localRot1")

        body_pos, body_rot = fk_step(body_pos, body_rot, lp0, lr0, angle, lp1, lr1)
        results.append((body_pos, body_rot))
        print(f"  {joint_name} (θ={math.degrees(angle):+.1f}°): "
              f"link{i+1} → ({body_pos[0]:.4f}, {body_pos[1]:.4f}, {body_pos[2]:.4f})")

    return results


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> int:
    usd_path = sys.argv[1] if len(sys.argv) > 1 else "/workspace/assets/digital_twin_fr3Duo.usd"
    usd_path = str(Path(usd_path).expanduser().resolve())

    layer = Sdf.Layer.FindOrOpen(usd_path)
    if layer is None:
        print(f"ERROR: Could not open {usd_path}", file=sys.stderr)
        return 1

    for arm in ARMS:
        print(f"\n=== {arm['name']} arm FK ===")
        link_transforms = run_fk(layer, arm)

        print(f"  Writing {len(link_transforms)} link transforms…")
        for link_path, (pos, rot) in zip(arm["child_link_paths"], link_transforms):
            lspec = layer.GetPrimAtPath(Sdf.Path(link_path))
            if lspec is None:
                print(f"  WARNING: link spec not found: {link_path}")
                continue
            write_xform(lspec, pos, rot)

    layer.Save()
    print("\nSaved in-place.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
