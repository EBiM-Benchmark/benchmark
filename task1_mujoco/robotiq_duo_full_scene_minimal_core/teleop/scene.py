"""Scene setup: model loading, cable identification and initial layout,
and spawn placement of the mobile base next to the cable."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import mujoco

from . import config, log
from .maths import frame_from_y_axis, mat_to_quat, sample_polyline
from .mjutil import obj_id, optional_obj_id
from .robot_arm import Arm, pad_slot_center

# the XML lives one directory above the package, next to main.py
XML = Path(__file__).resolve().parent.parent / "duo_full_scene_grasp.xml"


def load_model(
    *,
    timestep: float | None,
    noslip_iterations: int | None,
    wheel_collision: bool,
) -> mujoco.MjModel:
    """Load the scene XML and apply the physics command-line overrides."""
    model = mujoco.MjModel.from_xml_path(str(XML))
    if not wheel_collision:
        n_wheel = disable_wheel_ground_collision(model)
        if n_wheel:
            log(f"[base] wheel-ground collision off ({n_wheel} geoms); planar drive owns base motion")
    if noslip_iterations is not None:
        model.opt.noslip_iterations = max(0, int(noslip_iterations))
        log(f"[physics] noslip_iterations={model.opt.noslip_iterations}")
    if timestep is not None and timestep > 0:
        model.opt.timestep = float(timestep)
        log(f"[physics] timestep={model.opt.timestep}")
    return model


def disable_wheel_ground_collision(model: mujoco.MjModel) -> int:
    """Turn off wheel/caster contact for the planar-joint drive modes.

    The base has no z joint — it rides at fixed height on the virtual planar
    joints — so wheel-floor contact is purely parasitic friction that fights
    the planar drive (with braked wheels it is full sliding friction). Only
    the 'wheel' base-control mode needs real ground traction.
    """
    count = 0
    for gid in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or ""
        if "caster_" in body or "argo_drive_" in body:
            if model.geom_contype[gid] or model.geom_conaffinity[gid]:
                model.geom_contype[gid] = 0
                model.geom_conaffinity[gid] = 0
                count += 1
    return count


# --------------------------------------------------------------------------
# cable
# --------------------------------------------------------------------------


def cable_geom_ids(model: mujoco.MjModel) -> set[int]:
    """Composite cable capsule geoms (the plugin names them G0, G1, ...)."""
    ids: set[int] = set()
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith("G"):
            ids.add(gid)
    return ids


def cable_body_ids(model: mujoco.MjModel) -> list[int]:
    """Composite cable segment bodies (B_first, B_1, ..., B_last)."""
    ids: list[int] = []
    for bid in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
        if (name.startswith("B_") or name in ("B_first", "B_last")) and model.body_dofnum[bid] > 0:
            ids.append(bid)
    return ids


def initialize_cable_on_board(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Lay the composite cable on the board using the proven old-full S path."""
    chain = cable_body_ids(model)
    if len(chain) < 3:
        return

    anchor = model.body_pos[obj_id(model, mujoco.mjtObj.mjOBJ_BODY, "B_first")].copy()
    z = config.CABLE_BOARD_Z
    path = np.array(
        [
            anchor,
            [-0.4943, -0.3821, z],
            [-0.48, -0.32, z],
            [-0.42, 0.28, z],
            [-0.10, 0.28, z],
            [-0.10, -0.28, z],
            [0.18, -0.28, z],
            [0.18, 0.28, z],
            [0.50, 0.38, z],
        ],
        dtype=np.float64,
    )
    samples = sample_polyline(path, len(chain) + 1)
    directions = samples[1:] - samples[:-1]
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)

    # each segment's ball joint stores its rotation relative to the parent
    # segment, so walk the chain accumulating frames
    parent_frame = np.eye(3)
    for bid, direction in zip(chain, directions):
        joint_id = int(model.body_jntadr[bid])
        qadr = int(model.jnt_qposadr[joint_id])
        desired = frame_from_y_axis(direction)
        rel = parent_frame.T @ desired
        data.qpos[qadr : qadr + 4] = mat_to_quat(rel)
        parent_frame = desired
    mujoco.mj_forward(model, data)


# --------------------------------------------------------------------------
# spawn placement
# --------------------------------------------------------------------------


def teleport_base_near_cable(model: mujoco.MjModel, data: mujoco.MjData, arm: Arm) -> bool:
    """Place the mobile base so the arm's gripper slot hangs above the cable.

    Spawns the robot facing the board with the pad slot directly over a cable
    segment near the board's front edge, so grasping only needs a vertical
    descent and close. Pure planar qpos placement — no dynamics involved.
    """
    base_body = optional_obj_id(model, mujoco.mjtObj.mjOBJ_BODY, config.BASE_BODY)
    joints = {}
    for name, jn in (
        ("x", config.BASE_X_JOINT),
        ("y", config.BASE_Y_JOINT),
        ("yaw", config.BASE_YAW_JOINT),
    ):
        j = optional_obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if j is None:
            return False
        joints[name] = int(model.jnt_qposadr[j])
    if base_body is None:
        return False
    chain = cable_body_ids(model)
    if len(chain) < 16:
        return False
    # grab near the free end of the cable (the other end is anchored to the
    # board adapter, so the free side is the natural one to manipulate)
    target = data.xpos[chain[-4]].copy()

    def slot_xy() -> np.ndarray:
        return pad_slot_center(data, arm.pad_left, arm.pad_right)[:2].copy()

    # rotate at the spawn point (clear of furniture) so that, once the slot
    # is translated onto the target, the base body lands outside the table
    # footprint and faces the grasp point
    table_lo = np.array([0.84 - 0.42, -0.095 - 0.42])
    table_hi = np.array([2.34 + 0.42, 1.255 + 0.42])
    yaw0 = float(data.qpos[joints["yaw"]])
    best = (yaw0, -1e9)
    for cand in np.linspace(-math.pi, math.pi, 32, endpoint=False):
        data.qpos[joints["yaw"]] = yaw0 + cand
        mujoco.mj_kinematics(model, data)
        off = slot_xy() - data.xpos[base_body][:2]
        base_new = target[:2] - off
        outward = base_new - target[:2]
        outward /= max(float(np.linalg.norm(outward)), 1e-9)
        # prefer standing back along +x/+y (away from the board interior)
        score = float(np.dot(outward, [0.8, 0.6]))
        if np.any(base_new < table_lo) or np.any(base_new > table_hi):
            score += 10.0  # hard preference for poses clear of the table
        # keep the idle-arm sweep (~1.25 m) clear of the real walls:
        # back wall at y=+2.53, left partition at x=-1.59 (y>0.75)
        if float(base_new[1]) > 1.25 or float(base_new[0]) < -0.30:
            score -= 20.0
        if score > best[1]:
            best = (yaw0 + cand, score)
    data.qpos[joints["yaw"]] = best[0]
    mujoco.mj_kinematics(model, data)

    # the planar slide joints live in a rotated parent frame, so solve the
    # qpos -> world-xy map numerically instead of assuming identity
    jac = np.zeros((2, 2))
    slot0 = slot_xy()
    for col, qa in enumerate((joints["x"], joints["y"])):
        data.qpos[qa] += 1e-4
        mujoco.mj_kinematics(model, data)
        jac[:, col] = (slot_xy() - slot0) / 1e-4
        data.qpos[qa] -= 1e-4
    try:
        dq = np.linalg.solve(jac, target[:2] - slot0)
    except np.linalg.LinAlgError:
        return False
    data.qpos[joints["x"]] += dq[0]
    data.qpos[joints["y"]] += dq[1]
    mujoco.mj_forward(model, data)
    return True
