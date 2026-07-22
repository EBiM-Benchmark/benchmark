"""Scene setup: model loading, cable identification and initial layout,
and spawn placement of the mobile base next to the cable."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import mujoco

from . import config, log
from .maths import frame_from_y_axis, mat_to_quat, round_polyline_corners, sample_polyline
from .mjutil import obj_id, optional_obj_id, planar_body_axis
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

    b_first_id = obj_id(model, mujoco.mjtObj.mjOBJ_BODY, "B_first")
    anchor = model.body_pos[b_first_id].copy()
    z = config.CABLE_BOARD_Z

    # B_first has no joint (it's welded rigidly to fixture_group), so its
    # visible capsule geom (G0) and the fixed B_first->B_1 offset both come
    # straight from body_quat[B_first] - confirmed empirically that this is
    # the ONLY lever that moves G0 at all (geom_pos/geom_quat[G0] and
    # body_pos[B_1] are both ignored by the elasticity.cable plugin's
    # rendering). The compiler sets body_quat[B_first] to a fixed 90deg-
    # about-Z, pointing this ~4.3cm nub along local +Y (toward the board's
    # far edge) - which only looked right when adapter_0 sat elsewhere;
    # now that it's at the top edge it visibly kinks upward. Overriding it
    # to identity redirects the nub to point local +X ("right"), but that
    # rotation ALSO uniformly rotates every later segment's actual world
    # orientation (each B_i's true orientation is body_quat[B_first] @
    # frame_from_y_axis(direction[i-1]) - not just qpos alone), since the
    # original 90deg-about-Z exactly cancelled the -90deg that
    # frame_from_y_axis always introduces (its local X axis is
    # cross(direction, up), which for any direction in the XY plane is a
    # -90deg rotation of that direction). So every direction fed into the
    # loop below is pre-rotated +90deg about Z to restore that
    # cancellation - verified empirically to reproduce the exact same
    # B_1->B_2 offset as before this override.
    model.body_quat[b_first_id] = (1.0, 0.0, 0.0, 0.0)
    # 2026-07-15: all 7 fixture positions were recomputed from the official
    # Tier-2 grid coordinates via a verified grid->local similarity map
    # (~-89 deg rotation, i.e. an axis swap: local_x ~ grid_col, local_y ~
    # -grid_row) - the previous two attempts (a straight adapter position
    # swap, then a point-reflection of the peg layout) both turned out wrong
    # because they patched the old numbers with a guessed transform instead
    # of re-deriving positions from the grid. This path follows the ACTUAL
    # official visit order (0 -> -1 -> +2 -> +4 -> -3 -> -5 -> 6), each
    # waypoint nudged ~3cm radially outward from the board center so the
    # straight-line segments clear the fixtures' collision geometry instead
    # of cutting through their centers.
    path = np.array(
        [
            anchor,
            [-0.0490, 0.3363, z],  # near clip (-1)
            [0.3512, 0.3328, z],  # near peg (+2)
            [-0.0893, 0.0867, z],  # near peg (+4)
            [0.2533, 0.1087, z],  # near peg (-3)
            [-0.1680, -0.1417, z],  # near peg (-5)
            [0.50, -0.38, z],  # near adapter_1 (6)
        ],
        dtype=np.float64,
    )
    path = round_polyline_corners(path, radius_frac=0.35)
    samples = sample_polyline(path, len(chain) + 1)
    directions = samples[1:] - samples[:-1]
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    # compensate body_quat[B_first] above being forced to identity (see
    # comment there): rotate +90deg about Z, (x, y) -> (-y, x)
    directions = np.column_stack([-directions[:, 1], directions[:, 0], directions[:, 2]])

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


def teleport_base_fixed(
    model: mujoco.MjModel, data: mujoco.MjData, target_xy: tuple[float, float], yaw: float
) -> bool:
    """Place the mobile base at a fixed world (x, y, yaw), independent of the
    cable. For data-collection recording setups that need a repeatable spawn
    (e.g. a head-camera framing a specific fixture), not the cable-relative
    grasp spawn ``teleport_base_near_cable`` uses."""
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

    data.qpos[joints["yaw"]] = yaw
    mujoco.mj_kinematics(model, data)

    # planar slides live in a rotated parent frame, same numeric Jacobian
    # solve as teleport_base_near_cable but targeting the base body itself
    jac = np.zeros((2, 2))
    pos0 = data.xpos[base_body][:2].copy()
    for col, qa in enumerate((joints["x"], joints["y"])):
        data.qpos[qa] += 1e-4
        mujoco.mj_kinematics(model, data)
        jac[:, col] = (data.xpos[base_body][:2] - pos0) / 1e-4
        data.qpos[qa] -= 1e-4
    try:
        dq = np.linalg.solve(jac, np.asarray(target_xy, dtype=np.float64) - pos0)
    except np.linalg.LinAlgError:
        return False
    data.qpos[joints["x"]] += dq[0]
    data.qpos[joints["y"]] += dq[1]
    mujoco.mj_forward(model, data)
    return True


# fixed spawn for the data-collection recording setup (Jerry's reference
# screenshot: robot stands on the poster-wall side (world y=2.53, past the
# board's cclip-side long edge at y~1.045), back to the poster, facing the
# board so head_cam frames fixture 0 / adapter_0 clearly). Tuned by
# rendering head_cam at candidate poses until fixture 0 was well framed -
# not derived from a formula, so re-tune by rendering again if the board
# layout changes.
DATA_COLLECTION_SPAWN_XY = (1.25, 1.8)
DATA_COLLECTION_SPAWN_YAW_DEG = 0.0
# ZED Mini stereo camera spec (Stereolabs, stereolabs.com/store/products/zed-mini):
# FOV 90(H) x 60(V) x 100(D) deg, 63mm baseline, up to 2K (2208x1242/eye, 16:9)
DATA_COLLECTION_CAM_FOVY = 60.0
DATA_COLLECTION_CAM_BASELINE = 0.063  # meters, ZED Mini stereo baseline
# symmetric downward tilt, straight ahead (no yaw/roll skew toward any one
# fixture - a "look-at" rotation was tried and rejected: it left the board
# looking skewed/diagonal in frame). Re-tuned for the closer spawn above -
# 20deg (tuned for the earlier, farther spawn) mostly showed wall from here;
# 50deg frames the whole board cleanly.
DATA_COLLECTION_CAM_PITCH_DEG = 50.0


def aim_head_cam_straight_down(
    model: mujoco.MjModel, data: mujoco.MjData, forward_axis: str = "x"
) -> bool:
    """Tilt head_cam down by DATA_COLLECTION_CAM_PITCH_DEG from the robot's
    own straight-ahead heading (no left/right skew toward a specific
    fixture) and set its fovy to the real ZED Mini spec, by overriding its
    compile-time local quat/fovy - same late-mutation pattern as
    disable_wheel_ground_collision. Call this AFTER the base is at its
    final spawn pose (the heading is read from the base's current yaw)."""
    cam_id = optional_obj_id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_cam")
    base_body = optional_obj_id(model, mujoco.mjtObj.mjOBJ_BODY, config.BASE_BODY)
    if cam_id is None or base_body is None:
        return False
    mount_body = int(model.cam_bodyid[cam_id])
    mount_r = data.xmat[mount_body].reshape(3, 3)

    horiz_fwd = planar_body_axis(data, base_body, forward_axis)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(horiz_fwd, world_up)
    right /= np.linalg.norm(right)

    pitch = math.radians(DATA_COLLECTION_CAM_PITCH_DEG)
    forward = horiz_fwd * math.cos(pitch) - world_up * math.sin(pitch)
    forward /= np.linalg.norm(forward)
    up = np.cross(right, forward)
    # MuJoCo camera convention: local -Z is the view direction, local Y is up
    cam_world_r = np.column_stack([right, up, -forward])
    cam_quat = mat_to_quat(mount_r.T @ cam_world_r)
    model.cam_quat[cam_id] = cam_quat
    model.cam_fovy[cam_id] = DATA_COLLECTION_CAM_FOVY

    # right eye of the stereo pair: same tilt, repositioned along the NEW
    # (tilted) right axis by the ZED Mini baseline so it stays perpendicular
    # to the new view direction, not the XML's original untilted one
    right_id = optional_obj_id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_cam_right")
    if right_id is not None:
        model.cam_quat[right_id] = cam_quat
        model.cam_pos[right_id] = model.cam_pos[cam_id] + mount_r.T @ (right * DATA_COLLECTION_CAM_BASELINE)
        model.cam_fovy[right_id] = DATA_COLLECTION_CAM_FOVY

    mujoco.mj_forward(model, data)
    return True


def spawn_for_data_collection(
    model: mujoco.MjModel, data: mujoco.MjData, forward_axis: str = "x"
) -> bool:
    """Fixed spawn + head_cam aim for the data-collection recording setup.
    See DATA_COLLECTION_SPAWN_* above."""
    ok = teleport_base_fixed(
        model, data, DATA_COLLECTION_SPAWN_XY, math.radians(DATA_COLLECTION_SPAWN_YAW_DEG)
    )
    return aim_head_cam_straight_down(model, data, forward_axis) if ok else False
