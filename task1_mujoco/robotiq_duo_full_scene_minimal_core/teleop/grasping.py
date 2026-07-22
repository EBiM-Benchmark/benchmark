"""Grasping: gripper force-servo close, grasp assist, C-clip guide, haptics.

The gripper closes with a force servo (update_grasp): squeeze until both pads
press the cable with GRIPPER_FORCE_STOP total force, then hold that width.
While a segment is held, a capped-force "grasp assist" spring keeps it in the
pad slot — strong enough to drag the cable over pegs, weak enough that a hard
snag slips out instead of storing energy and catapulting.
"""

from __future__ import annotations

import numpy as np

import mujoco

from . import config, log
from .mjutil import nearest_bodies_to_point
from .robot_arm import Arm, pad_slot_center


def pad_cable_contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pad_left: int | set[int],
    pad_right: int | set[int],
    cable_geoms: set[int],
) -> tuple[bool, float, int]:
    """(both pads touching?, total normal force, contact count) for pad-cable pairs."""
    left_pads = pad_left if isinstance(pad_left, set) else {pad_left}
    right_pads = pad_right if isinstance(pad_right, set) else {pad_right}
    left_force = 0.0
    right_force = 0.0
    count = 0
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = int(con.geom1)
        g2 = int(con.geom2)
        on_left = (g1 in left_pads and g2 in cable_geoms) or (g2 in left_pads and g1 in cable_geoms)
        on_right = (g1 in right_pads and g2 in cable_geoms) or (g2 in right_pads and g1 in cable_geoms)
        if not (on_left or on_right):
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        if on_left:
            left_force += abs(float(force[0]))
        if on_right:
            right_force += abs(float(force[0]))
        count += 1
    return (
        (left_force > config.GRIPPER_CONTACT_SIDE_FORCE and right_force > config.GRIPPER_CONTACT_SIDE_FORCE),
        left_force + right_force,
        count,
    )


def apply_grasp_assist(
    data: mujoco.MjData,
    body_id: int,
    slot_pos: np.ndarray,
    prev_slot_pos: np.ndarray,
    dt: float,
    force_scale: float = 1.0,
) -> float:
    """Capped PD spring pulling one cable segment toward the pad slot.
    Returns the segment's current distance from the slot."""
    pos = data.xpos[body_id]
    err = slot_pos - pos
    dist = float(np.linalg.norm(err))
    slot_vel = (slot_pos - prev_slot_pos) / max(dt, 1e-6)
    slot_speed = float(np.linalg.norm(slot_vel))
    if slot_speed > config.GRASP_ASSIST_SLOT_VEL_LIMIT:
        slot_vel *= config.GRASP_ASSIST_SLOT_VEL_LIMIT / slot_speed
    body_vel = data.cvel[body_id, 3:6]
    force = config.GRASP_ASSIST_KP * err + config.GRASP_ASSIST_KD * (slot_vel - body_vel)
    norm = float(np.linalg.norm(force))
    if norm > config.GRASP_ASSIST_MAX_FORCE:
        force *= config.GRASP_ASSIST_MAX_FORCE / norm
    data.xfrc_applied[body_id, :3] += force * force_scale
    return dist


def release_grasp(data: mujoco.MjData, arm: Arm) -> None:
    """Clear all grasp state (does not open the gripper)."""
    arm.grasped_body = None
    if arm.grasped_neighbors is not None:
        arm.grasped_neighbors.clear()
    arm.grasp_assist_age = 0.0
    arm.grasp_nocontact_time = 0.0
    if arm.filtered_twist is not None:
        arm.filtered_twist[:] = 0.0


def open_gripper(data: mujoco.MjData, arm: Arm) -> None:
    arm.close_ramp = False
    release_grasp(data, arm)
    data.ctrl[arm.gripper_act] = config.GRIPPER_OPEN


def update_grasp(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: Arm,
    cable_geoms: set[int],
    cable_bodies: list[int],
    grasp_assist: bool,
    dt: float,
) -> None:
    """Advance the close servo and the grasp-assist state for one arm.
    Must run after xfrc_applied is cleared and before mj_step."""
    if arm.close_ramp:
        both, force, count = pad_cable_contacts(
            model,
            data,
            arm.pad_left_contact,
            arm.pad_right_contact,
            cable_geoms,
        )
        if both and grasp_assist and arm.grasped_body is None:
            # attach the assist at first two-sided contact for responsiveness
            slot = pad_slot_center(data, arm.pad_left, arm.pad_right)
            picked = nearest_bodies_to_point(data, cable_bodies, slot, count=3)
            arm.grasped_body = picked[0] if picked else None
            if arm.grasped_neighbors is None:
                arm.grasped_neighbors = []
            arm.grasped_neighbors[:] = picked[1:]
            arm.grasp_assist_age = 0.0
            arm.grasp_nocontact_time = 0.0
            arm.prev_slot_pos = slot.copy()
        if both and force >= config.GRIPPER_FORCE_STOP:
            # firmly squeezed: the fingers have visually closed onto the
            # cable (a low threshold stops while the cable only grazes
            # pad edges and the fingers still gape open)
            q_close = float(data.qpos[model.jnt_qposadr[arm.gripper_joint]])
            data.ctrl[arm.gripper_act] = min(config.GRIPPER_CLOSE, q_close + config.GRIPPER_OVERDRIVE)
            arm.close_ramp = False
            log(
                f"{arm.name} grasp hold: contacts={count} force={force:.3f} "
                f"target={data.ctrl[arm.gripper_act]:.3f} ncon={data.ncon}"
            )
        elif both:
            # touched but not firm yet — keep squeezing gently
            data.ctrl[arm.gripper_act] = min(
                config.GRIPPER_CLOSE,
                float(data.ctrl[arm.gripper_act]) + 0.5 * config.GRIPPER_CLOSE_RATE * dt,
            )
        else:
            data.ctrl[arm.gripper_act] = min(
                config.GRIPPER_CLOSE,
                float(data.ctrl[arm.gripper_act]) + config.GRIPPER_CLOSE_RATE * dt,
            )

    if grasp_assist and arm.grasped_body is not None:
        slot = pad_slot_center(data, arm.pad_left, arm.pad_right)
        prev = arm.prev_slot_pos if arm.prev_slot_pos is not None else slot.copy()
        arm.grasp_assist_age += dt
        if arm.grasp_assist_age < config.GRASP_ASSIST_START_DELAY:
            arm.prev_slot_pos = slot.copy()
            return
        # smoothstep ramp-in so the assist never kicks a fresh grasp
        assist_t = arm.grasp_assist_age - config.GRASP_ASSIST_START_DELAY
        ramp = min(1.0, assist_t / max(config.GRASP_ASSIST_RAMP_TIME, 1e-6))
        ramp = ramp * ramp * (3.0 - 2.0 * ramp)
        dist = apply_grasp_assist(data, arm.grasped_body, slot, prev, dt, force_scale=ramp)
        for neighbor in arm.grasped_neighbors or []:
            apply_grasp_assist(data, neighbor, slot, prev, dt, force_scale=0.25 * ramp)
        arm.prev_slot_pos = slot.copy()
        _, _, pad_count = pad_cable_contacts(
            model,
            data,
            arm.pad_left_contact,
            arm.pad_right_contact,
            cable_geoms,
        )
        if pad_count == 0:
            arm.grasp_nocontact_time += dt
        else:
            arm.grasp_nocontact_time = 0.0
        # a cable that ESCAPED at speed must be dropped immediately: keeping
        # the capped assist spring attached to a free-flying 0.7 g segment
        # for the usual grace period whips it violently (measured 19 m/s
        # spikes vs ~6 m/s for the escape itself)
        body_speed = float(np.linalg.norm(data.cvel[arm.grasped_body, 3:6]))
        fast_escape = (
            pad_count == 0
            and body_speed > config.GRASP_ESCAPE_SPEED
            and dist > config.GRASP_ESCAPE_DIST
        )
        escaped = fast_escape or arm.grasp_nocontact_time > config.GRASP_NOCONTACT_RELEASE_TIME
        if (
            escaped
            or dist > config.GRASP_ASSIST_RELEASE_DIST
            or float(data.ctrl[arm.gripper_act]) <= config.GRIPPER_OPEN + 0.02
        ):
            if escaped:
                log(f"{arm.name} grasp released: cable left the pads")
            release_grasp(data, arm)


def apply_clip_guide(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    clip_body: int,
    cable_bodies: list[int],
) -> None:
    """Hold clipped cable segments in the C-slot while letting them slide.

    Segments inside the pocket get a capped spring toward the slot
    centerline in the clip's y/z only — never along the slot axis — so the
    cable slides along and through the clip during routing; pulling out of
    the open mouth harder than CLIP_HOLD_FORCE simply extracts it. Must be
    called after xfrc_applied is cleared and before mj_step.
    """
    origin = data.xpos[clip_body]
    rel = data.xpos[cable_bodies] - origin
    inside = np.all((rel >= config.CLIP_ZONE_LO) & (rel <= config.CLIP_ZONE_HI), axis=1)
    for i in np.nonzero(inside)[0]:
        seg = int(cable_bodies[int(i)])
        err = config.CLIP_SEAT_LOCAL - rel[int(i), 1:3]
        vel = data.cvel[seg, 4:6]
        force_yz = config.CLIP_GUIDE_KP * err - config.CLIP_GUIDE_KD * vel
        norm = float(np.linalg.norm(force_yz))
        if norm > config.CLIP_HOLD_FORCE:
            force_yz *= config.CLIP_HOLD_FORCE / norm
        data.xfrc_applied[seg, 1] += force_yz[0]
        data.xfrc_applied[seg, 2] += force_yz[1]


# --------------------------------------------------------------------------
# haptics: contact anywhere on the gripper -> controller rumble
# --------------------------------------------------------------------------


def gripper_geom_ids(model: mujoco.MjModel, side_prefix: str) -> set[int]:
    """All collidable geoms of one gripper (pads, lips, finger/knuckle and
    housing meshes) — the haptic source: any gripper contact should rumble."""
    ids: set[int] = set()
    for gid in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or ""
        if body.startswith(side_prefix) and "robotiq" in body:
            if model.geom_contype[gid] or model.geom_conaffinity[gid]:
                ids.add(gid)
    return ids


def geoms_contact_force(model: mujoco.MjModel, data: mujoco.MjData, geoms: set[int]) -> float:
    """Total contact normal force involving any geom in the set."""
    total = 0.0
    wrench = np.zeros(6)
    for i in range(data.ncon):
        con = data.contact[i]
        if int(con.geom1) in geoms or int(con.geom2) in geoms:
            mujoco.mj_contactForce(model, data, i, wrench)
            total += abs(float(wrench[0]))
    return total
