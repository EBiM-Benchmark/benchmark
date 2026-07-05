"""Arm state and control: velocity-IK, idle hold, target seeding, ready pose.

The FR3 arms are driven by velocity actuators. Two consequences shape this
module:
  * moving = mapping a commanded TCP twist to joint velocities (apply_twist_ik)
  * idle   = the actuators produce zero force at step entry when qvel is
    zeroed, so gravity would ratchet the joints down; hard_hold_arm pins the
    joints back to a captured anchor pose every tick instead.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import mujoco

from . import config
from .maths import mat_to_quat
from .mjutil import geom_family_ids, obj_id


@dataclass
class Arm:
    name: str
    tcp_body: int
    target_mocap: int
    joint_ids: list[int]
    dof_ids: list[int]
    act_ids: list[int]
    gripper_joint: int
    gripper_act: int
    pad_left: int
    pad_right: int
    pad_left_contact: set[int]  # pad geom + its lip/bridge family
    pad_right_contact: set[int]
    lateral_sign: float
    q_ref: np.ndarray  # idle hold anchor, refreshed by seed_arm
    target_pos: np.ndarray
    target_quat: np.ndarray
    translate_lock_quat: np.ndarray  # orientation held during pure translation
    rotate_mode: bool = False
    close_ramp: bool = False  # gripper force-servo close in progress
    grasped_body: int | None = None  # cable segment the grasp assist tracks
    grasped_neighbors: list[int] | None = None
    grasp_assist_age: float = 0.0
    grasp_nocontact_time: float = 0.0
    prev_slot_pos: np.ndarray | None = None
    filtered_twist: np.ndarray | None = None  # low-passed twist while grasping
    vertical_xy_active: bool = False
    vertical_xy_anchor: np.ndarray | None = None
    was_command_active: bool = False


def pad_slot_center(data: mujoco.MjData, pad_left: int, pad_right: int) -> np.ndarray:
    """Midpoint between the two finger pads — where a grasped cable sits."""
    return 0.5 * (data.geom_xpos[pad_left] + data.geom_xpos[pad_right])


def make_arm(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> Arm:
    spec = config.ARM_SPECS[name]
    joint_names = spec["joints"]
    joint_ids = [obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) for joint_name in joint_names]
    act_ids = [obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name) for joint_name in joint_names]
    tcp_body = obj_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["tcp"])
    target_body = obj_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["target"])
    gripper_joint = obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, spec["gripper"])
    pad_left = obj_id(model, mujoco.mjtObj.mjOBJ_GEOM, spec["pad_left"])
    pad_right = obj_id(model, mujoco.mjtObj.mjOBJ_GEOM, spec["pad_right"])
    pad_left_contact = geom_family_ids(model, spec["pad_left_prefix"])
    pad_right_contact = geom_family_ids(model, spec["pad_right_prefix"])
    if not pad_left_contact:
        pad_left_contact = {pad_left}
    if not pad_right_contact:
        pad_right_contact = {pad_right}
    q_ref = np.array(
        [float(data.qpos[model.jnt_qposadr[jid]]) for jid in joint_ids],
        dtype=np.float64,
    )
    target_quat = mat_to_quat(data.xmat[tcp_body].reshape(3, 3).copy())
    return Arm(
        name=name,
        tcp_body=tcp_body,
        target_mocap=int(model.body_mocapid[target_body]),
        joint_ids=joint_ids,
        dof_ids=[int(model.jnt_dofadr[jid]) for jid in joint_ids],
        act_ids=act_ids,
        gripper_joint=gripper_joint,
        gripper_act=obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec["gripper"]),
        pad_left=pad_left,
        pad_right=pad_right,
        pad_left_contact=pad_left_contact,
        pad_right_contact=pad_right_contact,
        lateral_sign=float(spec["lateral_sign"]),
        q_ref=q_ref,
        target_pos=data.xpos[tcp_body].copy(),
        target_quat=target_quat.copy(),
        translate_lock_quat=target_quat.copy(),
        grasped_neighbors=[],
        filtered_twist=np.zeros(6, dtype=np.float64),
        prev_slot_pos=pad_slot_center(data, pad_left, pad_right),
        vertical_xy_anchor=data.xpos[tcp_body][:2].copy(),
    )


def sync_target(data: mujoco.MjData, arm: Arm) -> None:
    """Mirror the arm's target into its mocap marker (visual feedback only)."""
    data.mocap_pos[arm.target_mocap] = arm.target_pos
    data.mocap_quat[arm.target_mocap] = arm.target_quat / max(np.linalg.norm(arm.target_quat), 1e-9)


def write_arm_ctrl(model: mujoco.MjModel, data: mujoco.MjData, arm: Arm) -> None:
    for act in arm.act_ids:
        data.ctrl[act] = 0.0


def seed_arm(model: mujoco.MjModel, data: mujoco.MjData, arm: Arm) -> None:
    """Capture the current pose as the new idle-hold anchor and reset all
    per-motion state. Call whenever a motion ends."""
    arm.q_ref[:] = [float(data.qpos[model.jnt_qposadr[jid]]) for jid in arm.joint_ids]
    arm.target_pos = data.xpos[arm.tcp_body].copy()
    arm.target_quat = mat_to_quat(data.xmat[arm.tcp_body].reshape(3, 3).copy())
    arm.translate_lock_quat = arm.target_quat.copy()
    arm.prev_slot_pos = pad_slot_center(data, arm.pad_left, arm.pad_right)
    arm.grasp_assist_age = 0.0
    if arm.filtered_twist is not None:
        arm.filtered_twist[:] = 0.0
    arm.vertical_xy_anchor = data.xpos[arm.tcp_body][:2].copy()
    arm.vertical_xy_active = False
    arm.was_command_active = False
    sync_target(data, arm)
    write_arm_ctrl(model, data, arm)


def hard_hold_arm(model: mujoco.MjModel, data: mujoco.MjData, arm: Arm) -> None:
    """Pin the idle arm at its captured hold pose (arm.q_ref).

    The velocity actuators cannot hold position: with qvel zeroed at every
    tick their force at step entry is zero, so gravity ratchets the joints
    down between ticks. Restoring qpos to the anchor makes idle arms truly
    static; the anchor is refreshed by seed_arm whenever motion ends.
    """
    for i, jid in enumerate(arm.joint_ids):
        data.qpos[model.jnt_qposadr[jid]] = float(arm.q_ref[i])
    for dof in arm.dof_ids:
        data.qvel[dof] = 0.0
    write_arm_ctrl(model, data, arm)


def apply_twist_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: Arm,
    twist: np.ndarray,
) -> None:
    """Map a commanded TCP twist directly to FR3 velocity actuator commands
    (damped least-squares on the 7-dof arm jacobian)."""
    # Jacobians only need pose caches (kinematics + subtree COM); a full
    # mj_forward here would redo collision + constraint solving for nothing.
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    if float(np.linalg.norm(twist)) < 1e-8:
        return

    jacp = np.zeros((3, model.nv), dtype=np.float64)
    jacr = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacBody(model, data, jacp, jacr, arm.tcp_body)
    jac = np.vstack((jacp[:, arm.dof_ids], jacr[:, arm.dof_ids]))
    qvel = jac.T @ np.linalg.solve(jac @ jac.T + (config.IK_DAMPING**2) * np.eye(6), twist)
    norm = float(np.linalg.norm(qvel))
    if norm > config.JOINT_VEL_LIMIT:
        qvel *= config.JOINT_VEL_LIMIT / norm

    for i, act in enumerate(arm.act_ids):
        low, high = model.actuator_ctrlrange[act]
        data.ctrl[act] = float(np.clip(qvel[i], low, high))


def clamp_tcp_twist_for_contact(model: mujoco.MjModel, twist: np.ndarray, grasped: bool) -> np.ndarray:
    """Keep per-physics-step TCP motion small enough for cable contacts to survive."""
    out = twist.copy()
    timestep = max(float(model.opt.timestep), 1e-6)
    max_step = config.GRASPED_MAX_TCP_STEP if grasped else config.FREE_MAX_TCP_STEP
    max_lin_speed = max_step / timestep
    lin_norm = float(np.linalg.norm(out[:3]))
    if lin_norm > max_lin_speed:
        out[:3] *= max_lin_speed / lin_norm

    max_rot_speed = config.MAX_ROT_STEP / timestep
    rot_norm = float(np.linalg.norm(out[3:]))
    if rot_norm > max_rot_speed:
        out[3:] *= max_rot_speed / rot_norm
    return out


def set_arm_ready_pose(model: mujoco.MjModel, data: mujoco.MjData, arm: Arm) -> None:
    """Snap the arm joints to the reach-forward-and-down startup pose."""
    for jid, q in zip(arm.joint_ids, config.ARM_READY_QPOS[arm.name]):
        lo, hi = model.jnt_range[jid]
        data.qpos[model.jnt_qposadr[jid]] = float(np.clip(q, lo, hi))
