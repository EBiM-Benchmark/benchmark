#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Configurable MuJoCo teleoperation for the Mobile FR3 Duo coffee scene.

Two arm-control modes are supported, both using arm position actuators:

* cartesian_velocity_position_servo: keyboard TCP velocity -> damped
  least-squares IK -> limited joint velocity -> short-horizon joint-position
  target.  When the key is released, the last target is retained until reached.
* direct_joint_position: seven joint targets from config.json ->
  velocity-limited position-actuator trajectory. No IK is used.

The base uses velocity actuators. The spine and Robotiq grippers retain
position actuators and a contact-force-limited gripper preload procedure.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

if sys.platform.startswith("linux") and os.environ.get("XDG_SESSION_TYPE") == "wayland":
    os.environ.setdefault("GLFW_PLATFORM", "x11")

import glfw
import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent

ARM_SPECS = {
    "left": {
        "joints": [f"left_fr3v2_1_joint{i}" for i in range(1, 8)],
        "tcp_site": "left_arm_control_tcp_pivot",
        "target": "left_tcp_mocap_target",
        "gripper": "left_fr3v2_1_robotiq_85_left_knuckle_joint",
        "gripper_base": "left_fr3v2_1_robotiq_85_base_link",
        "flange": "left_fr3v2_1_link8",
        "attachment_freejoint": "left_gripper_attachment_freejoint",
        "mount_quat": np.array([0.92388, 0.0, 0.0, -0.382683]),
        "pad_left_prefix": "left_actual_left_tip_environment_collision",
        "pad_right_prefix": "left_actual_right_tip_environment_collision",
        "body_prefix": "left_fr3v2_1",
    },
    "right": {
        "joints": [f"right_fr3v2_1_joint{i}" for i in range(1, 8)],
        "tcp_site": "right_arm_control_tcp_pivot",
        "target": "right_tcp_mocap_target",
        "gripper": "right_fr3v2_1_robotiq_85_left_knuckle_joint",
        "gripper_base": "right_fr3v2_1_robotiq_85_base_link",
        "flange": "right_fr3v2_1_link8",
        "attachment_freejoint": "right_gripper_attachment_freejoint",
        "mount_quat": np.array([0.92388, 0.0, 0.0, -0.382683]),
        "pad_left_prefix": "right_actual_left_tip_environment_collision",
        "pad_right_prefix": "right_actual_right_tip_environment_collision",
        "body_prefix": "right_fr3v2_1",
    },
}

BASE_BODY = "base_link"
BASE_JOINTS = ("base_planar_x", "base_planar_y", "base_yaw")
BASE_ACTUATORS = ("base_planar_x_velocity", "base_planar_y_velocity", "base_yaw_velocity")
SPINE_JOINT = "franka_spine_vertical_joint"
SPINE_ACTUATOR = "franka_spine_vertical_joint"
OBJECT_FREEJOINTS = {
    "bowl": "dynamic_bowl2_freejoint",
    "plate": "dynamic_plate2_freejoint",
    "spoon": "dynamic_spoon2_freejoint",
    "cup": "dynamic_cup_freejoint",
    "simple_tray": "dynamic_simple_tray_freejoint",
}

HEAD_VISUAL_BODY = "configurable_visual_head"
HEAD_VISUAL_GEOMS = ("visual_head_skin", "visual_head_eye_left", "visual_head_eye_right")
HEAD_ANCHOR_BODIES = {label: f"usd_visual_{label}_edit" for label in "ABCDEFGHI"}

SCALE_FORCE_SENSOR = "ikea_scale_force_sensor"

# Public runtime values updated after every simulation step. These can be read
# by other code that imports this module and runs the simulation in-process.
scale_force_vector_n = np.zeros(3, dtype=np.float64)
scale_weight_force_n: float = 0.0
scale_weight_kg: float = 0.0

HELP = """
Controls
  7 : base/spine mode
  8 : left arm mode
  9 : right arm mode

Base mode
  W/S or Up/Down       : forward/backward
  A/D or Left/Right    : left/right
  Q/E or Home/End      : yaw left/right
  U/J or PageUp/PageDn : spine up/down

Arm mode (cartesian_velocity_position_servo only)
  Translation mode:
    W/S : +X/-X of configured end-effector frame
    A/D : -Y/+Y of configured end-effector frame
    U/J : +Z/-Z of configured end-effector frame
  R toggles rotation mode:
    U/J : roll +X/-X
    W/S : pitch +Y/-Y
    A/D : yaw +Z/-Z

Other
  G          : close selected gripper until the contact-force threshold
  V or Space : open selected gripper
  P          : print measured state and current commands/targets
  T          : tare the scale at its current load
  L          : reload direct joint-position targets from config.json
  Esc        : exit

Frames are selected in config.json: base, camera, or world.
"""


def log(message: str) -> None:
    print(message, flush=True)


def object_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    idx = int(mujoco.mj_name2id(model, obj_type, name))
    if idx < 0:
        raise RuntimeError(f"Required MuJoCo object not found: {name}")
    return idx


def optional_object_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int | None:
    idx = int(mujoco.mj_name2id(model, obj_type, name))
    return None if idx < 0 else idx


def vec(value: Any, length: int, label: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (length,) or not np.all(np.isfinite(arr)):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return arr


def positive_vec(value: Any, length: int, label: str) -> np.ndarray:
    arr = vec(value, length, label)
    if np.any(arr <= 0.0):
        raise ValueError(f"{label} values must be positive")
    return arr


def normalized_quat(value: Any, label: str) -> np.ndarray:
    quat = vec(value, 4, label)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        raise ValueError(f"{label} must have nonzero norm")
    return quat / norm


def load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text())
    mode = config["scene"]["arm_control_mode"]
    if mode == "cartesian_velocity_ik":
        # Backward-compatible alias for older config files.
        mode = "cartesian_velocity_position_servo"
        config["scene"]["arm_control_mode"] = mode
    if mode not in ("cartesian_velocity_position_servo", "direct_joint_position"):
        raise ValueError(
            "scene.arm_control_mode must be "
            "cartesian_velocity_position_servo or direct_joint_position"
        )
    if int(config["scene"]["bean_count"]) not in (100, 300):
        raise ValueError("scene.bean_count must be 100 or 300")
    for key in ("base", "end_effector"):
        if config["motion_frames"][key] not in ("base", "camera", "world"):
            raise ValueError(f"motion_frames.{key} must be base, camera, or world")
    for key in (
        "base_linear_m_s", "base_angular_rad_s", "end_effector_linear_m_s",
        "end_effector_angular_rad_s", "spine_m_s",
    ):
        if not math.isfinite(float(config["velocities"][key])) or float(config["velocities"][key]) < 0:
            raise ValueError(f"velocities.{key} must be finite and nonnegative")
    for key in ("arm_joint_velocity_rad_s", "arm_joint_torque_nm"):
        positive_vec(config["limits"][key], 7, f"limits.{key}")
    for key in (
        "base_force_n",
        "base_yaw_torque_nm",
        "spine_force_n",
        "spine_acceleration_m_s2",
        "spine_deceleration_m_s2",
    ):
        value = float(config["limits"][key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"limits.{key} must be finite and positive")
    for side in ("left", "right"):
        vec(config["initial_robot"][f"{side}_arm_joint_position_rad"], 7, f"initial_robot.{side}")
        vec(config["joint_position_targets"][f"{side}_rad"], 7, f"joint_position_targets.{side}")
    normalized_quat(config["initial_robot"]["base_orientation_wxyz"], "initial robot quaternion")
    for name in OBJECT_FREEJOINTS:
        normalized_quat(config["initial_objects"][name]["orientation_wxyz"], f"{name} quaternion")
        vec(config["initial_objects"][name]["position_world_m"], 3, f"{name} position")
    for key in (
        "arm_target_lookahead_s",
        "arm_max_target_lead_rad",
        "spine_position_kp",
        "spine_position_kv",
        "gripper_contact_force_threshold_n",
        "gripper_contact_hold_increment_rad",
    ):
        value = float(config["controller"][key])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"controller.{key} must be finite and positive")
    filter_alpha = float(config["controller"]["gripper_force_filter_alpha"])
    if not math.isfinite(filter_alpha) or not 0.0 < filter_alpha <= 1.0:
        raise ValueError("controller.gripper_force_filter_alpha must be in (0, 1]")
    confirmation_steps = int(config["controller"]["gripper_contact_confirmation_steps"])
    if confirmation_steps <= 0:
        raise ValueError("controller.gripper_contact_confirmation_steps must be positive")
    cup_friction = vec(config["contact"]["cup_friction"], 3, "contact.cup_friction")
    if np.any(cup_friction < 0.0):
        raise ValueError("contact.cup_friction entries must be nonnegative")
    cup_priority = int(config["contact"]["cup_contact_priority"])
    if cup_priority < 0:
        raise ValueError("contact.cup_contact_priority must be nonnegative")

    camera_views = config.get("camera_views", {})
    if not isinstance(camera_views.get("enabled", False), bool):
        raise ValueError("camera_views.enabled must be true or false")
    for camera_name in ("head_cam", "left_wrist_cam", "right_wrist_cam"):
        if not isinstance(camera_views.get(camera_name, False), bool):
            raise ValueError(f"camera_views.{camera_name} must be true or false")
    for key in ("width", "height"):
        value = int(camera_views.get(key, 0))
        if value <= 0:
            raise ValueError(f"camera_views.{key} must be a positive integer")
    camera_render_hz = float(camera_views.get("render_hz", 0.0))
    if not math.isfinite(camera_render_hz) or camera_render_hz <= 0.0:
        raise ValueError("camera_views.render_hz must be finite and positive")

    head_visual = config.get("head_visual", {})
    if not isinstance(head_visual.get("enabled", True), bool):
        raise ValueError("head_visual.enabled must be true or false")
    anchor = str(head_visual.get("anchor_label", "A")).upper()
    if anchor not in HEAD_ANCHOR_BODIES:
        raise ValueError("head_visual.anchor_label must be one of A, B, C, D, E, F, G, H, or I")
    head_visual["anchor_label"] = anchor
    vec(head_visual.get("offset_world_m", [0.0, 0.0, 0.1763177378]), 3, "head_visual.offset_world_m")
    normalized_quat(
        head_visual.get("orientation_wxyz", [0.7071067812, 0.7071067812, 0.0, 0.0]),
        "head_visual.orientation_wxyz",
    )
    config["head_visual"] = head_visual

    scale_sensor = config.get("scale_sensor", {})
    if not isinstance(scale_sensor.get("enabled", True), bool):
        raise ValueError("scale_sensor.enabled must be true or false")
    if not isinstance(scale_sensor.get("auto_tare", True), bool):
        raise ValueError("scale_sensor.auto_tare must be true or false")
    scale_alpha = float(scale_sensor.get("filter_alpha", 0.2))
    if not math.isfinite(scale_alpha) or not 0.0 < scale_alpha <= 1.0:
        raise ValueError("scale_sensor.filter_alpha must be in (0, 1]")
    scale_gravity = float(scale_sensor.get("gravity_m_s2", 9.81))
    if not math.isfinite(scale_gravity) or scale_gravity <= 0.0:
        raise ValueError("scale_sensor.gravity_m_s2 must be finite and positive")
    scale_tare = float(scale_sensor.get("tare_force_n", 0.0))
    if not math.isfinite(scale_tare) or scale_tare < 0.0:
        raise ValueError("scale_sensor.tare_force_n must be finite and nonnegative")
    config["scale_sensor"] = scale_sensor
    return config


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    out = np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=np.float64)
    return out / max(float(np.linalg.norm(out)), 1e-12)


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    p = np.array([0.0, v[0], v[1], v[2]], dtype=np.float64)
    r = quat_mul_raw(quat_mul_raw(q, p), quat_conjugate(q))
    return r[1:]


def quat_mul_raw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=np.float64)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def quat_to_rpy(q: np.ndarray) -> tuple[float, float, float]:
    w, x, y, z = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(float(np.clip(2*(w*y-z*x), -1.0, 1.0)))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def mat_to_quat(matrix: np.ndarray) -> np.ndarray:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix, dtype=np.float64).reshape(9))
    return quat / max(float(np.linalg.norm(quat)), 1e-12)


def set_ctrl_clipped(model: mujoco.MjModel, data: mujoco.MjData, aid: int, value: float) -> None:
    if bool(model.actuator_ctrllimited[aid]):
        lo, hi = model.actuator_ctrlrange[aid]
        value = float(np.clip(value, lo, hi))
    data.ctrl[aid] = float(value)


def freejoint_pose(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> tuple[np.ndarray, np.ndarray]:
    jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    adr = int(model.jnt_qposadr[jid])
    return data.qpos[adr:adr+3].copy(), normalized_quat(data.qpos[adr+3:adr+7], joint_name)


def set_freejoint_pose(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, position: np.ndarray, quat: np.ndarray) -> None:
    jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    qadr = int(model.jnt_qposadr[jid])
    dadr = int(model.jnt_dofadr[jid])
    data.qpos[qadr:qadr+3] = position
    data.qpos[qadr+3:qadr+7] = quat
    data.qvel[dadr:dadr+6] = 0.0


def apply_object_poses(model: mujoco.MjModel, data: mujoco.MjData, config: dict[str, Any]) -> None:
    bowl_pos0, bowl_q0 = freejoint_pose(model, data, OBJECT_FREEJOINTS["bowl"])
    bean_relative: list[tuple[str, np.ndarray, np.ndarray]] = []
    if bool(config["initial_objects"].get("move_beans_with_bowl", True)):
        for jid in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid) or ""
            if not name.startswith("dynamic_coffee_bean_") or not name.endswith("_freejoint"):
                continue
            pos, quat = freejoint_pose(model, data, name)
            rel_pos = quat_rotate(quat_conjugate(bowl_q0), pos - bowl_pos0)
            rel_quat = quat_mul(quat_conjugate(bowl_q0), quat)
            bean_relative.append((name, rel_pos, rel_quat))

    for name, joint_name in OBJECT_FREEJOINTS.items():
        entry = config["initial_objects"][name]
        pos = vec(entry["position_world_m"], 3, f"{name} position")
        quat = normalized_quat(entry["orientation_wxyz"], f"{name} quaternion")
        set_freejoint_pose(model, data, joint_name, pos, quat)

    if bean_relative:
        bowl = config["initial_objects"]["bowl"]
        new_pos = vec(bowl["position_world_m"], 3, "bowl position")
        new_quat = normalized_quat(bowl["orientation_wxyz"], "bowl quaternion")
        for joint_name, rel_pos, rel_quat in bean_relative:
            set_freejoint_pose(
                model, data, joint_name,
                new_pos + quat_rotate(new_quat, rel_pos),
                quat_mul(new_quat, rel_quat),
            )


def apply_head_visual(model: mujoco.MjModel, config: dict[str, Any]) -> None:
    settings = config["head_visual"]
    anchor_label = str(settings["anchor_label"]).upper()
    marker_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, HEAD_ANCHOR_BODIES[anchor_label])
    head_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, HEAD_VISUAL_BODY)
    if int(model.body_parentid[marker_body]) != 0 or int(model.body_parentid[head_body]) != 0:
        raise ValueError("The configurable head and A-I marker bodies must be direct children of worldbody")

    marker_position = model.body_pos[marker_body].copy()
    offset = vec(settings["offset_world_m"], 3, "head_visual.offset_world_m")
    orientation = normalized_quat(settings["orientation_wxyz"], "head_visual.orientation_wxyz")
    model.body_pos[head_body] = marker_position + offset
    model.body_quat[head_body] = orientation

    alpha = 1.0 if bool(settings["enabled"]) else 0.0
    for geom_name in HEAD_VISUAL_GEOMS:
        geom_id = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        model.geom_rgba[geom_id, 3] = alpha
    log(
        f"[head] {'enabled' if alpha else 'disabled'}, anchor={anchor_label}, "
        f"position={model.body_pos[head_body].tolist()}"
    )


def apply_robot_pose(model: mujoco.MjModel, data: mujoco.MjData, config: dict[str, Any]) -> None:
    base_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
    desired_pos = vec(config["initial_robot"]["base_position_world_m"], 3, "base position")
    desired_q = normalized_quat(config["initial_robot"]["base_orientation_wxyz"], "base orientation")
    roll, pitch, desired_yaw = quat_to_rpy(desired_q)
    if abs(roll) > 1e-5 or abs(pitch) > 1e-5:
        raise ValueError("The mobile base is planar; base roll and pitch must be zero")

    root_pos = model.body_pos[base_id].copy()
    root_q = normalized_quat(model.body_quat[base_id], "model base orientation")
    root_rpy = quat_to_rpy(root_q)
    root_rotation = quat_to_matrix(root_q)
    axes = np.column_stack((root_rotation[:2, 0], root_rotation[:2, 1]))
    qxy = np.linalg.solve(axes, desired_pos[:2] - root_pos[:2])
    model.body_pos[base_id, 2] = desired_pos[2]

    for name, value in zip(BASE_JOINTS[:2], qxy):
        jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[jid])] = value
        data.qvel[int(model.jnt_dofadr[jid])] = 0.0
    yaw_joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, BASE_JOINTS[2])
    data.qpos[int(model.jnt_qposadr[yaw_joint])] = wrap_angle(desired_yaw - root_rpy[2])
    data.qvel[int(model.jnt_dofadr[yaw_joint])] = 0.0

    spine = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, SPINE_JOINT)
    spine_pos = float(config["initial_robot"]["spine_position_m"])
    if bool(model.jnt_limited[spine]):
        spine_pos = float(np.clip(spine_pos, *model.jnt_range[spine]))
    data.qpos[int(model.jnt_qposadr[spine])] = spine_pos
    data.qvel[int(model.jnt_dofadr[spine])] = 0.0

    for side in ("left", "right"):
        values = vec(config["initial_robot"][f"{side}_arm_joint_position_rad"], 7, f"{side} initial arm")
        for name, value in zip(ARM_SPECS[side]["joints"], values):
            jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if bool(model.jnt_limited[jid]):
                value = float(np.clip(value, *model.jnt_range[jid]))
            data.qpos[int(model.jnt_qposadr[jid])] = value
            data.qvel[int(model.jnt_dofadr[jid])] = 0.0


def synchronize_gripper_attachment(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> None:
    spec = ARM_SPECS[side]
    flange = object_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["flange"])
    free_joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, spec["attachment_freejoint"])
    qadr = int(model.jnt_qposadr[free_joint])
    dadr = int(model.jnt_dofadr[free_joint])
    desired_quat = quat_mul(data.xquat[flange], spec["mount_quat"])
    data.qpos[qadr:qadr+3] = data.xpos[flange]
    data.qpos[qadr+3:qadr+7] = desired_quat
    data.qvel[dadr:dadr+6] = 0.0


def viewer_camera_basis(cam) -> np.ndarray:
    if cam is None:
        # Headless smoke tests have no viewer camera; use world axes.
        return np.eye(3)
    az = math.radians(float(cam.azimuth))
    el = math.radians(float(cam.elevation))
    forward = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
    right = np.array([math.sin(az), -math.cos(az), 0.0])
    up = np.cross(right, forward)
    forward /= max(float(np.linalg.norm(forward)), 1e-12)
    right /= max(float(np.linalg.norm(right)), 1e-12)
    up /= max(float(np.linalg.norm(up)), 1e-12)
    return np.column_stack((forward, right, up))


def planar_frame_matrix(frame: str, cam, data: mujoco.MjData, base_body: int) -> np.ndarray:
    if frame == "world":
        return np.eye(3)
    if frame == "base":
        rotation = data.xmat[base_body].reshape(3, 3).copy()
        x = rotation[:, 0]; x[2] = 0.0; x /= max(float(np.linalg.norm(x)), 1e-12)
        y = np.array([-x[1], x[0], 0.0])
        return np.column_stack((x, y, np.array([0.0, 0.0, 1.0])))
    rotation = viewer_camera_basis(cam)
    x = rotation[:, 0]; x[2] = 0.0; x /= max(float(np.linalg.norm(x)), 1e-12)
    # Base command component 1 is "left", while camera column 1 is screen-right.
    y = -rotation[:, 1]; y[2] = 0.0; y /= max(float(np.linalg.norm(y)), 1e-12)
    return np.column_stack((x, y, np.array([0.0, 0.0, 1.0])))


def full_frame_matrix(frame: str, cam, data: mujoco.MjData, base_body: int) -> np.ndarray:
    """Return the frame basis used for angular commands.

    The camera basis keeps its second column as screen-right so the existing
    rotational controls retain their original convention.
    """
    if frame == "world":
        return np.eye(3)
    if frame == "base":
        return data.xmat[base_body].reshape(3, 3).copy()
    return viewer_camera_basis(cam)


def translation_frame_matrix(frame: str, cam, data: mujoco.MjData, base_body: int) -> np.ndarray:
    """Return a translation basis with columns forward, left, and up.

    MuJoCo base/world +Y is treated as left.  The viewer camera helper returns
    forward, screen-right, and up, so its second column is negated here.  This
    gives one consistent keyboard convention in every frame:

      W/Up -> forward, A/Left -> left, D/Right -> right, U/PageUp -> up.
    """
    if frame == "world":
        return np.eye(3)
    if frame == "base":
        return data.xmat[base_body].reshape(3, 3).copy()
    camera_basis = viewer_camera_basis(cam)
    return np.column_stack((camera_basis[:, 0], -camera_basis[:, 1], camera_basis[:, 2]))


class KeyboardInput:
    MOTION_KEYS = {
        glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_LEFT, glfw.KEY_RIGHT,
        glfw.KEY_PAGE_UP, glfw.KEY_PAGE_DOWN, glfw.KEY_HOME, glfw.KEY_END,
        glfw.KEY_W, glfw.KEY_S, glfw.KEY_A, glfw.KEY_D,
        glfw.KEY_Q, glfw.KEY_E, glfw.KEY_U, glfw.KEY_J,
    }
    DISCRETE_KEYS = {
        glfw.KEY_ESCAPE, glfw.KEY_7, glfw.KEY_8, glfw.KEY_9,
        glfw.KEY_KP_7, glfw.KEY_KP_8, glfw.KEY_KP_9,
        glfw.KEY_R, glfw.KEY_G, glfw.KEY_V, glfw.KEY_SPACE,
        glfw.KEY_P, glfw.KEY_L,
    }

    def __init__(self) -> None:
        self._held: set[int] = set()
        self._pending: deque[int] = deque()
        self._lock = threading.Lock()
        self._listener = None
        self._special: dict[object, int] = {}

    def _char(self, char: str | None) -> int | None:
        if not char:
            return None
        return {
            "w": glfw.KEY_W, "s": glfw.KEY_S, "a": glfw.KEY_A, "d": glfw.KEY_D,
            "q": glfw.KEY_Q, "e": glfw.KEY_E, "u": glfw.KEY_U, "j": glfw.KEY_J,
            "7": glfw.KEY_7, "8": glfw.KEY_8, "9": glfw.KEY_9,
            "r": glfw.KEY_R, "g": glfw.KEY_G, "v": glfw.KEY_V,
            "p": glfw.KEY_P, "l": glfw.KEY_L, " ": glfw.KEY_SPACE,
        }.get(char.lower())

    def _map_pynput(self, key) -> int | None:
        return self._special.get(key, self._char(getattr(key, "char", None)))

    def _press(self, key) -> None:
        code = self._map_pynput(key)
        if code is None:
            return
        with self._lock:
            first = code not in self._held
            self._held.add(code)
            if first and code in self.DISCRETE_KEYS:
                self._pending.append(code)

    def _release(self, key) -> None:
        code = self._map_pynput(key)
        if code is None:
            return
        with self._lock:
            self._held.discard(code)

    def start(self) -> None:
        try:
            from pynput import keyboard as pk
            self._special = {
                pk.Key.up: glfw.KEY_UP, pk.Key.down: glfw.KEY_DOWN,
                pk.Key.left: glfw.KEY_LEFT, pk.Key.right: glfw.KEY_RIGHT,
                pk.Key.page_up: glfw.KEY_PAGE_UP, pk.Key.page_down: glfw.KEY_PAGE_DOWN,
                pk.Key.home: glfw.KEY_HOME, pk.Key.end: glfw.KEY_END,
                pk.Key.esc: glfw.KEY_ESCAPE, pk.Key.space: glfw.KEY_SPACE,
            }
            self._listener = pk.Listener(on_press=self._press, on_release=self._release)
            self._listener.daemon = True
            self._listener.start()
            self._listener.wait()
            log("[keyboard] using pynput press/release backend")
        except Exception as exc:
            self._listener = None
            log(f"[keyboard] pynput unavailable ({exc!r}); using GLFW live polling")

    def stop(self) -> None:
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception:
                pass
        with self._lock:
            self._held.clear()

    def callback(self, keycode: int) -> None:
        if self._listener is not None:
            return
        key = abs(int(keycode))
        with self._lock:
            if keycode < 0:
                self._held.discard(key)
                return
            # In the GLFW fallback, motion is read only with glfw.get_key().
            # We never retain a callback motion press because some viewer
            # versions do not deliver the corresponding release callback.
            if key in self.DISCRETE_KEYS:
                self._pending.append(key)

    def drain(self) -> list[int]:
        with self._lock:
            events = list(self._pending)
            self._pending.clear()
        return events

    def down(self, window) -> set[int]:
        with self._lock:
            result = set(self._held)
        if window is not None:
            try:
                if not glfw.get_window_attrib(window, glfw.FOCUSED):
                    return set()
                result |= {key for key in self.MOTION_KEYS if glfw.get_key(window, key) == glfw.PRESS}
            except Exception:
                pass
        return result

    def commands(self, window, mode: str, rotate: bool) -> tuple[np.ndarray, np.ndarray]:
        down = self.down(window)
        held = lambda *keys: any(key in down for key in keys)
        base = np.zeros(4)  # forward, left, spine, yaw
        arm = np.zeros(6)   # x, y, z, roll, pitch, yaw
        if mode == "base":
            base[0] = float(held(glfw.KEY_UP, glfw.KEY_W)) - float(held(glfw.KEY_DOWN, glfw.KEY_S))
            base[1] = float(held(glfw.KEY_LEFT, glfw.KEY_A)) - float(held(glfw.KEY_RIGHT, glfw.KEY_D))
            base[2] = float(held(glfw.KEY_PAGE_UP, glfw.KEY_U)) - float(held(glfw.KEY_PAGE_DOWN, glfw.KEY_J))
            base[3] = float(held(glfw.KEY_HOME, glfw.KEY_Q)) - float(held(glfw.KEY_END, glfw.KEY_E))
        elif rotate:
            arm[3] = float(held(glfw.KEY_PAGE_UP, glfw.KEY_U)) - float(held(glfw.KEY_PAGE_DOWN, glfw.KEY_J))
            arm[4] = float(held(glfw.KEY_UP, glfw.KEY_W)) - float(held(glfw.KEY_DOWN, glfw.KEY_S))
            arm[5] = float(held(glfw.KEY_LEFT, glfw.KEY_A)) - float(held(glfw.KEY_RIGHT, glfw.KEY_D))
        else:
            arm[0] = float(held(glfw.KEY_UP, glfw.KEY_W)) - float(held(glfw.KEY_DOWN, glfw.KEY_S))
            # Positive operator Y means left in every translation frame.
            arm[1] = float(held(glfw.KEY_LEFT, glfw.KEY_A)) - float(held(glfw.KEY_RIGHT, glfw.KEY_D))
            arm[2] = float(held(glfw.KEY_PAGE_UP, glfw.KEY_U)) - float(held(glfw.KEY_PAGE_DOWN, glfw.KEY_J))
        return base, arm


class CameraViewManager:
    """Render named MuJoCo cameras into independent OpenCV windows."""

    CAMERA_OPTIONS = (
        ("head_cam", "Head camera"),
        ("left_wrist_cam", "Left wrist camera"),
        ("right_wrist_cam", "Right wrist camera"),
    )

    def __init__(self, model: mujoco.MjModel, config: dict[str, Any]):
        settings = config["camera_views"]
        self.enabled = bool(settings["enabled"])
        self.renderer: mujoco.Renderer | None = None
        self.cv2: Any | None = None
        self.windows: list[tuple[str, str]] = []
        self.render_period = 1.0 / float(settings["render_hz"])
        self.next_render_time = 0.0

        if not self.enabled:
            return

        self.windows = [
            (camera_name, window_title)
            for camera_name, window_title in self.CAMERA_OPTIONS
            if bool(settings[camera_name])
        ]
        if not self.windows:
            log("[camera views] enabled, but no individual cameras are selected")
            self.enabled = False
            return

        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "Camera windows are enabled, but OpenCV is unavailable. "
                "Install the package requirements or set camera_views.enabled=false."
            ) from exc

        for camera_name, _ in self.windows:
            object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)

        self.cv2 = cv2
        self.renderer = mujoco.Renderer(
            model,
            height=int(settings["height"]),
            width=int(settings["width"]),
        )

        width = int(settings["width"])
        for index, (_, window_title) in enumerate(self.windows):
            cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_title, width, int(settings["height"]))
            cv2.moveWindow(window_title, 20 + index * (width + 30), 20)

        names = ", ".join(camera_name for camera_name, _ in self.windows)
        log(f"[camera views] showing separate RGB windows for: {names}")

    def update(self, data: mujoco.MjData, now: float | None = None) -> None:
        if not self.enabled or self.renderer is None or self.cv2 is None:
            return

        current_time = time.perf_counter() if now is None else float(now)
        if current_time < self.next_render_time:
            return

        for camera_name, window_title in self.windows:
            self.renderer.update_scene(data, camera=camera_name)
            rgb = self.renderer.render()
            bgr = self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR)
            self.cv2.imshow(window_title, bgr)

        # OpenCV requires event processing for the windows to repaint.
        self.cv2.waitKey(1)
        self.next_render_time = current_time + self.render_period

    def close(self) -> None:
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        if self.cv2 is not None:
            for _, window_title in self.windows:
                try:
                    self.cv2.destroyWindow(window_title)
                except Exception:
                    pass
            self.cv2.waitKey(1)


@dataclass
class ArmState:
    side: str
    joint_ids: np.ndarray
    dof_ids: np.ndarray
    qpos_ids: np.ndarray
    actuator_ids: np.ndarray
    tcp_site: int
    target_mocap: int
    gripper_joint: int
    gripper_actuator: int
    pad_left_geoms: set[int]
    pad_right_geoms: set[int]
    own_arm_geoms: set[int]
    previous_qdot: np.ndarray
    position_reference: np.ndarray
    direct_goal: np.ndarray
    close_ramp: bool = False
    gripper_hold_target: float = 0.0
    filtered_left_force: float = 0.0
    filtered_right_force: float = 0.0
    gripper_force_confirmation_count: int = 0
    gripper_force_limited: bool = False
    rotate_mode: bool = False
    z_lock_active: bool = False
    z_lock_target: float = 0.0
    motion_active: bool = False


@dataclass
class ScaleSensorState:
    enabled: bool
    sensor_id: int
    sensor_adr: int
    sensor_dim: int
    filter_alpha: float
    gravity_m_s2: float
    auto_tare: bool
    tare_force_n: float
    tare_complete: bool = False
    raw_force_vector_n: np.ndarray | None = None
    raw_vertical_force_n: float = 0.0
    filtered_weight_force_n: float = 0.0
    weight_kg: float = 0.0


def create_scale_sensor(model: mujoco.MjModel, config: dict[str, Any]) -> ScaleSensorState:
    settings = config["scale_sensor"]
    sensor_id = object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, SCALE_FORCE_SENSOR)
    sensor_adr = int(model.sensor_adr[sensor_id])
    sensor_dim = int(model.sensor_dim[sensor_id])
    if sensor_dim != 3:
        raise RuntimeError(
            f"{SCALE_FORCE_SENSOR} must have dimension 3, got {sensor_dim}"
        )
    auto_tare = bool(settings.get("auto_tare", True))
    state = ScaleSensorState(
        enabled=bool(settings.get("enabled", True)),
        sensor_id=sensor_id,
        sensor_adr=sensor_adr,
        sensor_dim=sensor_dim,
        filter_alpha=float(settings.get("filter_alpha", 0.2)),
        gravity_m_s2=float(settings.get("gravity_m_s2", 9.81)),
        auto_tare=auto_tare,
        tare_force_n=(0.0 if auto_tare else float(settings.get("tare_force_n", 0.0))),
        tare_complete=not auto_tare,
        raw_force_vector_n=np.zeros(3, dtype=np.float64),
    )
    return state


def update_scale_sensor(data: mujoco.MjData, state: ScaleSensorState) -> None:
    """Update the public load-cell values from the MuJoCo force sensor.

    The sensor site is authored with world-aligned axes, so component 2 is the
    world-vertical reaction force. Absolute value is used because MuJoCo's
    child-to-parent sign convention can make the supported load negative.
    """
    global scale_force_vector_n, scale_weight_force_n, scale_weight_kg

    if not state.enabled:
        scale_force_vector_n = np.zeros(3, dtype=np.float64)
        scale_weight_force_n = 0.0
        scale_weight_kg = 0.0
        return

    raw = np.asarray(
        data.sensordata[state.sensor_adr:state.sensor_adr + state.sensor_dim],
        dtype=np.float64,
    ).copy()
    vertical = abs(float(raw[2]))
    state.raw_force_vector_n = raw
    state.raw_vertical_force_n = vertical

    # Auto-tare on the first completed simulation step. This subtracts the
    # scale platform and empty knock-box weight.
    if state.auto_tare and not state.tare_complete:
        state.tare_force_n = vertical
        state.tare_complete = True
        state.filtered_weight_force_n = 0.0
        state.weight_kg = 0.0
    else:
        net = max(0.0, vertical - state.tare_force_n)
        alpha = state.filter_alpha
        state.filtered_weight_force_n = (
            alpha * net + (1.0 - alpha) * state.filtered_weight_force_n
        )
        state.weight_kg = state.filtered_weight_force_n / state.gravity_m_s2

    scale_force_vector_n = raw
    scale_weight_force_n = state.filtered_weight_force_n
    scale_weight_kg = state.weight_kg


def tare_scale_sensor(data: mujoco.MjData, state: ScaleSensorState) -> None:
    """Set the current vertical force as zero load."""
    global scale_force_vector_n, scale_weight_force_n, scale_weight_kg
    if not state.enabled:
        log("[scale] sensor is disabled")
        return
    raw = np.asarray(
        data.sensordata[state.sensor_adr:state.sensor_adr + state.sensor_dim],
        dtype=np.float64,
    ).copy()
    state.raw_force_vector_n = raw
    state.raw_vertical_force_n = abs(float(raw[2]))
    state.tare_force_n = state.raw_vertical_force_n
    state.tare_complete = True
    state.filtered_weight_force_n = 0.0
    state.weight_kg = 0.0
    scale_force_vector_n = raw
    scale_weight_force_n = 0.0
    scale_weight_kg = 0.0
    log(f"[scale] tare set to {state.tare_force_n:.4f} N")


def named_geom_family(model: mujoco.MjModel, prefix: str) -> set[int]:
    return {
        gid for gid in range(model.ngeom)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or "").startswith(prefix)
    }


def arm_geom_ids(model: mujoco.MjModel, prefix: str) -> set[int]:
    result = set()
    for gid in range(model.ngeom):
        body = int(model.geom_bodyid[gid])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
        if name.startswith(prefix):
            result.add(gid)
    return result


def create_arm(model: mujoco.MjModel, data: mujoco.MjData, side: str, goal: np.ndarray) -> ArmState:
    spec = ARM_SPECS[side]
    joints = np.array([object_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in spec["joints"]], dtype=np.int32)
    dofs = np.array([int(model.jnt_dofadr[j]) for j in joints], dtype=np.int32)
    qpos = np.array([int(model.jnt_qposadr[j]) for j in joints], dtype=np.int32)
    actuators = np.array([object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in spec["joints"]], dtype=np.int32)
    target_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["target"])
    mocap = int(model.body_mocapid[target_body])
    return ArmState(
        side=side,
        joint_ids=joints,
        dof_ids=dofs,
        qpos_ids=qpos,
        actuator_ids=actuators,
        tcp_site=object_id(model, mujoco.mjtObj.mjOBJ_SITE, spec["tcp_site"]),
        target_mocap=mocap,
        gripper_joint=object_id(model, mujoco.mjtObj.mjOBJ_JOINT, spec["gripper"]),
        gripper_actuator=object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec["gripper"]),
        pad_left_geoms=named_geom_family(model, spec["pad_left_prefix"]),
        pad_right_geoms=named_geom_family(model, spec["pad_right_prefix"]),
        own_arm_geoms=arm_geom_ids(model, spec["body_prefix"]),
        previous_qdot=np.zeros(7),
        position_reference=data.qpos[qpos].copy(),
        direct_goal=goal.copy(),
        z_lock_target=float(data.site_xpos[object_id(model, mujoco.mjtObj.mjOBJ_SITE, spec["tcp_site"])][2]),
    )


def apply_arm_bias(data: mujoco.MjData, arm: ArmState) -> None:
    data.qfrc_applied[arm.dof_ids] = data.qfrc_bias[arm.dof_ids]


def limit_joint_command(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmState,
    requested: np.ndarray,
    max_vel: np.ndarray,
) -> np.ndarray:
    """Clip arm joint velocity without manual acceleration limiting.

    The requested IK or trajectory velocity is applied immediately after
    per-joint velocity clipping. MuJoCo dynamics, actuator gains, torque limits,
    damping, and contacts determine the resulting physical acceleration.
    """
    command = np.clip(requested, -max_vel, max_vel)
    margin = 0.015
    for i, jid in enumerate(arm.joint_ids):
        if not bool(model.jnt_limited[jid]):
            continue
        lo, hi = model.jnt_range[jid]
        q = float(data.qpos[arm.qpos_ids[i]])
        if (q <= lo + margin and command[i] < 0.0) or (q >= hi - margin and command[i] > 0.0):
            command[i] = 0.0
    arm.previous_qdot = command.copy()
    return command


def apply_cartesian_position_servo(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmState,
    world_twist: np.ndarray,
    damping: float,
    max_vel: np.ndarray,
    dt: float,
    target_lookahead: float,
    max_target_lead: float,
    z_lock_enabled: bool,
    z_lock_gain: float,
    z_lock_max_speed: float,
) -> np.ndarray:
    """Convert a TCP velocity command into a short joint-position target.

    While a motion key is active:
        q_ref = q_measured + clip(qdot_IK * lookahead, +/- max_lead)

    When the key is released, q_ref is deliberately left unchanged.  The
    position actuators therefore complete the remaining motion to the last
    target and hold it instead of replacing it with the release position.
    """
    world_twist = np.asarray(world_twist, dtype=np.float64).copy()
    motion_active = float(np.linalg.norm(world_twist)) > 1.0e-9
    qdot = np.zeros(7, dtype=np.float64)

    if motion_active:
        horizontal = (
            float(np.linalg.norm(world_twist[:2])) > 1.0e-9
            and abs(float(world_twist[2])) <= 1.0e-9
        )
        if z_lock_enabled and horizontal:
            if not arm.z_lock_active:
                arm.z_lock_target = float(data.site_xpos[arm.tcp_site][2])
                arm.z_lock_active = True
            correction = float(z_lock_gain) * (
                arm.z_lock_target - float(data.site_xpos[arm.tcp_site][2])
            )
            world_twist[2] = float(
                np.clip(correction, -z_lock_max_speed, z_lock_max_speed)
            )
        else:
            arm.z_lock_active = False
            arm.z_lock_target = float(data.site_xpos[arm.tcp_site][2])

        jacp = np.zeros((3, model.nv), dtype=np.float64)
        jacr = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacSite(model, data, jacp, jacr, arm.tcp_site)
        jac = np.vstack((jacp[:, arm.dof_ids], jacr[:, arm.dof_ids]))
        regularized = jac @ jac.T + (float(damping) ** 2) * np.eye(6)
        try:
            qdot_raw = jac.T @ np.linalg.solve(regularized, world_twist)
        except np.linalg.LinAlgError:
            qdot_raw = np.zeros(7, dtype=np.float64)

        qdot = limit_joint_command(
            model, data, arm, qdot_raw, max_vel
        )
        measured_q = data.qpos[arm.qpos_ids].copy()
        horizon = max(float(dt), float(target_lookahead))
        target_delta = np.clip(
            qdot * horizon,
            -float(max_target_lead),
            float(max_target_lead),
        )
        arm.position_reference = measured_q + target_delta
        for i, jid in enumerate(arm.joint_ids):
            if bool(model.jnt_limited[jid]):
                arm.position_reference[i] = float(
                    np.clip(arm.position_reference[i], *model.jnt_range[jid])
                )
        arm.motion_active = True
    else:
        # Critical release rule: do not synchronize q_ref to q_measured.
        # The previous target remains active until the arm reaches it.
        if arm.motion_active:
            arm.previous_qdot.fill(0.0)
        arm.motion_active = False
        arm.z_lock_active = False
        arm.z_lock_target = float(data.site_xpos[arm.tcp_site][2])

    apply_arm_bias(data, arm)
    for aid, target in zip(arm.actuator_ids, arm.position_reference):
        set_ctrl_clipped(model, data, int(aid), float(target))

    marker_horizon = max(float(dt), float(target_lookahead))
    data.mocap_pos[arm.target_mocap] = (
        data.site_xpos[arm.tcp_site] + world_twist[:3] * marker_horizon
    )
    data.mocap_quat[arm.target_mocap] = mat_to_quat(
        data.site_xmat[arm.tcp_site].reshape(3, 3)
    )
    return qdot


def apply_direct_position(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmState, max_vel: np.ndarray, gain: float, dt: float) -> np.ndarray:
    error = arm.direct_goal - arm.position_reference
    requested = np.clip(float(gain) * error, -max_vel, max_vel)
    qdot = limit_joint_command(model, data, arm, requested, max_vel)
    step = qdot * dt
    overshoot = np.abs(step) > np.abs(error)
    step[overshoot] = error[overshoot]
    arm.position_reference += step
    for i, jid in enumerate(arm.joint_ids):
        if bool(model.jnt_limited[jid]):
            arm.position_reference[i] = float(np.clip(arm.position_reference[i], *model.jnt_range[jid]))
    apply_arm_bias(data, arm)
    for aid, value in zip(arm.actuator_ids, arm.position_reference):
        set_ctrl_clipped(model, data, int(aid), float(value))
    data.mocap_pos[arm.target_mocap] = data.site_xpos[arm.tcp_site]
    data.mocap_quat[arm.target_mocap] = mat_to_quat(data.site_xmat[arm.tcp_site].reshape(3, 3))
    return qdot


def external_pad_normal_force(model: mujoco.MjModel, data: mujoco.MjData, pad_geoms: set[int], own_geoms: set[int]) -> float:
    total = 0.0
    force = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        g1, g2 = int(contact.geom1), int(contact.geom2)
        if not ((g1 in pad_geoms and g2 not in own_geoms) or (g2 in pad_geoms and g1 not in own_geoms)):
            continue
        force.fill(0.0)
        mujoco.mj_contactForce(model, data, index, force)
        total += abs(float(force[0]))
    return total


def start_gripper_closing(arm: ArmState) -> None:
    if arm.gripper_hold_target > 0.0 and not arm.close_ramp:
        mode = "force-limited" if arm.gripper_force_limited else "position-limited"
        log(
            f"[{arm.side} gripper] already holding "
            f"{arm.gripper_hold_target:.3f} rad ({mode})"
        )
        return
    arm.close_ramp = True
    arm.gripper_hold_target = 0.0
    arm.gripper_force_confirmation_count = 0
    arm.gripper_force_limited = False


def open_gripper(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmState) -> None:
    arm.close_ramp = False
    arm.gripper_hold_target = 0.0
    arm.filtered_left_force = 0.0
    arm.filtered_right_force = 0.0
    arm.gripper_force_confirmation_count = 0
    arm.gripper_force_limited = False
    set_ctrl_clipped(model, data, arm.gripper_actuator, 0.0)


def update_gripper(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmState,
    dt: float,
    close_rate: float,
    close_target: float,
    contact_force_threshold: float,
    contact_hold_increment: float,
    confirmation_steps: int,
    filter_alpha: float,
) -> None:
    """Close until contact force is sustained, then apply a small preload.

    During closing, the controller filters the normal force on both finger-pad
    families.  Once the sum of the filtered pad forces stays above the
    threshold for ``confirmation_steps`` simulation cycles, the closing ramp
    stops and the new position target is set to:

        measured gripper joint position + contact_hold_increment

    The increment is in the positive/closing direction and is clipped by the
    configured fixed close target, the actuator control range, and the joint
    range.  The resulting target is continuously held until the gripper is
    opened.
    """
    alpha = float(np.clip(filter_alpha, 1.0e-6, 1.0))
    raw_left = external_pad_normal_force(
        model, data, arm.pad_left_geoms, arm.own_arm_geoms
    )
    raw_right = external_pad_normal_force(
        model, data, arm.pad_right_geoms, arm.own_arm_geoms
    )
    arm.filtered_left_force = (
        alpha * raw_left + (1.0 - alpha) * arm.filtered_left_force
    )
    arm.filtered_right_force = (
        alpha * raw_right + (1.0 - alpha) * arm.filtered_right_force
    )
    filtered_total_force = (
        arm.filtered_left_force + arm.filtered_right_force
    )

    _, actuator_high = model.actuator_ctrlrange[arm.gripper_actuator]
    _, joint_high = model.jnt_range[arm.gripper_joint]
    fixed = min(
        0.735,
        float(close_target),
        float(actuator_high),
        float(joint_high),
    )

    if not arm.close_ramp:
        if arm.gripper_hold_target > 0.0:
            set_ctrl_clipped(
                model, data, arm.gripper_actuator, arm.gripper_hold_target
            )
        return

    threshold_reached = (
        filtered_total_force >= max(0.0, float(contact_force_threshold))
    )
    if threshold_reached:
        arm.gripper_force_confirmation_count += 1
    else:
        arm.gripper_force_confirmation_count = 0

    if arm.gripper_force_confirmation_count >= max(1, int(confirmation_steps)):
        gripper_qpos = int(model.jnt_qposadr[arm.gripper_joint])
        measured_position = float(data.qpos[gripper_qpos])
        hold_target = min(
            fixed,
            measured_position + max(0.0, float(contact_hold_increment)),
        )
        set_ctrl_clipped(model, data, arm.gripper_actuator, hold_target)
        arm.gripper_hold_target = hold_target
        arm.close_ramp = False
        arm.gripper_force_limited = True
        log(
            f"[{arm.side} gripper] force threshold reached: "
            f"total={filtered_total_force:.2f} N "
            f"(L/R={arm.filtered_left_force:.2f}/"
            f"{arm.filtered_right_force:.2f} N); "
            f"measured={measured_position:.4f} rad, "
            f"hold={hold_target:.4f} rad"
        )
        return

    current = float(data.ctrl[arm.gripper_actuator])
    target = min(
        fixed,
        current + max(0.0, close_rate) * max(0.0, dt),
    )
    set_ctrl_clipped(model, data, arm.gripper_actuator, target)
    if target >= fixed - 1.0e-12:
        arm.gripper_hold_target = fixed
        arm.close_ramp = False
        arm.gripper_force_limited = False
        log(
            f"[{arm.side} gripper] maximum close target reached before "
            f"the force threshold: {fixed:.3f} rad; "
            f"forces L/R={arm.filtered_left_force:.2f}/"
            f"{arm.filtered_right_force:.2f} N"
        )


def configure_cup_contact(model: mujoco.MjModel, config: dict[str, Any]) -> None:
    """Apply high-friction contact parameters to the convex cup collision mesh."""
    cup_geom = object_id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "collision_dynamic_cup_convex_hull",
    )
    friction = vec(config["contact"]["cup_friction"], 3, "contact.cup_friction")
    model.geom_friction[cup_geom] = friction
    model.geom_priority[cup_geom] = int(config["contact"]["cup_contact_priority"])
    model.geom_condim[cup_geom] = 6


def configure_actuators(model: mujoco.MjModel, arms: dict[str, ArmState], config: dict[str, Any]) -> None:
    torque = positive_vec(config["limits"]["arm_joint_torque_nm"], 7, "arm torque")
    position_kp = positive_vec(config["controller"]["position_actuator_kp"], 7, "position kp")
    position_kv = positive_vec(config["controller"]["position_actuator_kv"], 7, "position kv")
    for arm in arms.values():
        for i, aid in enumerate(arm.actuator_ids):
            model.actuator_forcelimited[aid] = 1
            model.actuator_forcerange[aid] = (-torque[i], torque[i])
            if hasattr(model, "jnt_actfrcrange"):
                model.jnt_actfrcrange[arm.joint_ids[i]] = (-torque[i], torque[i])
            kp, kv = position_kp[i], position_kv[i]
            model.actuator_gainprm[aid, :] = 0.0
            model.actuator_biasprm[aid, :] = 0.0
            model.actuator_gainprm[aid, 0] = kp
            model.actuator_biasprm[aid, 1] = -kp
            model.actuator_biasprm[aid, 2] = -kv

    base_force = float(config["limits"]["base_force_n"])
    yaw_torque = float(config["limits"]["base_yaw_torque_nm"])
    for name, force in zip(BASE_ACTUATORS, (base_force, base_force, yaw_torque)):
        aid = object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        model.actuator_forcelimited[aid] = 1
        model.actuator_forcerange[aid] = (-force, force)
    spine_aid = object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, SPINE_ACTUATOR)
    spine_jid = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, SPINE_JOINT)
    spine_force = float(config["limits"]["spine_force_n"])
    if not math.isfinite(spine_force) or spine_force <= 0.0:
        raise ValueError("limits.spine_force_n must be finite and positive")

    # Configure the spine position servo for the fixed 3*dt target lead.
    # With timestep 0.001 s, kp*T/kv = 1 for the default gains, so the
    # position servo behaves approximately like a well-damped velocity servo.
    spine_kp = float(config["controller"]["spine_position_kp"])
    spine_kv = float(config["controller"]["spine_position_kv"])
    model.actuator_gainprm[spine_aid, :] = 0.0
    model.actuator_biasprm[spine_aid, :] = 0.0
    model.actuator_gainprm[spine_aid, 0] = spine_kp
    model.actuator_biasprm[spine_aid, 1] = -spine_kp
    model.actuator_biasprm[spine_aid, 2] = -spine_kv

    # Keep actuator-level and joint-level force clamps identical.  The joint
    # clamp limits the total actuator force transmitted to this scalar joint.
    model.actuator_forcelimited[spine_aid] = 1
    model.actuator_forcerange[spine_aid] = (-spine_force, spine_force)
    if hasattr(model, "jnt_actfrclimited"):
        model.jnt_actfrclimited[spine_jid] = 1
    if hasattr(model, "jnt_actfrcrange"):
        model.jnt_actfrcrange[spine_jid] = (-spine_force, spine_force)


@dataclass
class BaseState:
    body: int
    joint_ids: np.ndarray
    qpos_ids: np.ndarray
    dof_ids: np.ndarray
    actuator_ids: np.ndarray
    spine_velocity_command: float
    spine_joint: int
    spine_qpos: int
    spine_dof: int
    spine_actuator: int
    spine_target: float


def create_base(model: mujoco.MjModel, data: mujoco.MjData) -> BaseState:
    joints = np.array([object_id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in BASE_JOINTS], dtype=np.int32)
    spine = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, SPINE_JOINT)
    return BaseState(
        body=object_id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY),
        joint_ids=joints,
        qpos_ids=np.array([int(model.jnt_qposadr[j]) for j in joints]),
        dof_ids=np.array([int(model.jnt_dofadr[j]) for j in joints]),
        actuator_ids=np.array([object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in BASE_ACTUATORS]),
        spine_velocity_command=0.0,
        spine_joint=spine,
        spine_qpos=int(model.jnt_qposadr[spine]),
        spine_dof=int(model.jnt_dofadr[spine]),
        spine_actuator=object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, SPINE_ACTUATOR),
        spine_target=float(data.qpos[int(model.jnt_qposadr[spine])]),
    )


def apply_base_command(model: mujoco.MjModel, data: mujoco.MjData, state: BaseState, axes: np.ndarray, cam, config: dict[str, Any], dt: float) -> np.ndarray:
    """Apply direct base velocity commands and an acceleration-limited spine command.

    The base has no manual acceleration ramp: the requested translational and
    yaw velocities are written directly to the MuJoCo velocity actuators.

    The spine keeps a dedicated commanded velocity. It accelerates using
    spine_acceleration_m_s2 and decelerates using the faster
    spine_deceleration_m_s2. The ramped velocity generates a
    measured-position-relative target using a fixed 3*dt lookahead. After key
    release, the velocity reaches zero quickly; the final target is then
    retained and held.
    """
    frame = config["motion_frames"]["base"]
    linear_speed = float(config["velocities"]["base_linear_m_s"])
    angular_speed = float(config["velocities"]["base_angular_rad_s"])
    frame_rotation = planar_frame_matrix(frame, cam, data, state.body)
    desired_world = frame_rotation @ np.array([axes[0] * linear_speed, axes[1] * linear_speed, 0.0])

    current_rotation = planar_frame_matrix("base", cam, data, state.body)
    local_x = float(desired_world @ current_rotation[:, 0])
    local_y = float(desired_world @ current_rotation[:, 1])
    desired = np.array([local_x, local_y, axes[3] * angular_speed])

    # No Python-side base acceleration limiting.  Physical acceleration and
    # braking are determined by the velocity actuators, force limits, inertia,
    # contacts, and the MuJoCo solver.
    for aid, value in zip(state.actuator_ids, desired):
        set_ctrl_clipped(model, data, int(aid), float(value))

    desired_spine_velocity = (
        float(axes[2]) * float(config["velocities"]["spine_m_s"])
    )

    # Use a gentle acceleration when starting or increasing speed, but a
    # faster deceleration when releasing the key, slowing down, or reversing.
    # This preserves smooth spine motion without allowing the target to keep
    # advancing for a long time after release.
    current_spine_velocity = float(state.spine_velocity_command)
    same_direction = (
        abs(current_spine_velocity) <= 1.0e-12
        or abs(desired_spine_velocity) <= 1.0e-12
        or np.sign(current_spine_velocity) == np.sign(desired_spine_velocity)
    )
    speeding_up = (
        abs(current_spine_velocity) <= 1.0e-12
        or (same_direction and abs(desired_spine_velocity) > abs(current_spine_velocity))
    )
    rate_limit = float(
        config["limits"][
            "spine_acceleration_m_s2"
            if speeding_up
            else "spine_deceleration_m_s2"
        ]
    )
    max_spine_velocity_change = rate_limit * float(dt)
    state.spine_velocity_command += float(np.clip(
        desired_spine_velocity - current_spine_velocity,
        -max_spine_velocity_change,
        max_spine_velocity_change,
    ))

    if abs(state.spine_velocity_command) > 1.0e-9:
        measured_position = float(data.qpos[state.spine_qpos])
        lookahead = 3.0 * float(dt)
        state.spine_target = (
            measured_position
            + state.spine_velocity_command * lookahead
        )
    else:
        state.spine_velocity_command = 0.0

    if bool(model.jnt_limited[state.spine_joint]):
        state.spine_target = float(np.clip(
            state.spine_target,
            *model.jnt_range[state.spine_joint],
        ))

    data.qfrc_applied[state.spine_dof] = data.qfrc_bias[state.spine_dof]
    set_ctrl_clipped(model, data, state.spine_actuator, state.spine_target)
    return desired.copy()


def zero_base(model: mujoco.MjModel, data: mujoco.MjData, state: BaseState, config: dict[str, Any], dt: float) -> None:
    apply_base_command(model, data, state, np.zeros(4), None, config, dt)


def arm_world_twist(operator: np.ndarray, cam, data: mujoco.MjData, base_body: int, config: dict[str, Any]) -> np.ndarray:
    frame = config["motion_frames"]["end_effector"]
    linear_rotation = translation_frame_matrix(frame, cam, data, base_body)
    angular_rotation = full_frame_matrix(frame, cam, data, base_body)
    linear = linear_rotation @ operator[:3] * float(config["velocities"]["end_effector_linear_m_s"])
    angular = angular_rotation @ operator[3:] * float(config["velocities"]["end_effector_angular_rad_s"])
    return np.concatenate((linear, angular))


def reload_direct_goals(config_path: Path, arms: dict[str, ArmState]) -> None:
    latest = load_config(config_path)
    for side, arm in arms.items():
        arm.direct_goal = vec(latest["joint_position_targets"][f"{side}_rad"], 7, f"{side} direct target")
    log("[config] reloaded direct joint-position targets")


def print_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base: BaseState,
    arms: dict[str, ArmState],
    mode: str,
    scale_sensor: ScaleSensorState,
) -> None:
    log(f"[state] mode={mode} sim_time={data.time:.3f}")
    log(f"  base q={np.round(data.qpos[base.qpos_ids], 5)} qvel={np.round(data.qvel[base.dof_ids], 5)} ctrl={np.round(data.ctrl[base.actuator_ids], 5)}")
    log(f"  spine q={data.qpos[base.spine_qpos]:.6f} target={base.spine_target:.6f} v={data.qvel[base.spine_dof]:.6f} v_cmd={base.spine_velocity_command:.6f}")
    log(
        f"  scale raw={np.round(scale_sensor.raw_force_vector_n, 4)} N "
        f"vertical={scale_sensor.raw_vertical_force_n:.4f} N "
        f"tare={scale_sensor.tare_force_n:.4f} N "
        f"weight={scale_weight_force_n:.4f} N "
        f"mass={scale_weight_kg:.5f} kg "
        f"enabled={scale_sensor.enabled} tared={scale_sensor.tare_complete}"
    )
    for side, arm in arms.items():
        q = data.qpos[arm.qpos_ids]
        log(
            f"  {side} q={np.round(q, 5)} "
            f"q_ref={np.round(arm.position_reference, 5)} "
            f"qdot_ik={np.round(arm.previous_qdot, 5)} "
            f"active={arm.motion_active} "
            f"direct_goal={np.round(arm.direct_goal, 5)}"
        )
        log(f"  {side} tcp_world={np.round(data.site_xpos[arm.tcp_site], 5)} actuator_ctrl={np.round(data.ctrl[arm.actuator_ids], 5)}")
        gripper_qpos = int(model.jnt_qposadr[arm.gripper_joint])
        log(
            f"  {side} gripper q={data.qpos[gripper_qpos]:.5f} "
            f"ctrl={data.ctrl[arm.gripper_actuator]:.5f} "
            f"hold={arm.gripper_hold_target:.5f} "
            f"forces L/R/total={arm.filtered_left_force:.2f}/"
            f"{arm.filtered_right_force:.2f}/"
            f"{arm.filtered_left_force + arm.filtered_right_force:.2f} N "
            f"closing={arm.close_ramp} force_limited={arm.gripper_force_limited}"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=HERE / "config.json")
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--steps", type=int, default=20)
    return parser


# EBiM integration addition (not upstream): the visual meshes and textures
# above the repository's 2 MB file limit are fetched separately, so a fresh
# clone can be missing them. MuJoCo would otherwise fail deep in model
# compilation with a bare "Error opening file '...obj'". Fail fast with the fix.
def check_large_assets() -> None:
    manifest = HERE / "scripts" / "large_assets.txt"
    if not manifest.is_file():
        return
    missing = [
        rel for rel in manifest.read_text().split() if not (HERE / rel).is_file()
    ]
    if not missing:
        return
    listing = "\n".join(f"  {rel}" for rel in missing)
    raise SystemExit(
        f"Missing {len(missing)} large Task 3 asset(s) not tracked in git:\n"
        f"{listing}\n\n"
        f"Fetch them with:\n  {HERE / 'scripts' / 'download_large_assets.sh'}\n"
        "See task3_mujoco/README.md for details."
    )


def main() -> None:
    args = build_parser().parse_args()
    check_large_assets()
    config_path = args.config.resolve()
    config = load_config(config_path)
    mode = config["scene"]["arm_control_mode"]
    beans = int(config["scene"]["bean_count"])
    # Both arm modes use the same position-actuator robot model.
    scene = HERE / f"scene_{beans}.xml"
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.opt.timestep = float(config["scene"]["timestep_s"])
    data = mujoco.MjData(model)

    apply_robot_pose(model, data, config)
    apply_object_poses(model, data, config)
    apply_head_visual(model, config)
    mujoco.mj_forward(model, data)
    for side in ("left", "right"):
        synchronize_gripper_attachment(model, data, side)
    mujoco.mj_forward(model, data)

    arms = {
        side: create_arm(model, data, side, vec(config["joint_position_targets"][f"{side}_rad"], 7, f"{side} target"))
        for side in ("left", "right")
    }
    base = create_base(model, data)
    configure_actuators(model, arms, config)
    configure_cup_contact(model, config)
    for arm in arms.values():
        data.mocap_pos[arm.target_mocap] = data.site_xpos[arm.tcp_site]
        data.mocap_quat[arm.target_mocap] = mat_to_quat(data.site_xmat[arm.tcp_site].reshape(3, 3))
    mujoco.mj_forward(model, data)

    scale_sensor = create_scale_sensor(model, config)

    max_vel = positive_vec(config["limits"]["arm_joint_velocity_rad_s"], 7, "joint velocity")
    dt = float(model.opt.timestep)

    if args.no_viewer:
        for _ in range(max(0, args.steps)):
            data.qfrc_applied[:] = 0.0
            apply_base_command(model, data, base, np.zeros(4), None, config, dt)
            for arm in arms.values():
                if mode == "cartesian_velocity_position_servo":
                    apply_cartesian_position_servo(
                        model, data, arm, np.zeros(6),
                        float(config["controller"]["ik_damping"]),
                        max_vel, dt,
                        float(config["controller"]["arm_target_lookahead_s"]),
                        float(config["controller"]["arm_max_target_lead_rad"]),
                        bool(config["controller"]["horizontal_world_z_lock_enabled"]),
                        float(config["controller"]["horizontal_world_z_lock_gain"]),
                        float(config["controller"]["horizontal_world_z_lock_max_speed_m_s"]),
                    )
                else:
                    apply_direct_position(model, data, arm, max_vel, float(config["controller"]["position_tracking_gain"]), dt)
                update_gripper(
                    model, data, arm, dt,
                    float(config["controller"]["gripper_close_rate_rad_s"]),
                    float(config["controller"]["gripper_fixed_close_target_rad"]),
                    float(config["controller"]["gripper_contact_force_threshold_n"]),
                    float(config["controller"]["gripper_contact_hold_increment_rad"]),
                    int(config["controller"]["gripper_contact_confirmation_steps"]),
                    float(config["controller"]["gripper_force_filter_alpha"]),
                )
            mujoco.mj_step(model, data)
            update_scale_sensor(data, scale_sensor)
        print_state(model, data, base, arms, mode, scale_sensor)
        return

    keyboard = KeyboardInput()
    keyboard.start()
    selected = "base"
    last_arm = "right"
    should_exit = False
    log(HELP)
    log(f"[config] scene={scene.name}, mode={mode}, base_frame={config['motion_frames']['base']}, end_effector_frame={config['motion_frames']['end_effector']}")
    log(
        f"[scale] sensor={SCALE_FORCE_SENSOR}, enabled={scale_sensor.enabled}, "
        f"auto_tare={scale_sensor.auto_tare}; press P to print the reading"
    )

    camera_views = CameraViewManager(model, config)
    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=keyboard.callback) as viewer:
            window = getattr(viewer, "_window", None)
            render_period = 1.0 / max(float(config["scene"]["render_hz"]), 1e-6)
            next_render = 0.0
            while viewer.is_running() and not should_exit:
                window = getattr(viewer, "_window", window)
                for event in keyboard.drain():
                    if event == glfw.KEY_ESCAPE:
                        should_exit = True
                    elif event in (glfw.KEY_7, glfw.KEY_KP_7):
                        selected = "base"; log("[mode] base/spine")
                    elif event in (glfw.KEY_8, glfw.KEY_KP_8):
                        selected = "left"; last_arm = "left"; log("[mode] left arm")
                    elif event in (glfw.KEY_9, glfw.KEY_KP_9):
                        selected = "right"; last_arm = "right"; log("[mode] right arm")
                    elif event == glfw.KEY_R and selected in arms and mode == "cartesian_velocity_position_servo":
                        arms[selected].rotate_mode = not arms[selected].rotate_mode
                        log(f"[{selected}] {'rotation' if arms[selected].rotate_mode else 'translation'} mode")
                    elif event == glfw.KEY_G:
                        start_gripper_closing(arms[selected if selected in arms else last_arm])
                    elif event in (glfw.KEY_V, glfw.KEY_SPACE):
                        open_gripper(model, data, arms[selected if selected in arms else last_arm])
                    elif event == glfw.KEY_P:
                        print_state(model, data, base, arms, mode, scale_sensor)
                    elif event == glfw.KEY_T:
                        tare_scale_sensor(data, scale_sensor)
                    elif event == glfw.KEY_L:
                        reload_direct_goals(config_path, arms)

                base_axes, operator = keyboard.commands(
                    window,
                    selected,
                    arms[selected].rotate_mode if selected in arms else False,
                )
                data.qfrc_applied[:] = 0.0
                apply_base_command(model, data, base, base_axes if selected == "base" else np.zeros(4), viewer.cam, config, dt)

                for side, arm in arms.items():
                    if mode == "cartesian_velocity_position_servo":
                        twist = (
                            arm_world_twist(operator, viewer.cam, data, base.body, config)
                            if selected == side else np.zeros(6)
                        )
                        apply_cartesian_position_servo(
                            model, data, arm, twist,
                            float(config["controller"]["ik_damping"]),
                            max_vel, dt,
                            float(config["controller"]["arm_target_lookahead_s"]),
                            float(config["controller"]["arm_max_target_lead_rad"]),
                            bool(config["controller"]["horizontal_world_z_lock_enabled"]),
                            float(config["controller"]["horizontal_world_z_lock_gain"]),
                            float(config["controller"]["horizontal_world_z_lock_max_speed_m_s"]),
                        )
                    else:
                        apply_direct_position(model, data, arm, max_vel, float(config["controller"]["position_tracking_gain"]), dt)
                    update_gripper(
                        model, data, arm, dt,
                        float(config["controller"]["gripper_close_rate_rad_s"]),
                        float(config["controller"]["gripper_fixed_close_target_rad"]),
                        float(config["controller"]["gripper_contact_force_threshold_n"]),
                        float(config["controller"]["gripper_contact_hold_increment_rad"]),
                        int(config["controller"]["gripper_contact_confirmation_steps"]),
                        float(config["controller"]["gripper_force_filter_alpha"]),
                    )

                mujoco.mj_step(model, data)
                update_scale_sensor(data, scale_sensor)
                now = time.perf_counter()
                camera_views.update(data, now)
                if now >= next_render:
                    viewer.sync()
                    next_render = now + render_period
    finally:
        camera_views.close()
        keyboard.stop()


if __name__ == "__main__":
    main()
