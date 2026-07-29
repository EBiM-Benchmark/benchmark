#!/usr/bin/env python3
"""Keyboard teleoperation for the aligned Mobile FR3 Duo MuJoCo scene.

The control method follows the keyboard teleoperation implementation from
``model.zip`` while removing task-specific cable/C-clip logic:

* 7 / 8 / 9 select the mobile base, left arm, or right arm.
* Base arrows are relative to the current MuJoCo viewer camera.
* Arm arrows create a Cartesian twist in the current TCP local frame.
* Damped least-squares Jacobian IK maps the TCP twist to the seven FR3
  velocity actuators.
* Inactive arms are pinned at a synchronized joint-position anchor because
  velocity actuators cannot hold an arm against gravity by themselves.
* The FR3 arms, spine and Robotiq grippers use position actuators; arm and spine motion remain fully dynamic.
* The right gripper retains its standalone free-body dynamics and is rigidly
  attached to the right flange through a stiff MuJoCo weld. The arm Jacobian
  is evaluated at a flange-side control TCP with the same mounting transform.

Run from this directory:

    python teleop_keyboard.py

Use another scene file if needed:

    python teleop_keyboard.py --xml final_scene_spoon_mocap_runtime.xml

A display-free model/control smoke test is available with:

    python teleop_keyboard.py --no-viewer --steps 20
"""

from __future__ import annotations

import argparse
import ctypes
import math
import os
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

# The original teleoperation package forces GLFW onto XWayland so Linux can
# query held key state through X11. This must happen before importing GLFW.
if sys.platform.startswith("linux") and os.environ.get("XDG_SESSION_TYPE") == "wayland":
    os.environ.setdefault("GLFW_PLATFORM", "x11")

import glfw
import mujoco
import mujoco.viewer
import numpy as np


# ---------------------------------------------------------------------------
# Model names: these match the aligned robot.xml/final_scene.xml files.
# ---------------------------------------------------------------------------

ARM_SPECS = {
    "left": {
        "joints": [f"left_fr3v2_1_joint{i}" for i in range(1, 8)],
        "tcp": "left_arm_control_tcp",
        "tcp_site": "left_arm_control_tcp_pivot",
        "target": "left_tcp_mocap_target",
        "gripper": "left_fr3v2_1_robotiq_85_left_knuckle_joint",
        "gripper_base": "left_fr3v2_1_robotiq_85_base_link",
        "flange": "left_fr3v2_1_link8",
        "attachment_freejoint": "left_gripper_attachment_freejoint",
        "mount_quat": np.array([0.92388, 0.0, 0.0, -0.382683], dtype=np.float64),
        "pad_left_main": "left_grasp_left_main",
        "pad_right_main": "left_grasp_right_main",
        "pad_left_prefix": "left_grasp_left",
        "pad_right_prefix": "left_grasp_right",
        "body_prefix": "left_fr3v2_1",
    },
    "right": {
        "joints": [f"right_fr3v2_1_joint{i}" for i in range(1, 8)],
        "tcp": "right_arm_control_tcp",
        "tcp_site": "right_arm_control_tcp_pivot",
        "target": "right_tcp_mocap_target",
        "gripper": "right_fr3v2_1_robotiq_85_left_knuckle_joint",
        "gripper_base": "right_fr3v2_1_robotiq_85_base_link",
        "flange": "right_fr3v2_1_link8",
        "attachment_freejoint": "right_gripper_attachment_freejoint",
        "mount_quat": np.array([0.92388, 0.0, 0.0, -0.382683], dtype=np.float64),
        "pad_left_main": "right_grasp_left_main",
        "pad_right_main": "right_grasp_right_main",
        "pad_left_prefix": "right_grasp_left",
        "pad_right_prefix": "right_grasp_right",
        "body_prefix": "right_fr3v2_1",
    },
}

BASE_BODY = "base_link"
BASE_X_JOINT = "base_planar_x"
BASE_Y_JOINT = "base_planar_y"
BASE_YAW_JOINT = "base_yaw"
BASE_X_ACT = "base_planar_x_velocity"
BASE_Y_ACT = "base_planar_y_velocity"
BASE_YAW_ACT = "base_yaw_velocity"
SPINE_JOINT = "franka_spine_vertical_joint"
SPINE_ACT = "franka_spine_vertical_joint"

# Default initial configuration. The mobile base, spine, and right arm are
# positioned so the open right gripper starts directly above the spoon stem.
# The gripper's longitudinal +Z axis points along world -Z (the two axes are
# exactly parallel), and the lowest active pad surface has about 24.7 mm of
# clearance above the spoon-stem collision cuboid.
BASE_READY_QPOS = {
    BASE_X_JOINT: 0.2765271262,
    BASE_Y_JOINT: 0.0310855164,
    BASE_YAW_JOINT: 0.0,
}
SPINE_READY_QPOS = 0.27101102

ARM_READY_QPOS = {
    "left": np.array([1.208, -0.152, 0.591, -2.447, 1.099, 3.07, -0.573]),
    "right": np.array([
        -0.36951305500840786,
         0.42595500821455445,
        -0.17617752150292285,
        -1.3551565608736291,
        -0.8537811837683685,
         1.9529957509105405,
         1.0695779371342626,
    ]),
}

HELP = r"""
Mobile FR3 Duo keyboard teleoperation

Initial pose
  Right gripper starts with the proven small-model pose relative to the original coffee-scene spoon.

Mode selection
  7 : mobile base
  8 : left arm
  9 : right arm

Base mode
  Arrow Up / Down or W / S    : move into / out of the viewer screen
  Arrow Left / Right or A / D : move toward viewer screen-left / screen-right
  Home / End or Q / E         : rotate base left / right
  Page Up / Down or U / J      : dynamically move the vertical spine up / down

Arm mode (translation frame follows --arm-frame, default "base";
          rotation is always about the current TCP-local axes)
  Translation mode (default --arm-frame base; camera orientation is ignored):
    Arrow Up / Down or W / S    : move forward / back in the robot-base frame
    Arrow Left / Right or A / D : move right / left in the robot-base frame
    Page Up / Down or U / J     : move up / down in world Z
    With --arm-frame tcp these become the tool-local +X/-X, -Y/+Y, +Z/-Z axes.
    Note that at the initial spoon pose the TCP local +Z points downward, so
    under "tcp" Page Up / U moves the gripper DOWN. Prefer the default "base".
  R toggles rotation mode (always TCP-local, regardless of --arm-frame):
    Arrow Up / Down or W / S    : pitch about local +Y / -Y
    Arrow Left / Right or A / D : yaw about local +Z / -Z
    Page Up / Down or U / J     : roll about local +X / -X

Other commands
  G          : close to the fixed 0.735 rad preload
  V or Space : open the selected gripper
  C          : synchronize the selected arm hold/target to its current pose
  - / =      : decrease / increase all motion speeds
  B          : print current contacts
  N          : show/hide collision geometry group 3
  Esc        : exit

In base mode, G/V/C act on the most recently selected arm (right initially).
"""


# ---------------------------------------------------------------------------
# General helpers
# ---------------------------------------------------------------------------


def log(message: str) -> None:
    print(message, flush=True)


def object_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int:
    idx = int(mujoco.mj_name2id(model, obj_type, name))
    if idx < 0:
        raise RuntimeError(f"Required MuJoCo object not found: {name!r} ({obj_type})")
    return idx


def optional_object_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> int | None:
    idx = int(mujoco.mj_name2id(model, obj_type, name))
    return None if idx < 0 else idx


def set_ctrl_clipped(model: mujoco.MjModel, data: mujoco.MjData, actuator_id: int, value: float) -> None:
    if bool(model.actuator_ctrllimited[actuator_id]):
        low, high = model.actuator_ctrlrange[actuator_id]
        value = float(np.clip(value, low, high))
    data.ctrl[actuator_id] = float(value)


def mat_to_quat(matrix: np.ndarray) -> np.ndarray:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix, dtype=np.float64).reshape(9))
    norm = max(float(np.linalg.norm(quat)), 1e-12)
    return quat / norm


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    out = np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )
    return out / max(float(np.linalg.norm(out)), 1e-12)


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def synchronize_gripper_attachment(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    side: str,
    *,
    zero_velocity: bool = True,
) -> None:
    """Project a free Robotiq base exactly onto its FR3 flange pose.

    The physical connection remains a stiff MuJoCo weld during all simulated
    motion and contact. This projection is used only during initialization so
    both free gripper bases start exactly at their flange poses. Runtime arm,
    base and dynamic-spine motion is transmitted through the weld constraints.
    """
    spec = ARM_SPECS[side]
    flange = object_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["flange"])
    free_joint = object_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, spec["attachment_freejoint"]
    )
    qpos_address = int(model.jnt_qposadr[free_joint])
    dof_address = int(model.jnt_dofadr[free_joint])
    desired_quat = quat_mul(data.xquat[flange], spec["mount_quat"])
    desired_quat /= max(float(np.linalg.norm(desired_quat)), 1e-12)
    data.qpos[qpos_address:qpos_address + 3] = data.xpos[flange]
    data.qpos[qpos_address + 3:qpos_address + 7] = desired_quat
    if zero_velocity:
        data.qvel[dof_address:dof_address + 6] = 0.0


def synchronize_all_gripper_attachments(
    model: mujoco.MjModel, data: mujoco.MjData
) -> None:
    for side in ("left", "right"):
        synchronize_gripper_attachment(model, data, side)


def rotation_error(target_quat: np.ndarray, current_matrix: np.ndarray) -> np.ndarray:
    """Axis-angle rotation vector taking current orientation to target."""
    current_quat = mat_to_quat(current_matrix)
    delta = quat_mul(target_quat, quat_conjugate(current_quat))
    if delta[0] < 0.0:
        delta = -delta
    axis_norm = float(np.linalg.norm(delta[1:]))
    if axis_norm < 1e-10:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(axis_norm, max(float(delta[0]), 1e-10))
    return delta[1:] / axis_norm * angle


def planar_body_axis(
    data: mujoco.MjData,
    body_id: int,
    axis_name: str,
) -> np.ndarray:
    sign = -1.0 if axis_name.startswith("-") else 1.0
    local_axis = axis_name[-1].lower()
    column = 0 if local_axis == "x" else 1
    axis = sign * data.xmat[body_id].reshape(3, 3)[:, column].copy()
    axis[2] = 0.0
    norm = float(np.linalg.norm(axis))
    if norm < 1e-10:
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    return axis / norm


def robot_local_xy_to_world(
    data: mujoco.MjData,
    body_id: int,
    forward: float,
    left: float,
    forward_axis: str,
) -> np.ndarray:
    fwd = planar_body_axis(data, body_id, forward_axis)
    left_axis = np.cross(np.array([0.0, 0.0, 1.0]), fwd)
    left_axis /= max(float(np.linalg.norm(left_axis)), 1e-10)
    world = forward * fwd + left * left_axis
    return np.array([world[0], world[1], 0.0], dtype=np.float64)


def viewer_camera_basis(cam) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return MuJoCo viewer camera (forward, right, up) axes in world coordinates.

    The formula exactly follows MuJoCo's ``mjv_cameraFrame`` convention for
    free/tracking cameras:

      forward = [cos(elev) cos(az), cos(elev) sin(az), sin(elev)]
      right   = [sin(az), -cos(az), 0]
      up      = [-sin(elev) cos(az), -sin(elev) sin(az), cos(elev)]

    ``forward`` points from the viewer camera into the displayed scene,
    ``right`` points to screen-right, and ``up`` points to screen-up.
    """
    azimuth = math.radians(float(cam.azimuth))
    elevation = math.radians(float(cam.elevation))
    ca, sa = math.cos(azimuth), math.sin(azimuth)
    ce, se = math.cos(elevation), math.sin(elevation)

    forward = np.array([ce * ca, ce * sa, se], dtype=np.float64)
    right = np.array([sa, -ca, 0.0], dtype=np.float64)
    up = np.array([-se * ca, -se * sa, ce], dtype=np.float64)

    # Guard against numerical drift and make the basis explicitly orthonormal.
    forward /= max(float(np.linalg.norm(forward)), 1e-12)
    right /= max(float(np.linalg.norm(right)), 1e-12)
    up /= max(float(np.linalg.norm(up)), 1e-12)
    return forward, right, up


def camera_xy_to_world(cam, forward: float, right: float) -> np.ndarray:
    """Map viewer forward/right to an orthonormal horizontal world basis.

    Screen-right is always horizontal in MuJoCo's free camera. Horizontal
    screen-forward is reconstructed as ``world_up x screen_right``. This is
    numerically stable even when the camera is steeply tilted.
    """
    _, camera_right, _ = viewer_camera_basis(cam)
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    right_h = camera_right.copy()
    right_h[2] = 0.0
    right_norm = float(np.linalg.norm(right_h))
    if right_norm < 1e-12:
        right_h = np.array([0.0, -1.0, 0.0], dtype=np.float64)
    else:
        right_h /= right_norm

    forward_h = np.cross(world_up, right_h)
    forward_h /= max(float(np.linalg.norm(forward_h)), 1e-12)
    return forward * forward_h + right * right_h


def camera_translation_to_world(
    cam,
    forward: float,
    right: float,
    up: float,
) -> np.ndarray:
    """Map arm translation like the mobile-base camera control.

    Up/Down and Left/Right use the camera view projected onto the horizontal
    world XY plane. PageUp/PageDown are independent world-Z motion. Camera
    elevation therefore never introduces unintended vertical motion.
    """
    horizontal = camera_xy_to_world(cam, forward, right)
    return np.array([horizontal[0], horizontal[1], up], dtype=np.float64)


def camera_rotation_to_world(
    cam,
    pitch: float,
    roll: float,
    yaw: float,
) -> np.ndarray:
    """Map camera-relative pitch/roll/yaw rates to a world angular velocity.

    * pitch is rotation about screen-right;
    * roll is rotation about camera-forward (into the screen);
    * yaw is rotation about screen-up.
    """
    camera_forward, camera_right, camera_up = viewer_camera_basis(cam)
    return (
        pitch * camera_right
        + roll * camera_forward
        + yaw * camera_up
    )


def body_local_vector_to_world(
    data: mujoco.MjData,
    body_id: int,
    local_vector: np.ndarray,
) -> np.ndarray:
    """Rotate a vector from one MuJoCo body frame into world coordinates."""
    rotation = data.xmat[body_id].reshape(3, 3)
    return rotation @ np.asarray(local_vector, dtype=np.float64)


def rotation_pivot_linear_correction(
    data: mujoco.MjData,
    arm: "ArmState",
    gain: float,
    max_speed: float,
) -> np.ndarray:
    """Return a world-frame linear correction that holds the rotation TCP fixed.

    The TCP body is located at the midpoint of the lowest fingertip surfaces.
    Pure angular Jacobian commands should already rotate about that point, but
    finite-step integration and the articulated arm servos can introduce small
    drift. This proportional correction keeps the pivot at the position stored
    when rotation mode was entered.
    """
    error = np.asarray(arm.target_pos - data.site_xpos[arm.tcp_site], dtype=np.float64)
    correction = float(gain) * error
    speed = float(np.linalg.norm(correction))
    if speed > float(max_speed) > 0.0:
        correction *= float(max_speed) / speed
    return correction


def map_arm_operator_twist_to_world(
    cam,
    data: mujoco.MjData,
    tcp_body: int,
    base_body: int,
    operator_twist: np.ndarray,
    translation_frame: str,
    rotation_frame: str,
    robot_forward_axis: str,
) -> np.ndarray:
    """Convert keyboard operator axes into the world twist required by mj_jacBody.

    The input convention is:

      linear  = [local_X, local_Y, local_Z]
      angular = [roll_X, pitch_Y, yaw_Z]

    ``mj_jacBody`` returns a Jacobian whose linear and angular rows are expressed
    in world coordinates, so this function performs all frame conversion before
    damped least-squares IK.
    """
    operator_twist = np.asarray(operator_twist, dtype=np.float64)
    if operator_twist.shape != (6,):
        raise ValueError(f"operator_twist must have shape (6,), got {operator_twist.shape}")

    world_twist = np.zeros(6, dtype=np.float64)
    linear = operator_twist[:3]
    angular = operator_twist[3:]

    if translation_frame == "camera":
        world_twist[:3] = camera_translation_to_world(
            cam, float(linear[0]), float(linear[1]), float(linear[2])
        )
    elif translation_frame == "base":
        horizontal = robot_local_xy_to_world(
            data,
            base_body,
            float(linear[0]),
            float(-linear[1]),  # helper expects left; operator component is right
            robot_forward_axis,
        )
        world_twist[:3] = np.array(
            [horizontal[0], horizontal[1], float(linear[2])],
            dtype=np.float64,
        )
    elif translation_frame == "tcp":
        world_twist[:3] = body_local_vector_to_world(data, tcp_body, linear)
    else:
        raise ValueError(f"Unsupported arm translation frame: {translation_frame}")

    if rotation_frame == "camera":
        # Legacy compatibility only: angular = [roll, pitch, yaw].
        world_twist[3:] = camera_rotation_to_world(
            cam,
            pitch=float(angular[1]),
            roll=float(angular[0]),
            yaw=float(angular[2]),
        )
    elif rotation_frame == "world":
        # Legacy behavior: components are rotations about world X/Y/Z.
        world_twist[3:] = angular
    elif rotation_frame == "tcp":
        # Tool-local X/Y/Z angular velocity mapped into world coordinates.
        world_twist[3:] = body_local_vector_to_world(data, tcp_body, angular)
    else:
        raise ValueError(f"Unsupported arm rotation frame: {rotation_frame}")

    return world_twist


def screen_to_base_local(
    cam,
    screen_right: float,
    screen_forward: float,
    data: mujoco.MjData,
    base_body: int,
    forward_axis: str,
) -> tuple[float, float]:
    """Convert screen axes to robot-local (forward, left) axes."""
    world = camera_xy_to_world(cam, screen_forward, screen_right)
    fwd = planar_body_axis(data, base_body, forward_axis)
    left_axis = np.cross(np.array([0.0, 0.0, 1.0]), fwd)
    left_axis /= max(float(np.linalg.norm(left_axis)), 1e-10)
    return float(world @ fwd), float(world @ left_axis)


def log_robot_frames(model: mujoco.MjModel, data: mujoco.MjData, base_body: int, spine_joint: int) -> None:
    """Print the actual base and lift axes in world coordinates."""
    base_rotation = data.xmat[base_body].reshape(3, 3)
    local_x_world = base_rotation[:, 0]
    local_y_world = base_rotation[:, 1]
    local_z_world = base_rotation[:, 2]
    spine_axis_world = base_rotation @ model.jnt_axis[spine_joint]
    log(
        "[frames] base local +X(forward)->world="
        f"{np.round(local_x_world, 4)}, +Y(left)->world={np.round(local_y_world, 4)}, "
        f"+Z->world={np.round(local_z_world, 4)}"
    )
    log(f"[frames] spine local axis -> world={np.round(spine_axis_world, 4)}")


def dump_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    log(f"[contacts] ncon={data.ncon} nefc={data.nefc}")
    for i in range(data.ncon):
        contact = data.contact[i]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or f"geom{contact.geom1}"
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or f"geom{contact.geom2}"
        log(f"  {i:4d}: {g1} <-> {g2}, dist={float(contact.dist):+.6f}")


def disable_wheel_ground_collision(model: mujoco.MjModel) -> int:
    """Disable wheel/caster contacts when the virtual planar joints drive the base."""
    count = 0
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if "caster_" in body_name or "argo_drive_" in body_name:
            if model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]:
                model.geom_contype[geom_id] = 0
                model.geom_conaffinity[geom_id] = 0
                count += 1
    return count


def collision_masks_allow(model: mujoco.MjModel, geom_a: int, geom_b: int) -> bool:
    """MuJoCo's bit-mask rule for whether two geoms may generate contact."""
    return bool(
        (int(model.geom_contype[geom_a]) & int(model.geom_conaffinity[geom_b]))
        or (int(model.geom_contype[geom_b]) & int(model.geom_conaffinity[geom_a]))
    )


def robot_body_ids(model: mujoco.MjModel, base_body: int) -> set[int]:
    """Return all bodies belonging to the robot, including welded attachments.

    The right Robotiq gripper is intentionally represented as a top-level free
    body and rigidly connected to the FR3 flange with an equality weld.  It is
    therefore not a kinematic descendant of ``base_link`` even though it is a
    physical part of the robot.  Include each configured gripper root and all
    of its descendants so startup collision validation does not misclassify
    the welded gripper as an environment object.
    """
    roots = {base_body}
    for spec in ARM_SPECS.values():
        gripper_name = spec.get("gripper_base")
        if not gripper_name:
            continue
        gripper_body = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, gripper_name
        )
        if gripper_body >= 0:
            roots.add(int(gripper_body))

    result: set[int] = set()
    for body_id in range(model.nbody):
        ancestor = body_id
        while ancestor > 0:
            if ancestor in roots:
                result.add(body_id)
                break
            ancestor = int(model.body_parentid[ancestor])
    return result


def validate_robot_environment_contacts(model: mujoco.MjModel) -> dict[str, int]:
    """Fail fast if an active environment collider cannot contact the robot."""
    base_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
    robot_bodies = robot_body_ids(model, base_body)
    robot_geoms = [
        geom_id for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) in robot_bodies
        and (int(model.geom_contype[geom_id]) or int(model.geom_conaffinity[geom_id]))
    ]
    environment_geoms = [
        geom_id for geom_id in range(model.ngeom)
        if int(model.geom_bodyid[geom_id]) not in robot_bodies
        and (int(model.geom_contype[geom_id]) or int(model.geom_conaffinity[geom_id]))
    ]
    incompatible = [
        geom_id for geom_id in environment_geoms
        if not any(collision_masks_allow(model, geom_id, robot_geom) for robot_geom in robot_geoms)
    ]
    if incompatible:
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom{geom_id}"
            for geom_id in incompatible[:20]
        ]
        raise RuntimeError(
            "Environment collision geoms cannot contact the robot due to "
            f"contype/conaffinity masks: {names}"
        )

    counts = {"robot": len(robot_geoms), "environment": len(environment_geoms)}
    terms = {
        "spoon": ("spoon",),
        "bowl": ("bowl",),
        "plate": ("plate",),
        "beans": ("bean",),
        "floor": ("floor",),
    }
    for label, needles in terms.items():
        count = 0
        for geom_id in environment_geoms:
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
            body_name = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id])
            ) or ""
            text = f"{geom_name} {body_name}".lower()
            if any(needle in text for needle in needles):
                count += 1
        counts[label] = count
    return counts


def count_initial_robot_environment_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> int:
    base_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
    robot_bodies = robot_body_ids(model, base_body)
    count = 0
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        body_1 = int(model.geom_bodyid[int(contact.geom1)])
        body_2 = int(model.geom_bodyid[int(contact.geom2)])
        if (body_1 in robot_bodies) != (body_2 in robot_bodies):
            count += 1
    return count


# ---------------------------------------------------------------------------
# Keyboard handling: copied in behavior from the uploaded teleop method.
# ---------------------------------------------------------------------------

_MOTION_KEYS_TO_WINDOWS_VK = {
    glfw.KEY_UP: 0x26,
    glfw.KEY_DOWN: 0x28,
    glfw.KEY_LEFT: 0x25,
    glfw.KEY_RIGHT: 0x27,
    glfw.KEY_PAGE_UP: 0x21,
    glfw.KEY_PAGE_DOWN: 0x22,
    glfw.KEY_HOME: 0x24,
    glfw.KEY_END: 0x23,
    # Letter-key aliases are important on systems where the MuJoCo viewer
    # consumes the arrow keys for camera navigation before GLFW polling.
    glfw.KEY_W: 0x57,
    glfw.KEY_S: 0x53,
    glfw.KEY_A: 0x41,
    glfw.KEY_D: 0x44,
    glfw.KEY_Q: 0x51,
    glfw.KEY_E: 0x45,
    glfw.KEY_U: 0x55,
    glfw.KEY_J: 0x4A,
}

_GET_ASYNC_KEY_STATE = None
if os.name == "nt":
    try:
        _GET_ASYNC_KEY_STATE = ctypes.windll.user32.GetAsyncKeyState
    except Exception:
        _GET_ASYNC_KEY_STATE = None

_X11_KEYSYM_NAMES = {
    glfw.KEY_UP: "Up",
    glfw.KEY_DOWN: "Down",
    glfw.KEY_LEFT: "Left",
    glfw.KEY_RIGHT: "Right",
    glfw.KEY_PAGE_UP: "Page_Up",
    glfw.KEY_PAGE_DOWN: "Page_Down",
    glfw.KEY_HOME: "Home",
    glfw.KEY_END: "End",
    glfw.KEY_W: "w",
    glfw.KEY_S: "s",
    glfw.KEY_A: "a",
    glfw.KEY_D: "d",
    glfw.KEY_Q: "q",
    glfw.KEY_E: "e",
    glfw.KEY_U: "u",
    glfw.KEY_J: "j",
}
_X11_DISPLAY = None
_X11_ERROR: str | None = None
_X11_KEYCODES: dict[int, int] = {}


def _initialize_x11_polling() -> None:
    global _X11_DISPLAY, _X11_ERROR
    try:
        from Xlib import XK
        from Xlib.display import Display

        display = Display()
        for glfw_key, keysym_name in _X11_KEYSYM_NAMES.items():
            keysym = XK.string_to_keysym(keysym_name)
            keycode = display.keysym_to_keycode(keysym)
            if keycode:
                _X11_KEYCODES[glfw_key] = int(keycode)
        _X11_DISPLAY = display
    except Exception as exc:
        _X11_DISPLAY = None
        _X11_ERROR = repr(exc)


if sys.platform.startswith("linux"):
    _initialize_x11_polling()


def held_key_backend() -> str:
    if _GET_ASYNC_KEY_STATE is not None:
        return "GetAsyncKeyState (Windows)"
    if _X11_DISPLAY is not None:
        return "X11 query_keymap"
    return (
        "GLFW/press-timeout fallback"
        + (f" ({_X11_ERROR})" if _X11_ERROR else "")
        + "; install python-xlib on Linux for reliable held keys"
    )


class KeyboardInput:
    """Continuous press/release keyboard state plus queued one-shot events.

    Normal motion never uses a time-based hold timeout. The default ``auto``
    backend prefers pynput press/release events, then Windows GetAsyncKeyState
    or Linux X11 query_keymap. GLFW polling is available only when explicitly
    requested because some MuJoCo versions do not expose their viewer window.
    """

    _DISCRETE_KEYS = {
        glfw.KEY_ESCAPE,
        glfw.KEY_7,
        glfw.KEY_8,
        glfw.KEY_9,
        glfw.KEY_KP_7,
        glfw.KEY_KP_8,
        glfw.KEY_KP_9,
        glfw.KEY_R,
        glfw.KEY_G,
        glfw.KEY_V,
        glfw.KEY_SPACE,
        glfw.KEY_MINUS,
        glfw.KEY_EQUAL,
        glfw.KEY_KP_SUBTRACT,
        glfw.KEY_KP_ADD,
        glfw.KEY_C,
        glfw.KEY_B,
        glfw.KEY_N,
    }

    def __init__(
        self,
        hold_timeout: float = 0.40,
        callback_grace: float = 0.12,
        backend: str = "auto",
    ) -> None:
        # These two values are retained only for CLI compatibility.
        self.hold_timeout = float(hold_timeout)
        self.callback_grace = float(callback_grace)
        self.backend = str(backend)
        self.held_keys: set[int] = set()
        self._pending: deque[int] = deque()
        self._physical_keys: set[int] = set()
        self._pynput_down: set[int] = set()
        self._pynput_listener = None
        self._pynput_special: dict[object, int] = {}
        self._lock = threading.Lock()
        self._active_backend = ""
        self._backend_notes: list[str] = []

    @staticmethod
    def _char_to_glfw(char: str | None) -> int | None:
        if not char:
            return None
        char = char.lower()
        mapping = {
            "w": glfw.KEY_W,
            "s": glfw.KEY_S,
            "a": glfw.KEY_A,
            "d": glfw.KEY_D,
            "q": glfw.KEY_Q,
            "e": glfw.KEY_E,
            "u": glfw.KEY_U,
            "j": glfw.KEY_J,
            "7": glfw.KEY_7,
            "8": glfw.KEY_8,
            "9": glfw.KEY_9,
            "r": glfw.KEY_R,
            "g": glfw.KEY_G,
            "v": glfw.KEY_V,
            "c": glfw.KEY_C,
            "b": glfw.KEY_B,
            "n": glfw.KEY_N,
            "-": glfw.KEY_MINUS,
            "_": glfw.KEY_MINUS,
            "=": glfw.KEY_EQUAL,
            "+": glfw.KEY_EQUAL,
            " ": glfw.KEY_SPACE,
        }
        return mapping.get(char)

    def _pynput_to_glfw(self, key) -> int | None:
        mapped = self._pynput_special.get(key)
        if mapped is not None:
            return mapped
        return self._char_to_glfw(getattr(key, "char", None))

    def _on_pynput_press(self, key) -> None:
        code = self._pynput_to_glfw(key)
        if code is None:
            return
        with self._lock:
            first_press = code not in self._pynput_down
            self._pynput_down.add(code)
            if code in _MOTION_KEYS_TO_WINDOWS_VK:
                self._physical_keys.add(code)
            if first_press and code in self._DISCRETE_KEYS:
                self._pending.append(code)

    def _on_pynput_release(self, key) -> None:
        code = self._pynput_to_glfw(key)
        if code is None:
            return
        with self._lock:
            self._pynput_down.discard(code)
            self._physical_keys.discard(code)

    def start(self) -> None:
        if self.backend not in ("auto", "pynput", "x11", "glfw", "callback"):
            raise ValueError(f"Unsupported keyboard backend: {self.backend}")

        if self.backend in ("auto", "pynput"):
            try:
                from pynput import keyboard as pynput_keyboard

                self._pynput_special = {
                    pynput_keyboard.Key.up: glfw.KEY_UP,
                    pynput_keyboard.Key.down: glfw.KEY_DOWN,
                    pynput_keyboard.Key.left: glfw.KEY_LEFT,
                    pynput_keyboard.Key.right: glfw.KEY_RIGHT,
                    pynput_keyboard.Key.page_up: glfw.KEY_PAGE_UP,
                    pynput_keyboard.Key.page_down: glfw.KEY_PAGE_DOWN,
                    pynput_keyboard.Key.home: glfw.KEY_HOME,
                    pynput_keyboard.Key.end: glfw.KEY_END,
                    pynput_keyboard.Key.esc: glfw.KEY_ESCAPE,
                    pynput_keyboard.Key.space: glfw.KEY_SPACE,
                }
                listener = pynput_keyboard.Listener(
                    on_press=self._on_pynput_press,
                    on_release=self._on_pynput_release,
                )
                listener.daemon = True
                listener.start()
                listener.wait()
                self._pynput_listener = listener
                self._active_backend = "pynput press/release"
                return
            except Exception as exc:
                self._backend_notes.append(f"pynput unavailable: {exc!r}")
                self._pynput_listener = None
                if self.backend == "pynput":
                    raise RuntimeError(
                        "Could not start pynput. Install it with `pip install pynput` "
                        "and run the MuJoCo viewer through X11/XWayland."
                    ) from exc

        if self.backend in ("auto", "x11") and _X11_DISPLAY is not None:
            self._active_backend = "X11 query_keymap"
            return
        if _GET_ASYNC_KEY_STATE is not None and self.backend == "auto":
            self._active_backend = "GetAsyncKeyState"
            return
        # GLFW polling is always available with mujoco.viewer and needs no
        # pynput/python-xlib dependency. The actual viewer window is attached
        # after launch_passive() starts.
        if self.backend == "glfw":
            self._active_backend = "GLFW live key polling"
            return
        if self.backend == "callback":
            self._active_backend = "viewer callback press/release"
            return

        details = "; ".join(self._backend_notes)
        if _X11_ERROR:
            details = f"{details}; X11: {_X11_ERROR}" if details else f"X11: {_X11_ERROR}"
        raise RuntimeError(
            "No release-aware keyboard backend could be started. Install the package "
            "dependencies with `python -m pip install -r requirements_teleop.txt`. "
            "On Ubuntu/Wayland, run the viewer through XWayland or log into an X11 "
            "session. " + (f"Details: {details}" if details else "")
        )

    def stop(self) -> None:
        listener = self._pynput_listener
        self._pynput_listener = None
        if listener is not None:
            try:
                listener.stop()
            except Exception:
                pass
        self.clear_motion_state()

    def backend_description(self, window_available: bool = False) -> str:
        extra = " + GLFW" if window_available else ""
        return (self._active_backend or "not started") + extra

    def key_callback(self, keycode: int) -> None:
        # Pynput is the sole event source when active, preventing duplicates.
        if self._pynput_listener is not None:
            return
        key = abs(int(keycode))
        with self._lock:
            if keycode < 0:
                self.held_keys.discard(key)
                return

            # Motion keys are read continuously with glfw.get_key(window, key)
            # when the GLFW backend is active. Do not retain callback presses,
            # because some MuJoCo viewer versions do not emit release callbacks.
            if self._active_backend != "GLFW live key polling":
                self.held_keys.add(key)

            # One-shot mode/gripper/debug commands still come from the callback.
            if key in self._DISCRETE_KEYS:
                self._pending.append(key)

    def drain(self) -> list[int]:
        events: list[int] = []
        with self._lock:
            while self._pending:
                events.append(self._pending.popleft())
        return events

    def clear_motion_state(self) -> None:
        with self._lock:
            self.held_keys.clear()
            self._physical_keys.clear()
            self._pynput_down.clear()

    def _windows_down_keys(self, keys: tuple[int, ...]) -> set[int]:
        if _GET_ASYNC_KEY_STATE is None:
            return set()
        result: set[int] = set()
        for glfw_key in keys:
            vk = _MOTION_KEYS_TO_WINDOWS_VK.get(glfw_key)
            if vk is not None and bool(_GET_ASYNC_KEY_STATE(vk) & 0x8000):
                result.add(glfw_key)
        return result

    def _x11_down_keys(self, keys: tuple[int, ...]) -> set[int]:
        if _X11_DISPLAY is None:
            return set()
        keymap = _X11_DISPLAY.query_keymap()
        result: set[int] = set()
        for glfw_key in keys:
            keycode = _X11_KEYCODES.get(glfw_key)
            if keycode is not None and bool(keymap[keycode // 8] & (1 << (keycode % 8))):
                result.add(glfw_key)
        return result

    def _down_keys(self, window) -> set[int]:
        keys = tuple(_MOTION_KEYS_TO_WINDOWS_VK)
        if self._pynput_listener is not None:
            with self._lock:
                down = {key for key in self._physical_keys if key in keys}
        elif self._active_backend == "GetAsyncKeyState":
            down = self._windows_down_keys(keys)
        elif self._active_backend == "X11 query_keymap":
            down = self._x11_down_keys(keys)
        elif self._active_backend == "GLFW live key polling":
            down = set()
        else:
            # Explicit callback backend only.
            with self._lock:
                down = {key for key in self.held_keys if key in keys}

        if window is not None:
            try:
                # Safety: only accept continuous motion while the MuJoCo
                # viewer is focused. Losing focus immediately produces a zero
                # command, preventing a stuck moving robot.
                focused = bool(glfw.get_window_attrib(window, glfw.FOCUSED))
                if focused:
                    down |= {
                        key for key in keys
                        if glfw.get_key(window, key) == glfw.PRESS
                    }
                elif self._active_backend == "GLFW live key polling":
                    down.clear()
            except Exception:
                if self._active_backend == "GLFW live key polling":
                    down.clear()
        return down

    def poll(
        self,
        window,
        mode: str,
        rotate_mode: bool,
        move: float,
        rot: float,
    ) -> tuple[np.ndarray, np.ndarray, bool, bool]:
        base = np.zeros(4, dtype=np.float64)
        twist = np.zeros(6, dtype=np.float64)
        down = self._down_keys(window)

        def held(*codes: int) -> bool:
            return any(code in down for code in codes)

        if mode == "base":
            if held(glfw.KEY_UP, glfw.KEY_W):
                base[0] += 1.0
            if held(glfw.KEY_DOWN, glfw.KEY_S):
                base[0] -= 1.0
            if held(glfw.KEY_LEFT, glfw.KEY_A):
                base[1] += 1.0
            if held(glfw.KEY_RIGHT, glfw.KEY_D):
                base[1] -= 1.0
            if held(glfw.KEY_PAGE_UP, glfw.KEY_U):
                base[2] += 1.0
            if held(glfw.KEY_PAGE_DOWN, glfw.KEY_J):
                base[2] -= 1.0
            if held(glfw.KEY_HOME, glfw.KEY_Q):
                base[3] += 1.0
            if held(glfw.KEY_END, glfw.KEY_E):
                base[3] -= 1.0
            return base, twist, False, False

        if rotate_mode:
            # Current TCP-local angular command:
            # twist[3] = roll  about local X
            # twist[4] = pitch about local Y
            # twist[5] = yaw   about local Z
            if held(glfw.KEY_PAGE_UP, glfw.KEY_U):
                twist[3] += rot
            if held(glfw.KEY_PAGE_DOWN, glfw.KEY_J):
                twist[3] -= rot
            if held(glfw.KEY_UP, glfw.KEY_W):
                twist[4] += rot
            if held(glfw.KEY_DOWN, glfw.KEY_S):
                twist[4] -= rot
            if held(glfw.KEY_LEFT, glfw.KEY_A):
                twist[5] += rot
            if held(glfw.KEY_RIGHT, glfw.KEY_D):
                twist[5] -= rot
            return base, twist, False, float(np.linalg.norm(twist[3:])) > 1e-8

        # Operator linear command. The selected --arm-frame converts these
        # components to world coordinates. In the default base frame, U/PageUp
        # is always world +Z and J/PageDown is always world -Z.
        if held(glfw.KEY_UP, glfw.KEY_W):
            twist[0] += move
        if held(glfw.KEY_DOWN, glfw.KEY_S):
            twist[0] -= move
        if held(glfw.KEY_LEFT, glfw.KEY_A):
            twist[1] -= move
        if held(glfw.KEY_RIGHT, glfw.KEY_D):
            twist[1] += move
        if held(glfw.KEY_PAGE_UP, glfw.KEY_U):
            twist[2] += move
        if held(glfw.KEY_PAGE_DOWN, glfw.KEY_J):
            twist[2] -= move
        return base, twist, float(np.linalg.norm(twist[:3])) > 1e-8, False


# ---------------------------------------------------------------------------
# Arm and gripper control
# ---------------------------------------------------------------------------


@dataclass
class ArmState:
    name: str
    tcp_body: int
    tcp_site: int
    target_mocap: int
    joint_ids: np.ndarray
    dof_ids: np.ndarray
    actuator_ids: np.ndarray
    gripper_joint: int
    gripper_actuator: int
    gripper_base_body: int
    pad_left_main_geom: int
    pad_right_main_geom: int
    pad_left_body: int
    pad_right_body: int
    pad_left_geoms: set[int]
    pad_right_geoms: set[int]
    own_arm_geoms: set[int]
    static_wall_geoms: set[int]
    spoon_body: int
    spoon_stem_geoms: set[int]
    q_ref: np.ndarray
    target_pos: np.ndarray
    target_quat: np.ndarray
    translate_lock_quat: np.ndarray
    rotate_mode: bool = False
    close_ramp: bool = False
    was_command_active: bool = False
    filtered_left_normal_force: float = 0.0
    filtered_right_normal_force: float = 0.0
    force_threshold_steps: int = 0
    gripper_hold_target: float = 0.0
    wall_contact_active: bool = False


def named_geom_family(model: mujoco.MjModel, prefix: str) -> set[int]:
    result: set[int] = set()
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name.startswith(prefix):
            result.add(geom_id)
    return result


def arm_geom_ids(model: mujoco.MjModel, body_prefix: str) -> set[int]:
    result: set[int] = set()
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if body_name.startswith(body_prefix):
            result.add(geom_id)
    return result


def static_wall_geom_ids(model: mujoco.MjModel) -> set[int]:
    """Return fixed scene geoms using the dedicated wall material.

    Bowl side geoms also contain the word ``wall`` in their names, so name
    matching is intentionally avoided. The room walls use
    ``collision_wall_mat`` and belong to the static world root.
    """
    material_id = optional_object_id(
        model, mujoco.mjtObj.mjOBJ_MATERIAL, "collision_wall_mat"
    )
    if material_id is None:
        return set()
    return {
        geom_id
        for geom_id in range(model.ngeom)
        if int(model.geom_matid[geom_id]) == int(material_id)
        and int(model.body_rootid[int(model.geom_bodyid[geom_id])]) == 0
        and int(model.geom_contype[geom_id]) != 0
    }


def arm_static_wall_contacts(
    model: mujoco.MjModel, data: mujoco.MjData, arm: "ArmState"
) -> list[tuple[int, int]]:
    """Return active contacts between one arm/gripper and static room walls."""
    hits: list[tuple[int, int]] = []
    if not arm.static_wall_geoms:
        return hits
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        if geom1 in arm.own_arm_geoms and geom2 in arm.static_wall_geoms:
            hits.append((geom1, geom2))
        elif geom2 in arm.own_arm_geoms and geom1 in arm.static_wall_geoms:
            hits.append((geom2, geom1))
    return hits


def describe_wall_contacts(
    model: mujoco.MjModel, contacts: list[tuple[int, int]], max_items: int = 3
) -> str:
    items: list[str] = []
    for arm_geom, wall_geom in contacts[:max_items]:
        arm_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, arm_geom)
            or f"geom{arm_geom}"
        )
        wall_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, wall_geom)
            or f"geom{wall_geom}"
        )
        items.append(f"{arm_name}<->{wall_name}")
    if len(contacts) > max_items:
        items.append(f"+{len(contacts)-max_items} more")
    return ", ".join(items)


def body_is_descendant(model: mujoco.MjModel, body_id: int, ancestor_body_id: int) -> bool:
    """Return True for the ancestor itself or any rigid/dynamic child body below it."""
    current = int(body_id)
    while current >= 0:
        if current == int(ancestor_body_id):
            return True
        parent = int(model.body_parentid[current])
        if parent == current:
            break
        current = parent
    return False


def create_arm(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> ArmState:
    spec = ARM_SPECS[side]
    joint_ids = np.array(
        [object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in spec["joints"]],
        dtype=np.int32,
    )
    dof_ids = np.array([int(model.jnt_dofadr[jid]) for jid in joint_ids], dtype=np.int32)
    actuator_ids = np.array(
        [object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in spec["joints"]],
        dtype=np.int32,
    )
    tcp_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["tcp"])
    tcp_site = object_id(model, mujoco.mjtObj.mjOBJ_SITE, spec["tcp_site"])
    target_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["target"])
    mocap_id = int(model.body_mocapid[target_body])
    if mocap_id < 0:
        raise RuntimeError(f"Body {spec['target']!r} is not a mocap body")

    gripper_joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, spec["gripper"])
    gripper_actuator = object_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec["gripper"])
    gripper_base_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, spec["gripper_base"])
    pad_left_geoms = named_geom_family(model, spec["pad_left_prefix"])
    pad_right_geoms = named_geom_family(model, spec["pad_right_prefix"])
    if not pad_left_geoms or not pad_right_geoms:
        raise RuntimeError(f"No active grasp geoms were found for the {side} gripper")
    pad_left_main_geom = optional_object_id(model, mujoco.mjtObj.mjOBJ_GEOM, spec["pad_left_main"])
    pad_right_main_geom = optional_object_id(model, mujoco.mjtObj.mjOBJ_GEOM, spec["pad_right_main"])
    if pad_left_main_geom is None:
        pad_left_main_geom = min(pad_left_geoms)
    if pad_right_main_geom is None:
        pad_right_main_geom = min(pad_right_geoms)
    pad_left_body = int(model.geom_bodyid[pad_left_main_geom])
    pad_right_body = int(model.geom_bodyid[pad_right_main_geom])
    spoon_body = optional_object_id(model, mujoco.mjtObj.mjOBJ_BODY, "dynamic_spoon2")
    if spoon_body is None:
        spoon_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "spoon")
    spoon_stem_geoms = {
        geom_id
        for geom_id in range(model.ngeom)
        if body_is_descendant(model, int(model.geom_bodyid[geom_id]), spoon_body)
        and int(model.geom_contype[geom_id]) != 0
    }
    if not spoon_stem_geoms:
        raise RuntimeError("No active spoon collision geoms were found")
    q_ref = np.array([data.qpos[int(model.jnt_qposadr[jid])] for jid in joint_ids], dtype=np.float64)
    tcp_quat = mat_to_quat(data.site_xmat[tcp_site].reshape(3, 3))

    return ArmState(
        name=side,
        tcp_body=tcp_body,
        tcp_site=tcp_site,
        target_mocap=mocap_id,
        joint_ids=joint_ids,
        dof_ids=dof_ids,
        actuator_ids=actuator_ids,
        gripper_joint=gripper_joint,
        gripper_actuator=gripper_actuator,
        gripper_base_body=gripper_base_body,
        pad_left_main_geom=pad_left_main_geom,
        pad_right_main_geom=pad_right_main_geom,
        pad_left_body=pad_left_body,
        pad_right_body=pad_right_body,
        pad_left_geoms=pad_left_geoms,
        pad_right_geoms=pad_right_geoms,
        own_arm_geoms=arm_geom_ids(model, spec["body_prefix"]),
        static_wall_geoms=static_wall_geom_ids(model),
        spoon_body=spoon_body,
        spoon_stem_geoms=spoon_stem_geoms,
        q_ref=q_ref,
        target_pos=data.site_xpos[tcp_site].copy(),
        target_quat=tcp_quat.copy(),
        translate_lock_quat=tcp_quat.copy(),
    )


def align_gripper_collision_cuboids(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmState,
) -> None:
    """No-op for the exact small-model mesh contact variant.

    The dedicated collision STL is rigidly attached to each fingertip body.
    Its orientation must never be rewritten at runtime.
    """
    return

def sync_target_marker(data: mujoco.MjData, arm: ArmState) -> None:
    data.mocap_pos[arm.target_mocap] = arm.target_pos
    data.mocap_quat[arm.target_mocap] = arm.target_quat / max(float(np.linalg.norm(arm.target_quat)), 1e-12)


def write_zero_arm_velocity(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmState) -> None:
    """Compatibility name: continuously apply the persistent position target."""
    for actuator_id, target in zip(arm.actuator_ids, arm.q_ref):
        set_ctrl_clipped(model, data, int(actuator_id), float(target))


def synchronize_arm(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmState) -> None:
    """Capture the current simulated pose as the arm's new idle target."""
    for i, joint_id in enumerate(arm.joint_ids):
        arm.q_ref[i] = float(data.qpos[int(model.jnt_qposadr[int(joint_id)])])
    arm.target_pos = data.site_xpos[arm.tcp_site].copy()
    arm.target_quat = mat_to_quat(data.site_xmat[arm.tcp_site].reshape(3, 3))
    arm.translate_lock_quat = arm.target_quat.copy()
    arm.was_command_active = False
    sync_target_marker(data, arm)
    write_zero_arm_velocity(model, data, arm)


def apply_arm_gravity_compensation(data: mujoco.MjData, arm: ArmState) -> None:
    """Cancel model bias forces on the seven FR3 joints.

    The standalone small model holds the gripper base with a mocap weld and
    therefore has no arm gravity sag. Applying the current MuJoCo bias force
    as feed-forward torque makes the articulated arm reproduce that condition
    while preserving dynamic joint and contact simulation.
    """
    data.qfrc_applied[arm.dof_ids] = data.qfrc_bias[arm.dof_ids]


def hard_hold_arm(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmState) -> None:
    """Hold an idle FR3 using persistent position targets plus gravity compensation."""
    apply_arm_gravity_compensation(data, arm)
    write_zero_arm_velocity(model, data, arm)


def clamp_tcp_twist(
    model: mujoco.MjModel,
    twist: np.ndarray,
    max_linear_step: float,
    max_rotation_step: float,
) -> np.ndarray:
    """Limit requested TCP travel per MuJoCo physics step."""
    out = twist.copy()
    timestep = max(float(model.opt.timestep), 1e-8)
    max_linear_speed = max_linear_step / timestep
    linear_norm = float(np.linalg.norm(out[:3]))
    if linear_norm > max_linear_speed:
        out[:3] *= max_linear_speed / linear_norm

    max_rotation_speed = max_rotation_step / timestep
    angular_norm = float(np.linalg.norm(out[3:]))
    if angular_norm > max_rotation_speed:
        out[3:] *= max_rotation_speed / angular_norm
    return out


def apply_twist_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmState,
    twist: np.ndarray,
    damping: float,
    joint_velocity_limit: float,
    target_lookahead: float = 0.030,
    max_target_lead: float = 0.040,
) -> np.ndarray:
    """Map a world-frame TCP twist into bounded FR3 position targets.

    The IK result is a desired joint velocity, while the XML uses position
    actuators. Each target is rebuilt from the measured joint position plus
    a short look-ahead, so a slow or blocked joint cannot accumulate backlog.
    """
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)

    jacobian_position = np.zeros((3, model.nv), dtype=np.float64)
    jacobian_rotation = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(model, data, jacobian_position, jacobian_rotation, arm.tcp_site)
    jacobian = np.vstack(
        (
            jacobian_position[:, arm.dof_ids],
            jacobian_rotation[:, arm.dof_ids],
        )
    )

    regularized = jacobian @ jacobian.T + float(damping) ** 2 * np.eye(6)
    qdot = jacobian.T @ np.linalg.solve(regularized, twist)
    norm = float(np.linalg.norm(qdot))
    if norm > joint_velocity_limit > 0.0:
        qdot *= joint_velocity_limit / norm

    measured_q = np.array(
        [data.qpos[int(model.jnt_qposadr[int(jid)])] for jid in arm.joint_ids],
        dtype=np.float64,
    )
    horizon = max(float(model.opt.timestep), float(target_lookahead))
    # A position servo needs a small target lead to realize the desired qdot.
    # Rebuild that lead from measured q every cycle, and cap each joint's
    # position error so neither IK nor a slow/contact-loaded joint can create
    # a large servo impulse. This is a bounded velocity-to-position conversion,
    # not an accumulated trajectory target.
    target_delta = qdot * horizon
    if max_target_lead > 0.0:
        target_delta = np.clip(
            target_delta,
            -float(max_target_lead),
            float(max_target_lead),
        )
    arm.q_ref = measured_q + target_delta
    for i, joint_id in enumerate(arm.joint_ids):
        if int(model.jnt_limited[int(joint_id)]):
            lo, hi = model.jnt_range[int(joint_id)]
            arm.q_ref[i] = float(np.clip(arm.q_ref[i], lo, hi))
    apply_arm_gravity_compensation(data, arm)
    for actuator_id, target in zip(arm.actuator_ids, arm.q_ref):
        set_ctrl_clipped(model, data, int(actuator_id), float(target))
    return qdot


def external_pad_normal_force(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pad_geoms: set[int],
    own_arm_geoms: set[int],
    count_static_contact: bool = True,
    allowed_other_geoms: set[int] | None = None,
) -> float:
    """Sum normal contact force between one pad family and external geoms.

    ``mj_contactForce`` returns force in the contact frame. Component zero is
    the normal component; components one and two are tangential friction.
    Only the normal component is used for gripper stopping, so tangential
    sliding cannot create a false force-threshold event.
    """
    total_normal = 0.0
    contact_force = np.zeros(6, dtype=np.float64)
    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        is_external_pad_contact = (
            geom1 in pad_geoms and geom2 not in own_arm_geoms
        ) or (
            geom2 in pad_geoms and geom1 not in own_arm_geoms
        )
        if not is_external_pad_contact:
            continue

        other_geom = geom2 if geom1 in pad_geoms else geom1
        if allowed_other_geoms is not None and other_geom not in allowed_other_geoms:
            continue
        other_body = int(model.geom_bodyid[other_geom])
        if not count_static_contact and int(model.body_rootid[other_body]) == 0:
            continue

        contact_force.fill(0.0)
        mujoco.mj_contactForce(model, data, contact_index, contact_force)
        total_normal += abs(float(contact_force[0]))
    return total_normal


def spoon_pad_contact_summary(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmState,
) -> tuple[int, int, set[int], np.ndarray | None, dict[str, int]]:
    """Summarize active spoon-stem contact points on the two pad families.

    Each entry in ``mjData.contact`` is one geometric contact point. Its
    ``dim`` value is the number of constraint dimensions assigned to that
    point. For the spoon-pad pairs this should be 6: one normal, two sliding,
    one torsional, and two rolling-friction dimensions.
    """
    left_count = 0
    right_count = 0
    dimensions: set[int] = set()
    friction: np.ndarray | None = None
    by_geom: dict[str, int] = {}

    for contact_index in range(data.ncon):
        contact = data.contact[contact_index]
        geom1, geom2 = int(contact.geom1), int(contact.geom2)
        stem_geom = None
        if geom1 in arm.spoon_stem_geoms:
            stem_geom = geom1
            other_geom = geom2
        elif geom2 in arm.spoon_stem_geoms:
            stem_geom = geom2
            other_geom = geom1
        else:
            continue
        if other_geom in arm.pad_left_geoms:
            left_count += 1
        elif other_geom in arm.pad_right_geoms:
            right_count += 1
        else:
            continue

        geom_name = (
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other_geom)
            or f"geom_{other_geom}"
        )
        by_geom[geom_name] = by_geom.get(geom_name, 0) + 1
        dimensions.add(int(contact.dim))
        if friction is None:
            friction = np.array(contact.friction, dtype=np.float64)

    return left_count, right_count, dimensions, friction, by_geom


def start_gripper_closing(arm: ArmState) -> bool:
    """Start closing unless the gripper is already holding a fixed target.

    Pressing G repeatedly after a grasp is intentionally idempotent: it does
    not reset the held target or restart a close/open oscillation.
    """
    if arm.gripper_hold_target > 0.0 and not arm.close_ramp:
        log(
            f"[{arm.name} gripper] already holding target "
            f"{arm.gripper_hold_target:.4f}"
        )
        return False

    arm.close_ramp = True
    arm.gripper_hold_target = 0.0
    arm.filtered_left_normal_force = 0.0
    arm.filtered_right_normal_force = 0.0
    arm.force_threshold_steps = 0
    return True


def update_gripper(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm: ArmState,
    dt: float,
    close_rate: float,
    force_stop: float,
    force_filter_alpha: float,
    force_hold_steps: int,
    minimum_side_force: float,
    closed_tolerance: float,
    count_static_contact: bool = True,
    close_target: float = 0.735,
    hold_overdrive: float = 0.0,
) -> None:
    """Ramp to and continuously hold the small-model 0.735 rad preload.

    Contact forces are measured only for diagnostics. They never stop closure.
    This intentionally reproduces the standalone small model: pressing G ramps
    the main knuckle command to exactly 0.735 rad and reapplies that fixed
    position target on every subsequent simulation step.
    """
    del force_stop, force_hold_steps, minimum_side_force, closed_tolerance, hold_overdrive
    align_gripper_collision_cuboids(model, data, arm)

    alpha = float(np.clip(force_filter_alpha, 0.0, 1.0))
    raw_left = external_pad_normal_force(
        model, data, arm.pad_left_geoms, arm.own_arm_geoms,
        count_static_contact=count_static_contact,
    )
    raw_right = external_pad_normal_force(
        model, data, arm.pad_right_geoms, arm.own_arm_geoms,
        count_static_contact=count_static_contact,
    )
    arm.filtered_left_normal_force = (
        alpha * raw_left + (1.0 - alpha) * arm.filtered_left_normal_force
    )
    arm.filtered_right_normal_force = (
        alpha * raw_right + (1.0 - alpha) * arm.filtered_right_normal_force
    )

    _, actuator_high = model.actuator_ctrlrange[arm.gripper_actuator]
    _, joint_high = model.jnt_range[arm.gripper_joint]
    fixed_target = min(0.735, float(close_target), float(actuator_high), float(joint_high))

    if not arm.close_ramp:
        if arm.gripper_hold_target > 0.0:
            set_ctrl_clipped(model, data, arm.gripper_actuator, fixed_target)
            arm.gripper_hold_target = fixed_target
        return

    current_control = float(data.ctrl[arm.gripper_actuator])
    next_control = min(fixed_target, current_control + max(0.0, close_rate) * max(0.0, dt))
    set_ctrl_clipped(model, data, arm.gripper_actuator, next_control)

    if next_control >= fixed_target - 1e-12:
        arm.gripper_hold_target = fixed_target
        arm.close_ramp = False
        left_contacts, right_contacts, dimensions, friction, by_geom = (
            spoon_pad_contact_summary(model, data, arm)
        )
        friction_text = (
            np.array2string(friction, precision=3)
            if friction is not None else "none"
        )
        log(
            f"[{arm.name} gripper] fixed preload reached: target={fixed_target:.3f} rad; "
            f"filtered normal force left/right="
            f"{arm.filtered_left_normal_force:.2f}/{arm.filtered_right_normal_force:.2f} N; "
            f"spoon contacts={left_contacts}/{right_contacts}, "
            f"dim={sorted(dimensions)}, friction={friction_text}, by_geom={by_geom}"
        )

def open_gripper(model: mujoco.MjModel, data: mujoco.MjData, arm: ArmState) -> None:
    arm.close_ramp = False
    arm.gripper_hold_target = 0.0
    arm.filtered_left_normal_force = 0.0
    arm.filtered_right_normal_force = 0.0
    arm.force_threshold_steps = 0
    set_ctrl_clipped(model, data, arm.gripper_actuator, 0.0)


# ---------------------------------------------------------------------------
# Mobile base and vertical spine
# ---------------------------------------------------------------------------


@dataclass
class BaseDriver:
    model: mujoco.MjModel
    data: mujoco.MjData
    base_speed: float
    yaw_speed: float
    spine_speed: float
    forward_axis: str
    speed_scale: list[float]
    control_mode: str = "actuator"
    spine_target_lookahead: float = 0.100
    spine_release_brake_lookahead: float = 0.025
    base_body: int = field(init=False)
    planar_joint_ids: tuple[int, int, int] = field(init=False)
    planar_qpos_addresses: tuple[int, int, int] = field(init=False)
    planar_dof_addresses: tuple[int, int, int] = field(init=False)
    planar_actuator_ids: tuple[int, int, int] = field(init=False)
    spine_joint: int = field(init=False)
    spine_actuator: int = field(init=False)
    spine_qpos_address: int = field(init=False)
    spine_dof_address: int = field(init=False)
    spine_target: float = field(init=False)
    spine_hold_reference: float = field(init=False)
    spine_kp: float = field(init=False)
    spine_kv: float = field(init=False)
    idle_anchor: np.ndarray | None = field(default=None, init=False)
    planar_active: bool = field(default=False, init=False)
    spine_active: bool = field(default=False, init=False)
    spine_was_active: bool = field(default=False, init=False)
    spine_braking: bool = field(default=False, init=False)
    spine_brake_direction: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self.base_body = object_id(self.model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        self.planar_joint_ids = tuple(
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in (BASE_X_JOINT, BASE_Y_JOINT, BASE_YAW_JOINT)
        )
        self.planar_qpos_addresses = tuple(
            int(self.model.jnt_qposadr[joint_id]) for joint_id in self.planar_joint_ids
        )
        self.planar_dof_addresses = tuple(
            int(self.model.jnt_dofadr[joint_id]) for joint_id in self.planar_joint_ids
        )
        self.planar_actuator_ids = tuple(
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in (BASE_X_ACT, BASE_Y_ACT, BASE_YAW_ACT)
        )
        self.spine_joint = object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, SPINE_JOINT)
        self.spine_actuator = object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, SPINE_ACT)
        self.spine_qpos_address = int(self.model.jnt_qposadr[self.spine_joint])
        self.spine_dof_address = int(self.model.jnt_dofadr[self.spine_joint])
        self.spine_target = float(self.data.qpos[self.spine_qpos_address])
        self.spine_hold_reference = self.spine_target
        self.spine_kp = float(self.model.actuator_gainprm[self.spine_actuator, 0])
        self.spine_kv = float(-self.model.actuator_biasprm[self.spine_actuator, 2])
        if self.spine_kp <= 0.0:
            raise RuntimeError("The spine position actuator must have a positive kp")
        set_ctrl_clipped(self.model, self.data, self.spine_actuator, self.spine_target)

    def _update_spine(self, command: float, dt: float) -> None:
        """Drive the spine dynamically without accumulating target backlog.

        Every target is computed from the current measured spine height plus a
        bounded look-ahead. Variable wall-clock loop time is deliberately not
        integrated. The position and velocity states remain fully dynamic.
        """
        del dt
        low, high = self.model.jnt_range[self.spine_joint]
        was_active = bool(self.spine_active)
        self.spine_active = abs(command) > 1e-9
        measured = float(self.data.qpos[self.spine_qpos_address])

        measured_velocity = float(self.data.qvel[self.spine_dof_address])

        if self.spine_active:
            horizon = max(
                float(self.model.opt.timestep),
                float(self.spine_target_lookahead),
            )
            lead = float(
                command * self.spine_speed * horizon * self.speed_scale[0]
            )
            # During a held key, always command a bounded lead relative to the
            # measured spine height. The target can never accumulate farther
            # ahead simply because the key has been held longer.
            self.spine_target = float(np.clip(measured + lead, low, high))
            self.spine_braking = False
            self.spine_brake_direction = 0.0
        elif was_active:
            # Start a velocity-aware braking phase. The braking target is rebuilt
            # from the current measured position and velocity, so it cannot store
            # a backlog. We latch the final hold position as soon as velocity
            # reaches zero, before it can reverse and make the spine visibly move
            # back down after an upward key release.
            if abs(measured_velocity) > 1e-4:
                self.spine_braking = True
                self.spine_brake_direction = float(np.sign(measured_velocity))
                release_lead = measured_velocity * float(self.spine_release_brake_lookahead)
                self.spine_target = float(np.clip(measured + release_lead, low, high))
            else:
                self.spine_braking = False
                self.spine_brake_direction = 0.0
                self.spine_hold_reference = float(np.clip(measured, low, high))
                load_force = float(
                    self.data.qfrc_constraint[self.spine_dof_address]
                    + self.data.qfrc_passive[self.spine_dof_address]
                )
                load_offset = float(np.clip(-load_force / self.spine_kp, -0.010, 0.010))
                self.spine_target = float(np.clip(self.spine_hold_reference + load_offset, low, high))
        elif self.spine_braking:
            direction_velocity = self.spine_brake_direction * measured_velocity
            if direction_velocity > 1e-4:
                release_lead = measured_velocity * float(self.spine_release_brake_lookahead)
                self.spine_target = float(np.clip(measured + release_lead, low, high))
            else:
                # Velocity has reached zero (or tried to reverse). Capture this
                # measured position once and hold it with the position servo.
                self.spine_braking = False
                self.spine_brake_direction = 0.0
                self.spine_hold_reference = float(np.clip(measured, low, high))
                load_force = float(
                    self.data.qfrc_constraint[self.spine_dof_address]
                    + self.data.qfrc_passive[self.spine_dof_address]
                )
                load_offset = float(np.clip(-load_force / self.spine_kp, -0.010, 0.010))
                self.spine_target = float(np.clip(self.spine_hold_reference + load_offset, low, high))
        else:
            # Keep a fixed physical hold reference while compensating the current
            # static weld/contact load. This offset is computed algebraically from
            # generalized force; it is not integrated and cannot accumulate.
            load_force = float(
                self.data.qfrc_constraint[self.spine_dof_address]
                + self.data.qfrc_passive[self.spine_dof_address]
            )
            load_offset = float(np.clip(-load_force / self.spine_kp, -0.010, 0.010))
            self.spine_target = float(np.clip(self.spine_hold_reference + load_offset, low, high))

        self.spine_was_active = was_active

        # Feed-forward gravity/bias compensation reduces steady-state sag while
        # the moderate position servo supplies only tracking/contact correction.
        self.data.qfrc_applied[self.spine_dof_address] = float(
            self.data.qfrc_bias[self.spine_dof_address]
        )
        set_ctrl_clipped(self.model, self.data, self.spine_actuator, self.spine_target)

    def drive(
        self, forward: float, left: float, spine: float, yaw: float, dt: float,
        motion_scale: float = 1.0,
    ) -> None:
        """Drive the virtual planar base.

        ``actuator`` is the default and reproduces the original velocity-servo
        behavior without overwriting contact impulses. ``jointvel`` directly
        injects qvel and is retained only as an optional diagnostic mode.
        """
        # IMPORTANT FRAME DETAIL:
        # base_planar_x and base_planar_y are joints on base_link, therefore
        # their qvel/actuator coordinates are expressed along base_link local
        # +X and +Y. ``forward`` and ``left`` are already in that local frame
        # (screen_to_base_local performed the camera/world -> base conversion).
        # Converting them to world XY here and then assigning those world
        # components to the local joint coordinates rotates the command twice.
        scale = self.speed_scale[0] * max(1.0, float(motion_scale))
        qd_x = float(forward * self.base_speed * scale)
        qd_y = float(left * self.base_speed * scale)
        qd_yaw = float(yaw * self.yaw_speed * scale)

        # Set velocity targets in JOINT coordinates, not world coordinates.
        set_ctrl_clipped(self.model, self.data, self.planar_actuator_ids[0], qd_x)
        set_ctrl_clipped(self.model, self.data, self.planar_actuator_ids[1], qd_y)
        set_ctrl_clipped(self.model, self.data, self.planar_actuator_ids[2], qd_yaw)

        planar_active = abs(forward) + abs(left) + abs(yaw) > 1e-9
        self.planar_active = planar_active
        if planar_active:
            self.idle_anchor = None
            if self.control_mode == "jointvel":
                self.data.qvel[self.planar_dof_addresses[0]] = qd_x
                self.data.qvel[self.planar_dof_addresses[1]] = qd_y
                self.data.qvel[self.planar_dof_addresses[2]] = qd_yaw
            self._update_spine(spine, dt)
            return

        self._update_spine(spine, dt)

        # As in the original package, anchor the virtual planar joints while
        # idle to prevent slow drift from contacts or solver error.
        if self.idle_anchor is None:
            self.idle_anchor = np.array(
                [self.data.qpos[address] for address in self.planar_qpos_addresses],
                dtype=np.float64,
            )
        for qpos_address, dof_address, anchor in zip(
            self.planar_qpos_addresses,
            self.planar_dof_addresses,
            self.idle_anchor,
        ):
            self.data.qpos[qpos_address] = float(anchor)
            self.data.qvel[dof_address] = 0.0

    def post_step_hold(self) -> None:
        """Reapply idle base and spine anchors after ``mj_step``."""
        changed = False

        if not self.planar_active and self.idle_anchor is not None:
            for actuator_id in self.planar_actuator_ids:
                set_ctrl_clipped(self.model, self.data, actuator_id, 0.0)
            for qpos_address, dof_address, anchor in zip(
                self.planar_qpos_addresses,
                self.planar_dof_addresses,
                self.idle_anchor,
            ):
                self.data.qpos[qpos_address] = float(anchor)
                self.data.qvel[dof_address] = 0.0
            changed = True

        # The spine is never kinematically anchored. Keep only its position
        # target active; MuJoCo integrates its acceleration and velocity.
        set_ctrl_clipped(self.model, self.data, self.spine_actuator, self.spine_target)

        if changed:
            mujoco.mj_forward(self.model, self.data)


# ---------------------------------------------------------------------------
# Scene initialization and loop
# ---------------------------------------------------------------------------


def apply_ready_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Apply the collision-free Task 3 start pose above the spoon."""
    for joint_name, value in BASE_READY_QPOS.items():
        joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = float(value)

    spine_joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, SPINE_JOINT)
    spine_low, spine_high = model.jnt_range[spine_joint]
    data.qpos[int(model.jnt_qposadr[spine_joint])] = float(
        np.clip(SPINE_READY_QPOS, spine_low, spine_high)
    )

    for side, values in ARM_READY_QPOS.items():
        for joint_name, value in zip(ARM_SPECS[side]["joints"], values):
            joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            low, high = model.jnt_range[joint_id]
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(np.clip(value, low, high))

    # Initialize both free gripper bases exactly at their flange poses before
    # the first constrained dynamics step.
    mujoco.mj_kinematics(model, data)
    synchronize_all_gripper_attachments(model, data)


def update_target_markers(data: mujoco.MjData, arms: Iterable[ArmState]) -> None:
    for arm in arms:
        arm.target_pos = data.site_xpos[arm.tcp_site].copy()
        if arm.rotate_mode:
            arm.target_quat = mat_to_quat(data.site_xmat[arm.tcp_site].reshape(3, 3))
            arm.translate_lock_quat = arm.target_quat.copy()
        else:
            arm.target_quat = arm.translate_lock_quat.copy()
        sync_target_marker(data, arm)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Keyboard teleoperation for the aligned Mobile FR3 Duo scene")
    parser.add_argument(
        "--xml",
        type=Path,
        default=Path(__file__).resolve().parent / "final_scene_300_beans_mesh.xml",
        help="Coffee-beans task scene XML (default: final_scene_300_beans_mesh.xml)",
    )
    parser.add_argument("--no-viewer", action="store_true", help="run a headless initialization/physics smoke test")
    parser.add_argument("--steps", type=int, default=20, help="steps used by --no-viewer")
    parser.add_argument("--timestep", type=float, default=None, help="override the XML timestep")
    parser.add_argument("--noslip-iterations", type=int, default=None, help="override XML noslip_iterations")
    parser.add_argument(
        "--keep-wheel-collision",
        action="store_true",
        help=(
            "keep wheel/caster ground contacts while using planar-joint drive; "
            "normally disabled exactly as in model.zip because they oppose the virtual base motion"
        ),
    )
    parser.add_argument(
        "--disable-wheel-collision",
        action="store_true",
        help=argparse.SUPPRESS,
    )

    parser.add_argument("--move-speed", type=float, default=0.25, help="nominal arm translation speed in m/s")
    parser.add_argument("--rot-speed-deg", type=float, default=45.0, help="nominal TCP-local angular speed in deg/s")
    parser.add_argument("--ik-damping", type=float, default=0.03)
    parser.add_argument(
        "--rotation-ik-damping",
        type=float,
        default=0.01,
        help="damped-IK regularization used in rotation mode to keep the fingertip TCP fixed",
    )
    parser.add_argument("--joint-vel-limit", type=float, default=2.0, help="norm limit for the 7-D qdot vector in rad/s")
    parser.add_argument(
        "--arm-target-lookahead",
        type=float,
        default=0.030,
        help=(
            "position-target lead in seconds, rebuilt from measured joints; "
            "prevents q_ref accumulation when joints lag"
        ),
    )
    parser.add_argument(
        "--arm-max-target-lead-rad",
        type=float,
        default=0.040,
        help=(
            "maximum per-joint position-target error used to realize IK velocity; "
            "limits servo impulses without accumulating q_ref"
        ),
    )
    parser.add_argument("--orientation-lock-gain", type=float, default=4.0)
    parser.add_argument("--orientation-lock-max-deg", type=float, default=180.0)
    parser.add_argument(
        "--rotation-pivot-gain",
        type=float,
        default=20.0,
        help="position-feedback gain that keeps the lowest-point TCP fixed during rotation",
    )
    parser.add_argument(
        "--rotation-pivot-max-speed",
        type=float,
        default=0.25,
        help="maximum linear correction speed used to hold the rotation pivot, in m/s",
    )
    parser.add_argument("--max-tcp-step", type=float, default=0.0005, help="maximum linear TCP travel per physics step")
    parser.add_argument("--max-rot-step-deg", type=float, default=1.5, help="maximum TCP rotation per physics step")
    parser.add_argument(
        "--arm-frame",
        choices=("base", "tcp", "camera"),
        default="base",
        help=(
            "translation frame: base (default: W/S forward/back, A/D lateral, "
            "U/J world up/down), tcp (all axes local to the tool), or camera"
        ),
    )
    parser.add_argument(
        "--arm-rotation-frame",
        choices=("tcp",),
        default="tcp",
        help="arm rotation is always roll/pitch/yaw about the current TCP local XYZ axes",
    )

    parser.add_argument(
        "--base-control",
        choices=("actuator", "jointvel"),
        default="jointvel",
        help=(
            "actuator (default) preserves MuJoCo collision response and matches "
            "model.zip; jointvel directly overwrites qvel and can defeat contacts"
        ),
    )
    parser.add_argument(
        "--base-speed",
        type=float,
        default=2.0,
        help=(
            "nominal base translation request in m/s before wall-clock compensation; wheel-floor contact is disabled and "
            "the XML velocity actuators allow up to +/-4.0 m/s"
        ),
    )
    parser.add_argument("--base-yaw-speed-deg", type=float, default=120.0)
    parser.add_argument(
        "--spine-speed", "--spine-step",
        dest="spine_speed",
        type=float,
        default=0.18,
        help="dynamic spine target speed in m/s (legacy alias: --spine-step)",
    )
    parser.add_argument(
        "--spine-target-lookahead",
        type=float,
        default=0.100,
        help=(
            "bounded target lead in seconds, measured from the current spine "
            "position instead of accumulated key-press time"
        ),
    )
    parser.add_argument(
        "--spine-release-brake-lookahead",
        type=float,
        default=0.025,
        help=(
            "one-time velocity-based stopping-distance prediction used when the "
            "spine key is released; the resulting hold target is then latched"
        ),
    )
    parser.add_argument(
        "--robot-forward-axis",
        choices=("x", "-x", "y", "-y"),
        default="x",
    )

    parser.add_argument("--gripper-close-rate", type=float, default=1.0)
    parser.add_argument(
        "--gripper-force-stop",
        type=float,
        default=18.0,
        help="diagnostic compatibility option; fixed 0.735 rad closure does not stop on force",
    )
    parser.add_argument(
        "--gripper-force-filter-alpha",
        type=float,
        default=0.20,
        help="normal-force low-pass coefficient in [0,1]",
    )
    parser.add_argument(
        "--gripper-force-hold-steps",
        type=int,
        default=8,
        help="consecutive loops above threshold required before stopping",
    )
    parser.add_argument(
        "--gripper-min-side-force",
        type=float,
        default=0.5,
        help="diagnostic compatibility option; fixed preload does not stop on force",
    )
    parser.add_argument(
        "--gripper-closed-tolerance",
        type=float,
        default=0.005,
        help="actual joint-position tolerance for detecting full closure",
    )
    parser.add_argument(
        "--gripper-hold-overdrive",
        type=float,
        default=0.0,
        help="deprecated compatibility option; stable contact now holds the reached command",
    )
    parser.add_argument(
        "--gripper-ignore-static-contact",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="ignore fixed table/wall contacts in force stopping (default: enabled)",
    )
    parser.add_argument("--loop-hz", type=float, default=500.0)
    parser.add_argument("--render-hz", type=float, default=60.0)
    parser.add_argument(
        "--wall-time-compensation",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "optionally scale base and arm commands when the scene runs slower than real time; "
            "disabled by default for safer collision behavior"
        ),
    )
    parser.add_argument(
        "--max-wall-time-scale",
        type=float,
        default=2.0,
        help="maximum wall/simulation time compensation applied to base and arm motion",
    )
    parser.add_argument(
        "--gripper-time-scale",
        type=float,
        default=8.0,
        help="maximum safe acceleration of the fixed-preload closing ramp relative to simulation time",
    )
    parser.add_argument(
        "--debug-input",
        action="store_true",
        help="print detected held motion keys and nonzero commands",
    )
    parser.add_argument(
        "--keyboard-backend",
        choices=("auto", "pynput", "x11", "glfw", "callback"),
        default="auto",
        help="continuous key-state backend; auto prefers pynput, then X11/Windows; GLFW is explicit only",
    )
    parser.add_argument(
        "--callback-grace",
        type=float,
        default=0.12,
        help="deprecated; continuous keys no longer use a timeout",
    )
    parser.add_argument("--no-ready-pose", action="store_true", help="do not apply the original teleop ready arm pose")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    xml_path = args.xml.expanduser().resolve()
    if not xml_path.is_file():
        raise FileNotFoundError(f"Scene XML does not exist: {xml_path}")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    if args.timestep is not None:
        if args.timestep <= 0.0:
            raise ValueError("--timestep must be positive")
        model.opt.timestep = float(args.timestep)
    if args.noslip_iterations is not None:
        model.opt.noslip_iterations = max(0, int(args.noslip_iterations))
    # Match model.zip: when the virtual planar joints own base motion, wheel
    # and caster contacts are parasitic.  They are unpowered/braked and create
    # large floor friction, making actuator control extremely slow and uneven.
    # Only those wheel/caster geoms are disabled; the base chassis, spine, arms,
    # grippers, walls, tables, spoon, bowl, plate, beans, and floor remain fully
    # collision-enabled.
    planar_drive = args.base_control in ("actuator", "jointvel")
    disable_wheels = args.disable_wheel_collision or (planar_drive and not args.keep_wheel_collision)
    if disable_wheels:
        disabled = disable_wheel_ground_collision(model)
        log(
            f"[base] wheel/caster contacts disabled on {disabled} geoms; "
            "planar drive owns base motion (same policy as model.zip)"
        )

    data = mujoco.MjData(model)
    if not args.no_ready_pose:
        apply_ready_pose(model, data)
    mujoco.mj_forward(model, data)
    collision_counts = validate_robot_environment_contacts(model)
    initial_robot_env_contacts = count_initial_robot_environment_contacts(model, data)

    arms = {
        side: create_arm(model, data, side)
        for side in ("left", "right")
    }
    for arm in arms.values():
        open_gripper(model, data, arm)
        synchronize_arm(model, data, arm)
        align_gripper_collision_cuboids(model, data, arm)
    mujoco.mj_forward(model, data)

    speed_scale = [1.0]
    base_driver = BaseDriver(
        model=model,
        data=data,
        base_speed=float(args.base_speed),
        yaw_speed=math.radians(float(args.base_yaw_speed_deg)),
        spine_speed=float(args.spine_speed),
        spine_target_lookahead=float(args.spine_target_lookahead),
        spine_release_brake_lookahead=float(args.spine_release_brake_lookahead),
        forward_axis=args.robot_forward_axis,
        speed_scale=speed_scale,
        control_mode=str(args.base_control),
    )

    if args.no_viewer:
        for _ in range(max(0, int(args.steps))):
            base_driver.drive(0.0, 0.0, 0.0, 0.0, model.opt.timestep)
            for arm in arms.values():
                hard_hold_arm(model, data, arm)
                align_gripper_collision_cuboids(model, data, arm)
                update_gripper(
                    model,
                    data,
                    arm,
                    model.opt.timestep,
                    float(args.gripper_close_rate),
                    float(args.gripper_force_stop),
                    float(args.gripper_force_filter_alpha),
                    int(args.gripper_force_hold_steps),
                    float(args.gripper_min_side_force),
                    float(args.gripper_closed_tolerance),
                    not bool(args.gripper_ignore_static_contact),
                    hold_overdrive=float(args.gripper_hold_overdrive)
                )
            mujoco.mj_step(model, data)
            base_driver.post_step_hold()
        log(
            f"[smoke] loaded {xml_path.name}: nbody={model.nbody}, "
            f"njnt={model.njnt}, nu={model.nu}, steps={args.steps}"
        )
        return

    keyboard = KeyboardInput(
        callback_grace=max(0.0, float(args.callback_grace)),
        backend=str(args.keyboard_backend),
    )
    keyboard.start()

    log(HELP)
    log(f"[model] {xml_path}")
    log(f"[keyboard] continuous backend: {keyboard.backend_description()}")
    if keyboard._active_backend == "GLFW live key polling":
        log("[keyboard] GLFW motion keys require the MuJoCo viewer to have focus")
    else:
        log("[keyboard] global press/release tracking is active; release a key to stop immediately")
    log("[keyboard] held keys stay active until release; use 7/8/9 and the motion keys")
    log(f"[physics] timestep={model.opt.timestep:g}, noslip_iterations={model.opt.noslip_iterations}")
    log(
        f"[base] control={args.base_control}, nominal speed={args.base_speed:g} m/s, "
        "XML translational ctrlrange=+/-2.0 m/s"
    )
    log(
        f"[motion] wall-time compensation={'on' if args.wall_time_compensation else 'off'}, "
        f"maximum scale={args.max_wall_time_scale:g}x; gripper ramp maximum={args.gripper_time_scale:g}x"
    )
    log("[spine] dynamic position control: PageUp/U raises; PageDown/J lowers")
    log(
        f"[arm] translation_frame={args.arm_frame}, "
        f"rotation_frame={args.arm_rotation_frame}; viewer camera does not affect arm motion"
    )
    log(
        "[collision] robot/environment masks enabled: "
        f"robot={collision_counts['robot']} environment={collision_counts['environment']} "
        f"spoon={collision_counts['spoon']} bowl={collision_counts['bowl']} "
        f"plate={collision_counts['plate']} beans={collision_counts['beans']} "
        f"floor={collision_counts['floor']}; initial robot-env contacts={initial_robot_env_contacts}"
    )
    if args.arm_frame == "base":
        log("[arm] vertical control: U/PageUp = world up; J/PageDown = world down")
    elif args.arm_frame == "tcp":
        log("[arm] TCP-frame warning: at the initial spoon pose local +Z points downward; use J/PageDown to lift")

    log_robot_frames(model, data, base_driver.base_body, base_driver.spine_joint)
    mode = ["right"]
    last_arm_name = ["right"]
    viewer_ref: list = [None]
    exit_requested = [False]

    def selected_arm() -> ArmState:
        return arms[last_arm_name[0]]

    def set_mode(new_mode: str) -> None:
        if new_mode not in ("base", "left", "right"):
            return
        mode[0] = new_mode
        if new_mode in arms:
            last_arm_name[0] = new_mode
        for hold_arm in arms.values():
            hard_hold_arm(model, data, hold_arm)
            synchronize_arm(model, data, hold_arm)
        log(f"[mode] {new_mode}")

    def toggle_rotation(arm: ArmState) -> None:
        arm.rotate_mode = not arm.rotate_mode
        mujoco.mj_kinematics(model, data)
        if arm.rotate_mode:
            # Lock the midpoint of the lowest fingertip surfaces in world space.
            # The Jacobian is evaluated at this body, so angular commands rotate
            # the entire arm/gripper about the held point instead of the flange.
            arm.target_pos = data.site_xpos[arm.tcp_site].copy()
            arm.target_quat = mat_to_quat(data.site_xmat[arm.tcp_site].reshape(3, 3))
        else:
            arm.translate_lock_quat = mat_to_quat(data.site_xmat[arm.tcp_site].reshape(3, 3))
            arm.target_quat = arm.translate_lock_quat.copy()
            arm.target_pos = data.site_xpos[arm.tcp_site].copy()
        sync_target_marker(data, arm)
        state = "ROTATION" if arm.rotate_mode else "TRANSLATION"
        log(f"[{arm.name} arm] {state} mode; pivot=lowest gripper point")

    def bump_speed(factor: float) -> None:
        speed_scale[0] = float(np.clip(speed_scale[0] * factor, 0.25, 30.0))
        log(f"[speed] x{speed_scale[0]:.3f}")

    def handle_discrete_key(key: int) -> None:
        arm = selected_arm()
        if key == glfw.KEY_ESCAPE:
            exit_requested[0] = True
        elif key in (glfw.KEY_7, glfw.KEY_KP_7):
            set_mode("base")
        elif key in (glfw.KEY_8, glfw.KEY_KP_8):
            set_mode("left")
        elif key in (glfw.KEY_9, glfw.KEY_KP_9):
            set_mode("right")
        elif key == glfw.KEY_R:
            if mode[0] == "base":
                log("[key R] select an arm with 8 or 9 before toggling rotation")
            else:
                toggle_rotation(arms[mode[0]])
        elif key == glfw.KEY_G:
            if start_gripper_closing(arm):
                log(f"[{arm.name} gripper] closing")
        elif key in (glfw.KEY_V, glfw.KEY_SPACE):
            open_gripper(model, data, arm)
            log(f"[{arm.name} gripper] opening")
        elif key in (glfw.KEY_MINUS, glfw.KEY_KP_SUBTRACT):
            bump_speed(1.0 / 1.5)
        elif key in (glfw.KEY_EQUAL, glfw.KEY_KP_ADD):
            bump_speed(1.5)
        elif key == glfw.KEY_C:
            synchronize_arm(model, data, arm)
            log(f"[{arm.name} arm] target synchronized")
        elif key == glfw.KEY_B:
            dump_contacts(model, data)
        elif key == glfw.KEY_N and viewer_ref[0] is not None:
            group = 3
            viewer_ref[0].opt.geomgroup[group] = 0 if viewer_ref[0].opt.geomgroup[group] else 1
            log(
                f"[viewer] collision group {group} "
                f"{'shown' if viewer_ref[0].opt.geomgroup[group] else 'hidden'}"
            )

    loop_period = 0.0 if args.loop_hz <= 0 else 1.0 / float(args.loop_hz)
    render_period = 0.0 if args.render_hz <= 0 else 1.0 / float(args.render_hz)
    last_time = time.perf_counter()
    next_render = last_time
    previous_debug_keys: set[int] = set()

    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard.key_callback) as viewer:
        viewer_ref[0] = viewer
        viewer.opt.geomgroup[0] = 1
        viewer.opt.geomgroup[1] = 1
        viewer.opt.geomgroup[3] = 0
        viewer.cam.lookat[:] = data.xpos[base_driver.base_body] + np.array([0.0, 0.0, 0.9])
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -25.0

        viewer_window = getattr(viewer, "_window", None)
        if keyboard._active_backend == "GLFW live key polling" and viewer_window is None:
            raise RuntimeError(
                "MuJoCo viewer window handle is unavailable, so GLFW cannot poll held keys. "
                "Install an optional backend with `pip install pynput python-xlib`, or run "
                "with `--keyboard-backend pynput`/`x11`."
            )
        if keyboard._active_backend == "GLFW live key polling":
            log("[keyboard] using dependency-free GLFW polling; click the viewer window to give it focus")

        while viewer.is_running() and not exit_requested[0]:
            cycle_start = time.perf_counter()
            dt = min(max(cycle_start - last_time, model.opt.timestep), 1.0 / 60.0)
            last_time = cycle_start
            if args.wall_time_compensation:
                motion_time_scale = float(np.clip(
                    dt / max(float(model.opt.timestep), 1e-9),
                    1.0,
                    max(1.0, float(args.max_wall_time_scale)),
                ))
            else:
                motion_time_scale = 1.0
            gripper_dt = min(
                dt,
                float(model.opt.timestep) * max(1.0, float(args.gripper_time_scale)),
            )

            for key in keyboard.drain():
                handle_discrete_key(key)

            current_arm = arms[mode[0]] if mode[0] in arms else selected_arm()
            viewer_window = getattr(viewer, "_window", viewer_window)
            base_cmd, key_twist, translation_active, rotation_active = keyboard.poll(
                viewer_window,
                mode[0],
                current_arm.rotate_mode,
                float(args.move_speed) * speed_scale[0],
                math.radians(float(args.rot_speed_deg)) * speed_scale[0],
            )

            debug_key_changed = False
            if args.debug_input:
                debug_keys = keyboard._down_keys(viewer_window)
                debug_key_changed = debug_keys != previous_debug_keys
                if debug_key_changed:
                    key_names = [glfw.get_key_name(key, 0) or str(key) for key in sorted(debug_keys)]
                    base_q = np.array(
                        [data.qpos[a] for a in base_driver.planar_qpos_addresses],
                        dtype=np.float64,
                    )
                    base_ctrl = np.array(
                        [data.ctrl[a] for a in base_driver.planar_actuator_ids],
                        dtype=np.float64,
                    )
                    spine_q = float(data.qpos[int(model.jnt_qposadr[base_driver.spine_joint])])
                    spine_ctrl = float(data.ctrl[base_driver.spine_actuator])
                    log(
                        f"[input] held={key_names} mode={mode[0]} "
                        f"base={np.round(base_cmd, 3)} twist={np.round(key_twist, 3)} "
                        f"base_q={np.round(base_q, 4)} base_ctrl={np.round(base_ctrl, 3)} "
                        f"spine_cmd={base_cmd[2]:+.1f} "
                        f"spine_q={spine_q:.6f} spine_ctrl(before)={spine_ctrl:.6f}"
                    )
                    previous_debug_keys = set(debug_keys)

            if mode[0] == "base":
                # Keyboard base arrows are screen-relative. Convert them to
                # robot local forward/left before BaseDriver maps to world X/Y.
                if abs(base_cmd[0]) > 0.0 or abs(base_cmd[1]) > 0.0:
                    local_forward, local_left = screen_to_base_local(
                        viewer.cam,
                        -base_cmd[1],
                        base_cmd[0],
                        data,
                        base_driver.base_body,
                        args.robot_forward_axis,
                    )
                    base_cmd[0] = local_forward
                    base_cmd[1] = local_left
                base_driver.drive(
                    float(base_cmd[0]),
                    float(base_cmd[1]),
                    float(base_cmd[2]),
                    float(base_cmd[3]),
                    dt,
                    motion_time_scale,
                )
                for arm in arms.values():
                    hard_hold_arm(model, data, arm)
            else:
                base_driver.drive(0.0, 0.0, 0.0, 0.0, dt, motion_time_scale)
                arm = arms[mode[0]]
                operator_twist = key_twist.copy()
                twist = map_arm_operator_twist_to_world(
                    viewer.cam,
                    data,
                    arm.tcp_body,
                    base_driver.base_body,
                    operator_twist,
                    str(args.arm_frame),
                    str(args.arm_rotation_frame),
                    str(args.robot_forward_axis),
                )

                # The complete 100/300-bean scene often advances much slower
                # than real time. Compensate command velocity (not physics or
                # contact stiffness) so held keys produce practical wall-clock
                # motion. The per-step clamp below remains the safety limit.
                twist *= motion_time_scale
                command_active = float(np.linalg.norm(twist)) > 1e-8
                if args.debug_input and debug_key_changed and command_active:
                    tcp_rotation = data.xmat[arm.tcp_body].reshape(3, 3)
                    log(
                        "[arm-map] "
                        f"translation_frame={args.arm_frame} rotation_frame={args.arm_rotation_frame} "
                        f"tcp_X={np.round(tcp_rotation[:, 0], 4)} "
                        f"tcp_Y={np.round(tcp_rotation[:, 1], 4)} "
                        f"tcp_Z={np.round(tcp_rotation[:, 2], 4)} "
                        f"local_twist={np.round(operator_twist, 4)} "
                        f"world_twist={np.round(twist, 4)}"
                    )
                if arm.rotate_mode and rotation_active:
                    twist[:3] += rotation_pivot_linear_correction(
                        data,
                        arm,
                        float(args.rotation_pivot_gain),
                        float(args.rotation_pivot_max_speed),
                    )
                    command_active = float(np.linalg.norm(twist)) > 1e-8

                if (
                    translation_active
                    and not rotation_active
                    and args.orientation_lock_gain > 0.0
                ):
                    error = rotation_error(
                        arm.translate_lock_quat,
                        data.site_xmat[arm.tcp_site].reshape(3, 3),
                    )
                    max_correction = math.radians(float(args.orientation_lock_max_deg))
                    twist[3:] += np.clip(
                        float(args.orientation_lock_gain) * error,
                        -max_correction,
                        max_correction,
                    )
                    command_active = float(np.linalg.norm(twist)) > 1e-8

                operator_command_active = command_active
                wall_contacts = arm_static_wall_contacts(model, data, arm)
                if arm.wall_contact_active:
                    # Collision latch: keep resetting the position reference to
                    # measured q and ignore held motion keys. The latch is only
                    # released after the operator releases the key and contact
                    # has cleared. This prevents repeated push/release cycles.
                    synchronize_arm(model, data, arm)
                    hard_hold_arm(model, data, arm)
                    arm.was_command_active = False
                    command_active = False
                    if not operator_command_active and not wall_contacts:
                        log(f"[collision-guard] {arm.name} arm wall latch cleared")
                        arm.wall_contact_active = False
                elif wall_contacts:
                    # Reset the target to the measured configuration immediately.
                    # This removes accumulated position error (controller wind-up)
                    # and stops q_ref integration while a static wall constrains the arm.
                    log(
                        f"[collision-guard] {arm.name} arm contacted a static wall; "
                        f"release the motion key before commanding again: "
                        f"{describe_wall_contacts(model, wall_contacts)}"
                    )
                    synchronize_arm(model, data, arm)
                    hard_hold_arm(model, data, arm)
                    arm.wall_contact_active = True
                    arm.was_command_active = False
                    command_active = False

                if command_active:
                    twist = clamp_tcp_twist(
                        model,
                        twist,
                        float(args.max_tcp_step),
                        math.radians(float(args.max_rot_step_deg)),
                    )
                    active_ik_damping = (
                        float(args.rotation_ik_damping)
                        if arm.rotate_mode and rotation_active
                        else float(args.ik_damping)
                    )
                    apply_twist_ik(
                        model,
                        data,
                        arm,
                        twist,
                        active_ik_damping,
                        float(args.joint_vel_limit),
                        float(args.arm_target_lookahead),
                        float(args.arm_max_target_lead_rad),
                    )
                    arm.was_command_active = True
                else:
                    if arm.was_command_active:
                        synchronize_arm(model, data, arm)
                    hard_hold_arm(model, data, arm)
                    arm.was_command_active = False

                for other_name, other_arm in arms.items():
                    if other_name != arm.name:
                        hard_hold_arm(model, data, other_arm)

            for arm in arms.values():
                align_gripper_collision_cuboids(model, data, arm)
                update_gripper(
                    model,
                    data,
                    arm,
                    gripper_dt,
                    float(args.gripper_close_rate),
                    float(args.gripper_force_stop),
                    float(args.gripper_force_filter_alpha),
                    int(args.gripper_force_hold_steps),
                    float(args.gripper_min_side_force),
                    float(args.gripper_closed_tolerance),
                    not bool(args.gripper_ignore_static_contact),
                    hold_overdrive=float(args.gripper_hold_overdrive)
                )

            mujoco.mj_step(model, data)
            if args.debug_input and abs(float(base_cmd[2])) > 1e-9:
                spine_q_after = float(data.qpos[base_driver.spine_qpos_address])
                spine_v_after = float(data.qvel[base_driver.spine_dof_address])
                spine_ctrl_after = float(data.ctrl[base_driver.spine_actuator])
                log(
                    f"[spine post-step] command={base_cmd[2]:+.1f} "
                    f"q={spine_q_after:.6f} v={spine_v_after:.6f} "
                    f"ctrl={spine_ctrl_after:.6f}"
                )
            # Catch contacts created by this physics step. This is the critical
            # anti-windup path: q_ref is reset before the next control cycle can
            # continue integrating into an immovable wall.
            if mode[0] in arms:
                active_arm = arms[mode[0]]
                wall_contacts_after = arm_static_wall_contacts(model, data, active_arm)
                if wall_contacts_after and not active_arm.wall_contact_active:
                    log(
                        f"[collision-guard] {active_arm.name} arm hit a static wall; "
                        f"resetting q_ref: "
                        f"{describe_wall_contacts(model, wall_contacts_after)}"
                    )
                    active_arm.wall_contact_active = True
                if active_arm.wall_contact_active:
                    # Keep q_ref equal to the measured joints after every step
                    # while latched, even if compliance briefly clears contact.
                    synchronize_arm(model, data, active_arm)
                    hard_hold_arm(model, data, active_arm)

            base_driver.post_step_hold()
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            update_target_markers(data, arms.values())

            if render_period <= 0.0 or time.perf_counter() >= next_render:
                viewer.sync()
                next_render = time.perf_counter() + render_period

            remaining = loop_period - (time.perf_counter() - cycle_start)
            if remaining > 0.0:
                time.sleep(remaining)

    # Ensure no listener or stale key state survives viewer shutdown.
    keyboard.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
