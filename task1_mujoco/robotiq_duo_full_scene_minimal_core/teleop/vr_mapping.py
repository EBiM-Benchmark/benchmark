"""VR hand/clutch state and coordinate-frame mappings.

Frames: the VR standing space is x-right, y-up, z-backward (OpenXR / OpenVR
convention). Mirror teleop ("--facing front") maps hands to what the operator
SEES on the monitor; "--facing behind" maps into the robot's own heading
frame.
"""

from __future__ import annotations

import math

import numpy as np

import mujoco

from .mjutil import planar_body_axis


class HandState:
    """One controller's pose + inputs, as delivered by a VR backend."""

    __slots__ = (
        "pos",
        "rot",
        "trigger",
        "grip",
        "stick",
        "stick_click",
        "a",
        "b",
        "valid",
    )

    def __init__(self):
        self.pos = np.zeros(3)
        self.rot = np.eye(3)
        self.trigger = 0.0
        self.grip = 0.0
        self.stick = np.zeros(2)
        self.stick_click = False
        self.a = False
        self.b = False
        self.valid = False


class ClutchState:
    """Clutch anchor: controller and TCP poses captured at grip-engage, plus
    the VR->world map frozen at that moment."""

    __slots__ = (
        "engaged",
        "ctrl_pos",
        "ctrl_rot",
        "tcp_pos",
        "tcp_rot",
        "map",
    )

    def __init__(self):
        self.engaged = False
        self.ctrl_pos = np.zeros(3)
        self.ctrl_rot = np.eye(3)
        self.tcp_pos = np.zeros(3)
        self.tcp_rot = np.eye(3)
        self.map = np.eye(3)


def vr_to_world_map(data: mujoco.MjData, base_body: int | None, forward_axis: str) -> np.ndarray:
    """VR standing frame -> world, assuming the operator faces the same way
    as the robot base (--facing behind)."""
    fwd = planar_body_axis(data, base_body, forward_axis)
    left = np.cross(np.array([0.0, 0.0, 1.0]), fwd)
    up = np.array([0.0, 0.0, 1.0])
    # columns: where VR x / y / z land in the world
    return np.column_stack((-left, up, -fwd))


def vr_to_screen_map(cam) -> np.ndarray:
    """VR frame -> world, matched to what the operator SEES on the screen
    (mirror teleop): hand forward = into the screen, hand up = world up,
    hand right = the mirrored arm's right (screen LEFT — user-tested; the
    screen-right variant read as inverted)."""
    az = math.radians(float(cam.azimuth))
    fwd_h = np.array([-math.cos(az), -math.sin(az), 0.0])  # into the screen
    right_h = np.array([fwd_h[1], -fwd_h[0], 0.0])
    up = np.array([0.0, 0.0, 1.0])
    # columns: VR x -> screen left (mirror), VR y -> up, VR z (backward) -> -fwd
    return np.column_stack((-right_h, up, -fwd_h))


def screen_to_base_local(
    cam,
    sx: float,
    sy: float,
    data: mujoco.MjData,
    base_body: int | None,
    forward_axis: str,
) -> tuple[float, float]:
    """Base stick in plain screen axes (not mirrored like the arms):
    stick left = base moves screen-left, stick up = into the screen.
    Returns the (forward, left) command in the robot's heading frame."""
    # MuJoCo free camera (measured via mjv_updateScene's mjvGLCamera):
    # into-screen = [+cos(az), +sin(az)], screen-right = [+sin(az), -cos(az)]
    az = math.radians(float(cam.azimuth))
    fwd_h = np.array([math.cos(az), math.sin(az), 0.0])  # into the screen
    right_h = np.array([math.sin(az), -math.cos(az), 0.0])  # screen right
    world = right_h * sx + fwd_h * sy
    fwd = planar_body_axis(data, base_body, forward_axis)
    left_axis = np.cross(np.array([0.0, 0.0, 1.0]), fwd)
    return float(world @ fwd), float(world @ left_axis)
