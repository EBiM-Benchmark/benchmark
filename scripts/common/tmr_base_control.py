# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

"""Keyboard and wheel-target helpers for the diagonal TMR steer-drive base."""

import math
from dataclasses import dataclass

import torch

WHEEL_RADIUS_M = 0.05
LINEAR_SPEED_MPS = 0.5
ANGULAR_SPEED_RADPS = 1.2
MAX_WHEEL_SPEED_RADPS = 18.0
STOP_EPS = 1.0e-4
STEERING_FULL_SPEED_ERROR_RAD = math.radians(8.0)
STEERING_ZERO_SPEED_ERROR_RAD = math.radians(35.0)
HEADING_HOLD_KP = 2.0
HEADING_HOLD_KD = 0.35
MAX_HEADING_COMP_RADPS = 0.8


@dataclass(frozen=True)
class DriveModule:
    steer_joint: str
    drive_joint: str
    x: float
    y: float


# Body-frame locations from the URDF. ROS convention: +x forward, +y left.
DRIVE_MODULES = (
    DriveModule("tmrv0_2_joint_0", "tmrv0_2_joint_1", 0.3, -0.2),
    DriveModule("tmrv0_2_joint_2", "tmrv0_2_joint_3", -0.3, 0.2),
)


def get_keyboard_twist(pressed_keys: set[str]) -> tuple[float, float, float]:
    """Map pressed keys to body-frame (vx, vy, wz)."""
    vx = 0.0
    vy = 0.0
    wz = 0.0

    if "w" in pressed_keys:
        vx += LINEAR_SPEED_MPS
    if "s" in pressed_keys:
        vx -= LINEAR_SPEED_MPS
    if "a" in pressed_keys:
        vy += LINEAR_SPEED_MPS
    if "d" in pressed_keys:
        vy -= LINEAR_SPEED_MPS
    if "q" in pressed_keys or "left" in pressed_keys:
        wz += ANGULAR_SPEED_RADPS
    if "e" in pressed_keys or "right" in pressed_keys:
        wz -= ANGULAR_SPEED_RADPS

    return vx, vy, wz


def find_drive_joint_ids(
    joint_names: list[str],
) -> tuple[list[int], list[int]]:
    """Return steering and wheel-spin joint ids in DRIVE_MODULES order."""
    name_to_id = {name: idx for idx, name in enumerate(joint_names)}
    missing = [
        joint_name
        for module in DRIVE_MODULES
        for joint_name in (module.steer_joint, module.drive_joint)
        if joint_name not in name_to_id
    ]
    if missing:
        raise RuntimeError(f"Missing TMR base joints: {missing}")

    steering_ids = [name_to_id[module.steer_joint] for module in DRIVE_MODULES]
    drive_ids = [name_to_id[module.drive_joint] for module in DRIVE_MODULES]
    return steering_ids, drive_ids


def compute_drive_targets(
    robot,
    steering_ids: list[int],
    vx: float,
    vy: float,
    wz: float,
    *,
    num_envs: int,
    device: str,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Convert body twist into steering-position and wheel-velocity targets."""
    steering_targets = torch.zeros(
        (num_envs, len(DRIVE_MODULES)), device=device
    )
    drive_targets = torch.zeros((num_envs, len(DRIVE_MODULES)), device=device)

    wheel_vectors = []
    max_speed_mps = 0.0
    for module in DRIVE_MODULES:
        wheel_vx = vx - wz * module.y
        wheel_vy = vy + wz * module.x
        speed_mps = math.hypot(wheel_vx, wheel_vy)
        wheel_vectors.append((wheel_vx, wheel_vy, speed_mps))
        max_speed_mps = max(max_speed_mps, speed_mps)

    max_speed_mps_allowed = MAX_WHEEL_SPEED_RADPS * WHEEL_RADIUS_M
    speed_scale = 1.0
    if max_speed_mps > max_speed_mps_allowed:
        speed_scale = max_speed_mps_allowed / max_speed_mps

    for module_index, (wheel_vx, wheel_vy, speed_mps) in enumerate(
        wheel_vectors
    ):
        wheel_vx *= speed_scale
        wheel_vy *= speed_scale
        speed_mps *= speed_scale
        current_angle = robot.data.joint_pos[:, steering_ids[module_index]]

        if speed_mps < STOP_EPS:
            steering_targets[:, module_index] = current_angle
            continue

        raw_target = torch.full_like(
            current_angle, math.atan2(wheel_vy, wheel_vx)
        )
        direct_delta = _wrap_to_pi(raw_target - current_angle)
        flipped_delta = _wrap_to_pi(raw_target + math.pi - current_angle)
        use_flipped = torch.abs(flipped_delta) < torch.abs(direct_delta)
        steering_delta = torch.where(use_flipped, flipped_delta, direct_delta)

        steering_targets[:, module_index] = current_angle + steering_delta

        wheel_speed = torch.full_like(
            current_angle, speed_mps / WHEEL_RADIUS_M
        )
        wheel_speed *= _steering_alignment_scale(torch.abs(steering_delta))
        drive_targets[:, module_index] = torch.where(
            use_flipped, -wheel_speed, wheel_speed
        )

    return steering_targets, drive_targets


def compensate_yaw_rate(
    robot,
    vx: float,
    vy: float,
    wz: float,
    desired_yaw: float,
    *,
    manual_rotation: bool,
) -> tuple[float, float]:
    """Hold heading during translation and reset hold heading otherwise."""
    current_yaw = get_root_yaw(robot)
    if manual_rotation or math.hypot(vx, vy) < STOP_EPS:
        return wz, current_yaw

    yaw_error = _wrap_to_pi_scalar(desired_yaw - current_yaw)
    yaw_rate = get_root_yaw_rate(robot)
    compensation = HEADING_HOLD_KP * yaw_error - HEADING_HOLD_KD * yaw_rate
    compensation = max(
        -MAX_HEADING_COMP_RADPS,
        min(MAX_HEADING_COMP_RADPS, compensation),
    )
    return wz + compensation, desired_yaw


def get_root_yaw(robot, env_id: int = 0) -> float:
    """Return root yaw from Isaac Lab's wxyz quaternion."""
    quat = robot.data.root_quat_w[env_id]
    w, x, y, z = [value.item() for value in quat]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_root_yaw_rate(robot, env_id: int = 0) -> float:
    """Return world-frame yaw rate when available."""
    if hasattr(robot.data, "root_ang_vel_w"):
        return robot.data.root_ang_vel_w[env_id, 2].item()
    if hasattr(robot.data, "root_ang_vel_b"):
        return robot.data.root_ang_vel_b[env_id, 2].item()
    return 0.0


def _wrap_to_pi(angle: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angle), torch.cos(angle))


def _wrap_to_pi_scalar(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _steering_alignment_scale(error: torch.Tensor) -> torch.Tensor:
    """Fade wheel speed down while the steering module is still turning."""
    scale = (STEERING_ZERO_SPEED_ERROR_RAD - error) / (
        STEERING_ZERO_SPEED_ERROR_RAD - STEERING_FULL_SPEED_ERROR_RAD
    )
    return torch.clamp(scale, min=0.0, max=1.0)
