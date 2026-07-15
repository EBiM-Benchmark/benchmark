#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Plain Isaac Sim 5.1 Task 3 dual-arm RMPflow keyboard demo.

Run this entry point in the Isaac Sim 5.1 container.  It intentionally does
not import or initialize Isaac Lab; the Isaac Lab Task 3 teleop remains a
separate executable.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TASK3_ROOT = Path(__file__).resolve().parents[1]
SCENES_DIR = REPO_ROOT / "scripts" / "scenes"
if str(SCENES_DIR) not in sys.path:
    sys.path.insert(0, str(SCENES_DIR))

import scene_robot_room_keyboard as room_scene  # noqa: E402

ROBOT_PRIM_PATH = "/World/Robot"
CONFIG_DIR = TASK3_ROOT / "config"
LULA_CONFIG_DIR = CONFIG_DIR / "task3_teleop"
RMPFLOW_CONFIG_DIR = CONFIG_DIR / "task3_rmpflow"
LULA_URDF_PATH = LULA_CONFIG_DIR / "mobile_fr3_duo_v0_2_franka_hand.urdf"
RMPFLOW_MAX_SUBSTEP_SIZE = 0.0034
FULL_GUI_EXPERIENCE = "/isaac-sim/apps/isaacsim.exp.full.kit"
HEADLESS_EXPERIENCE = "/isaac-sim/apps/isaacsim.exp.base.kit"
WHEEL_RADIUS_M = 0.05
MAX_WHEEL_SPEED_RADPS = 18.0
STOP_EPS = 1.0e-4
STEERING_FULL_SPEED_ERROR_RAD = math.radians(8.0)
STEERING_ZERO_SPEED_ERROR_RAD = math.radians(35.0)
MIN_STEERING_ALIGNMENT_SCALE = 0.2
HEADING_HOLD_KP = 2.0
HEADING_HOLD_KD = 0.35
MAX_HEADING_COMP_RADPS = 0.8
DRIVE_MODULES = (
    ("tmrv0_2_joint_0", "tmrv0_2_joint_1", 0.3, -0.2),
    ("tmrv0_2_joint_2", "tmrv0_2_joint_3", -0.3, 0.2),
)

ARM_CONFIGS = {
    "left": {
        "description": LULA_CONFIG_DIR / "left_arm_description.yaml",
        "rmpflow": RMPFLOW_CONFIG_DIR / "left_arm_rmpflow_config.yaml",
        "end_effector_frame": "left_fr3v2_hand_tcp",
        "joints": tuple(f"left_fr3v2_joint{i}" for i in range(1, 8)),
        "fingers": (
            "left_fr3v2_finger_joint1",
            "left_fr3v2_finger_joint2",
        ),
    },
    "right": {
        "description": LULA_CONFIG_DIR / "right_arm_description.yaml",
        "rmpflow": RMPFLOW_CONFIG_DIR / "right_arm_rmpflow_config.yaml",
        "end_effector_frame": "right_fr3v2_hand_tcp",
        "joints": tuple(f"right_fr3v2_joint{i}" for i in range(1, 8)),
        "fingers": (
            "right_fr3v2_finger_joint1",
            "right_fr3v2_finger_joint2",
        ),
    },
}

CONTROL_HELP = """\
+---------------- RMPFLOW CONTROL PANEL ----------------+
| Isaac Sim window must have keyboard focus.             |
|                                                         |
| LEFT ARM                              RIGHT ARM         |
| Move: [W/S] fwd/back                  [O/L] fwd/back   |
|       [A/D] left/right                [K/;] left/right |
|       [Q/E] down/up                   [I/P] down/up    |
| Rotate: [Z/X] roll +/-                [N/M] roll +/-   |
|         [T/G] pitch +/-               [U/J] pitch +/-  |
|         [C/V] yaw +/-                 [,/.] yaw +/-    |
| Grip: [F] toggle                      ['] toggle        |
|                                                         |
| MOBILE BASE -- hold Shift (arm keys are suppressed):    |
| [SHIFT + H] Forward       [SHIFT + N] Backward          |
| [SHIFT + B] Left          [SHIFT + M] Right             |
| [SHIFT + G] Rotate CCW    [SHIFT + J] Rotate CW         |
|                                                         |
| [R] Reset both arm targets to the startup pose.         |
+---------------------------------------------------------+
"""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Task 3 dual-arm RMPflow in the plain Isaac Sim 5.1 container "
            "(not Isaac Lab)."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--room-usd",
        type=Path,
        default=room_scene.asset_path("robot_room.usd"),
    )
    parser.add_argument(
        "--robot-usd",
        type=Path,
        default=room_scene.franka_urdf_path(
            "mobile_fr3_duo_v0_2_franka_hand.usd"
        ),
    )
    parser.add_argument("--robot-x", type=float, default=None)
    parser.add_argument("--robot-y", type=float, default=None)
    parser.add_argument("--robot-z", type=float, default=None)
    parser.add_argument("--robot-yaw", type=float, default=None)
    parser.add_argument(
        "--head-placement",
        type=room_scene.head_placement_arg,
        default="random",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--arm-keyboard-teleop",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable dual-arm RMPflow control; always off headless.",
    )
    parser.add_argument("--physics-hz", type=float, default=240.0)
    parser.add_argument("--render-hz", type=float, default=60.0)
    parser.add_argument("--linear-speed", type=float, default=0.18)
    parser.add_argument("--angular-speed-deg", type=float, default=60.0)
    parser.add_argument("--base-linear-speed", type=float, default=0.25)
    parser.add_argument("--base-angular-speed", type=float, default=0.75)
    parser.add_argument(
        "--base-heading-hold",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compensate yaw drift while translating the mobile base.",
    )
    parser.add_argument(
        "--base-diagnostics-interval",
        type=float,
        default=0.0,
        help=(
            "Print mobile-base command tracking every N seconds; "
            "zero disables diagnostics."
        ),
    )
    return parser


def arm_keyboard_enabled(args: argparse.Namespace) -> bool:
    return bool(args.arm_keyboard_teleop and not args.headless)


def experience_path(args: argparse.Namespace) -> str:
    """Use Isaac Sim's complete GUI for interactive teleoperation."""
    return HEADLESS_EXPERIENCE if args.headless else FULL_GUI_EXPERIENCE


def base_twist_from_held_key_names(held_keys) -> tuple[float, float, float]:
    """Return unit body-frame twist only while Shift is held."""
    held = {str(key).lower() for key in held_keys}
    if "shift" not in held:
        return 0.0, 0.0, 0.0
    return (
        float(("h" in held) - ("n" in held)),
        float(("b" in held) - ("m" in held)),
        float(("g" in held) - ("j" in held)),
    )


def format_base_diagnostic_line(
    *,
    elapsed_s,
    yaw_rad,
    body_command,
    displacement,
    steering_command,
    steering_actual,
    wheel_command,
    wheel_actual,
    wheel_effort=None,
) -> str:
    def values(items) -> str:
        return ", ".join(f"{float(item):.3f}" for item in items)

    line = (
        f"base-diag t={float(elapsed_s):.2f}s "
        f"yaw={float(yaw_rad):.3f} "
        f"twist=[{values(body_command)}] "
        f"delta_xyz=[{values(displacement)}] "
        f"steer cmd/actual=[{values(steering_command)}]/"
        f"[{values(steering_actual)}] "
        f"wheel cmd/actual=[{values(wheel_command)}]/"
        f"[{values(wheel_actual)}]"
    )
    if wheel_effort is not None:
        line += f" effort=[{values(wheel_effort)}]"
    return line


def quaternion_multiply(first, second) -> np.ndarray:
    aw, ax, ay, az = (float(value) for value in first)
    bw, bx, by, bz = (float(value) for value in second)
    return np.array(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        dtype=np.float64,
    )


def _quaternion_conjugate(quaternion) -> np.ndarray:
    w, x, y, z = (float(value) for value in quaternion)
    return np.array((w, -x, -y, -z), dtype=np.float64)


def _rotate_vector(quaternion, vector) -> np.ndarray:
    pure = np.array((0.0, *vector), dtype=np.float64)
    return quaternion_multiply(
        quaternion_multiply(quaternion, pure),
        _quaternion_conjugate(quaternion),
    )[1:]


def compose_world_pose(
    parent_position,
    parent_orientation_wxyz,
    child_position,
    child_orientation_wxyz,
) -> tuple[np.ndarray, np.ndarray]:
    parent_quaternion = np.asarray(parent_orientation_wxyz, dtype=np.float64)
    parent_quaternion /= np.linalg.norm(parent_quaternion)
    position = np.asarray(parent_position, dtype=np.float64) + _rotate_vector(
        parent_quaternion, child_position
    )
    orientation = quaternion_multiply(
        parent_quaternion, child_orientation_wxyz
    )
    orientation /= np.linalg.norm(orientation)
    return position, orientation


def _axis_angle_quaternion(axis, angle: float) -> np.ndarray:
    half = 0.5 * float(angle)
    return np.array(
        (math.cos(half), *(math.sin(half) * np.asarray(axis, dtype=float)))
    )


def merge_policy_actions(actions) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    positions: dict[int, float] = {}
    velocities: dict[int, float] = {}
    for action in actions:
        if action is None or action.joint_positions is None:
            continue
        action_velocities = action.joint_velocities
        if action_velocities is None:
            action_velocities = np.zeros_like(action.joint_positions)
        for index, position, velocity in zip(
            action.joint_indices,
            action.joint_positions,
            action_velocities,
        ):
            positions[int(index)] = float(position)
            velocities[int(index)] = float(velocity)
    indices = np.array(sorted(positions), dtype=np.int64)
    return (
        np.array([positions[index] for index in indices], dtype=np.float32),
        np.array([velocities[index] for index in indices], dtype=np.float32),
        indices,
    )


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion_wxyz) -> float:
    """Return world yaw from an Isaac Sim wxyz quaternion."""
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def compensate_heading_yaw_rate(
    *,
    current_yaw: float,
    current_yaw_rate: float,
    vx: float,
    vy: float,
    wz: float,
    desired_yaw: float,
    manual_rotation: bool,
) -> tuple[float, float]:
    """Match Isaac Lab's heading hold for uncommanded translation drift."""
    if manual_rotation or math.hypot(vx, vy) < STOP_EPS:
        return wz, current_yaw
    yaw_error = _wrap_to_pi(desired_yaw - current_yaw)
    compensation = (
        HEADING_HOLD_KP * yaw_error - HEADING_HOLD_KD * current_yaw_rate
    )
    compensation = max(
        -MAX_HEADING_COMP_RADPS, min(MAX_HEADING_COMP_RADPS, compensation)
    )
    return wz + compensation, desired_yaw


def _steering_alignment_scale(error: float) -> float:
    return min(
        max(
            (STEERING_ZERO_SPEED_ERROR_RAD - error)
            / (STEERING_ZERO_SPEED_ERROR_RAD - STEERING_FULL_SPEED_ERROR_RAD),
            MIN_STEERING_ALIGNMENT_SCALE,
        ),
        1.0,
    )


def _physx_continuous_target(current: float, delta: float) -> float:
    """Keep the nearest equivalent target inside PhysX's continuous range."""
    target = current + delta
    period = 2.0 * math.pi
    while target >= period:
        target -= period
    while target <= -period:
        target += period
    return target


def compute_base_drive_targets(
    joint_positions: np.ndarray,
    steering_indices: tuple[int, int],
    vx: float,
    vy: float,
    wz: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a body twist to steering angles and wheel angular velocities."""
    wheel_vectors = [
        (vx - wz * y, vy + wz * x) for _steer, _drive, x, y in DRIVE_MODULES
    ]
    max_speed = max(
        (math.hypot(*vector) for vector in wheel_vectors), default=0.0
    )
    max_speed_allowed = MAX_WHEEL_SPEED_RADPS * WHEEL_RADIUS_M
    scale = min(1.0, max_speed_allowed / max_speed) if max_speed else 1.0
    steering_targets = np.zeros(len(DRIVE_MODULES), dtype=np.float32)
    drive_targets = np.zeros(len(DRIVE_MODULES), dtype=np.float32)
    unscaled_drive_targets = np.zeros(len(DRIVE_MODULES), dtype=np.float32)
    alignment_scales = np.ones(len(DRIVE_MODULES), dtype=np.float32)
    for module_index, (wheel_vx, wheel_vy) in enumerate(wheel_vectors):
        wheel_vx *= scale
        wheel_vy *= scale
        speed = math.hypot(wheel_vx, wheel_vy)
        current = float(joint_positions[steering_indices[module_index]])
        if speed < STOP_EPS:
            steering_targets[module_index] = current
            continue
        target_angle = math.atan2(wheel_vy, wheel_vx)
        direct_delta = _wrap_to_pi(target_angle - current)
        flipped_delta = _wrap_to_pi(target_angle + math.pi - current)
        use_flipped = abs(flipped_delta) < abs(direct_delta)
        steering_delta = flipped_delta if use_flipped else direct_delta
        # PhysX only accepts reduced-coordinate drive targets in [-2π, 2π].
        # Preserve the nearest representation instead of jumping between the
        # equivalent +π and -π targets at the wrap boundary.
        steering_targets[module_index] = _physx_continuous_target(
            current, steering_delta
        )
        wheel_speed = speed / WHEEL_RADIUS_M
        alignment_scales[module_index] = _steering_alignment_scale(
            abs(steering_delta)
        )
        unscaled_drive_targets[module_index] = (
            -wheel_speed if use_flipped else wheel_speed
        )
    shared_alignment_scale = float(np.min(alignment_scales))
    if shared_alignment_scale <= MIN_STEERING_ALIGNMENT_SCALE + 1.0e-6:
        shared_alignment_scale = 0.0
    drive_targets[:] = unscaled_drive_targets * shared_alignment_scale
    return steering_targets, drive_targets


def task3_initial_arm_targets(
    joint_names,
) -> tuple[np.ndarray, np.ndarray]:
    """Return Task 3 arm-home values indexed in articulation DOF order."""
    selected = [
        (index, room_scene.INITIAL_ROBOT_JOINT_POS[name])
        for index, name in enumerate(joint_names)
        if name in room_scene.INITIAL_ROBOT_JOINT_POS
    ]
    return (
        np.array([value for _, value in selected], dtype=np.float32),
        np.array([index for index, _ in selected], dtype=np.int64),
    )


class DualArmRmpFlowTeleop:
    def __init__(
        self,
        robot,
        *,
        physics_dt: float,
        linear_speed: float,
        angular_speed_deg: float,
        base_linear_speed: float,
        base_angular_speed: float,
        base_heading_hold: bool,
        base_diagnostics_interval: float,
        rmpflow_cls,
        policy_cls,
        action_cls,
        rotation_matrix_to_quaternion,
    ) -> None:
        self._robot = robot
        self._physics_dt = physics_dt
        self._linear_speed = float(linear_speed)
        self._angular_speed = math.radians(float(angular_speed_deg))
        self._base_linear_speed = float(base_linear_speed)
        self._base_angular_speed = float(base_angular_speed)
        self._base_heading_hold_enabled = bool(base_heading_hold)
        self._base_diagnostics_interval = max(
            0.0, float(base_diagnostics_interval)
        )
        self._diagnostic_elapsed = 0.0
        self._next_diagnostic_time = self._base_diagnostics_interval
        self._action_cls = action_cls
        self._rotation_matrix_to_quaternion = rotation_matrix_to_quaternion
        self._joint_names = list(robot.dof_names)
        self._spine_index = self._joint_names.index(
            "franka_spine_vertical_joint"
        )
        drive_name_to_index = {
            name: index for index, name in enumerate(self._joint_names)
        }
        missing_drive_joints = [
            name
            for steering_name, drive_name, _x, _y in DRIVE_MODULES
            for name in (steering_name, drive_name)
            if name not in drive_name_to_index
        ]
        if missing_drive_joints:
            raise RuntimeError(
                "Missing mobile-base drive joints: "
                + ", ".join(missing_drive_joints)
            )
        self._steering_indices = tuple(
            drive_name_to_index[steering_name]
            for steering_name, _drive_name, _x, _y in DRIVE_MODULES
        )
        self._drive_indices = tuple(
            drive_name_to_index[drive_name]
            for _steering_name, drive_name, _x, _y in DRIVE_MODULES
        )
        self._held = set()
        self._arms = {}
        for side, config in ARM_CONFIGS.items():
            missing = [
                name
                for name in (*config["joints"], *config["fingers"])
                if name not in self._joint_names
            ]
            if missing:
                raise RuntimeError(f"Missing {side} RMPflow joints: {missing}")
            rmpflow = rmpflow_cls(
                robot_description_path=str(config["description"]),
                urdf_path=str(LULA_URDF_PATH),
                rmpflow_config_path=str(config["rmpflow"]),
                end_effector_frame_name=config["end_effector_frame"],
                maximum_substep_size=RMPFLOW_MAX_SUBSTEP_SIZE,
                ignore_robot_state_updates=True,
            )
            policy = policy_cls(robot, rmpflow, physics_dt)
            self._arms[side] = {
                "rmpflow": rmpflow,
                "policy": policy,
                "finger_indices": tuple(
                    self._joint_names.index(name) for name in config["fingers"]
                ),
                "gripper_open": True,
            }
        self._initialize_targets_from_fk()
        _root_position, root_orientation = self._robot.get_world_pose()
        self._diagnostic_origin = np.asarray(_root_position, dtype=np.float64)
        self._heading_hold_yaw = quaternion_yaw(root_orientation)
        self._last_root_yaw = self._heading_hold_yaw
        self._subscribe_keyboard()

    def _initialize_targets_from_fk(self) -> None:
        spine = float(self._robot.get_joint_positions()[self._spine_index])
        for arm in self._arms.values():
            rmpflow = arm["rmpflow"]
            rmpflow.set_robot_base_pose(
                np.zeros(3), np.array((1.0, 0.0, 0.0, 0.0))
            )
            active = (
                arm["policy"].get_active_joints_subset().get_joint_positions()
            )
            position, rotation = rmpflow.get_end_effector_pose(active)
            position = np.asarray(position, dtype=np.float64)
            position[2] += spine
            orientation = np.asarray(
                self._rotation_matrix_to_quaternion(rotation), dtype=np.float64
            )
            arm["home_position"] = position.copy()
            arm["home_orientation"] = orientation.copy()
            arm["target_position"] = position.copy()
            arm["target_orientation"] = orientation.copy()

    def _subscribe_keyboard(self) -> None:
        import carb.input
        import omni.appwindow

        self._carb_input = carb.input
        keys = carb.input.KeyboardInput
        layouts = {
            "left": (
                (keys.W, keys.S, keys.A, keys.D, keys.E, keys.Q),
                (keys.Z, keys.X, keys.T, keys.G, keys.C, keys.V),
                keys.F,
            ),
            "right": (
                (keys.O, keys.L, keys.K, keys.SEMICOLON, keys.P, keys.I),
                (keys.N, keys.M, keys.U, keys.J, keys.COMMA, keys.PERIOD),
                keys.APOSTROPHE,
            ),
        }
        for side, (linear_keys, angular_keys, gripper_key) in layouts.items():
            arm = self._arms[side]
            arm["linear_map"] = dict(
                zip(
                    linear_keys,
                    (
                        np.array((1, 0, 0)),
                        np.array((-1, 0, 0)),
                        np.array((0, 1, 0)),
                        np.array((0, -1, 0)),
                        np.array((0, 0, 1)),
                        np.array((0, 0, -1)),
                    ),
                )
            )
            arm["angular_map"] = dict(
                zip(
                    angular_keys,
                    (
                        np.array((1, 0, 0)),
                        np.array((-1, 0, 0)),
                        np.array((0, 1, 0)),
                        np.array((0, -1, 0)),
                        np.array((0, 0, 1)),
                        np.array((0, 0, -1)),
                    ),
                )
            )
            arm["gripper_key"] = gripper_key
        self._base_keys = {
            keys.H: "h",
            keys.N: "n",
            keys.B: "b",
            keys.M: "m",
            keys.G: "g",
            keys.J: "j",
        }
        self._shift_keys = {
            getattr(keys, name)
            for name in ("LEFT_SHIFT", "RIGHT_SHIFT", "SHIFT")
            if hasattr(keys, name)
        }
        self._reset_key = keys.R
        self._input = carb.input.acquire_input_interface()
        app_window = omni.appwindow.get_default_app_window()
        if app_window is None:
            raise RuntimeError("No Isaac Sim Kit window is available")
        self._keyboard = app_window.get_keyboard()
        self._subscription = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )
        print(CONTROL_HELP, flush=True)

    def _on_keyboard_event(self, event, *_args) -> bool:
        event_types = self._carb_input.KeyboardEventType
        if event.type in (event_types.KEY_PRESS, event_types.KEY_REPEAT):
            self._held.add(event.input)
            for side, arm in self._arms.items():
                if (
                    event.input == arm["gripper_key"]
                    and event.type == event_types.KEY_PRESS
                ):
                    arm["gripper_open"] = not arm["gripper_open"]
                    print(f"{side} gripper toggled", flush=True)
            if (
                event.input == self._reset_key
                and event.type == event_types.KEY_PRESS
            ):
                for arm in self._arms.values():
                    arm["target_position"] = arm["home_position"].copy()
                    arm["target_orientation"] = arm["home_orientation"].copy()
        elif event.type == event_types.KEY_RELEASE:
            self._held.discard(event.input)
        return True

    def _integrate_keys(self, dt: float) -> None:
        base_control_active = bool(self._held & self._shift_keys)
        for key in tuple(self._held):
            if base_control_active and key in self._base_keys:
                continue
            for arm in self._arms.values():
                direction = arm["linear_map"].get(key)
                if direction is not None:
                    arm["target_position"] += (
                        direction * self._linear_speed * dt
                    )
                axis = arm["angular_map"].get(key)
                if axis is not None:
                    orientation = quaternion_multiply(
                        _axis_angle_quaternion(axis, self._angular_speed * dt),
                        arm["target_orientation"],
                    )
                    arm["target_orientation"] = orientation / np.linalg.norm(
                        orientation
                    )

    def _base_twist(self) -> tuple[float, float, float]:
        held_names = {
            self._base_keys[key]
            for key in self._held
            if key in self._base_keys
        }
        if self._held & self._shift_keys:
            held_names.add("shift")
        unit_vx, unit_vy, unit_wz = base_twist_from_held_key_names(held_names)
        return (
            unit_vx * self._base_linear_speed,
            unit_vy * self._base_linear_speed,
            unit_wz * self._base_angular_speed,
        )

    def _apply_base(
        self,
        joint_positions: np.ndarray,
        root_position: np.ndarray,
        root_orientation: np.ndarray,
        dt: float,
    ) -> None:
        vx, vy, requested_wz = self._base_twist()
        current_yaw = quaternion_yaw(root_orientation)
        yaw_rate = _wrap_to_pi(current_yaw - self._last_root_yaw) / max(
            dt, 1.0e-6
        )
        self._last_root_yaw = current_yaw
        if self._base_heading_hold_enabled:
            wz, self._heading_hold_yaw = compensate_heading_yaw_rate(
                current_yaw=current_yaw,
                current_yaw_rate=yaw_rate,
                vx=vx,
                vy=vy,
                wz=requested_wz,
                desired_yaw=self._heading_hold_yaw,
                manual_rotation=abs(requested_wz) > STOP_EPS,
            )
        else:
            wz = requested_wz
            self._heading_hold_yaw = current_yaw
        steering_targets, drive_targets = compute_base_drive_targets(
            joint_positions, self._steering_indices, vx, vy, wz
        )
        controller = self._robot.get_articulation_controller()
        controller.apply_action(
            self._action_cls(
                joint_positions=steering_targets,
                joint_indices=np.asarray(
                    self._steering_indices, dtype=np.int64
                ),
            )
        )
        controller.apply_action(
            self._action_cls(
                joint_velocities=drive_targets,
                joint_indices=np.asarray(self._drive_indices, dtype=np.int64),
            )
        )
        self._diagnostic_elapsed += dt
        if (
            self._base_diagnostics_interval > 0.0
            and self._diagnostic_elapsed >= self._next_diagnostic_time
        ):
            joint_velocities = np.asarray(
                self._robot.get_joint_velocities(), dtype=np.float64
            )
            wheel_effort = None
            try:
                measured_efforts = self._robot.get_measured_joint_efforts()
                if measured_efforts is not None:
                    wheel_effort = np.asarray(measured_efforts)[
                        list(self._drive_indices)
                    ]
            except (AttributeError, RuntimeError):
                pass
            print(
                format_base_diagnostic_line(
                    elapsed_s=self._diagnostic_elapsed,
                    yaw_rad=current_yaw,
                    body_command=(vx, vy, wz),
                    displacement=root_position - self._diagnostic_origin,
                    steering_command=steering_targets,
                    steering_actual=joint_positions[
                        list(self._steering_indices)
                    ],
                    wheel_command=drive_targets,
                    wheel_actual=joint_velocities[list(self._drive_indices)],
                    wheel_effort=wheel_effort,
                ),
                flush=True,
            )
            self._next_diagnostic_time += self._base_diagnostics_interval

    def apply(self, dt: float) -> None:
        self._integrate_keys(dt)
        joint_positions = self._robot.get_joint_positions()
        root_position, root_orientation = self._robot.get_world_pose()
        root_position = np.asarray(root_position, dtype=np.float64)
        root_orientation = np.asarray(root_orientation, dtype=np.float64)
        self._apply_base(joint_positions, root_position, root_orientation, dt)
        spine = float(joint_positions[self._spine_index])
        base_position, base_orientation = compose_world_pose(
            root_position,
            root_orientation,
            np.array((0.0, 0.0, spine)),
            np.array((1.0, 0.0, 0.0, 0.0)),
        )
        actions = []
        for arm in self._arms.values():
            arm["rmpflow"].set_robot_base_pose(base_position, base_orientation)
            target_position, target_orientation = compose_world_pose(
                root_position,
                root_orientation,
                arm["target_position"],
                arm["target_orientation"],
            )
            arm["rmpflow"].set_end_effector_target(
                target_position, target_orientation
            )
            actions.append(arm["policy"].get_next_articulation_action(dt))
        positions, velocities, indices = merge_policy_actions(actions)
        merged_positions = {
            int(index): float(position)
            for index, position in zip(indices, positions)
        }
        merged_velocities = {
            int(index): float(velocity)
            for index, velocity in zip(indices, velocities)
        }
        for arm in self._arms.values():
            target = 0.04 if arm["gripper_open"] else 0.0
            for index in arm["finger_indices"]:
                merged_positions[index] = target
                merged_velocities[index] = 0.0
        if merged_positions:
            merged_indices = np.array(sorted(merged_positions), dtype=np.int64)
            self._robot.get_articulation_controller().apply_action(
                self._action_cls(
                    joint_positions=np.array(
                        [merged_positions[index] for index in merged_indices],
                        dtype=np.float32,
                    ),
                    joint_velocities=np.array(
                        [merged_velocities[index] for index in merged_indices],
                        dtype=np.float32,
                    ),
                    joint_indices=merged_indices,
                )
            )

    def close(self) -> None:
        if getattr(self, "_subscription", None) is not None:
            unsubscribe = getattr(
                self._input, "unsubscribe_to_keyboard_events", None
            )
            if unsubscribe is not None:
                unsubscribe(self._keyboard, self._subscription)
            self._subscription = None


def _find_physics_scene_path(stage) -> str:
    for prim in stage.Traverse():
        if str(prim.GetTypeName()) == "PhysicsScene":
            return str(prim.GetPath())
    return "/physicsScene"


def _find_articulation_root_path(stage, usd_physics) -> str:
    root = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    pending = [root]
    while pending:
        prim = pending.pop(0)
        if prim.HasAPI(usd_physics.ArticulationRootAPI):
            return str(prim.GetPath())
        pending.extend(prim.GetChildren())
    return ROBOT_PRIM_PATH


def robot_drive_gains(joint_name: str) -> tuple[float, float, float] | None:
    """Return the Task 3 actuator gains used by the Isaac Lab runtime."""
    base_gains = {
        "tmrv0_2_joint_0": (500.0, 50.0, 200.0),
        "tmrv0_2_joint_2": (500.0, 50.0, 200.0),
        "tmrv0_2_joint_1": (0.0, 5.0, 500.0),
        "tmrv0_2_joint_3": (0.0, 5.0, 500.0),
    }
    if joint_name in base_gains:
        return base_gains[joint_name]
    if joint_name == "rocker_arm_joint":
        return 0.0, 0.003, 500.0
    if "caster" in joint_name:
        return 0.0, 0.0, 0.0
    if joint_name == "franka_spine_vertical_joint" or re.fullmatch(
        r"(?:left|right)_fr3v2_joint[1-7]", joint_name
    ):
        return 5000.0, 500.0, 200.0
    if "finger" in joint_name:
        return 200.0, 20.0, 50.0
    return None


def _configure_robot_drives(stage, usd_physics) -> None:
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    pending = [robot_prim]
    while pending:
        prim = pending.pop()
        gains = robot_drive_gains(prim.GetName())
        if gains is not None:
            stiffness, damping, max_force = gains
            drive_type = (
                "linear" if prim.IsA(usd_physics.PrismaticJoint) else "angular"
            )
            drive = usd_physics.DriveAPI.Apply(prim, drive_type)
            drive.CreateStiffnessAttr().Set(stiffness)
            drive.CreateDampingAttr().Set(damping)
            drive.CreateMaxForceAttr().Set(max_force)
        pending.extend(prim.GetChildren())


def run(args: argparse.Namespace) -> None:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {"headless": args.headless, "width": 1280, "height": 720},
        experience=experience_path(args),
    )
    teleop = None
    try:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.robot_motion.motion_generation")
        simulation_app.update()

        import omni.kit.app
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.rotations import rot_matrix_to_quat
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.robot_motion.motion_generation.articulation_motion_policy import (  # noqa: E501
            ArticulationMotionPolicy,
        )
        from isaacsim.robot_motion.motion_generation.lula.motion_policies import (  # noqa: E501
            RmpFlow,
        )
        from pxr import UsdPhysics

        room_path = Path(args.room_usd).expanduser().resolve()
        robot_path = Path(args.robot_usd).expanduser().resolve()
        required = [room_path, robot_path, LULA_URDF_PATH]
        required.extend(
            config["description"] for config in ARM_CONFIGS.values()
        )
        required.extend(config["rmpflow"] for config in ARM_CONFIGS.values())
        missing = [path for path in required if not path.is_file()]
        if missing:
            raise FileNotFoundError(
                "Missing Task 3 RMPflow assets: "
                + ", ".join(map(str, missing))
            )

        args.task = "task3"
        robot_position = room_scene.resolve_robot_position(args)
        robot_yaw = room_scene.resolve_robot_yaw(args)
        room_scene.build_stage(
            omni.kit.app.get_app(),
            room_path=room_path,
            robot_path=robot_path,
            task="task3",
            robot_position=robot_position,
            robot_rotation=room_scene.yaw_to_quat(robot_yaw),
            robot_yaw=robot_yaw,
            head_placement=args.head_placement,
        )
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        _configure_robot_drives(stage, UsdPhysics)
        world = World(
            physics_prim_path=_find_physics_scene_path(stage),
            stage_units_in_meters=1.0,
            physics_dt=1.0 / max(args.physics_hz, 1.0),
            rendering_dt=1.0 / max(args.render_hz, 1.0),
        )
        root_path = _find_articulation_root_path(stage, UsdPhysics)
        robot = SingleArticulation(prim_path=root_path, name="task3_robot")
        world.scene.add(robot)
        world.reset()
        initial_positions, initial_indices = task3_initial_arm_targets(
            robot.dof_names
        )
        if len(initial_indices) != len(room_scene.INITIAL_ROBOT_JOINT_POS):
            raise RuntimeError(
                "Task 3 robot is missing one or more named initial arm joints"
            )
        robot.set_joint_positions(
            initial_positions, joint_indices=initial_indices
        )
        robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_positions=initial_positions,
                joint_indices=initial_indices,
            )
        )
        world.step(render=not args.headless)

        if arm_keyboard_enabled(args):
            teleop = DualArmRmpFlowTeleop(
                robot,
                physics_dt=1.0 / max(args.physics_hz, 1.0),
                linear_speed=args.linear_speed,
                angular_speed_deg=args.angular_speed_deg,
                base_linear_speed=args.base_linear_speed,
                base_angular_speed=args.base_angular_speed,
                base_heading_hold=args.base_heading_hold,
                base_diagnostics_interval=args.base_diagnostics_interval,
                rmpflow_cls=RmpFlow,
                policy_cls=ArticulationMotionPolicy,
                action_cls=ArticulationAction,
                rotation_matrix_to_quaternion=rot_matrix_to_quat,
            )
        elif args.headless:
            print(
                "Headless mode: arm keyboard teleop is disabled.", flush=True
            )

        while simulation_app.is_running():
            if teleop is not None:
                teleop.apply(1.0 / max(args.render_hz, 1.0))
            world.step(render=not args.headless)
    finally:
        if teleop is not None:
            teleop.close()
        simulation_app.close()


def main() -> None:
    run(build_arg_parser().parse_args())


if __name__ == "__main__":
    main()
