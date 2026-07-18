# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared runtime for the Task 2 Isaac Sim 5.1.0 teleop bridge scripts.

IMPORTANT: this module imports Isaac Sim and rclpy modules at import time —
only import it *after* SimulationApp has been created and the
isaacsim.ros2.bridge extension has been enabled.

The joint names, gripper coupling, and topic layout are reused from Task 1
(task1_isaacsim/scripts/isaac_bridge_constants.py); the swerve-base and spine
control logic is ported from
task1_isaacsim/scripts/isaaclab_fr3duo_newton_bridge.py.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
# Reuse the Task 1 shared constants (joint names, gripper coupling, defaults).
sys.path.insert(0, str(_REPO_ROOT / "task1_isaacsim" / "scripts"))
# Make the local recording package importable regardless of the entry point.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from isaac_bridge_constants import (  # noqa: E402
    LEFT_GRIPPER_COUPLED_JOINT_MULTIPLIERS,
    LEFT_GRIPPER_DRIVER_JOINT,
    LEFT_JOINTS,
    RIGHT_GRIPPER_COUPLED_JOINT_MULTIPLIERS,
    RIGHT_GRIPPER_DRIVER_JOINT,
    RIGHT_JOINTS,
)
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import omni.usd  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.rotations import rot_matrix_to_quat  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import (  # noqa: E402, E501
    ArticulationMotionPolicy,
)
from isaacsim.robot_motion.motion_generation.lula.motion_policies import (  # noqa: E402
    RmpFlow,
)
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML ships with Isaac Sim
    yaml = None

from topics import TOPICS_YAML, load_topics  # noqa: E402

# All topic names come from the shared contract (config/topics.yaml);
# load_topics() fails hard if the file or a required key is missing.
_TOPICS = load_topics()

PEDAL_STATE_TOPIC = _TOPICS["teleop"]["pedal_state"]
# Recording streams (published only with --publish-recording-topics): the
# post-arbitration joint position targets, base odometry/applied twist, and
# link8 end-effector poses consumed by
# task2_isaacsim/services/recording/record_task2.py.
CLOCK_TOPIC = _TOPICS["clock"]
APPLIED_COMMANDS_TOPIC = _TOPICS["recording"]["applied_joint_commands"]
FULL_STATES_TOPIC = _TOPICS["recording"]["joint_states_full"]
ODOM_TOPIC = _TOPICS["recording"]["odom"]
CMD_VEL_APPLIED_TOPIC = _TOPICS["recording"]["cmd_vel_applied"]
EE_POSE_TOPICS = dict(_TOPICS["recording"]["ee_pose"])
EE_POSE_PRIM_NAMES = {
    "left": "left_fr3v2_link8",
    "right": "right_fr3v2_link8",
}
WHEEL_RADIUS_M = 0.05
MAX_WHEEL_SPEED_RADPS = 18.0
STOP_EPS = 1.0e-4
STEERING_FULL_SPEED_ERROR_RAD = math.radians(8.0)
STEERING_ZERO_SPEED_ERROR_RAD = math.radians(35.0)

# Same ready pose the Task 1 bridge uses as the articulation init state.
ARM_READY_POSE = {
    "left_fr3v2_joint1": 0.0,
    "left_fr3v2_joint2": -0.7854,
    "left_fr3v2_joint3": 0.0,
    "left_fr3v2_joint4": -2.3562,
    "left_fr3v2_joint5": 0.0,
    "left_fr3v2_joint6": 1.5708,
    "left_fr3v2_joint7": 0.7854,
    "right_fr3v2_joint1": 0.0,
    "right_fr3v2_joint2": -0.7854,
    "right_fr3v2_joint3": 0.0,
    "right_fr3v2_joint4": -2.3562,
    "right_fr3v2_joint5": 0.0,
    "right_fr3v2_joint6": 1.5708,
    "right_fr3v2_joint7": 0.7854,
}


# Dual-arm keyboard teleop (RMPflow) assets. The Lula descriptors, URDF, and
# RMPflow configs are adapted from the archived dual_arm_rmp_widget demo and
# Isaac Sim's FR3 motion policy config; see the files for provenance notes.
LULA_ASSETS_DIR = (
    _REPO_ROOT / "task2_isaacsim" / "assets" / "lula" / "mobile_fr3_duo"
)
LULA_URDF_PATH = LULA_ASSETS_DIR / "mobile_fr3_duo_v0_2_lula.urdf"
RMPFLOW_MAX_SUBSTEP_SIZE = 0.0034

ARM_TELEOP_CONFIGS = {
    "left": {
        "robot_description_path": (
            LULA_ASSETS_DIR / "left_arm_description.yaml"
        ),
        "rmpflow_config_path": (
            LULA_ASSETS_DIR / "left_arm_rmpflow_config.yaml"
        ),
        "end_effector_frame_name": "left_tcp",
        "arm_joint_names": tuple(LEFT_JOINTS),
        "gripper_label": "left_gripper",
        "gripper_driver_joint": LEFT_GRIPPER_DRIVER_JOINT,
    },
    "right": {
        "robot_description_path": (
            LULA_ASSETS_DIR / "right_arm_description.yaml"
        ),
        "rmpflow_config_path": (
            LULA_ASSETS_DIR / "right_arm_rmpflow_config.yaml"
        ),
        "end_effector_frame_name": "right_tcp",
        "arm_joint_names": tuple(RIGHT_JOINTS),
        "gripper_label": "right_gripper",
        "gripper_driver_joint": RIGHT_GRIPPER_DRIVER_JOINT,
    },
}

ARM_TELEOP_CONTROLS_BANNER = """\
Dual-arm keyboard teleop enabled (keys act in the Isaac Sim window; arm
moves are in the robot base frame):
  LEFT arm:
    W/S A/D Q/E : move end effector (fwd/back, left/right, up/down)
    Z/X T/G C/V : rotate end effector (roll / pitch / yaw)
    F           : toggle gripper
  RIGHT arm (same layout on the right half of the keyboard):
    O/L K/; I/P : move end effector (fwd/back, left/right, up/down)
    N/M U/J ,/. : rotate end effector (roll / pitch / yaw)
    '           : toggle gripper
  R           : reset both arm targets to the ready pose
ROS arm/gripper commands are NOT applied while this teleop is active."""


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


@dataclass(frozen=True)
class JointGroup:
    label: str
    state_topic: str
    command_topics: list[str]
    requested_names: list[str]


def _command_topics(
    primary_topic: str, browser_topic: str, include_browser: bool
) -> list[str]:
    topics = [primary_topic]
    if include_browser:
        topics.append(browser_topic)
    return topics


def _load_joint_groups(
    franka_root: Path,
    embodiment: str,
    *,
    include_browser_commands: bool = True,
) -> list[JointGroup]:
    contract_path = (
        franka_root
        / "assets"
        / "embodiments"
        / embodiment
        / "data_contract_recording.yaml"
    )
    left_names = list(LEFT_JOINTS)
    right_names = list(RIGHT_JOINTS)

    if contract_path.exists() and yaml is not None:
        with contract_path.open("r", encoding="utf-8") as f:
            contract = yaml.safe_load(f) or {}
        state = contract.get("state_structure", {})
        arms = state.get("arms", {})
        left_names = list(
            arms.get("left", {}).get("joint_names") or left_names
        )
        right_names = list(
            arms.get("right", {}).get("joint_names") or right_names
        )

    requested_names = {
        "left_arm": left_names,
        "right_arm": right_names,
        "left_gripper": [LEFT_GRIPPER_DRIVER_JOINT],
        "right_gripper": [RIGHT_GRIPPER_DRIVER_JOINT],
    }
    group_topics = _TOPICS["bridge"]["joint_groups"]
    return [
        JointGroup(
            label=label,
            state_topic=group_topics[label]["state"],
            command_topics=_command_topics(
                group_topics[label]["command"],
                group_topics[label]["browser_command"],
                include_browser_commands,
            ),
            requested_names=names,
        )
        for label, names in requested_names.items()
    ]


GRIPPER_COUPLED_MULTIPLIERS = {
    "left_gripper": LEFT_GRIPPER_COUPLED_JOINT_MULTIPLIERS,
    "right_gripper": RIGHT_GRIPPER_COUPLED_JOINT_MULTIPLIERS,
}


def _candidate_joint_names(name: str) -> Iterable[str]:
    yield name
    if "fr3v2_joint" in name:
        yield name.replace("fr3v2_joint", "fr3v2_1_joint")
    if "fr3v2_1_joint" in name:
        yield name.replace("fr3v2_1_joint", "fr3v2_joint")
    if name == "left_robotiq_85_left_knuckle_joint":
        yield "left_fr3v2_finger_joint1"
    if name == "right_robotiq_85_left_knuckle_joint":
        yield "right_fr3v2_finger_joint1"
    # Franka hand robot USDs: the Robotiq driver name maps onto finger joint 1.
    if name == "left_right_finger_joint":
        yield "left_fr3v2_finger_joint1"
    if name == "right_right_finger_joint":
        yield "right_fr3v2_finger_joint1"
    if name.endswith("_joint"):
        yield name[:-6]


def _resolve_joint_index(requested_name: str, actual_by_name: dict[str, int]):
    for candidate in _candidate_joint_names(requested_name):
        if candidate in actual_by_name:
            return actual_by_name[candidate]
    return None


def _resolve_group_indices(
    groups: list[JointGroup], actual_names: list[str]
) -> dict[str, dict[str, int]]:
    actual_by_name = {name: idx for idx, name in enumerate(actual_names)}
    resolved: dict[str, dict[str, int]] = {}
    for group in groups:
        group_map = {}
        for requested_name in group.requested_names:
            joint_index = _resolve_joint_index(requested_name, actual_by_name)
            if joint_index is not None:
                group_map[requested_name] = joint_index
        resolved[group.label] = group_map
    return resolved


def _resolve_coupled_indices(
    groups: list[JointGroup],
    group_indices: dict[str, dict[str, int]],
    actual_names: list[str],
    *,
    include_robotiq_coupled: bool,
) -> dict[str, dict[int, float]]:
    """Map gripper group label -> {dof index: multiplier} for joints that
    must follow the driver."""
    actual_by_name = {name: idx for idx, name in enumerate(actual_names)}
    resolved: dict[str, dict[int, float]] = {}
    for group in groups:
        multipliers = GRIPPER_COUPLED_MULTIPLIERS.get(group.label)
        if multipliers is None:
            continue
        index_map: dict[int, float] = {}
        if include_robotiq_coupled:
            for joint_name, multiplier in multipliers.items():
                joint_index = _resolve_joint_index(joint_name, actual_by_name)
                if joint_index is not None:
                    index_map[joint_index] = float(multiplier)
        # Franka hand grippers have no mimic API: finger_joint2 carries its own
        # drive and must mirror the driver (finger_joint1).
        driver_index = group_indices.get(group.label, {}).get(
            group.requested_names[0]
        )
        if driver_index is not None:
            driver_actual = actual_names[driver_index]
            if driver_actual.endswith("finger_joint1"):
                follower = driver_actual[:-1] + "2"
                if follower in actual_by_name:
                    index_map[actual_by_name[follower]] = 1.0
        resolved[group.label] = index_map
    return resolved


def _iter_prims_under(root_prim):
    yield root_prim
    for child in root_prim.GetChildren():
        yield from _iter_prims_under(child)


def _fix_single_articulation_root(robot_prim_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print(
            "Warning: cannot patch articulation roots: no USD stage",
            file=sys.stderr,
        )
        return
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(
            "Warning: cannot patch articulation roots: "
            f"robot prim not found: {robot_prim_path}",
            file=sys.stderr,
        )
        return

    root_prims = [
        prim
        for prim in _iter_prims_under(robot_prim)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(root_prims) <= 1:
        return

    keep_prim = None
    for preferred_path in (
        f"{robot_prim_path}/base",
        f"{robot_prim_path}/base_link",
    ):
        candidate = stage.GetPrimAtPath(preferred_path)
        if candidate in root_prims:
            keep_prim = candidate
            break
    if keep_prim is None:
        keep_prim = root_prims[0]

    removed = []
    for prim in root_prims:
        if prim == keep_prim:
            continue
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        removed.append(str(prim.GetPath()))

    print(f"Keeping articulation root: {keep_prim.GetPath()}")
    if removed:
        print("Removed extra articulation roots:")
        for prim_path in removed:
            print(f"  {prim_path}")


def _deactivate_embedded_graphs(robot_prim_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return
    deactivated = []
    for prim in _iter_prims_under(robot_prim):
        if str(prim.GetTypeName()) == "OmniGraph":
            prim.SetActive(False)
            deactivated.append(str(prim.GetPath()))
    if deactivated:
        print("Deactivated embedded OmniGraph graphs:")
        for prim_path in deactivated:
            print(f"  {prim_path}")


def _find_articulation_root_path(robot_prim_path: str) -> str:
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    for prim in _iter_prims_under(robot_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return robot_prim_path


def _find_physics_scene_path():
    """Return the path of an existing PhysicsScene prim on the stage, if
    any."""
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None
    for prim in stage.Traverse():
        if str(prim.GetTypeName()) == "PhysicsScene":
            return str(prim.GetPath())
    return None


def _joint_drive(prim):
    if prim.IsA(UsdPhysics.PrismaticJoint):
        return UsdPhysics.DriveAPI.Apply(prim, "linear")
    return UsdPhysics.DriveAPI.Apply(prim, "angular")


def _set_drive_gains(
    prim, *, stiffness: float, damping: float, max_force: float
) -> None:
    drive = _joint_drive(prim)
    drive.CreateStiffnessAttr().Set(float(stiffness))
    drive.CreateDampingAttr().Set(float(damping))
    drive.CreateMaxForceAttr().Set(float(max_force))


def _configure_drives(robot_prim_path: str, gains_by_joint_name) -> None:
    """Author drive gains on joints selected by name;
    gains_by_joint_name(name) -> dict or None."""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(
            "Warning: cannot configure drives: "
            f"robot prim not found: {robot_prim_path}",
            file=sys.stderr,
        )
        return
    configured = []
    for prim in _iter_prims_under(robot_prim):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        gains = gains_by_joint_name(prim.GetName())
        if gains is None:
            continue
        _set_drive_gains(prim, **gains)
        configured.append(prim.GetName())
    if configured:
        print("Configured joint drives:", ", ".join(sorted(configured)))


def _base_drive_gains(joint_name: str):
    # Same values as the Task 1 base actuators. Wheel drive joints must have
    # zero position stiffness so velocity targets take effect.
    if joint_name in ("tmrv0_2_joint_0", "tmrv0_2_joint_2"):
        return {"stiffness": 500.0, "damping": 50.0, "max_force": 200.0}
    if joint_name in ("tmrv0_2_joint_1", "tmrv0_2_joint_3"):
        return {"stiffness": 0.0, "damping": 5.0, "max_force": 500.0}
    return None


def _find_drive_joint_ids(
    joint_names: list[str],
) -> tuple[list[int], list[int]]:
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


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _steering_alignment_scale(error: float) -> float:
    scale = (STEERING_ZERO_SPEED_ERROR_RAD - error) / (
        STEERING_ZERO_SPEED_ERROR_RAD - STEERING_FULL_SPEED_ERROR_RAD
    )
    return min(max(scale, 0.0), 1.0)


def _compute_drive_targets(
    joint_positions: np.ndarray,
    steering_ids: list[int],
    vx: float,
    vy: float,
    wz: float,
) -> tuple[np.ndarray, np.ndarray]:
    steering_targets = np.zeros(len(DRIVE_MODULES), dtype=np.float32)
    drive_targets = np.zeros(len(DRIVE_MODULES), dtype=np.float32)

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
        current_angle = float(joint_positions[steering_ids[module_index]])

        if speed_mps < STOP_EPS:
            steering_targets[module_index] = current_angle
            continue

        raw_target = math.atan2(wheel_vy, wheel_vx)
        direct_delta = _wrap_to_pi(raw_target - current_angle)
        flipped_delta = _wrap_to_pi(raw_target + math.pi - current_angle)
        use_flipped = abs(flipped_delta) < abs(direct_delta)
        steering_delta = flipped_delta if use_flipped else direct_delta

        steering_targets[module_index] = current_angle + steering_delta
        wheel_speed = (speed_mps / WHEEL_RADIUS_M) * _steering_alignment_scale(
            abs(steering_delta)
        )
        drive_targets[module_index] = (
            -wheel_speed if use_flipped else wheel_speed
        )

    return steering_targets, drive_targets


def _quat_mul(q1, q2):
    """Hamilton product of two wxyz quaternions (q1 applied after q2)."""
    w1, x1, y1, z1 = (float(v) for v in q1)
    w2, x2, y2, z2 = (float(v) for v in q2)
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def _axis_angle_quat(axis, angle: float):
    """wxyz quaternion for a rotation of ``angle`` rad about ``axis``."""
    half = 0.5 * float(angle)
    return np.array(
        [math.cos(half), *(math.sin(half) * np.asarray(axis, dtype=float))]
    )


def _compose_world_pose(parent_pos, parent_quat, child_pos, child_quat):
    """World pose of a child frame from the parent world pose and the
    child's pose in the parent frame. Quaternions are wxyz numpy arrays."""

    def _mat(pos, quat):
        m = Gf.Matrix4d().SetRotate(
            Gf.Quatd(
                float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
            )
        )
        m.SetTranslateOnly(
            Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2]))
        )
        return m

    world = _mat(child_pos, child_quat) * _mat(parent_pos, parent_quat)
    translation = world.ExtractTranslation()
    rotation = world.ExtractRotationQuat()
    pos = np.array([translation[0], translation[1], translation[2]])
    quat = np.array(
        [
            rotation.GetReal(),
            rotation.GetImaginary()[0],
            rotation.GetImaginary()[1],
            rotation.GetImaginary()[2],
        ]
    )
    return pos, quat


def _disable_conflicting_kit_hotkeys(used_keys) -> None:
    """Deregister Kit hotkeys bound to bare keys the teleop uses.

    The Isaac Sim viewport binds plain keys (F = frame selection, Q/W/E/R =
    transform tools, ...) that would fire while teleoperating with the
    viewport focused. Only UNMODIFIED combinations on teleop keys are
    removed; everything else survives -- modified combos (Ctrl+S), function
    keys (F7 toggle UI, F11 fullscreen), Space (play/pause), Esc, Del. No-op
    when the hotkey extension is not loaded (headless runs).
    """
    try:
        import omni.kit.hotkeys.core as hotkeys_core  # noqa: PLC0415
    except ImportError:
        return
    registry = hotkeys_core.get_hotkey_registry()
    removed = []
    for hotkey in list(registry.get_all_hotkeys()):
        combo = hotkey.key_combination
        if combo is None or combo.modifiers or combo.key not in used_keys:
            continue
        if registry.deregister_hotkey(hotkey):
            removed.append(f"{combo.as_string}->{hotkey.action_id}")
    if removed:
        print(
            f"Disabled {len(removed)} conflicting Kit hotkeys: "
            f"{', '.join(removed)}"
        )


class SpineKeyboardController:
    def __init__(
        self,
        robot: SingleArticulation,
        joint_names: list[str],
        *,
        step_m: float,
        min_m: float,
        max_m: float,
    ):
        self.robot = robot
        self.joint_name = "franka_spine_vertical_joint"
        self.joint_index = joint_names.index(self.joint_name)
        self.step_m = float(step_m)
        self.min_m = float(min_m)
        self.max_m = float(max_m)
        if self.min_m > self.max_m:
            self.min_m, self.max_m = self.max_m, self.min_m

        initial = float(robot.get_joint_positions()[self.joint_index])
        self.target = max(self.min_m, min(self.max_m, initial))

        self._subscription = None
        self._input = None
        self._keyboard = None
        try:
            import carb.input  # noqa: PLC0415
            import omni.appwindow  # noqa: PLC0415

            self._carb_input = carb.input
            self._input = carb.input.acquire_input_interface()
            app_window = omni.appwindow.get_default_app_window()
            if app_window is None:
                raise RuntimeError("No Omniverse app window found")
            self._keyboard = app_window.get_keyboard()
            self._subscription = self._input.subscribe_to_keyboard_events(
                self._keyboard, self._on_keyboard_event
            )
            print(
                "Spine keyboard control enabled: Up/Down arrows command "
                f"{self.joint_name}, step={self.step_m:.4f} m, "
                f"range=[{self.min_m:.4f}, {self.max_m:.4f}] m",
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001 - keyboard is optional in headless sessions
            self._carb_input = None
            print(
                f"Warning: spine keyboard control unavailable: {exc}",
                file=sys.stderr,
            )

    @property
    def available(self) -> bool:
        return self._subscription is not None

    def _set_target(self, value: float) -> None:
        value = max(self.min_m, min(self.max_m, float(value)))
        self.target = value
        print(f"{self.joint_name}: target={value:.4f} m", flush=True)

    def _on_keyboard_event(self, event, *args, **kwargs):
        if self._carb_input is None:
            return True
        if event.type not in (
            self._carb_input.KeyboardEventType.KEY_PRESS,
            self._carb_input.KeyboardEventType.KEY_REPEAT,
        ):
            return True

        key_name = str(getattr(event.input, "name", event.input)).upper()
        if key_name in {"UP", "KEY_UP", "ARROW_UP"} or key_name.endswith(
            "_UP"
        ):
            self._set_target(self.target + self.step_m)
            return True
        if key_name in {"DOWN", "KEY_DOWN", "ARROW_DOWN"} or key_name.endswith(
            "_DOWN"
        ):
            self._set_target(self.target - self.step_m)
            return True
        return True

    def apply(self) -> None:
        self.robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_positions=np.array([self.target], dtype=np.float32),
                joint_indices=np.array([self.joint_index]),
            )
        )

    def reset_target(self) -> None:
        """Re-read the live joint position as the target (after world reset,
        so the spine does not snap back to a stale height)."""
        current = float(self.robot.get_joint_positions()[self.joint_index])
        self.target = max(self.min_m, min(self.max_m, current))


class DualArmKeyboardTeleop:
    """Keyboard end-effector teleop for both FR3 arms via dual RMPflow.

    Ported from the archived dual_arm_rmp_widget demo: one RmpFlow per arm
    (each Lula descriptor's cspace covers only that arm's seven joints),
    wrapped in ArticulationMotionPolicy; the two per-arm actions are merged
    by joint index into a single ArticulationAction each step.

    Each arm's target pose is stored in the robot ROOT frame so it rides
    along while the base drives; the solver base pose is re-set every step
    from the live root pose plus the spine lift (the descriptors pin the
    spine joint at zero, and only wheels/casters -- irrelevant to arm EE
    kinematics -- sit below the spine, so lifting the whole URDF by the
    live spine position reproduces the lifted arm mounts on a flat floor).

    RmpFlow runs with ignore_robot_state_updates=True: it rolls out its own
    smooth kinematic trajectory and the PhysX drives track it. Feeding the
    PhysX-measured joint state back into the policy (the default) stalls it
    in false equilibria decimeters from the target -- verified empirically;
    the internal rollout tracks the same targets to millimeters, with the
    simulated robot following within ~2 cm.
    """

    def __init__(
        self,
        robot: SingleArticulation,
        joint_names: list[str],
        group_indices: dict[str, dict[str, int]],
        coupled_indices: dict[str, dict[int, float]],
        args,
    ):
        self.robot = robot
        self._physics_dt = 1.0 / max(float(args.physics_hz), 1.0)
        self._linear_speed = float(args.arm_teleop_linear_speed)
        self._angular_speed = math.radians(
            float(args.arm_teleop_angular_speed_deg)
        )
        self._gripper_positions = {
            True: float(args.arm_teleop_gripper_open),
            False: float(args.arm_teleop_gripper_closed),
        }

        # RmpFlow binds articulation joints by URDF name, so the USD must
        # use the exact URDF names (no _candidate_joint_names fuzzing).
        missing = [
            name
            for config in ARM_TELEOP_CONFIGS.values()
            for name in config["arm_joint_names"]
            if name not in joint_names
        ]
        if missing:
            raise RuntimeError(
                "robot joint names do not match the Lula URDF, missing: "
                + ", ".join(missing)
            )

        self._spine_index = (
            joint_names.index("franka_spine_vertical_joint")
            if "franka_spine_vertical_joint" in joint_names
            else None
        )

        self._arms = {}
        for arm_name, config in ARM_TELEOP_CONFIGS.items():
            rmpflow = RmpFlow(
                robot_description_path=str(config["robot_description_path"]),
                urdf_path=str(LULA_URDF_PATH),
                rmpflow_config_path=str(config["rmpflow_config_path"]),
                end_effector_frame_name=config["end_effector_frame_name"],
                maximum_substep_size=RMPFLOW_MAX_SUBSTEP_SIZE,
                ignore_robot_state_updates=True,
            )
            policy = ArticulationMotionPolicy(robot, rmpflow, self._physics_dt)
            self._arms[arm_name] = {
                "rmpflow": rmpflow,
                "policy": policy,
                "gripper_driver_index": group_indices.get(
                    config["gripper_label"], {}
                ).get(config["gripper_driver_joint"]),
                "gripper_coupled": dict(
                    coupled_indices.get(config["gripper_label"], {})
                ),
                "gripper_open": True,
            }
        self._init_home_targets()

        self._held = set()
        self._reset_key = None
        self._subscription = None
        self._input = None
        self._keyboard = None
        try:
            import carb.input  # noqa: PLC0415
            import omni.appwindow  # noqa: PLC0415

            self._carb_input = carb.input
            self._build_key_maps(carb.input.KeyboardInput)
            self._input = carb.input.acquire_input_interface()
            app_window = omni.appwindow.get_default_app_window()
            if app_window is None:
                raise RuntimeError("No Omniverse app window found")
            self._keyboard = app_window.get_keyboard()
            self._subscription = self._input.subscribe_to_keyboard_events(
                self._keyboard, self._on_keyboard_event
            )
            _disable_conflicting_kit_hotkeys(self._teleop_keys())
            print(ARM_TELEOP_CONTROLS_BANNER, flush=True)
        except Exception as exc:  # noqa: BLE001 - keyboard is optional in headless sessions
            self._carb_input = None
            print(
                f"Warning: dual-arm keyboard teleop unavailable: {exc}",
                file=sys.stderr,
            )

    @property
    def available(self) -> bool:
        return self._subscription is not None

    def _build_key_maps(self, keyboard_input) -> None:
        # Per-arm key clusters: (fwd, back, left, right, up, down),
        # (roll+/-, pitch+/-, yaw+/-), gripper toggle. The right-arm
        # cluster is the left-arm layout shifted onto the right half of
        # the keyboard. Same bindings as examples/demo_teleop_aloha.py.
        arm_keys = {
            "left": (
                (
                    keyboard_input.W,
                    keyboard_input.S,
                    keyboard_input.A,
                    keyboard_input.D,
                    keyboard_input.Q,
                    keyboard_input.E,
                ),
                (
                    keyboard_input.Z,
                    keyboard_input.X,
                    keyboard_input.T,
                    keyboard_input.G,
                    keyboard_input.C,
                    keyboard_input.V,
                ),
                keyboard_input.F,
            ),
            "right": (
                (
                    keyboard_input.O,
                    keyboard_input.L,
                    keyboard_input.K,
                    keyboard_input.SEMICOLON,
                    keyboard_input.I,
                    keyboard_input.P,
                ),
                (
                    keyboard_input.N,
                    keyboard_input.M,
                    keyboard_input.U,
                    keyboard_input.J,
                    keyboard_input.COMMA,
                    keyboard_input.PERIOD,
                ),
                keyboard_input.APOSTROPHE,
            ),
        }
        for arm_name, (linear, angular, gripper_key) in arm_keys.items():
            fwd, back, left, right, up, down = linear
            roll_p, roll_n, pitch_p, pitch_n, yaw_p, yaw_n = angular
            arm = self._arms[arm_name]
            arm["linear_map"] = {
                fwd: np.array([+1.0, 0.0, 0.0]),
                back: np.array([-1.0, 0.0, 0.0]),
                left: np.array([0.0, +1.0, 0.0]),
                right: np.array([0.0, -1.0, 0.0]),
                up: np.array([0.0, 0.0, +1.0]),
                down: np.array([0.0, 0.0, -1.0]),
            }
            arm["angular_map"] = {
                roll_p: np.array([+1.0, 0.0, 0.0]),
                roll_n: np.array([-1.0, 0.0, 0.0]),
                pitch_p: np.array([0.0, +1.0, 0.0]),
                pitch_n: np.array([0.0, -1.0, 0.0]),
                yaw_p: np.array([0.0, 0.0, +1.0]),
                yaw_n: np.array([0.0, 0.0, -1.0]),
            }
            arm["gripper_key"] = gripper_key
        self._reset_key = keyboard_input.R

    def _teleop_keys(self) -> set:
        keys = {self._reset_key}
        for arm in self._arms.values():
            keys.update(arm["linear_map"])
            keys.update(arm["angular_map"])
            keys.add(arm["gripper_key"])
        return keys

    def _spine_lift(self, joint_positions) -> float:
        if self._spine_index is None:
            return 0.0
        return float(joint_positions[self._spine_index])

    def _init_home_targets(self) -> None:
        """Initialize each arm's target from FK of the current joint state.

        With the solver base at identity, RmpFlow FK yields the TCP pose in
        the URDF-root frame with the spine at zero; adding the live spine
        lift converts it to the robot ROOT frame the targets are stored in.
        Starting from FK means the first commanded target coincides with
        the actual TCP pose, so nothing jumps when the teleop engages.
        """
        spine = self._spine_lift(self.robot.get_joint_positions())
        for arm in self._arms.values():
            rmpflow = arm["rmpflow"]
            rmpflow.set_robot_base_pose(
                np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
            )
            active_positions = (
                arm["policy"].get_active_joints_subset().get_joint_positions()
            )
            position, rot_matrix = rmpflow.get_end_effector_pose(
                active_positions
            )
            position = np.array(position, dtype=float)
            position[2] += spine
            arm["home_position"] = position
            # Store the orientation as a quaternion: an euler round-trip
            # is lossy near the flange's pitch singularity (the left arm's
            # ready pose sits close to one, which sent RMPflow sideways).
            arm["home_quat"] = rot_matrix_to_quat(rot_matrix)
            arm["target_position"] = arm["home_position"].copy()
            arm["target_quat"] = arm["home_quat"].copy()

    def _on_keyboard_event(self, event, *args, **kwargs):
        if self._carb_input is None:
            return True
        event_type = self._carb_input.KeyboardEventType
        if event.type == event_type.KEY_PRESS:
            self._held.add(event.input)
            for arm_name, arm in self._arms.items():
                if event.input == arm["gripper_key"]:
                    arm["gripper_open"] = not arm["gripper_open"]
                    state = "open" if arm["gripper_open"] else "closed"
                    print(f"{arm_name} gripper: {state}", flush=True)
            if event.input == self._reset_key:
                for arm in self._arms.values():
                    arm["target_position"] = arm["home_position"].copy()
                    arm["target_quat"] = arm["home_quat"].copy()
                print("Arm teleop targets reset to the ready pose", flush=True)
        elif event.type == event_type.KEY_RELEASE:
            self._held.discard(event.input)
        return True

    def _integrate_held_keys(self, frame_duration: float) -> None:
        linear_step = self._linear_speed * frame_duration
        angular_step = self._angular_speed * frame_duration
        for key in list(self._held):
            for arm in self._arms.values():
                direction = arm["linear_map"].get(key)
                if direction is not None:
                    arm["target_position"] = (
                        arm["target_position"] + direction * linear_step
                    )
                axis = arm["angular_map"].get(key)
                if axis is not None:
                    # Rotate about the robot-base axis (pre-multiply in the
                    # parent frame).
                    quat = _quat_mul(
                        _axis_angle_quat(axis, angular_step),
                        arm["target_quat"],
                    )
                    arm["target_quat"] = quat / np.linalg.norm(quat)

    def _apply_gripper_targets(self, controller) -> None:
        gripper_targets: dict[int, float] = {}
        for arm in self._arms.values():
            driver_index = arm["gripper_driver_index"]
            if driver_index is None:
                continue
            position = self._gripper_positions[arm["gripper_open"]]
            gripper_targets[driver_index] = position
            for coupled_index, multiplier in arm["gripper_coupled"].items():
                gripper_targets[coupled_index] = position * multiplier
        if not gripper_targets:
            return
        indices = np.array(sorted(gripper_targets), dtype=np.int64)
        controller.apply_action(
            ArticulationAction(
                joint_positions=np.array(
                    [gripper_targets[i] for i in indices], dtype=np.float32
                ),
                joint_indices=indices,
            )
        )

    def apply(self, frame_duration: float | None = None) -> None:
        """Advance targets from held keys and apply one merged action.

        frame_duration is the wall-clock period of one teleop loop
        iteration: world.step(render=True) advances rendering_dt (several
        physics steps), so the loop runs at render rate in GUI sessions and
        at physics rate headless. Defaults to the physics dt.
        """
        if frame_duration is None:
            frame_duration = self._physics_dt
        self._integrate_held_keys(frame_duration)

        joint_positions = self.robot.get_joint_positions()
        root_pos, root_quat = self.robot.get_world_pose()
        root_pos = np.array(root_pos, dtype=float)
        root_quat = np.array(root_quat, dtype=float)
        spine_offset = np.array([0.0, 0.0, self._spine_lift(joint_positions)])
        base_pos, base_quat = _compose_world_pose(
            root_pos,
            root_quat,
            spine_offset,
            np.array([1.0, 0.0, 0.0, 0.0]),
        )

        positions: dict[int, float] = {}
        velocities: dict[int, float] = {}
        for arm in self._arms.values():
            rmpflow = arm["rmpflow"]
            rmpflow.set_robot_base_pose(base_pos, base_quat)
            world_pos, world_quat = _compose_world_pose(
                root_pos,
                root_quat,
                arm["target_position"],
                arm["target_quat"],
            )
            rmpflow.set_end_effector_target(world_pos, world_quat)
            # get_next_articulation_action steps the policy itself (the
            # older .update() API is gone in Isaac Sim 5.x); with
            # ignore_robot_state_updates=True it advances the internal
            # rollout by frame_duration.
            action = arm["policy"].get_next_articulation_action(frame_duration)
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

        controller = self.robot.get_articulation_controller()
        if positions:
            indices = np.array(sorted(positions), dtype=np.int64)
            controller.apply_action(
                ArticulationAction(
                    joint_positions=np.array(
                        [positions[i] for i in indices], dtype=np.float32
                    ),
                    joint_velocities=np.array(
                        [velocities[i] for i in indices], dtype=np.float32
                    ),
                    joint_indices=indices,
                )
            )
        self._apply_gripper_targets(controller)

    def reset_targets(self) -> None:
        """Re-initialize both arm targets from FK of the live joint state
        and reopen the grippers (for scene resets between episodes)."""
        for arm in self._arms.values():
            arm["gripper_open"] = True
            rmpflow = arm["rmpflow"]
            # Drop the internal rollout state so the policy does not glide
            # back from the pre-reset pose.
            if hasattr(rmpflow, "reset"):
                rmpflow.reset()
        self._init_home_targets()


class IsaacSimRosBridge(Node):
    def __init__(
        self,
        groups: list[JointGroup],
        *,
        node_name: str = "isaacsim_fr3duo_teleop_bridge",
        publish_recording_topics: bool = False,
    ):
        super().__init__(node_name)
        self.groups = groups
        self.latest_commands: dict[str, dict[str, float]] = {
            group.label: {} for group in groups
        }
        self._latest_pedal_state = "NONE"
        self._latest_pedal_time_sec = None
        # Applied base twist of the current loop iteration (body frame),
        # stored by run_teleop_loop so it can be republished for recording.
        self.last_base_twist = (0.0, 0.0, 0.0)
        self._recording_enabled = bool(publish_recording_topics)
        self._ee_prims = None  # lazily resolved link8 prims, {side: UsdPrim}
        if self._recording_enabled:
            # The clock topic is published from the main loop's
            # world.current_time (physics time, rebases to 0 on scene
            # resets). The OmniGraph IsaacReadSimulationTime route was
            # abandoned: its monotonic time-sample lookups produce jumping
            # values after a world.stop()/reset() ("onStop: Cleared time
            # samples"). The topic is /isaac/clock rather than /clock
            # because the robot USD's embedded ROS_JointStates graph leaks
            # a ros2_publish_clock publisher (instantiated during stage
            # build, before _deactivate_embedded_graphs runs) that keeps
            # emitting 0.0 on /clock every tick.
            self._clock_pub = self.create_publisher(Clock, CLOCK_TOPIC, 10)
            self._applied_commands_pub = self.create_publisher(
                JointState, APPLIED_COMMANDS_TOPIC, 10
            )
            self._full_states_pub = self.create_publisher(
                JointState, FULL_STATES_TOPIC, 10
            )
            self._odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
            self._cmd_vel_applied_pub = self.create_publisher(
                Twist, CMD_VEL_APPLIED_TOPIC, 10
            )
            self._ee_pose_pubs = {
                side: self.create_publisher(PoseStamped, topic, 10)
                for side, topic in EE_POSE_TOPICS.items()
            }
        self._state_publishers = {
            group.label: self.create_publisher(
                JointState, group.state_topic, 10
            )
            for group in groups
        }
        self._command_subscriptions = []
        for group in groups:
            for topic in group.command_topics:
                sub = self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, label=group.label: self._on_joint_command(
                        label, msg
                    ),
                    10,
                )
                self._command_subscriptions.append(sub)
        self._pedal_sub = self.create_subscription(
            String,
            PEDAL_STATE_TOPIC,
            self._on_pedal_state,
            10,
        )
        # Log the loaded contract once; the command topics are duplicated in
        # the task1 helper services, so this is the quickest drift check.
        command_topics = [t for g in groups for t in g.command_topics]
        state_topics = [g.state_topic for g in groups]
        self.get_logger().info(
            f"Topic contract ({TOPICS_YAML}): "
            f"commands={command_topics} states={state_topics} "
            f"pedal={PEDAL_STATE_TOPIC} recording={self._recording_enabled}"
        )

    def _on_joint_command(self, label: str, msg: JointState):
        command = self.latest_commands[label]
        for idx, name in enumerate(msg.name):
            if idx >= len(msg.position):
                break
            if math.isfinite(float(msg.position[idx])):
                command[name] = float(msg.position[idx])

    def _on_pedal_state(self, msg: String):
        state = msg.data.strip().upper().replace(" ", "")
        self._latest_pedal_state = state or "NONE"
        self._latest_pedal_time_sec = self.get_clock().now().nanoseconds * 1e-9

    def pedal_base_twist(
        self,
        linear_speed_mps: float,
        angular_speed_radps: float,
        timeout_sec: float,
    ) -> tuple[float, float, float]:
        if self._latest_pedal_time_sec is None:
            return 0.0, 0.0, 0.0
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if (
            timeout_sec >= 0.0
            and now_sec - self._latest_pedal_time_sec > timeout_sec
        ):
            self._latest_pedal_state = "NONE"
            return 0.0, 0.0, 0.0
        state = self._latest_pedal_state
        # Forward/back tokens are emitted by keyboard_to_base.py (w/s
        # keys); the foot pedal only produces the strafe/yaw tokens below.
        if state == "FWD":
            return linear_speed_mps, 0.0, 0.0
        if state == "BACK":
            return -linear_speed_mps, 0.0, 0.0
        if state == "A":
            return 0.0, linear_speed_mps, 0.0
        if state == "B":
            return 0.0, -linear_speed_mps, 0.0
        if state in {"A+C", "C+A"}:
            return 0.0, 0.0, angular_speed_radps
        if state in {"B+C", "C+B"}:
            return 0.0, 0.0, -angular_speed_radps
        return 0.0, 0.0, 0.0

    def apply_commands(
        self,
        robot: SingleArticulation,
        group_indices: dict[str, dict[str, int]],
        coupled_indices: dict[str, dict[int, float]],
    ):
        targets: dict[int, float] = {}
        for group in self.groups:
            resolved = group_indices.get(group.label, {})
            coupled = coupled_indices.get(group.label)
            for requested_name, position in self.latest_commands[
                group.label
            ].items():
                joint_index = resolved.get(requested_name)
                if joint_index is None:
                    continue
                targets[joint_index] = position
                if coupled:
                    for coupled_index, multiplier in coupled.items():
                        targets[coupled_index] = position * multiplier

        if not targets:
            return

        indices = np.array(sorted(targets), dtype=np.int64)
        positions = np.array([targets[i] for i in indices], dtype=np.float32)
        robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_positions=positions, joint_indices=indices
            )
        )

    def publish_states(
        self,
        robot: SingleArticulation,
        group_indices: dict[str, dict[str, int]],
    ):
        joint_pos = robot.get_joint_positions()
        joint_vel = robot.get_joint_velocities()

        stamp = self.get_clock().now().to_msg()
        for group in self.groups:
            msg = JointState()
            msg.header.stamp = stamp
            names = []
            positions = []
            velocities = []
            for requested_name in group.requested_names:
                joint_index = group_indices.get(group.label, {}).get(
                    requested_name
                )
                if joint_index is None:
                    continue
                names.append(requested_name)
                positions.append(float(joint_pos[joint_index]))
                velocities.append(float(joint_vel[joint_index]))
            msg.name = names
            msg.position = positions
            msg.velocity = velocities
            msg.effort = [0.0] * len(names)
            self._state_publishers[group.label].publish(msg)

    def _resolve_ee_prims(self) -> dict:
        stage = omni.usd.get_context().get_stage()
        prims = {}
        wanted = dict(EE_POSE_PRIM_NAMES)
        for prim in stage.Traverse():
            for side, prim_name in list(wanted.items()):
                if prim.GetName() == prim_name and prim.IsA(UsdGeom.Xformable):
                    prims[side] = prim
                    del wanted[side]
            if not wanted:
                break
        if wanted:
            print(
                "Warning: EE pose publishing disabled for "
                f"{sorted(wanted)}: prims {sorted(wanted.values())} "
                "not found on the stage",
                file=sys.stderr,
            )
        return prims

    def publish_clock(self, sim_time: float) -> None:
        """Publish simulation time on the clock topic (recorders pace on
        it)."""
        if not self._recording_enabled:
            return
        msg = Clock()
        msg.clock = rclpy.time.Time(
            nanoseconds=max(int(sim_time * 1e9), 0)
        ).to_msg()
        self._clock_pub.publish(msg)

    def publish_recording_streams(
        self, robot: SingleArticulation, sim_time: float
    ) -> None:
        """Publish the applied joint targets, base odometry, applied base
        twist, and link8 EE poses (all stamped with simulation time)."""
        if not self._recording_enabled:
            return
        stamp = rclpy.time.Time(
            nanoseconds=max(int(sim_time * 1e9), 0)
        ).to_msg()

        # Post-arbitration joint position targets, regardless of source
        # (ROS commands, keyboard RMPflow, spine keys, gripper toggles).
        applied = robot.get_articulation_controller().get_applied_action()
        applied_positions = (
            applied.joint_positions if applied is not None else None
        )
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(robot.dof_names)
        if applied_positions is None:
            msg.position = [math.nan] * len(msg.name)
        else:
            msg.position = [
                float(p) if p is not None else math.nan
                for p in applied_positions
            ]
        self._applied_commands_pub.publish(msg)

        # Full measured joint state (the per-group /isaac/*_joint_states
        # topics omit the spine and base joints).
        joint_pos = robot.get_joint_positions()
        joint_vel = robot.get_joint_velocities()
        full = JointState()
        full.header.stamp = stamp
        full.name = list(robot.dof_names)
        full.position = [float(p) for p in joint_pos]
        full.velocity = [float(v) for v in joint_vel]
        self._full_states_pub.publish(full)

        # Base odometry: world pose + body-frame velocities.
        root_pos, root_quat = robot.get_world_pose()
        qw, qx, qy, qz = (float(v) for v in root_quat)
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
        )
        lin_world = np.asarray(robot.get_linear_velocity(), dtype=float)
        ang_world = np.asarray(robot.get_angular_velocity(), dtype=float)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(root_pos[0])
        odom.pose.pose.position.y = float(root_pos[1])
        odom.pose.pose.position.z = float(root_pos[2])
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.twist.twist.linear.x = (
            cos_yaw * lin_world[0] + sin_yaw * lin_world[1]
        )
        odom.twist.twist.linear.y = (
            -sin_yaw * lin_world[0] + cos_yaw * lin_world[1]
        )
        odom.twist.twist.linear.z = float(lin_world[2])
        odom.twist.twist.angular.z = float(ang_world[2])
        self._odom_pub.publish(odom)

        twist = Twist()
        vx, vy, wz = self.last_base_twist
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(wz)
        self._cmd_vel_applied_pub.publish(twist)

        # link8 end-effector world poses (matches the data contract's
        # wrench_frame convention).
        if self._ee_prims is None:
            self._ee_prims = self._resolve_ee_prims()
        if self._ee_prims:
            xform_cache = UsdGeom.XformCache()
            for side, prim in self._ee_prims.items():
                world = xform_cache.GetLocalToWorldTransform(prim)
                translation = world.ExtractTranslation()
                rotation = world.ExtractRotationQuat()
                pose = PoseStamped()
                pose.header.stamp = stamp
                pose.header.frame_id = "world"
                pose.pose.position.x = float(translation[0])
                pose.pose.position.y = float(translation[1])
                pose.pose.position.z = float(translation[2])
                pose.pose.orientation.w = float(rotation.GetReal())
                imaginary = rotation.GetImaginary()
                pose.pose.orientation.x = float(imaginary[0])
                pose.pose.orientation.y = float(imaginary[1])
                pose.pose.orientation.z = float(imaginary[2])
                self._ee_pose_pubs[side].publish(pose)


def _add_dome_light(stage, prim_path: str = "/World/Light") -> None:
    light = UsdLux.DomeLight.Define(stage, prim_path)
    light.CreateIntensityAttr(500.0)
    light.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))


def _place_objects(stage, prim_path: str, position, yaw_deg: float) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(
            f"Warning: objects prim not found for placement: {prim_path}",
            file=sys.stderr,
        )
        return
    xform_api = UsdGeom.XformCommonAPI(prim)
    xform_api.SetTranslate(Gf.Vec3d(*[float(v) for v in position]))
    xform_api.SetRotate(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))


def _apply_ready_pose(
    robot: SingleArticulation, actual_names: list[str]
) -> None:
    actual_by_name = {name: idx for idx, name in enumerate(actual_names)}
    indices = []
    positions = []
    for joint_name, position in ARM_READY_POSE.items():
        joint_index = _resolve_joint_index(joint_name, actual_by_name)
        if joint_index is None:
            continue
        indices.append(joint_index)
        positions.append(position)
    if not indices:
        return
    indices = np.array(indices, dtype=np.int64)
    positions = np.array(positions, dtype=np.float32)
    robot.set_joint_positions(positions, joint_indices=indices)
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=positions, joint_indices=indices)
    )


def prepare_robot_prim(robot_prim_path: str, args) -> None:
    """USD-level fixes to run after referencing the robot, before
    world.reset()."""
    _fix_single_articulation_root(robot_prim_path)
    if args.disable_embedded_omnigraph:
        _deactivate_embedded_graphs(robot_prim_path)
    if args.configure_base_drives:
        _configure_drives(robot_prim_path, _base_drive_gains)


def setup_robot_control(
    robot: SingleArticulation, groups: list[JointGroup], args
):
    """Resolve joint indices, apply the ready pose, and build the controllers.

    Returns (group_indices, coupled_indices, steering_ids, drive_ids,
    spine_controller, arm_keyboard_teleop).
    """
    actual_joint_names = list(robot.dof_names)
    group_indices = _resolve_group_indices(groups, actual_joint_names)
    coupled_indices = _resolve_coupled_indices(
        groups,
        group_indices,
        actual_joint_names,
        include_robotiq_coupled=args.apply_gripper_coupled_targets,
    )
    missing = [
        f"{group.label}:{name}"
        for group in groups
        for name in group.requested_names
        if name not in group_indices.get(group.label, {})
    ]
    if missing:
        print(
            "Warning: unresolved joints:", ", ".join(missing), file=sys.stderr
        )
    print("Actual joint names:", ", ".join(actual_joint_names))

    _apply_ready_pose(robot, actual_joint_names)

    try:
        steering_ids, drive_ids = _find_drive_joint_ids(actual_joint_names)
        print(
            "Pedal base control enabled: "
            f"topic={PEDAL_STATE_TOPIC} steering_ids={steering_ids} "
            f"drive_ids={drive_ids} "
            f"linear_speed={args.pedal_linear_speed:.3f} m/s "
            f"angular_speed={args.pedal_angular_speed:.3f} rad/s "
            f"timeout={args.pedal_timeout:.3f} s"
        )
    except RuntimeError as exc:
        steering_ids, drive_ids = [], []
        print(f"Warning: pedal base control disabled: {exc}", file=sys.stderr)

    spine_keyboard_controller = None
    if args.spine_keyboard_control:
        if "franka_spine_vertical_joint" in actual_joint_names:
            spine_keyboard_controller = SpineKeyboardController(
                robot,
                actual_joint_names,
                step_m=args.spine_keyboard_step,
                min_m=args.spine_keyboard_min,
                max_m=args.spine_keyboard_max,
            )
        else:
            print(
                "Warning: spine keyboard control disabled: "
                "franka_spine_vertical_joint not found",
                file=sys.stderr,
            )

    arm_keyboard_teleop = None
    if args.arm_keyboard_teleop and args.headless:
        # Kit still creates an app-window keyboard headless, so the teleop
        # would "work" while silently blocking ROS arm commands.
        print(
            "Warning: dual-arm keyboard teleop disabled in headless "
            "sessions; ROS arm commands stay active.",
            file=sys.stderr,
        )
    elif args.arm_keyboard_teleop:
        try:
            arm_keyboard_teleop = DualArmKeyboardTeleop(
                robot,
                actual_joint_names,
                group_indices,
                coupled_indices,
                args,
            )
        except Exception as exc:  # noqa: BLE001 - fall back to ROS commands
            print(
                f"Warning: dual-arm keyboard teleop disabled: {exc}",
                file=sys.stderr,
            )

    return (
        group_indices,
        coupled_indices,
        steering_ids,
        drive_ids,
        spine_keyboard_controller,
        arm_keyboard_teleop,
    )


def run_teleop_loop(
    simulation_app,
    world,
    robot: SingleArticulation,
    groups: list[JointGroup],
    group_indices,
    coupled_indices,
    steering_ids,
    drive_ids,
    spine_keyboard_controller,
    arm_keyboard_teleop,
    args,
    *,
    force_render: bool = False,
    tick_callbacks=(),
) -> None:
    """Blocking ROS <-> sim loop; owns rclpy and app shutdown.

    force_render keeps rendering in headless sessions so render-product
    consumers (e.g. the task2 eval camera OmniGraph) still publish.

    tick_callbacks: objects called as cb.tick(sim_time) once per loop
    iteration (after world.step); an optional cb.bind(node) runs once before
    the loop so callbacks can create publishers on the bridge node. Used by
    the recording helpers in scripts/recording/.
    """
    # The ros2 bridge extension may already have initialized the default
    # context.
    if not rclpy.ok():
        rclpy.init()
    node = IsaacSimRosBridge(
        groups,
        publish_recording_topics=getattr(
            args, "publish_recording_topics", False
        ),
    )
    for callback in tick_callbacks:
        if hasattr(callback, "bind"):
            callback.bind(node)
    publish_period = 1.0 / max(args.ros_publish_rate, 1.0)
    next_publish_time = 0.0
    # When rendering, world.step advances one rendering_dt (several physics
    # steps), so the loop iterates at render rate; headless-without-render
    # iterates at physics rate.
    rendering = force_render or not args.headless
    loop_dt = (
        1.0 / max(args.render_hz, 1.0)
        if rendering
        else 1.0 / max(args.physics_hz, 1.0)
    )

    try:
        while simulation_app.is_running() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            arm_teleop_active = (
                arm_keyboard_teleop is not None
                and arm_keyboard_teleop.available
            )
            # Exclusive arbitration: the keyboard teleop replaces the ROS
            # arm and gripper commands (the only groups apply_commands
            # handles); joint states are still published below.
            if not arm_teleop_active:
                node.apply_commands(robot, group_indices, coupled_indices)
            vx, vy, wz = node.pedal_base_twist(
                args.pedal_linear_speed,
                args.pedal_angular_speed,
                args.pedal_timeout,
            )
            node.last_base_twist = (vx, vy, wz)
            if steering_ids and drive_ids:
                steering_targets, drive_targets = _compute_drive_targets(
                    robot.get_joint_positions(),
                    steering_ids,
                    vx,
                    vy,
                    wz,
                )
                controller = robot.get_articulation_controller()
                controller.apply_action(
                    ArticulationAction(
                        joint_positions=steering_targets,
                        joint_indices=np.array(steering_ids),
                    )
                )
                controller.apply_action(
                    ArticulationAction(
                        joint_velocities=drive_targets,
                        joint_indices=np.array(drive_ids),
                    )
                )
            if spine_keyboard_controller is not None:
                spine_keyboard_controller.apply()
            if arm_teleop_active:
                arm_keyboard_teleop.apply(loop_dt)
            # With render=False each iteration advances a single physics_dt, so
            # headless keeps the fastest possible ROS command/state loop.
            world.step(render=rendering)
            sim_time = float(world.current_time)
            # Every iteration, not publish_period-throttled: the recorder
            # paces its frame sampling on this.
            node.publish_clock(sim_time)
            for callback in tick_callbacks:
                callback.tick(sim_time)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now >= next_publish_time:
                node.publish_states(robot, group_indices)
                node.publish_recording_streams(robot, sim_time)
                next_publish_time = now + publish_period
    finally:
        node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()
