#!/usr/bin/env python3
"""IsaacLab Newton/MJWarp ROS bridge for the fr3duo mobile USD.

This script is meant to run inside the IsaacLab ROS2 container, while the
existing franka_isaacSim ROS helper containers keep running on the host
network.  It intentionally publishes/subscribes the same `/isaac/*` topics as
the older Isaac Sim bridge so `ros_republisher`, `position_controller`,
`browser_controller`, and `gello_pedal_teleop` can be reused unchanged.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from isaaclab.app import AppLauncher
import traceback

def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd-path",
        default="../assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd",
        help="USD file to load into the IsaacLab scene.",
    )
    parser.add_argument(
        "--embodiment",
        default="fr3duo_mobile",
        help="Embodiment name to load configuration from (default: fr3duo_mobile)",
    )
    parser.add_argument(
        "--franka-root",
        default="/workspace/franka_isaacSim",
        help="franka_isaacSim repository mount inside the isaac-lab-ros2_jazzy container.",
    )
    parser.add_argument(
        "--robot-prim-path",
        default="{ENV_REGEX_NS}/Robot",
        help="Stage prim path for the robot. /World/envs/env_0/Robot.",
    )
    parser.add_argument(
        "--room-usd-path",
        default="../assets/robot_room_v2/robot_room_v2.usdc",
        help="Room USD/USDC path to load under {ENV_REGEX_NS}/Room, relative to --franka-root unless absolute.",
    )
    parser.add_argument(
        "--no-room",
        action="store_true",
        help="Do not load the room USD.",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not subscribe to /isaac/browser/* command topics.",
    )
    parser.add_argument(
        "--ros-publish-rate",
        type=float,
        default=60.0,
        help="Joint-state publish rate in Hz for /isaac/*_joint_states",
    )
    parser.add_argument(
        "--pedal-linear-speed",
        type=float,
        default=0.5,
        help="Base lateral translation speed in m/s used for pedal A/B commands.",
    )
    parser.add_argument(
        "--pedal-angular-speed",
        type=float,
        default=1.2,
        help="Base yaw speed in rad/s used for pedal A+C/B+C commands.",
    )
    parser.add_argument(
        "--pedal-timeout",
        type=float,
        default=1.0,
        help="Seconds without a new /pedal/state message before forcing the base command to NONE.",
    )
    parser.add_argument(
        "--spine-keyboard-step",
        type=float,
        default=0.01,
        help="Height target increment in meters for each Up/Down key press or repeat.",
    )
    parser.add_argument(
        "--spine-keyboard-min",
        type=float,
        default=0.0,
        help="Minimum franka_spine_vertical_joint target in meters for keyboard control.",
    )
    parser.add_argument(
        "--spine-keyboard-max",
        type=float,
        default=0.850,
        help="Maximum franka_spine_vertical_joint target in meters for keyboard control.",
    )
    parser.add_argument(
        "--physics-hz",
        type=float,
        default=240.0,
        help="Isaac physics stepping frequency in Hz",
    )
    parser.add_argument(
        "--render-hz",
        type=float,
        default=60.0,
        help="Render frequency in Hz (physics steps are decimated to this rate)",
    )
    parser.add_argument(
        "--physics-substeps",
        type=int,
        default=2,
        help="Number of physics substeps requested from Isaac",
    )
    parser.add_argument("--mj-njmax", type=int, default=2048)
    parser.add_argument("--mj-nconmax", type=int, default=512)
    parser.add_argument("--mj-cone", default="pyramidal")
    parser.add_argument("--mj-integrator", default="implicitfast")
    parser.add_argument("--mj-impratio", type=float, default=1.0)
    parser.add_argument(
        "--camera-position",
        type=float,
        nargs=3,
        default=(5.0, 0.0, 3.0),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--camera-target",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--with-cable",
        action="store_true",
        help="Run the raw Newton VBD board-cable world alongside the IsaacLab robot.",
    )
    parser.add_argument(
        "--cable-config-path",
        default="cable_world/configs/table_board_fixture_cable.yaml",
        help="Cable VBD config path, relative to --franka-root unless absolute.",
    )
    parser.add_argument(
        "--cable-gripper-config-path",
        default="cable_world/configs/gripper.yaml",
        help="Cable gripper config path, relative to --franka-root unless absolute.",
    )
    parser.add_argument(
        "--cable-device",
        default=None,
        help="Device for the raw Newton cable world. Defaults to the IsaacLab --device value.",
    )
    parser.add_argument(
        "--cable-robotiq-finger-targets",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish live Robotiq inner-finger poses to the cable world as kinematic collision targets.",
    )
    parser.add_argument(
        "--cable-robotiq-finger-target-topic",
        default="/isaac/robotiq_finger_targets",
        help="PointCloud topic carrying cable-world Robotiq finger target poses.",
    )
    parser.add_argument(
        "--cable-robotiq-finger-size",
        type=float,
        nargs=3,
        default=(0.007, 0.010, 0.028),
        metavar=("X", "Y", "Z"),
        help="Collision box size in meters used for each Robotiq inner finger target.",
    )
    parser.add_argument(
        "--cable-robotiq-contact-x-offset",
        type=float,
        default=0.0,
        help="Additional local X offset in meters from the Robotiq visual bbox center to the red contact box center.",
    )
    parser.add_argument(
        "--cable-robotiq-contact-y-offset",
        type=float,
        default=0.024,
        help="Absolute local Y offset in meters from each Robotiq inner_finger frame to the red contact box center.",
    )
    parser.add_argument(
        "--cable-robotiq-contact-z-offset",
        type=float,
        default=-0.010,
        help="Additional local Z offset in meters from the Robotiq visual bbox center to the red contact box center.",
    )
    parser.add_argument(
        "--cable-robotiq-invert-opening",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Use the opposite local-Y contact side on each Robotiq inner finger when red boxes open opposite to the visual pads.",
    )
    parser.add_argument(
        "--cable-world-position-offset",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="IsaacLab-world translation applied to cable/table/board visuals; robot gripper poses are shifted by the inverse before driving VBD.",
    )
    parser.add_argument(
        "--cable-world-yaw-deg",
        type=float,
        default=0.0,
        help="IsaacLab-world yaw rotation in degrees applied to the whole cable/table/board visual world.",
    )
    parser.add_argument(
        "--show-table-board-fixture-collisions",
        action="store_true",
        help="Show collision meshes under /World/TableBoardFixtureVisual for debugging.",
    )
    AppLauncher.add_app_launcher_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import omni.usd
from pxr import UsdPhysics

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import CameraCfg
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

try:
    from isaaclab_visualizers.kit import KitVisualizerCfg
except Exception:  # pragma: no cover - optional extension/package
    KitVisualizerCfg = None

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import ChannelFloat32, JointState, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import String

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML should exist in the ROS image
    yaml = None


def _path_relative_to_franka_root(path_value: str, franka_root: Path) -> Path:
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return path
    return franka_root / path


def _prepare_robot_room_texture_links(room_usd_path: Path) -> None:
    room_dir = room_usd_path.parent
    expected = room_dir / "textures" / "color_0C0C0C.exr"
    source = room_dir / "whole_scene" / "color_0C0C0C.exr"
    if expected.exists() or not source.exists():
        return
    try:
        expected.parent.mkdir(parents=True, exist_ok=True)
        expected.symlink_to(Path("..") / "whole_scene" / "color_0C0C0C.exr")
    except OSError as exc:
        print(
            "Warning: failed to create robot room texture link "
            f"{expected} -> {source}: {exc}",
            file=sys.stderr,
        )


LEFT_FALLBACK_JOINTS = [
    "left_fr3v2_joint1",
    "left_fr3v2_joint2",
    "left_fr3v2_joint3",
    "left_fr3v2_joint4",
    "left_fr3v2_joint5",
    "left_fr3v2_joint6",
    "left_fr3v2_joint7",
]

RIGHT_FALLBACK_JOINTS = [
    "right_fr3v2_joint1",
    "right_fr3v2_joint2",
    "right_fr3v2_joint3",
    "right_fr3v2_joint4",
    "right_fr3v2_joint5",
    "right_fr3v2_joint6",
    "right_fr3v2_joint7",
]

LEFT_GRIPPER_DRIVER = "left_right_finger_joint"
RIGHT_GRIPPER_DRIVER = "right_right_finger_joint"

PEDAL_STATE_TOPIC = "/pedal/state"
WHEEL_RADIUS_M = 0.05
MAX_WHEEL_SPEED_RADPS = 18.0
STOP_EPS = 1.0e-4
STEERING_FULL_SPEED_ERROR_RAD = math.radians(8.0)
STEERING_ZERO_SPEED_ERROR_RAD = math.radians(35.0)


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
    command_topics: List[str]
    requested_names: List[str]


def _load_joint_groups(*, include_browser_commands: bool = True) -> List[JointGroup]:
    left_arm = list(LEFT_FALLBACK_JOINTS)
    right_arm = list(RIGHT_FALLBACK_JOINTS)
    left_gripper = LEFT_GRIPPER_DRIVER
    right_gripper = RIGHT_GRIPPER_DRIVER

    return [
        JointGroup(
            label="left_arm",
            state_topic="/isaac/left_joint_states",
            command_topics=[
                "/isaac/left_joint_commands",
                *(
                    ["/isaac/browser/left_joint_commands"]
                    if include_browser_commands
                    else []
                ),
            ],
            requested_names=left_arm,
        ),
        JointGroup(
            label="right_arm",
            state_topic="/isaac/right_joint_states",
            command_topics=[
                "/isaac/right_joint_commands",
                *(
                    ["/isaac/browser/right_joint_commands"]
                    if include_browser_commands
                    else []
                ),
            ],
            requested_names=right_arm,
        ),
        JointGroup(
            label="left_gripper",
            state_topic="/isaac/left_robotiq_joint_states",
            command_topics=[
                "/isaac/left_robotiq_joint_commands",
                *(
                    ["/isaac/browser/left_robotiq_joint_commands"]
                    if include_browser_commands
                    else []
                ),
            ],
            requested_names=[left_gripper],
        ),
        JointGroup(
            label="right_gripper",
            state_topic="/isaac/right_robotiq_joint_states",
            command_topics=[
                "/isaac/right_robotiq_joint_commands",
                *(
                    ["/isaac/browser/right_robotiq_joint_commands"]
                    if include_browser_commands
                    else []
                ),
            ],
            requested_names=[right_gripper],
        ),
    ]


def _resolve_group_indices(groups: List[JointGroup], actual_names: List[str]) -> Dict[str, Dict[str, int]]:
    actual_by_name = {name: idx for idx, name in enumerate(actual_names)}
    resolved: Dict[str, Dict[str, int]] = {}
    for group in groups:
        group_map = {}
        for requested_name in group.requested_names:
            if requested_name in actual_by_name:
                group_map[requested_name] = actual_by_name[requested_name]
        resolved[group.label] = group_map
    return resolved


NEWTON_REVERSED_FIXED_JOINTS = (
    "argo_drive_front_fixed_joint",
    "base_joint",
    "zed_mini_camera_joint",
)



def _swap_relationship_targets(prim, rel0_name: str, rel1_name: str) -> bool:
    rel0 = prim.GetRelationship(rel0_name)
    rel1 = prim.GetRelationship(rel1_name)
    targets0 = rel0.GetTargets()
    targets1 = rel1.GetTargets()
    if not targets0 or not targets1:
        return False
    rel0.SetTargets(targets1)
    rel1.SetTargets(targets0)
    return True


def _swap_attr_values(prim, attr0_name: str, attr1_name: str) -> None:
    attr0 = prim.GetAttribute(attr0_name)
    attr1 = prim.GetAttribute(attr1_name)
    if not attr0.IsValid() or not attr1.IsValid():
        return
    value0 = attr0.Get()
    value1 = attr1.Get()
    attr0.Set(value1)
    attr1.Set(value0)


def _env_robot_prim_path(robot_prim_path: str, env_index: int = 0) -> str:
    return robot_prim_path.replace("{ENV_REGEX_NS}", f"/World/envs/env_{env_index}")


def _fix_single_articulation_root(robot_prim_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("Warning: cannot patch articulation roots: no USD stage", file=sys.stderr)
        return
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"Warning: cannot patch articulation roots: robot prim not found: {robot_prim_path}", file=sys.stderr)
        return

    root_prims = []
    prim_stack = [robot_prim]
    while prim_stack:
        prim = prim_stack.pop()
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            root_prims.append(prim)
        prim_stack.extend(reversed(prim.GetChildren()))
    if len(root_prims) <= 1:
        return

    keep_prim = None
    for preferred_path in (f"{robot_prim_path}/base", f"{robot_prim_path}/base_link"):
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


def _fix_newton_reversed_fixed_joints(robot_prim_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("Warning: cannot patch reversed joints: no USD stage", file=sys.stderr)
        return
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"Warning: cannot patch reversed joints: robot prim not found: {robot_prim_path}", file=sys.stderr)
        return

    wanted = set(NEWTON_REVERSED_FIXED_JOINTS)
    patched = []
    seen = set()
    prim_stack = [robot_prim]
    while prim_stack:
        prim = prim_stack.pop()
        prim_stack.extend(reversed(prim.GetChildren()))
        if prim.GetName() not in wanted:
            continue
        seen.add(prim.GetName())
        joint_path = str(prim.GetPath())
        if prim.GetTypeName() != "PhysicsFixedJoint":
            print(
                f"Warning: skipping reversed-joint patch for {joint_path}: "
                f"expected PhysicsFixedJoint, got {prim.GetTypeName()}",
                file=sys.stderr,
            )
            continue
        if not _swap_relationship_targets(prim, "physics:body0", "physics:body1"):
            print(f"Warning: could not swap body0/body1 for {joint_path}: missing targets", file=sys.stderr)
            continue
        _swap_attr_values(prim, "physics:localPos0", "physics:localPos1")
        _swap_attr_values(prim, "physics:localRot0", "physics:localRot1")
        patched.append(joint_path)

    missing = wanted - seen
    if missing:
        print("Warning: reversed-joint patch names not found: " + ", ".join(sorted(missing)), file=sys.stderr)
    if patched:
        print("Patched Newton-reversed fixed joints:")
        for joint_path in patched:
            print(f"  {joint_path}")


def _load_home_joint_positions(franka_root: Path, embodiment: str) -> dict[str, float]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to read joint_parametrization.yaml")

    path = franka_root / "assets" / "embodiments" / embodiment / "joint_parametrization.yaml"
    if not path.exists():
        raise FileNotFoundError(f"Joint parametrization file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    wanted = LEFT_FALLBACK_JOINTS + RIGHT_FALLBACK_JOINTS
    wanted_set = set(wanted)
    home_positions: dict[str, float] = {}
    arm_groups = cfg.get("arm_groups", {}) or {}
    for side in ("left", "right"):
        for joint_cfg in (arm_groups.get(side, {}) or {}).get("controllable_joints", []) or []:
            if not isinstance(joint_cfg, dict):
                continue
            name = joint_cfg.get("name")
            if name not in wanted_set:
                continue
            if "home_position_rad" not in joint_cfg:
                raise ValueError(f"Missing home_position_rad for joint {name} in {path}")
            home_positions[str(name)] = float(joint_cfg["home_position_rad"])

    missing = [name for name in wanted if name not in home_positions]
    if missing:
        raise ValueError(
            "Missing home_position_rad entries in joint_parametrization.yaml for: " + ", ".join(missing)
        )

    return {name: home_positions[name] for name in wanted}


def _load_arm_velocity_limits(franka_root: Path, embodiment: str) -> dict[str, float]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to read joint_parametrization.yaml")

    path = franka_root / "assets" / "embodiments" / embodiment / "joint_parametrization.yaml"
    if not path.exists():
        raise FileNotFoundError(f"Joint parametrization file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    wanted = LEFT_FALLBACK_JOINTS + RIGHT_FALLBACK_JOINTS
    wanted_set = set(wanted)
    velocity_limits: dict[str, float] = {}
    arm_groups = cfg.get("arm_groups", {}) or {}
    for side in ("left", "right"):
        for joint_cfg in (arm_groups.get(side, {}) or {}).get("controllable_joints", []) or []:
            if not isinstance(joint_cfg, dict):
                continue
            name = joint_cfg.get("name")
            if name not in wanted_set:
                continue
            if "velocity_limit_rad_s" not in joint_cfg:
                raise ValueError(f"Missing velocity_limit_rad_s for joint {name} in {path}")
            velocity_limits[str(name)] = float(joint_cfg["velocity_limit_rad_s"])

    missing = [name for name in wanted if name not in velocity_limits]
    if missing:
        raise ValueError(
            "Missing velocity_limit_rad_s entries in joint_parametrization.yaml for: " + ", ".join(missing)
        )

    return {name: velocity_limits[name] for name in wanted}


def _load_scaled_joint_drive_configs(franka_root: Path, embodiment: str) -> dict[str, dict[str, float]]:
    if yaml is None:
        raise RuntimeError("PyYAML is required to read joint drive config files")

    embodiment_dir = franka_root / "assets" / "embodiments" / embodiment
    joint_drive_config_path = embodiment_dir / "joint_drive_config.yaml"
    if not joint_drive_config_path.exists():
        raise FileNotFoundError(f"Joint drive config file not found: {joint_drive_config_path}")

    with joint_drive_config_path.open("r", encoding="utf-8") as f:
        joint_drive_config = yaml.safe_load(f) or {}

    joint_drives_meta = joint_drive_config.get("isaac_joint_drives", {}) or {}
    raw_drive_path = joint_drives_meta.get("joint_drive_config", "isaac_joint_drives.yaml")
    drive_path = Path(raw_drive_path)
    if not drive_path.is_absolute():
        drive_path = embodiment_dir / drive_path
    if not drive_path.exists():
        raise FileNotFoundError(f"Isaac joint drive config file not found: {drive_path}")

    with drive_path.open("r", encoding="utf-8") as f:
        drive_config = yaml.safe_load(f) or {}

    scaling = joint_drive_config.get("scaling_parameters", {}) or {}
    arm_scaling = scaling.get("arms", scaling) or {}
    arm_stiffness_scale = float(arm_scaling.get("stiffness_scale", 1.0))
    arm_damping_scale = float(arm_scaling.get("damping_scale", 1.0))
    arm_max_force_scale = float(arm_scaling.get("max_force_scale", 1.0))

    scaled: dict[str, dict[str, float]] = {}
    wanted = set(LEFT_FALLBACK_JOINTS + RIGHT_FALLBACK_JOINTS)
    for joint_name, cfg in (drive_config.get("joint_drives", {}) or {}).items():
        if joint_name not in wanted or not isinstance(cfg, dict):
            continue
        scaled[str(joint_name)] = {
            "effort_limit_sim": float(cfg.get("max_force", 0.0)) * arm_max_force_scale,
            "stiffness": float(cfg.get("stiffness", 0.0)) * arm_stiffness_scale,
            "damping": float(cfg.get("damping", 0.0)) * arm_damping_scale,
        }

    missing = [name for name in LEFT_FALLBACK_JOINTS + RIGHT_FALLBACK_JOINTS if name not in scaled]
    if missing:
        raise ValueError("Missing arm joint drive entries in isaac_joint_drives.yaml for: " + ", ".join(missing))

    print(
        "Loaded arm actuator drive config: "
        f"{drive_path} (scales: k={arm_stiffness_scale:g}, d={arm_damping_scale:g}, "
        f"force={arm_max_force_scale:g})"
    )
    return scaled


def _make_arm_actuator_cfgs(franka_root: Path, embodiment: str) -> dict[str, ImplicitActuatorCfg]:
    drive_cfgs = _load_scaled_joint_drive_configs(franka_root, embodiment)
    velocity_limits = _load_arm_velocity_limits(franka_root, embodiment)

    actuators: dict[str, ImplicitActuatorCfg] = {}
    for joint_name in LEFT_FALLBACK_JOINTS + RIGHT_FALLBACK_JOINTS:
        drive = drive_cfgs[joint_name]
        actuators[f"arm_{joint_name}"] = ImplicitActuatorCfg(
            joint_names_expr=[joint_name],
            effort_limit_sim=drive["effort_limit_sim"],
            velocity_limit_sim=velocity_limits[joint_name],
            stiffness=drive["stiffness"],
            damping=drive["damping"],
        )
    return actuators


GRIPPER_ACTUATOR_JOINTS = [
    "left_outer_knuckle_joint",
    "left_right_inner_finger_joint",
    "left_right_inner_finger_knuckle_joint",
    "left_right_finger_joint",
    "left_left_inner_finger_joint",
    "left_left_inner_finger_knuckle_joint",
    "right_outer_knuckle_joint",
    "right_right_inner_finger_joint",
    "right_right_inner_finger_knuckle_joint",
    "right_right_finger_joint",
    "right_left_inner_finger_joint",
    "right_left_inner_finger_knuckle_joint",
]

GRIPPER_DRIVER_JOINTS = ["left_right_finger_joint", "right_right_finger_joint"]



def _set_angular_drive_type_for_joints(robot_prim_path: str, joint_names: list[str], drive_type: str) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("Warning: cannot patch joint drive types: no USD stage", file=sys.stderr)
        return
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"Warning: cannot patch joint drive types: robot prim not found: {robot_prim_path}", file=sys.stderr)
        return

    wanted = set(joint_names)
    patched = []
    seen = set()
    prim_stack = [robot_prim]
    while prim_stack:
        prim = prim_stack.pop()
        prim_stack.extend(reversed(prim.GetChildren()))
        if prim.GetName() not in wanted:
            continue
        seen.add(prim.GetName())
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetTypeAttr().Set(drive_type)
        patched.append(str(prim.GetPath()))

    missing = wanted - seen
    if missing:
        print("Warning: drive-type patch joint names not found: " + ", ".join(sorted(missing)), file=sys.stderr)
    if patched:
        print(f"Set angular drive type to {drive_type} for:")
        for joint_path in patched:
            print(f"  {joint_path}")


def _make_scene_cfg(usd_path: str, prim_path: str, room_usd_path: str | None = None):
    franka_root = Path(args_cli.franka_root).expanduser()
    embodiment = args_cli.embodiment
    actuators = {
        "base_steering": ImplicitActuatorCfg(
            joint_names_expr=["tmrv0_2_joint_0", "tmrv0_2_joint_2"],
            effort_limit_sim=500.0,
            velocity_limit_sim=20.0,
            stiffness=500.0,
            damping=50.0,
        ),
        "base_drive": ImplicitActuatorCfg(
            joint_names_expr=["tmrv0_2_joint_1", "tmrv0_2_joint_3"],
            effort_limit_sim=500.0,
            velocity_limit_sim=20.0,
            stiffness=0.0,
            damping=5.0,
        ),
        "passive_base": ImplicitActuatorCfg(
            joint_names_expr=[
                "caster_front_left_steering_joint",
                "caster_front_left_joint",
                "caster_rear_right_steering_joint",
                "caster_rear_right_joint",
                "rocker_arm_joint",
            ],
            effort_limit_sim=500.0,
            velocity_limit_sim=20.0,
            stiffness=0.0,
            damping=0.0,
        ),
        "spine": ImplicitActuatorCfg(
            joint_names_expr=["franka_spine_vertical_joint"],
            stiffness=2000,
            damping=500,
        ),
        "grippers": ImplicitActuatorCfg(
            joint_names_expr=GRIPPER_ACTUATOR_JOINTS,
            effort_limit_sim=200,
            velocity_limit_sim=None,
            stiffness=5.0,
            damping=0.5,
        ),
    }
    actuators.update(_make_arm_actuator_cfgs(franka_root, embodiment))

    robot_cfg = ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos=_load_home_joint_positions(
                franka_root,
                embodiment,
            )
        ),
        actuators=actuators,
        actuator_value_resolution_debug_print=True,
    )
    room_cfg = None
    if room_usd_path is not None:
        room_cfg = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Room",
            spawn=sim_utils.UsdFileCfg(usd_path=room_usd_path),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=(1.385, -4.39, 0.0),
                rot=(0.0, 0.0, 0.70710678, 0.70710678),
            ),
        )

    class TeleopSceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
        dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=500.0, color=(0.85, 0.9, 1.0)),
        )
        table_camera = CameraCfg(
            prim_path="{ENV_REGEX_NS}/TableTopCamera",
            update_period=0.05,
            height=720,
            width=1280,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0,
                focus_distance=3.0,
                horizontal_aperture=20.955,
                clipping_range=(0.01, 100.0),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=(1.5, -0.1, 2.7),
                rot=(0.0, 0.0, 0.0, 1.0),
                convention="opengl",
            ),
        )
        robot = robot_cfg
        if room_cfg is not None:
            room = room_cfg

    return TeleopSceneCfg


def _make_visualizer_cfgs():
    if KitVisualizerCfg is None:
        return []
    cfg = KitVisualizerCfg()
    desired_attrs = {
        "viewport_name": "Visualizer Viewport",
        "create_viewport": True,
        "dock_position": "SAME",
        "window_width": 1280,
        "window_height": 720,
        "camera_position": tuple(args_cli.camera_position),
        "camera_target": tuple(args_cli.camera_target),
        "enable_markers": True,
        "enable_live_plots": True,
    }
    for name, value in desired_attrs.items():
        if hasattr(cfg, name):
            setattr(cfg, name, value)
    return [cfg]


def _find_drive_joint_ids(joint_names: List[str]) -> tuple[List[int], List[int]]:
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


def _compute_drive_targets(
    robot,
    steering_ids: List[int],
    vx: float,
    vy: float,
    wz: float,
    *,
    num_envs: int,
    device: str,
) -> tuple[torch.Tensor, torch.Tensor]:
    steering_targets = torch.zeros((num_envs, len(DRIVE_MODULES)), device=device, dtype=torch.float32)
    drive_targets = torch.zeros((num_envs, len(DRIVE_MODULES)), device=device, dtype=torch.float32)

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

    joint_pos = robot.data.joint_pos.torch if hasattr(robot.data.joint_pos, "torch") else robot.data.joint_pos
    if joint_pos.ndim == 1:
        joint_pos = joint_pos.unsqueeze(0)

    for module_index, (wheel_vx, wheel_vy, speed_mps) in enumerate(wheel_vectors):
        wheel_vx *= speed_scale
        wheel_vy *= speed_scale
        speed_mps *= speed_scale
        current_angle = joint_pos[:, steering_ids[module_index]]

        if speed_mps < STOP_EPS:
            steering_targets[:, module_index] = current_angle
            continue

        raw_target = torch.full_like(current_angle, math.atan2(wheel_vy, wheel_vx))
        direct_delta = raw_target - current_angle
        direct_delta = torch.atan2(torch.sin(direct_delta), torch.cos(direct_delta))
        flipped_delta = raw_target + math.pi - current_angle
        flipped_delta = torch.atan2(torch.sin(flipped_delta), torch.cos(flipped_delta))
        use_flipped = torch.abs(flipped_delta) < torch.abs(direct_delta)
        steering_delta = torch.where(use_flipped, flipped_delta, direct_delta)

        steering_targets[:, module_index] = current_angle + steering_delta
        wheel_speed = torch.full_like(current_angle, speed_mps / WHEEL_RADIUS_M)
        steering_error = torch.abs(steering_delta)
        alignment_scale = (STEERING_ZERO_SPEED_ERROR_RAD - steering_error) / (
            STEERING_ZERO_SPEED_ERROR_RAD - STEERING_FULL_SPEED_ERROR_RAD
        )
        wheel_speed *= torch.clamp(alignment_scale, min=0.0, max=1.0)
        drive_targets[:, module_index] = torch.where(use_flipped, -wheel_speed, wheel_speed)

    return steering_targets, drive_targets


class SpineKeyboardController:
    def __init__(self, robot, joint_names: List[str], *, step_m: float, min_m: float, max_m: float):
        self.robot = robot
        self.joint_name = "franka_spine_vertical_joint"
        self.joint_index = joint_names.index(self.joint_name)
        self.step_m = float(step_m)
        self.min_m = float(min_m)
        self.max_m = float(max_m)
        if self.min_m > self.max_m:
            self.min_m, self.max_m = self.max_m, self.min_m

        joint_pos = robot.data.joint_pos.torch if hasattr(robot.data.joint_pos, "torch") else robot.data.joint_pos
        if joint_pos.ndim == 1:
            joint_pos = joint_pos.unsqueeze(0)
        self.target = joint_pos[:, self.joint_index : self.joint_index + 1].clone()
        initial = float(self.target[0, 0].item())
        self.target[:, 0] = max(self.min_m, min(self.max_m, initial))

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
            self._subscription = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
            print(
                "Spine keyboard control enabled: Up/Down arrows command "
                f"{self.joint_name}, step={self.step_m:.4f} m, range=[{self.min_m:.4f}, {self.max_m:.4f}] m",
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001 - keyboard is optional in headless/no-window sessions
            self._carb_input = None
            print(f"Warning: spine keyboard control unavailable: {exc}", file=sys.stderr)

    @property
    def available(self) -> bool:
        return self._subscription is not None

    def _set_target(self, value: float) -> None:
        value = max(self.min_m, min(self.max_m, float(value)))
        self.target[:, 0] = value
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
        current = float(self.target[0, 0].item())
        if key_name in {"UP", "KEY_UP", "ARROW_UP"} or key_name.endswith("_UP"):
            self._set_target(current + self.step_m)
            return True
        if key_name in {"DOWN", "KEY_DOWN", "ARROW_DOWN"} or key_name.endswith("_DOWN"):
            self._set_target(current - self.step_m)
            return True
        return True

    def apply(self) -> None:
        if hasattr(self.robot, "set_joint_position_target_index"):
            self.robot.set_joint_position_target_index(target=self.target, joint_ids=[self.joint_index])
        else:
            joint_pos = (
                self.robot.data.joint_pos.torch
                if hasattr(self.robot.data.joint_pos, "torch")
                else self.robot.data.joint_pos
            )
            if joint_pos.ndim == 1:
                joint_pos = joint_pos.unsqueeze(0)
            full_target = joint_pos.clone()
            full_target[:, self.joint_index] = self.target[:, 0]
            self.robot.set_joint_position_target(full_target)


class IsaacLabRosBridge(Node):
    def __init__(self, groups: List[JointGroup], *, enable_cable: bool = False):
        super().__init__("isaaclab_fr3duo_newton_bridge")
        self.groups = groups
        self.latest_commands: Dict[str, Dict[str, float]] = {group.label: {} for group in groups}
        self._latest_cable_points: Optional[List[tuple[float, float, float]]] = None
        self._latest_cable_gripper_boxes: Optional[List[dict]] = None
        self._latest_pedal_state = "NONE"
        self._latest_pedal_time_sec = None
        self._state_publishers = {
            group.label: self.create_publisher(JointState, group.state_topic, 10)
            for group in groups
        }
        self._command_subscriptions = []
        for group in groups:
            for topic in group.command_topics:
                sub = self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, label=group.label: self._on_joint_command(label, msg),
                    10,
                )
                self._command_subscriptions.append(sub)
        self._pedal_sub = self.create_subscription(
            String,
            PEDAL_STATE_TOPIC,
            self._on_pedal_state,
            10,
        )
        self._cable_robotiq_finger_pub = None
        self._cable_point_sub = None
        self._cable_gripper_box_sub = None
        if enable_cable:
            self._cable_robotiq_finger_pub = self.create_publisher(PointCloud, args_cli.cable_robotiq_finger_target_topic, 10)
            self._cable_point_sub = self.create_subscription(
                PointCloud,
                "/cable/body_centers",
                self._on_cable_points,
                10,
            )
            self._cable_gripper_box_sub = self.create_subscription(
                PointCloud,
                "/cable/gripper_collision_boxes",
                self._on_cable_gripper_boxes,
                10,
            )
        self.get_logger().info("IsaacLab ROS bridge listening on /isaac command topics")

    def _on_cable_points(self, msg: PointCloud):
        self._latest_cable_points = [
            (float(point.x), float(point.y), float(point.z))
            for point in msg.points
        ]

    def _on_cable_gripper_boxes(self, msg: PointCloud):
        channel_values = {channel.name: list(channel.values) for channel in msg.channels}

        def channel_value(name: str, index: int, default: float) -> float:
            values = channel_values.get(name)
            if values is None or index >= len(values):
                return float(default)
            return float(values[index])

        boxes = []
        for index, point in enumerate(msg.points):
            boxes.append(
                {
                    "position_m": (float(point.x), float(point.y), float(point.z)),
                    "quat_xyzw": (
                        channel_value("qx", index, 0.0),
                        channel_value("qy", index, 0.0),
                        channel_value("qz", index, 0.0),
                        channel_value("qw", index, 1.0),
                    ),
                    "size_m": (
                        channel_value("sx", index, 0.01),
                        channel_value("sy", index, 0.01),
                        channel_value("sz", index, 0.01),
                    ),
                    "finger_id": int(round(channel_value("finger", index, 0.0))),
                    "box_id": int(round(channel_value("box", index, float(index)))),
                }
            )
        self._latest_cable_gripper_boxes = boxes

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
        if timeout_sec >= 0.0 and now_sec - self._latest_pedal_time_sec > timeout_sec:
            self._latest_pedal_state = "NONE"
            return 0.0, 0.0, 0.0
        state = self._latest_pedal_state
        if state == "A":
            return 0.0, linear_speed_mps, 0.0
        if state == "B":
            return 0.0, -linear_speed_mps, 0.0
        if state in {"A+C", "C+A"}:
            return 0.0, 0.0, angular_speed_radps
        if state in {"B+C", "C+B"}:
            return 0.0, 0.0, -angular_speed_radps
        return 0.0, 0.0, 0.0

    def apply_commands(self, robot, group_indices: Dict[str, Dict[str, int]]):
        joint_pos = robot.data.joint_pos.torch if hasattr(robot.data.joint_pos, "torch") else robot.data.joint_pos
        if joint_pos.ndim == 1:
            joint_pos = joint_pos.unsqueeze(0)
        target = joint_pos.clone()

        any_command = False
        for group in self.groups:
            resolved = group_indices.get(group.label, {})
            for requested_name, position in self.latest_commands[group.label].items():
                joint_index = resolved.get(requested_name)
                if joint_index is None:
                    continue
                target[:, joint_index] = position
                any_command = True

        if not any_command:
            return

        if hasattr(robot, "set_joint_position_target_index"):
            robot.set_joint_position_target_index(target=target)
        else:
            robot.set_joint_position_target(target)

    def publish_states(self, robot, group_indices: Dict[str, Dict[str, int]]):
        joint_pos = robot.data.joint_pos.torch if hasattr(robot.data.joint_pos, "torch") else robot.data.joint_pos
        joint_vel = robot.data.joint_vel.torch if hasattr(robot.data.joint_vel, "torch") else robot.data.joint_vel
        if joint_pos.ndim == 2:
            joint_pos = joint_pos[0]
        if joint_vel.ndim == 2:
            joint_vel = joint_vel[0]

        stamp = self.get_clock().now().to_msg()
        for group in self.groups:
            msg = JointState()
            msg.header.stamp = stamp
            names = []
            positions = []
            velocities = []
            for requested_name in group.requested_names:
                joint_index = group_indices.get(group.label, {}).get(requested_name)
                if joint_index is None:
                    continue
                names.append(requested_name)
                positions.append(float(joint_pos[joint_index].item()))
                velocities.append(float(joint_vel[joint_index].item()))
            msg.name = names
            msg.position = positions
            msg.velocity = velocities
            msg.effort = [0.0] * len(names)
            self._state_publishers[group.label].publish(msg)

    def publish_cable_robotiq_finger_targets(self, targets: List[dict]):
        if self._cable_robotiq_finger_pub is None or not targets:
            return

        stamp = self.get_clock().now().to_msg()
        msg = PointCloud()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        channels = {
            name: ChannelFloat32(name=name)
            for name in ("qx", "qy", "qz", "qw", "sx", "sy", "sz", "finger", "box")
        }
        for index, target in enumerate(targets):
            position_m = target["position_m"]
            qx, qy, qz, qw = (float(v) for v in target["quat_xyzw"])
            q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if q_norm <= 0.0:
                quat_xyzw = (0.0, 0.0, 0.0, 1.0)
            else:
                inv_q_norm = 1.0 / q_norm
                quat_xyzw = (qx * inv_q_norm, qy * inv_q_norm, qz * inv_q_norm, qw * inv_q_norm)
            size_m = target["size_m"]
            finger_id = int(target.get("finger_id", index))
            msg.points.append(Point32(x=float(position_m[0]), y=float(position_m[1]), z=float(position_m[2])))
            for channel_name, value in (
                ("qx", quat_xyzw[0]),
                ("qy", quat_xyzw[1]),
                ("qz", quat_xyzw[2]),
                ("qw", quat_xyzw[3]),
                ("sx", size_m[0]),
                ("sy", size_m[1]),
                ("sz", size_m[2]),
                ("finger", finger_id),
                ("box", 0.0),
            ):
                channels[channel_name].values.append(float(value))
        msg.channels = [channels[name] for name in ("qx", "qy", "qz", "qw", "sx", "sy", "sz", "finger", "box")]
        self._cable_robotiq_finger_pub.publish(msg)

    def latest_cable_points(self) -> Optional[List[tuple[float, float, float]]]:
        return self._latest_cable_points

    def latest_cable_gripper_boxes(self) -> Optional[List[dict]]:
        return self._latest_cable_gripper_boxes


def _create_cable_stage_visuals(
    franka_root: Path,
    cable_config_path: Path | None = None,
    visual_offset_m=(0.0, 0.0, 0.0),
    visual_yaw_deg: float = 0.0,
    show_collision_visuals: bool = False,
):
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    visual_offset_m = tuple(float(v) for v in visual_offset_m)
    visual_yaw_rad = math.radians(float(visual_yaw_deg))
    visual_yaw_half = 0.5 * visual_yaw_rad
    visual_yaw_xyzw = (0.0, 0.0, math.sin(visual_yaw_half), math.cos(visual_yaw_half))

    def make_preview_material(name: str, color, roughness: float):
        material = UsdShade.Material.Define(stage, Sdf.Path(f"/World/Looks/{name}"))
        shader = UsdShade.Shader.Define(stage, Sdf.Path(f"/World/Looks/{name}/PreviewSurface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(tuple(float(v) for v in color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    board_material = make_preview_material("CableBoardDarkGray", (0.12, 0.12, 0.12), 0.60)
    c_clip_material = make_preview_material("CableCClipYellow", (1.0, 0.82, 0.0), 0.45)
    adapter_material = make_preview_material("CableWireAdapterDarkBlue", (0.0, 0.04, 0.20), 0.55)
    gray_material = make_preview_material("CableFixtureGray", (0.45, 0.45, 0.45), 0.65)
    collision_material = make_preview_material("CableFixtureCollisionOrange", (1.0, 0.18, 0.02), 0.35)

    board_usd = franka_root / "cable_world" / "assets" / "table_board_fixture" / "table_board_fixture.usd"
    if cable_config_path is not None and Path(cable_config_path).is_file():
        try:
            import yaml  # noqa: PLC0415

            with Path(cable_config_path).open("r", encoding="utf-8") as f:
                cable_config = yaml.safe_load(f) or {}
            raw_scene_usd = cable_config.get("scene_usd_path")
            if raw_scene_usd:
                raw_scene_path = Path(raw_scene_usd).expanduser()
                if raw_scene_path.is_absolute():
                    board_usd = raw_scene_path
                else:
                    board_usd = (franka_root / "cable_world" / raw_scene_path).resolve()
        except Exception as exc:  # pragma: no cover - visualization fallback only
            print(f"Warning: failed to read cable scene USD from {cable_config_path}: {exc}", file=sys.stderr)

    if board_usd.is_file():
        board_prim = stage.DefinePrim("/World/TableBoardFixtureVisual", "Xform")
        board_prim.GetReferences().AddReference(str(board_usd))
        board_xform = UsdGeom.Xformable(board_prim)
        translation_attr = board_xform.GetPrim().GetAttribute("xformOp:translate")
        rotate_attr = board_xform.GetPrim().GetAttribute("xformOp:rotateZYX")
        if translation_attr and translation_attr.IsValid():
            translation_attr.Set(Gf.Vec3d(*visual_offset_m))
        else:
            board_xform.AddTranslateOp().Set(Gf.Vec3d(*visual_offset_m))
        if rotate_attr and rotate_attr.IsValid():
            rotate_attr.Set(Gf.Vec3f(0.0, 0.0, float(visual_yaw_deg)))
        else:
            board_xform.AddRotateZYXOp().Set(Gf.Vec3f(0.0, 0.0, float(visual_yaw_deg)))
        board_prefix = str(board_prim.GetPath())
        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if not prim_path.startswith(board_prefix):
                continue
            relative_path = prim_path[len(board_prefix):].lower().replace("-", "_")
            is_collision_visual = "/collisions" in relative_path or "collider" in relative_path
            if is_collision_visual and prim.IsA(UsdGeom.Imageable):
                imageable = UsdGeom.Imageable(prim)
                if show_collision_visuals:
                    imageable.CreatePurposeAttr().Set(UsdGeom.Tokens.default_)
                    imageable.CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
                else:
                    imageable.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
                    imageable.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            if not prim.IsA(UsdGeom.Gprim):
                continue
            if is_collision_visual:
                if not show_collision_visuals:
                    continue
                material = collision_material
                color = (1.0, 0.18, 0.02)
            elif "c_clip" in relative_path:
                material = c_clip_material
                color = (1.0, 0.82, 0.0)
            elif "wire_to_base_adapter" in relative_path:
                material = adapter_material
                color = (0.0, 0.04, 0.20)
            elif "board_segment" in relative_path:
                material = board_material
                color = (0.12, 0.12, 0.12)
            else:
                material = gray_material
                color = (0.45, 0.45, 0.45)
            UsdShade.MaterialBindingAPI(prim).Bind(material)
            gprim = UsdGeom.Gprim(prim)
            display_color = gprim.GetDisplayColorAttr()
            if not display_color:
                display_color = gprim.CreateDisplayColorAttr()
            display_color.Set([color])
    else:
        print(f"Warning: cable board/fixture visual USD not found: {board_usd}", file=sys.stderr)

    curve_path = Sdf.Path("/World/CableVBDVisual")
    if stage.GetPrimAtPath(curve_path).IsValid():
        stage.RemovePrim(curve_path)
    curve = UsdGeom.BasisCurves.Define(stage, curve_path)
    curve_xform = UsdGeom.Xformable(curve.GetPrim())
    curve_xform.AddTranslateOp().Set(Gf.Vec3d(*visual_offset_m))
    curve_xform.AddOrientOp().Set(
        Gf.Quatf(visual_yaw_xyzw[3], visual_yaw_xyzw[0], visual_yaw_xyzw[1], visual_yaw_xyzw[2])
    )
    curve.CreateTypeAttr("linear")
    curve.CreateWrapAttr("nonperiodic")
    curve.CreateWidthsAttr([0.006, 0.006])
    curve.CreateCurveVertexCountsAttr([2])
    curve.CreatePointsAttr([(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)])
    curve.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])
    return curve


def _find_stage_prim_by_path_suffix(stage, selector: str):
    selector = str(selector).strip()
    if not selector:
        return None
    if selector.startswith("/"):
        prim = stage.GetPrimAtPath(selector)
        return prim if prim.IsValid() else None
    suffix = "/" + selector.strip("/")
    matches = [prim for prim in stage.Traverse() if str(prim.GetPath()).endswith(suffix)]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        print(
            f"Warning: Robotiq finger selector '{selector}' matched multiple prims: "
            + ", ".join(str(prim.GetPath()) for prim in matches),
            file=sys.stderr,
        )
    return None


def _fabric_prim_world_pose_by_selector(selector: str):
    try:
        import omni.usd  # noqa: PLC0415
        import usdrt  # noqa: PLC0415
        from pxr import UsdUtils  # noqa: PLC0415
        from usdrt import Rt  # noqa: PLC0415

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        usd_prim = _find_stage_prim_by_path_suffix(stage, selector)
        if usd_prim is None or not usd_prim.IsValid():
            return None

        stage_cache = UsdUtils.StageCache.Get()
        stage_id = stage_cache.GetId(stage).ToLongInt()
        if stage_id < 0:
            stage_id = stage_cache.Insert(stage).ToLongInt()
        rt_stage = usdrt.Usd.Stage.Attach(stage_id)
        if rt_stage is None:
            return None

        rt_prim = rt_stage.GetPrimAtPath(str(usd_prim.GetPath()))
        if rt_prim is None or not rt_prim.IsValid():
            return None
        rt_xformable = Rt.Xformable(rt_prim)
        if rt_xformable is None or not rt_xformable.GetPrim().IsValid():
            return None
        world_matrix_attr = rt_xformable.GetFabricHierarchyWorldMatrixAttr()
        if world_matrix_attr is None:
            return None
        world_matrix = world_matrix_attr.Get()
        if world_matrix is None:
            return None
        translation = world_matrix.ExtractTranslation()
        quat = world_matrix.ExtractRotationQuat()
        quat_imag = quat.GetImaginary()
        return (
            (float(translation[0]), float(translation[1]), float(translation[2])),
            (float(quat_imag[0]), float(quat_imag[1]), float(quat_imag[2]), float(quat.GetReal())),
            str(usd_prim.GetPath()),
        )
    except Exception as exc:  # noqa: BLE001 - optional Fabric path
        if not getattr(_fabric_prim_world_pose_by_selector, "_warned", False):
            print(f"Warning: failed to read Fabric live pose for Robotiq finger selector: {exc}", file=sys.stderr)
            _fabric_prim_world_pose_by_selector._warned = True
        return None


def _robot_world_pose_to_cable_world(position_m, quat_xyzw, world_offset_m, world_yaw_deg, gripper_offset_m):
    px, py, pz = (float(v) for v in position_m)
    ox, oy, oz = (float(v) for v in world_offset_m)
    gx, gy, gz = (float(v) for v in gripper_offset_m)
    yaw_half = 0.5 * math.radians(float(world_yaw_deg))
    yaw_s = math.sin(yaw_half)
    yaw_c = math.cos(yaw_half)

    dx = px - ox
    dy = py - oy
    dz = pz - oz
    local_position = (
        (1.0 - 2.0 * yaw_s * yaw_s) * dx + (2.0 * yaw_s * yaw_c) * dy + gx,
        -(2.0 * yaw_s * yaw_c) * dx + (1.0 - 2.0 * yaw_s * yaw_s) * dy + gy,
        dz + gz,
    )

    qx, qy, qz, qw = (float(v) for v in quat_xyzw)
    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if q_norm <= 0.0:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        inv_q_norm = 1.0 / q_norm
        qx, qy, qz, qw = qx * inv_q_norm, qy * inv_q_norm, qz * inv_q_norm, qw * inv_q_norm

    # Equivalent to inverse(cable_world_yaw) * robot_quat, with quaternions in xyzw order.
    local_quat = (
        yaw_c * qx + yaw_s * qy,
        yaw_c * qy - yaw_s * qx,
        yaw_c * qz - yaw_s * qw,
        yaw_c * qw + yaw_s * qz,
    )
    lqx, lqy, lqz, lqw = local_quat
    local_norm = math.sqrt(lqx * lqx + lqy * lqy + lqz * lqz + lqw * lqw)
    if local_norm <= 0.0:
        return local_position, (0.0, 0.0, 0.0, 1.0)
    inv_local_norm = 1.0 / local_norm
    return local_position, (
        lqx * inv_local_norm,
        lqy * inv_local_norm,
        lqz * inv_local_norm,
        lqw * inv_local_norm,
    )

def _stage_visual_bbox_center_world_by_selector(selector: str):
    try:
        import omni.usd  # noqa: PLC0415
        from pxr import Gf, Usd, UsdGeom  # noqa: PLC0415

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        visual_prim = _find_stage_prim_by_path_suffix(stage, f"{selector.rstrip('/')}/visuals")
        if visual_prim is None or not visual_prim.IsValid():
            return None

        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            useExtentsHint=False,
        )
        aligned_box = bbox_cache.ComputeWorldBound(visual_prim).ComputeAlignedBox()
        min_pt = aligned_box.GetMin()
        max_pt = aligned_box.GetMax()
        world_center = Gf.Vec3d(
            0.5 * (float(min_pt[0]) + float(max_pt[0])),
            0.5 * (float(min_pt[1]) + float(max_pt[1])),
            0.5 * (float(min_pt[2]) + float(max_pt[2])),
        )
        return (float(world_center[0]), float(world_center[1]), float(world_center[2])), str(visual_prim.GetPath())
    except Exception as exc:  # noqa: BLE001 - visualization helper must not stop teleop
        if not getattr(_stage_visual_bbox_center_world_by_selector, "_warned", False):
            print(f"Warning: failed to read Robotiq finger visual bbox world center: {exc}", file=sys.stderr)
            _stage_visual_bbox_center_world_by_selector._warned = True
        return None


def _fabric_pose_local_offset_to_world_center(position_m, quat_xyzw, world_center_m):
    qx, qy, qz, qw = (float(v) for v in quat_xyzw)
    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if q_norm <= 0.0:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        inv_q_norm = 1.0 / q_norm
        qx, qy, qz, qw = -qx * inv_q_norm, -qy * inv_q_norm, -qz * inv_q_norm, qw * inv_q_norm

    vx = float(world_center_m[0]) - float(position_m[0])
    vy = float(world_center_m[1]) - float(position_m[1])
    vz = float(world_center_m[2]) - float(position_m[2])
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _robotiq_inner_finger_contact_offset(
    selector: str,
    visual_center_offset_m,
    contact_x_offset_m,
    contact_y_offset_m,
    contact_z_offset_m,
    invert_opening: bool,
):
    # Keep the contact point attached to the live Robotiq inner_finger frame.
    # If the red boxes open opposite to the gray pads, use the opposite local-Y
    # side of the same moving finger instead of mirroring the world position.
    del selector
    visual_x, visual_y, visual_z = (float(v) for v in visual_center_offset_m)
    x = visual_x + float(contact_x_offset_m)
    z = visual_z + float(contact_z_offset_m)
    y = visual_y
    if abs(y) > 1e-6:
        side = -1.0 if bool(invert_opening) else 1.0
        y = side * math.copysign(abs(float(contact_y_offset_m)), y)
    return (x, y, z)


def _collect_robotiq_finger_targets(
    selectors,
    size_m,
    world_offset_m,
    world_yaw_deg,
    contact_x_offset_m,
    contact_y_offset_m,
    contact_z_offset_m,
    invert_opening,
):
    targets = []
    resolved_paths = []
    for finger_id, selector in enumerate(selectors):
        pose = _fabric_prim_world_pose_by_selector(selector)
        if pose is None:
            return None, resolved_paths
        position_m, quat_xyzw, prim_path = pose
        offset_cache = getattr(_collect_robotiq_finger_targets, "_offset_cache", {})
        cache_key = str(selector)
        cached = offset_cache.get(cache_key)
        if cached is None:
            visual_center = _stage_visual_bbox_center_world_by_selector(selector)
            if visual_center is not None:
                world_center_m, visual_path = visual_center
                visual_center_offset_m = _fabric_pose_local_offset_to_world_center(position_m, quat_xyzw, world_center_m)
                local_center_m = _robotiq_inner_finger_contact_offset(
                    selector,
                    visual_center_offset_m,
                    contact_x_offset_m,
                    contact_y_offset_m,
                    contact_z_offset_m,
                    invert_opening,
                )
                cached = (local_center_m, visual_path)
                offset_cache[cache_key] = cached
                _collect_robotiq_finger_targets._offset_cache = offset_cache
        if cached is not None:
            local_center_m, visual_path = cached
            qx, qy, qz, qw = (float(v) for v in quat_xyzw)
            q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if q_norm <= 0.0:
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
            else:
                inv_q_norm = 1.0 / q_norm
                qx, qy, qz, qw = qx * inv_q_norm, qy * inv_q_norm, qz * inv_q_norm, qw * inv_q_norm
            vx, vy, vz = (float(v) for v in local_center_m)
            tx = 2.0 * (qy * vz - qz * vy)
            ty = 2.0 * (qz * vx - qx * vz)
            tz = 2.0 * (qx * vy - qy * vx)
            rotated_center = (
                vx + qw * tx + qy * tz - qz * ty,
                vy + qw * ty + qz * tx - qx * tz,
                vz + qw * tz + qx * ty - qy * tx,
            )
            centered_position_m = (
                float(position_m[0]) + rotated_center[0],
                float(position_m[1]) + rotated_center[1],
                float(position_m[2]) + rotated_center[2],
            )
            resolved_paths.append(f"{prim_path} visual={visual_path}")
        else:
            centered_position_m = position_m
            resolved_paths.append(prim_path)


        position_m, quat_xyzw = _robot_world_pose_to_cable_world(
            centered_position_m,
            quat_xyzw,
            world_offset_m,
            world_yaw_deg,
            (0.0, 0.0, 0.0),
        )
        targets.append(
            {
                "position_m": position_m,
                "quat_xyzw": quat_xyzw,
                "size_m": tuple(float(v) for v in size_m),
                "finger_id": finger_id,
            }
        )
    return targets, resolved_paths


def _update_cable_stage_visual(curve, points):
    if curve is None or points is None:
        return
    from pxr import Vt  # noqa: PLC0415

    if len(points) == 0:
        return
    point_list = [tuple(float(v) for v in point) for point in points]
    curve.GetPointsAttr().Set(Vt.Vec3fArray(point_list))
    curve.GetCurveVertexCountsAttr().Set([len(point_list)])
    curve.GetWidthsAttr().Set([0.006] * len(point_list))
    curve.GetDisplayColorAttr().Set([(1.0, 0.0, 0.0)])


def _create_cable_gripper_collision_box_visual(visual_offset_m, visual_yaw_deg: float = 0.0):
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    root_path = "/World/CableGripperCollisionVisual"
    root_prim = stage.GetPrimAtPath(root_path)
    if root_prim.IsValid():
        stage.RemovePrim(root_path)

    material = UsdShade.Material.Define(stage, Sdf.Path("/World/Looks/CableGripperCollisionRed"))
    shader = UsdShade.Shader.Define(stage, Sdf.Path("/World/Looks/CableGripperCollisionRed/PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((1.0, 0.0, 0.0))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.45)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    root = UsdGeom.Xform.Define(stage, Sdf.Path(root_path))
    root_xform = UsdGeom.Xformable(root.GetPrim())
    root_xform.AddTranslateOp().Set(Gf.Vec3d(*tuple(float(v) for v in visual_offset_m)))
    visual_yaw_half = 0.5 * math.radians(float(visual_yaw_deg))
    root_xform.AddOrientOp().Set(Gf.Quatf(math.cos(visual_yaw_half), 0.0, 0.0, math.sin(visual_yaw_half)))

    return {
        "root_path": root_path,
        "material": material,
        "boxes": {},
    }


def _update_cable_gripper_collision_box_visual(visual, boxes):
    if visual is None or boxes is None:
        return
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return

    active_keys = set()
    for index, box in enumerate(boxes):
        finger_id = int(box.get("finger_id", 0))
        box_id = int(box.get("box_id", index))
        key = (finger_id, box_id)
        active_keys.add(key)
        box_visual = visual["boxes"].get(key)
        if box_visual is None:
            finger_name = f"finger_{finger_id}"
            box_path = f"{visual['root_path']}/{finger_name}_collision_box_{box_id}"
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(box_path))
            cube.CreateSizeAttr(1.0)
            UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(visual["material"])
            xform = UsdGeom.Xformable(cube.GetPrim())
            box_visual = {
                "prim_path": box_path,
                "translate_op": xform.AddTranslateOp(),
                "orient_op": xform.AddOrientOp(),
                "scale_op": xform.AddScaleOp(),
            }
            visual["boxes"][key] = box_visual

        position_m = tuple(float(v) for v in box.get("position_m", (0.0, 0.0, 0.0)))
        qx, qy, qz, qw = (float(v) for v in box.get("quat_xyzw", (0.0, 0.0, 0.0, 1.0)))
        q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if q_norm <= 0.0:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            inv_q_norm = 1.0 / q_norm
            qx, qy, qz, qw = qx * inv_q_norm, qy * inv_q_norm, qz * inv_q_norm, qw * inv_q_norm
        size_m = tuple(float(v) for v in box.get("size_m", (0.01, 0.01, 0.01)))
        box_visual["translate_op"].Set(Gf.Vec3d(*position_m))
        box_visual["orient_op"].Set(Gf.Quatf(qw, qx, qy, qz))
        box_visual["scale_op"].Set(Gf.Vec3f(*size_m))

    for key, box_visual in list(visual["boxes"].items()):
        if key in active_keys:
            continue
        prim = stage.GetPrimAtPath(box_visual["prim_path"])
        if prim.IsValid():
            stage.RemovePrim(box_visual["prim_path"])
        del visual["boxes"][key]


def main():
    usd_path = Path(args_cli.usd_path).expanduser()
    franka_root = Path(args_cli.franka_root).expanduser()
    if not usd_path.exists():
        raise FileNotFoundError(f"USD path does not exist: {usd_path}")
    room_usd_path = None
    if not args_cli.no_room:
        room_path = _path_relative_to_franka_root(args_cli.room_usd_path, franka_root)
        if not room_path.exists():
            raise FileNotFoundError(f"Room USD path does not exist: {room_path}")
        _prepare_robot_room_texture_links(room_path)
        room_usd_path = str(room_path)

    groups = _load_joint_groups(include_browser_commands=not args_cli.no_browser)
    visualizer_cfgs = _make_visualizer_cfgs()
    solver_cfg = MJWarpSolverCfg(
        njmax=args_cli.mj_njmax,
        nconmax=args_cli.mj_nconmax,
        cone=args_cli.mj_cone,
        integrator=args_cli.mj_integrator,
        impratio=args_cli.mj_impratio,
    )
    render_interval = max(1, int(round(args_cli.physics_hz / max(args_cli.render_hz, 1.0))))
    sim_cfg = sim_utils.SimulationCfg(
        device=args_cli.device,
        dt=1.0 / args_cli.physics_hz,
        render_interval=render_interval,
        physics=NewtonCfg(
            solver_cfg=solver_cfg,
            num_substeps=args_cli.physics_substeps,
            debug_mode=False,
        ),
    )
    if visualizer_cfgs and hasattr(sim_cfg, "visualizer_cfgs"):
        sim_cfg.visualizer_cfgs = visualizer_cfgs

    sim = sim_utils.SimulationContext(sim_cfg)

    SceneCfg = _make_scene_cfg(str(usd_path), args_cli.robot_prim_path, room_usd_path)
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=0.0, replicate_physics=False))
    robot_prim_path = _env_robot_prim_path(args_cli.robot_prim_path)
    _fix_single_articulation_root(robot_prim_path)
    _fix_newton_reversed_fixed_joints(robot_prim_path)
    _set_angular_drive_type_for_joints(robot_prim_path, GRIPPER_DRIVER_JOINTS, "acceleration")
    sim.reset()
    scene.reset()

    robot = scene["robot"]
    if hasattr(robot.data, "joint_names"):
        actual_joint_names = list(robot.data.joint_names)
    elif hasattr(robot, "joint_names"):
        actual_joint_names = list(robot.joint_names)
    else:
        raise RuntimeError("Could not read joint names from IsaacLab articulation")
    group_indices = _resolve_group_indices(groups, actual_joint_names)
    missing = [
        f"{group.label}:{name}"
        for group in groups
        for name in group.requested_names
        if name not in group_indices.get(group.label, {})
    ]
    if missing:
        print("Warning: unresolved joints:", ", ".join(missing), file=sys.stderr)
    print("IsaacLab fr3duo Newton bridge started")
    print("Actual joint names:", ", ".join(actual_joint_names))
    try:
        steering_ids, drive_ids = _find_drive_joint_ids(actual_joint_names)
        print(
            "Pedal base control enabled: "
            f"topic={PEDAL_STATE_TOPIC} steering_ids={steering_ids} drive_ids={drive_ids} "
            f"linear_speed={args_cli.pedal_linear_speed:.3f} m/s "
            f"angular_speed={args_cli.pedal_angular_speed:.3f} rad/s "
            f"timeout={args_cli.pedal_timeout:.3f} s"
        )
    except RuntimeError as exc:
        steering_ids, drive_ids = [], []
        print(f"Warning: pedal base control disabled: {exc}", file=sys.stderr)

    spine_keyboard_controller = None
    if "franka_spine_vertical_joint" in actual_joint_names:
        spine_keyboard_controller = SpineKeyboardController(
            robot,
            actual_joint_names,
            step_m=args_cli.spine_keyboard_step,
            min_m=args_cli.spine_keyboard_min,
            max_m=args_cli.spine_keyboard_max,
        )
    else:
        print("Warning: spine keyboard control disabled: franka_spine_vertical_joint not found", file=sys.stderr)

    cable_curve_visual = None
    cable_gripper_collision_visual = None
    if args_cli.with_cable:
        cable_config_path = Path(args_cli.cable_config_path).expanduser()
        if not cable_config_path.is_absolute():
            cable_config_path = franka_root / cable_config_path
        cable_world_offset = tuple(float(v) for v in args_cli.cable_world_position_offset)
        cable_world_yaw_deg = float(args_cli.cable_world_yaw_deg)
        cable_curve_visual = _create_cable_stage_visuals(
            franka_root,
            cable_config_path,
            cable_world_offset,
            cable_world_yaw_deg,
            args_cli.show_table_board_fixture_collisions,
        )
        cable_gripper_collision_visual = _create_cable_gripper_collision_box_visual(
            cable_world_offset, cable_world_yaw_deg
        )
        print(
            "Cable VBD ROS coupling enabled: "
            f"config={cable_config_path} "
            f"visual_offset={cable_world_offset} visual_yaw_deg={cable_world_yaw_deg:.3f}"
        )

    rclpy.init()
    node = IsaacLabRosBridge(groups, enable_cable=args_cli.with_cable)
    publish_period = 1.0 / max(args_cli.ros_publish_rate, 1.0)
    next_publish_time = 0.0
    logged_robotiq_finger_targets = False
    logged_missing_robotiq_finger_targets = False
    # Cable-world finger ids are ordered opposite to the Robotiq inner_finger link
    # names, so swap each gripper pair to make the red collision boxes open/close
    # in the same visual direction as the gray Robotiq fingers.
    robotiq_finger_selectors = (
        "left_Robotiq_2F_85/left__Robotiq_2F_85/right_inner_finger",
        "left_Robotiq_2F_85/left__Robotiq_2F_85/left_inner_finger",
        "right_Robotiq_2F_85/right_Robotiq_2F_85/right_inner_finger",
        "right_Robotiq_2F_85/right_Robotiq_2F_85/left_inner_finger",
    )

    try:
        while simulation_app.is_running() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.apply_commands(robot, group_indices)
            if steering_ids and drive_ids:
                vx, vy, wz = node.pedal_base_twist(
                    args_cli.pedal_linear_speed,
                    args_cli.pedal_angular_speed,
                    args_cli.pedal_timeout,
                )
                num_envs = int(getattr(robot, "num_instances", 1))
                steering_targets, drive_targets = _compute_drive_targets(
                    robot,
                    steering_ids,
                    vx,
                    vy,
                    wz,
                    num_envs=num_envs,
                    device=sim.device,
                )
                robot.set_joint_position_target_index(target=steering_targets, joint_ids=steering_ids)
                robot.set_joint_velocity_target_index(target=drive_targets, joint_ids=drive_ids)
            if spine_keyboard_controller is not None:
                spine_keyboard_controller.apply()
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.get_physics_dt())
            if args_cli.with_cable:
                live_gripper_boxes = None
                if args_cli.cable_robotiq_finger_targets:
                    world_offset = tuple(float(v) for v in args_cli.cable_world_position_offset)
                    finger_targets, resolved_paths = _collect_robotiq_finger_targets(
                        robotiq_finger_selectors,
                        args_cli.cable_robotiq_finger_size,
                        world_offset,
                        args_cli.cable_world_yaw_deg,
                        args_cli.cable_robotiq_contact_x_offset,
                        args_cli.cable_robotiq_contact_y_offset,
                        args_cli.cable_robotiq_contact_z_offset,
                        args_cli.cable_robotiq_invert_opening,
                    )
                    if finger_targets is None:
                        if not logged_missing_robotiq_finger_targets:
                            logged_missing_robotiq_finger_targets = True
                            print(
                                "Warning: could not resolve all Robotiq finger selectors for cable targets: "
                                + ", ".join(robotiq_finger_selectors),
                                file=sys.stderr,
                            )
                    else:
                        if not logged_robotiq_finger_targets:
                            logged_robotiq_finger_targets = True
                            print(
                                "Cable gripper collision uses live Robotiq finger poses: "
                                + ", ".join(resolved_paths),
                                flush=True,
                            )
                        node.publish_cable_robotiq_finger_targets(finger_targets)
                        live_gripper_boxes = finger_targets
                _update_cable_stage_visual(cable_curve_visual, node.latest_cable_points())
                _update_cable_gripper_collision_box_visual(
                    cable_gripper_collision_visual,
                    live_gripper_boxes if live_gripper_boxes is not None else node.latest_cable_gripper_boxes(),
                )
            now = node.get_clock().now().nanoseconds * 1e-9
            if now >= next_publish_time:
                node.publish_states(robot, group_indices)
                next_publish_time = now + publish_period

    except BaseException as e:
        print("MAIN LOOP EXCEPTION:", repr(e), flush=True)
        traceback.print_exc()
        raise

    finally:
        node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
