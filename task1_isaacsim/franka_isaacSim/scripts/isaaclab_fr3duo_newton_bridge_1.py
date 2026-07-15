#!/usr/bin/env python3
"""IsaacLab Newton/MJWarp ROS bridge for the fr3duo mobile USD.

This script is meant to run inside the Isaac Lab ROS2 container, while the
Task 1 ROS helper containers keep running on the host network.  It
publishes/subscribes the `/isaac/*` topics so `ros_republisher`,
`position_controller`, `browser_controller`, and the `teleop_adapters`
(keyboard/GELLO) can be reused unchanged.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from isaaclab.app import AppLauncher
import traceback

def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd-path",
        default="/workspace/EBiM_Challenge/task1_isaacsim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd",
        help="USD file to load into the IsaacLab scene.",
    )
    parser.add_argument(
        "--embodiment",
        default="fr3duo_mobile",
        help="Embodiment key under assets/embodiments.",
    )
    parser.add_argument(
        "--franka-root",
        default="/workspace/EBiM_Challenge/task1_isaacsim",
        help="Task 1 root (containing assets/embodiments and cable_world) inside the Isaac Lab container.",
    )
    parser.add_argument("--robot-prim-path", default="{ENV_REGEX_NS}/Robot")
    parser.add_argument(
        "--disable-browser-command-topics",
        action="store_true",
        help="Do not subscribe to /isaac/browser/* command topics.",
    )
    parser.add_argument("--ros-publish-rate", type=float, default=60.0)
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
        "--spine-keyboard-control",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use keyboard Up/Down arrows to command franka_spine_vertical_joint height.",
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
        default=-0.05,
        help="Minimum franka_spine_vertical_joint target in meters for keyboard control.",
    )
    parser.add_argument(
        "--spine-keyboard-max",
        type=float,
        default=0.50,
        help="Maximum franka_spine_vertical_joint target in meters for keyboard control.",
    )
    parser.add_argument("--physics-hz", type=float, default=240.0)
    parser.add_argument("--render-hz", type=float, default=60.0)
    parser.add_argument("--physics-substeps", type=int, default=2)
    parser.add_argument("--effort-limit", type=float, default=400.0)
    parser.add_argument("--velocity-limit", type=float, default=8.0)
    parser.add_argument("--stiffness", type=float, default=800.0)
    parser.add_argument("--damping", type=float, default=80.0)
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
        "--cable-gripper-body-name",
        default="left_fr3v2_hand",
        help="IsaacLab robot body whose world pose drives the VBD cable gripper.",
    )
    parser.add_argument(
        "--cable-gripper-side",
        choices=["left", "right"],
        default="left",
        help="Robot gripper command side used to derive cable gripper gap.",
    )
    parser.add_argument(
        "--cable-gripper-gap-m",
        type=float,
        default=None,
        help="Fixed cable gripper gap in meters. If omitted, maps robot gripper joint state to the cable gap.",
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
        help="Additional local X offset in meters from each Robotiq inner_finger frame to the red contact box center.",
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
        "--cable-robotiq-finger-prim",
        action="append",
        default=[],
        help=(
            "Robotiq finger prim selector. Repeat four times if overriding defaults. "
            "A selector may be an absolute prim path or a path suffix such as "
            "left_Robotiq_2F_85/left_inner_finger."
        ),
    )
    parser.add_argument(
        "--cable-gripper-position-offset",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="Cable-world offset added after mapping the robot body pose into the VBD gripper frame.",
    )
    parser.add_argument(
        "--cable-proxy-finger-z-offset",
        type=float,
        default=0.0,
        help="Visual-only offset in meters applied to proxy fingers along CableGripperProxyVisual local +Z.",
    )
    parser.add_argument(
        "--cable-proxy-finger-cube-z-offset",
        type=float,
        default=0.0,
        help="Visual-only offset in meters from each finger axes origin to the red cube center along local +Z.",
    )
    parser.add_argument(
        "--cable-proxy-root-rpy-deg",
        type=float,
        nargs=3,
        default=(0.0, 0.0, 0.0),
        metavar=("ROLL", "PITCH", "YAW"),
        help="Visual-only fixed local XYZ Euler offset in degrees applied to CableGripperProxyVisual root orientation.",
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
        "--cable-robot-quat-order",
        choices=["wxyz", "xyzw"],
        default="wxyz",
        help="Quaternion order reported by the IsaacLab robot body pose.",
    )
    parser.add_argument(
        "--cable-extra-arg",
        action="append",
        default=[],
        help="Additional single argument forwarded to cable_world/run_board_cable.py. Repeat for multiple args.",
    )
    AppLauncher.add_app_launcher_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import omni.usd
from pxr import UsdPhysics

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

try:
    from isaaclab_visualizers.kit import KitVisualizerCfg
except Exception:  # pragma: no cover - optional extension/package
    KitVisualizerCfg = None

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import ChannelFloat32, JointState, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32, String

try:
    import yaml
except Exception:  # pragma: no cover - PyYAML should exist in the ROS image
    yaml = None


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


def _command_topics(primary_topic: str, browser_topic: str, include_browser: bool) -> List[str]:
    topics = [primary_topic]
    if include_browser:
        topics.append(browser_topic)
    return topics


def _load_joint_groups(franka_root: Path, embodiment: str, *, include_browser_commands: bool = True) -> List[JointGroup]:
    contract_path = franka_root / "assets" / "embodiments" / embodiment / "data_contract.yaml"
    left_names = list(LEFT_FALLBACK_JOINTS)
    right_names = list(RIGHT_FALLBACK_JOINTS)
    left_gripper = LEFT_GRIPPER_DRIVER
    right_gripper = RIGHT_GRIPPER_DRIVER

    if contract_path.exists() and yaml is not None:
        with contract_path.open("r", encoding="utf-8") as f:
            contract = yaml.safe_load(f) or {}
        state = contract.get("state_structure", {})
        arms = state.get("arms", {})
        left_names = list(arms.get("left", {}).get("joint_names") or left_names)
        right_names = list(arms.get("right", {}).get("joint_names") or right_names)

    return [
        JointGroup(
            label="left_arm",
            state_topic="/isaac/left_joint_states",
            command_topics=_command_topics("/isaac/left_joint_commands", "/isaac/browser/left_joint_commands", include_browser_commands),
            requested_names=left_names,
        ),
        JointGroup(
            label="right_arm",
            state_topic="/isaac/right_joint_states",
            command_topics=_command_topics("/isaac/right_joint_commands", "/isaac/browser/right_joint_commands", include_browser_commands),
            requested_names=right_names,
        ),
        JointGroup(
            label="left_gripper",
            state_topic="/isaac/left_robotiq_joint_states",
            command_topics=_command_topics("/isaac/left_robotiq_joint_commands", "/isaac/browser/left_robotiq_joint_commands", include_browser_commands),
            requested_names=[left_gripper],
        ),
        JointGroup(
            label="right_gripper",
            state_topic="/isaac/right_robotiq_joint_states",
            command_topics=_command_topics("/isaac/right_robotiq_joint_commands", "/isaac/browser/right_robotiq_joint_commands", include_browser_commands),
            requested_names=[right_gripper],
        ),
    ]


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
    if name.endswith("_joint"):
        yield name[:-6]


def _resolve_group_indices(groups: List[JointGroup], actual_names: List[str]) -> Dict[str, Dict[str, int]]:
    actual_by_name = {name: idx for idx, name in enumerate(actual_names)}
    resolved: Dict[str, Dict[str, int]] = {}
    for group in groups:
        group_map = {}
        for requested_name in group.requested_names:
            for candidate in _candidate_joint_names(requested_name):
                if candidate in actual_by_name:
                    group_map[requested_name] = actual_by_name[candidate]
                    break
        resolved[group.label] = group_map
    return resolved


def _as_tensor(value):
    if hasattr(value, "torch"):
        return value.torch
    return value


NEWTON_REVERSED_FIXED_JOINTS = (
    "argo_drive_front_fixed_joint",
    "base_joint",
    "zed_mini_camera_joint",
)


def _iter_prims_under(root_prim):
    yield root_prim
    for child in root_prim.GetChildren():
        yield from _iter_prims_under(child)


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

    root_prims = [
        prim
        for prim in _iter_prims_under(robot_prim)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
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
    for prim in _iter_prims_under(robot_prim):
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


def _make_scene_cfg(usd_path: str, prim_path: str):
    robot_cfg = ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
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
        ),
        actuators={
            "base_steering": ImplicitActuatorCfg(
                joint_names_expr=["tmrv0_2_joint_0", "tmrv0_2_joint_2"],
                effort_limit_sim=200.0,
                velocity_limit_sim=4.0,
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
                joint_names_expr=[".*caster.*", "rocker_arm_joint"],
                effort_limit_sim=50.0,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=0.0,
                damping=0.0,
            ),
            "spine": ImplicitActuatorCfg(
                joint_names_expr=["franka_spine_vertical_joint"],
                effort_limit_sim=args_cli.effort_limit,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=args_cli.stiffness,
                damping=args_cli.damping,
            ),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=[".*fr3v2_joint[1-7]"],
                effort_limit_sim=args_cli.effort_limit,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=args_cli.stiffness,
                damping=args_cli.damping,
            ),
            "grippers": ImplicitActuatorCfg(
                joint_names_expr=[".*(finger|knuckle).*"],
                effort_limit_sim=200.0,
                velocity_limit_sim=1000000000 * args_cli.velocity_limit,
                stiffness=5.0,
                damping=0.5,
            ),
        },
    )

    class TeleopSceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
        dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=500.0, color=(0.85, 0.9, 1.0)),
        )
        robot = robot_cfg

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


def _wrap_to_pi(angle: torch.Tensor) -> torch.Tensor:
    return torch.atan2(torch.sin(angle), torch.cos(angle))


def _steering_alignment_scale(error: torch.Tensor) -> torch.Tensor:
    scale = (STEERING_ZERO_SPEED_ERROR_RAD - error) / (
        STEERING_ZERO_SPEED_ERROR_RAD - STEERING_FULL_SPEED_ERROR_RAD
    )
    return torch.clamp(scale, min=0.0, max=1.0)


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

    joint_pos = _as_tensor(robot.data.joint_pos)
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
        direct_delta = _wrap_to_pi(raw_target - current_angle)
        flipped_delta = _wrap_to_pi(raw_target + math.pi - current_angle)
        use_flipped = torch.abs(flipped_delta) < torch.abs(direct_delta)
        steering_delta = torch.where(use_flipped, flipped_delta, direct_delta)

        steering_targets[:, module_index] = current_angle + steering_delta
        wheel_speed = torch.full_like(current_angle, speed_mps / WHEEL_RADIUS_M)
        wheel_speed *= _steering_alignment_scale(torch.abs(steering_delta))
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

        joint_pos = _as_tensor(robot.data.joint_pos)
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
            joint_pos = _as_tensor(self.robot.data.joint_pos)
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
        self._latest_cable_gripper_root_pose: Optional[dict] = None
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
        self._cable_pose_pub = None
        self._cable_gap_pub = None
        self._cable_robotiq_finger_pub = None
        self._cable_point_sub = None
        self._cable_gripper_box_sub = None
        self._cable_gripper_root_sub = None
        if enable_cable:
            self._cable_pose_pub = self.create_publisher(PoseStamped, "/isaac/left_gripper_pose", 10)
            self._cable_gap_pub = self.create_publisher(Float32, "/isaac/left_gripper_gap", 10)
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
            self._cable_gripper_root_sub = self.create_subscription(
                PoseStamped,
                "/cable/gripper_root_pose",
                self._on_cable_gripper_root_pose,
                10,
            )
        self.get_logger().info("IsaacLab ROS bridge listening on /isaac command topics")

    def _on_cable_points(self, msg: PointCloud):
        self._latest_cable_points = [
            (float(point.x), float(point.y), float(point.z))
            for point in msg.points
        ]

    def _on_cable_gripper_root_pose(self, msg: PoseStamped):
        self._latest_cable_gripper_root_pose = {
            "position_m": (
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ),
            "quat_xyzw": (
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ),
        }

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
        # Forward/back tokens are emitted by keyboard_to_base.py (w/s keys); the
        # foot pedal only produces the strafe/yaw tokens below.
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

    def apply_commands(self, robot, group_indices: Dict[str, Dict[str, int]]):
        joint_pos = _as_tensor(robot.data.joint_pos)
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
        joint_pos = _as_tensor(robot.data.joint_pos)
        joint_vel = _as_tensor(robot.data.joint_vel)
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

    def publish_cable_gripper(
        self,
        *,
        position_m: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
        gap_m: float,
    ):
        if self._cable_pose_pub is None or self._cable_gap_pub is None:
            return

        stamp = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(position_m[0])
        pose_msg.pose.position.y = float(position_m[1])
        pose_msg.pose.position.z = float(position_m[2])
        pose_msg.pose.orientation.x = float(quat_xyzw[0])
        pose_msg.pose.orientation.y = float(quat_xyzw[1])
        pose_msg.pose.orientation.z = float(quat_xyzw[2])
        pose_msg.pose.orientation.w = float(quat_xyzw[3])
        self._cable_pose_pub.publish(pose_msg)

        gap_msg = Float32()
        gap_msg.data = float(gap_m)
        self._cable_gap_pub.publish(gap_msg)

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
            quat_xyzw = _quat_xyzw_normalize(target["quat_xyzw"])
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

    def latest_cable_gripper_root_pose(self) -> Optional[dict]:
        return self._latest_cable_gripper_root_pose


def _joint_names(robot) -> List[str]:
    if hasattr(robot.data, "joint_names"):
        return list(robot.data.joint_names)
    if hasattr(robot, "joint_names"):
        return list(robot.joint_names)
    raise RuntimeError("Could not read joint names from IsaacLab articulation")


def _resolve_path(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = root / path
    return path


def _robot_body_pose(robot, body_name: str) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    body_ids, body_names = robot.find_bodies(body_name, preserve_order=True)
    if len(body_ids) != 1:
        raise RuntimeError(f"Expected one body matching '{body_name}', found {len(body_ids)}: {body_names}")
    body_idx = int(body_ids[0])
    body_pos_w = _as_tensor(robot.data.body_pos_w)
    body_quat_w = _as_tensor(robot.data.body_quat_w)
    if body_pos_w.ndim == 3:
        pos = body_pos_w[0, body_idx]
        quat = body_quat_w[0, body_idx]
    else:
        pos = body_pos_w[body_idx]
        quat = body_quat_w[body_idx]
    return (
        tuple(float(v) for v in pos.detach().cpu().tolist()),
        tuple(float(v) for v in quat.detach().cpu().tolist()),
    )


def _to_xyzw(quat: tuple[float, float, float, float], order: str) -> tuple[float, float, float, float]:
    if order == "xyzw":
        return quat
    w, x, y, z = quat
    return x, y, z, w


def _robot_gripper_gap_m(
    robot,
    *,
    side: str,
    actual_joint_names: List[str],
    group_indices: Dict[str, Dict[str, int]],
    cable_gap_range: tuple[float, float],
    fixed_gap_m: Optional[float],
) -> float:
    if fixed_gap_m is not None:
        return float(fixed_gap_m)

    joint_pos = _as_tensor(robot.data.joint_pos)
    if joint_pos.ndim == 2:
        joint_pos = joint_pos[0]

    min_gap, max_gap = cable_gap_range
    actual_by_name = {name: idx for idx, name in enumerate(actual_joint_names)}
    finger_joint_names = (
        f"{side}_fr3v2_finger_joint1",
        f"{side}_fr3v2_finger_joint2",
    )
    if all(name in actual_by_name for name in finger_joint_names):
        gap = sum(float(joint_pos[actual_by_name[name]].item()) for name in finger_joint_names)
        return min(max(gap, min_gap), max_gap)

    label = f"{side}_gripper"
    group_map = group_indices.get(label, {})
    if not group_map:
        return cable_gap_range[0]
    joint_index = next(iter(group_map.values()))
    joint_value = float(joint_pos[joint_index].item())

    # Fallback for normalized gripper commands: data_contract says 1=open, 0=closed.
    open_fraction = min(max(joint_value, 0.0), 1.0)
    return min_gap + open_fraction * (max_gap - min_gap)


def _create_cable_stage_visuals(
    franka_root: Path,
    cable_config_path: Path | None = None,
    visual_offset_m=(0.0, 0.0, 0.0),
    visual_yaw_deg: float = 0.0,
):
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    visual_offset_m = tuple(float(v) for v in visual_offset_m)
    visual_yaw_xyzw = _local_z_rotation_xyzw(math.radians(float(visual_yaw_deg)))

    board_material = UsdShade.Material.Define(stage, Sdf.Path("/World/Looks/CableBoardBlue"))
    board_shader = UsdShade.Shader.Define(stage, Sdf.Path("/World/Looks/CableBoardBlue/PreviewSurface"))
    board_shader.CreateIdAttr("UsdPreviewSurface")
    board_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.0, 0.18, 1.0))
    board_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.55)
    board_material.CreateSurfaceOutput().ConnectToSource(board_shader.ConnectableAPI(), "surface")

    gray_material = UsdShade.Material.Define(stage, Sdf.Path("/World/Looks/CableFixtureGray"))
    gray_shader = UsdShade.Shader.Define(stage, Sdf.Path("/World/Looks/CableFixtureGray/PreviewSurface"))
    gray_shader.CreateIdAttr("UsdPreviewSurface")
    gray_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.45, 0.45, 0.45))
    gray_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
    gray_material.CreateSurfaceOutput().ConnectToSource(gray_shader.ConnectableAPI(), "surface")

    board_usd = franka_root / "cable_world" / "assets" / "table_board_fixture" / "table_board_fixture.usd"
    if cable_config_path is not None and Path(cable_config_path).is_file():
        try:
            import yaml  # noqa: PLC0415

            with Path(cable_config_path).open("r", encoding="utf-8") as f:
                cable_config = yaml.safe_load(f) or {}
            raw_scene_usd = cable_config.get("scene_usd_path") or cable_config.get("board_usd_path")
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
        _set_existing_translate_rotate_zyx(board_xform, visual_offset_m, visual_yaw_deg)
        board_prefix = str(board_prim.GetPath())
        for prim in stage.Traverse():
            prim_path = str(prim.GetPath())
            if not prim_path.startswith(board_prefix) or not prim.IsA(UsdGeom.Gprim):
                continue
            relative_path = prim_path[len(board_prefix):].lower()
            # In board.usd, fixtures are named round_peg, wire_to_base_adapter,
            # Plug, etc. The actual board surface is the board_segment prim.
            is_board_part = "board_segment" in relative_path
            material = board_material if is_board_part else gray_material
            color = (0.0, 0.18, 1.0) if is_board_part else (0.45, 0.45, 0.45)
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
    curve_xform.AddOrientOp().Set(_gf_quatf_from_xyzw(visual_yaw_xyzw))
    curve.CreateTypeAttr("linear")
    curve.CreateWrapAttr("nonperiodic")
    curve.CreateWidthsAttr([0.006, 0.006])
    curve.CreateCurveVertexCountsAttr([2])
    curve.CreatePointsAttr([(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)])
    curve.CreateDisplayColorAttr([(1.0, 0.0, 0.0)])
    return curve


def _create_axes_visual(stage, parent_path: str, axis_length_m: float = 0.08, width_m: float = 0.004):
    from pxr import Sdf, UsdGeom  # noqa: PLC0415

    axes_root = UsdGeom.Xform.Define(stage, Sdf.Path(parent_path))
    axis_specs = (
        ("x_axis", (axis_length_m, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ("y_axis", (0.0, axis_length_m, 0.0), (0.0, 1.0, 0.0)),
        ("z_axis", (0.0, 0.0, axis_length_m), (0.0, 0.25, 1.0)),
    )
    for axis_name, end_point, color in axis_specs:
        curve = UsdGeom.BasisCurves.Define(stage, Sdf.Path(f"{parent_path}/{axis_name}"))
        curve.CreateTypeAttr("linear")
        curve.CreateWrapAttr("nonperiodic")
        curve.CreateCurveVertexCountsAttr([2])
        curve.CreatePointsAttr([(0.0, 0.0, 0.0), end_point])
        curve.CreateWidthsAttr([width_m, width_m])
        curve.CreateDisplayColorAttr([color])
    return UsdGeom.Xformable(axes_root.GetPrim())


def _create_cable_gripper_proxy_visual(
    body_name: str = "left_fr3v2_hand",
    finger_z_offset_m: float = 0.0,
    finger_cube_z_offset_m: float = 0.0,
):
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    material = UsdShade.Material.Define(stage, Sdf.Path("/World/Looks/CableProxyRed"))
    shader = UsdShade.Shader.Define(stage, Sdf.Path("/World/Looks/CableProxyRed/PreviewSurface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((1.0, 0.0, 0.0))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.45)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    root_path = "/World/CableGripperProxyVisual"
    body_prim = _find_stage_prim_by_name(stage, body_name)
    if body_prim is not None:
        print(f"CableGripperProxyVisual will track world transform of {body_prim.GetPath()}")
    else:
        print(
            f"Warning: body prim {body_name} not found; driving world-space CableGripperProxyVisual from IsaacLab body pose",
            file=sys.stderr,
        )

    # Keep the red gripper proxy out from under Fabric-driven robot prims.
    # Updating it explicitly in world space avoids USD parent-inheritance lag
    # and flicker when the robot mesh is driven by IsaacLab/Fabric.
    stale_paths = []
    for prim in stage.Traverse():
        if "CableGripperProxyVisual" in prim.GetName():
            stale_paths.append(str(prim.GetPath()))
    for stale_path in sorted(stale_paths, key=len, reverse=True):
        stale_prim = stage.GetPrimAtPath(stale_path)
        if stale_prim.IsValid():
            stage.RemovePrim(stale_path)
            print(f"Removed stale CableGripperProxyVisual prim at {stale_path}")

    root = UsdGeom.Xform.Define(stage, Sdf.Path(root_path))
    root_xform = UsdGeom.Xformable(root.GetPrim())
    root_translate_op = root_xform.AddTranslateOp()
    root_orient_op = root_xform.AddOrientOp()
    root_orient_op.Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    if body_prim is not None and body_prim.IsValid():
        root_axes_path = f"{body_prim.GetPath()}/CableGripperProxyVisual_root_axes"
    else:
        root_axes_path = f"{root_path}/root_axes"
    _create_axes_visual(stage, root_axes_path, axis_length_m=0.10, width_m=0.005)

    finger_visuals = []
    finger_joint_z_m = 0.0584
    franka_finger_collision_boxes = (
        ((0.0, 18.5e-3, 11e-3), (0.0, 0.0, 0.0), (22e-3, 15e-3, 20e-3)),
        ((0.0, 6.8e-3, 2.2e-3), (0.0, 0.0, 0.0), (22e-3, 8.8e-3, 3.8e-3)),
        ((0.0, 15.9e-3, 28.35e-3), (0.5235987755982988, 0.0, 0.0), (17.5e-3, 7e-3, 23.5e-3)),
        ((0.0, 7.58e-3, 45.25e-3), (0.0, 0.0, 0.0), (17.5e-3, 15.2e-3, 18.5e-3)),
    )
    franka_right_finger_collision_rpy = (
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (-0.5235987755982988, 0.0, math.pi),
        (0.0, 0.0, 0.0),
    )
    # Finger body origin in the hand frame. The red collision boxes reproduce
    # cable/sra_gripper.py's local finger collision boxes relative to this origin.
    finger_center_z_m = -(finger_joint_z_m + 0.032) + float(finger_z_offset_m)
    for name, side, roll_rad in (("left_finger", 1.0, 0.0), ("right_finger", -1.0, math.pi)):
        finger_path = f"{root_path}/{name}"
        UsdGeom.Xform.Define(stage, Sdf.Path(finger_path))
        finger_orient_xyzw = _local_z_rotation_xyzw(roll_rad)
        finger_orient = _gf_quatf_from_xyzw(finger_orient_xyzw)

        collision_boxes = []
        for box_index, (box_position_m, left_box_rpy_rad, box_size_m) in enumerate(franka_finger_collision_boxes):
            box_rpy_rad = franka_right_finger_collision_rpy[box_index] if name == "right_finger" else left_box_rpy_rad
            box_orient_xyzw = _quat_xyzw_from_rpy_rad(box_rpy_rad)
            box_path = f"{finger_path}/collision_box_{box_index}"
            box_cube = UsdGeom.Cube.Define(stage, Sdf.Path(box_path))
            box_cube.CreateSizeAttr(1.0)
            UsdShade.MaterialBindingAPI(box_cube.GetPrim()).Bind(material)
            box_xform = UsdGeom.Xformable(box_cube.GetPrim())
            box_translate_op = box_xform.AddTranslateOp()
            box_orient_op = box_xform.AddOrientOp()
            box_scale_op = box_xform.AddScaleOp()
            box_scale_op.Set(Gf.Vec3f(*box_size_m))
            collision_boxes.append(
                {
                    "position_m": tuple(float(v) for v in box_position_m),
                    "orient_xyzw": box_orient_xyzw,
                    "translate_op": box_translate_op,
                    "orient_op": box_orient_op,
                }
            )

        axes_path = f"{root_path}/{name}_axes"
        axes_xform = _create_axes_visual(
            stage,
            axes_path,
            axis_length_m=0.06,
            width_m=0.003,
        )
        axes_translate_op = axes_xform.AddTranslateOp()
        axes_orient_op = axes_xform.AddOrientOp()
        axes_orient_op.Set(finger_orient)

        finger_visuals.append(
            {
                "side": side,
                "collision_boxes": collision_boxes,
                "axes_translate_op": axes_translate_op,
                "axes_orient_op": axes_orient_op,
                "finger_orient": finger_orient,
                "finger_orient_xyzw": finger_orient_xyzw,
                "half_width_m": 0.04,
                "center_z_m": finger_center_z_m,
                "cube_z_offset_m": float(finger_cube_z_offset_m),
            }
        )

    return {
        "root_path": root_path,
        "body_prim": body_prim,
        "root_orientation_offset_xyzw": None,
        "root_translate_op": root_translate_op,
        "root_orient_op": root_orient_op,
        "fingers": finger_visuals,
    }

def _find_stage_prim_by_name(stage, prim_name: str):
    for prim in stage.Traverse():
        if prim.GetName() == prim_name:
            return prim
    return None


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
        return (
            (float(translation[0]), float(translation[1]), float(translation[2])),
            _quat_xyzw_from_gf_quat(quat),
            str(usd_prim.GetPath()),
        )
    except Exception as exc:  # noqa: BLE001 - optional Fabric path
        if not getattr(_fabric_prim_world_pose_by_selector, "_warned", False):
            print(f"Warning: failed to read Fabric live pose for Robotiq finger selector: {exc}", file=sys.stderr)
            _fabric_prim_world_pose_by_selector._warned = True
        return None


def _fabric_prim_world_pose_by_name(prim_name: str):
    try:
        import omni.usd  # noqa: PLC0415
        import usdrt  # noqa: PLC0415
        from pxr import UsdUtils  # noqa: PLC0415
        from usdrt import Rt  # noqa: PLC0415

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        usd_prim = _find_stage_prim_by_name(stage, prim_name)
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
        return (
            (float(translation[0]), float(translation[1]), float(translation[2])),
            _quat_xyzw_from_gf_quat(quat),
        )
    except Exception as exc:  # noqa: BLE001 - optional Fabric path
        if not getattr(_fabric_prim_world_pose_by_name, "_warned", False):
            print(f"Warning: failed to read Fabric live pose for {prim_name}: {exc}", file=sys.stderr)
            _fabric_prim_world_pose_by_name._warned = True
        return None


def _quat_xyzw_multiply(a, b):
    ax, ay, az, aw = (float(v) for v in a)
    bx, by, bz, bw = (float(v) for v in b)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_xyzw_inverse(q):
    x, y, z, w = (float(v) for v in q)
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv_norm = 1.0 / norm_sq
    return (-x * inv_norm, -y * inv_norm, -z * inv_norm, w * inv_norm)


def _quat_xyzw_normalize(q):
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    inv_norm = 1.0 / norm
    return (x * inv_norm, y * inv_norm, z * inv_norm, w * inv_norm)


def _quat_xyzw_from_gf_quat(quat):
    imag = quat.GetImaginary()
    return (float(imag[0]), float(imag[1]), float(imag[2]), float(quat.GetReal()))


def _local_x_rotation_xyzw(angle_rad: float):
    half_angle = 0.5 * float(angle_rad)
    return (math.sin(half_angle), 0.0, 0.0, math.cos(half_angle))


def _local_y_rotation_xyzw(angle_rad: float):
    half_angle = 0.5 * float(angle_rad)
    return (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))


def _local_z_rotation_xyzw(angle_rad: float):
    half_angle = 0.5 * float(angle_rad)
    return (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))


def _quat_xyzw_from_rpy_rad(rpy_rad):
    roll, pitch, yaw = (float(v) for v in rpy_rad)
    q = _local_x_rotation_xyzw(roll)
    q = _quat_xyzw_multiply(q, _local_y_rotation_xyzw(pitch))
    q = _quat_xyzw_multiply(q, _local_z_rotation_xyzw(yaw))
    return _quat_xyzw_normalize(q)


def _quat_xyzw_from_rpy_deg(rpy_deg):
    return _quat_xyzw_from_rpy_rad(tuple(math.radians(float(v)) for v in rpy_deg))


def _gf_quatf_from_xyzw(q):
    from pxr import Gf  # noqa: PLC0415

    qx, qy, qz, qw = _quat_xyzw_normalize(q)
    return Gf.Quatf(qw, qx, qy, qz)


def _set_existing_translate_rotate_zyx(xformable, translation_m, yaw_deg: float):
    from pxr import Gf  # noqa: PLC0415

    prim = xformable.GetPrim()
    translation_attr = prim.GetAttribute("xformOp:translate")
    rotate_attr = prim.GetAttribute("xformOp:rotateZYX")
    if not translation_attr or not translation_attr.IsValid():
        raise RuntimeError(f"Expected existing xformOp:translate on {prim.GetPath()}")
    if not rotate_attr or not rotate_attr.IsValid():
        raise RuntimeError(f"Expected existing xformOp:rotateZYX on {prim.GetPath()}")

    translation_attr.Set(Gf.Vec3d(*tuple(float(v) for v in translation_m)))
    rotate_attr.Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))


def _quat_xyzw_rotate_vector(q, vector):
    vx, vy, vz = (float(v) for v in vector)
    rotated = _quat_xyzw_multiply(
        _quat_xyzw_multiply(q, (vx, vy, vz, 0.0)),
        _quat_xyzw_inverse(q),
    )
    return (rotated[0], rotated[1], rotated[2])


def _vec3d_add(a, b):
    from pxr import Gf  # noqa: PLC0415

    return Gf.Vec3d(float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2]))


def _vec3_sub(a, b):
    return (float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2]))


def _vec3_add_tuple(a, b):
    return (float(a[0]) + float(b[0]), float(a[1]) + float(b[1]), float(a[2]) + float(b[2]))


def _cable_world_yaw_xyzw(yaw_deg: float):
    return _local_z_rotation_xyzw(math.radians(float(yaw_deg)))


def _robot_world_pose_to_cable_world(position_m, quat_xyzw, world_offset_m, world_yaw_deg, gripper_offset_m):
    cable_world_q = _cable_world_yaw_xyzw(world_yaw_deg)
    inv_cable_world_q = _quat_xyzw_inverse(cable_world_q)
    local_position = _quat_xyzw_rotate_vector(inv_cable_world_q, _vec3_sub(position_m, world_offset_m))
    local_position = _vec3_add_tuple(local_position, gripper_offset_m)
    local_quat = _quat_xyzw_multiply(inv_cable_world_q, _quat_xyzw_normalize(quat_xyzw))
    return local_position, _quat_xyzw_normalize(local_quat)


def _default_robotiq_finger_selectors() -> tuple[str, str, str, str]:
    # Cable-world finger ids are ordered opposite to the Robotiq inner_finger link
    # names, so swap each gripper pair to make the red collision boxes open/close
    # in the same visual direction as the gray Robotiq fingers.
    return (
        "left_Robotiq_2F_85/left__Robotiq_2F_85/right_inner_finger",
        "left_Robotiq_2F_85/left__Robotiq_2F_85/left_inner_finger",
        "right_Robotiq_2F_85/right_Robotiq_2F_85/right_inner_finger",
        "right_Robotiq_2F_85/right_Robotiq_2F_85/left_inner_finger",
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
    return _quat_xyzw_rotate_vector(
        _quat_xyzw_inverse(_quat_xyzw_normalize(quat_xyzw)),
        _vec3_sub(world_center_m, position_m),
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
    x, y, z = (float(v) for v in visual_center_offset_m)
    x += float(contact_x_offset_m)
    if abs(y) > 1e-6:
        side = -1.0 if bool(invert_opening) else 1.0
        y = side * math.copysign(abs(float(contact_y_offset_m)), y)
    return (x, y, z + float(contact_z_offset_m))


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
            centered_position_m = _vec3_add_tuple(position_m, _quat_xyzw_rotate_vector(quat_xyzw, local_center_m))
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


def _update_cable_gripper_proxy_visual(
    visual,
    position_m,
    quat_xyzw,
    gap_m,
    visual_offset_m,
    root_local_offset_m=(0.0, 0.0, 0.0),
    root_rpy_offset_deg=(0.0, 0.0, 0.0),
):
    if visual is None:
        return
    from pxr import Gf, UsdGeom  # noqa: PLC0415

    if visual.get("root_translate_op") is not None and visual.get("root_orient_op") is not None:
        live_quat_xyzw = _quat_xyzw_normalize(quat_xyzw)

        # The IsaacLab body tensor is the only source that moves every frame.
        # Calibrate its orientation once against the USD hand prim so the
        if visual.get("root_orientation_offset_xyzw") is None:
            body_prim = visual.get("body_prim")
            if body_prim is not None and body_prim.IsValid():
                world_xform = UsdGeom.XformCache().GetLocalToWorldTransform(body_prim)
                hand_quat = world_xform.ExtractRotationQuat()
                hand_imag = hand_quat.GetImaginary()
                usd_quat_xyzw = _quat_xyzw_normalize(
                    (float(hand_imag[0]), float(hand_imag[1]), float(hand_imag[2]), float(hand_quat.GetReal()))
                )
                visual["root_orientation_offset_xyzw"] = _quat_xyzw_normalize(
                    _quat_xyzw_multiply(_quat_xyzw_inverse(live_quat_xyzw), usd_quat_xyzw)
                )
            else:
                visual["root_orientation_offset_xyzw"] = (0.0, 0.0, 0.0, 1.0)

        root_quat_xyzw = _quat_xyzw_normalize(
            _quat_xyzw_multiply(live_quat_xyzw, visual["root_orientation_offset_xyzw"])
        )
        manual_offset = _quat_xyzw_from_rpy_deg(root_rpy_offset_deg)
        root_quat_xyzw = _quat_xyzw_normalize(_quat_xyzw_multiply(root_quat_xyzw, manual_offset))

        local_offset = tuple(float(v) for v in root_local_offset_m)
        rotated_local_offset = _quat_xyzw_rotate_vector(root_quat_xyzw, local_offset)
        visual_position = tuple(
            float(position_m[i]) + float(visual_offset_m[i]) + rotated_local_offset[i]
            for i in range(3)
        )

        visual["root_translate_op"].Set(Gf.Vec3d(*visual_position))
        qx, qy, qz, qw = root_quat_xyzw
        visual["root_orient_op"].Set(Gf.Quatf(qw, qx, qy, qz))

    gap_m = max(0.0, float(gap_m))
    for finger in visual["fingers"]:
        y = finger["side"] * (0.5 * gap_m)
        axes_position = Gf.Vec3d(0.0, y, finger["center_z_m"])
        finger["axes_translate_op"].Set(axes_position)
        finger["axes_orient_op"].Set(finger["finger_orient"])

        finger_q = finger["finger_orient_xyzw"]
        for box in finger["collision_boxes"]:
            box_local = box["position_m"]
            if finger["cube_z_offset_m"] != 0.0:
                box_local = (box_local[0], box_local[1], box_local[2] + finger["cube_z_offset_m"])
            box_offset = _quat_xyzw_rotate_vector(finger_q, box_local)
            box_position = _vec3d_add(axes_position, box_offset)
            box_orient_xyzw = _quat_xyzw_multiply(finger_q, box["orient_xyzw"])
            box["translate_op"].Set(box_position)
            box["orient_op"].Set(_gf_quatf_from_xyzw(box_orient_xyzw))


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


def _remove_cable_gripper_proxy_visuals(stage) -> None:
    stale_paths = []
    for prim in stage.Traverse():
        if "CableGripperProxyVisual" in prim.GetName():
            stale_paths.append(str(prim.GetPath()))
    for stale_path in sorted(stale_paths, key=len, reverse=True):
        stale_prim = stage.GetPrimAtPath(stale_path)
        if stale_prim.IsValid():
            stage.RemovePrim(stale_path)
            print(f"Removed stale CableGripperProxyVisual prim at {stale_path}")


def _create_cable_gripper_collision_box_visual(visual_offset_m, visual_yaw_deg: float = 0.0):
    from pxr import Gf, Sdf, UsdGeom, UsdShade  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    _remove_cable_gripper_proxy_visuals(stage)

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
    root_xform.AddOrientOp().Set(_gf_quatf_from_xyzw(_local_z_rotation_xyzw(math.radians(float(visual_yaw_deg)))))

    return {
        "root_path": root_path,
        "material": material,
        "boxes": {},
    }


def _create_cable_gripper_root_pose_visual(visual_offset_m, visual_yaw_deg: float = 0.0):
    from pxr import Gf, Sdf, UsdGeom  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    root_path = "/World/CableGripperRootVisual"
    root_prim = stage.GetPrimAtPath(root_path)
    if root_prim.IsValid():
        stage.RemovePrim(root_path)

    root = UsdGeom.Xform.Define(stage, Sdf.Path(root_path))
    root_xform = UsdGeom.Xformable(root.GetPrim())
    root_xform.AddTranslateOp().Set(Gf.Vec3d(*tuple(float(v) for v in visual_offset_m)))
    root_xform.AddOrientOp().Set(_gf_quatf_from_xyzw(_local_z_rotation_xyzw(math.radians(float(visual_yaw_deg)))))

    axes = _create_axes_visual(stage, f"{root_path}/root_axes", axis_length_m=0.12, width_m=0.006)
    axes_translate_op = axes.AddTranslateOp()
    axes_orient_op = axes.AddOrientOp()
    axes_orient_op.Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return {
        "translate_op": axes_translate_op,
        "orient_op": axes_orient_op,
    }


def _update_cable_gripper_root_pose_visual(visual, root_pose):
    if visual is None or root_pose is None:
        return
    from pxr import Gf  # noqa: PLC0415

    position_m = tuple(float(v) for v in root_pose.get("position_m", (0.0, 0.0, 0.0)))
    quat_xyzw = _quat_xyzw_normalize(root_pose.get("quat_xyzw", (0.0, 0.0, 0.0, 1.0)))
    qx, qy, qz, qw = quat_xyzw
    visual["translate_op"].Set(Gf.Vec3d(*position_m))
    visual["orient_op"].Set(Gf.Quatf(qw, qx, qy, qz))


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
        quat_xyzw = _quat_xyzw_normalize(box.get("quat_xyzw", (0.0, 0.0, 0.0, 1.0)))
        size_m = tuple(float(v) for v in box.get("size_m", (0.01, 0.01, 0.01)))
        qx, qy, qz, qw = quat_xyzw
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

    groups = _load_joint_groups(
        franka_root,
        args_cli.embodiment,
        include_browser_commands=not args_cli.disable_browser_command_topics,
    )
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

    SceneCfg = _make_scene_cfg(str(usd_path), args_cli.robot_prim_path)
    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=0.0, replicate_physics=False))
    robot_prim_path = _env_robot_prim_path(args_cli.robot_prim_path)
    _fix_single_articulation_root(robot_prim_path)
    _fix_newton_reversed_fixed_joints(robot_prim_path)
    sim.reset()
    scene.reset()

    robot = scene["robot"]
    actual_joint_names = _joint_names(robot)
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
    if args_cli.spine_keyboard_control:
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
    cable_gripper_root_visual = None
    if args_cli.with_cable:
        cable_config_path = _resolve_path(franka_root, args_cli.cable_config_path)
        cable_world_offset = tuple(float(v) for v in args_cli.cable_world_position_offset)
        cable_world_yaw_deg = float(args_cli.cable_world_yaw_deg)
        cable_curve_visual = _create_cable_stage_visuals(
            franka_root, cable_config_path, cable_world_offset, cable_world_yaw_deg
        )
        cable_gripper_collision_visual = _create_cable_gripper_collision_box_visual(
            cable_world_offset, cable_world_yaw_deg
        )
        cable_gripper_root_visual = _create_cable_gripper_root_pose_visual(cable_world_offset, cable_world_yaw_deg)
        print(
            "Cable VBD ROS coupling enabled: "
            f"config={cable_config_path} gripper_body={args_cli.cable_gripper_body_name} "
            f"visual_offset={cable_world_offset} visual_yaw_deg={cable_world_yaw_deg:.3f}"
        )

    rclpy.init()
    node = IsaacLabRosBridge(groups, enable_cable=args_cli.with_cable)
    publish_period = 1.0 / max(args_cli.ros_publish_rate, 1.0)
    next_publish_time = 0.0
    logged_fabric_gripper_pose = False
    logged_missing_fabric_gripper_pose = False
    logged_robotiq_finger_targets = False
    logged_missing_robotiq_finger_targets = False
    robotiq_finger_selectors = tuple(args_cli.cable_robotiq_finger_prim) or _default_robotiq_finger_selectors()

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
                fabric_pose = _fabric_prim_world_pose_by_name(args_cli.cable_gripper_body_name)
                if fabric_pose is None:
                    if not logged_missing_fabric_gripper_pose:
                        logged_missing_fabric_gripper_pose = True
                        print(
                            f"Warning: Fabric live USD prim pose unavailable for {args_cli.cable_gripper_body_name}; skipping cable gripper publish",
                            file=sys.stderr,
                        )
                else:
                    position_m, quat_xyzw = fabric_pose
                    if not logged_fabric_gripper_pose:
                        logged_fabric_gripper_pose = True
                        print(
                            f"Cable gripper pose uses Fabric live USD prim pose for {args_cli.cable_gripper_body_name}",
                            flush=True,
                        )

                    gripper_offset = tuple(float(v) for v in args_cli.cable_gripper_position_offset)
                    world_offset = tuple(float(v) for v in args_cli.cable_world_position_offset)
                    position_m, quat_xyzw = _robot_world_pose_to_cable_world(
                        position_m,
                        quat_xyzw,
                        world_offset,
                        args_cli.cable_world_yaw_deg,
                        gripper_offset,
                    )
                    gap_m = _robot_gripper_gap_m(
                        robot,
                        side=args_cli.cable_gripper_side,
                        actual_joint_names=actual_joint_names,
                        group_indices=group_indices,
                        cable_gap_range=(0.0, 0.08),
                        fixed_gap_m=args_cli.cable_gripper_gap_m,
                    )
                    node.publish_cable_gripper(
                        position_m=position_m,
                        quat_xyzw=quat_xyzw,
                        gap_m=gap_m,
                    )
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
                _update_cable_stage_visual(cable_curve_visual, node.latest_cable_points())
                _update_cable_gripper_collision_box_visual(
                    cable_gripper_collision_visual,
                    node.latest_cable_gripper_boxes(),
                )
                _update_cable_gripper_root_pose_visual(
                    cable_gripper_root_visual,
                    node.latest_cable_gripper_root_pose(),
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
