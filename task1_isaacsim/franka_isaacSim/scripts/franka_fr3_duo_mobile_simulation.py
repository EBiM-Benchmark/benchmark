#!/usr/bin/env python3
"""Launch IsaacLab and visualize the FR3 Duo mobile robot asset.

This mirrors the structure of ``franka_panda_simulation.py`` and the official
``FRANKA_PANDA_CFG``: the robot is described as an ``ArticulationCfg`` with a
USD spawn config, initial joint targets, and implicit actuators.

Example:

    cd /workspace/isaaclab
    ./isaaclab.sh -p /workspace/franka_isaacSim/scripts/franka_fr3_duo_mobile_simulation.py \
        --fr3duo_mobile-visualizer kit
"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher


DEFAULT_USD_PATH = (
    "/workspace/franka_isaacSim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd"
)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd-path", default=DEFAULT_USD_PATH, help="FR3 Duo mobile robot USD file to load.")
    parser.add_argument("--prim-path", default="/World/Env_0/Robot", help="Prim path where the robot is spawned.")
    parser.add_argument(
        "--fr3duo-visualizer",
        "--fr3duo_mobile-visualizer",
        dest="fr3duo_mobile_visualizer",
        choices=["none", "kit"],
        default="kit",
        help="IsaacLab visualizer backend for this demo.",
    )
    parser.add_argument("--camera-position", type=float, nargs=3, default=(0.0, 2.0, 5.0))
    parser.add_argument("--camera-target", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    AppLauncher.add_app_launcher_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
if args_cli.fr3duo_mobile_visualizer == "kit" and not getattr(args_cli, "visualizer", None):
    # AppLauncher must know about the Kit visualizer before SimulationApp starts;
    # otherwise it may select a minimal/headless experience without viewport modules.
    args_cli.visualizer = ["kit"]
    args_cli.visualizer_explicit = True
simulation_app = AppLauncher(args_cli).app

import torch  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, build_simulation_context  # noqa: E402
from isaaclab_newton.assets import Articulation  # noqa: E402
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg  # noqa: E402

try:
    from isaaclab_visualizers.kit import KitVisualizerCfg  # noqa: E402
except ImportError:
    KitVisualizerCfg = None


NEWTON_REVERSED_FIXED_JOINTS = (
    "argo_drive_front_fixed_joint",
    "base_joint",
    "zed_mini_camera_joint",
)


FR3_DUO_HOME_JOINT_POS = {
    # Left FR3 arm.
    "left_fr3v2_joint1": 0.0,
    "left_fr3v2_joint2": -0.7854,
    "left_fr3v2_joint3": 0.0,
    "left_fr3v2_joint4": -2.3562,
    "left_fr3v2_joint5": 0.0,
    "left_fr3v2_joint6": 1.5708,
    "left_fr3v2_joint7": 0.7854,
    # Right FR3 arm.
    "right_fr3v2_joint1": 0.0,
    "right_fr3v2_joint2": -0.7854,
    "right_fr3v2_joint3": 0.0,
    "right_fr3v2_joint4": -2.3562,
    "right_fr3v2_joint5": 0.0,
    "right_fr3v2_joint6": 1.5708,
    "right_fr3v2_joint7": 0.7854,
    # Robotiq finger joints in this USD do not contain the word "robotiq".
    "(left|right)_.*(finger|knuckle).*": 0.04,
}


def _resolve_asset_path(path_text: str) -> str:
    path = Path(path_text).expanduser()
    if path.exists():
        return str(path)

    mappings = [
        (Path("/homeL/qiguan/dataSSD/franka_isaacSim"), Path("/workspace/franka_isaacSim")),
        (Path("/workspace/franka_isaacSim"), Path("/homeL/qiguan/dataSSD/franka_isaacSim")),
    ]
    for src, dst in mappings:
        try:
            rel = path.relative_to(src)
        except ValueError:
            continue
        candidate = dst / rel
        if candidate.exists():
            return str(candidate)

    raise FileNotFoundError(f"Robot USD path does not exist: {path_text}")


def _make_visualizer_cfgs():
    if args_cli.fr3duo_mobile_visualizer == "none" or KitVisualizerCfg is None:
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


def _iter_prims_under(root_prim):
    yield root_prim
    for child in root_prim.GetChildren():
        yield from _iter_prims_under(child)



def _fix_single_articulation_root(robot_prim_path: str) -> None:
    """Ensure IsaacLab sees exactly one ArticulationRootAPI under the robot prim."""
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"[WARN] Cannot patch articulation roots: robot prim not found: {robot_prim_path}", flush=True)
        return

    root_prims = [
        prim
        for prim in _iter_prims_under(robot_prim)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(root_prims) <= 1:
        return

    preferred_paths = (
        f"{robot_prim_path}/base",
        f"{robot_prim_path}/base_link",
    )
    keep_prim = None
    for preferred_path in preferred_paths:
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

    print(f"[INFO] Keeping articulation root: {keep_prim.GetPath()}", flush=True)
    if removed:
        print("[INFO] Removed extra articulation roots:", flush=True)
        for prim_path in removed:
            print(f"  {prim_path}", flush=True)


def _fix_newton_reversed_fixed_joints(robot_prim_path: str) -> None:
    """Patch fixed joints whose body0/body1 order is rejected by Newton.

    The Isaac Sim importer can author some fixed joints with the child body in
    physics:body0 and the parent body in physics:body1. PhysX tolerates that,
    but Newton's USD importer rejects it. For fixed joints, swapping body0/body1
    together with localPos/localRot preserves the constraint frame while making
    the graph direction Newton-compatible.
    """
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"[WARN] Cannot patch reversed joints: robot prim not found: {robot_prim_path}", flush=True)
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
                f"[WARN] Skipping reversed-joint patch for {joint_path}: "
                f"expected PhysicsFixedJoint, got {prim.GetTypeName()}",
                flush=True,
            )
            continue
        if not _swap_relationship_targets(prim, "physics:body0", "physics:body1"):
            print(f"[WARN] Could not swap body0/body1 for {joint_path}: missing targets", flush=True)
            continue
        _swap_attr_values(prim, "physics:localPos0", "physics:localPos1")
        _swap_attr_values(prim, "physics:localRot0", "physics:localRot1")
        patched.append(joint_path)

    missing = wanted - seen
    if missing:
        print("[WARN] Reversed-joint patch names not found: " + ", ".join(sorted(missing)), flush=True)
    if patched:
        print("[INFO] Patched Newton-reversed fixed joints:", flush=True)
        for joint_path in patched:
            print(f"  {joint_path}", flush=True)


def _make_fr3_duo_mobile_cfg(usd_path: str) -> ArticulationCfg:
    return ArticulationCfg(
        prim_path=args_cli.prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=usd_path,
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos=FR3_DUO_HOME_JOINT_POS),
        actuators={
            "left_fr3v2_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["left_fr3v2_joint[1-4]"],
                effort_limit_sim=87.0,
                velocity_limit_sim=2.175,
                stiffness=625.0,
                damping=60.0,
                armature=1e-3,
            ),
            "left_fr3v2_forearm": ImplicitActuatorCfg(
                joint_names_expr=["left_fr3v2_joint[5-7]"],
                effort_limit_sim=12.0,
                velocity_limit_sim=2.61,
                stiffness=625.0,
                damping=40.0,
                armature=1e-3,
            ),
            "right_fr3v2_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["right_fr3v2_joint[1-4]"],
                effort_limit_sim=87.0,
                velocity_limit_sim=2.175,
                stiffness=625.0,
                damping=60.0,
                armature=1e-3,
            ),
            "right_fr3v2_forearm": ImplicitActuatorCfg(
                joint_names_expr=["right_fr3v2_joint[5-7]"],
                effort_limit_sim=12.0,
                velocity_limit_sim=2.61,
                stiffness=625.0,
                damping=40.0,
                armature=1e-3,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=[
                    "left_right_finger_joint",
                    "right_right_finger_joint",
                    "left_right_inner_finger_joint",
                    "left_left_inner_finger_joint",
                    "right_right_inner_finger_joint",
                    "right_left_inner_finger_joint",      
                    "left_right_inner_finger_knuckle_joint",
                    "left_left_inner_finger_knuckle_joint",
                    "right_right_inner_finger_knuckle_joint",
                    "right_left_inner_finger_knuckle_joint",
                    "left_outer_knuckle_joint",
                    "right_outer_knuckle_joint",                          
                ],
                effort_limit_sim=200,
                velocity_limit_sim=10.0,
                stiffness=1000,
                damping=100,
            ),
            "mobile_base": ImplicitActuatorCfg(
                joint_names_expr=[
                    "tmrv0_2_joint_.*",
                    "caster_.*",
                    "franka_spine_vertical_joint",
                ],
                effort_limit_sim=50,
                velocity_limit_sim=1.5708,
                stiffness=0.0,
                damping=1e5,
            ),
            "lift": ImplicitActuatorCfg(
                joint_names_expr=[
                    "franka_spine_vertical_joint",
                ],
                effort_limit_sim=500,
                velocity_limit_sim=1,
                stiffness=0.0,
                damping=1e5,
            ),
        },
        soft_joint_pos_limit_factor=1.0,
    )


def main() -> None:
    usd_path = _resolve_asset_path(args_cli.usd_path)
    print(f"[INFO] Loading FR3 Duo mobile USD: {usd_path}", flush=True)

    visualizer_cfgs = _make_visualizer_cfgs()
    sim_cfg = SimulationCfg(
        dt=1.0 / 120,
        physics=NewtonCfg(
            solver_cfg=MJWarpSolverCfg(
                njmax=2048,
                nconmax=512,
                ls_iterations=20,
                cone="pyramidal",
                impratio=1.0,
                ls_parallel=False,
                integrator="implicitfast",
            ),
            num_substeps=1,
            debug_mode=False,
        ),
    )
    if visualizer_cfgs and hasattr(sim_cfg, "visualizer_cfgs"):
        sim_cfg.visualizer_cfgs = visualizer_cfgs

    with build_simulation_context(
        device=args_cli.device,
        auto_add_lighting=True,
        add_ground_plane=True,
        sim_cfg=sim_cfg,
    ) as sim:
        sim._app_control_on_stop_handle = None
        sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))
        sim_utils.create_prim("/World/Env_0", "Xform", translation=(0.0, 0.0, 0.0))

        robot_cfg = _make_fr3_duo_mobile_cfg(usd_path)
        robot = Articulation(robot_cfg)
        _fix_single_articulation_root(args_cli.prim_path)
        _fix_newton_reversed_fixed_joints(args_cli.prim_path)

        sim.reset()
        sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))
        print("[INFO] Simulation started.", flush=True)
        print(f"[INFO] Robot initialized: {robot.is_initialized}", flush=True)
        print("[INFO] Joint names:", ", ".join(robot.data.joint_names), flush=True)

        # Hold the configured initial pose using the same pattern as the Panda demo.
        joint_pos_target = robot.data.default_joint_pos.torch.clone()
        robot.write_joint_position_to_sim_index(position=joint_pos_target)
        robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch.clone())

        while simulation_app.is_running():
            robot.set_joint_position_target_index(target=joint_pos_target)
            robot.write_data_to_sim()
            sim.step()
            robot.update(sim.cfg.dt)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
