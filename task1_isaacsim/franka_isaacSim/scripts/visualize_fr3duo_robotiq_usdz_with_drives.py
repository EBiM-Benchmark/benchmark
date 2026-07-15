#!/usr/bin/env python3
"""Visualize the FR3 Duo mobile Robotiq USDZ with drive gains from YAML.

This script loads the USDZ asset, builds an IsaacLab ArticulationCfg, and maps
joint drive values from:

- assets/embodiments/fr3duo_mobile/joint_drive_config.yaml
- assets/embodiments/fr3duo_mobile/isaac_joint_drives.yaml

Example inside the IsaacLab container:

    cd /workspace/isaaclab
    ./isaaclab.sh -p /workspace/franka_isaacSim/scripts/visualize_fr3duo_robotiq_usdz_with_drives.py \
        --visualizer kit
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher


DEFAULT_FRANKA_ROOT = "/workspace/franka_isaacSim"
DEFAULT_USDZ_PATH = (
    "/workspace/franka_isaacSim/assets/franka_description/urdfs/mobile_fr3_duo_v0_2_robotiq_2f_85/"
    "mobile_fr3_duo_v0_2_robotiq_2f_85.usd"
)
DEFAULT_EMBODIMENT = "fr3duo_mobile"

LEFT_ARM_JOINTS = tuple(f"left_fr3v2_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"right_fr3v2_joint{i}" for i in range(1, 8))

HOME_JOINT_POS = {
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
    ".*(finger|knuckle).*": 0.04,
}


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usdz-path", default=DEFAULT_USDZ_PATH, help="Robot USDZ path to spawn.")
    parser.add_argument("--franka-root", default=DEFAULT_FRANKA_ROOT, help="franka_isaacSim root path.")
    parser.add_argument("--embodiment", default=DEFAULT_EMBODIMENT, help="Embodiment key under assets/embodiments.")
    parser.add_argument("--prim-path", default="/World/Env_0/Robot", help="Prim path where the robot is spawned.")
    parser.add_argument(
        "--physics-backend",
        choices=("physx", "newton"),
        default="newton",
        help="Use Newton/MJWarp or switch to physx for comparison.",
    )
    parser.add_argument("--camera-position", type=float, nargs=3, default=(2.8, 2.2, 1.8))
    parser.add_argument("--camera-target", type=float, nargs=3, default=(0.0, 0.0, 0.55))
    parser.add_argument(
        "--fr3duo-visualizer",
        choices=("none", "kit"),
        default="kit",
        help="IsaacLab visualizer backend for this standalone viewer.",
    )
    parser.add_argument("--print-drives", action="store_true", help="Print generated actuator drive values.")
    AppLauncher.add_app_launcher_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
if args_cli.fr3duo_visualizer == "kit" and not getattr(args_cli, "visualizer", None):
    args_cli.visualizer = ["kit"]
    args_cli.visualizer_explicit = True
simulation_app = AppLauncher(args_cli).app

import omni.usd  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation as PhysxArticulation  # noqa: E402
from isaaclab.assets import ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, build_simulation_context  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

try:
    import yaml  # noqa: E402
except ImportError:  # pragma: no cover - depends on container image
    yaml = None

try:
    from isaaclab_visualizers.kit import KitVisualizerCfg  # noqa: E402
except ImportError:  # pragma: no cover - depends on app experience
    KitVisualizerCfg = None

try:
    from isaaclab_newton.assets import Articulation as NewtonArticulation  # noqa: E402
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg  # noqa: E402
except ImportError:  # pragma: no cover - optional backend
    NewtonArticulation = None
    MJWarpSolverCfg = None
    NewtonCfg = None


def _resolve_path(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    if path.exists():
        return path

    mappings = (
        (Path("/homeL/qiguan/dataSSD/franka_isaacSim"), Path("/workspace/franka_isaacSim")),
        (Path("/workspace/franka_isaacSim"), Path("/homeL/qiguan/dataSSD/franka_isaacSim")),
        (Path("/dataSSD/qiguan/franka_isaacSim"), Path("/workspace/franka_isaacSim")),
        (Path("/workspace/franka_isaacSim"), Path("/dataSSD/qiguan/franka_isaacSim")),
    )
    for src, dst in mappings:
        try:
            rel = path.relative_to(src)
        except ValueError:
            continue
        candidate = dst / rel
        if candidate.exists():
            return candidate

    raise FileNotFoundError(f"Path does not exist: {path_text}")


def _make_visualizer_cfgs():
    if args_cli.fr3duo_visualizer == "none" or KitVisualizerCfg is None:
        return []
    cfg = KitVisualizerCfg()
    desired_attrs = {
        "viewport_name": "FR3 Duo Robotiq Viewer",
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


def _load_scaled_joint_drive_configs(franka_root: Path, embodiment: str) -> dict[str, dict[str, float]]:
    if yaml is None:
        raise RuntimeError("PyYAML is not available; cannot load joint drive YAML files.")

    embodiment_dir = franka_root / "assets" / "embodiments" / embodiment
    joint_drive_config_path = embodiment_dir / "joint_drive_config.yaml"
    if not joint_drive_config_path.exists():
        raise FileNotFoundError(f"joint_drive_config.yaml not found: {joint_drive_config_path}")

    with joint_drive_config_path.open("r", encoding="utf-8") as f:
        joint_drive_config = yaml.safe_load(f) or {}

    raw_drive_path = (
        (joint_drive_config.get("isaac_joint_drives", {}) or {}).get("joint_drive_config")
        or "isaac_joint_drives.yaml"
    )
    drive_path = Path(raw_drive_path)
    if not drive_path.is_absolute():
        drive_path = embodiment_dir / drive_path
    if not drive_path.exists():
        raise FileNotFoundError(f"isaac_joint_drives.yaml not found: {drive_path}")

    with drive_path.open("r", encoding="utf-8") as f:
        drive_config = yaml.safe_load(f) or {}

    scaling = joint_drive_config.get("scaling_parameters", {}) or {}
    arm_scaling = scaling.get("arms", {}) or {}
    gripper_scaling = scaling.get("grippers", {}) or {}

    scales = {
        "arm": {
            "stiffness": float(arm_scaling.get("stiffness_scale", 1.0)),
            "damping": float(arm_scaling.get("damping_scale", 1.0)),
            "max_force": float(arm_scaling.get("max_force_scale", 1.0)),
        },
        "gripper": {
            "stiffness": float(gripper_scaling.get("stiffness_scale", 1.0)),
            "damping": float(gripper_scaling.get("damping_scale", 1.0)),
            "max_force": float(gripper_scaling.get("max_force_scale", 1.0)),
        },
    }

    scaled = {}
    for joint_name, cfg in (drive_config.get("joint_drives", {}) or {}).items():
        if not isinstance(cfg, dict):
            continue
        lower_name = str(joint_name).lower()
        group = "gripper" if "robotiq" in lower_name or "finger" in lower_name or "knuckle" in lower_name else "arm"
        scaled[str(joint_name)] = {
            "stiffness": float(cfg.get("stiffness", 0.0)) * scales[group]["stiffness"],
            "damping": float(cfg.get("damping", 0.0)) * scales[group]["damping"],
            "max_force": float(cfg.get("max_force", 0.0)) * scales[group]["max_force"],
        }

    print(f"[INFO] Loaded joint drive config: {drive_path}", flush=True)
    print(
        "[INFO] Drive scaling: "
        f"arms={scales['arm']} grippers={scales['gripper']}",
        flush=True,
    )
    return scaled


def _first_gripper_drive(scaled_drives: dict[str, dict[str, float]]) -> dict[str, float]:
    for joint_name, drive in scaled_drives.items():
        lower_name = joint_name.lower()
        if "robotiq" in lower_name or "finger" in lower_name or "knuckle" in lower_name:
            return drive
    return {"stiffness": 1000.0, "damping": 100.0, "max_force": 200.0}


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


def _fix_single_articulation_root(robot_prim_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[WARN] Cannot patch articulation roots: no USD stage", file=sys.stderr, flush=True)
        return
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"[WARN] Cannot patch articulation roots: robot prim not found: {robot_prim_path}", file=sys.stderr, flush=True)
        return

    try:
        stage.Load(robot_prim.GetPath(), Usd.LoadWithDescendants)
    except Exception as exc:
        print(f"[WARN] Could not force-load robot payloads: {exc}", file=sys.stderr, flush=True)

    root_prims = [
        prim
        for prim in stage.TraverseAll()
        if (str(prim.GetPath()) == robot_prim_path or str(prim.GetPath()).startswith(robot_prim_path + "/"))
        and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
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

    print(f"[INFO] Keeping articulation root: {keep_prim.GetPath()}", flush=True)
    if removed:
        print("[INFO] Removed extra articulation roots:", flush=True)
        for prim_path in removed:
            print(f"  {prim_path}", flush=True)


def _fix_newton_reversed_fixed_joints(robot_prim_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[WARN] Cannot patch reversed joints: no USD stage", file=sys.stderr, flush=True)
        return
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        print(f"[WARN] Cannot patch reversed joints: robot prim not found: {robot_prim_path}", file=sys.stderr, flush=True)
        return

    try:
        stage.Load(robot_prim.GetPath(), Usd.LoadWithDescendants)
    except Exception as exc:
        print(f"[WARN] Could not force-load robot payloads before reversed-joint patch: {exc}", file=sys.stderr, flush=True)

    wanted = set(NEWTON_REVERSED_FIXED_JOINTS)
    patched = []
    seen = set()
    for prim in stage.TraverseAll():
        prim_path = str(prim.GetPath())
        if prim_path != robot_prim_path and not prim_path.startswith(robot_prim_path + "/"):
            continue
        if prim.GetName() not in wanted:
            continue
        seen.add(prim.GetName())
        if not _swap_relationship_targets(prim, "physics:body0", "physics:body1"):
            print(f"[WARN] Could not swap body0/body1 for {prim_path}: missing targets", file=sys.stderr, flush=True)
            continue
        _swap_attr_values(prim, "physics:localPos0", "physics:localPos1")
        _swap_attr_values(prim, "physics:localRot0", "physics:localRot1")
        patched.append(prim_path)

    missing = wanted - seen
    if missing:
        print("[WARN] Reversed fixed joints not found: " + ", ".join(sorted(missing)), file=sys.stderr, flush=True)
    if patched:
        print("[INFO] Patched Newton-reversed fixed joints:", flush=True)
        for prim_path in patched:
            print(f"  {prim_path}", flush=True)


def _patch_stage_for_newton(robot_prim_path: str) -> None:
    if args_cli.physics_backend != "newton":
        return
    _fix_single_articulation_root(robot_prim_path)
    _fix_newton_reversed_fixed_joints(robot_prim_path)


def _make_actuator_cfgs(franka_root: Path, embodiment: str) -> dict[str, ImplicitActuatorCfg]:
    scaled_drives = _load_scaled_joint_drive_configs(franka_root, embodiment)
    actuators: dict[str, ImplicitActuatorCfg] = {}

    for joint_name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS:
        drive = scaled_drives.get(joint_name)
        if drive is None:
            print(f"[WARN] Missing drive entry for arm joint: {joint_name}", file=sys.stderr, flush=True)
            continue
        actuators[f"drive_{joint_name}"] = ImplicitActuatorCfg(
            joint_names_expr=[joint_name],
            effort_limit_sim=drive["max_force"],
            stiffness=drive["stiffness"],
            damping=drive["damping"],
        )

    gripper_drive = _first_gripper_drive(scaled_drives)
    actuators["robotiq_grippers"] = ImplicitActuatorCfg(
        joint_names_expr=[".*(finger|knuckle).*"],
        effort_limit_sim=gripper_drive["max_force"],
        stiffness=gripper_drive["stiffness"],
        damping=gripper_drive["damping"],
    )

    if args_cli.print_drives:
        print("[INFO] Generated IsaacLab actuators:", flush=True)
        for name, cfg in actuators.items():
            print(
                f"  {name}: joints={cfg.joint_names_expr}, "
                f"effort_limit_sim={cfg.effort_limit_sim}, stiffness={cfg.stiffness}, damping={cfg.damping}",
                flush=True,
            )

    return actuators


def _make_robot_cfg(usdz_path: Path, franka_root: Path) -> ArticulationCfg:
    return ArticulationCfg(
        prim_path=args_cli.prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usdz_path),
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
        init_state=ArticulationCfg.InitialStateCfg(joint_pos=HOME_JOINT_POS),
        actuators=_make_actuator_cfgs(franka_root, args_cli.embodiment),
        soft_joint_pos_limit_factor=1.0,
    )


def _make_sim_cfg() -> SimulationCfg:
    visualizer_cfgs = _make_visualizer_cfgs()
    if args_cli.physics_backend == "newton":
        if NewtonCfg is None or MJWarpSolverCfg is None:
            raise RuntimeError("Newton backend is unavailable in this IsaacLab environment.")
        cfg = SimulationCfg(
            dt=1.0 / 120.0,
            physics=NewtonCfg(
                solver_cfg=MJWarpSolverCfg(
                    njmax=4096,
                    nconmax=1024,
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
    else:
        cfg = SimulationCfg(dt=1.0 / 120.0)

    if visualizer_cfgs and hasattr(cfg, "visualizer_cfgs"):
        cfg.visualizer_cfgs = visualizer_cfgs
    return cfg


def main() -> None:
    usdz_path = _resolve_path(args_cli.usdz_path)
    franka_root = _resolve_path(args_cli.franka_root)
    articulation_cls = NewtonArticulation if args_cli.physics_backend == "newton" else PhysxArticulation
    if articulation_cls is None:
        raise RuntimeError(f"Articulation class for backend {args_cli.physics_backend!r} is unavailable.")

    print(f"[INFO] Loading robot USDZ: {usdz_path}", flush=True)
    print(f"[INFO] Franka root: {franka_root}", flush=True)
    print(f"[INFO] Physics backend: {args_cli.physics_backend}", flush=True)

    with build_simulation_context(
        device=args_cli.device,
        auto_add_lighting=True,
        add_ground_plane=True,
        sim_cfg=_make_sim_cfg(),
    ) as sim:
        sim._app_control_on_stop_handle = None
        sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))
        sim_utils.create_prim("/World/Env_0", "Xform", translation=(0.0, 0.0, 0.0))

        robot_cfg = _make_robot_cfg(usdz_path, franka_root)
        robot_cfg.spawn.func(
            args_cli.prim_path,
            robot_cfg.spawn,
            translation=robot_cfg.init_state.pos,
            orientation=robot_cfg.init_state.rot,
        )
        stage = omni.usd.get_context().get_stage()
        robot_cfg._post_spawn(stage)
        _patch_stage_for_newton(args_cli.prim_path)
        robot_cfg.spawn = None
        robot = articulation_cls(robot_cfg)
        sim.reset()
        sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))

        print(f"[INFO] Robot initialized: {robot.is_initialized}", flush=True)
        print("[INFO] Joint names:", ", ".join(robot.data.joint_names), flush=True)

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
