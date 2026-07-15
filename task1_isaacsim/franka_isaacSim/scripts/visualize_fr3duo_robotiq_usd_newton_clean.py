#!/usr/bin/env python3
"""Visualize FR3 Duo mobile Robotiq USD with Newton and YAML joint drives.

This is intentionally a clean loader: it does not patch ArticulationRootAPI,
physics:body0/body1, or any joint direction. The USD is loaded exactly as
provided, while IsaacLab actuator gains are assigned from:

- assets/embodiments/fr3duo_mobile/isaac_joint_drives.yaml
- assets/embodiments/fr3duo_mobile/joint_drive_config.yaml

Example inside the IsaacLab container:

    cd /workspace/isaaclab
    ./isaaclab.sh -p /workspace/franka_isaacSim/scripts/visualize_fr3duo_robotiq_usd_newton_clean.py \
        --visualizer kit --print-drives
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher


DEFAULT_FRANKA_ROOT = "/workspace/franka_isaacSim"
DEFAULT_USD_PATH = (
    "/workspace/franka_isaacSim/assets/franka_description/urdfs/"
    "mobile_fr3_duo_v0_2_robotiq_2f_85/mobile_fr3_duo_v0_2_robotiq_2f_85.usd"
)
DEFAULT_EMBODIMENT = "fr3duo_mobile"

LEFT_ARM_JOINTS = tuple(f"left_fr3v2_joint{i}" for i in range(1, 8))
RIGHT_ARM_JOINTS = tuple(f"right_fr3v2_joint{i}" for i in range(1, 8))
GRIPPER_JOINT_EXPR = ".*(robotiq|finger|knuckle).*"
MOBILE_BASE_HOLD_JOINT_EXPR = ".*(caster|rocker|tmrv0_2|spine).*"

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
    GRIPPER_JOINT_EXPR: 0.0,
    MOBILE_BASE_HOLD_JOINT_EXPR: 0.0,
}


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd-path", default=DEFAULT_USD_PATH, help="Robot USD path to load.")
    parser.add_argument("--franka-root", default=DEFAULT_FRANKA_ROOT, help="franka_isaacSim root path.")
    parser.add_argument("--embodiment", default=DEFAULT_EMBODIMENT, help="Embodiment key under assets/embodiments.")
    parser.add_argument("--prim-path", default="/World/Env_0/Robot", help="Prim path where the robot is spawned.")
    parser.add_argument("--camera-position", type=float, nargs=3, default=(2.8, 2.2, 1.8))
    parser.add_argument("--camera-target", type=float, nargs=3, default=(0.0, 0.0, 0.55))
    parser.add_argument(
        "--fr3duo-visualizer",
        choices=("none", "kit"),
        default="kit",
        help="IsaacLab visualizer backend for this standalone viewer.",
    )
    parser.add_argument("--print-drives", action="store_true", help="Print generated actuator drive values.")
    parser.add_argument(
        "--physics-backend",
        choices=("newton", "physx"),
        default="newton",
        help="Physics backend to use. 'physx' uses IsaacLab's default PhysX manager.",
    )
    parser.add_argument(
        "--enable-gravity",
        action="store_true",
        help="Enable gravity on robot rigid bodies. By default gravity is disabled for stable USD/Newton visualization.",
    )
    parser.add_argument(
        "--no-physics-step",
        action="store_true",
        help="Load and display the USD after reset, but do not advance physics steps.",
    )
    AppLauncher.add_app_launcher_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
if args_cli.fr3duo_visualizer == "kit" and not getattr(args_cli, "visualizer", None):
    args_cli.visualizer = ["kit"]
    args_cli.visualizer_explicit = True
simulation_app = AppLauncher(args_cli).app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation as PhysxArticulation  # noqa: E402
from isaaclab.assets import ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, build_simulation_context  # noqa: E402
from isaaclab_newton.assets import Articulation as NewtonArticulation  # noqa: E402
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg  # noqa: E402

try:
    import yaml  # noqa: E402
except ImportError:  # pragma: no cover
    yaml = None

try:
    from isaaclab_visualizers.kit import KitVisualizerCfg  # noqa: E402
except ImportError:  # pragma: no cover
    KitVisualizerCfg = None


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
        "viewport_name": f"FR3 Duo Robotiq {args_cli.physics_backend.upper()} Viewer",
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
    print(f"[INFO] Drive scaling: arms={scales['arm']} grippers={scales['gripper']}", flush=True)
    return scaled


def _first_gripper_drive(scaled_drives: dict[str, dict[str, float]]) -> dict[str, float]:
    for joint_name, drive in scaled_drives.items():
        lower_name = joint_name.lower()
        if "robotiq" in lower_name or "finger" in lower_name or "knuckle" in lower_name:
            return drive
    return {"stiffness": 1000.0, "damping": 100.0, "max_force": 200.0}


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
        joint_names_expr=[GRIPPER_JOINT_EXPR],
        effort_limit_sim=gripper_drive["max_force"],
        stiffness=gripper_drive["stiffness"],
        damping=gripper_drive["damping"],
    )

    # The generated mobile USD contains non-arm joints such as the base drive, caster,
    # rocker, and spine joints. This standalone viewer does not command them, so hold
    # them at their default pose; otherwise Newton is free to solve them dynamically and
    # the robot can visibly collapse or rotate away from the imported pose.
    actuators["mobile_base_and_spine_hold"] = ImplicitActuatorCfg(
        joint_names_expr=[MOBILE_BASE_HOLD_JOINT_EXPR],
        effort_limit_sim=500.0,
        stiffness=1000.0,
        damping=100.0,
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


def _make_robot_cfg(usd_path: Path, franka_root: Path) -> ArticulationCfg:
    return ArticulationCfg(
        prim_path=args_cli.prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            activate_contact_sensors=False,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=not args_cli.enable_gravity,
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
    physics_cfg = None
    if args_cli.physics_backend == "newton":
        physics_cfg = NewtonCfg(
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
        )

    return SimulationCfg(
        dt=1.0 / 120.0,
        device=args_cli.device,
        physics=physics_cfg,
        visualizer_cfgs=_make_visualizer_cfgs(),
    )


def main() -> None:
    usd_path = _resolve_path(args_cli.usd_path)
    franka_root = _resolve_path(args_cli.franka_root)

    print(f"[INFO] Loading robot USD: {usd_path}", flush=True)
    print(f"[INFO] Franka root: {franka_root}", flush=True)
    print(f"[INFO] Physics backend: {args_cli.physics_backend}", flush=True)
    print("[INFO] Stage patching: disabled (USD is loaded as-authored)", flush=True)

    print("[INFO] Creating simulation context...", flush=True)
    with build_simulation_context(
        device=args_cli.device,
        auto_add_lighting=True,
        add_ground_plane=True,
        sim_cfg=_make_sim_cfg(),
    ) as sim:
        print("[INFO] Simulation context created.", flush=True)
        sim._app_control_on_stop_handle = None
        sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))
        sim_utils.create_prim("/World/Env_0", "Xform", translation=(0.0, 0.0, 0.0))

        print("[INFO] Creating Articulation from USD...", flush=True)
        articulation_cls = NewtonArticulation if args_cli.physics_backend == "newton" else PhysxArticulation
        robot = articulation_cls(_make_robot_cfg(usd_path, franka_root))
        print("[INFO] Calling sim.reset()...", flush=True)
        sim.reset()
        print("[INFO] sim.reset() finished.", flush=True)
        sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))

        print(f"[INFO] Robot initialized: {robot.is_initialized}", flush=True)
        print("[INFO] Joint names:", ", ".join(robot.data.joint_names), flush=True)

        joint_pos_target = robot.data.default_joint_pos.torch.clone()
        robot.write_joint_position_to_sim_index(position=joint_pos_target)
        robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch.clone())
        robot.set_joint_position_target_index(target=joint_pos_target)
        robot.write_data_to_sim()

        if args_cli.no_physics_step:
            print("[INFO] --no-physics-step enabled: holding the app open without advancing Newton.", flush=True)
            while simulation_app.is_running():
                simulation_app.update()
        else:
            while simulation_app.is_running():
                robot.set_joint_position_target_index(target=joint_pos_target)
                robot.write_data_to_sim()
                sim.step()
                robot.update(sim.get_physics_dt())


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("[ERROR] visualize_fr3duo_robotiq_usd_newton_clean.py failed:", file=sys.stderr, flush=True)
        traceback.print_exc()
        raise
    finally:
        simulation_app.close()
