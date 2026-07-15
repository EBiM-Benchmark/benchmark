#!/usr/bin/env python3
"""Load the FR3 Duo mobile USDZ and control the left arm from the keyboard.

Run this with IsaacLab's Python, for example:

    cd /workspace/isaaclab
    ./isaaclab.sh -p /workspace/franka_isaacSim/scripts/load_usdz_keyboard_left_arm.py

Keyboard controls:
    Q/A joint1, W/S joint2, E/D joint3, R/F joint4,
    T/G joint5, Y/H joint6, U/J joint7, 0 home, P print target.
"""

from __future__ import annotations

import argparse
import atexit
import re
import select
import sys
import termios
import tty
from pathlib import Path

from isaaclab.app import AppLauncher


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usdz-path",
        default=(
            "/homeL/qiguan/dataSSD/franka_isaacSim/assets/isaaclab_fixed/"
            "mobile_fr3_duo_v0_2_franka_hand/mobile_fr3_duo_v0_2_franka_hand.usd"
        ),
        help="USD/USDZ file to load.",
    )
    parser.add_argument("--prim-path", default="/World/Robot", help="Prim path for the loaded articulation.")
    parser.add_argument("--physics-hz", type=float, default=240.0)
    parser.add_argument("--render-hz", type=float, default=60.0)
    parser.add_argument("--physics-substeps", type=int, default=2)
    parser.add_argument("--mj-njmax", type=int, default=2048)
    parser.add_argument("--mj-nconmax", type=int, default=512)
    parser.add_argument("--mj-cone", default="pyramidal")
    parser.add_argument("--mj-integrator", default="implicitfast")
    parser.add_argument("--mj-impratio", type=float, default=1.0)
    parser.add_argument("--step-rad", type=float, default=0.05, help="Joint target increment per key press.")
    parser.add_argument(
        "--force-home",
        action="store_true",
        help="Overwrite the loaded USD initial joint pose with the hard-coded FR3 Duo home pose.",
    )
    parser.add_argument("--effort-limit", type=float, default=400.0)
    parser.add_argument("--velocity-limit", type=float, default=8.0)
    parser.add_argument("--stiffness", type=float, default=800.0)
    parser.add_argument("--damping", type=float, default=80.0)
    parser.add_argument("--camera-position", type=float, nargs=3, default=(3.2, 3.0, 2.0))
    parser.add_argument("--camera-target", type=float, nargs=3, default=(0.0, 0.0, 0.8))
    AppLauncher.add_app_launcher_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb.input  # noqa: E402
import omni  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.utils.configclass import configclass  # noqa: E402
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg  # noqa: E402


LEFT_JOINTS = [
    "left_fr3v2_joint1",
    "left_fr3v2_joint2",
    "left_fr3v2_joint3",
    "left_fr3v2_joint4",
    "left_fr3v2_joint5",
    "left_fr3v2_joint6",
    "left_fr3v2_joint7",
]

HOME_JOINT_POS = {
    "left_fr3v2_joint1": 0.0,
    "left_fr3v2_joint2": -0.7854,
    "left_fr3v2_joint3": 0.0,
    "left_fr3v2_joint4": -2.3562,
    "left_fr3v2_joint5": 0.0,
    "left_fr3v2_joint6": 1.5708,
    "left_fr3v2_joint7": 0.7854,
    "left_fr3v2_finger_joint.*": 0.04,
    "right_fr3v2_joint1": 0.0,
    "right_fr3v2_joint2": -0.7854,
    "right_fr3v2_joint3": 0.0,
    "right_fr3v2_joint4": -2.3562,
    "right_fr3v2_joint5": 0.0,
    "right_fr3v2_joint6": 1.5708,
    "right_fr3v2_joint7": 0.7854,
    "right_fr3v2_finger_joint.*": 0.04,
}


def _resolve_asset_path(path_text: str) -> Path:
    """Resolve common host/container mount spellings for this repository."""
    path = Path(path_text).expanduser()
    if path.exists():
        return path

    mappings = [
        (
            Path("/homeL/qiguan/dataSSD/franka_isaacSim"),
            Path("/workspace/franka_isaacSim"),
        ),
        (
            Path("/dataSSD/qiguan/franka_isaacSim"),
            Path("/workspace/franka_isaacSim"),
        ),
    ]
    for src, dst in mappings:
        try:
            rel = path.relative_to(src)
        except ValueError:
            continue
        candidate = dst / rel
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        f"USD asset does not exist: {path_text}\n"
        "If you run inside a container, mount the repository and pass the container path, "
        "for example /workspace/franka_isaacSim/assets/..."
    )


def _joint_names(robot) -> list[str]:
    names = getattr(robot.data, "joint_names", None)
    if names is not None:
        return list(names)
    names = getattr(robot, "joint_names", None)
    if names is not None:
        return list(names)
    raise RuntimeError("Could not read articulation joint names.")


def _as_torch_tensor(value):
    if hasattr(value, "torch"):
        value = value.torch
    return value


def _home_target_tensor(robot, joint_names: list[str]):
    joint_pos = _as_torch_tensor(robot.data.joint_pos)
    if joint_pos.ndim == 1:
        joint_pos = joint_pos.unsqueeze(0)
    target = joint_pos.clone()

    for pattern, value in HOME_JOINT_POS.items():
        if pattern.endswith(".*"):
            expr = re.compile(pattern)
            matched = [index for index, name in enumerate(joint_names) if expr.fullmatch(name)]
        else:
            matched = [joint_names.index(pattern)] if pattern in joint_names else []
        for index in matched:
            target[:, index] = float(value)
    return target


def _force_home_state(robot, joint_names: list[str]):
    target = _home_target_tensor(robot, joint_names)
    zero_velocity = torch.zeros_like(target)

    if hasattr(robot, "write_joint_position_to_sim_index"):
        robot.write_joint_position_to_sim_index(position=target)
        robot.write_joint_velocity_to_sim_index(velocity=zero_velocity)
    else:
        robot.write_joint_state_to_sim(target, zero_velocity)

    if hasattr(robot, "set_joint_position_target_index"):
        robot.set_joint_position_target_index(target=target)
    else:
        robot.set_joint_position_target(target)
    return target


def _apply_joint_delta(target, joint_indices, joint_name: str, delta: float):
    joint_index = joint_indices[joint_name]
    target[:, joint_index] += delta
    value = float(target[0, joint_index].item())
    print(f"{joint_name}: target={value:.4f} rad", flush=True)


def _reset_left_arm_home(target, joint_indices):
    for joint_name, joint_index in joint_indices.items():
        target[:, joint_index] = HOME_JOINT_POS[joint_name]
    print("Left arm target reset to home.", flush=True)


def _print_left_arm_target(target, joint_indices):
    values = {
        name: round(float(target[0, index].item()), 4)
        for name, index in joint_indices.items()
    }
    print(f"Left arm target: {values}", flush=True)


class KeyboardJointController:
    def __init__(self, robot, joint_indices: dict[str, int], step_rad: float, initial_target=None):
        self.robot = robot
        self.joint_indices = joint_indices
        self.step_rad = float(step_rad)
        if initial_target is None:
            joint_pos = _as_torch_tensor(robot.data.joint_pos)
            if joint_pos.ndim == 1:
                joint_pos = joint_pos.unsqueeze(0)
            self.target = joint_pos.clone()
        else:
            self.target = initial_target.clone()

        self._input = carb.input.acquire_input_interface()
        self._app_window = omni.appwindow.get_default_app_window()
        if self._app_window is None:
            raise RuntimeError("No Omniverse app window found. Run without --headless.")
        self._keyboard = self._app_window.get_keyboard()
        self._subscription = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

        self._bindings = {}
        for key_name, joint_name, sign in [
            ("Q", LEFT_JOINTS[0], +1.0),
            ("A", LEFT_JOINTS[0], -1.0),
            ("W", LEFT_JOINTS[1], +1.0),
            ("S", LEFT_JOINTS[1], -1.0),
            ("E", LEFT_JOINTS[2], +1.0),
            ("D", LEFT_JOINTS[2], -1.0),
            ("R", LEFT_JOINTS[3], +1.0),
            ("F", LEFT_JOINTS[3], -1.0),
            ("T", LEFT_JOINTS[4], +1.0),
            ("G", LEFT_JOINTS[4], -1.0),
            ("Y", LEFT_JOINTS[5], +1.0),
            ("H", LEFT_JOINTS[5], -1.0),
            ("U", LEFT_JOINTS[6], +1.0),
            ("J", LEFT_JOINTS[6], -1.0),
        ]:
            self._bindings[key_name] = (joint_name, sign)

        self._home_keys = {"KEY_0", "NUM_0", "N0", "0"}
        self._print_key = "P"

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type not in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_REPEAT,
        ):
            return True

        key_name = getattr(event.input, "name", str(event.input))

        if key_name in self._bindings:
            joint_name, sign = self._bindings[key_name]
            _apply_joint_delta(self.target, self.joint_indices, joint_name, sign * self.step_rad)
            return True

        if key_name in self._home_keys:
            _reset_left_arm_home(self.target, self.joint_indices)
            return True

        if key_name == self._print_key:
            _print_left_arm_target(self.target, self.joint_indices)
            return True

        return True

    def apply(self):
        self.robot.set_joint_position_target(self.target)

    def print_target(self):
        values = {
            name: round(float(self.target[0, index].item()), 4)
            for name, index in self.joint_indices.items()
        }
        print(f"Left arm target: {values}", flush=True)


class TerminalJointController:
    """Fallback keyboard controller that reads key presses from the terminal."""

    def __init__(self, robot, joint_indices: dict[str, int], step_rad: float, initial_target=None):
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY, so terminal keyboard fallback is unavailable.")

        self.robot = robot
        self.joint_indices = joint_indices
        self.step_rad = float(step_rad)
        if initial_target is None:
            joint_pos = _as_torch_tensor(robot.data.joint_pos)
            if joint_pos.ndim == 1:
                joint_pos = joint_pos.unsqueeze(0)
            self.target = joint_pos.clone()
        else:
            self.target = initial_target.clone()

        self._bindings = {}
        for key_name, joint_name, sign in [
            ("q", LEFT_JOINTS[0], +1.0),
            ("a", LEFT_JOINTS[0], -1.0),
            ("w", LEFT_JOINTS[1], +1.0),
            ("s", LEFT_JOINTS[1], -1.0),
            ("e", LEFT_JOINTS[2], +1.0),
            ("d", LEFT_JOINTS[2], -1.0),
            ("r", LEFT_JOINTS[3], +1.0),
            ("f", LEFT_JOINTS[3], -1.0),
            ("t", LEFT_JOINTS[4], +1.0),
            ("g", LEFT_JOINTS[4], -1.0),
            ("y", LEFT_JOINTS[5], +1.0),
            ("h", LEFT_JOINTS[5], -1.0),
            ("u", LEFT_JOINTS[6], +1.0),
            ("j", LEFT_JOINTS[6], -1.0),
        ]:
            self._bindings[key_name] = (joint_name, sign)

        self._stdin_fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        atexit.register(self.close)

    def close(self):
        if self._old_termios is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_termios)
            self._old_termios = None

    def poll(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key_name = sys.stdin.read(1).lower()
            if key_name in self._bindings:
                joint_name, sign = self._bindings[key_name]
                _apply_joint_delta(self.target, self.joint_indices, joint_name, sign * self.step_rad)
            elif key_name == "0":
                _reset_left_arm_home(self.target, self.joint_indices)
            elif key_name == "p":
                _print_left_arm_target(self.target, self.joint_indices)

    def apply(self):
        self.poll()
        self.robot.set_joint_position_target(self.target)


def _make_controller(robot, joint_indices: dict[str, int], step_rad: float, initial_target=None):
    try:
        controller = KeyboardJointController(robot, joint_indices, step_rad, initial_target=initial_target)
        print("Omniverse keyboard ready. Focus the viewport and press Q/A W/S E/D R/F T/G Y/H U/J.", flush=True)
        return controller
    except Exception as exc:
        print(f"Warning: Omniverse keyboard input unavailable: {exc}", flush=True)
        print("Falling back to terminal keyboard. Focus this terminal and press the same keys.", flush=True)
        return TerminalJointController(robot, joint_indices, step_rad, initial_target=initial_target)


def main():
    usd_path = _resolve_asset_path(args_cli.usdz_path)
    print(f"Loading USD asset: {usd_path}")

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
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))

    robot_cfg = ArticulationCfg(
        prim_path=args_cli.prim_path,
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            variants={"Physics": "mujoco"},
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
        actuators={
            "left_fr3v2_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["left_fr3v2_joint[1-4]"],
                effort_limit_sim=87.0,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=80.0,
                damping=4.0,
                armature=1e-3,
            ),
            "left_fr3v2_forearm": ImplicitActuatorCfg(
                joint_names_expr=["left_fr3v2_joint[5-7]"],
                effort_limit_sim=12.0,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=80.0,
                damping=4.0,
                armature=1e-3,
            ),
            "left_fr3v2_hand": ImplicitActuatorCfg(
                joint_names_expr=["left_fr3v2_finger_joint.*"],
                effort_limit_sim=200.0,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=2e3,
                damping=1e2,
            ),
            "right_fr3v2_shoulder": ImplicitActuatorCfg(
                joint_names_expr=["right_fr3v2_joint[1-4]"],
                effort_limit_sim=87.0,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=80.0,
                damping=4.0,
                armature=1e-3,
            ),
            "right_fr3v2_forearm": ImplicitActuatorCfg(
                joint_names_expr=["right_fr3v2_joint[5-7]"],
                effort_limit_sim=12.0,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=80.0,
                damping=4.0,
                armature=1e-3,
            ),
            "right_fr3v2_hand": ImplicitActuatorCfg(
                joint_names_expr=["right_fr3v2_finger_joint.*"],
                effort_limit_sim=200.0,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=2e3,
                damping=1e2,
            ),
            "mobile_base_and_lift": ImplicitActuatorCfg(
                joint_names_expr=[
                    "tmrv0_2_joint_.*",
                    "caster_.*",
                    "franka_spine_vertical_joint",
                    "rocker_arm_joint",
                ],
                effort_limit_sim=args_cli.effort_limit,
                velocity_limit_sim=args_cli.velocity_limit,
                stiffness=args_cli.stiffness,
                damping=args_cli.damping,
            ),
        },
        soft_joint_pos_limit_factor=1.0,
    )

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
        dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=600.0, color=(0.85, 0.9, 1.0)),
        )
        robot = robot_cfg

    scene = InteractiveScene(SceneCfg(num_envs=1, env_spacing=0.0))
    sim.reset()
    scene.reset()

    robot = scene["robot"]
    actual_joint_names = _joint_names(robot)
    if args_cli.force_home:
        controller_target = _force_home_state(robot, actual_joint_names)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
        print("Forced hard-coded FR3 Duo home pose after simulation reset.", flush=True)
    else:
        joint_pos = _as_torch_tensor(robot.data.joint_pos)
        if joint_pos.ndim == 1:
            joint_pos = joint_pos.unsqueeze(0)
        controller_target = joint_pos.clone()
        if hasattr(robot, "set_joint_position_target_index"):
            robot.set_joint_position_target_index(target=controller_target)
        else:
            robot.set_joint_position_target(controller_target)
        print("Using the USD/Newton default joint pose as the initial keyboard target.", flush=True)

    joint_indices = {}
    missing = []
    for joint_name in LEFT_JOINTS:
        if joint_name in actual_joint_names:
            joint_indices[joint_name] = actual_joint_names.index(joint_name)
        else:
            missing.append(joint_name)
    if missing:
        raise RuntimeError(
            "Could not find left-arm joints in loaded asset: "
            + ", ".join(missing)
            + "\nActual joints: "
            + ", ".join(actual_joint_names)
        )

    controller = _make_controller(robot, joint_indices, args_cli.step_rad, initial_target=controller_target)

    try:
        while simulation_app.is_running():
            controller.apply()
            scene.write_data_to_sim()
            sim.step()
            scene.update(sim.get_physics_dt())
    finally:
        if hasattr(controller, "close"):
            controller.close()
        simulation_app.close()


if __name__ == "__main__":
    main()
