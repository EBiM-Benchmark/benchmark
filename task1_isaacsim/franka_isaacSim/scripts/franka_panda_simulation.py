# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# SPDX-License-Identifier: BSD-3-Clause

"""Launch IsaacLab and visualize one Franka Panda robot."""

import argparse

from isaaclab.app import AppLauncher


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--panda-visualizer",
        choices=["none", "kit"],
        default="kit",
        help="IsaacLab visualizer backend for this demo.",
    )
    parser.add_argument("--camera-position", type=float, nargs=3, default=(2.5, 2.0, 1.5))
    parser.add_argument("--camera-target", type=float, nargs=3, default=(0.0, 0.0, 0.5))
    AppLauncher.add_app_launcher_args(parser)
    return parser


args_cli = _build_arg_parser().parse_args()
simulation_app = AppLauncher(args_cli).app

import torch

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationCfg, build_simulation_context
from isaaclab_assets import FRANKA_PANDA_CFG

from isaaclab_newton.assets import Articulation
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

try:
    from isaaclab_visualizers.kit import KitVisualizerCfg
except ImportError:
    KitVisualizerCfg = None


def _make_visualizer_cfgs():
    if args_cli.panda_visualizer == "none" or KitVisualizerCfg is None:
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


def main():
    # 仿真配置：使用 Newton 后端
    visualizer_cfgs = _make_visualizer_cfgs()
    sim_cfg = SimulationCfg(
        dt=1 / 120,
        physics=NewtonCfg(
            solver_cfg=MJWarpSolverCfg(
                njmax=20,
                nconmax=20,
                ls_iterations=20,
                cone="pyramidal",
                impratio=1,
                ls_parallel=False,
                integrator="implicitfast",
            ),
            num_substeps=1,
            debug_mode=False,
        ),
    )
    if visualizer_cfgs and hasattr(sim_cfg, "visualizer_cfgs"):
        sim_cfg.visualizer_cfgs = visualizer_cfgs

    # 创建仿真上下文
    with build_simulation_context(
        device=args_cli.device,
        auto_add_lighting=True,
        add_ground_plane=True,
        sim_cfg=sim_cfg,
    ) as sim:
        # 防止某些测试环境里的 stop handle 干扰
        sim._app_control_on_stop_handle = None
        sim.set_camera_view(tuple(args_cli.camera_position), tuple(args_cli.camera_target))

        # 创建一个环境 prim
        sim_utils.create_prim(
            "/World/Env_0",
            "Xform",
            translation=(0.0, 0.0, 0.0),
        )

        # 只加载一个 Panda 机器人
        panda_cfg = FRANKA_PANDA_CFG.copy().replace(prim_path="/World/Env_0/Robot")
        robot = Articulation(panda_cfg)

        # 启动仿真
        sim.reset()
        print("[INFO] Simulation started.")
        print("[INFO] Panda initialized:", robot.is_initialized)

        # 把 Panda 设置到默认关节位置
        robot.write_joint_position_to_sim_index(position=robot.data.default_joint_pos.torch.clone())
        robot.write_joint_velocity_to_sim_index(velocity=robot.data.default_joint_vel.torch.clone())

        # 主循环：让 Isaac Sim 窗口或 visualizer 持续运行
        while simulation_app.is_running():
            joint_pos_target = robot.data.default_joint_pos.torch.clone()
            joint_pos_target[:, 3] = 0.5

            robot.set_joint_position_target_index(target=joint_pos_target)
            robot.write_data_to_sim()

            sim.step()
            robot.update(sim.cfg.dt)


if __name__ == "__main__":
    main()
    simulation_app.close()
