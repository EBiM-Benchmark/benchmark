#!/usr/bin/env python3

import argparse
import math
import time

from isaaclab.app import AppLauncher


from pxr import UsdPhysics

import omni.usd
import isaaclab.sim as sim_utils


def _iter_prims_under(root_prim):
    yield root_prim
    for child in root_prim.GetChildren():
        yield from _iter_prims_under(child)


def _fix_single_articulation_root(robot_prim_path: str):
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)

    root_prims = [
        prim
        for prim in _iter_prims_under(robot_prim)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]

    print("Articulation roots:", [str(p.GetPath()) for p in root_prims])

    if len(root_prims) <= 1:
        return

    keep_path = f"{robot_prim_path}/base"
    keep_prim = stage.GetPrimAtPath(keep_path)
    if not keep_prim.IsValid():
        keep_prim = root_prims[0]

    for prim in root_prims:
        if prim != keep_prim:
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            print(f"Removed extra articulation root: {prim.GetPath()}")

    print(f"Keeping articulation root: {keep_prim.GetPath()}")


parser = argparse.ArgumentParser()
parser.add_argument(
    "--usd-path",
    default="/workspace/franka_isaacSim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd",
)
parser.add_argument("--robot-prim-path", default="/World/Robotiq")
parser.add_argument("--command-topic", default="/isaac/left_robotiq_joint_commands")
parser.add_argument("--stiffness", type=float, default=200.0)
parser.add_argument("--damping", type=float, default=20.0)
parser.add_argument("--default-open-pos", type=float, default=0.04)

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg


class RobotiqCommandNode(Node):
    def __init__(self, topic: str):
        super().__init__("robotiq_only_test_node")
        self.latest_command = None
        self.create_subscription(JointState, topic, self._on_command, 10)
        self.get_logger().info(f"Listening on {topic}")

    def _on_command(self, msg: JointState):
        if not msg.position:
            return
        self.latest_command = {
            name: float(pos)
            for name, pos in zip(msg.name, msg.position)
            if math.isfinite(float(pos))
        }


def main():
    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 240.0,
        render_interval=4,
        device=args.device,
    )
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([1.0, 1.0, 0.8], [0.0, 0.0, 0.1])

    gripper_cfg = ArticulationCfg(
        prim_path=args.robot_prim_path,
        spawn=sim_utils.UsdFileCfg(usd_path=args.usd_path),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.2),
            joint_pos={
                ".*finger.*": args.default_open_pos,
                ".*knuckle.*": args.default_open_pos,
            },
        ),
        actuators={
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=[".*finger.*", ".*knuckle.*"],
                effort_limit_sim=200.0,
                velocity_limit_sim=2.0,
                stiffness=args.stiffness,
                damping=args.damping,
            ),
        },
    )

    gripper = Articulation(gripper_cfg)
    _fix_single_articulation_root(args.robot_prim_path)

    sim.reset()
    gripper.reset()

    print("Robotiq-only ROS test started.")
    print("Joint names:")
    for name in gripper.joint_names:
        print("  ", name)

    rclpy.init()
    node = RobotiqCommandNode(args.command_topic)

    actual_names = list(gripper.joint_names)

    def resolve_command_to_targets(command):
        target = gripper.data.joint_pos.clone()

        # Case 1: command names directly match USD joint names
        matched = False
        for cmd_name, cmd_pos in command.items():
            if cmd_name in actual_names:
                idx = actual_names.index(cmd_name)
                target[:, idx] = cmd_pos
                matched = True

        if matched:
            return target

        # Case 2: old bridge command has one driver joint, map it to all Robotiq finger/knuckle joints
        value = next(iter(command.values()))
        for idx, joint_name in enumerate(actual_names):
            lower = joint_name.lower()
            if "finger" in lower or "knuckle" in lower:
                target[:, idx] = value

        return target

    try:
        while simulation_app.is_running() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            if node.latest_command is not None:
                target = resolve_command_to_targets(node.latest_command)
                gripper.set_joint_position_target(target)

            gripper.write_data_to_sim()
            sim.step()
            gripper.update(sim.get_physics_dt())

    finally:
        node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
