"""
文件名: test_my_robot.py
描述: 专为双臂升降移动机器人定制的 Python 原生 ROS 2 关节控制与状态收发脚本（路径精准指引版）
"""
import argparse
import os
import sys

# =========================================================================
# 🟢 终极防御：在导入任何框架前，精准捕获并置顶 Isaac Sim 内置的 Python 3.11 ROS2 路径
# =========================================================================
# 1. 强行剔除任何可能串门过来的不兼容 3.12 系统路径
# sys.path = [p for p in sys.path if not ("ros" in p and "3.12" in p)]

# 2. 全自动定位 Isaac Sim 内置的预集成 rclpy 路径
# 无论是直接在主目录下还是通过 Kit 缓存，以下两个标准内置路径总能精准命中：
possible_ros_paths = [
    os.path.expanduser("~/isaacsim/kit/exts/omni.isaac.ros2_bridge/pip_prebundled"),
    os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-4.5.0/kit/exts/omni.isaac.ros2_bridge/pip_prebundled"),
    os.path.expanduser("~/.local/share/ov/pkg/isaac-sim-4.2.0/kit/exts/omni.isaac.ros2_bridge/pip_prebundled")
]

for p_path in possible_ros_paths:
    if os.path.exists(p_path):
        sys.path.insert(0, p_path)
        print(f"[路径激活] 成功硬挂载内置原生 ROS2 路径: {p_path}")

# 1. 第一步：导入 AppLauncher
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Python 原生 ROS 2 关节状态收发与控制。")
parser.add_argument("--robot-prim-path", default="{ENV_REGEX_NS}/Robot", help="机器人在仿真 Stage 中的路径")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动仿真 App
app_launcher = AppLauncher(
    args_cli,
    custom_config={
        "extensions": {
            "omni.isaac.ros2_bridge": {}  # 激活底层的桥接插件环境
        }
    }
)
simulation_app = app_launcher.app

# 2. 导入核心物理资产、场景模块以及 ROS 2 相关的 Python 库
import omni.usd
from pxr import UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg

# 此时直接 import，有顶层的精准手工置顶，绝对能安全上车！
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# =========================================================================
# 3. 补丁逻辑：自动剪枝修复多物理根节点冲突
# =========================================================================
def _iter_prims_under(root_prim):
    yield root_prim
    for child in root_prim.GetChildren():
        yield from _iter_prims_under(child)

def _fix_single_articulation_root(robot_prim_path: str) -> None:
    stage = omni.usd.get_context().get_stage()
    if stage is None: return
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid(): return

    root_prims = [
        prim for prim in _iter_prims_under(robot_prim)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(root_prims) <= 1: return

    keep_prim = None
    for preferred_path in (f"{robot_prim_path}/base", f"{robot_prim_path}/base_link"):
        candidate = stage.GetPrimAtPath(preferred_path)
        if candidate.IsValid():
            keep_prim = candidate
            break
    if keep_prim is None: keep_prim = root_prims[0]

    for prim in root_prims:
        if prim != keep_prim:
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    print(f"[补丁成功] 物理根节点已收拢至: {keep_prim.GetPath()}")

# =========================================================================
# 4. 全权接管关节收发的 ROS 2 节点
# =========================================================================
class RobotJointRosBridge(Node):
    def __init__(self, target_joint_names: list[str]):
        super().__init__("isaaclab_joint_bridge")
        self.target_joint_names = target_joint_names

        # 接收缓存
        self.target_positions = {name: 0.0 for name in target_joint_names}

        # 接收端
        self.subscription = self.create_subscription(
            JointState,
            "/joint_command",
            self._listener_callback,
            10
        )

        # 发送端
        self.state_publisher = self.create_publisher(JointState, "/joint_states", 10)

        self.get_logger().info("成功创建原生 ROS 2 桥接节点！")
        self.get_logger().info("  -> 监听控制话题: /joint_command")
        self.get_logger().info("  -> 发布状态话题: /joint_states")

    def _listener_callback(self, msg: JointState):
        for idx, name in enumerate(msg.name):
            if name in self.target_positions:
                if idx < len(msg.position):
                    self.target_positions[name] = float(msg.position[idx])
                    self.get_logger().info(f"[接收指令] 关节: {name} -> 目标位置: {msg.position[idx]:.4f}")

    def publish_robot_states(self, actual_names: list[str], current_positions: list[float], current_velocities: list[float]):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = actual_names
        msg.position = current_positions
        msg.velocity = current_velocities
        msg.effort = [0.0] * len(actual_names)
        self.state_publisher.publish(msg)

# =========================================================================
# 5. 机器人属性配置
# =========================================================================
MOBILE_FR3_DUO_CONFIG = ArticulationCfg(
    prim_path=args_cli.robot_prim_path,
    spawn=sim_utils.UsdFileCfg(
        usd_path="/workspace/franka_isaacSim/assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.05),
        joint_pos={
            "franka_spine_vertical_joint": 0.2,
            "left_fr3v2_joint4": -2.3562,
            "right_fr3v2_joint4": -2.3562,
            "left_fr3v2_joint6": 1.5708,
            "right_fr3v2_joint6": 1.5708,
        }
    ),
    actuators={
        "base_steering": ImplicitActuatorCfg(joint_names_expr=["tmrv0_2_joint_0", "tmrv0_2_joint_2"], stiffness=None, damping=None),
        "base_drive": ImplicitActuatorCfg(joint_names_expr=["tmrv0_2_joint_1", "tmrv0_2_joint_3"], stiffness=None, damping=None),
        "passive_base": ImplicitActuatorCfg(joint_names_expr=[".*caster.*", "rocker_arm_joint"], stiffness=None, damping=None),
        "spine": ImplicitActuatorCfg(joint_names_expr=["franka_spine_vertical_joint"], stiffness=None, damping=None),
        "arms": ImplicitActuatorCfg(joint_names_expr=[".*fr3v2_joint[1-7]"], stiffness=None, damping=None),
        "grippers": ImplicitActuatorCfg(joint_names_expr=["left_right_finger_joint", "right_right_finger_joint"], stiffness=None, damping=None),
        # "grippers": ImplicitActuatorCfg(
        #     joint_names_expr=[
        #         "left_right_finger_joint",
        #         "right_right_finger_joint",
        #     ],
        #     stiffness=200.0,
        #     damping=20.0,
        #     effort_limit_sim=200.0,
        #     velocity_limit_sim=2.0,
        # ),
    },
)



class EmptyRobotSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.85, 0.9, 1.0)),
    )
    robot = MOBILE_FR3_DUO_CONFIG

# =========================================================================
# 6. 主仿真推进循环
# =========================================================================
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, ros_node: RobotJointRosBridge):
    sim_dt = sim.get_physics_dt()
    robot = scene["robot"]

    actual_joint_names = list(robot.data.joint_names) if hasattr(robot.data, "joint_names") else list(robot.joint_names)
    target_joint_names = ["left_right_finger_joint", "right_right_finger_joint"]

    print("\n" + "="*60)
    print("[提示]: ROS 2 Python 原生双向收发接口已就绪！")
    print("[提示]: 发送控制端: 通过外部 ros2 topic pub 指令往 /joint_command 发控制")
    print("[提示]: 状态接收端: 打开新终端执行 ros2 topic echo /joint_states 观察当前角度")
    print("="*60 + "\n")

    while simulation_app.is_running() and rclpy.ok():
        rclpy.spin_once(ros_node, timeout_sec=0.0)

        pos_list = robot.data.joint_pos[0].detach().cpu().tolist()
        vel_list = robot.data.joint_vel[0].detach().cpu().tolist()
        ros_node.publish_robot_states(actual_joint_names, pos_list, vel_list)

        current_targets = robot.data.default_joint_pos.clone()
        # current_targets = robot.data.joint_pos.clone()
        for name in target_joint_names:
            if name in actual_joint_names:
                idx = actual_joint_names.index(name)
                current_targets[:, idx] = ros_node.target_positions[name]

        robot.set_joint_position_target(current_targets)

        idx = robot.joint_names.index("right_right_finger_joint")

        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

def main():
    sim_cfg = sim_utils.SimulationCfg(device="cpu")
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.5, 3.5, 2.5], [0.0, 0.0, 0.6])

    scene_cfg = EmptyRobotSceneCfg(num_envs=1, env_spacing=5.0)
    scene = InteractiveScene(scene_cfg)

    actual_robot_prim_path = args_cli.robot_prim_path.replace("{ENV_REGEX_NS}", "/World/envs/env_0")
    _fix_single_articulation_root(actual_robot_prim_path)

    sim.reset()

    robot = scene["robot"]
    robot.write_data_to_sim()
    sim.step()
    scene.update(sim.get_physics_dt())

    rclpy.init()
    ros_node = RobotJointRosBridge(["left_right_finger_joint", "right_right_finger_joint"])

    try:
        run_simulator(sim, scene, ros_node)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    simulation_app.close()
