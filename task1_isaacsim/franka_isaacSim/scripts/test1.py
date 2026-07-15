"""
文件名: test_my_robot.py
描述: 专为 GUI Tools -> Physics Inspector 关节调试优化的独立测试脚本
"""

import argparse
import sys

# 1. 第一步：导入 AppLauncher
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="通过 Physics Inspector 调机与测试关节限位。")
parser.add_argument("--robot-prim-path", default="{ENV_REGEX_NS}/Robot", help="机器人在仿真 Stage 中的路径")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# 启动仿真 App（不需要强制 ROS2 桥接插件了，保持干净的 GUI 启动）
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# 2. 导入核心物理资产和场景模块
import omni.usd
from pxr import UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg

# =========================================================================
# 3. 补丁逻辑：引入自动拓扑修复函数，现场修剪多根冲突
# =========================================================================
def _iter_prims_under(root_prim):
    yield root_prim
    for child in root_prim.GetChildren():
        yield from _iter_prims_under(child)

def _fix_single_articulation_root(robot_prim_path: str) -> None:
    """自动剪枝，强制确保只有 base_link 带有 ArticulationRoot 属性"""
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
# 4. 机器人属性配置
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
    # 保持 None 状态，完全继承 USD 原生的 stiffness/damping/limits 方便调试
    actuators={
        "base_steering": ImplicitActuatorCfg(joint_names_expr=["tmrv0_2_joint_0", "tmrv0_2_joint_2"], stiffness=None, damping=None),
        "base_drive": ImplicitActuatorCfg(joint_names_expr=["tmrv0_2_joint_1", "tmrv0_2_joint_3"], stiffness=None, damping=None),
        "passive_base": ImplicitActuatorCfg(joint_names_expr=[".*caster.*", "rocker_arm_joint"], stiffness=None, damping=None),
        "spine": ImplicitActuatorCfg(joint_names_expr=["franka_spine_vertical_joint"], stiffness=None, damping=None),
        "arms": ImplicitActuatorCfg(joint_names_expr=[".*fr3v2_joint[1-7]"], stiffness=None, damping=None),
        "grippers": ImplicitActuatorCfg(joint_names_expr=["left_right_finger_joint", "right_right_finger_joint"], stiffness=None, damping=None),
    },
)

# =========================================================================
# 5. 空白环境配置
# =========================================================================
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
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    print("\n" + "="*60)
    print("[提示]: 调试就绪！请在上方菜单栏选择: Tools -> Physics -> Physics Inspector")
    print("[提示]: 点击弹窗中的机器人骨架，现在可以随意拉动滑块测试 36 个关节了！")
    print("="*60 + "\n")
    
    while simulation_app.is_running():
        # 🟢 核心修改一：【注释掉】scene.write_data_to_sim()
        # 切断 Python 内部 Tensor 数据覆盖，把关节的控制驱动权完全移交给 GUI 界面
        # scene.write_data_to_sim()
        
        sim.step()       # 推进仿真时钟，确保鼠标拉动时产生动态物理变化
        scene.update(sim_dt)

def main():
    # 🟢 核心修改二：强行关闭 GPU DIRECT_GPU_API 管道，兼容 CPU 类型的 UI 调试命令
    sim_cfg = sim_utils.SimulationCfg(device="cpu")
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([3.5, 3.5, 2.5], [0.0, 0.0, 0.6])
    
    scene_cfg = EmptyRobotSceneCfg(num_envs=1, env_spacing=5.0)
    scene = InteractiveScene(scene_cfg)
    
    # 动态剪枝修复 USD
    actual_robot_prim_path = args_cli.robot_prim_path.replace("{ENV_REGEX_NS}", "/World/envs/env_0")
    _fix_single_articulation_root(actual_robot_prim_path)
    
    sim.reset()
    run_simulator(sim, scene)

if __name__ == "__main__":
    main()
    simulation_app.close()
