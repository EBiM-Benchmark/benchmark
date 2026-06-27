#!/usr/bin/env python3
"""
键盘控制移动底盘和机械臂的演示
"""

import argparse
import math
import sys
from pathlib import Path

print("=" * 80)
print("移动机器人键盘控制演示")
print("=" * 80)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 导入必要的模块
from pynput import keyboard  # 需要 pip install pynput

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.terrains import TerrainImporterCfg

COMMON_DIR = Path(__file__).resolve().parents[1] / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import franka_urdf_path
from tmr_base_control import (
    compensate_yaw_rate,
    compute_drive_targets,
    find_drive_joint_ids,
    get_keyboard_twist,
    get_root_yaw,
)

usd_path = str(franka_urdf_path("mobile_fr3_duo_v0_2_franka_hand.usd"))

print("\n创建场景配置...")

# 场景配置
scene_cfg = InteractiveSceneCfg(num_envs=args.num_envs, env_spacing=2.5)

# 地面 - 大幅增加摩擦力
scene_cfg.terrain = TerrainImporterCfg(
    prim_path="/World/ground",
    terrain_type="plane",
    collision_group=-1,
    physics_material=sim_utils.RigidBodyMaterialCfg(
        static_friction=2.0,  # 大幅增加静摩擦
        dynamic_friction=1.5,  # 大幅增加动摩擦
        restitution=0.0,  # 无弹性
    ),
    debug_vis=False,
)

# 光照
scene_cfg.dome_light = AssetBaseCfg(
    prim_path="/World/Light",
    spawn=sim_utils.DomeLightCfg(intensity=3000.0),
)

# 设置初始关节位置 - 机械臂完全向后收起，防止前倾
initial_joint_pos = {
    # 左臂 - 垂直向上收起（更稳定的姿态）
    # joint1限制: [-2.9007, 2.9007] 弧度 (约±166度)
    "left_fr3v2_joint1": 0.0,  # 0度，保持向前
    "left_fr3v2_joint2": -1.5,  # 向上抬高
    "left_fr3v2_joint3": 0.0,  # 保持居中
    "left_fr3v2_joint4": -2.2,  # 弯曲
    "left_fr3v2_joint5": 0.0,  # 保持居中
    "left_fr3v2_joint6": 1.5,  # 手腕向上
    "left_fr3v2_joint7": 0.785,  # 45度
    # 右臂 - 垂直向上收起（更稳定的姿态）
    "right_fr3v2_joint1": 0.0,  # 0度，保持向前
    "right_fr3v2_joint2": -1.5,
    "right_fr3v2_joint3": 0.0,
    "right_fr3v2_joint4": -2.2,
    "right_fr3v2_joint5": 0.0,
    "right_fr3v2_joint6": 1.5,
    "right_fr3v2_joint7": 0.785,
}

# 机器人配置 - 分别为底盘和机械臂设置不同的执行器参数
scene_cfg.robot = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),  # 直接放置在地面上
        joint_pos=initial_joint_pos,
    ),
    actuators={
        # 转向关节 - 使用位置控制，高刚度保持转向角
        "steering_joints": ImplicitActuatorCfg(
            joint_names_expr=["tmrv0_2_joint_0", "tmrv0_2_joint_2"],
            stiffness=500.0,  # 高刚度用于位置控制
            damping=50.0,
            effort_limit=200.0,
        ),
        # 主动驱动轮 - 使用速度控制
        "drive_joints": ImplicitActuatorCfg(
            joint_names_expr=["tmrv0_2_joint_1", "tmrv0_2_joint_3"],
            stiffness=0.0,  # 速度控制
            damping=5.0,
            effort_limit_sim=500.0,
            velocity_limit_sim=20.0,
        ),
        # 被动万向轮和后摇臂 - 不给速度/位置伺服，避免锁住侧移和原地旋转
        "passive_base_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*caster.*", "rocker_arm_joint"],
            stiffness=0.0,
            damping=0.0,
        ),
        # 机械臂关节 - 使用位置控制，超高刚度和阻尼防止晃动
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*fr3v2_joint[1-7]"],
            stiffness=5000.0,  # 大幅增加刚度，锁住机械臂
            damping=500.0,  # 大幅增加阻尼，防止震荡
            effort_limit=200.0,
        ),
        # 夹爪 - 位置控制
        "grippers": ImplicitActuatorCfg(
            joint_names_expr=[".*finger.*"],
            stiffness=200.0,
            damping=20.0,
            effort_limit=50.0,
        ),
    },
)

print("✓ 场景配置完成")

# 创建仿真上下文
sim_cfg = sim_utils.SimulationCfg(
    dt=0.005,  # 减小时间步长，提高稳定性
    device="cuda:0",
    gravity=(0.0, 0.0, -9.81),
)
sim = SimulationContext(sim_cfg)
sim.set_camera_view([3.5, 3.5, 2.5], [0.0, 0.0, 0.5])

print("✓ 仿真上下文创建完成")

# 创建场景
scene = InteractiveScene(scene_cfg)
print("✓ 场景创建完成")

# 初始化物理
sim.reset()
scene.reset()
print("✓ 物理初始化完成")

# 让机器人稳定站立 (进行更多步进以确保稳定)
print("\n等待机器人稳定...")
robot = scene["robot"]
for i in range(500):  # 增加到500步
    # 保持所有关节在初始位置
    joint_pos_targets = robot.data.default_joint_pos.clone()
    robot.set_joint_position_target(joint_pos_targets)
    scene.write_data_to_sim()
    sim.step()
    scene.update(sim.cfg.dt)
    if i % 100 == 0:
        print(f"  稳定中... {i}/500")
print("✓ 机器人已稳定")

# 获取机器人
print("\n机器人信息:")
print(f"  - 关节数: {robot.num_joints}")
print(f"  - 刚体数: {robot.num_bodies}")
print("\n关节名称:")
for i, name in enumerate(robot.joint_names):
    print(f"  {i:2d}: {name}")

# 识别底盘关节索引
base_joint_indices = []
for i, name in enumerate(robot.joint_names):
    if any(
        keyword in name
        for keyword in ["tmrv0_2_joint", "caster", "rocker_arm"]
    ):
        base_joint_indices.append(i)

print(f"\n底盘关节索引: {base_joint_indices}")
steering_indices, drive_indices = find_drive_joint_ids(robot.joint_names)
print(f"主动转向关节索引: {steering_indices}")
print(f"主动驱动轮索引: {drive_indices}")
heading_hold_yaw = get_root_yaw(robot)

# 键盘监听 - 使用 pynput 避免UI拦截
_pressed = set()
_listener = None


def _on_press(key):
    """按键按下回调"""
    try:
        _pressed.add(key.char.lower())
    except:
        _pressed.add(key.name)


def _on_release(key):
    """按键释放回调"""
    try:
        _pressed.discard(key.char.lower())
    except:
        _pressed.discard(key.name)
    if key == keyboard.Key.esc:
        return False


def start_listener():
    """启动键盘监听器"""
    global _listener
    # suppress=True 避免 Omniverse UI 抢键盘
    _listener = keyboard.Listener(
        on_press=_on_press, on_release=_on_release, suppress=True
    )
    _listener.daemon = True
    _listener.start()


def get_velocity_command():
    """根据当前按键状态获取速度命令"""
    return get_keyboard_twist(_pressed)


# 启动键盘监听器
start_listener()

print("\n" + "=" * 80)
print("✓ 开始仿真!")
print("=" * 80)
print("\n控制说明:")
print("  W - 向前移动")
print("  S - 向后移动")
print("  A - 向左平移")
print("  D - 向右平移")
print("  Q / ← - 原地左转")
print("  E / → - 原地右转")
print("  可组合按键，例如 W+A、W+Q")
print("  ESC - 退出")
print("  Ctrl+C - 退出")
print("\n提示: 直接按WASD键即可控制，无需点击窗口!")
print("      这是双转向驱动底盘，A/D 是侧移，Q/E 是旋转。\n")

count = 0
try:
    while simulation_app.is_running():
        # 获取键盘命令
        vx, vy, wz_cmd = get_velocity_command()
        wz, heading_hold_yaw = compensate_yaw_rate(
            robot,
            vx,
            vy,
            wz_cmd,
            heading_hold_yaw,
            manual_rotation=abs(wz_cmd) > 1.0e-4,
        )

        # 控制策略：
        # 1. 机械臂和夹爪 - 用位置控制保持固定姿态
        # 2. 底盘 - 这是一个双轮独立转向驱动底盘（类似购物车）
        #    - tmrv0_2_joint_0/2: 转向关节（Z轴旋转）- 位置控制
        #    - tmrv0_2_joint_1/3: 驱动关节（Y轴旋转，轮子滚动）- 速度控制

        # 为机械臂和夹爪设置位置目标（保持稳定）
        arm_gripper_pos_targets = robot.data.default_joint_pos.clone()
        robot.set_joint_position_target(arm_gripper_pos_targets)

        # 根据底盘速度命令计算两个主动转向驱动模块的角度和轮速
        steering_pos_targets, drive_vel_targets = compute_drive_targets(
            robot,
            steering_indices,
            vx,
            vy,
            wz,
            num_envs=args.num_envs,
            device=sim.device,
        )
        robot.set_joint_position_target(
            steering_pos_targets, joint_ids=steering_indices
        )
        robot.set_joint_velocity_target(
            drive_vel_targets, joint_ids=drive_indices
        )

        # 写入仿真
        scene.write_data_to_sim()

        # 步进仿真
        sim.step()
        scene.update(sim.cfg.dt)

        count += 1
        if count % 50 == 0:
            if vx != 0 or vy != 0 or wz != 0:
                print(
                    f"\n  步数 {count} | vx: {vx:+.2f} m/s | vy: {vy:+.2f} m/s | wz: {wz:+.2f} rad/s"
                )
                print(f"    按键状态: {_pressed}")
                # 调试：显示驱动轮
                print("    驱动轮:")
                for target_i, idx in enumerate(drive_indices):
                    print(
                        f"      [{idx}] {robot.joint_names[idx]}: 目标={drive_vel_targets[0, target_i].item():+.1f}, 实际={robot.data.joint_vel[0, idx].item():+.1f}"
                    )
                # 调试：显示转向关节
                print("    转向关节:")
                for target_i, idx in enumerate(steering_indices):
                    target_angle = steering_pos_targets[0, target_i].item()
                    print(
                        f"      [{idx}] {robot.joint_names[idx]}: 目标={math.degrees(target_angle):.1f}°, 实际={math.degrees(robot.data.joint_pos[0, idx].item()):.1f}°"
                    )
                # 显示机器人位置和朝向
                robot_pos = robot.data.root_pos_w[0].cpu().numpy()
                robot_quat = robot.data.root_quat_w[0].cpu().numpy()
                siny_cosp = 2 * (
                    robot_quat[0] * robot_quat[3]
                    + robot_quat[1] * robot_quat[2]
                )
                cosy_cosp = 1 - 2 * (
                    robot_quat[2] * robot_quat[2]
                    + robot_quat[3] * robot_quat[3]
                )
                yaw = math.atan2(siny_cosp, cosy_cosp)
                print(
                    f"    机器人位置: [{robot_pos[0]:.3f}, {robot_pos[1]:.3f}], 朝向: {math.degrees(yaw):.1f}°"
                )

except KeyboardInterrupt:
    print("\n✓ 用户停止")
except Exception as e:
    print(f"\n❌ 运行时错误: {e}")
    import traceback

    traceback.print_exc()
finally:
    # 停止键盘监听器
    if _listener:
        _listener.stop()
    print("\n关闭仿真...")
    simulation_app.close()
    print("✓ 完成!")
