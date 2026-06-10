#!/usr/bin/env python3
"""
完整场景 + 键盘控制：10个桌子 + 9个字母 + 3个餐具 + 1个可控机器人
"""

import argparse
import sys

print("=" * 80)
print("完整场景 + 键盘控制")
print("=" * 80)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1, help="环境数量")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 导入Isaac Lab模块
from pynput import keyboard

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

SCRIPT_ROOT = __file__
COMMON_DIR = (
    __import__("pathlib").Path(__file__).resolve().parents[1] / "common"
)
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import asset_path, franka_urdf_path
from tmr_base_control import (
    compensate_yaw_rate,
    compute_drive_targets,
    find_drive_joint_ids,
    get_keyboard_twist,
    get_root_yaw,
)

print("✓ 模块导入完成")

table_path = str(asset_path("table_edit.usd"))
robot_path = str(franka_urdf_path("mobile_fr3_duo_v0_2_franka_hand.usd"))

# 定义所有字母路径
letter_paths = {}
for letter in ["A", "B", "C", "D", "E", "F", "G", "H", "I"]:
    letter_file = asset_path(f"{letter}_edit.usd")
    if not letter_file.exists():
        raise FileNotFoundError(f"找不到字母文件: {letter_file}")
    letter_paths[letter] = str(letter_file)

# 定义餐具路径
cutlery_paths = {}
for item in ["bowl", "plate", "spoon"]:
    cutlery_file = asset_path(f"{item}.usd")
    if not cutlery_file.exists():
        raise FileNotFoundError(f"找不到餐具文件: {cutlery_file}")
    cutlery_paths[item] = str(cutlery_file)

print("✓ 找到所有资源文件")


# 场景配置
@configclass
class CompleteSceneCfg(InteractiveSceneCfg):
    """完整场景配置"""

    # 地面
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(size=(20.0, 20.0)),
    )

    # 光照
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=5000.0, color=(1.0, 1.0, 1.0)),
    )

    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(
            intensity=3000.0, color=(1.0, 1.0, 1.0), angle=0.5
        ),
    )

    # 左列5个桌子
    table_left_1 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Left_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, 3.0, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_left_2 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Left_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, 1.5, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_left_3 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Left_3",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, 0.0, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_left_4 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Left_4",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, -1.5, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_left_5 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Left_5",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, -3.0, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    # 右列5个桌子
    table_right_1 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Right_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, 3.0, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_right_2 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Right_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, 1.5, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_right_3 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Right_3",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, 0.0, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_right_4 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Right_4",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, -1.5, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    table_right_5 = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Right_5",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, -3.0, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )

    # 底部中间1个桌子
    table_bottom_center = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Bottom_Center",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, -4.5, 0.7), rot=(0.7071, 0.7071, 0.0, 0.0)
        ),
    )


# 机器人配置
initial_joint_pos = {
    "left_fr3v2_joint1": 0.0,
    "left_fr3v2_joint2": -1.5,
    "left_fr3v2_joint3": 0.0,
    "left_fr3v2_joint4": -2.2,
    "left_fr3v2_joint5": 0.0,
    "left_fr3v2_joint6": 1.5,
    "left_fr3v2_joint7": 0.785,
    "right_fr3v2_joint1": 0.0,
    "right_fr3v2_joint2": -1.5,
    "right_fr3v2_joint3": 0.0,
    "right_fr3v2_joint4": -2.2,
    "right_fr3v2_joint5": 0.0,
    "right_fr3v2_joint6": 1.5,
    "right_fr3v2_joint7": 0.785,
}

robot_cfg = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=robot_path),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 4.5, 0.0),  # 顶部中间位置
        joint_pos=initial_joint_pos,
    ),
    actuators={
        "steering_joints": ImplicitActuatorCfg(
            joint_names_expr=["tmrv0_2_joint_0", "tmrv0_2_joint_2"],
            stiffness=500.0,
            damping=50.0,
            effort_limit=200.0,
        ),
        "drive_joints": ImplicitActuatorCfg(
            joint_names_expr=["tmrv0_2_joint_1", "tmrv0_2_joint_3"],
            stiffness=0.0,
            damping=5.0,
            effort_limit_sim=200.0,
            velocity_limit_sim=20.0,
        ),
        "passive_base_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*caster.*", "rocker_arm_joint"],
            stiffness=0.0,
            damping=0.0,
        ),
        "arm_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*fr3v2_joint.*"],
            stiffness=100.0,
            damping=20.0,
            effort_limit=87.0,
        ),
    },
)

print("✓ 场景配置创建完成")

# 创建场景实例
scene_cfg = CompleteSceneCfg(num_envs=args.num_envs, env_spacing=10.0)

# 动态添加机器人
scene_cfg.robot = robot_cfg

# 动态添加9个字母
letter_configs = {
    "A": {"table_pos": (-2.0, 1.5, 0.7)},
    "B": {"table_pos": (2.0, 1.5, 0.7)},
    "C": {"table_pos": (-2.0, 0.0, 0.7)},
    "D": {"table_pos": (2.0, 0.0, 0.7)},
    "E": {"table_pos": (-2.0, -1.5, 0.7)},
    "F": {"table_pos": (2.0, -1.5, 0.7)},
    "G": {"table_pos": (-2.0, -3.0, 0.7)},
    "H": {"table_pos": (2.0, -3.0, 0.7)},
    "I": {"table_pos": (0.0, -4.5, 0.7)},
}

for letter, config in letter_configs.items():
    tx, ty, tz = config["table_pos"]
    letter_pos = (tx + 0.35, ty - 0.45, tz + 0.061)

    setattr(
        scene_cfg,
        f"letter_{letter}",
        AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Letter_" + letter,
            spawn=sim_utils.UsdFileCfg(
                usd_path=letter_paths[letter],
                scale=(0.001, 0.001, 0.001),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.0, 0.0, 0.0),
                    emissive_color=(0.0, 0.0, 0.0),
                    metallic=0.0,
                    roughness=0.8,
                ),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=letter_pos,
                rot=(0.7071, 0.7071, 0.0, 0.0),
            ),
        ),
    )

# 动态添加3个餐具
ikea_configs = {
    "bowl": {
        "offset": (0.45, -0.3, 0.15),
        "color": (1.0, 0.0, 0.0),
        "rot": (0.5, 0.5, 0.5, 0.5),
    },
    "plate": {
        "offset": (0.2, -0.3, 0.08),
        "color": (1.0, 1.0, 0.0),
        "rot": (0.5, 0.5, 0.5, 0.5),
    },
    "spoon": {
        "offset": (0.4, -0.3, 0.07),
        "color": (0.0, 0.0, 1.0),
        "rot": (0.0, 0.7071, 0.0, 0.7071),
    },
}

ikea_table_pos = (-2.0, 3.0, 0.7)

for item, config in ikea_configs.items():
    offset_x, offset_y, offset_z = config["offset"]
    item_pos = (
        ikea_table_pos[0] + offset_x,
        ikea_table_pos[1] + offset_y,
        ikea_table_pos[2] + offset_z,
    )

    setattr(
        scene_cfg,
        f"ikea_{item}",
        AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Ikea_" + item.capitalize(),
            spawn=sim_utils.UsdFileCfg(
                usd_path=cutlery_paths[item],
                scale=(0.001, 0.001, 0.001),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=config["color"],
                    metallic=0.2,
                    roughness=0.4,
                ),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=item_pos,
                rot=config["rot"],
            ),
        ),
    )

print("✓ 完整场景配置创建完成（10桌 + 9字母 + 3餐具 + 机器人）")

# 创建仿真上下文
sim_cfg = sim_utils.SimulationCfg(dt=1 / 60, device="cuda:0")
sim = SimulationContext(sim_cfg)
sim.set_camera_view(eye=[0.0, 8.0, 8.0], target=[0.0, 0.0, 0.0])

print("✓ 仿真上下文创建完成")

# 创建场景
scene = InteractiveScene(scene_cfg)

print("✓ 场景创建完成")

# 物理初始化
sim.reset()
scene.reset()

# 强制设置所有字母为黑色
print("正在设置字母颜色...")
from pxr import Usd, UsdShade

stage = sim.stage

for letter in ["A", "B", "C", "D", "E", "F", "G", "H", "I"]:
    letter_prim_path = f"/World/envs/env_0/Letter_{letter}"
    letter_prim = stage.GetPrimAtPath(letter_prim_path)

    if letter_prim.IsValid():
        for child_prim in Usd.PrimRange(letter_prim):
            if child_prim.IsA(UsdShade.Material):
                material = UsdShade.Material(child_prim)
                shader = material.GetSurfaceOutput().GetConnectedSource()[0]
                if shader:
                    shader_prim = shader.GetPrim()
                    if shader_prim.HasAttribute("inputs:diffuseColor"):
                        shader_prim.GetAttribute("inputs:diffuseColor").Set(
                            (0.0, 0.0, 0.0)
                        )

print("✓ 物理初始化完成")

# 获取机器人对象
robot = scene["robot"]
steering_indices, drive_indices = find_drive_joint_ids(robot.joint_names)
print(f"主动转向关节索引: {steering_indices}")
print(f"主动驱动轮索引: {drive_indices}")
heading_hold_yaw = get_root_yaw(robot)

# ========== 键盘控制设置 ==========
_pressed = set()
_listener = None


def _on_press(key):
    """按键按下时触发"""
    try:
        if hasattr(key, "char") and key.char:
            _pressed.add(key.char.lower())
        elif hasattr(key, "name"):
            _pressed.add(key.name)
    except AttributeError:
        if key == keyboard.Key.esc:
            return False


def _on_release(key):
    """按键释放时触发"""
    try:
        if hasattr(key, "char") and key.char:
            _pressed.discard(key.char.lower())
        elif hasattr(key, "name"):
            _pressed.discard(key.name)
    except AttributeError:
        pass
    if key == keyboard.Key.esc:
        return False


def start_listener():
    """启动键盘监听器"""
    global _listener
    _listener = keyboard.Listener(
        on_press=_on_press, on_release=_on_release, suppress=False
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
print("      机器人在顶部中间位置，A/D 是侧移，Q/E 是旋转。\n")

# 仿真循环
count = 0
scene_saved = False
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
        if count % 200 == 0:
            print(f"步数 {count} - 场景运行正常")

        # 在第1000步保存场景
        # if count == 1000 and not scene_saved:
        #     save_path = os.path.join(script_dir, "assets/complete_scene_with_robot_keyboard.usd")
        #     sim.stage.Export(save_path)
        #     print(f"\n✓ 完整场景已保存到: {save_path}")
        #     print(f"   包含: 10个桌子 + 1个机器人 + 9个字母 + 3个餐具\n")
        #     scene_saved = True

        # 显示控制状态
        if count % 50 == 0:
            if vx != 0 or vy != 0 or wz != 0:
                print(
                    f"  步数 {count} | vx: {vx:+.2f} m/s | vy: {vy:+.2f} m/s | wz: {wz:+.2f} rad/s | 按键: {_pressed}"
                )

except KeyboardInterrupt:
    print("\n仿真已停止")
finally:
    if _listener:
        _listener.stop()
    simulation_app.close()
