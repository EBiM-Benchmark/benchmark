#!/usr/bin/env python3
"""
完整场景：10个桌子 + 9个字母 + 3个餐具 + 1个机器人
"""

import argparse
import sys

print("=" * 80)
print("完整场景：机器人 + 桌子 + 字母 + 餐具")
print("=" * 80)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1, help="环境数量")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

# 导入Isaac Lab模块
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

COMMON_DIR = (
    __import__("pathlib").Path(__file__).resolve().parents[1] / "common"
)
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import asset_path, franka_urdf_path

print("✓ 模块导入完成")

table_path = str(asset_path("table_edit.usd"))
robot_path = str(franka_urdf_path("mobile_fr3_duo_v0_2_franka_hand.usd"))

# 定义所有字母路径并检查文件存在
letter_paths = {}
for letter in ["A", "B", "C", "D", "E", "F", "G", "H", "I"]:
    letter_file = asset_path(f"{letter}_edit.usd")
    if not letter_file.exists():
        raise FileNotFoundError(f"找不到字母文件: {letter_file}")
    letter_paths[letter] = str(letter_file)

# 定义餐具路径并检查文件存在
cutlery_paths = {}
for item in ["bowl", "plate", "spoon"]:
    cutlery_file = asset_path(f"{item}.usd")
    if not cutlery_file.exists():
        raise FileNotFoundError(f"找不到餐具文件: {cutlery_file}")
    cutlery_paths[item] = str(cutlery_file)

if not os.path.exists(table_path):
    raise FileNotFoundError(f"找不到桌子文件: {table_path}")

if not os.path.exists(robot_path):
    raise FileNotFoundError(f"找不到机器人文件: {robot_path}")

print(f"✓ 找到桌子文件: {table_path}")
print("✓ 找到9个字母文件 (A-I)")
print("✓ 找到3个餐具文件 (bowl, plate, spoon)")
print(f"✓ 找到机器人文件: {robot_path}")


# 场景配置
@configclass
class CompleteSceneCfg(InteractiveSceneCfg):
    """完整场景配置：10桌 + 机器人 + 9字母 + 3餐具"""

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

    # 左列5个桌子（X=-2.0）
    table_left_1 = AssetBaseCfg(  # 餐具桌
        prim_path="{ENV_REGEX_NS}/Table_Left_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, 3.0, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_left_2 = AssetBaseCfg(  # 字母A
        prim_path="{ENV_REGEX_NS}/Table_Left_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, 1.5, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_left_3 = AssetBaseCfg(  # 字母C
        prim_path="{ENV_REGEX_NS}/Table_Left_3",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, 0.0, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_left_4 = AssetBaseCfg(  # 字母E
        prim_path="{ENV_REGEX_NS}/Table_Left_4",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, -1.5, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_left_5 = AssetBaseCfg(  # 字母G
        prim_path="{ENV_REGEX_NS}/Table_Left_5",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(-2.0, -3.0, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    # 右列5个桌子（X=2.0）
    table_right_1 = AssetBaseCfg(  # 空桌
        prim_path="{ENV_REGEX_NS}/Table_Right_1",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, 3.0, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_right_2 = AssetBaseCfg(  # 字母B
        prim_path="{ENV_REGEX_NS}/Table_Right_2",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, 1.5, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_right_3 = AssetBaseCfg(  # 字母D
        prim_path="{ENV_REGEX_NS}/Table_Right_3",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, 0.0, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_right_4 = AssetBaseCfg(  # 字母F
        prim_path="{ENV_REGEX_NS}/Table_Right_4",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, -1.5, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    table_right_5 = AssetBaseCfg(  # 字母H
        prim_path="{ENV_REGEX_NS}/Table_Right_5",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(2.0, -3.0, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
        ),
    )

    # 底部中间1个桌子（字母I）
    table_bottom_center = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table_Bottom_Center",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path, scale=(0.001, 0.001, 0.001)
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, -4.5, 0.7),
            rot=(0.7071, 0.7071, 0.0, 0.0),
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
        pos=(0.0, 4.5, 0.0),  # 顶部中间位置，替代原来的桌子
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
            joint_names_expr=[
                "tmrv0_2_joint_1",
                "tmrv0_2_joint_3",
                ".*caster.*",
                "rocker_arm_joint",
            ],
            stiffness=0.0,
            damping=5.0,
            effort_limit=200.0,
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

# 动态添加9个字母配置
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

print("✓ 9个字母配置添加完成")

# 动态添加3个餐具配置（在左上角餐具桌上）
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

print("✓ 3个餐具配置添加完成")

# 创建仿真上下文
sim_cfg = sim_utils.SimulationCfg(dt=1 / 60, device="cuda:0")
sim = SimulationContext(sim_cfg)
sim.set_camera_view(eye=[0.0, 0.0, 15.0], target=[0.0, 0.0, 0.0])

print("✓ 仿真上下文创建完成")

# 创建场景
scene = InteractiveScene(scene_cfg)

print("✓ 场景创建完成")

# 物理初始化
sim.reset()
scene.reset()

# 强制设置所有字母为黑色材质
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

print("✓ 物理初始化和字母颜色设置完成")
print("\n开始仿真...")
print("提示: 完整场景已加载")
print("  - 10个桌子（移除顶部中间桌子）")
print("  - 1个机器人（顶部中间位置）")
print("  - 9个字母（黑色）")
print("  - 3个餐具（红碗/黄盘/蓝勺）")
print("按 Ctrl+C 退出\n")

# 仿真循环
count = 0
scene_saved = False
try:
    while simulation_app.is_running():
        sim.step()
        scene.update(sim.cfg.dt)

        count += 1
        if count % 200 == 0:
            print(f"步数 {count} - 场景运行正常")

        # 在第1000步保存场景
        if count == 1000 and not scene_saved:
            save_path = str(asset_path("complete_scene_with_robot.usd"))
            sim.stage.Export(save_path)
            print(f"\n✓ 完整场景已保存到: {save_path}")
            print("   包含: 10个桌子 + 1个机器人 + 9个字母 + 3个餐具\n")
            scene_saved = True

except KeyboardInterrupt:
    print("\n仿真已停止")
finally:
    simulation_app.close()
