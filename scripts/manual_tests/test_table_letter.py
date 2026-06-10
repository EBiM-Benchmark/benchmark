#!/usr/bin/env python3
"""
测试：1个桌子 + 1个字母
"""

import argparse
import sys
from pathlib import Path

print("=" * 80)
print("桌子与字母测试场景")
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
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

COMMON_DIR = Path(__file__).resolve().parents[1] / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import asset_path

print("✓ 模块导入完成")

# 获取文件路径
table_path = str(asset_path("table_edit.usd"))
letter_path = str(asset_path("A_edit.usd"))

if not os.path.exists(table_path):
    raise FileNotFoundError(f"找不到桌子文件: {table_path}")
if not os.path.exists(letter_path):
    raise FileNotFoundError(f"找不到字母文件: {letter_path}")

print(f"✓ 找到桌子文件: {table_path}")
print(f"✓ 找到字母文件: {letter_path}")


# 场景配置
@configclass
class TableLetterSceneCfg(InteractiveSceneCfg):
    """桌子与字母的场景配置"""

    # 地面
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(size=(10.0, 10.0)),
    )

    # 光照
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(1.0, 1.0, 1.0)),
    )

    # 桌子 - scale = 0.001，旋转90度站立
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path,
            scale=(0.001, 0.001, 0.001),  # 用户指定的scale
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(1.5, 0.0, 0.0),  # 桌子位置
            rot=(0.7071, 0.7071, 0.0, 0.0),  # 绕X轴旋转90度（四元数）
        ),
    )

    # 字母A - scale = 0.001，黑色，使用USD文件中保存的朝向
    letter_A = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Letter_A",
        spawn=sim_utils.UsdFileCfg(
            usd_path=letter_path,
            scale=(0.001, 0.001, 0.001),  # 保持与桌子相同的scale
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 0.0, 0.0),  # 黑色
                metallic=0.0,
                roughness=0.5,
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(1.85, -0.45, 0.7),  # 桌面中央
            rot=(0.7071, 0.7071, 0.0, 0.0),  # 绕X轴90度 + 绕Y轴180度
        ),
    )


print("✓ 场景配置创建完成")

# 创建仿真上下文
sim_cfg = sim_utils.SimulationCfg(dt=1 / 60, device="cuda:0")
sim = SimulationContext(sim_cfg)
sim.set_camera_view(eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.5])

print("✓ 仿真上下文创建完成")

# 创建场景
scene_cfg = TableLetterSceneCfg(num_envs=args.num_envs, env_spacing=3.0)
scene = InteractiveScene(scene_cfg)

print("✓ 场景创建完成")

# 物理初始化
sim.reset()
scene.reset()

print("✓ 物理初始化完成")
print("\n=== 场景检查 ===")
print("桌子prim路径: /World/envs/env_0/Table")
print("字母prim路径: /World/envs/env_0/Letter_A")

# 检查prims是否存在
stage = sim.stage
table_prim = stage.GetPrimAtPath("/World/envs/env_0/Table")
letter_prim = stage.GetPrimAtPath("/World/envs/env_0/Letter_A")

print(f"桌子是否存在: {table_prim.IsValid()}")
print(f"字母是否存在: {letter_prim.IsValid()}")

if letter_prim.IsValid():
    print(f"字母类型: {letter_prim.GetTypeName()}")
    # 列出字母的子prims
    children = letter_prim.GetChildren()
    print(f"字母子节点数量: {len(children)}")
    for child in children[:5]:  # 只显示前5个
        print(f"  - {child.GetPath()} ({child.GetTypeName()})")

print("开始仿真...")
print("提示: 字母A应该显示在桌子上方（高度1.5m）")
print("按 Ctrl+C 退出\n")

# 仿真循环
count = 0
try:
    while simulation_app.is_running():
        # 步进仿真
        sim.step()
        scene.update(sim.cfg.dt)

        count += 1
        if count % 100 == 0:
            # 打印桌子和字母的位置
            print(f"步数 {count}")
            print("  桌子路径: /World/envs/env_0/Table")
            print("  字母路径: /World/envs/env_0/Letter_A")

except KeyboardInterrupt:
    print("\n仿真已停止")
finally:
    simulation_app.close()
