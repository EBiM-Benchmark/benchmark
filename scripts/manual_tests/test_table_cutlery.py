#!/usr/bin/env python3
"""
测试桌子和餐具的摆放
"""

import argparse
import sys
from pathlib import Path

print("=" * 80)
print("桌子 + 餐具测试场景")
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
bowl_path = str(asset_path("bowl.usd"))
plate_path = str(asset_path("plate.usd"))
spoon_path = str(asset_path("spoon.usd"))

for path, name in [
    (table_path, "桌子"),
    (bowl_path, "碗"),
    (plate_path, "盘子"),
    (spoon_path, "勺子"),
]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"找不到{name}文件: {path}")

print("✓ 找到所有文件")


# 场景配置
@configclass
class TableCutlerySceneCfg(InteractiveSceneCfg):
    """桌子和餐具的测试场景配置"""

    # 地面
    ground = AssetBaseCfg(
        prim_path="/World/Ground",
        spawn=sim_utils.GroundPlaneCfg(size=(10.0, 10.0)),
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

    # 桌子
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_path,
            scale=(0.001, 0.001, 0.001),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.70),  # 抬高桌子，让桌腿在地面上
            rot=(0.7071, 0.7071, 0.0, 0.0),  # X90 + Y180，让桌子站立
        ),
    )

    # 碗（红色）- 绕X转90度 + 绕Y转90度
    bowl = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Bowl",
        spawn=sim_utils.UsdFileCfg(
            usd_path=bowl_path,
            scale=(0.001, 0.001, 0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 0.0, 0.0),
                metallic=0.2,
                roughness=0.4,
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.45, -0.3, 0.85),  # 按用户要求
            rot=(0.5, 0.5, 0.5, 0.5),  # X90 + Y90 组合旋转
        ),
    )

    # 盘子（黄色）- 绕X转90度 + 绕Y转90度
    plate = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Plate",
        spawn=sim_utils.UsdFileCfg(
            usd_path=plate_path,
            scale=(0.001, 0.001, 0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(1.0, 1.0, 0.0),
                metallic=0.2,
                roughness=0.4,
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.2, -0.3, 0.78),  # z改为0.78
            rot=(0.5, 0.5, 0.5, 0.5),  # X90 + Y90 组合旋转
        ),
    )

    # 勺子（蓝色）- 绕Y转90度
    spoon = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Spoon",
        spawn=sim_utils.UsdFileCfg(
            usd_path=spoon_path,
            scale=(0.001, 0.001, 0.001),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.0, 0.0, 1.0),
                metallic=0.2,
                roughness=0.4,
            ),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.4, -0.3, 0.77),  # z改为0.77
            rot=(0.0, 0.7071, 0.0, 0.7071),  # Y轴90度
        ),
    )


print("✓ 场景配置创建完成")

# 创建仿真上下文
sim_cfg = sim_utils.SimulationCfg(dt=1 / 60, device="cuda:0")
sim = SimulationContext(sim_cfg)
sim.set_camera_view(eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.5])

print("✓ 仿真上下文创建完成")

# 创建场景
scene_cfg = TableCutlerySceneCfg(num_envs=args.num_envs, env_spacing=3.0)
scene = InteractiveScene(scene_cfg)

print("✓ 场景创建完成")

# 物理初始化
sim.reset()
scene.reset()

print("✓ 物理初始化完成")
print("\n开始仿真...")
print("提示: 1个桌子 + 3个餐具（红碗/黄盘/蓝勺）")
print("按 Ctrl+C 退出\n")

# 仿真循环
count = 0
scene_saved = False
try:
    while simulation_app.is_running():
        # 步进仿真
        sim.step()
        scene.update(sim.cfg.dt)

        count += 1
        if count % 200 == 0:
            print(f"步数 {count} - 场景运行正常")

        # 在第120步（2秒后）保存场景，此时物体应该已经稳定
        if count == 1800 and not scene_saved:
            save_path = str(asset_path("saved_table_cutlery_scene.usd"))
            sim.stage.Export(save_path)
            print(f"\n✓ 场景已保存到: {save_path}\n")
            scene_saved = True

except KeyboardInterrupt:
    print("\n仿真已停止")
finally:
    simulation_app.close()
