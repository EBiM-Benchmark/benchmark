#!/usr/bin/env python3
"""
在场景中放置桌子的演示脚本
"""

import argparse
import sys
from pathlib import Path

print("=" * 80)
print("场景中的桌子演示")
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
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass

COMMON_DIR = Path(__file__).resolve().parents[1] / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import asset_path

print("✓ 模块导入完成")

# 获取桌子USD文件路径
table_usd_path = str(asset_path("table_edit.usd"))
if not Path(table_usd_path).exists():
    raise FileNotFoundError(f"找不到桌子文件: {table_usd_path}")
print(f"✓ 找到桌子文件: {table_usd_path}")


# 创建场景配置
@configclass
class TableSceneCfg(InteractiveSceneCfg):
    """带桌子的场景配置"""

    # 地面
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0,
                dynamic_friction=0.8,
                restitution=0.0,
            ),
        ),
    )

    # 光照
    dome_light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(1.0, 1.0, 1.0)),
    )

    # 桌子 - 作为刚体对象
    table = RigidObjectCfg(
        prim_path="/World/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=table_usd_path,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,  # 启用重力
                max_depenetration_velocity=1.0,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(
            pos=(1.0, 0.0, 0.0),  # 桌子位置：前方1米
            rot=(1.0, 0.0, 0.0, 0.0),  # 四元数：无旋转
        ),
    )


print("✓ 场景配置创建完成")

# 创建场景实例
scene_cfg = TableSceneCfg(num_envs=args.num_envs, env_spacing=2.0)

# 创建仿真上下文
sim_cfg = sim_utils.SimulationCfg(
    dt=0.01,
    device="cuda:0",
    gravity=(0.0, 0.0, -9.81),
)
sim = SimulationContext(sim_cfg)
sim.set_camera_view([2.5, 2.5, 2.0], [0.0, 0.0, 0.5])

print("✓ 仿真上下文创建完成")

# 创建场景
scene = InteractiveScene(scene_cfg)
print("✓ 场景创建完成")

# 重置场景
sim.reset()
scene.reset()
print("✓ 场景重置完成")

print("\n" + "=" * 80)
print("✓ 场景初始化完成!")
print("=" * 80)
print("\n提示: 按 Ctrl+C 退出\n")

# 仿真循环
count = 0
try:
    while simulation_app.is_running():
        # 步进仿真
        sim.step()
        scene.update(sim.cfg.dt)

        count += 1
        if count % 500 == 0:
            print(f"仿真步数: {count}")
            # 打印桌子状态
            table = scene["table"]
            table_pos = table.data.root_pos_w[0].cpu().numpy()
            print(
                f"  桌子位置: [{table_pos[0]:.3f}, {table_pos[1]:.3f}, {table_pos[2]:.3f}]"
            )

except KeyboardInterrupt:
    print("\n✓ 用户停止")
except Exception as e:
    print(f"\n❌ 运行时错误: {e}")
    import traceback

    traceback.print_exc()
finally:
    simulation_app.close()
    print("✓ 仿真关闭")
