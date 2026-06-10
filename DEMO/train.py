#!/usr/bin/env python3
"""
【双臂移动操作专属】极简、独立的 LeRobot 大模型模型训练脚本
支持 4 路高清相机输入 + 17 维复合动作状态空间的端到端模仿学习
"""

import os
import time
from pathlib import Path
import torch
import numpy as np

# 引入 LeRobot 核心组件
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy, ACTConfig
from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy, DiffusionConfig

def find_latest_dataset():
    """自动寻找本地录制好的最新双臂数据集"""
    dataset_root = Path(os.getcwd()) / "outputs/datasets"
    if not dataset_root.exists():
        raise FileNotFoundError(f"未找到数据集根目录: {dataset_root}，请先运行录制脚本。")
    
    folders = [f for f in dataset_root.iterdir() if f.is_dir() and f.name.startswith("bimanual_isaac_dataset")]
    if not folders:
        raise FileNotFoundError("outputs/datasets 下没有找到以 bimanual_isaac_dataset 开头的数据集！")
    
    # 按修改时间排序，拿到最新录制的那一个
    folders.sort(key=lambda x: x.stat().st_mtime)
    return folders[-1]

def main():
    print("==========================================================")
    print("🏋️  复合双臂移动机器人大模型训练流水线启动")
    print("==========================================================")
    
    # 1. 自动绑定你刚刚录制好的最新黄金数据集
    dataset_path = find_latest_dataset()
    print(f"📦 成功加载本地最新数据集: {dataset_path.name}")
    
    # 2. 初始化 LeRobot 数据集加载器
    # 框架会自动识别我们在 record.py 里声明的 17维 observation.state 和 4路 video 通道
    dataset = LeRobotDataset(
        repo_id=str(dataset_path),
        use_videos=True,
        tolerance_s=0.1
    )
    
    print(f"📊 数据集总帧数: {len(dataset)}")
    print(f"🎥 视频视角通道: cam_front, cam_head, cam_left, cam_right")
    print(f"🤖 机器人状态空间: 17 维复合轴状态")
    
    # 3. 硬核配置策略模型（以最常用的 ACT 银弹模型为例）
    # 必须把 4 路相机名字和 17 维特征死死注册进去，否则模型输入层会和数据对不上
    print("🧠 正在构建大模型网络拓扑结构...")
    
    policy_cfg = ACTConfig()
    
    # 注入输入的观测特征：包含 4 路图像和 1 维物理状态
    policy_cfg.input_features = {
        "observation.images.cam_front": {"dtype": "video", "shape": [3, 720, 1280]},
        "observation.images.cam_head":  {"dtype": "video", "shape": [3, 720, 1280]},
        "observation.images.cam_left":  {"dtype": "video", "shape": [3, 720, 1280]},
        "observation.images.cam_right": {"dtype": "video", "shape": [3, 720, 1280]},
        "observation.state":            {"dtype": "float32", "shape": [17]},
    }
    
    # 注入输出的动作特征：17维连续控制动作
    policy_cfg.output_features = {
        "action": {"dtype": "float32", "shape": [10, 17]}, # 10 代表一次前向预测未来 10 步动作 (Chunk Size)
    }
    
    # 超参数调优
    policy_cfg.n_action_steps = 10
    policy_cfg.chunk_size = 10
    policy_cfg.hidden_dim = 512
    policy_cfg.dim_feedforward = 3200
    
    # 实例化策略
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy = ACTPolicy(policy_cfg)
    policy.to(device)
    policy.train()
    
    # 4. 配置优化器与数据加载器 (DataLoader)
    optimizer = torch.optim.AdamW(policy.parameters(), lr=1e-4, weight_decay=1e-4)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        batch_size=4,        # 4路720P显存开销极大，若GPU爆显存(OOM)请将 batch_size 改为 2 或 1
        shuffle=True,
        num_workers=2,
        pin_memory=True
    )
    
    # 创建模型权重保存目录
    output_dir = Path(os.getcwd()) / "outputs/train" / f"act_run_{int(time.time())}"
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 5. 开启核心训练循环
    EPOCHS = 50  # 先跑 50 个 Epoch 快速验证收敛性
    print(f"\n🚀 开始在设备 [{device}] 上进行过拟合实验与收敛性训练...")
    print(f"💾 模型检查点将实时保存至: {output_dir}")
    
    for epoch in range(EPOCHS):
        epoch_loss = 0.0
        start_time = time.time()
        
        for batch_idx, batch in enumerate(dataloader):
            # 将所有复杂的张量安全移交到 GPU
            batch = {k: v.to(device) if isinstance(v, torch.Tensor) else v for k, v in batch.items()}
            
            # 模型前向传播计算损失值
            loss_dict = policy.forward(batch)
            loss = loss_dict["loss"]
            
            # 反向传播与梯度更新
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            epoch_loss += loss.item()
            
        avg_loss = epoch_loss / len(dataloader)
        elapsed = time.time() - start_time
        print(f"Epoch [{epoch+1:02d}/{EPOCHS:02d}] - 🌟 Avg Loss: {avg_loss:.6f} - ⏱️ 耗时: {elapsed:.2f}s")
        
        # 每隔 10 个 Epoch 自动固化一次权重
        if (epoch + 1) % 10 == 0 or (epoch + 1) == EPOCHS:
            checkpoint_path = output_dir / f"checkpoint_epoch_{epoch+1}.pt"
            torch.save({
                'epoch': epoch + 1,
                'model_state_dict': policy.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'loss': avg_loss,
            }, checkpoint_path)
            print(f"💾 检查点固化成功: {checkpoint_path.name}")

    print(f"\n🎉 恭喜！双臂 VLA 模仿学习大模型初步训练完成！")
    print(f"🌟 最终权重已安全封存在: {output_dir}")

if __name__ == "__main__":
    main()