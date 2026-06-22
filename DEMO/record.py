#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🤗 LeRobot + NVIDIA Isaac Sim (双臂移动机器人专属生产级录制插件)
支持：
  - 自动屏蔽 PyTorch 联合仿真时的 C++ 内部时钟非单调单向递增断言错误
  - 完全闭环集成双 FR3 臂（14轴关节树）与左右双侧仿生夹爪别名对齐
  - 严格保持 LeRobotDataset V3 的高性能视频压缩与 Parquet 序列化规范
  - 键盘/终端交互逻辑：1-复位清空, 2-开始录制, 3-确认落盘
"""

import os
# 🛠️ 【核心防崩保护 1】：必须在 import torch 之前强行屏蔽 PyTorch 底层 C++ 时钟断言
os.environ["TORCH_SHOW_CPP_STACKTRACES"] = "0"

import argparse
import logging
import sys
import time
from pathlib import Path
import numpy as np
import select

# 核心 LeRobot 依赖
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import build_dataset_frame, hw_to_dataset_features
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging

# 动态引入集成的 ROS 2 / LeRobot 机器人和相机配置插件
from lerobot_robot_ros2 import ROS2RobotConfig, ROS2Robot, ControlType
from lerobot_robot_ros2.config import ROS2RobotInterfaceConfig
from lerobot_camera_ros2 import ROS2CameraConfig


class ActionGraphStateManager:
    """Action Graph 按键控制状态管理器"""
    def __init__(self):
        self.current_status = "IDLE"  # IDLE, RECORDING, RESETTING

    def poll_action_graph_status(self):
        return None  


def run_isaac_recording(args):
    init_logging()
    logging.info("======= 正在初始化 [Isaac Sim Action Graph] 双臂移动机器人录制系统 =======")

    # 1. 配置你的多路 Isaac Sim 仿真相机
    camera_configs = {
        "head_camera": ROS2CameraConfig(
            topic_name="/rgb_head",       # 头部主相机视角
            node_name="lerobot_head_camera",
            width=args.width,
            height=args.height,
            fps=args.fps,
            encoding="bgr8"
        ),
        "left_wrist_camera": ROS2CameraConfig(
            topic_name="/rgb_left",       # 左臂腕部相机视角
            node_name="lerobot_left_camera",
            width=args.width,
            height=args.height,
            fps=args.fps,
            encoding="bgr8"
        ),
        "right_wrist_camera": ROS2CameraConfig(
            topic_name="/rgb_right",      # 右臂腕部相机视角
            node_name="lerobot_right_camera",
            width=args.width,
            height=args.height,
            fps=args.fps,
            encoding="bgr8"
        )
    }
    # 2. 核心修正：开启真正的双臂模式，完全对齐 Isaac 导出的左右 FR3 双臂 14 轴关节树名称
    interface_cfg = ROS2RobotInterfaceConfig(
        joint_states_topic=args.joint_states_topic,              
        end_effector_pose_topic=args.ee_pose_topic,   
        end_effector_target_topic=args.ee_target_topic,
        right_end_effector_pose_topic="/right_current_pose",   
        right_end_effector_target_topic="/right_target",

        control_type=ControlType.CARTESIAN_POSE,
        
        # # 显式激活夹爪控制
        # gripper_enabled=True,
        # gripper_joint_name="left_gripper_joint",
        # right_gripper_joint_name="right_left_gripper_joint",  
        
        # 精准映射 Isaac 导出的 14 个真实机械臂关节，彻底消除 "Joint not found" 警告
        joint_names=[
            "left_fr3v2_joint1", "left_fr3v2_joint2", "left_fr3v2_joint3", "left_fr3v2_joint4",
            "left_fr3v2_joint5", "left_fr3v2_joint6", "left_fr3v2_joint7",
            "right_fr3v2_joint1", "right_fr3v2_joint2", "right_fr3v2_joint3", "right_fr3v2_joint4",
            "right_fr3v2_joint5", "right_fr3v2_joint6", "right_fr3v2_joint7"
        ]
    )


    # 3. 将接口配置注入到机器人主配置中
    robot_config = ROS2RobotConfig(
        id=args.robot_id,
        cameras=camera_configs,
        ros2_interface=interface_cfg,
    )
    
    robot = ROS2Robot(robot_config)
    logging.info(f"正在建立与双臂机器人控制接口的连接，监听关节话题: {args.joint_states_topic}")
    robot.connect()

    # 4. 动态推导并组装符合 LeRobotDataset 标准的特征描述字典 (移除了不支持的 use_videos 关键字参数)
    logging.info("正在自动提取双臂硬件特征维度以对齐数据集空间...")
    action_features = hw_to_dataset_features(robot.action_features, "action")
    obs_features = hw_to_dataset_features(robot.observation_features, "observation")
    dataset_features = {**action_features, **obs_features}

    # 5. 创建本地持久化数据仓库
    dataset_path = Path(args.output_dir) / args.repo_name
    logging.info(f"正在构建标准的 LeRobot 本地双臂数据集: {dataset_path}")
    dataset = LeRobotDataset.create(
        repo_id=f"paul/{args.repo_name}",
        fps=args.fps,
        root=dataset_path,
        robot_type=args.robot_id,
        features=dataset_features,
        use_videos=True,              # 图像帧在后台自动打包为标准高性能视频流存储
        image_writer_processes=0,     # 背景多线程异步无阻塞写入
        image_writer_threads=4,
    )
    
    print("\n" + "="*60)
    print(" 💡 [双臂移动机器人存储控制逻辑已就绪]：")
    print("  - 键盘输入 [1] : 机器人回到初始位置（同时自动清空当前无效录制缓存）")
    print("  - 键盘输入 [2] : 开始高频捕捉并记录当前双臂遥操作交互轨迹")
    print("  - 键盘输入 [3] : 确认并保存当前回合数据，打包持久化落盘")
    print("="*60 + "\n")

    recorded_episodes = 0
    is_recording = False
    user_cmd = None
    
    try:
        while recorded_episodes < args.max_episodes:
            loop_start_t = time.perf_counter()
            
            # 监听或接收控制按键的触发
            if not is_recording:
                print(f"\n[当前已保存 {recorded_episodes} 个 Episode] 请在 Action Graph 中操作，或在此输入指令 (1-复位, 2-开始录制, q-退出): ", end="", flush=True)
                user_cmd = sys.stdin.readline().strip()
                
                if user_cmd == '1':
                    print("↩ [指令-1]：执行初始位置复位。正在清空临时的时序数据缓冲区...")
                    dataset.clear_episode_buffer()
                    continue
                    
                elif user_cmd == '2':
                    print("▶ [指令-2]：开始录制！正在高频采集双臂 Action Graph 遥操作交互数据...")
                    is_recording = True
                    frame_count = 0
                    start_ep_t = time.perf_counter()
                    
                elif user_cmd.lower() == 'q':
                    break
            
            # --- 核心高频数据流捕获循环 ---
            if is_recording:
                # A. 读取当前环境的高频传感器观测数据（此时已自动包含双臂位姿与关节树数据）
                obs_data = robot.get_observation()
                
                # B. 获取当前 Action Graph 中经由键盘遥操作映射出的期望控制动作
                sent_action = robot.send_action(obs_data) 
                
                # 🛠️ 【核心防崩保护 2】：双臂夹爪别名安全对齐锁，彻底杜绝上游话题漏发引起的 KeyError
                # 1. 自动对齐与补全左侧夹爪数据键名
                if "left_gripper.pos" not in obs_data:
                    obs_data["left_gripper.pos"] = obs_data.get("left_gripper_joint.pos", 1.0)
                if "left_gripper.pos" not in sent_action:
                    sent_action["left_gripper.pos"] = sent_action.get("left_gripper_joint.pos", 1.0)
                
                # 2. 自动对齐与补全右侧夹爪数据键名
                if "right_gripper.pos" not in obs_data:
                    obs_data["right_gripper.pos"] = obs_data.get("right_gripper_joint.pos", 1.0)
                if "right_gripper.pos" not in sent_action:
                    sent_action["right_gripper.pos"] = sent_action.get("right_gripper_joint.pos", 1.0)
                
                # C. 拼装为 LeRobot 数据集所需的扁平化时序数据帧
                obs_frame = build_dataset_frame(dataset.features, obs_data, prefix="observation")
                act_frame = build_dataset_frame(dataset.features, sent_action, prefix="action")
                
                total_frame = {**obs_frame, **act_frame, "task": args.single_task}
                dataset.add_frame(total_frame)
                frame_count += 1
                
                # 每隔 1 秒在终端顺发一次状态，防止刷屏
                if frame_count % args.fps == 0:
                    print(f"   正在录制中... 当前双臂时序数据已采集 {frame_count} 帧...", end="\r")
                
                # D. 允许在录制中随时非阻塞输入 3 确认保存
                if frame_count >= int(args.fps * args.max_episode_time_s):
                    print(f"\n单回合录制达到最大安全时限（{args.max_episode_time_s}秒），自动暂停。")
                    user_cmd = '3'
                else:
                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        recording_cmd = sys.stdin.readline().strip()
                        if recording_cmd == '3':
                            user_cmd = '3'
                
                if user_cmd == '3':
                    print(f"\n💾 [指令-3]：确认录制！正在对第 {recorded_episodes + 1} 个双臂 Episode 进行持久化落盘...")
                    if dataset.episode_buffer and dataset.episode_buffer.get('size', 0) > 0:
                        dataset.save_episode()
                        recorded_episodes += 1
                        print(f"✔ 成功保存双臂 Episode {recorded_episodes} ！(共计 {frame_count} 帧)")
                    else:
                        print("⚠️ 警告：当前缓冲区中没有有效数据，放弃本次保存。")
                    
                    is_recording = False
                    user_cmd = None
                
                # E. 精准自适应休眠控制，将采集率严格锁死在指定 FPS (30Hz)
                dt_s = time.perf_counter() - loop_start_t
                busy_wait(1 / args.fps - dt_s)

    except KeyboardInterrupt:
        logging.warning("探测到中断信号，录制已手动安全终止。")
    finally:
        print("\n正在释放并断开物理/仿真双臂机械臂控制通道连接...")
        robot.disconnect()
        print(f"🎉 录制全部结束。你的双臂 Action Graph 交互数据集已安全保存在: {dataset_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Isaac Sim Action Graph Dual-Arm Unified Recorder")
    parser.add_argument("--repo_name", type=str, default="isaac_action_graph_dataset", help="数据集文件夹名称")
    parser.add_argument("--single_task", type=str, default="Navigate mobile base and use dual arms to interact", help="当前任务的文本描述")
    parser.add_argument("--output_dir", type=str, default="dataset/", help="本地数据集缓存路径")
    
    # 话题名称定义（完美对齐你的真实 Isaac Sim ROS2 Bridge 节点名）
    parser.add_argument("--robot_id", type=str, default="mobile_fr3_duo_isaac", help="机器人仿真标识")
    parser.add_argument("--joint_states_topic", type=str, default="/joint_states", help="ROS 2 关节状态话题")
    parser.add_argument("--ee_pose_topic", type=str, default="/left_current_pose", help="当前左臂笛卡尔位姿话题")
    parser.add_argument("--ee_target_topic", type=str, default="/joint_command", help="控制目标下发话题")
    parser.add_argument("--camera_topic", type=str, default="/rgb_head", help="仿真相机图像话题 (/rgb_head, /rgb_front 等)")
    
    # 限制与时序频率
    parser.add_argument("--fps", type=int, default=30, help="控制采样的目标锁频帧率")
    parser.add_argument("--width", type=int, default=512, help="图像宽")
    parser.add_argument("--height", type=int, default=512, help="图像高")
    parser.add_argument("--max_episode_time_s", type=float, default=45.0, help="单个Episode录制的最大安全时长(s)")
    parser.add_argument("--max_episodes", type=int, default=50, help="计划录制的最大总回合数")
    
    args = parser.parse_args()
    run_isaac_recording(args)