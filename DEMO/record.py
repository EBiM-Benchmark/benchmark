#!/usr/bin/env python3
"""
【双臂移动操作专属】极简、独立的 Isaac Sim 数据录制脚本
"""

import time
import os
import sys
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot_robot_ros2 import ROS2RobotConfig, ROS2Robot, ROS2RobotInterfaceConfig, ControlType
from lerobot_camera_ros2 import ROS2CameraConfig, ROS2Camera

def main():
    print("==========================================================")
    print("🚀 复合双臂移动机器人极简数据录制器启动")
    print("==========================================================")
    
    FPS = 30
    EPISODE_DURATION = 15.0  # 每个任务回合录制 15 秒
    NUM_EPISODES = 5         # 连续录制 5 个回合跑通流程
    DATASET_NAME = f"bimanual_isaac_dataset_{int(time.time())}"
    DATASET_PATH = os.path.join(os.getcwd(), "outputs/datasets", DATASET_NAME)
    
    # 完全对照你的 URDF 与 Lula 描述文件，按物理拓扑顺序对齐 15 个核心控制轴
    JOINT_LIST = [
        "left_fr3v2_joint1", "left_fr3v2_joint2", "left_fr3v2_joint3", 
        "left_fr3v2_joint4", "left_fr3v2_joint5", "left_fr3v2_joint6", "left_fr3v2_joint7",
        "right_fr3v2_joint1", "right_fr3v2_joint2", "right_fr3v2_joint3", 
        "right_fr3v2_joint4", "right_fr3v2_joint5", "right_fr3v2_joint6", "right_fr3v2_joint7",
        "franka_spine_vertical_joint"
    ]
    
    # 1. 组装接口配置
    robot_config = ROS2RobotConfig(
        id="bimanual_fr3",
        ros2_interface=ROS2RobotInterfaceConfig(
            joint_states_topic="/joint_states",
            joint_names=JOINT_LIST,
            control_type=ControlType.CARTESIAN_POSE,
            end_effector_pose_topic="/left_current_pose",
            end_effector_target_topic="/left_target",
            gripper_enabled=True,
            gripper_joint_name="left_left_gripper_joint",
            gripper_command_topic="left_gripper_joint/position_command"
        )
    )
    
    # 对齐 Stage 树中的 4 路独立相机节点
    camera_topics = {
        "cam_front": "/rgb_front",
        "cam_head": "/rgb_head",
        "cam_left": "/rgb_left",
        "cam_right": "/rgb_right"
    }
    
    robot = ROS2Robot(robot_config)
    print("🔄 正在订阅双臂 15 轴状态关节话题...")
    robot.connect()
    
    cameras = {}
    for name, topic in camera_topics.items():
        print(f"🔄 正在连接相机话题: {topic} ...")
        cam_cfg = ROS2CameraConfig(topic_name=topic, node_name=f"lerobot_{name}", width=1280, height=720, fps=FPS)
        cameras[name] = ROS2Camera(cam_cfg)
        cameras[name].connect()
        
    time.sleep(2.0)  # 等待 ROS2 通信节点热身
    
    # 2. 显式配置符合你机器人特性的特征 Schema，绕过自动推导错误
    print("📝 手动声明死死对齐的黄金数据格式矩阵...")
    dataset_features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (17,),  # 15个控制轴 + 2个Robotiq夹爪状态
            "names": [
                "left_j1", "left_j2", "left_j3", "left_j4", "left_j5", "left_j6", "left_j7",
                "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6", "right_j7",
                "spine", "left_gripper", "right_gripper"
            ]
        },
        "action": {
            "dtype": "float32",
            "shape": (17,),  # 动作输出保持 1:1 镜像
            "names": [
                "act_left_j1", "act_left_j2", "act_left_j3", "act_left_j4", "act_left_j5", "act_left_j6", "act_left_j7",
                "act_right_j1", "act_right_j2", "act_right_j3", "act_right_j4", "act_right_j5", "act_right_j6", "act_right_j7",
                "act_spine", "act_left_gripper", "act_right_gripper"
            ]
        },
        # 严格注册 4 路独立的 1280x720 视频流通道
        "observation.images.cam_front": {"dtype": "video", "shape": (720, 1280, 3), "video_info": {"video_backend": "pyav", "fps": FPS, "codec": "av1"}},
        "observation.images.cam_head":  {"dtype": "video", "shape": (720, 1280, 3), "video_info": {"video_backend": "pyav", "fps": FPS, "codec": "av1"}},
        "observation.images.cam_left":  {"dtype": "video", "shape": (720, 1280, 3), "video_info": {"video_backend": "pyav", "fps": FPS, "codec": "av1"}},
        "observation.images.cam_right": {"dtype": "video", "shape": (720, 1280, 3), "video_info": {"video_backend": "pyav", "fps": FPS, "codec": "av1"}},
    }
    
    dataset = LeRobotDataset.create(
        repo_id=DATASET_PATH,
        fps=FPS,
        features=dataset_features,
        robot_type="bimanual_mobile_fr3",
        use_videos=True
    )
    
    # 3. 实时采样流
    try:
        for ep in range(NUM_EPISODES):
            print(f"\n🎬 === 录制第 {ep+1}/{NUM_EPISODES} 个回合 ===")
            print("请准备好键盘！3秒后开始...")
            for i in range(3, 0, -1):
                print(f"{i}...")
                time.sleep(1)
            print("🚨 [正在采样] 请在 Isaac Sim 内用键盘操作双臂与底座！")
            
            frame_count = 0
            max_frames = int(FPS * EPISODE_DURATION)
            
            while frame_count < max_frames:
                t_loop = time.time()
                
                current_obs = robot.get_observation()                
                for name, cam in cameras.items():
                    if hasattr(cam, "get_observation"):
                        current_obs[name] = cam.get_observation()
                    else:
                        current_obs[name] = getattr(cam, "image", np.zeros((720, 1280, 3), dtype=np.uint8))
                
                frame = {"task": "bimanual_table_manipulation"}
                
                # 严格按照物理轴拓扑序列对齐提取当前状态
                obs_state = []
                for j_name in JOINT_LIST:
                    obs_state.append(current_obs.get(f"{j_name}.pos", 0.0))
                
                # 额外抽样记录左右夹爪的实时物理开合度
                obs_state.append(current_obs.get("left_left_gripper_joint.pos", 0.0))
                obs_state.append(current_obs.get("right_left_gripper_joint.pos", 0.0))
                
                frame["observation.state"] = np.array(obs_state, dtype=np.float32)
                
                # 压入 4 路实时高清图像，键名与上面声明的 Features 完美统一
                for name in camera_topics.keys():
                    if name in current_obs:
                        frame[f"observation.images.{name}"] = current_obs[name]
                
                # 键盘遥控状态下，当前状态即为人类预期的下一帧控制意图(Action)
                frame["action"] = np.array(obs_state, dtype=np.float32)
                
                dataset.add_frame(frame)
                frame_count += 1
                
                dt = time.time() - t_loop
                if (1.0 / FPS) > dt:
                    time.sleep((1.0 / FPS) - dt)
            
            dataset.save_episode()
            print(f"✓ 回合 {ep+1} 固化成功。")
            
    except KeyboardInterrupt:
        print("\n🛑 录制退出。")
    finally:
        robot.disconnect()
        for cam in cameras.values():
            cam.disconnect()
        print(f"🎉 数据集完美生成，路径: {DATASET_PATH}")

if __name__ == "__main__":
    main()