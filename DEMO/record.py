#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ["TORCH_SHOW_CPP_STACKTRACES"] = "0"

import argparse
import logging
import subprocess
import sys
import time
from pathlib import Path
import numpy as np
import select
import threading

# 核心 LeRobot 依赖
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import init_logging

# 动态引入集成的 ROS 2 插件
from lerobot_robot_ros2 import ROS2RobotConfig, ROS2Robot, ControlType
from lerobot_robot_ros2.config import ROS2RobotInterfaceConfig
from lerobot_camera_ros2 import ROS2CameraConfig
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def get_next_dataset_version_path(output_dir, repo_name):
    """Return the next available dataset/<repo_name>_vN path."""
    output_root = Path(output_dir)
    version = 1
    while True:
        versioned_repo_name = f"{repo_name}_v{version}"
        candidate = output_root / versioned_repo_name
        if not candidate.exists():
            return candidate, versioned_repo_name
        if not candidate.is_dir():
            raise FileExistsError(f"数据集路径已存在但不是文件夹: {candidate}")
        version += 1


def launch_dataset_visualization(repo_id, dataset_path, episode_index):
    env = os.environ.copy()
    lerobot_src = str((Path.cwd() / "submodules/lerobot/src").resolve())
    current_pythonpath = env.get("PYTHONPATH")
    env["PYTHONPATH"] = lerobot_src if not current_pythonpath else f"{lerobot_src}{os.pathsep}{current_pythonpath}"

    cmd = [
        sys.executable,
        "-m",
        "lerobot.scripts.visualize_dataset",
        "--repo-id",
        repo_id,
        "--root",
        str(dataset_path),
        "--episode-index",
        str(episode_index),
    ]
    return subprocess.Popen(cmd, env=env)


class JointDataInterceptor:
    """全功能拦截器：同时捕获双臂、脊柱、底盘里程计(/odom)与底盘速度指令(/cmd_vel)"""
    def __init__(self, ros2_node):
        self.node = ros2_node
        self.lock = threading.Lock()
        
        # 机械臂与脊柱缓存
        self.left_joint_cmds = [0.0] * 7
        self.right_joint_cmds = [0.0] * 7
        self.real_joint_map = {}

        # 🚀 底盘缓存数据初始化
        self.base_action_vel = [0.0, 0.0, 0.0]  # [vx, vy, wz]
        self.base_state_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]

        # 1. 订阅控制器发出的指令位置
        self.left_sub = self.node.create_subscription(JointState, '/leftarm_current_pose', self.left_cb, 10)
        self.right_sub = self.node.create_subscription(JointState, '/rightarm_current_pose', self.right_cb, 10)
        
        # 2. 订阅仿真环境返回的真实状态
        self.real_sub = self.node.create_subscription(JointState, '/joint_states', self.real_cb, 10)
        
        # 🚀 3. 新增底盘相关订阅
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_vel_sub = self.node.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        
        logging.info("往拦截器成功挂载底盘数据链：/odom 与 /cmd_vel")

    def left_cb(self, msg):
        if msg.position and len(msg.position) >= 7:
            with self.lock:
                self.left_joint_cmds = list(msg.position[:7])

    def right_cb(self, msg):
        if msg.position and len(msg.position) >= 7:
            with self.lock:
                self.right_joint_cmds = list(msg.position[:7])

    def real_cb(self, msg):
        if msg.name and msg.position:
            with self.lock:
                self.real_joint_map = dict(zip(msg.name, msg.position))

    def odom_cb(self, msg):
        """处理底盘里程计，提取 x, y 并通过四元数计算 yaw"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # 四元数转 Yaw 角公式
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        with self.lock:
            self.base_state_pose = [float(pos.x), float(pos.y), float(yaw)]

    def cmd_vel_cb(self, msg):
        """拦截键盘发出的速度控制指令"""
        with self.lock:
            self.base_action_vel = [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]

    def get_data(self):
        with self.lock:
            return (
                list(self.left_joint_cmds), 
                list(self.right_joint_cmds), 
                dict(self.real_joint_map),
                list(self.base_action_vel),
                list(self.base_state_pose)
            )


def run_isaac_recording(args):
    init_logging()
    logging.info("======= 正在初始化 [双臂+脊柱+全向移动底盘] 32维全状态录制系统 =======")
    
    camera_configs = {
        "head_camera": ROS2CameraConfig(topic_name="/rgb_head", node_name="lerobot_head_camera", width=args.width, height=args.height, fps=args.fps, encoding="bgr8"),
        "left_wrist_camera": ROS2CameraConfig(topic_name="/rgb_left", node_name="lerobot_left_camera", width=args.width, height=args.height, fps=args.fps, encoding="bgr8"),
        "right_wrist_camera": ROS2CameraConfig(topic_name="/rgb_right", node_name="lerobot_right_camera", width=args.width, height=args.height, fps=args.fps, encoding="bgr8")
    }

    interface_cfg = ROS2RobotInterfaceConfig(
        joint_states_topic=args.joint_states_topic,              
        end_effector_pose_topic=args.ee_pose_topic,   
        end_effector_target_topic=args.ee_target_topic,
        right_end_effector_pose_topic="/right_current_pose",   
        right_end_effector_target_topic="/right_target",
        control_type=ControlType.CARTESIAN_POSE,
        joint_names=[
            "left_fr3v2_joint1", "left_fr3v2_joint2", "left_fr3v2_joint3", "left_fr3v2_joint4","left_fr3v2_joint5", "left_fr3v2_joint6", "left_fr3v2_joint7",
            "right_fr3v2_joint1", "right_fr3v2_joint2", "right_fr3v2_joint3", "right_fr3v2_joint4","right_fr3v2_joint5", "right_fr3v2_joint6", "right_fr3v2_joint7"
        ]
    )

    robot_config = ROS2RobotConfig(id=args.robot_id, cameras=camera_configs, ros2_interface=interface_cfg)
    robot = ROS2Robot(robot_config)
    robot.connect()

    # ============================================================================
    # 手写 32维 复合特征 Schema
    # ============================================================================
    ee_pose_names = [
        "left_ee.pos.x", "left_ee.pos.y", "left_ee.pos.z", "left_ee.quat.x", "left_ee.quat.y", "left_ee.quat.z", "left_ee.quat.w",
        "right_ee.pos.x", "right_ee.pos.y", "right_ee.pos.z", "right_ee.quat.x", "right_ee.quat.y", "right_ee.quat.z", "right_ee.quat.w"
    ]
    joint_names_list = [
        "left_fr3v2_joint1", "left_fr3v2_joint2", "left_fr3v2_joint3", "left_fr3v2_joint4", "left_fr3v2_joint5", "left_fr3v2_joint6", "left_fr3v2_joint7",
        "right_fr3v2_joint1", "right_fr3v2_joint2", "right_fr3v2_joint3", "right_fr3v2_joint4", "right_fr3v2_joint5", "right_fr3v2_joint6", "right_fr3v2_joint7"
    ]
    all_joints_list = joint_names_list + ["franka_spine_vertical_joint"]
    joint_pos_names = [f"{name}.pos" for name in all_joints_list]

    # 🚀 追加底盘维度标签名
    base_action_names = ["base.cmd_vel.vx", "base.cmd_vel.vy", "base.cmd_vel.wz"]
    base_state_names = ["base.odom.x", "base.odom.y", "base.odom.yaw"]

    # 拼装最终特征集合
    custom_action_names = ee_pose_names + joint_pos_names + base_action_names
    custom_state_names = ee_pose_names + joint_pos_names + base_state_names

    custom_dataset_features = {
        "action": {"dtype": "float32", "shape": (32,), "names": custom_action_names},
        "observation.state": {"dtype": "float32", "shape": (32,), "names": custom_state_names},
        "observation.images.head_camera": {"dtype": "video", "shape": [args.height, args.width, 3], "names": ["height", "width", "channels"]},
        "observation.images.left_wrist_camera": {"dtype": "video", "shape": [args.height, args.width, 3], "names": ["height", "width", "channels"]},
        "observation.images.right_wrist_camera": {"dtype": "video", "shape": [args.height, args.width, 3], "names": ["height", "width", "channels"]}
    }

    # 创建数据仓库
    dataset_path, versioned_repo_name = get_next_dataset_version_path(args.output_dir, args.repo_name)
    dataset_repo_id = f"paul/{versioned_repo_name}"
    logging.info("本次录制数据集路径: %s", dataset_path)
    dataset = LeRobotDataset.create(
        repo_id=dataset_repo_id, fps=args.fps, root=dataset_path, robot_type=args.robot_id,
        features=custom_dataset_features, use_videos=True, image_writer_processes=0, image_writer_threads=4,
    )

    # 初始化大一统拦截器
    interceptor = JointDataInterceptor(robot.ros2_interface.robot_node)

    print("\n" + "="*60)
    print(" 💡 [32维 全状态整体录制就绪]：")
    print("  - 键盘输入 [1] : 复位缓冲区")
    print("  - 键盘输入 [2] : 开始录制")
    print("  - 键盘输入 [3] : 保存落盘")
    print("  - 键盘输入 [4] : 可视化本次数据集")
    print("="*60 + "\n")

    recorded_episodes = 0
    is_recording = False
    user_cmd = None
    visualization_process = None
    
    try:
        while recorded_episodes < args.max_episodes:
            loop_start_t = time.perf_counter()
            
            if not is_recording:
                print(f"\n[当前已保存 {recorded_episodes} 个 Episode] 输入指令 (1-复位, 2-开始录制, 4-可视化, q-退出): ", end="", flush=True)
                user_cmd = sys.stdin.readline().strip()
                if user_cmd == '1':
                    dataset.clear_episode_buffer()
                    continue
                elif user_cmd == '2':
                    is_recording = True
                    frame_count = 0
                elif user_cmd == '4':
                    if recorded_episodes == 0:
                        print("⚠️ 当前数据集还没有已保存的 Episode，请先录制并按 3 保存后再可视化。")
                    elif visualization_process and visualization_process.poll() is None:
                        print("ℹ️ 可视化程序已经在运行中。")
                    else:
                        print(f"👀 正在启动可视化: repo_id={dataset_repo_id}, root={dataset_path}, episode={args.visualize_episode_index}")
                        visualization_process = launch_dataset_visualization(dataset_repo_id, dataset_path, args.visualize_episode_index)
                elif user_cmd.lower() == 'q':
                    break
            
            if is_recording:
                import rclpy
                rclpy.spin_once(robot.ros2_interface.robot_node, timeout_sec=0.001)

                raw_obs = robot.get_observation()
                left_pose = robot.ros2_interface.get_end_effector_pose()
                right_pose = robot.ros2_interface.get_right_end_effector_pose()
                
                # 🚀 捞取这一帧所有的传感器和控制器拦截数据
                l_cmds, r_cmds, real_map, b_actions, b_states = interceptor.get_data()

                # 1. 组装 Action
                action_data = [
                    left_pose.position.x if left_pose else 0.0, left_pose.position.y if left_pose else 0.0, left_pose.position.z if left_pose else 0.0,
                    left_pose.orientation.x if left_pose else 0.0, left_pose.orientation.y if left_pose else 0.0, left_pose.orientation.z if left_pose else 0.0, left_pose.orientation.w if left_pose else 1.0,
                    right_pose.position.x if right_pose else 0.0, right_pose.position.y if right_pose else 0.0, right_pose.position.z if right_pose else 0.0,
                    right_pose.orientation.x if right_pose else 0.0, right_pose.orientation.y if right_pose else 0.0, right_pose.orientation.z if right_pose else 0.0, right_pose.orientation.w if right_pose else 1.0,
                ]
                action_data.extend(l_cmds)
                action_data.extend(r_cmds)
                action_data.append(float(real_map.get("franka_spine_vertical_joint", 0.0)))
                action_data.extend(b_actions)  # 🚀 追加底盘速度指令 3 维 [vx, vy, wz]
                action_tensor = np.array(action_data, dtype=np.float32)

                # 2. 组装 Observation.state
                state_data = [
                    left_pose.position.x if left_pose else 0.0, left_pose.position.y if left_pose else 0.0, left_pose.position.z if left_pose else 0.0,
                    left_pose.orientation.x if left_pose else 0.0, left_pose.orientation.y if left_pose else 0.0, left_pose.orientation.z if left_pose else 0.0, left_pose.orientation.w if left_pose else 1.0,
                    right_pose.position.x if right_pose else 0.0, right_pose.position.y if right_pose else 0.0, right_pose.position.z if right_pose else 0.0,
                    right_pose.orientation.x if right_pose else 0.0, right_pose.orientation.y if right_pose else 0.0, right_pose.orientation.z if right_pose else 0.0, right_pose.orientation.w if right_pose else 1.0,
                ]
                for name in all_joints_list:
                    state_data.append(float(real_map.get(name, 0.0)))
                state_data.extend(b_states)   # 🚀 追加底盘里程计状态 3 维 [x, y, yaw]
                state_tensor = np.array(state_data, dtype=np.float32)

                total_frame = {
                    "action": action_tensor,
                    "observation.state": state_tensor,
                    "task": args.single_task
                }
                
                for cam_name in ["head_camera", "left_wrist_camera", "right_wrist_camera"]:
                    total_frame[f"observation.images.{cam_name}"] = raw_obs[cam_name]

                dataset.add_frame(total_frame)
                frame_count += 1
                
                if frame_count % args.fps == 0:
                    print(f"   录制中... 当前已采集 {frame_count} 帧...", end="\r")
                
                if frame_count >= int(args.fps * args.max_episode_time_s):
                    user_cmd = '3'
                else:
                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        if sys.stdin.readline().strip() == '3':
                            user_cmd = '3'
                
                if user_cmd == '3':
                    print(f"\n💾 正在对第 {recorded_episodes + 1} 个 Episode 进行持久化落盘...")
                    if dataset.episode_buffer and dataset.episode_buffer.get('size', 0) > 0:
                        dataset.save_episode()
                        recorded_episodes += 1
                        print(f"✔ 成功保存 Episode {recorded_episodes} ！（共计 {frame_count} 帧）")
                    is_recording = False
                    user_cmd = None
                
                dt_s = time.perf_counter() - loop_start_t
                busy_wait(1 / args.fps - dt_s)

    except KeyboardInterrupt:
        logging.warning("手动安全终止。")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Isaac Sim Action Graph Mobile Dual-Arm Unified Recorder")
    parser.add_argument("--repo_name", type=str, default="isaac_action_graph_dataset")
    parser.add_argument("--single_task", type=str, default="Navigate mobile base and use dual arms to interact")
    parser.add_argument("--output_dir", type=str, default="dataset/")
    parser.add_argument("--robot_id", type=str, default="mobile_fr3_duo_isaac")
    parser.add_argument("--joint_states_topic", type=str, default="/joint_states")
    parser.add_argument("--ee_pose_topic", type=str, default="/left_current_pose")
    parser.add_argument("--ee_target_topic", type=str, default="/joint_command")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=512)
    parser.add_argument("--height", type=int, default=512)
    parser.add_argument("--max_episode_time_s", type=float, default=45.0)
    parser.add_argument("--max_episodes", type=int, default=50)
    parser.add_argument("--visualize_episode_index", type=int, default=0)
    
    args = parser.parse_args()
    run_isaac_recording(args)
