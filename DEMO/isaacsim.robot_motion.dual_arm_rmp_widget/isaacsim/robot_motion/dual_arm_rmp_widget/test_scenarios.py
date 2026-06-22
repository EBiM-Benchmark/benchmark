# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import numpy as np
from isaacsim.core.api.objects.cone import VisualCone
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.core.api.objects.cylinder import VisualCylinder
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.numpy import rot_matrices_to_quats
from isaacsim.core.utils.prims import delete_prim, is_prim_path_valid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.motion_policies import RmpFlow
from isaacsim.robot_motion.motion_generation.motion_policy_controller import MotionPolicyController# 在文件顶部添加：
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import omni.timeline


from .controllers import KinematicsController


class LulaTestScenarios:
    def __init__(self):
        self._target_left = None
        self._target_right = None
        self._obstacles = []

        self._controller = None
        # 定义双臂解耦控制所需的私有控制器对象
        self._controller_left = None
        self._controller_right = None

        # ====== 新增：初始化 ROS 2 ======
        if not rclpy.ok():
            rclpy.init()
        self._ros_node = Node("isaac_target_pose_publisher")
        
        # 创建左臂和右臂的发布者，使用标准 PoseStamped 消息
        self._left_pub = self._ros_node.create_publisher(PoseStamped, "/left_target", 10)
        self._right_pub = self._ros_node.create_publisher(PoseStamped, "/right_target", 10)
        # ====== 在 __init__ 中新增：当前位姿发布者 ======
        self._left_current_pub = self._ros_node.create_publisher(PoseStamped, "/left_current_pose", 10)
        self._right_current_pub = self._ros_node.create_publisher(PoseStamped, "/right_current_pose", 10)

        self.timestep = 0
        self.use_orientation = True
        self.scenario_name = ""
        self.rmpflow_debug_mode = False

        self.rmpflow_left = None
        self.rmpflow_right = None

        self.art_ik_left = None
        self.art_ik_right = None
        self._ee_frame_prim_left = None
        self._ee_frame_prim_right = None

    def get_yaml_frame_names(self, robot_description_path, urdf_path):
        """辅助函数：用于为 UI 渲染提供特定描述文件下的坐标系名称"""
        try:
            temp_solver = LulaKinematicsSolver(robot_description_path, urdf_path)
            return temp_solver.get_all_frame_names()
        except Exception as e:
            carb.log_error(f"Failed to load frame names from yaml: {str(e)}")
            return []

    def visualize_ee_frame(self, articulation, ee_frame_left, ee_frame_right, robot_description_path_left, urdf_path=None):
        if articulation is None or not robot_description_path_left or not urdf_path:
            return
        self.stop_visualize_ee_frame()

        try:
            # 严格使用安全传入的实参绝对路径
            temp_ik = LulaKinematicsSolver(robot_description_path_left, urdf_path)
            
            self.art_ik_left = ArticulationKinematicsSolver(articulation, temp_ik, ee_frame_left)
            pos_l, rot_l = self.art_ik_left.compute_end_effector_pose()
            self._ee_frame_prim_left = self._create_frame_prim(pos_l, rot_matrices_to_quats(rot_l), "/Lula/end_effector_left")

            self.art_ik_right = ArticulationKinematicsSolver(articulation, temp_ik, ee_frame_right)
            pos_r, rot_r = self.art_ik_right.compute_end_effector_pose()
            self._ee_frame_prim_right = self._create_frame_prim(pos_r, rot_matrices_to_quats(rot_r), "/Lula/end_effector_right")
        except Exception as e:
            carb.log_warn(f"EE Visualization failed: {e}")

    def stop_visualize_ee_frame(self):
        if self._ee_frame_prim_left is not None:
            delete_prim(self._ee_frame_prim_left.prim_path)
        if self._ee_frame_prim_right is not None:
            delete_prim(self._ee_frame_prim_right.prim_path)
        self._ee_frame_prim_left = None
        self._ee_frame_prim_right = None

    def toggle_rmpflow_debug_mode(self):
        self.rmpflow_debug_mode = not self.rmpflow_debug_mode
        if self.rmpflow_left is None or self.rmpflow_right is None:
            return

        if self.rmpflow_debug_mode:
            self.rmpflow_left.set_ignore_state_updates(True)
            self.rmpflow_left.visualize_collision_spheres()
            self.rmpflow_right.set_ignore_state_updates(True)
            self.rmpflow_right.visualize_collision_spheres()
        else:
            self.rmpflow_left.set_ignore_state_updates(False)
            self.rmpflow_left.stop_visualizing_collision_spheres()
            self.rmpflow_right.set_ignore_state_updates(False)
            self.rmpflow_right.stop_visualizing_collision_spheres()

    def on_ik_follow_target_dual(self, articulation, ee_frame_left, ee_frame_right, desc_path_left, desc_path_right, urdf_path):
        self.scenario_reset()
        self.scenario_name = "IK_Dual"
        
        # 针对双核心提供相互拆分的底层 LulaIK 解算支撑
        ik_solver_left = LulaKinematicsSolver(desc_path_left, urdf_path)
        ik_solver_right = LulaKinematicsSolver(desc_path_right, urdf_path)
        
        self.art_ik_left = ArticulationKinematicsSolver(articulation, ik_solver_left, ee_frame_left)
        self.art_ik_right = ArticulationKinematicsSolver(articulation, ik_solver_right, ee_frame_right)
        
        self._controller = KinematicsController("Lula Dual Arm Kinematics Controller", self.art_ik_left, self.art_ik_right)
        self._create_dual_targets()

    def on_rmpflow_follow_target_obstacles_dual(self, articulation, **rmp_config):
        """符合 Isaac Sim 官方多策略控制体系的标准双臂 RmpFlow 挂载"""
        self.scenario_reset()
        self.scenario_name = "RmpFlow_Dual"

        ee_frame_left = rmp_config.get("end_effector_frame_name_left")
        ee_frame_right = rmp_config.get("end_effector_frame_name_right")

        # 实例化左臂、右臂的原生 Lula 核心
        self.rmpflow_left = RmpFlow(
            robot_description_path=rmp_config.get("robot_description_path_left"),
            urdf_path=rmp_config.get("urdf_path"),
            rmpflow_config_path=rmp_config.get("rmpflow_config_path"),
            end_effector_frame_name=ee_frame_left,
            maximum_substep_size=rmp_config.get("maximum_substep_size", 0.0034)
        )

        self.rmpflow_right = RmpFlow(
            robot_description_path=rmp_config.get("robot_description_path_right"),
            urdf_path=rmp_config.get("urdf_path"),
            rmpflow_config_path=rmp_config.get("rmpflow_config_path"),
            end_effector_frame_name=ee_frame_right,
            maximum_substep_size=rmp_config.get("maximum_substep_size", 0.0034)
        )

        base_pos, base_rot = articulation.get_world_pose()
        self.rmpflow_left.set_robot_base_pose(base_pos, base_rot)
        self.rmpflow_right.set_robot_base_pose(base_pos, base_rot)

        if self.rmpflow_debug_mode:
            self.rmpflow_left.set_ignore_state_updates(True)
            self.rmpflow_left.visualize_collision_spheres()
            self.rmpflow_right.set_ignore_state_updates(True)
            self.rmpflow_right.visualize_collision_spheres()

        # 🌟 规范重构：直接调用你在 controllers.py 里面写的复合控制器
        # 将原生两套 rmpflow 丢给复合控制器统一管理，彻底移除原先零散的 _controller_left/right 临时中间件
        from .controllers import DualRmpFlowController
        self._controller = DualRmpFlowController(
            name="Lula_Dual_Arm_RmpFlow_Controller",
            rmpflow_left=self.rmpflow_left,
            rmpflow_right=self.rmpflow_right,
            articulation=articulation
        )

        self._create_dual_targets()
        self._create_wall()
        for obstacle in self._obstacles:
            self.rmpflow_left.add_obstacle(obstacle)
            self.rmpflow_right.add_obstacle(obstacle)

    def _create_dual_targets(self):
        self._target_left = VisualCuboid(
            "/World/Target_Left", size=0.05, position=np.array([0.8, 0.5, 1.0]), 
            orientation=np.array([0, -1, 0, 0]), color=np.array([1.0, 0.0, 0.0])
        )
        self._target_right = VisualCuboid(
            "/World/Target_Right", size=0.05, position=np.array([0.8, -0.5, 1.0]), 
            orientation=np.array([0, -1, 0, 0]), color=np.array([0.0, 1.0, 0.0])
        )

    def _create_frame_prim(self, position, orientation, parent_prim_path):
        frame_xform = SingleXFormPrim(parent_prim_path, position=position, orientation=orientation)
        line_len, line_width, cone_radius, cone_len = 0.04, 0.004, 0.01, 0.02

        VisualCylinder(parent_prim_path + "/X_line", translation=np.array([line_len/2, 0, 0]), orientation=euler_angles_to_quat([0, np.pi/2, 0]), color=np.array([1, 0, 0]), height=line_len, radius=line_width)
        VisualCone(parent_prim_path + "/X_tip", translation=np.array([line_len + cone_len/2, 0, 0]), orientation=euler_angles_to_quat([0, np.pi/2, 0]), color=np.array([1, 0, 0]), height=cone_len, radius=cone_radius)
        VisualCylinder(parent_prim_path + "/Y_line", translation=np.array([0, line_len/2, 0]), orientation=euler_angles_to_quat([-np.pi/2, 0, 0]), color=np.array([0, 1, 0]), height=line_len, radius=line_width)
        VisualCone(parent_prim_path + "/Y_tip", translation=np.array([0, line_len + cone_len/2, 0]), orientation=euler_angles_to_quat([-np.pi/2, 0, 0]), color=np.array([0, 1, 0]), height=cone_len, radius=cone_radius)
        VisualCylinder(parent_prim_path + "/Z_line", translation=np.array([0, 0, line_len/2]), orientation=euler_angles_to_quat([0, 0, 0]), color=np.array([0, 0, 1]), height=line_len, radius=line_width)
        VisualCone(parent_prim_path + "/Z_tip", translation=np.array([0, 0, line_len + cone_len/2]), orientation=euler_angles_to_quat([0, 0, 0]), color=np.array([0, 0, 1]), height=cone_len, radius=cone_radius)
        return frame_xform

    def _create_wall(self, position=None, orientation=None):
        cube_prim_path = find_unique_string_name(initial_name="/World/WallObstacle", is_unique_fn=lambda x: not is_prim_path_valid(x))
        if position is None:
            position = np.array([0.65, 0.0, 0.30])
        if orientation is None:
            orientation = euler_angles_to_quat(np.array([0, 0, np.pi / 2]))
        cube = VisualCuboid(prim_path=cube_prim_path, position=position, orientation=orientation, size=1.0, scale=np.array([0.05, 0.35, 0.5]), color=np.array([0, 0, 1.0]))
        self._obstacles.append(cube)

    def set_use_orientation(self, use_orientation):
        self.use_orientation = use_orientation

    def full_reset(self):
        self.scenario_reset()

    def scenario_reset(self):
        if self._target_left is not None:
            delete_prim(self._target_left.prim_path)
        if self._target_right is not None:
            delete_prim(self._target_right.prim_path)
        if self.rmpflow_left is not None:
            self.rmpflow_left.stop_visualizing_collision_spheres()
        if self.rmpflow_right is not None:
            self.rmpflow_right.stop_visualizing_collision_spheres()

        for obstacle in self._obstacles:
            delete_prim(obstacle.prim_path)
            
        self.rmpflow_left = None
        self.rmpflow_right = None
        self._target_left = None
        self._target_right = None
        self._obstacles = []
        self._controller = None
        self._controller_left = None
        self._controller_right = None
        self.scenario_name = ""

    def get_next_action(self, **scenario_params):
        if self._controller is None:
            return ArticulationAction()

        if self.scenario_name == "IK_Dual":
            if self._target_left is not None and self._target_right is not None:
                pos_left, rot_left = self._target_left.get_local_pose()
                pos_right, rot_right = self._target_right.get_local_pose()
                # ====== 新增：在每次 IK 更新时发布到 ROS 2 ======
                # 如果是 local_pose，frame_id 应该设为你的机器人 base_link 名称
                self._publish_target_pose(self._left_pub, pos_left, rot_left, frame_id="base_link")
                self._publish_target_pose(self._right_pub, pos_right, rot_right, frame_id="base_link")

                # ====== 新增：获取并发布实际当前位姿 (Current Pose) ======
                # 从我们组合的控制器中拿到左臂和右臂的底层 Lula Kinematics Solver 实例
                art_solver_l = self._controller._art_kinematics_left
                art_solver_r = self._controller._art_kinematics_right
                
                if art_solver_l and art_solver_r:
                    # 这个官方 API 会自动读取当前物理世界机器人的状态并计算末端位姿
                    # 返回的格式： position (numpy 数组), rotation (3x3 旋转矩阵)
                    curr_pos_l, curr_rot_mat_l = art_solver_l.compute_end_effector_pose()
                    curr_pos_r, curr_rot_mat_r = art_solver_r.compute_end_effector_pose()
                    
                    # 转换为 [w, x, y, z] 四元数
                    curr_quat_l = rot_matrices_to_quats(curr_rot_mat_l)
                    curr_quat_r = rot_matrices_to_quats(curr_rot_mat_r)
                    
                    # 发布当前实际位姿话题
                    self._publish_target_pose(self._left_current_pub, curr_pos_l, curr_quat_l, frame_id="base_link")
                    self._publish_target_pose(self._right_current_pub, curr_pos_r, curr_quat_r, frame_id="base_link")
                # ========================================================
                
                # 让 ROS 2 节点处理回调（非阻塞）
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)
                # ===============================================
                if not self.use_orientation:
                    rot_left, rot_right = None, None
                return self._controller.forward(pos_left, pos_right, rot_left, rot_right)
            return ArticulationAction()

        # 🌟 规范重构：RmpFlow 状态下的数据流极其优雅、直观
        if self.scenario_name == "RmpFlow_Dual":
            if self._target_left is None or self._target_right is None:
                return ArticulationAction()

            pos_left, rot_left = self._target_left.get_world_pose()
            pos_right, rot_right = self._target_right.get_world_pose()
            
            if not self.use_orientation:
                rot_left, rot_right = None, None

            # 统一交由规范控制器的 forward 进行双向解算、自动状态同步与合并映射
            return self._controller.forward(pos_left, pos_right, rot_left, rot_right)
        
        return ArticulationAction()

    def _publish_target_pose(self, publisher, pos, rot, frame_id="world"):
        """将位置和四元数发布到指定的 ROS 2 topic"""
        msg = PoseStamped()
        # 填充时间戳（使用仿真当前时间或系统时间）
        msg.header.stamp = self._ros_node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Lula 内部单位如果是米，可以直接赋给 ROS 2
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        
        # Isaac Sim 的四元数顺序通常是 [w, x, y, z]，而 ROS 2 是 [x, y, z, w]
        # 请根据你具体 Lula 输出的四元数格式检查，以下假设输入是 Isaac 标准的 [w, x, y, z]
        if rot is not None and len(rot) == 4:
            msg.pose.orientation.w = float(rot[0])
            msg.pose.orientation.x = float(rot[1])
            msg.pose.orientation.y = float(rot[2])
            msg.pose.orientation.z = float(rot[3])
        else:
            msg.pose.orientation.w = 1.0  # 默认无旋转
            
        publisher.publish(msg)