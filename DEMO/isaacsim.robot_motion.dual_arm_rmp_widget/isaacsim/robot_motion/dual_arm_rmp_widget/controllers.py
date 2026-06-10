# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Optional

import carb
import numpy as np
from isaacsim.core.api import objects
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_trajectory import ArticulationTrajectory
from isaacsim.robot_motion.motion_generation.path_planner_visualizer import PathPlannerVisualizer


class LulaController(BaseController):
    def __init__(self, name: str):
        BaseController.__init__(self, name)

    def forward(self, *args, **kwargs) -> ArticulationAction:
        return ArticulationAction()


class KinematicsController(LulaController):
    def __init__(self, name: str, art_kinematics_left: ArticulationKinematicsSolver, art_kinematics_right: ArticulationKinematicsSolver):
        BaseController.__init__(self, name)
        self._art_kinematics_left = art_kinematics_left
        self._art_kinematics_right = art_kinematics_right

    def forward(
        self, 
        target_pos_left: np.ndarray, 
        target_pos_right: np.ndarray,
        target_rot_left: Optional[np.ndarray] = None,
        target_rot_right: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        
        # 1. 计算左臂 IK
        action_left, succ_left = self._art_kinematics_left.compute_inverse_kinematics(
            target_pos_left, target_rot_left
        )
        
        # 2. 计算右臂 IK
        action_right, succ_right = self._art_kinematics_right.compute_inverse_kinematics(
            target_pos_right, target_rot_right
        )

        if not succ_left:
            carb.log_warn("Failed to compute Left Arm Inverse Kinematics")
        if not succ_right:
            carb.log_warn("Failed to compute Right Arm Inverse Kinematics")

        # 3. 合并两者的 ArticulationAction
        # 提取两者的 joint_positions 并通过 joint_indices 正确映射
        combined_positions = {}
        
        if succ_left and action_left.joint_positions is not None:
            for idx, pos in zip(action_left.joint_indices, action_left.joint_positions):
                combined_positions[idx] = pos
                
        if succ_right and action_right.joint_positions is not None:
            for idx, pos in zip(action_right.joint_indices, action_right.joint_positions):
                combined_positions[idx] = pos

        if len(combined_positions) == 0:
            return ArticulationAction()

        indices = np.array(list(combined_positions.keys()))
        positions = np.array(list(combined_positions.values()))

        return ArticulationAction(joint_positions=positions, joint_indices=indices)

class TrajectoryController(LulaController):
    def __init__(self, name: str, art_trajectory: ArticulationTrajectory):
        BaseController.__init__(self, name)
        self._art_trajectory = art_trajectory
        self._actions = self._art_trajectory.get_action_sequence(1 / 60)
        self._action_index = 0

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ):
        if self._action_index == 0:
            first_action = self._actions[0]
            desired_joint_positions = first_action.joint_positions

            robot_articulation = self._art_trajectory.get_robot_articulation()
            current_joint_positions = robot_articulation.get_joint_positions()

            is_none_mask = desired_joint_positions == None
            desired_joint_positions[is_none_mask] = current_joint_positions[is_none_mask]

            robot_articulation.set_joint_positions(desired_joint_positions)
            action = first_action
        elif self._action_index >= len(self._actions):
            return ArticulationAction(
                self._actions[-1].joint_positions,
                np.zeros_like(self._actions[-1].joint_velocities),
                self._actions[-1].joint_indices,
            )
        else:
            action = self._actions[self._action_index]

        self._action_index += 1
        return action


class PathPlannerController(LulaController):
    def __init__(
        self,
        name: str,
        path_planner_visualizer: PathPlannerVisualizer,
        cspace_interpolation_max_dist: float = 0.5,
        frames_per_waypoint: int = 30,
    ):
        BaseController.__init__(self, name)

        self._path_planner_visualizer = path_planner_visualizer
        self._path_planner = path_planner_visualizer.get_path_planner()

        self._cspace_interpolation_max_dist = cspace_interpolation_max_dist
        self._frames_per_waypoint = frames_per_waypoint

        self._plan = None

        self._frame_counter = 1

    def make_new_plan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> None:
        self._path_planner.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)
        self._path_planner.update_world()
        self._plan = self._path_planner_visualizer.compute_plan_as_articulation_actions(
            max_cspace_dist=self._cspace_interpolation_max_dist
        )
        if self._plan is None or self._plan == []:
            carb.log_warn("No plan could be generated to target pose: " + str(target_end_effector_position))

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        if self._plan is None:
            # This will only happen the first time the forward function is used
            self.make_new_plan(target_end_effector_position, target_end_effector_orientation)

        if len(self._plan) == 0:
            # The plan is completed; return null action to remain in place
            self._frame_counter = 1
            return ArticulationAction()

        if self._frame_counter % self._frames_per_waypoint != 0:
            # Stop at each waypoint in the plan for self._frames_per_waypoint frames
            self._frame_counter += 1
            return self._plan[0]
        else:
            self._frame_counter += 1
            return self._plan.pop(0)

    def add_obstacle(self, obstacle: objects, static: bool = False) -> None:
        self._path_planner.add_obstacle(obstacle, static)

    def remove_obstacle(self, obstacle: objects) -> None:
        self._path_planner.remove_obstacle(obstacle)

    def reset(self) -> None:
        # PathPlannerController will make one plan per reset
        self._path_planner.reset()
        self._plan = None
        self._frame_counter = 1

from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy

class DualRmpFlowController(LulaController):
    def __init__(self, name: str, rmpflow_left, rmpflow_right, articulation):
        BaseController.__init__(self, name)
        
        # 关键修改：用 Isaac Sim 官方的 ArticulationMotionPolicy 包装原生 RmpFlow
        # 1/60 对应你的物理仿真步长
        self._art_rmp_left = ArticulationMotionPolicy(articulation, rmpflow_left, 1 / 60)
        self._art_rmp_right = ArticulationMotionPolicy(articulation, rmpflow_right, 1 / 60)

    def forward(
        self, 
        target_pos_left: np.ndarray, 
        target_pos_right: np.ndarray,
        target_rot_left: Optional[np.ndarray] = None,
        target_rot_right: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        
        # 1. 设定左右手各自的目标 (通过包装器拿到里面的底层的 motion policy 设定目标)
        if target_pos_left is not None:
            self._art_rmp_left.get_motion_policy().set_end_effector_target(target_pos_left, target_rot_left)
        if target_pos_right is not None:
            self._art_rmp_right.get_motion_policy().set_end_effector_target(target_pos_right, target_rot_right)
        
        # 2. 调用包装器的 update()。它会自动获取机器人当前的最新关节状态(position/velocity)同步给底层
        self._art_rmp_left.update()
        self._art_rmp_right.update()
        
        # 3. 计算出各自手臂产生的 ArticulationAction 指令
        action_left = self._art_rmp_left.get_next_articulation_action()
        action_right = self._art_rmp_right.get_next_articulation_action()
        
        # 4. 融合两者的指令 (完全对齐你上面 KinematicsController 的双臂映射逻辑)
        combined_positions = {}
        combined_velocities = {}
        
        if action_left and action_left.joint_positions is not None:
            for idx, pos, vel in zip(action_left.joint_indices, action_left.joint_positions, action_left.joint_velocities):
                combined_positions[idx] = pos
                combined_velocities[idx] = vel
                
        if action_right and action_right.joint_positions is not None:
            for idx, pos, vel in zip(action_right.joint_indices, action_right.joint_positions, action_right.joint_velocities):
                combined_positions[idx] = pos
                combined_velocities[idx] = vel

        if len(combined_positions) == 0:
            return ArticulationAction()

        indices = np.array(list(combined_positions.keys()))
        positions = np.array(list(combined_positions.values()))
        velocities = np.array(list(combined_velocities.values()))

        # 返回融合后的双臂控制 action
        return ArticulationAction(
            joint_positions=positions, 
            joint_velocities=velocities, 
            joint_indices=indices
        )
