# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import asyncio
import gc
import os
import weakref

import carb
import numpy as np
import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import get_prim_object_type
from isaacsim.gui.components.menu import make_menu_item_description
from isaacsim.gui.components.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    float_builder,
    get_style,
    setup_ui_headers,
    state_btn_builder,
    str_builder,
)
from isaacsim.gui.components.widgets import DynamicComboBoxModel
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.window.extensions import SimpleCheckBox
from omni.kit.window.property.templates import LABEL_WIDTH
from pxr import Usd

from .test_scenarios import LulaTestScenarios

EXTENSION_NAME = "Dual Arm RmpFlow Dashboard"


def is_yaml_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext in [".yaml", ".YAML"]


def is_urdf_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext in [".urdf", ".URDF"]


def on_filter_yaml_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_yaml_file(item.path)


def on_filter_urdf_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_urdf_file(item.path)


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.get_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

        self._window = ui.Window(
            title=EXTENSION_NAME, width=600, height=550, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        self._models = {}
        self._ext_id = ext_id
        menu_entry = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription("Robotics", sub_menu=menu_entry)]
        add_menu_items(self._menu_items, "Tools")

        self._new_window = True
        self.new_selection = True
        self._selected_index = None
        self._selected_prim_path = None
        self._prev_art_prim_path = None

        self.articulation = None
        self.num_dof = 0

        # 修改：拆分为独立的左右描述文件路径变量
        self._selected_robot_description_file_left = None
        self._selected_robot_description_file_right = None
        self._selected_robot_urdf_file = None
        
        self._robot_description_file_left = None
        self._robot_description_file_right = None
        self._robot_urdf_file = None

        self._ee_frame_options_left = []
        self._ee_frame_options_right = []

        self._rmpflow_config_yaml = None
        self._test_scenarios = LulaTestScenarios()
        self._visualize_end_effector = True

    def on_shutdown(self):
        self._test_scenarios.full_reset()
        self.articulation = None
        self._usd_context = None
        self._stage_event_sub = None
        self._timeline_event_sub = None
        self._physx_subscription = None
        self._models = {}
        remove_menu_items(self._menu_items, "Tools")
        if self._window:
            self._window = None
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
            if not self._new_window and self.articulation:
                self._refresh_ui(self.articulation)
            self._new_window = False
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        if self._timeline.is_playing():
            self._refresh_selection_combobox()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_info_ui()
                self._build_selection_ui()
                self._build_kinematics_ui()
                self._build_rmpflow_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()
            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window
            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
        self._task = asyncio.ensure_future(dock_window())

    def _on_selection(self, prim_path):
        if prim_path == self._prev_art_prim_path:
            return
        else:
            self._prev_art_prim_path = prim_path

        self.new_selection = True
        if self.articulation_list and prim_path != "None":
            self.articulation = SingleArticulation(prim_path)
            if not self.articulation.handles_initialized:
                self.articulation.initialize()
            self._refresh_ui(self.articulation)
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
        else:
            if self.articulation is not None:
                self._reset_ui()
                self._refresh_selection_combobox()
            self.articulation = None

    def _on_combobox_selection(self, model=None, val=None):
        index = self._models["ar_selection_model"].get_item_value_model().as_int
        if index >= 0 and index < len(self.articulation_list):
            self._selected_index = index
            item = self.articulation_list[index]
            self._selected_prim_path = item
            self._on_selection(item)

    def _refresh_selection_combobox(self):
        self.articulation_list = self.get_all_articulations()
        if self._prev_art_prim_path is not None and self._prev_art_prim_path not in self.articulation_list:
            self._reset_ui()
        self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
        self._models["ar_selection_combobox"].model = self._models["ar_selection_model"]
        self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)
        if self._selected_index is not None and self._selected_prim_path is not None:
            if self._selected_prim_path in self.articulation_list:
                self._models["ar_selection_combobox"].model.set_item_value_model(
                    ui.SimpleIntModel(self._selected_index)
                )

    def _clear_selection_combobox(self):
        self._selected_index = None
        self._selected_prim_path = None
        self.articulation_list = []
        self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
        self._models["ar_selection_combobox"].model = self._models["ar_selection_model"]
        self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

    def get_all_articulations(self):
        articulations = ["None"]
        if self._timeline.is_stopped():
            return articulations
        stage = self._usd_context.get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                if get_prim_object_type(path) == "articulation":
                    articulations.append(path)
        return articulations

    def _refresh_ee_frame_combobox(self):
        # 针对左臂提取对应的 Frame 名称列表
        if self._robot_description_file_left is not None and self._robot_urdf_file is not None:
            frames_l = self._test_scenarios.get_yaml_frame_names(self._robot_description_file_left, self._robot_urdf_file)
        else:
            frames_l = []

        # 针对右臂提取对应的 Frame 名称列表
        if self._robot_description_file_right is not None and self._robot_urdf_file is not None:
            frames_r = self._test_scenarios.get_yaml_frame_names(self._robot_description_file_right, self._robot_urdf_file)
        else:
            frames_r = []

        self._ee_frame_options_left = frames_l
        self._ee_frame_options_right = frames_r

        # 刷新左手臂选单并模糊定位默认项
        self._models["ee_frame_left"] = DynamicComboBoxModel(frames_l)
        self._models["ee_frame_left_combobox"].model = self._models["ee_frame_left"]
        if frames_l:
            l_default = next((i for i, s in enumerate(frames_l) if "left_fr3v2_link7" in s.lower() or "left" in s.lower()), 0)
            self._models["ee_frame_left"].get_item_value_model().set_value(l_default)

        # 刷新右手臂选单并模糊定位默认项
        self._models["ee_frame_right"] = DynamicComboBoxModel(frames_r)
        self._models["ee_frame_right_combobox"].model = self._models["ee_frame_right"]
        if frames_r:
            r_default = next((i for i, s in enumerate(frames_r) if "right_fr3v2_link7" in s.lower() or "right" in s.lower()), 0)
            self._models["ee_frame_right"].get_item_value_model().set_value(r_default)

        self._models["ee_frame_left"].add_item_changed_fn(self._reset_scenario)
        self._models["ee_frame_right"].add_item_changed_fn(self._reset_scenario)

    def _reset_scenario(self, model=None, value=None):
        self._enable_lula_dropdowns()
        if self.articulation is not None:
            self.articulation.post_reset()

    def _refresh_ui(self, articulation):
        if self.new_selection:
            self.num_dof = articulation.num_dof
            self.new_selection = False
        self._enable_load_button()

    def _reset_ui(self):
        self._clear_selection_combobox()
        self._disable_lula_dropdowns()
        self._test_scenarios.full_reset()
        self._prev_art_prim_path = None
        self._visualize_end_effector = True

    def _on_stage_event(self, event):
        self._refresh_selection_combobox()
        if event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):
            self._refresh_selection_combobox()
            index = self._models["ar_selection_model"].get_item_value_model().as_int
            selected_articulation = self.articulation_list[index]
            self._on_selection(selected_articulation)
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):
            if self._timeline.is_stopped():
                self._on_selection("None")

    def _on_physics_step(self, step):
        if self.articulation is not None:
            if not self.articulation.handles_initialized:
                self.articulation.initialize()
            action = self._test_scenarios.get_next_action()
            self.articulation.get_articulation_controller().apply_action(action)

    def _on_timeline_event(self, e):
        pass

    def _build_info_ui(self):
        title = EXTENSION_NAME
        doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/index.html"
        overview = "Dual-Arm RmpFlow control plugin dashboard. Supports standalone left/right arm description configuration."
        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_selection_ui(self):
        frame = ui.CollapsableFrame(title="Selection Panel", height=0, collapsed=False, style=get_style())
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.articulation_list = []
                self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
                with ui.HStack():
                    ui.Label("Select Articulation", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
                    self._models["ar_selection_combobox"] = ui.ComboBox(self._models["ar_selection_model"])
                    add_line_rect_flourish(False)
                self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

                # ================= 核心修改：分离左、右 Robot Description 选择框 =================
                def check_left_yaml(model=None):
                    path = model.get_value_as_string()
                    if is_yaml_file(path):
                        self._selected_robot_description_file_left = path
                        self._enable_load_button()
                    else:
                        self._selected_robot_description_file_left = None

                kwargs_l = {
                    "label": "Left Robot Description YAML",
                    "default_val": "",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_yaml_item,
                    "folder_dialog_title": "Select Left Robot Description YAML",
                    "folder_button_title": "Select Left YAML",
                }
                self._models["input_robot_description_file_left"] = str_builder(**kwargs_l)
                self._models["input_robot_description_file_left"].add_value_changed_fn(check_left_yaml)

                def check_right_yaml(model=None):
                    path = model.get_value_as_string()
                    if is_yaml_file(path):
                        self._selected_robot_description_file_right = path
                        self._enable_load_button()
                    else:
                        self._selected_robot_description_file_right = None

                kwargs_r = {
                    "label": "Right Robot Description YAML",
                    "default_val": "",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_yaml_item,
                    "folder_dialog_title": "Select Right Robot Description YAML",
                    "folder_button_title": "Select Right YAML",
                }
                self._models["input_robot_description_file_right"] = str_builder(**kwargs_r)
                self._models["input_robot_description_file_right"].add_value_changed_fn(check_right_yaml)
                # =========================================================================

                def check_urdf_file_type(model=None):
                    path = model.get_value_as_string()
                    if is_urdf_file(path):
                        self._selected_robot_urdf_file = path
                        self._enable_load_button()
                    else:
                        self._selected_robot_urdf_file = None

                kwargs_u = {
                    "label": "Robot URDF",
                    "default_val": "",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_urdf_item,
                    "folder_dialog_title": "Select Robot URDF file",
                    "folder_button_title": "Select URDF",
                }
                self._models["input_robot_urdf_file"] = str_builder(**kwargs_u)
                self._models["input_robot_urdf_file"].add_value_changed_fn(check_urdf_file_type)

                def on_load_config(model=None, val=None):
                    self._robot_description_file_left = self._selected_robot_description_file_left
                    self._robot_description_file_right = self._selected_robot_description_file_right
                    self._robot_urdf_file = self._selected_robot_urdf_file
                    self._refresh_ee_frame_combobox()
                    self._enable_lula_dropdowns()

                # ================= 新增：Load Franka 快捷预设按钮 =================
                def on_load_franka_preset(model=None, val=None):
                    # 🌟 核心修改：动态获取当前文件（extension.py）所在的绝对路径
                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    
                    # 🌟 动态拼接同级目录下的 robotdescription 文件夹路径
                    robot_desc_dir = os.path.join(current_dir, "robot_description")
                    
                    # 🌟 转换为动态相对路径组合
                    left_yaml = os.path.join(robot_desc_dir, "left_arm_description.yaml")
                    right_yaml = os.path.join(robot_desc_dir, "right_arm_description.yaml")
                    robot_urdf = os.path.join(robot_desc_dir, "mobile_fr3_duo_v0_2_.urdf")
                    
                    # 修复：SimpleStringModel 直接调用 .set_value() 即可
                    if "input_robot_description_file_left" in self._models:
                        self._models["input_robot_description_file_left"].set_value(left_yaml)
                    if "input_robot_description_file_right" in self._models:
                        self._models["input_robot_description_file_right"].set_value(right_yaml)
                    if "input_robot_urdf_file" in self._models:
                        self._models["input_robot_urdf_file"].set_value(robot_urdf)
                    
                    # 自动执行一次 Load 操作
                    on_load_config()

                    target_articulation = "/World/mobile_fr3_duo_v0_2/base"
                    
                    # 如果用户在没点 Play 的情况下按了按钮，先刷新一次列表防止为空
                    if not self._timeline.is_playing():
                        carb.log_warn("[Load Franka] Timeline is not playing. Please press PLAY first to discover the Articulation node.")
                    
                    # 重新刷新当前可用的 Articulation 列表
                    self.articulation_list = self.get_all_articulations()
                    
                    if target_articulation in self.articulation_list:
                        art_idx = self.articulation_list.index(target_articulation)
                        # 更新 UI ComboBox 的选中索引
                        self._models["ar_selection_combobox"].model.set_item_value_model(
                            ui.SimpleIntModel(art_idx)
                        )
                        # 显式触发选择回调，确保后台数据同步更新
                        self._selected_index = art_idx
                        self._selected_prim_path = target_articulation
                        self._on_selection(target_articulation)
                        carb.log_info(f"[Load Franka] Successfully selected articulation: {target_articulation}")
                    else:
                        carb.log_error(f"[Load Franka] Cannot find target articulation '{target_articulation}' in current stage. Make sure timeline is PLAYING.")

                    # 4. 新增：自动在生成的下拉列表中检索并选中 left_tcp 和 right_tcp
                    if "ee_frame_left" in self._models and self._ee_frame_options_left:
                        try:
                            # 寻找完全匹配 left_tcp 的索引位置（不区分大小写）
                            l_idx = next(i for i, s in enumerate(self._ee_frame_options_left) if s.strip().lower() == "left_tcp")
                            self._models["ee_frame_left"].get_item_value_model().set_value(l_idx)
                        except StopIteration:
                            carb.log_warn("[Load Franka] 在左臂描述文件中未找到 'left_tcp' 坐标系")

                    if "ee_frame_right" in self._models and self._ee_frame_options_right:
                        try:
                            # 寻找完全匹配 right_tcp 的索引位置（不区分大小写）
                            r_idx = next(i for i, s in enumerate(self._ee_frame_options_right) if s.strip().lower() == "right_tcp")
                            self._models["ee_frame_right"].get_item_value_model().set_value(r_idx)
                        except StopIteration:
                            carb.log_warn("[Load Franka] 在右臂描述文件中未找到 'right_tcp' 坐标系")

                    # 利用 HStack 让两个常规操作并排排列
                with ui.HStack():
                    ui.Label("Load Selected Config", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
                    with ui.HStack(spacing=10):
                        self._models["load_config_btn"] = ui.Button("LOAD", name="load_btn", clicked_fn=on_load_config)
                        self._models["load_franka_btn"] = ui.Button("Load Franka", name="franka_preset_btn", clicked_fn=on_load_franka_preset)
                    add_line_rect_flourish(False)
                    # =========================================================================

                self._models["ee_frame_left"] = DynamicComboBoxModel([])
                with ui.HStack():
                    ui.Label("Left End Effector Frame", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
                    self._models["ee_frame_left_combobox"] = ui.ComboBox(self._models["ee_frame_left"])
                    add_line_rect_flourish(False)
                
                self._models["ee_frame_right"] = DynamicComboBoxModel([])
                with ui.HStack():
                    ui.Label("Right End Effector Frame", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
                    self._models["ee_frame_right_combobox"] = ui.ComboBox(self._models["ee_frame_right"])
                    add_line_rect_flourish(False)

                def on_clicked_fn(use_orientation):
                    self._test_scenarios.set_use_orientation(use_orientation)

                with ui.HStack(width=0):
                    ui.Label("Use Orientation Targets", width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_TOP)
                    cb = ui.SimpleBoolModel(default_value=1)
                    SimpleCheckBox(1, on_clicked_fn, model=cb)

                def on_vis_ee_clicked_fn(visualize_ee):
                    self._visualize_end_effector = visualize_ee
                    if visualize_ee and self.articulation:
                        ee_l, ee_r = self._get_selected_dual_ee_frames()
                        self._test_scenarios.visualize_ee_frame(self.articulation, ee_l, ee_r, self._robot_description_file_left)
                    else:
                        self._test_scenarios.stop_visualize_ee_frame()

                with ui.HStack(width=0):
                    ui.Label("Visualize End Effector Pose", width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_TOP)
                    cb = ui.SimpleBoolModel(default_value=1)
                    SimpleCheckBox(1, on_vis_ee_clicked_fn, model=cb)

    def _build_kinematics_ui(self):
        frame = ui.CollapsableFrame(title="Lula Kinematics Solver", height=0, collapsed=True, enabled=False, style=get_style())
        self._models["kinematics_frame"] = frame
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                def ik_follow_target(model=None):
                    ee_l, ee_r = self._get_selected_dual_ee_frames()
                    self.articulation.post_reset()
                    self._test_scenarios.on_ik_follow_target_dual(
                        self.articulation, ee_l, ee_r, 
                        self._robot_description_file_left, self._robot_description_file_right, self._robot_urdf_file
                    )

                self._models["kinematics_follow_target_btn"] = btn_builder(
                    label="Follow Target", text="Follow Target", on_clicked_fn=ik_follow_target
                )

    def _build_rmpflow_ui(self):
        frame = ui.CollapsableFrame(title="RmpFlow", height=0, collapsed=True, enabled=False, style=get_style())
        self._models["rmpflow_frame"] = frame
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                def check_file_type(model=None):
                    path = model.get_value_as_string()
                    if is_yaml_file(path):
                        self._rmpflow_config_yaml = path
                        self._set_enable_rmpflow_buttons(True)
                    else:
                        self._rmpflow_config_yaml = None
                        self._set_enable_rmpflow_buttons(False)

                kwargs = {
                    "label": "RmpFlow Config YAML",
                    "default_val": "",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_yaml_item,
                    "folder_dialog_title": "Select RmpFlow config YAML file",
                    "folder_button_title": "Select YAML",
                }
                self._models["input_rmp_config_file"] = str_builder(**kwargs)
                self._models["input_rmp_config_file"].add_value_changed_fn(check_file_type)

                def toggle_rmpflow_debug_mode(model=None):
                    self._test_scenarios.toggle_rmpflow_debug_mode()

                self._models["rmpflow_debug_mode"] = state_btn_builder(
                    label="Debugger", a_text="Debugging Mode", b_text="Normal Mode", on_clicked_fn=toggle_rmpflow_debug_mode
                )

                def rmpflow_follow_target(model=None):
                    # 1. 严格从 UI 面板提取当前的左右 End Effector 帧名称
                    ee_frame_left, ee_frame_right = self._get_selected_dual_ee_frames()
                    
                    # 2. 规范化：严格从 Extension 的类变量中提取用户通过 UI 选定并 Loaded 的配置文件路径
                    path_left = self._robot_description_file_left
                    path_right = self._robot_description_file_right
                    path_urdf = self._robot_urdf_file
                    path_rmp = self._rmpflow_config_yaml  # 用户在 RmpFlow 面板选择的 YAML 路径

                    # 安全校验：确保所有必填配置路径均已被 UI 加载
                    if not all([path_left, path_right, path_urdf, path_rmp]):
                        carb.log_error("Cannot launch RmpFlow: Make sure both Description YAMLs, Robot URDF, and RmpFlow Config are fully loaded via UI!")
                        return

                    # 3. 打印规范的运行日志
                    carb.log_info("[Dual-Arm RmpFlow] Launching pipeline with UI-configured paths:")
                    carb.log_info(f" -> Left Dec: '{path_left}' | Right Dec: '{path_right}'")
                    carb.log_info(f" -> URDF: '{path_urdf}' | RmpFlow Config: '{path_rmp}'")
                    
                    # 4. 打包标准参数字典（键名与后端的 LulaTestScenarios 保持精准对齐）
                    rmp_config_dict = {
                        "end_effector_frame_name_left": ee_frame_left,
                        "end_effector_frame_name_right": ee_frame_right,
                        "maximum_substep_size": 0.0034,
                        "robot_description_path_left": path_left,
                        "robot_description_path_right": path_right,
                        "urdf_path": path_urdf,
                        "rmpflow_config_path": path_rmp,
                    }
                    
                    # 5. 触发物理场景重置并启动双臂 RmpFlow
                    if self.articulation is not None:
                        self.articulation.post_reset()
                        self._test_scenarios.on_rmpflow_follow_target_obstacles_dual(
                            self.articulation, **rmp_config_dict
                        )

                self._models["rmpflow_follow_target_btn"] = btn_builder(
                    label="Follow Target", text="Follow Target", on_clicked_fn=rmpflow_follow_target
                )

    def _disable_lula_dropdowns(self):
        for n in ["kinematics_frame", "rmpflow_frame"]:
            if n in self._models:
                self._models[n].enabled = False
                self._models[n].collapsed = True

    def _enable_load_button(self):
        if self._selected_robot_description_file_left and self._selected_robot_description_file_right and self._selected_robot_urdf_file:
            self._models["load_config_btn"].enabled = True
        else:
            self._models["load_config_btn"].enabled = False

    def _enable_lula_dropdowns(self):
        if self.articulation is None or self._robot_description_file_left is None or self._robot_description_file_right is None or self._robot_urdf_file is None:
            return
        for n in ["kinematics_frame", "rmpflow_frame"]:
            self._models[n].enabled = True

        self._test_scenarios.scenario_reset()
        if self._visualize_end_effector:
            ee_l, ee_r = self._get_selected_dual_ee_frames()
            self._test_scenarios.visualize_ee_frame(self.articulation, ee_l, ee_r, self._robot_description_file_left)

    def _set_enable_rmpflow_buttons(self, enable):
        self._models["rmpflow_follow_target_btn"].enabled = enable

    def _get_selected_dual_ee_frames(self):
        if not self._ee_frame_options_left or not self._ee_frame_options_right:
            return "None", "None"
        idx_l = self._models["ee_frame_left"].get_item_value_model().as_int
        idx_r = self._models["ee_frame_right"].get_item_value_model().as_int
        return self._ee_frame_options_left[idx_l], self._ee_frame_options_right[idx_r]