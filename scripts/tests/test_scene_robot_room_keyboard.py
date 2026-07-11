# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

import importlib
import importlib.util
import sys
import types
from pathlib import Path

SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scenes"
    / "scene_robot_room_keyboard.py"
)
spec = importlib.util.spec_from_file_location(
    "scene_robot_room_keyboard",
    SCRIPT_PATH,
)
assert spec is not None
assert spec.loader is not None
scene_keyboard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene_keyboard)


def test_task3_enables_keyboard_control_by_default(monkeypatch):
    monkeypatch.delenv(scene_keyboard.INSIDE_KIT_ENV_VAR, raising=False)
    monkeypatch.delenv(scene_keyboard.INNER_ARGV_ENV_VAR, raising=False)
    monkeypatch.setattr(
        "sys.argv",
        ["scene_robot_room_keyboard.py", "--task", "task3"],
    )

    args = scene_keyboard.parse_args()

    assert scene_keyboard.should_enable_keyboard_control(args) is True


def test_keyboard_control_can_be_disabled_for_viewer_mode(monkeypatch):
    monkeypatch.delenv(scene_keyboard.INSIDE_KIT_ENV_VAR, raising=False)
    monkeypatch.delenv(scene_keyboard.INNER_ARGV_ENV_VAR, raising=False)
    monkeypatch.setattr(
        "sys.argv",
        [
            "scene_robot_room_keyboard.py",
            "--task",
            "task3",
            "--no-keyboard-control",
        ],
    )

    args = scene_keyboard.parse_args()

    assert scene_keyboard.should_enable_keyboard_control(args) is False


def test_robot_actuator_patterns_match_tmr_base_control():
    actuators = scene_keyboard.robot_actuator_cfg_specs()

    assert actuators["steering_joints"]["joint_names_expr"] == [
        "tmrv0_2_joint_0",
        "tmrv0_2_joint_2",
    ]
    assert actuators["drive_joints"]["joint_names_expr"] == [
        "tmrv0_2_joint_1",
        "tmrv0_2_joint_3",
    ]
    assert actuators["drive_joints"]["stiffness"] == 0.0
    assert actuators["drive_joints"]["velocity_limit_sim"] == 20.0
    assert actuators["spine"]["joint_names_expr"] == [
        "franka_spine_vertical_joint",
    ]
    assert "effort_limit" not in actuators["steering_joints"]
    assert "effort_limit" not in actuators["arms"]
    assert "effort_limit" not in actuators["grippers"]


def test_kit_keyboard_event_names_are_normalized():
    class KeyInput:
        name = "LEFT_ARROW"

    assert scene_keyboard.normalize_keyboard_event_input(KeyInput()) == "left"
    assert (
        scene_keyboard.normalize_keyboard_event_input("KeyboardInput.W") == "w"
    )
    assert scene_keyboard.normalize_keyboard_event_input("ESCAPE") == "esc"


def test_create_keyboard_teleop_prefers_kit_backend(monkeypatch):
    kit_teleop = object()
    monkeypatch.setattr(
        scene_keyboard,
        "KitKeyboardTeleop",
        lambda carb_input, appwindow: kit_teleop,
    )
    monkeypatch.setattr(
        scene_keyboard,
        "PynputKeyboardTeleop",
        lambda _keyboard: (_ for _ in ()).throw(
            AssertionError("pynput backend should not be selected")
        ),
    )

    carb_input_module = types.ModuleType("carb.input")
    carb_module = types.ModuleType("carb")
    carb_module.input = carb_input_module
    omni_appwindow_module = types.ModuleType("omni.appwindow")
    omni_module = types.ModuleType("omni")
    omni_module.appwindow = omni_appwindow_module

    monkeypatch.setitem(sys.modules, "carb", carb_module)
    monkeypatch.setitem(sys.modules, "carb.input", carb_input_module)
    monkeypatch.setitem(sys.modules, "omni", omni_module)
    monkeypatch.setitem(sys.modules, "omni.appwindow", omni_appwindow_module)

    assert scene_keyboard.create_keyboard_teleop() is kit_teleop


def test_disable_robot_external_wrenches_resets_composers():
    class Composer:
        def __init__(self):
            self.reset_count = 0

        def reset(self):
            self.reset_count += 1

    class Robot:
        def __init__(self):
            self.instantaneous_wrench_composer = Composer()
            self.permanent_wrench_composer = Composer()

    robot = Robot()

    scene_keyboard.disable_robot_external_wrenches(robot)

    assert robot.instantaneous_wrench_composer.reset_count == 1
    assert robot.permanent_wrench_composer.reset_count == 1
