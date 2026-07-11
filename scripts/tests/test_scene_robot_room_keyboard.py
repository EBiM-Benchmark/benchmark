import importlib
import importlib.util
import sys
import types
from pathlib import Path

import pytest


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
    assert scene_keyboard.normalize_keyboard_event_input("KeyboardInput.W") == "w"
    assert scene_keyboard.normalize_keyboard_event_input("ESCAPE") == "esc"
    assert scene_keyboard.normalize_keyboard_event_input("KEY_1") == "1"
    assert scene_keyboard.normalize_keyboard_event_input("KeyboardInput.KEY_2") == "2"

    class ModeKey:
        name = "KEY_3"

    assert scene_keyboard.normalize_keyboard_event_input(ModeKey()) == "3"


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


def test_keyboard_dual_arm_control_rejects_multiple_environments():
    with pytest.raises(RuntimeError, match="exactly one environment"):
        scene_keyboard.require_single_teleop_environment(2)


def test_motion_generation_extension_is_enabled_before_lula_loading():
    calls = []

    class ExtensionManager:
        def is_extension_enabled(self, extension_name):
            calls.append(("is_enabled", extension_name))
            return False

        def set_extension_enabled_immediate(self, extension_name, enabled):
            calls.append(("enable", extension_name, enabled))
            return True

    scene_keyboard.enable_motion_generation_extension(ExtensionManager())

    assert calls == [
        ("is_enabled", "isaacsim.robot_motion.motion_generation"),
        ("enable", "isaacsim.robot_motion.motion_generation", True),
    ]


def test_measured_position_targets_are_cloned_once():
    import torch

    measured = torch.tensor([[0.3, -0.4, 0.5]])
    robot = types.SimpleNamespace(
        data=types.SimpleNamespace(joint_pos=measured)
    )

    targets = scene_keyboard.measured_position_targets(robot)
    measured[0, 0] = 9.0

    assert torch.allclose(targets, torch.tensor([[0.3, -0.4, 0.5]]))
    assert targets.data_ptr() != measured.data_ptr()


def test_reset_robot_to_default_state_writes_configured_state_into_physx():
    import torch

    calls = []
    robot = types.SimpleNamespace(
        data=types.SimpleNamespace(
            default_root_state=torch.tensor(
                [[1.0, 2.0, 0.3, 1.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 0.0, 0.0, 0.0]]
            ),
            default_joint_pos=torch.tensor([[0.0, -1.5, 0.0, -2.2]]),
            default_joint_vel=torch.zeros((1, 4)),
        ),
        write_root_pose_to_sim=lambda value: calls.append(("root_pose", value.clone())),
        write_root_velocity_to_sim=lambda value: calls.append(("root_velocity", value.clone())),
        write_joint_state_to_sim=lambda pos, vel: calls.append(
            ("joint_state", pos.clone(), vel.clone())
        ),
        set_joint_position_target=lambda value: calls.append(
            ("position_target", value.clone())
        ),
        set_joint_velocity_target=lambda value: calls.append(
            ("velocity_target", value.clone())
        ),
    )

    scene_keyboard.reset_robot_to_default_state(
        robot, torch.tensor([[10.0, 20.0, 0.0]])
    )

    assert [call[0] for call in calls] == [
        "root_pose",
        "root_velocity",
        "joint_state",
        "position_target",
        "velocity_target",
    ]
    assert torch.allclose(
        calls[0][1],
        torch.tensor([[11.0, 22.0, 0.3, 1.0, 0.0, 0.0, 0.0]]),
    )
    assert torch.equal(calls[2][1], robot.data.default_joint_pos)
    assert torch.equal(calls[3][1], robot.data.default_joint_pos)


def test_robot_root_world_pose_reads_first_environment():
    import torch

    robot = types.SimpleNamespace(
        data=types.SimpleNamespace(
            root_pos_w=torch.tensor([[1.0, 2.0, 0.3]]),
            root_quat_w=torch.tensor([[0.5, 0.0, 0.0, -0.5]]),
        )
    )

    position, orientation = scene_keyboard.robot_root_world_pose(robot)

    assert position == pytest.approx((1.0, 2.0, 0.3))
    assert orientation == pytest.approx((0.5, 0.0, 0.0, -0.5))


def test_control_stage_configuration_omits_passive_viewer_robot_reference():
    calls = []

    def configure(*args, **kwargs):
        calls.append((args, kwargs))

    scene_keyboard.configure_keyboard_control_stage(
        configure,
        object(),
        object(),
        room_path=Path("room.usd"),
        task="task3",
        head_placement="A",
        robot_position=(1.0, 2.0, 0.0),
        robot_yaw=-90.0,
        dynamic_beans=False,
    )

    assert calls[0][1]["robot_path"] is None
    assert calls[0][1]["robot_position"] == (1.0, 2.0, 0.0)


def test_application_cleanup_runs_when_setup_fails():
    class App:
        closed = False

        def close(self):
            self.closed = True

    app = App()

    with pytest.raises(RuntimeError, match="IK setup"):
        scene_keyboard.run_with_app_cleanup(
            app, lambda: (_ for _ in ()).throw(RuntimeError("IK setup"))
        )

    assert app.closed is True


def test_direct_command_fails_clearly_without_articulation_soft_limits():
    from teleop_commands import TeleopCommand

    command = TeleopCommand(
        timestamp=1.0,
        source="gello",
        active=True,
        left_joint_positions=(0.0,) * 7,
    )
    robot = types.SimpleNamespace(data=types.SimpleNamespace())
    groups = types.SimpleNamespace(left_arm=tuple(range(7)), right_arm=tuple(range(7, 14)))

    with pytest.raises(RuntimeError, match="soft_joint_pos_limits"):
        scene_keyboard.clamp_direct_joint_command(command, robot, groups)
