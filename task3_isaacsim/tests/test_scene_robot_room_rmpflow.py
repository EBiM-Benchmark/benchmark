# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import pytest

SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "scene_robot_room_rmpflow.py"
)
spec = importlib.util.spec_from_file_location(
    "scene_robot_room_rmpflow", SCRIPT_PATH
)
assert spec is not None and spec.loader is not None
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


def test_module_import_is_plain_python_without_isaac_sim_imports():
    assert not any(name.startswith("isaacsim") for name in sys.modules)


def test_cli_identifies_plain_isaac_sim_and_headless_disables_keyboard():
    parser = scene.build_arg_parser()
    assert "Isaac Sim 5.1" in parser.description
    assert scene.arm_keyboard_enabled(parser.parse_args([])) is True
    assert (
        scene.arm_keyboard_enabled(parser.parse_args(["--headless"])) is False
    )
    assert (
        scene.arm_keyboard_enabled(
            parser.parse_args(["--no-arm-keyboard-teleop"])
        )
        is False
    )


def test_gui_launch_uses_full_experience_while_headless_uses_base():
    parser = scene.build_arg_parser()

    assert scene.experience_path(parser.parse_args([])).endswith(
        "isaacsim.exp.full.kit"
    )
    assert scene.experience_path(parser.parse_args(["--headless"])).endswith(
        "isaacsim.exp.base.kit"
    )


def test_rmpflow_base_speed_defaults_match_requested_rates():
    args = scene.build_arg_parser().parse_args([])

    assert args.base_linear_speed == pytest.approx(0.25)
    assert args.base_angular_speed == pytest.approx(0.75)
    assert args.base_diagnostics_interval == pytest.approx(0.0)
    assert args.base_heading_hold is True
    assert (
        scene.build_arg_parser()
        .parse_args(["--no-base-heading-hold"])
        .base_heading_hold
        is False
    )


def test_base_diagnostic_line_reports_command_tracking_and_displacement():
    line = scene.format_base_diagnostic_line(
        elapsed_s=2.5,
        yaw_rad=-0.25,
        body_command=(0.25, 0.0, 0.0),
        displacement=(0.10, -0.01, 0.0),
        steering_command=(0.0, 0.0),
        steering_actual=(0.02, -0.03),
        wheel_command=(5.0, 5.0),
        wheel_actual=(0.4, 0.5),
        wheel_effort=(12.0, 13.0),
    )

    assert "base-diag t=2.50s" in line
    assert "yaw=-0.250" in line
    assert "twist=[0.250, 0.000, 0.000]" in line
    assert "delta_xyz=[0.100, -0.010, 0.000]" in line
    assert "steer cmd/actual=[0.000, 0.000]/[0.020, -0.030]" in line
    assert "wheel cmd/actual=[5.000, 5.000]/[0.400, 0.500]" in line
    assert "effort=[12.000, 13.000]" in line


def test_plain_isaac_sim_uses_task3_arm_spine_and_gripper_drive_gains():
    assert scene.robot_drive_gains("tmrv0_2_joint_1") == (0.0, 5.0, 500.0)
    assert scene.robot_drive_gains("tmrv0_2_joint_3") == (0.0, 5.0, 500.0)
    assert scene.robot_drive_gains("caster_front_left_joint") == (
        0.0,
        0.0,
        0.0,
    )
    assert scene.robot_drive_gains("caster_rear_right_steering_joint") == (
        0.0,
        0.0,
        0.0,
    )
    assert scene.robot_drive_gains("rocker_arm_joint") == (0.0, 0.003, 500.0)
    assert scene.robot_drive_gains("left_fr3v2_joint1") == (
        5000.0,
        500.0,
        200.0,
    )
    assert scene.robot_drive_gains("right_fr3v2_joint7") == (
        5000.0,
        500.0,
        200.0,
    )
    assert scene.robot_drive_gains("franka_spine_vertical_joint") == (
        5000.0,
        500.0,
        200.0,
    )
    assert scene.robot_drive_gains("left_fr3v2_finger_joint1") == (
        200.0,
        20.0,
        50.0,
    )


def test_merge_policy_actions_merges_independent_joint_indices():
    left = types.SimpleNamespace(
        joint_indices=np.array([3, 1]),
        joint_positions=np.array([0.3, 0.1]),
        joint_velocities=np.array([3.0, 1.0]),
    )
    right = types.SimpleNamespace(
        joint_indices=np.array([8, 7]),
        joint_positions=np.array([0.8, 0.7]),
        joint_velocities=None,
    )

    positions, velocities, indices = scene.merge_policy_actions([left, right])

    assert indices.tolist() == [1, 3, 7, 8]
    assert positions.tolist() == pytest.approx([0.1, 0.3, 0.7, 0.8])
    assert velocities.tolist() == pytest.approx([1.0, 3.0, 0.0, 0.0])


def test_root_pose_composition_rotates_robot_relative_target():
    half = np.sqrt(0.5)
    position, orientation = scene.compose_world_pose(
        np.array((4.0, 2.0, 0.0)),
        np.array((half, 0.0, 0.0, -half)),
        np.array((1.0, 0.0, 0.5)),
        np.array((1.0, 0.0, 0.0, 0.0)),
    )

    assert position == pytest.approx((4.0, 1.0, 0.5))
    assert orientation == pytest.approx((half, 0.0, 0.0, -half))


def test_versioned_rmpflow_configs_use_current_hand_tcp_frames():
    task3_root = Path(__file__).resolve().parents[1]
    config_dir = task3_root / "assets" / "lula" / "mobile_fr3_duo"
    assert (
        scene.ARM_CONFIGS["left"]["end_effector_frame"]
        == "left_fr3v2_hand_tcp"
    )
    assert (
        scene.ARM_CONFIGS["right"]["end_effector_frame"]
        == "right_fr3v2_hand_tcp"
    )
    for side in ("left", "right"):
        path = config_dir / f"{side}_arm_rmpflow_config.yaml"
        assert path.is_file()
        assert f"{side}_fr3v2_link7" in path.read_text()


def test_task3_initial_arm_targets_follow_articulation_joint_order():
    names = [
        "unowned",
        "right_fr3v2_joint2",
        "left_fr3v2_joint7",
        "left_fr3v2_joint2",
        "right_fr3v2_joint7",
    ]

    positions, indices = scene.task3_initial_arm_targets(names)

    assert indices.tolist() == [1, 2, 3, 4]
    assert positions.tolist() == pytest.approx([-1.5, 0.785, -1.5, 0.785])


def test_shift_base_key_mapping_uses_requested_directions():
    assert scene.base_twist_from_held_key_names({"h"}) == pytest.approx(
        (0, 0, 0)
    )
    assert scene.base_twist_from_held_key_names(
        {"shift", "h"}
    ) == pytest.approx((1, 0, 0))
    assert scene.base_twist_from_held_key_names(
        {"shift", "n"}
    ) == pytest.approx((-1, 0, 0))
    assert scene.base_twist_from_held_key_names(
        {"shift", "b"}
    ) == pytest.approx((0, 1, 0))
    assert scene.base_twist_from_held_key_names(
        {"shift", "m"}
    ) == pytest.approx((0, -1, 0))
    assert scene.base_twist_from_held_key_names(
        {"shift", "g"}
    ) == pytest.approx((0, 0, 1))
    assert scene.base_twist_from_held_key_names(
        {"shift", "j"}
    ) == pytest.approx((0, 0, -1))


def test_heading_hold_counteracts_ccw_drift_during_straight_translation():
    wz, desired_yaw = scene.compensate_heading_yaw_rate(
        current_yaw=0.2,
        current_yaw_rate=0.0,
        vx=0.15,
        vy=0.0,
        wz=0.0,
        desired_yaw=0.0,
        manual_rotation=False,
    )

    assert wz < 0.0
    assert desired_yaw == pytest.approx(0.0)


def test_heading_hold_resets_when_rotation_is_commanded_or_translation_stops():
    manual_wz, manual_desired = scene.compensate_heading_yaw_rate(
        current_yaw=0.2,
        current_yaw_rate=0.5,
        vx=0.15,
        vy=0.0,
        wz=0.35,
        desired_yaw=0.0,
        manual_rotation=True,
    )
    idle_wz, idle_desired = scene.compensate_heading_yaw_rate(
        current_yaw=0.3,
        current_yaw_rate=0.0,
        vx=0.0,
        vy=0.0,
        wz=0.0,
        desired_yaw=0.0,
        manual_rotation=False,
    )

    assert manual_wz == pytest.approx(0.35)
    assert manual_desired == pytest.approx(0.2)
    assert idle_wz == pytest.approx(0.0)
    assert idle_desired == pytest.approx(0.3)


def test_plain_isaac_sim_steering_targets_stay_inside_physx_drive_range():
    targets, _drive = scene.compute_base_drive_targets(
        np.array((13.0, 0.0, -13.0, 0.0)),
        steering_indices=(0, 2),
        vx=0.15,
        vy=0.0,
        wz=0.0,
    )

    assert np.all(np.abs(targets) < 2.0 * np.pi)


def test_forward_after_yaw_waits_for_both_modules():
    targets, drive = scene.compute_base_drive_targets(
        np.array((0.98, 0.0, -2.16, 0.0)),
        steering_indices=(0, 2),
        vx=0.25,
        vy=0.0,
        wz=0.0,
    )

    assert targets.tolist() == pytest.approx((0.0, -np.pi), abs=1.0e-5)
    assert drive.tolist() == pytest.approx((0.0, 0.0), abs=1.0e-5)


def test_forward_with_flipped_module_preserves_wheel_ratio_after_alignment():
    targets, drive = scene.compute_base_drive_targets(
        np.array((0.0, 0.0, -np.pi, 0.0)),
        steering_indices=(0, 2),
        vx=0.25,
        vy=0.0,
        wz=0.0,
    )

    assert targets.tolist() == pytest.approx((0.0, -np.pi), abs=1.0e-5)
    assert drive.tolist() == pytest.approx((5.0, -5.0), abs=1.0e-5)


def test_rmpflow_help_uses_a_terminal_control_panel_layout():
    assert "+---------------- RMPFLOW CONTROL PANEL" in scene.CONTROL_HELP
    assert "[SHIFT + H] Forward" in scene.CONTROL_HELP
    assert "[SHIFT + J] Rotate CW" in scene.CONTROL_HELP
    assert "LEFT ARM" in scene.CONTROL_HELP
    assert "RIGHT ARM" in scene.CONTROL_HELP
