# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Tests for Task 3 robot/gripper profile selection."""

import importlib.util
from pathlib import Path

import pytest

TASK3_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = TASK3_ROOT / "scripts" / "gripper_profiles.py"


def load_profiles_module():
    assert MODULE_PATH.is_file(), "Task 3 gripper profile module is missing"
    spec = importlib.util.spec_from_file_location(
        "gripper_profiles", MODULE_PATH
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_robotiq_is_the_default_benchmark_profile():
    profiles = load_profiles_module()

    profile = profiles.get_gripper_profile(None)

    assert profiles.DEFAULT_GRIPPER == "robotiq"
    assert profile.name == "robotiq"
    assert profile.robot_usd.name == (
        "Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd"
    )
    assert profile.republisher_range == (0.0, 0.8)
    assert profile.republisher_invert is False
    assert profile.keyboard_positions == (0.0, 0.8)


def test_panda_profile_uses_franka_hand_asset_and_stroke():
    profiles = load_profiles_module()

    profile = profiles.get_gripper_profile("panda")

    assert profile.name == "panda"
    assert profile.robot_usd.name == "mobile_fr3_duo_v0_2_franka_hand.usd"
    assert profile.republisher_range == (0.0, 0.04)
    assert profile.republisher_invert is True
    assert profile.keyboard_positions == (0.04, 0.0)


@pytest.mark.parametrize(
    "joint_name",
    [
        "franka_spine_vertical_joint",
        "left_fr3v2_joint1",
        "left_fr3v2_joint7",
        "right_fr3v2_joint1",
        "right_fr3v2_joint7",
    ],
)
def test_panda_profile_authors_arm_holding_gains(joint_name):
    profiles = load_profiles_module()

    gains = profiles.get_profile_drive_gains("panda", joint_name)

    assert gains == {
        "stiffness": 5000.0,
        "damping": 500.0,
        "max_force": 200.0,
    }


def test_robotiq_profile_preserves_authored_arm_drives():
    profiles = load_profiles_module()

    gains = profiles.get_profile_drive_gains("robotiq", "left_fr3v2_joint1")

    assert gains is None


def test_unknown_gripper_profile_is_rejected():
    profiles = load_profiles_module()

    with pytest.raises(ValueError, match="unknown gripper profile"):
        profiles.get_gripper_profile("suction")
