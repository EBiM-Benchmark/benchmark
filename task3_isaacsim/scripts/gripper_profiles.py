# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Import-safe robot and gripper profiles for Task 3 launchers."""

from __future__ import annotations

import re
from pathlib import Path
from typing import NamedTuple

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GRIPPER = "robotiq"
GRIPPER_PROFILE_NAMES = ("robotiq", "panda")


class GripperProfile(NamedTuple):
    name: str
    robot_usd: Path
    republisher_range: tuple[float, float]
    republisher_invert: bool
    keyboard_positions: tuple[float, float]


GRIPPER_PROFILES = {
    "robotiq": GripperProfile(
        name="robotiq",
        robot_usd=(
            REPO_ROOT
            / "task1_isaacsim"
            / "assets"
            / "Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd"
        ),
        republisher_range=(0.0, 0.8),
        republisher_invert=False,
        keyboard_positions=(0.0, 0.8),
    ),
    "panda": GripperProfile(
        name="panda",
        robot_usd=(
            REPO_ROOT
            / "third_party"
            / "franka_description"
            / "urdfs"
            / "mobile_fr3_duo_v0_2_franka_hand.usd"
        ),
        republisher_range=(0.0, 0.04),
        republisher_invert=True,
        keyboard_positions=(0.04, 0.0),
    ),
}


def get_gripper_profile(name: str | None) -> GripperProfile:
    """Return one complete Task 3 robot/gripper profile."""
    normalized = DEFAULT_GRIPPER if name is None else str(name).lower()
    try:
        return GRIPPER_PROFILES[normalized]
    except KeyError as exc:
        choices = ", ".join(GRIPPER_PROFILE_NAMES)
        raise ValueError(
            f"unknown gripper profile {name!r}; choose one of: {choices}"
        ) from exc


def get_profile_drive_gains(
    profile_name: str, joint_name: str
) -> dict[str, float] | None:
    """Return runtime drive overrides required by a gripper profile."""
    profile = get_gripper_profile(profile_name)
    is_panda_arm = (
        joint_name == "franka_spine_vertical_joint"
        or re.fullmatch(r"(?:left|right)_fr3v2_joint[1-7]", joint_name)
        is not None
    )
    if profile.name == "panda" and is_panda_arm:
        return {
            "stiffness": 5000.0,
            "damping": 500.0,
            "max_force": 200.0,
        }
    return None
