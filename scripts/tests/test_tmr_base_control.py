import sys
import math
from pathlib import Path

import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))

from tmr_base_control import compute_drive_targets


class _Robot:
    def __init__(self, steering_positions):
        self.data = type(
            "Data", (), {"joint_pos": torch.tensor([steering_positions])}
        )()


def test_drive_wheels_creep_while_modules_turn_toward_a_new_translation_direction():
    robot = _Robot([0.0, 0.0, 0.0, 0.0])

    _steering, drive = compute_drive_targets(
        robot,
        steering_ids=[0, 2],
        vx=0.0,
        vy=0.15,
        wz=0.0,
        num_envs=1,
        device="cpu",
    )

    assert torch.all(torch.abs(drive) > 0.0)


def test_steering_targets_stay_in_physx_revolute_drive_range():
    robot = _Robot([13.0, 0.0, -13.0, 0.0])

    steering, _drive = compute_drive_targets(
        robot,
        steering_ids=[0, 2],
        vx=0.15,
        vy=0.0,
        wz=0.0,
        num_envs=1,
        device="cpu",
    )

    assert torch.all(torch.abs(steering) < 2.0 * math.pi)
