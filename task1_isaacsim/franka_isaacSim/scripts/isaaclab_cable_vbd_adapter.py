#!/usr/bin/env python3
"""Adapter that runs the existing raw Newton VBD board-cable demo from IsaacLab.

The adapter keeps the cable simulation in its original Newton ``SolverVBD``
world, but exposes a small interface that can be driven from an IsaacLab robot
end-effector pose.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Iterable

import numpy as np


def _quat_xyzw_to_euler_xyz(quat_xyzw: Iterable[float]) -> tuple[float, float, float]:
    x, y, z, w = [float(v) for v in quat_xyzw]

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw




class _NullCableViewer:
    """Minimal viewer interface used by cable_world/run_board_cable.py."""

    def set_model(self, model) -> None:
        self.model = model

    def apply_forces(self, state) -> None:
        return None

    def begin_frame(self, sim_time: float) -> None:
        return None

    def log_state(self, state) -> None:
        return None

    def log_contacts(self, contacts, state) -> None:
        return None

    def end_frame(self) -> None:
        return None

    def set_camera(self, *args, **kwargs) -> None:
        return None


class CableVbdAdapter:
    """Small wrapper around ``cable_world/run_board_cable.py``."""

    def __init__(
        self,
        *,
        franka_root: Path,
        config_path: Path,
        gripper_config_path: Path | None,
        device: str,
        extra_args: list[str] | None = None,
    ) -> None:
        self.franka_root = Path(franka_root)
        self.cable_dir = self.franka_root / "cable_world"
        if not self.cable_dir.is_dir():
            raise FileNotFoundError(f"Missing cable package directory: {self.cable_dir}")

        if str(self.cable_dir) not in sys.path:
            sys.path.insert(0, str(self.cable_dir))

        from run_board_cable import (  # noqa: PLC0415
            DEFAULT_GRIPPER_CONFIG_PATH,
            Example,
            _config_base_dir,
            _first_config_value,
            _load_yaml_mapping,
            _make_parser,
            _resolve_config_path,
        )
        from sra_gripper import load_gripper_config  # noqa: PLC0415

        resolved_config_path = Path(config_path).expanduser().resolve()
        if not resolved_config_path.is_file():
            raise FileNotFoundError(f"Missing board cable config YAML: {resolved_config_path}")
        config_data = _load_yaml_mapping(resolved_config_path)
        config_base = _config_base_dir(resolved_config_path)

        if gripper_config_path is None:
            raw_gripper_config_path = _first_config_value(
                config_data,
                (("gripper_config_path",),),
                DEFAULT_GRIPPER_CONFIG_PATH,
            )
            resolved_gripper_config_path = _resolve_config_path(raw_gripper_config_path, config_base)
        else:
            resolved_gripper_config_path = Path(gripper_config_path).expanduser()
            if not resolved_gripper_config_path.is_absolute():
                resolved_gripper_config_path = self.franka_root / resolved_gripper_config_path
            resolved_gripper_config_path = resolved_gripper_config_path.resolve()

        gripper_config = load_gripper_config(resolved_gripper_config_path)
        parser = _make_parser(resolved_config_path, config_data, resolved_gripper_config_path, gripper_config)
        argv = [
            "--viewer",
            "null",
            "--device",
            str(device),
            "--gripper",
            "--no-gripper-teleop",
        ]
        if extra_args:
            argv += list(extra_args)

        parsed_args = parser.parse_args(argv)
        viewer = _NullCableViewer()

        self.example = Example(viewer, parsed_args)
        if self.example.gripper_controller is None:
            raise RuntimeError("Cable VBD world was created without a gripper controller.")

        self._last_gap_m = float(self.example.gripper_controller.command_gap_m())

    @property
    def gripper_gap_range(self) -> tuple[float, float]:
        cfg = self.example.gripper_controller.config
        return float(cfg.gap.min_m), float(cfg.gap.max_m)

    def set_gripper_pose(
        self,
        *,
        position_m: Iterable[float],
        quat_xyzw: Iterable[float],
        gap_m: float | None = None,
    ) -> None:
        position = tuple(float(v) for v in position_m)
        euler_xyz = _quat_xyzw_to_euler_xyz(quat_xyzw)
        if gap_m is None:
            gap_m = self._last_gap_m
        self._last_gap_m = float(gap_m)
        self.example.gripper_controller.set_command(position, euler_xyz, self._last_gap_m)

    def step(self) -> None:
        self.example.step()

    def cable_body_positions(self) -> np.ndarray:
        cable_body_ids = np.asarray(self.example.import_result.cable_body_ids, dtype=np.int64)
        if cable_body_ids.size == 0:
            return np.zeros((0, 3), dtype=np.float32)
        body_q = self.example.state_0.body_q.numpy()
        return np.asarray(body_q[cable_body_ids, :3], dtype=np.float32)
