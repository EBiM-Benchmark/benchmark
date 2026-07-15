#!/usr/bin/env python3
"""Headless ROS bridge for the raw Newton VBD board-cable example.

This runs the existing ``cable_world`` package in a separate process from IsaacLab/Kit.
It subscribes to gripper pose/gap topics and publishes cable body centers.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy


class NullCableViewer:
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


def _resolve_cli_path(path_value: str | Path, cwd: Path) -> Path:
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        path = cwd / path
    return path.resolve()


def _load_configs(cable_dir: Path, config_path: Path | None, gripper_config_path: Path | None):
    if str(cable_dir) not in sys.path:
        sys.path.insert(0, str(cable_dir))

    from run_board_cable import (  # noqa: PLC0415
        DEFAULT_CONFIG_PATH,
        DEFAULT_GRIPPER_CONFIG_PATH,
        _config_base_dir,
        _first_config_value,
        _load_yaml_mapping,
        _resolve_config_path,
    )
    from sra_gripper import load_gripper_config  # noqa: PLC0415

    resolved_config_path = _resolve_cli_path(config_path or DEFAULT_CONFIG_PATH, cable_dir)
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
        resolved_gripper_config_path = _resolve_cli_path(gripper_config_path, cable_dir)

    gripper_config = load_gripper_config(resolved_gripper_config_path)
    return resolved_config_path, config_data, resolved_gripper_config_path, gripper_config


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    cable_dir = repo_root / "cable_world"
    if str(cable_dir) not in sys.path:
        sys.path.insert(0, str(cable_dir))

    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config-path", type=Path, default=None)
    pre_parser.add_argument("--gripper-config-path", type=Path, default=None)
    pre_args, _ = pre_parser.parse_known_args()

    config_path, config_data, gripper_config_path, gripper_config = _load_configs(
        cable_dir,
        pre_args.config_path,
        pre_args.gripper_config_path,
    )

    from run_board_cable import Example, _make_parser  # noqa: PLC0415
    from run_board_cable_ros import CableRosBridge, _add_ros_args  # noqa: PLC0415

    parser = _make_parser(config_path, config_data, gripper_config_path, gripper_config)
    _add_ros_args(parser)
    # In the regular example these options are supplied by newton.examples.init().
    # This headless path deliberately bypasses init() to avoid opening a GL viewer.
    if "--viewer" not in parser._option_string_actions:
        parser.add_argument("--viewer", default="null")
    if "--device" not in parser._option_string_actions:
        parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    viewer = NullCableViewer()
    example = Example(viewer, args)

    rclpy.init()
    node = CableRosBridge(example, args)
    num_frames = int(getattr(args, "num_frames", 0) or 0)
    frame_dt = float(example.frame_dt)
    frame_count = 0

    try:
        while rclpy.ok():
            frame_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            node.apply_robotiq_finger_targets()
            example.step()
            node.publish_cable_state()
            node.publish_gripper_collision_boxes()
            frame_count += 1

            if num_frames > 0 and frame_count >= num_frames:
                break

            if bool(args.real_time):
                elapsed = time.monotonic() - frame_start
                time.sleep(max(0.0, frame_dt - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
