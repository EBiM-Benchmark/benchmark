"""
real_to_sim_bridge_reader.py — Cyclonedds-side reader for the cross-RMW bridge.

Spawned as a subprocess by real_to_sim_bridge.py with:
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ROS_DOMAIN_ID=<real robot domain>
    BRIDGE_CONFIG_PATH=<path to bridge_config.yaml>

Subscribes to the real robot joint state topics (defined in bridge_config.yaml)
and writes one JSON line per message to stdout so the parent process can
republish on the sim's ROS domain.

Do NOT run this directly — use real_to_sim_bridge.py or run_real_to_sim_bridge.sh.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bridge_config_loader import load_bridge_config


class BridgeReader(Node):
    """Minimal ROS node that subscribes to real robot topics and writes JSON to stdout."""

    def __init__(self) -> None:
        super().__init__("real_to_sim_bridge_reader")

        # Load configuration from file
        config_path = os.environ.get("BRIDGE_CONFIG_PATH")
        if not config_path or not Path(config_path).exists():
            self.get_logger().error(f"Bridge config not found: {config_path}")
            self.get_logger().error("Set BRIDGE_CONFIG_PATH environment variable")
            return

        config = load_bridge_config(config_path)
        self.get_logger().info(f"Loaded config: {config_path}")

        # Subscribe to all enabled mappings
        for mapping in config.get_enabled_mappings():
            if mapping.is_arm:
                self.create_subscription(
                    JointState,
                    mapping.input_topic,
                    self._make_arm_cb(mapping.name, mapping.num_joints),
                    10
                )
                self.get_logger().info(f"Subscribed to {mapping.name}: {mapping.input_topic}")
            elif mapping.is_gripper:
                idx = mapping.gripper_joint_index or 0
                self.create_subscription(
                    JointState,
                    mapping.input_topic,
                    self._make_gripper_cb(mapping.name, idx),
                    10
                )
                self.get_logger().info(f"Subscribed to {mapping.name}: {mapping.input_topic}")

        if not config.get_enabled_mappings():
            self.get_logger().warning("No enabled mappings in configuration")

    def _make_arm_cb(self, side: str, num_joints: int):
        def _cb(msg: JointState) -> None:
            positions = list(msg.position[:num_joints])
            if len(positions) < num_joints:
                return
            _write({"side": side, "positions": positions})
        return _cb

    def _make_gripper_cb(self, side: str, joint_index: int):
        def _cb(msg: JointState) -> None:
            if len(msg.position) > joint_index:
                _write({"side": side, "positions": [float(msg.position[joint_index])]})
        return _cb


def _write(record: dict) -> None:
    sys.stdout.write(json.dumps(record) + "\n")
    sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = BridgeReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
