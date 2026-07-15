#!/usr/bin/env python3
"""Standalone ROS2 republisher between Isaac topics and bridge topics.

This process is intended to run in its own container. It keeps Isaac-side
topics stable on `/isaac/*` and exposes browser/controller-friendly topics on
`/bridge/*`.
"""

import argparse
from functools import partial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from isaac_bridge_constants import (
    LEFT_JOINTS as LEFT_ARM_JOINTS,
    RIGHT_JOINTS as RIGHT_ARM_JOINTS,
    LEFT_GRIPPER_DRIVER_JOINT as LEFT_ROBOTIQ_DRIVER_JOINT,
    RIGHT_GRIPPER_DRIVER_JOINT as RIGHT_ROBOTIQ_DRIVER_JOINT,
)

LEFT_ROBOTIQ_OPENING_JOINT = "left_robotiq_opening"
RIGHT_ROBOTIQ_OPENING_JOINT = "right_robotiq_opening"


def _topic(prefix, suffix):
    clean_prefix = "/" + prefix.strip("/")
    clean_suffix = "/" + suffix.strip("/")
    return f"{clean_prefix}{clean_suffix}"


GROUP_DEFINITIONS = [
    {
        "label": "Left Arm",
        "mode": "joint_passthrough",
        "raw_state_suffix": "left_joint_states",
        "raw_command_suffix": "browser/left_joint_commands",
        "bridge_state_suffix": "left_joint_states",
        "bridge_command_suffix": "left_joint_commands",
        "expected_joint_names": LEFT_ARM_JOINTS,
    },
    {
        "label": "Right Arm",
        "mode": "joint_passthrough",
        "raw_state_suffix": "right_joint_states",
        "raw_command_suffix": "browser/right_joint_commands",
        "bridge_state_suffix": "right_joint_states",
        "bridge_command_suffix": "right_joint_commands",
        "expected_joint_names": RIGHT_ARM_JOINTS,
    },
    {
        "label": "Left Robotiq",
        "mode": "gripper_opening",
        "raw_state_suffix": "left_robotiq_joint_states",
        "raw_command_suffix": "left_robotiq_joint_commands",
        "bridge_state_suffix": "left_robotiq_joint_states",
        "bridge_command_suffix": "left_robotiq_joint_commands",
        "raw_driver_joint": LEFT_ROBOTIQ_DRIVER_JOINT,
        "bridge_opening_joint": LEFT_ROBOTIQ_OPENING_JOINT,
    },
    {
        "label": "Right Robotiq",
        "mode": "gripper_opening",
        "raw_state_suffix": "right_robotiq_joint_states",
        "raw_command_suffix": "right_robotiq_joint_commands",
        "bridge_state_suffix": "right_robotiq_joint_states",
        "bridge_command_suffix": "right_robotiq_joint_commands",
        "raw_driver_joint": RIGHT_ROBOTIQ_DRIVER_JOINT,
        "bridge_opening_joint": RIGHT_ROBOTIQ_OPENING_JOINT,
    },
]


class JointRepublisher(Node):
    def __init__(
        self,
        bridge_prefix,
        isaac_prefix,
        gripper_open_position,
        gripper_closed_position,
        gripper_invert=False,
        disable_browser_command_topics=False,
    ):
        super().__init__("isaac_joint_republisher")
        self._gripper_open_position = float(gripper_open_position)
        self._gripper_closed_position = float(gripper_closed_position)
        self._gripper_invert = bool(gripper_invert)
        self._disable_browser_command_topics = bool(disable_browser_command_topics)
        if self._gripper_closed_position <= self._gripper_open_position:
            raise ValueError("gripper_closed_position must be greater than gripper_open_position")

        self._groups = {}
        for definition in GROUP_DEFINITIONS:
            raw_state_topic = _topic(isaac_prefix, definition["raw_state_suffix"])
            raw_command_suffix = definition["raw_command_suffix"]
            if self._disable_browser_command_topics and raw_command_suffix.startswith("browser/"):
                raw_command_suffix = raw_command_suffix[len("browser/"):]
            raw_command_topic = _topic(isaac_prefix, raw_command_suffix)
            bridge_state_topic = _topic(bridge_prefix, definition["bridge_state_suffix"])
            bridge_command_topic = _topic(bridge_prefix, definition["bridge_command_suffix"])
            group = dict(definition)
            group["raw_state_topic"] = raw_state_topic
            group["raw_command_topic"] = raw_command_topic
            group["bridge_state_topic"] = bridge_state_topic
            group["bridge_command_topic"] = bridge_command_topic
            group["bridge_state_pub"] = self.create_publisher(JointState, bridge_state_topic, 10)
            group["raw_command_pub"] = self.create_publisher(JointState, raw_command_topic, 10)
            group["raw_state_sub"] = self.create_subscription(
                JointState,
                raw_state_topic,
                partial(self._on_raw_state, group_key=definition["label"]),
                10,
            )
            group["bridge_command_sub"] = self.create_subscription(
                JointState,
                bridge_command_topic,
                partial(self._on_bridge_command, group_key=definition["label"]),
                10,
            )
            self._groups[definition["label"]] = group

        self._log_configuration()

    def _log_configuration(self):
        for group in self._groups.values():
            self.get_logger().info(
                f"{group['label']}: {group['raw_state_topic']} -> {group['bridge_state_topic']}, "
                f"{group['bridge_command_topic']} -> {group['raw_command_topic']}"
            )

    @staticmethod
    def _extract_joint_position(names, positions, desired_name):
        by_name = {}
        for index, name in enumerate(names):
            if index >= len(positions):
                break
            by_name[name] = positions[index]

        if desired_name in by_name:
            return by_name[desired_name]

        desired_tail = desired_name.split("_", 1)[-1] if "_" in desired_name else desired_name
        for name, value in by_name.items():
            if name.endswith(desired_name) or name.endswith(desired_tail):
                return value

        if positions:
            return positions[0]
        return None

    def _normalize_opening(self, driver_position):
        stroke = self._gripper_closed_position - self._gripper_open_position
        normalized_close = (driver_position - self._gripper_open_position) / stroke
        normalized_close = min(max(normalized_close, 0.0), 1.0)
        normalized_open = 1.0 - normalized_close
        # Apply invert flag if gripper convention is inverted
        if self._gripper_invert:
            return 1.0 - normalized_open
        return normalized_open

    def _driver_from_opening(self, opening):
        normalized_open = min(max(float(opening), 0.0), 1.0)
        # Apply invert flag if gripper convention is inverted
        if self._gripper_invert:
            normalized_open = 1.0 - normalized_open
        stroke = self._gripper_closed_position - self._gripper_open_position
        normalized_close = 1.0 - normalized_open
        return self._gripper_open_position + normalized_close * stroke

    @staticmethod
    def _stamp_like(source, target):
        try:
            target.header.stamp = source.header.stamp
        except Exception:
            pass

    def _build_passthrough_state(self, msg, expected_names):
        output_names = []
        output_positions = []
        output_velocities = []
        output_efforts = []
        names = list(msg.name)
        positions = list(msg.position)
        velocities = list(msg.velocity)
        efforts = list(msg.effort)

        if names:
            by_name_pos = {}
            by_name_vel = {}
            by_name_eff = {}
            for index, name in enumerate(names):
                if index < len(positions):
                    by_name_pos[name] = positions[index]
                if index < len(velocities):
                    by_name_vel[name] = velocities[index]
                if index < len(efforts):
                    by_name_eff[name] = efforts[index]
            for name in expected_names:
                if name in by_name_pos:
                    output_names.append(name)
                    output_positions.append(float(by_name_pos[name]))
                    output_velocities.append(float(by_name_vel.get(name, 0.0)))
                    output_efforts.append(float(by_name_eff.get(name, 0.0)))

        if not output_names:
            output_names = list(expected_names)
            output_positions = [0.0] * len(output_names)
            output_velocities = [0.0] * len(output_names)
            output_efforts = [0.0] * len(output_names)
            for index in range(min(len(output_names), len(positions))):
                output_positions[index] = float(positions[index])
            for index in range(min(len(output_names), len(velocities))):
                output_velocities[index] = float(velocities[index])
            for index in range(min(len(output_names), len(efforts))):
                output_efforts[index] = float(efforts[index])

        out = JointState()
        self._stamp_like(msg, out)
        out.name = output_names
        out.position = output_positions
        out.velocity = output_velocities
        out.effort = output_efforts
        return out

    def _build_gripper_opening_state(self, msg, raw_driver_joint, bridge_opening_joint):
        names = list(msg.name)
        positions = list(msg.position)
        driver_position = self._extract_joint_position(names, positions, raw_driver_joint)
        if driver_position is None:
            return None

        out = JointState()
        self._stamp_like(msg, out)
        out.name = [bridge_opening_joint]
        out.position = [self._normalize_opening(float(driver_position))]
        return out

    def _on_raw_state(self, msg, group_key):
        group = self._groups.get(group_key)
        if group is None:
            return

        if group["mode"] == "joint_passthrough":
            out = self._build_passthrough_state(msg, group["expected_joint_names"])
        else:
            out = self._build_gripper_opening_state(
                msg, group["raw_driver_joint"], group["bridge_opening_joint"]
            )
            if out is None:
                return
        group["bridge_state_pub"].publish(out)

    @staticmethod
    def _extract_command_value(msg, preferred_joint):
        names = list(msg.name)
        positions = list(msg.position)
        if names and preferred_joint in names:
            index = names.index(preferred_joint)
            if index < len(positions):
                return positions[index]
        if positions:
            return positions[0]
        return None

    def _build_passthrough_command(self, msg, expected_names):
        names = list(msg.name)
        positions = list(msg.position)
        output_names = []
        output_positions = []

        if names:
            allowed = set(expected_names)
            for index, name in enumerate(names):
                if index >= len(positions):
                    break
                if name in allowed:
                    output_names.append(name)
                    output_positions.append(float(positions[index]))
        else:
            for index, name in enumerate(expected_names):
                if index >= len(positions):
                    break
                output_names.append(name)
                output_positions.append(float(positions[index]))

        if not output_positions:
            return None

        out = JointState()
        self._stamp_like(msg, out)
        out.name = output_names
        out.position = output_positions
        return out

    def _build_gripper_command(self, msg, bridge_opening_joint, raw_driver_joint):
        opening = self._extract_command_value(msg, bridge_opening_joint)
        if opening is None:
            return None
        driver_position = self._driver_from_opening(opening)

        out = JointState()
        self._stamp_like(msg, out)
        out.name = [raw_driver_joint]
        out.position = [driver_position]
        return out

    def _on_bridge_command(self, msg, group_key):
        group = self._groups.get(group_key)
        if group is None:
            return

        if group["mode"] == "joint_passthrough":
            out = self._build_passthrough_command(msg, group["expected_joint_names"])
        else:
            out = self._build_gripper_command(
                msg, group["bridge_opening_joint"], group["raw_driver_joint"]
            )

        if out is None:
            return
        group["raw_command_pub"].publish(out)


def parse_args():
    parser = argparse.ArgumentParser(description="Isaac ROS joint republisher")
    parser.add_argument(
        "--bridge-prefix",
        type=str,
        default="/bridge",
        help="Prefix for republished state/command topics",
    )
    parser.add_argument(
        "--isaac-prefix",
        type=str,
        default="/isaac",
        help="Prefix for Isaac state/command topics",
    )
    parser.add_argument(
        "--gripper-open-position",
        type=float,
        default=0.0,
        help="Driver joint position that represents fully open",
    )
    parser.add_argument(
        "--gripper-closed-position",
        type=float,
        default=0.8,
        help="Driver joint position that represents fully closed",
    )
    parser.add_argument(
        "--gripper-invert",
        action="store_true",
        help="Invert gripper state normalization (swap open/closed convention)",
    )
    parser.add_argument(
        "--disable-browser-command-topics",
        action="store_true",
        help="Route /bridge commands to /isaac/*_commands instead of /isaac/browser/*_commands.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init(args=None)
    node = None
    try:
        node = JointRepublisher(
            bridge_prefix=args.bridge_prefix,
            isaac_prefix=args.isaac_prefix,
            gripper_open_position=args.gripper_open_position,
            gripper_closed_position=args.gripper_closed_position,
            gripper_invert=args.gripper_invert,
            disable_browser_command_topics=args.disable_browser_command_topics,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
