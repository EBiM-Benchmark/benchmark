#!/usr/bin/env python3
"""Standalone ROS2 position controller for the Isaac Sim joint bridge.

Listens to position targets on the /bridge arm command topics (published by
the ros_republisher or the episode replay recorder) and republishes them as
PRIMARY position commands on the /isaac arm command topics.

In position-controller mode the bridge is launched with no
required_primary_controller, so commands here are accepted immediately
without an impedance layer.  The bridge's own command-smoothing
(command_smoothing_alpha) and max_position_step_rad still apply, giving
safe, rate-limited position tracking.

This mode intentionally avoids an external impedance controller service.
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Bridge topics (input) → Isaac primary topics (output)
_BRIDGE_TO_ISAAC = [
    ("/bridge/left_joint_commands", "/isaac/left_joint_commands"),
    ("/bridge/right_joint_commands", "/isaac/right_joint_commands"),
]


class JointPositionController(Node):
    """Forwards bridge position commands to Isaac primary command topics."""

    def __init__(self) -> None:
        super().__init__("joint_position_controller")
        self._pubs: dict[str, object] = {}
        self._subs: list = []

        for bridge_topic, isaac_topic in _BRIDGE_TO_ISAAC:
            pub = self.create_publisher(JointState, isaac_topic, 10)
            self._pubs[bridge_topic] = pub
            self._subs.append(
                self.create_subscription(
                    JointState,
                    bridge_topic,
                    lambda msg, src=bridge_topic: self._on_command(msg, src),
                    10,
                )
            )

        routes = ", ".join(
            f"{b} -> {i}" for b, i in _BRIDGE_TO_ISAAC
        )
        self.get_logger().info(
            f"Joint position controller started. Routing: {routes}"
        )

    def _on_command(self, msg: JointState, source_topic: str) -> None:
        pub = self._pubs.get(source_topic)
        if pub is None:
            return
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        # effort[] intentionally omitted — bridge routes as position command
        pub.publish(out)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Isaac ROS position controller passthrough"
    )
    parser.parse_args()

    rclpy.init(args=None)
    node = JointPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
