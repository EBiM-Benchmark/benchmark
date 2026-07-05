#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Keyboard -> mobile-base adapter for the Task 1 teleoperation pipeline.

The EBiM ``teleoperation`` repository's ``keyboard_state_publisher`` publishes the
pressed key on ``/keyboard/state`` as a ``std_msgs/String`` with one of::

    w  -> forward       s  -> backward
    a  -> strafe left   d  -> strafe right
    q  -> turn left     e  -> turn right

The Isaac Lab Newton bridge (``isaaclab_fr3duo_newton_bridge.py``) drives the
mobile base from ``/pedal/state`` tokens. This node remaps the keyboard tokens to
the pedal vocabulary the bridge understands so a plain keyboard can drive the
simulated base:

    w -> FWD    s -> BACK
    a -> A      d -> B
    q -> A+C    e -> B+C

The keyboard publisher only emits messages while a key is pressed (auto-repeat),
so releasing every key stops the stream and the bridge's ``--pedal-timeout``
forces the base command back to ``NONE`` (i.e. the robot stops). No explicit
"stop" key is required.

Run it in any sourced ROS 2 environment that shares the ROS graph with the
teleoperation publisher and the bridge (matching ``RMW_IMPLEMENTATION``), e.g.::

    python3 task1_isaacsim/scripts/adapters/keyboard_to_base.py
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Keyboard token (from keyboard_state_publisher) -> bridge /pedal/state token.
KEY_TO_PEDAL = {
    "w": "FWD",
    "s": "BACK",
    "a": "A",
    "d": "B",
    "q": "A+C",
    "e": "B+C",
}


class KeyboardToBase(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_to_base")
        self._pub = self.create_publisher(String, "/pedal/state", 10)
        self._sub = self.create_subscription(String, "/keyboard/state", self._on_key, 10)
        self.get_logger().info(
            "keyboard_to_base started: /keyboard/state (w/a/s/d/q/e) -> /pedal/state"
        )

    def _on_key(self, msg: String) -> None:
        token = KEY_TO_PEDAL.get(msg.data.strip().lower())
        if token is None:
            return
        out = String()
        out.data = token
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardToBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
