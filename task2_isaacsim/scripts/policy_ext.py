# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Policy-evaluation extensions for the Task 2 teleop bridge.

Adds ROS ingress that the teleoperation bridge does not need but a policy
evaluation client does, without touching
isaacsim_fr3duo_teleop_bridge_core.py: objects here plug into
run_teleop_loop's tick_callbacks hook (bind(node) once before the loop,
tick(sim_time) once per iteration).

Currently provides SpineCommandCallback, which commands
franka_spine_vertical_joint over /isaac/spine_joint_commands (see
config/topics.yaml, bridge.spine.command) using the same
sensor_msgs/JointState convention as the arm and gripper command topics.
"""

from __future__ import annotations

import math

from sensor_msgs.msg import JointState
from topics import load_topics

SPINE_JOINT = "franka_spine_vertical_joint"


class SpineCommandCallback:
    """Routes JointState spine commands into the SpineKeyboardController.

    The controller's target/apply machinery works without the carb
    keyboard (headless sessions construct it with the keyboard
    unavailable), and run_teleop_loop already calls its apply() every
    tick, so this callback only has to update the target. Keyboard
    Up/Down keeps working; last writer wins, matching the arm topics'
    convention.
    """

    def __init__(self, spine_controller):
        self._spine_controller = spine_controller
        self._topic = load_topics()["bridge"]["spine"]["command"]

    def bind(self, node) -> None:
        node.create_subscription(JointState, self._topic, self._on_command, 10)
        print(
            f"Spine command ingress enabled: {self._topic} ({SPINE_JOINT})",
            flush=True,
        )

    def _on_command(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name != SPINE_JOINT or not math.isfinite(position):
                continue
            # _set_target logs each call; skip no-op updates so 30 Hz
            # policy streams do not spam the console.
            if abs(float(position) - self._spine_controller.target) > 1e-4:
                self._spine_controller._set_target(float(position))  # noqa: SLF001

    def tick(self, sim_time: float) -> None:  # noqa: ARG002 - hook signature
        """No per-tick work: the teleop loop applies the target itself."""
