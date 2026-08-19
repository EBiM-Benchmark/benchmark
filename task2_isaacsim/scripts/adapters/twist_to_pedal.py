#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Twist -> pedal-token adapter for the Task 2 mobile base.

The teleop bridge (isaacsim_fr3duo_teleop_bridge_core.py) drives the mobile
base from the discrete ``/pedal/state`` tokens (``FWD``/``BACK``/``A``/``B``/
``A+C``/``B+C``/``NONE``; see config/topics.yaml, teleop.pedal_state). This
node lets any controller that emits a continuous ``geometry_msgs/Twist``
(e.g. a policy or scripted driver) command the base: it quantizes each twist
on ``teleop.base_twist_command`` to the nearest pedal token and republishes
it.

Quantization is faithful to the teleop data: the recorded base twists only
ever take the discrete pedal values (+-0.5 m/s linear, +-1.2 rad/s angular,
or zero), so dominant-axis selection with a dead-band reproduces them
exactly. Every incoming twist publishes a token, including ``NONE``
(explicit stop); if the twist stream stops entirely, the bridge's
``--pedal-timeout`` forces the base back to zero on its own.

Run it in any sourced ROS 2 environment that shares the ROS graph with the
bridge (matching ``RMW_IMPLEMENTATION``), e.g.::

    python3 task2_isaacsim/scripts/adapters/twist_to_pedal.py

The quantizer (``pedal_token``) is a pure function; this module imports
without rclpy so its unit tests run on a ROS-less host.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# scripts/ holds the shared topic-contract loader (topics.py).
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from topics import load_topics  # noqa: E402

# Half the 0.5 m/s pedal speed: small, noisy twists quantize to NONE
# instead of flapping between a token and a stop.
DEFAULT_PEDAL_THRESHOLD_MPS = 0.25


def pedal_token(vx: float, vy: float, wz: float, threshold: float) -> str:
    """Quantize a base twist to the bridge's pedal-token vocabulary.

    Dominant-axis selection with a dead-band. The angular axis is compared
    after normalizing by its higher magnitude (1.2/0.5) so a full-rate yaw
    competes fairly with a full-rate strafe; ties go to the earlier axis
    (forward > strafe > yaw).
    """
    wz_scaled = wz * (0.5 / 1.2)
    magnitudes = (abs(vx), abs(vy), abs(wz_scaled))
    peak = max(magnitudes)
    if peak < threshold:
        return "NONE"
    axis = magnitudes.index(peak)
    if axis == 0:
        return "FWD" if vx > 0 else "BACK"
    if axis == 1:
        return "A" if vy > 0 else "B"
    return "A+C" if wz > 0 else "B+C"


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--pedal-threshold",
        type=float,
        default=DEFAULT_PEDAL_THRESHOLD_MPS,
        help="Dead-band (m/s) below which the base command is NONE.",
    )
    args = parser.parse_args(argv)

    import rclpy  # noqa: PLC0415 - keep the module import-safe without ROS
    from geometry_msgs.msg import Twist  # noqa: PLC0415
    from rclpy.node import Node  # noqa: PLC0415
    from std_msgs.msg import String  # noqa: PLC0415

    topics = load_topics()["teleop"]

    class TwistToPedal(Node):
        def __init__(self) -> None:
            super().__init__("twist_to_pedal")
            self._pub = self.create_publisher(
                String, topics["pedal_state"], 10
            )
            self._sub = self.create_subscription(
                Twist, topics["base_twist_command"], self._on_twist, 10
            )
            self.get_logger().info(
                "twist_to_pedal started: "
                f"{topics['base_twist_command']} -> {topics['pedal_state']} "
                f"(threshold {args.pedal_threshold} m/s)"
            )

        def _on_twist(self, msg: Twist) -> None:
            out = String()
            out.data = pedal_token(
                msg.linear.x,
                msg.linear.y,
                msg.angular.z,
                args.pedal_threshold,
            )
            self._pub.publish(out)

    rclpy.init()
    node = TwistToPedal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
