#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Publish six mobile-base commands from two three-button USB foot pedals."""

from __future__ import annotations

import argparse
import os
import select
import time
from pathlib import Path

from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


PEDAL_ONE_MAP = {
    ecodes.KEY_A: "FWD",
    ecodes.KEY_B: "BACK",
    ecodes.KEY_C: "A",
}
PEDAL_TWO_MAP = {
    ecodes.KEY_A: "B",
    ecodes.KEY_B: "A+C",
    ecodes.KEY_C: "B+C",
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pedal-one-device",
        default=os.environ.get("PEDAL_ONE_DEVICE", ""),
        help="evdev path for forward/back/left pedal; auto-detected when omitted",
    )
    parser.add_argument(
        "--pedal-two-device",
        default=os.environ.get("PEDAL_TWO_DEVICE", ""),
        help="evdev path for right/CCW/CW pedal; auto-detected when omitted",
    )
    parser.add_argument("--topic", default="/pedal/state")
    parser.add_argument("--publish-rate", type=float, default=30.0)
    parser.add_argument(
        "--no-grab",
        action="store_true",
        help="Do not exclusively grab the two input devices",
    )
    return parser.parse_args()


def _open_footswitch(path: str) -> InputDevice:
    device = InputDevice(path)
    if "FootSwitch" not in device.name or "Keyboard" not in device.name:
        device.close()
        raise RuntimeError(f"{path} is not a FootSwitch keyboard device")
    return device


def _discover_footswitches() -> list[str]:
    candidates: list[str] = []
    for path in sorted(Path("/dev/input/by-path").glob("*-event-kbd")):
        try:
            device = _open_footswitch(str(path))
        except (OSError, RuntimeError):
            continue
        candidates.append(str(path))
        device.close()
    return candidates


def _resolve_device_paths(pedal_one: str, pedal_two: str) -> tuple[str, str]:
    if pedal_one and pedal_two:
        paths = (pedal_one, pedal_two)
    else:
        discovered = _discover_footswitches()
        if len(discovered) != 2:
            raise RuntimeError(
                "Expected exactly two PCsensor FootSwitch keyboard devices under "
                f"/dev/input/by-path, found {len(discovered)}: {discovered}. "
                "Set PEDAL_ONE_DEVICE and PEDAL_TWO_DEVICE explicitly."
            )
        paths = (
            pedal_one or discovered[0],
            pedal_two or discovered[1],
        )

    resolved = tuple(str(Path(path).resolve()) for path in paths)
    if resolved[0] == resolved[1]:
        raise RuntimeError("Pedal one and pedal two resolve to the same input device")
    return paths


class DualPedalPublisher(Node):
    def __init__(self, topic: str):
        super().__init__("dual_pedal_state_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._last_state: str | None = None

    def publish_state(self, state: str, *, force: bool = False) -> None:
        state_changed = state != self._last_state
        if not force and not state_changed:
            return
        message = String()
        message.data = state
        self._publisher.publish(message)
        self._last_state = state
        if state_changed:
            self.get_logger().info(f"Published: {state}")


def _active_state(pressed: list[set[int]]) -> str:
    active = [
        token
        for key_map, keys in zip((PEDAL_ONE_MAP, PEDAL_TWO_MAP), pressed, strict=True)
        for key_code, token in key_map.items()
        if key_code in keys
    ]
    if len(active) == 1:
        return active[0]
    if len(active) > 1:
        return "STOP"
    return "NONE"


def main() -> None:
    args = _parse_args()
    if args.publish_rate <= 0.0:
        raise ValueError("--publish-rate must be greater than zero")

    pedal_paths = _resolve_device_paths(args.pedal_one_device, args.pedal_two_device)
    devices = [_open_footswitch(path) for path in pedal_paths]
    grabbed: list[InputDevice] = []

    rclpy.init()
    node = DualPedalPublisher(args.topic)
    node.get_logger().info(
        "Pedal 1: "
        f"{pedal_paths[0]} [left=FWD, middle=BACK, right=LEFT]"
    )
    node.get_logger().info(
        "Pedal 2: "
        f"{pedal_paths[1]} [left=RIGHT, middle=CCW, right=CW]"
    )

    try:
        if not args.no_grab:
            for device in devices:
                device.grab()
                grabbed.append(device)

        pressed = [set(), set()]
        period_sec = 1.0 / args.publish_rate
        next_publish_sec = 0.0
        node.publish_state("NONE", force=True)

        while rclpy.ok():
            now_sec = time.monotonic()
            timeout_sec = max(0.0, min(0.05, next_publish_sec - now_sec))
            readable, _, _ = select.select(devices, [], [], timeout_sec)

            for device in readable:
                device_index = devices.index(device)
                key_map = PEDAL_ONE_MAP if device_index == 0 else PEDAL_TWO_MAP
                for event in device.read():
                    if event.type != ecodes.EV_KEY or event.code not in key_map:
                        continue
                    if event.value == 0:
                        pressed[device_index].discard(event.code)
                    elif event.value in (1, 2):
                        pressed[device_index].add(event.code)

            state = _active_state(pressed)
            now_sec = time.monotonic()
            state_changed = state != node._last_state
            if state_changed or now_sec >= next_publish_sec:
                node.publish_state(state, force=not state_changed)
                next_publish_sec = now_sec + period_sec
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_state("NONE", force=True)
        for device in grabbed:
            try:
                device.ungrab()
            except OSError:
                pass
        for device in devices:
            device.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
