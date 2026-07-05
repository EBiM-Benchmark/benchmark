#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Thin client that triggers the eval service and prints the result.

Convenience alternative to ``ros2 service call`` -- run it from inside the
eval_task2 container (so the ROS env is sourced), e.g.::

    python3 / workspace / scripts / evaluation / task2 / client.py
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def call_evaluate(service_name: str, timeout_sec: float = 10.0) -> int:
    rclpy.init()
    node = Node("eval_camera_evaluate_client")
    client = node.create_client(Trigger, service_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().error(f"Service not available: {service_name}")
            return 1

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        result = future.result()
        if result is None:
            node.get_logger().error(
                "Service call timed out / returned no result"
            )
            return 1

        node.get_logger().info(f"success={result.success}")
        print(result.message)
        return 0 if result.success else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Trigger the task2 eval service"
    )
    parser.add_argument(
        "--service-name",
        type=str,
        default="/isaac/eval_camera/evaluate",
        help="Trigger service name (default: /isaac/eval_camera/evaluate)",
    )
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    args = parser.parse_args(argv)
    return call_evaluate(args.service_name, args.timeout_sec)


if __name__ == "__main__":
    sys.exit(main())
