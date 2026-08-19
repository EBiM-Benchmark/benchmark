#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Mock sim: synthetic publisher for offline recorder tests (no Isaac).

Publishes on the eval topic contract: /isaac/clock, a subset of cameras
(moving test pattern, rgb8), joint_states_full, object_poses, pad_points,
and scene_reset events; mirrors scene_reset_request -> scene_reset with
the real payload shape and rebases sim time on reset like the bridge.

  python3 mock_sim.py [--cameras head,eval_camera]
                      [--image-hz 30] [--rate 60] [--reset-every 0]
"""

import argparse
import contextlib
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, String

# Shared Task 2 topic contract (config/topics.yaml via scripts/topics.py,
# resolved relative to this file so /repo and host checkouts both work).
TASK2_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(TASK2_ROOT / "scripts"))
from topics import camera_topic, load_topics  # noqa: E402


class MockSim(Node):
    def __init__(self, args):
        super().__init__("mock_sim")
        topics = load_topics()
        table = dict(topics["cameras"]["robot"])
        table["eval_camera"] = topics["cameras"]["eval"]
        for key, entry in topics["cameras"].items():
            if key not in ("subtopics", "robot", "eval"):
                table[key] = entry
        self.args = args
        self.sim_time = 0.0
        self.reset_count = 0
        self._image_step = 0
        self.clock_pub = self.create_publisher(Clock, topics["clock"], 10)
        self.camera_pubs = {}
        self.camera_shapes = {}
        for key in args.cameras.split(","):
            key = key.strip()
            entry = table[key]
            topic = camera_topic(topics, entry["namespace"], "image")
            self.camera_pubs[key] = self.create_publisher(
                Image, topic, qos_profile_sensor_data
            )
            self.camera_shapes[key] = tuple(entry["shape"])
        ground_truth = topics["ground_truth"]
        self.joint_pub = self.create_publisher(
            JointState, topics["recording"]["joint_states_full"], 10
        )
        self.poses_pub = self.create_publisher(
            String, ground_truth["object_poses"], 10
        )
        self.pad_pub = self.create_publisher(
            Float32MultiArray, ground_truth["pad_points"], 10
        )
        self.reset_pub = self.create_publisher(
            String, ground_truth["scene_reset"], 10
        )
        self.create_subscription(
            String,
            ground_truth["scene_reset_request"],
            lambda _msg: self.do_reset(),
            10,
        )
        self.create_timer(1.0 / args.rate, self.step)
        self.create_timer(1.0 / args.image_hz, self.publish_images)
        if args.reset_every > 0:
            self.create_timer(args.reset_every, self.do_reset)

    def do_reset(self):
        self.reset_count += 1
        self.sim_time = 0.0  # the bridge rebases sim time on reset
        self.reset_pub.publish(
            String(
                data=json.dumps(
                    {
                        "event": "scene_reset",
                        "reset_index": self.reset_count,
                        "sim_time": self.sim_time,
                        "randomized": True,
                        "offsets": {"thermal_pad": [0.01, -0.005, 0.0]},
                    }
                )
            )
        )
        print(f"published scene_reset #{self.reset_count}", flush=True)

    def _stamp(self, msg):
        msg.header.stamp.sec = int(self.sim_time)
        msg.header.stamp.nanosec = int((self.sim_time % 1.0) * 1e9)

    def step(self):
        self.sim_time += 1.0 / self.args.rate
        clock = Clock()
        clock.clock.sec = int(self.sim_time)
        clock.clock.nanosec = int((self.sim_time % 1.0) * 1e9)
        self.clock_pub.publish(clock)
        joints = JointState()
        self._stamp(joints)
        joints.name = [f"joint{i}" for i in range(5)]
        joints.position = list(np.sin(self.sim_time + np.arange(5)))
        self.joint_pub.publish(joints)
        self.poses_pub.publish(
            String(
                data=json.dumps(
                    {
                        "sim_time": self.sim_time,
                        "objects": [
                            {"name": "thermal_pad", "pos": [1.75, 1.95, 0.8]}
                        ],
                    }
                )
            )
        )
        pad = Float32MultiArray()
        pad.data = [1.7, 1.9, 0.8, 1.8, 2.0, 0.8]
        self.pad_pub.publish(pad)

    def publish_images(self):
        self._image_step += 1
        for key, pub in self.camera_pubs.items():
            height, width, _ = self.camera_shapes[key]
            msg = Image()
            self._stamp(msg)
            msg.height = height
            msg.width = width
            msg.encoding = "rgb8"
            msg.step = width * 3
            array = np.zeros((height, width, 3), dtype=np.uint8)
            array[:, :, 0] = 40
            x = (self._image_step * 8) % max(width - 80, 1)
            y = hash(key) % max(height - 80, 1)
            array[y : y + 80, x : x + 80] = (0, 200, 120)
            msg.data = array.tobytes()
            pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cameras", default="head,eval_camera")
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--image-hz", type=float, default=30.0)
    parser.add_argument(
        "--reset-every",
        type=float,
        default=0.0,
        help="seconds between automatic resets (0 = only "
        "on scene_reset_request)",
    )
    args = parser.parse_args()
    rclpy.init()
    node = MockSim(args)
    print(f"mock sim up: cameras {sorted(node.camera_pubs)}", flush=True)
    with contextlib.suppress(KeyboardInterrupt):
        rclpy.spin(node)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
