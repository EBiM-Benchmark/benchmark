#!/usr/bin/env python3
"""Run the Newton board-cable example with a small ROS2 state bridge."""

from __future__ import annotations

import argparse
import time
from typing import Iterable

import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sensor_msgs.msg import ChannelFloat32, PointCloud

import newton.examples

from run_board_cable import Example, _load_runtime_configs, _make_parser


def _add_ros_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--cable-point-topic",
        default="/cable/body_centers",
        help="ROS topic that publishes the simulated cable body centers as sensor_msgs/PointCloud.",
    )
    parser.add_argument(
        "--gripper-collision-box-topic",
        default="/cable/gripper_collision_boxes",
        help=(
            "ROS topic that publishes Newton gripper collision boxes as sensor_msgs/PointCloud. "
            "Each point is a box center; channels qx/qy/qz/qw/sx/sy/sz/finger/box encode orientation, size, and ids."
        ),
    )
    parser.add_argument(
        "--cable-frame-id",
        default="world",
        help="Frame id used for the cable point cloud.",
    )
    parser.add_argument(
        "--robotiq-finger-targets",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Create and drive four kinematic Robotiq finger collision bodies from a PointCloud target topic.",
    )
    parser.add_argument(
        "--robotiq-finger-target-topic",
        default="/isaac/robotiq_finger_targets",
        help="PointCloud topic carrying target poses for the four Robotiq finger collision bodies.",
    )
    parser.add_argument(
        "--robotiq-finger-size",
        type=float,
        nargs=3,
        default=(0.007, 0.010, 0.028),
        metavar=("X", "Y", "Z"),
        help="Default collision box size in meters for each Robotiq finger if the topic omits size channels.",
    )
    parser.add_argument(
        "--robotiq-finger-friction",
        type=float,
        default=0.8,
        help="Friction coefficient for Robotiq finger target collision boxes.",
    )
    parser.add_argument(
        "--publish-every-n-frames",
        type=int,
        default=1,
        help="Publish cable state every N Newton frames.",
    )
    parser.add_argument(
        "--real-time",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Sleep between frames to approximately match the configured Newton fps.",
    )


def _point_cloud_from_positions(
    positions_m: Iterable[Iterable[float]],
    *,
    frame_id: str,
    stamp,
) -> PointCloud:
    msg = PointCloud()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.points = [
        Point32(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        for point in positions_m
    ]
    return msg


def _normalize_quat_xyzw(q: Iterable[float]) -> np.ndarray:
    q_np = np.asarray(tuple(float(v) for v in q), dtype=np.float64)
    norm = float(np.linalg.norm(q_np))
    if norm <= 0.0:
        return np.asarray((0.0, 0.0, 0.0, 1.0), dtype=np.float64)
    return q_np / norm


class CableRosBridge(Node):
    def __init__(self, example: Example, args: argparse.Namespace):
        super().__init__("newton_cable_bridge")
        self._example = example
        self._args = args
        self._frame_id = str(args.cable_frame_id)
        self._publish_every_n_frames = max(int(args.publish_every_n_frames), 1)
        self._frame_index = 0
        self._robotiq_finger_targets: list[dict] | None = None

        self._point_pub = self.create_publisher(PointCloud, str(args.cable_point_topic), 10)
        self._gripper_box_pub = self.create_publisher(PointCloud, str(args.gripper_collision_box_topic), 10)
        self.create_subscription(PointCloud, str(args.robotiq_finger_target_topic), self._on_robotiq_finger_targets, 10)

        self.get_logger().info(
            f"Newton cable bridge publishing {args.cable_point_topic}; "
            f"gripper boxes {args.gripper_collision_box_topic}; "
            f"Robotiq finger targets {args.robotiq_finger_target_topic}"
        )

    def _on_robotiq_finger_targets(self, msg: PointCloud) -> None:
        channel_values = {channel.name: list(channel.values) for channel in msg.channels}

        def channel_value(name: str, index: int, default: float) -> float:
            values = channel_values.get(name)
            if values is None or index >= len(values):
                return float(default)
            return float(values[index])

        default_size = tuple(float(v) for v in getattr(self._args, "robotiq_finger_size", (0.007, 0.010, 0.028)))
        targets = []
        for index, point in enumerate(msg.points):
            targets.append(
                {
                    "position_m": (float(point.x), float(point.y), float(point.z)),
                    "quat_xyzw": _normalize_quat_xyzw(
                        (
                            channel_value("qx", index, 0.0),
                            channel_value("qy", index, 0.0),
                            channel_value("qz", index, 0.0),
                            channel_value("qw", index, 1.0),
                        )
                    ),
                    "size_m": (
                        channel_value("sx", index, default_size[0]),
                        channel_value("sy", index, default_size[1]),
                        channel_value("sz", index, default_size[2]),
                    ),
                    "finger_id": int(round(channel_value("finger", index, float(index)))),
                    "box_id": int(round(channel_value("box", index, 0.0))),
                }
            )
        self._robotiq_finger_targets = targets

    def _apply_robotiq_finger_targets_to_state(self, state) -> bool:
        body_ids = tuple(int(v) for v in getattr(self._example, "robotiq_finger_body_ids", ()))
        if not body_ids or not self._robotiq_finger_targets:
            return False
        body_q = state.body_q.numpy()
        body_qd = state.body_qd.numpy()
        applied = False
        for target in self._robotiq_finger_targets:
            finger_id = int(target.get("finger_id", 0))
            if finger_id < 0 or finger_id >= len(body_ids):
                continue
            body_id = body_ids[finger_id]
            position = np.asarray(target["position_m"], dtype=np.float32)
            quat = _normalize_quat_xyzw(target["quat_xyzw"]).astype(np.float32)
            body_q[body_id, :3] = position
            body_q[body_id, 3:] = quat
            body_qd[body_id, :] = 0.0
            applied = True
        if applied:
            state.body_q.assign(body_q)
            state.body_qd.assign(body_qd)
        return applied

    def apply_robotiq_finger_targets(self) -> bool:
        applied_0 = self._apply_robotiq_finger_targets_to_state(self._example.state_0)
        applied_1 = self._apply_robotiq_finger_targets_to_state(self._example.state_1)
        return bool(applied_0 or applied_1)

    def publish_cable_state(self) -> None:
        self._frame_index += 1
        if self._frame_index % self._publish_every_n_frames != 0:
            return

        cable_body_ids = np.asarray(self._example.import_result.cable_body_ids, dtype=np.int64)
        if cable_body_ids.size == 0:
            return

        body_q = self._example.state_0.body_q.numpy()
        positions_m = body_q[cable_body_ids, :3]
        msg = _point_cloud_from_positions(
            positions_m,
            frame_id=self._frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        self._point_pub.publish(msg)

    def _publish_robotiq_finger_collision_boxes(self) -> bool:
        if not self.apply_robotiq_finger_targets():
            return False
        body_ids = tuple(int(v) for v in getattr(self._example, "robotiq_finger_body_ids", ()))
        if not body_ids:
            return False
        body_q = self._example.state_0.body_q.numpy()
        default_size = tuple(float(v) for v in getattr(self._example, "robotiq_finger_size_m", (0.007, 0.010, 0.028)))
        size_by_finger = {
            int(target.get("finger_id", index)): tuple(float(v) for v in target.get("size_m", default_size))
            for index, target in enumerate(self._robotiq_finger_targets or [])
        }

        msg = PointCloud()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        channels = {name: ChannelFloat32(name=name) for name in ("qx", "qy", "qz", "qw", "sx", "sy", "sz", "finger", "box")}
        for finger_id, body_id in enumerate(body_ids):
            pose = np.asarray(body_q[body_id], dtype=np.float64)
            quat = _normalize_quat_xyzw(pose[3:7])
            size_m = size_by_finger.get(finger_id, default_size)
            msg.points.append(Point32(x=float(pose[0]), y=float(pose[1]), z=float(pose[2])))
            for channel_name, value in (
                ("qx", quat[0]),
                ("qy", quat[1]),
                ("qz", quat[2]),
                ("qw", quat[3]),
                ("sx", size_m[0]),
                ("sy", size_m[1]),
                ("sz", size_m[2]),
                ("finger", finger_id),
                ("box", 0.0),
            ):
                channels[channel_name].values.append(float(value))
        msg.channels = [channels[name] for name in ("qx", "qy", "qz", "qw", "sx", "sy", "sz", "finger", "box")]
        self._gripper_box_pub.publish(msg)
        return True

    def publish_gripper_collision_boxes(self) -> None:
        self._publish_robotiq_finger_collision_boxes()


def main() -> None:
    config_path, config_data, gripper_config_path, gripper_config = _load_runtime_configs()
    parser = _make_parser(config_path, config_data, gripper_config_path, gripper_config)
    _add_ros_args(parser)
    viewer, args = newton.examples.init(parser)
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
            example.render()
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
