#!/usr/bin/env python3
"""Run the Newton board-cable example with a small ROS2 state bridge."""

from __future__ import annotations

import argparse
import time
from typing import Iterable

import numpy as np
import rclpy
import warp as wp
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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
        "--robotiq-finger-max-linear-speed",
        type=float,
        default=0.35,
        help="Maximum linear speed in m/s used while smoothing each Robotiq finger collision body target.",
    )
    parser.add_argument(
        "--robotiq-finger-max-angular-speed",
        type=float,
        default=2.0,
        help="Maximum angular speed in rad/s used while smoothing each Robotiq finger collision body target.",
    )
    parser.add_argument(
        "--robotiq-finger-target-timeout",
        type=float,
        default=0.5,
        help="Freeze Robotiq finger collision bodies after this many seconds without a fresh target.",
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


def _quat_xyzw_conjugate(q: np.ndarray) -> np.ndarray:
    return np.asarray((-q[0], -q[1], -q[2], q[3]), dtype=np.float64)


def _quat_xyzw_multiply(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return np.asarray(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ),
        dtype=np.float64,
    )


def _quat_xyzw_angle(lhs: np.ndarray, rhs: np.ndarray) -> float:
    dot = abs(float(np.dot(lhs, rhs)))
    return 2.0 * float(np.arccos(np.clip(dot, -1.0, 1.0)))


def _quat_xyzw_slerp(lhs: np.ndarray, rhs: np.ndarray, fraction: float) -> np.ndarray:
    fraction = float(np.clip(fraction, 0.0, 1.0))
    dot = float(np.dot(lhs, rhs))
    if dot < 0.0:
        rhs = -rhs
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return _normalize_quat_xyzw(lhs + fraction * (rhs - lhs))
    angle = float(np.arccos(dot))
    sin_angle = float(np.sin(angle))
    lhs_weight = np.sin((1.0 - fraction) * angle) / sin_angle
    rhs_weight = np.sin(fraction * angle) / sin_angle
    return _normalize_quat_xyzw(lhs_weight * lhs + rhs_weight * rhs)


def _quat_xyzw_angular_velocity(previous: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
    delta = _normalize_quat_xyzw(_quat_xyzw_multiply(current, _quat_xyzw_conjugate(previous)))
    if delta[3] < 0.0:
        delta = -delta
    vector_norm = float(np.linalg.norm(delta[:3]))
    if vector_norm < 1.0e-9 or dt <= 0.0:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * float(np.arctan2(vector_norm, np.clip(delta[3], -1.0, 1.0)))
    return delta[:3] * (angle / (vector_norm * dt))


@wp.kernel
def _write_kinematic_body_targets(
    body_ids: wp.array(dtype=wp.int32),
    pose_values: wp.array(dtype=wp.float32),
    velocity_values: wp.array(dtype=wp.float32),
    body_q: wp.array(dtype=wp.transform),
    body_qd: wp.array(dtype=wp.spatial_vector),
):
    index = wp.tid()
    body_id = body_ids[index]
    pose_offset = index * 7
    velocity_offset = index * 6
    body_q[body_id] = wp.transform(
        wp.vec3(
            pose_values[pose_offset],
            pose_values[pose_offset + 1],
            pose_values[pose_offset + 2],
        ),
        wp.quat(
            pose_values[pose_offset + 3],
            pose_values[pose_offset + 4],
            pose_values[pose_offset + 5],
            pose_values[pose_offset + 6],
        ),
    )
    # Newton spatial vectors store angular velocity first, then linear velocity.
    body_qd[body_id] = wp.spatial_vector(
        wp.vec3(
            velocity_values[velocity_offset + 3],
            velocity_values[velocity_offset + 4],
            velocity_values[velocity_offset + 5],
        ),
        wp.vec3(
            velocity_values[velocity_offset],
            velocity_values[velocity_offset + 1],
            velocity_values[velocity_offset + 2],
        ),
    )


class CableRosBridge(Node):
    def __init__(self, example: Example, args: argparse.Namespace):
        super().__init__("newton_cable_bridge")
        self._example = example
        self._args = args
        self._frame_id = str(args.cable_frame_id)
        self._publish_every_n_frames = max(int(args.publish_every_n_frames), 1)
        self._frame_index = 0
        self._robotiq_finger_targets: list[dict] | None = None
        self._robotiq_finger_target_stamp_ns = -1
        self._robotiq_finger_target_received_at: float | None = None
        self._finger_body_ids = tuple(int(v) for v in getattr(example, "robotiq_finger_body_ids", ()))
        self._finger_max_linear_speed = float(args.robotiq_finger_max_linear_speed)
        self._finger_max_angular_speed = float(args.robotiq_finger_max_angular_speed)
        self._finger_target_timeout = float(args.robotiq_finger_target_timeout)
        if self._finger_max_linear_speed <= 0.0:
            raise ValueError("--robotiq-finger-max-linear-speed must be positive")
        if self._finger_max_angular_speed <= 0.0:
            raise ValueError("--robotiq-finger-max-angular-speed must be positive")
        if self._finger_target_timeout <= 0.0:
            raise ValueError("--robotiq-finger-target-timeout must be positive")

        self._applied_finger_positions: np.ndarray | None = None
        self._applied_finger_quaternions: np.ndarray | None = None
        self._finger_body_ids_wp = None
        self._finger_pose_values_wp = None
        self._finger_velocity_values_wp = None
        if self._finger_body_ids:
            device = example.state_0.body_q.device
            self._finger_body_ids_wp = wp.array(self._finger_body_ids, dtype=wp.int32, device=device)
            self._finger_pose_values_wp = wp.zeros(len(self._finger_body_ids) * 7, dtype=wp.float32, device=device)
            self._finger_velocity_values_wp = wp.zeros(len(self._finger_body_ids) * 6, dtype=wp.float32, device=device)

        self._point_pub = self.create_publisher(PointCloud, str(args.cable_point_topic), 10)
        self._gripper_box_pub = self.create_publisher(PointCloud, str(args.gripper_collision_box_topic), 10)
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._finger_target_sub = self.create_subscription(
            PointCloud,
            str(args.robotiq_finger_target_topic),
            self._on_robotiq_finger_targets,
            target_qos,
        )

        self.get_logger().info(
            f"Newton cable bridge publishing {args.cable_point_topic}; "
            f"gripper boxes {args.gripper_collision_box_topic}; "
            f"Robotiq finger targets {args.robotiq_finger_target_topic}; "
            f"max speeds=({self._finger_max_linear_speed:.3f} m/s, "
            f"{self._finger_max_angular_speed:.3f} rad/s); "
            f"timeout={self._finger_target_timeout:.3f} s"
        )

    def _on_robotiq_finger_targets(self, msg: PointCloud) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns > 0 and stamp_ns <= self._robotiq_finger_target_stamp_ns:
            return
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
        if stamp_ns > 0:
            self._robotiq_finger_target_stamp_ns = stamp_ns
        self._robotiq_finger_target_received_at = time.monotonic()

    def apply_robotiq_finger_targets(self, state, dt: float) -> bool:
        if not self._finger_body_ids or not self._robotiq_finger_targets:
            return False
        target_by_finger = {
            int(target.get("finger_id", index)): target
            for index, target in enumerate(self._robotiq_finger_targets)
        }
        if any(finger_id not in target_by_finger for finger_id in range(len(self._finger_body_ids))):
            return False

        target_positions = np.asarray(
            [target_by_finger[index]["position_m"] for index in range(len(self._finger_body_ids))],
            dtype=np.float64,
        )
        target_quaternions = np.asarray(
            [
                _normalize_quat_xyzw(target_by_finger[index]["quat_xyzw"])
                for index in range(len(self._finger_body_ids))
            ],
            dtype=np.float64,
        )
        if self._applied_finger_positions is None or self._applied_finger_quaternions is None:
            next_positions = target_positions.copy()
            next_quaternions = target_quaternions.copy()
            linear_velocities = np.zeros_like(next_positions)
            angular_velocities = np.zeros_like(next_positions)
        else:
            previous_positions = self._applied_finger_positions
            previous_quaternions = self._applied_finger_quaternions
            target_is_fresh = (
                self._robotiq_finger_target_received_at is not None
                and time.monotonic() - self._robotiq_finger_target_received_at <= self._finger_target_timeout
            )
            if not target_is_fresh:
                target_positions = previous_positions
                target_quaternions = previous_quaternions

            next_positions = previous_positions.copy()
            next_quaternions = previous_quaternions.copy()
            max_position_step = self._finger_max_linear_speed * float(dt)
            max_rotation_step = self._finger_max_angular_speed * float(dt)
            for index in range(len(self._finger_body_ids)):
                position_delta = target_positions[index] - previous_positions[index]
                distance = float(np.linalg.norm(position_delta))
                if distance > 0.0:
                    position_fraction = min(1.0, max_position_step / distance)
                    next_positions[index] += position_fraction * position_delta

                angle = _quat_xyzw_angle(previous_quaternions[index], target_quaternions[index])
                rotation_fraction = 1.0 if angle <= 0.0 else min(1.0, max_rotation_step / angle)
                next_quaternions[index] = _quat_xyzw_slerp(
                    previous_quaternions[index],
                    target_quaternions[index],
                    rotation_fraction,
                )

            linear_velocities = (next_positions - previous_positions) / float(dt)
            angular_velocities = np.asarray(
                [
                    _quat_xyzw_angular_velocity(previous_quaternions[index], next_quaternions[index], float(dt))
                    for index in range(len(self._finger_body_ids))
                ],
                dtype=np.float64,
            )

        self._applied_finger_positions = next_positions
        self._applied_finger_quaternions = next_quaternions
        pose_values = np.concatenate((next_positions, next_quaternions), axis=1).astype(np.float32, copy=False)
        velocity_values = np.concatenate((linear_velocities, angular_velocities), axis=1).astype(np.float32, copy=False)
        self._finger_pose_values_wp.assign(pose_values.reshape(-1))
        self._finger_velocity_values_wp.assign(velocity_values.reshape(-1))
        wp.launch(
            _write_kinematic_body_targets,
            dim=len(self._finger_body_ids),
            inputs=[
                self._finger_body_ids_wp,
                self._finger_pose_values_wp,
                self._finger_velocity_values_wp,
                state.body_q,
                state.body_qd,
            ],
            device=state.body_q.device,
        )
        return True

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
        body_ids = tuple(int(v) for v in getattr(self._example, "robotiq_finger_body_ids", ()))
        if not body_ids or self._applied_finger_positions is None:
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
    example.pre_substep_callback = node.apply_robotiq_finger_targets
    num_frames = int(getattr(args, "num_frames", 0) or 0)
    frame_dt = float(example.frame_dt)
    frame_count = 0

    try:
        while rclpy.ok():
            frame_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
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
