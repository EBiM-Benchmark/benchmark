"""
real_robot_follower.py — Mirror real robot joint states → bridge command topics.

Uses the same cross-RMW subprocess architecture as real_to_sim_bridge.py
so that it works even when the real robots and the sim stack use different
ROS middlewares / domain IDs:

    Real robots: RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, ROS_DOMAIN_ID=53
    Sim stack:   RMW_IMPLEMENTATION=rmw_fastrtps_cpp,   ROS_DOMAIN_ID=0

A reader subprocess (real_to_sim_bridge_reader.py) runs with the real-robot
RMW environment, subscribes to real robot JointState topics, and writes
JSON lines to stdout.  The parent process reads those lines on the sim
domain and publishes commands on the /bridge/* topics.

Compared to real_to_sim_bridge.py this follower adds:
  - stale-timeout detection: sides without fresh data are held
  - optional fixed-rate publishing (--publish-rate)

Typical usage
-------------
    python3 scripts/real_robot/real_robot_follower.py \\
        --left-topic /left/joint_states \\
        --right-topic /right/joint_states

    # Or use the convenience wrapper:
    bash scripts/run_real_robot_follower.sh

Configuration via stack_defaults.yaml (section: real_robot_follower) provides
default values for all CLI flags.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from isaac_bridge_constants import LEFT_JOINTS, RIGHT_JOINTS
from stack_config import apply_config_defaults

# ── Bridge command topics (Republisher forwards these to Isaac) ───────────────
BRIDGE_LEFT_CMD_TOPIC = "/bridge/left_joint_commands"
BRIDGE_RIGHT_CMD_TOPIC = "/bridge/right_joint_commands"
BRIDGE_LEFT_GRIPPER_CMD_TOPIC = "/bridge/left_robotiq_joint_commands"
BRIDGE_RIGHT_GRIPPER_CMD_TOPIC = "/bridge/right_robotiq_joint_commands"

_LEFT_GRIPPER_OPENING_JOINT = "left_robotiq_opening"
_RIGHT_GRIPPER_OPENING_JOINT = "right_robotiq_opening"

# Gripper calibration (must match Republisher settings).
_GRIPPER_OPEN_POSITION = 0.0
_GRIPPER_CLOSED_POSITION = 0.8

_NUM_ARM_JOINTS = 7


class RealRobotFollower(Node):
    """ROS 2 node on the sim domain that publishes mirrored joint commands."""

    def __init__(
        self,
        stale_timeout_s: float,
        publish_rate_hz: float,
    ) -> None:
        super().__init__("real_robot_follower")

        self._stale_timeout_s = stale_timeout_s
        self._lock = threading.Lock()

        # Latest received positions keyed by side ("left" / "right")
        self._latest_arm: dict[str, tuple[list[float], float]] = {}
        self._latest_gripper: dict[str, tuple[float, float]] = {}

        # Publishers created lazily on first message for each side.
        self._arm_pubs: dict[str, object] = {}
        self._gripper_pubs: dict[str, object] = {}

        # Timer-based publishing (fixed rate).
        period = 1.0 / publish_rate_hz if publish_rate_hz > 0 else None
        if period is not None:
            self.create_timer(period, self._timer_publish)
            self.get_logger().info(f"Publish rate: {publish_rate_hz:.1f} Hz (timer mode)")
        else:
            self.get_logger().info("Publish rate: pass-through (publish on every received message)")
        self._publish_period_s = period

    # ── Called from the pipe-reading thread ───────────────────────────────────

    def on_arm_data(self, side: str, positions: list[float]) -> None:
        now = time.monotonic()
        with self._lock:
            self._latest_arm[side] = (positions, now)
        if self._publish_period_s is None:
            self._publish_arm(side, positions)

    def on_gripper_data(self, side: str, position: float) -> None:
        now = time.monotonic()
        with self._lock:
            self._latest_gripper[side] = (position, now)
        if self._publish_period_s is None:
            self._publish_gripper(side, position)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _ensure_arm_pub(self, side: str):
        if side not in self._arm_pubs:
            topic = BRIDGE_LEFT_CMD_TOPIC if side == "left" else BRIDGE_RIGHT_CMD_TOPIC
            self._arm_pubs[side] = self.create_publisher(JointState, topic, 10)
            self.get_logger().info(f"{side} arm → {topic}")
        return self._arm_pubs[side]

    def _ensure_gripper_pub(self, side: str):
        if side not in self._gripper_pubs:
            topic = BRIDGE_LEFT_GRIPPER_CMD_TOPIC if side == "left" else BRIDGE_RIGHT_GRIPPER_CMD_TOPIC
            self._gripper_pubs[side] = self.create_publisher(JointState, topic, 10)
            self.get_logger().info(f"{side} gripper → {topic}")
        return self._gripper_pubs[side]

    def _publish_arm(self, side: str, positions: list[float]) -> None:
        pub = self._ensure_arm_pub(side)
        joint_names = LEFT_JOINTS if side == "left" else RIGHT_JOINTS
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = positions[:_NUM_ARM_JOINTS]
        pub.publish(msg)

    def _publish_gripper(self, side: str, position: float) -> None:
        pub = self._ensure_gripper_pub(side)
        opening_joint = _LEFT_GRIPPER_OPENING_JOINT if side == "left" else _RIGHT_GRIPPER_OPENING_JOINT
        opening = self._normalize_opening(position)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [opening_joint]
        msg.position = [opening]
        pub.publish(msg)

    @staticmethod
    def _normalize_opening(driver_position: float) -> float:
        """Convert raw driver-joint position to normalized opening (0=closed, 1=open)."""
        stroke = _GRIPPER_CLOSED_POSITION - _GRIPPER_OPEN_POSITION
        if stroke <= 0:
            return 0.0
        close_ratio = (driver_position - _GRIPPER_OPEN_POSITION) / stroke
        close_ratio = min(max(close_ratio, 0.0), 1.0)
        return 1.0 - close_ratio

    # ── Timer (fixed-rate mode) ───────────────────────────────────────────────

    def _timer_publish(self) -> None:
        now = time.monotonic()
        with self._lock:
            arm_snapshot = dict(self._latest_arm)
            gripper_snapshot = dict(self._latest_gripper)

        for side, (positions, recv_time) in arm_snapshot.items():
            if now - recv_time > self._stale_timeout_s:
                self.get_logger().warning(
                    f"[{side} arm] No data for {now - recv_time:.1f}s — holding"
                )
                continue
            self._publish_arm(side, positions)

        for side, (position, recv_time) in gripper_snapshot.items():
            if now - recv_time > self._stale_timeout_s:
                continue
            self._publish_gripper(side, position)


# ── Subprocess pipe reader ────────────────────────────────────────────────────

def _build_reader_env(args: argparse.Namespace) -> dict[str, str]:
    """Build the subprocess environment for the reader (real robot domain)."""
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = args.real_rmw
    env["ROS_DOMAIN_ID"] = str(args.real_domain_id)
    env.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    if args.cyclonedds_uri:
        env["CYCLONEDDS_URI"] = args.cyclonedds_uri
    for attr, key in [
        ("left_topic", "BRIDGE_LEFT_TOPIC"),
        ("right_topic", "BRIDGE_RIGHT_TOPIC"),
        ("left_gripper_topic", "BRIDGE_LEFT_GRIPPER_TOPIC"),
        ("right_gripper_topic", "BRIDGE_RIGHT_GRIPPER_TOPIC"),
    ]:
        value = getattr(args, attr, None) or ""
        env[key] = value
    env["BRIDGE_GRIPPER_JOINT_INDEX"] = str(args.gripper_joint_index)
    return env


def _pipe_loop(proc: subprocess.Popen, node: RealRobotFollower) -> None:
    """Read JSON lines from the reader subprocess and dispatch to the node."""
    assert proc.stdout is not None
    for raw in proc.stdout:
        line = raw.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            node.get_logger().warning(f"Unrecognised reader output: {line!r}")
            continue
        side = record.get("side", "")
        positions = record.get("positions", [])
        if side in ("left", "right"):
            node.on_arm_data(side, positions)
        elif side in ("left_gripper", "right_gripper"):
            if positions:
                node.on_gripper_data(side, float(positions[0]))


# ── Entry point ───────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Mirror real robot joint states into Isaac sim commands."
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Path to stack_defaults.yaml (default: repo default)",
    )
    parser.add_argument(
        "--left-topic",
        default=None,
        help="JointState topic for the real left arm (7 DOF, positions by index)",
    )
    parser.add_argument(
        "--right-topic",
        default=None,
        help="JointState topic for the real right arm (7 DOF, positions by index)",
    )
    parser.add_argument(
        "--left-gripper-topic",
        default=None,
        help="JointState topic for the real left gripper (optional)",
    )
    parser.add_argument(
        "--right-gripper-topic",
        default=None,
        help="JointState topic for the real right gripper (optional)",
    )
    parser.add_argument(
        "--gripper-joint-index",
        type=int,
        default=0,
        help="Index into the gripper JointState to use as the driver joint position (default: 0)",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=0.5,
        help="Seconds without a message before a side is considered stale (default: 0.5)",
    )
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=60.0,
        help="Publish rate in Hz; set 0 to publish on every received message (default: 60)",
    )
    parser.add_argument(
        "--real-rmw",
        default="rmw_cyclonedds_cpp",
        help="RMW implementation of the real robots (default: rmw_cyclonedds_cpp)",
    )
    parser.add_argument(
        "--real-domain-id",
        type=int,
        default=100,
        help="ROS_DOMAIN_ID of the real robots (default: 100)",
    )
    parser.add_argument(
        "--cyclonedds-uri",
        default=None,
        help="CYCLONEDDS_URI for the reader subprocess",
    )
    return parser


def main():
    parser = build_parser()
    apply_config_defaults(parser, ["real_robot_follower"])
    args = parser.parse_args()

    if not args.left_topic and not args.right_topic:
        parser.error("Provide at least --left-topic or --right-topic.")

    # Parent process runs on the sim domain (fastrtps / domain 0).
    rclpy.init()
    node = RealRobotFollower(
        stale_timeout_s=args.stale_timeout,
        publish_rate_hz=args.publish_rate,
    )

    # Spawn the reader subprocess on the real-robot domain.
    reader_script = Path(__file__).parent.parent.parent / "services" / "real_to_sim_bridge" / "real_to_sim_bridge_reader.py"
    reader_env = _build_reader_env(args)
    proc = subprocess.Popen(
        [sys.executable, str(reader_script)],
        stdout=subprocess.PIPE,
        stderr=None,
        env=reader_env,
        text=True,
        bufsize=1,
    )
    node.get_logger().info(
        f"Reader subprocess started (PID {proc.pid}): "
        f"RMW={args.real_rmw}, domain={args.real_domain_id}"
    )

    pipe_thread = threading.Thread(target=_pipe_loop, args=(proc, node), daemon=True)
    pipe_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        proc.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
