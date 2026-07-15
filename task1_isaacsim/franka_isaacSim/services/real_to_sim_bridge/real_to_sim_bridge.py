"""
real_to_sim_bridge.py — Cross-RMW bridge: real robot joint states → bridge command topics.

The real robots publish on a different ROS middleware and domain than the sim:
    Real robots: RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, ROS_DOMAIN_ID=53
    Sim stack:   RMW_IMPLEMENTATION=rmw_fastrtps_cpp,   ROS_DOMAIN_ID=0

A single rclpy process can only use one RMW, so this script bridges the gap by
spawning real_to_sim_bridge_reader.py as a subprocess with the real robot's RMW
environment, then reading its JSON-line output and republishing on the sim domain.

The bridge publishes to /bridge/* command topics.  The Republisher then forwards
these to the Isaac /isaac/browser/* command topics, keeping the Republisher as
the single interface between the sim and all other components.

    ┌─────────────────────────────────────────────────────────────────────┐
    │  real_to_sim_bridge.py  (rmw_fastrtps_cpp / domain 0)               │
    │                                                                     │
    │  ┌──────────────────────────────────────────────────────────────┐  │
    │  │  real_to_sim_bridge_reader.py  (rmw_cyclonedds_cpp / domain 53)│  │
    │  │  → subscribes to /left/joint_states etc.                     │  │
    │  │  → writes JSON lines to stdout                               │  │
    │  └──────────────────────────┬───────────────────────────────────┘  │
    │                             │ pipe                                  │
    │                             ▼                                       │
    │  BridgePublisher node reads JSON, publishes:                        │
    │    /bridge/left_joint_commands                                      │
    │    /bridge/right_joint_commands                                     │
    │    /bridge/left_robotiq_joint_commands  (optional)                  │
    │    /bridge/right_robotiq_joint_commands (optional)                  │
    └─────────────────────────────────────────────────────────────────────┘

Typical usage
-------------
    # Ensure the sim is running in position-controller mode:
    #   bash scripts/run_native_stream.sh --controller-mode position

    python3 scripts/real_to_sim_bridge.py \\
        --left-topic  /left/joint_states \\
        --right-topic /right/joint_states

    # Or use the convenience wrapper:
    bash scripts/run_real_to_sim_bridge.sh

    # Or via Docker Compose (follower profile):
    docker compose --profile follower up real_to_sim_bridge
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bridge_config_loader import BridgeConfig, load_bridge_config
from stack_config import apply_config_defaults


class BridgePublisher(Node):
    """ROS 2 node on the sim domain that publishes forwarded joint commands."""

    def __init__(self, config: BridgeConfig) -> None:
        super().__init__("real_to_sim_bridge")
        self._config = config
        self._pubs: dict[str, object] = {}
        # Build mapping dict from config
        self._output_topics = {m.name: m.output_topic for m in config.get_enabled_mappings()}
        self._arm_joint_names = {
            m.name: m.expected_joint_names 
            for m in config.get_arm_mappings()
        }
        self._gripper_joints = {
            m.name: m.gripper_opening_joint 
            for m in config.get_gripper_mappings() 
            if m.gripper_opening_joint
        }

    def _pub(self, side: str):
        if side not in self._pubs:
            topic = self._output_topics.get(side)
            if not topic:
                self.get_logger().warning(f"No output topic configured for {side}")
                return None
            self._pubs[side] = self.create_publisher(JointState, topic, 10)
            self.get_logger().info(f"Publishing {side} → {topic}")
        return self._pubs[side]

    def publish_arm(self, side: str, positions: list[float]) -> None:
        pub = self._pub(side)
        if not pub:
            return
        joint_names = self._arm_joint_names.get(side, [])
        num_joints = len(joint_names)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = positions[:num_joints]
        pub.publish(msg)

    def _normalize_opening(self, driver_position: float) -> float:
        """Convert raw driver-joint position to normalized opening (0=closed, 1=open)."""
        cfg = self._config.gripper
        stroke = cfg.driver_joint_open - cfg.driver_joint_closed
        if stroke <= 0:
            return 0.0
        close_ratio = (driver_position - cfg.driver_joint_closed) / stroke
        close_ratio = min(max(close_ratio, 0.0), 1.0)
        opening = close_ratio if not cfg.invert else (1.0 - close_ratio)
        return opening

    def publish_gripper(self, side: str, position: float) -> None:
        pub = self._pub(side)
        if not pub:
            return
        joint_name = self._gripper_joints.get(side)
        if not joint_name:
            self.get_logger().warning(f"No gripper joint configured for {side}")
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint_name]
        msg.position = [self._normalize_opening(position)]
        pub.publish(msg)


def _build_reader_env(config: BridgeConfig, cyclonedds_uri_override: str | None = None) -> dict[str, str]:
    """Build the subprocess environment for the reader (real robot domain)."""
    env = os.environ.copy()
    env["RMW_IMPLEMENTATION"] = config.real_robot.rmw_implementation
    env["ROS_DOMAIN_ID"] = str(config.real_robot.domain_id)
    # Strip FastDDS-specific vars so cyclonedds doesn't get confused
    env.pop("FASTDDS_BUILTIN_TRANSPORTS", None)
    # If a CycloneDDS config is provided, pass it to the reader
    cyclonedds_uri = cyclonedds_uri_override or config.real_robot.cyclonedds_uri
    if cyclonedds_uri:
        env["CYCLONEDDS_URI"] = cyclonedds_uri
    
    # Pass config file path to reader subprocess
    env["BRIDGE_CONFIG_PATH"] = str(Path(__file__).parent / "bridge_config.yaml")
    
    return env


def _pipe_loop(proc: subprocess.Popen, node: BridgePublisher) -> None:
    """Read JSON lines from the reader subprocess and dispatch to publishers."""
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
        # Check if it's an arm mapping (contains "arm" in name)
        if "arm" in side.lower():
            node.publish_arm(side, positions)
        # Check if it's a gripper mapping (contains "gripper" in name)
        elif "gripper" in side.lower():
            if positions:
                node.publish_gripper(side, float(positions[0]))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Cross-RMW bridge: real robot joint states → Isaac sim commands."
    )
    parser.add_argument("--config", default=None,
                        help="Path to stack_defaults.yaml (default: repo default)")
    parser.add_argument("--bridge-config", default=None,
                        help="Path to bridge_config.yaml (default: services/real_to_sim_bridge/bridge_config.yaml)")
    parser.add_argument("--cyclonedds-uri", default=None,
                        help="CYCLONEDDS_URI for the reader subprocess (override config file)")
    # Legacy CLI args kept for backward compatibility (will be deprecated)
    parser.add_argument("--left-topic", default=None,
                        help="[DEPRECATED] Use bridge_config.yaml instead")
    parser.add_argument("--right-topic", default=None,
                        help="[DEPRECATED] Use bridge_config.yaml instead")
    parser.add_argument("--left-gripper-topic", default=None,
                        help="[DEPRECATED] Use bridge_config.yaml instead")
    parser.add_argument("--right-gripper-topic", default=None,
                        help="[DEPRECATED] Use bridge_config.yaml instead")
    parser.add_argument("--gripper-joint-index", type=int, default=None,
                        help="[DEPRECATED] Use bridge_config.yaml instead")
    parser.add_argument("--real-rmw", default=None,
                        help="[DEPRECATED] Use bridge_config.yaml instead")
    parser.add_argument("--real-domain-id", type=int, default=None,
                        help="[DEPRECATED] Use bridge_config.yaml instead")
    return parser


def main() -> None:
    parser = build_parser()
    apply_config_defaults(parser, ["real_to_sim_bridge"])
    args = parser.parse_args()

    # Load bridge configuration
    config_path = args.bridge_config or (Path(__file__).parent / "bridge_config.yaml")
    try:
        config = load_bridge_config(config_path)
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        print(f"Expected config at: {config_path}", file=sys.stderr)
        sys.exit(1)

    reader_script = Path(__file__).parent / "real_to_sim_bridge_reader.py"
    reader_env = _build_reader_env(config, args.cyclonedds_uri)

    proc = subprocess.Popen(
        [sys.executable, str(reader_script)],
        stdout=subprocess.PIPE,
        env=reader_env,
        text=True,
    )

    rclpy.init()
    node = BridgePublisher(config)
    node.get_logger().info(f"Loaded bridge config: {config_path}")
    node.get_logger().info(
        f"Reader: {config.real_robot.rmw_implementation} / domain {config.real_robot.domain_id}"
    )
    node.get_logger().info(
        f"Publisher: {os.environ.get('RMW_IMPLEMENTATION', 'default')} / "
        f"domain {os.environ.get('ROS_DOMAIN_ID', '0')}"
    )
    node.get_logger().info(f"Enabled mappings: {len(config.get_enabled_mappings())}")
    for mapping in config.get_enabled_mappings():
        node.get_logger().info(f"  {mapping.name}: {mapping.input_topic} → {mapping.output_topic}")

    pipe_thread = threading.Thread(target=_pipe_loop, args=(proc, node), daemon=True)
    pipe_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        proc.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
