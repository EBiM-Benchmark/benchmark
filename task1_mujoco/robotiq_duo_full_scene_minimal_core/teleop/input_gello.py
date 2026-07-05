"""GELLO input bridge: subscribes to the official EBiM competition GELLO ROS 2
topics (github.com/EBiM-Benchmark/teleoperation, franka_gello_state_publisher)
instead of talking to the Dynamixel hardware ourselves.

That publisher already reads the two OpenRB-150 controller boards, applies
assembly-offset/sign correction and multi-turn unwrapping, and clamps to the
real FR3 joint limits — so this side only has to subscribe and hand the
values to the control loop. This mirrors vr_openxr.py's shape: a background
thread owns the ROS 2 executor, the control loop reads thread-safe snapshots.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from . import config, log


class GelloBridge:
    """Background rclpy node; get_state() returns the latest thread-safe
    snapshot for each arm. Raises ImportError at construction if rclpy is
    unavailable (mirrors mnet_bridge.py's degrade-gracefully pattern)."""

    def __init__(self) -> None:
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float32, String
        except ImportError as exc:
            raise RuntimeError(
                "rclpy is not available - GELLO needs a sourced ROS 2 environment "
                "(the eval Docker image, WSL2/Ubuntu with ROS 2, or a RoboStack "
                "conda env). Teleop cannot start without it in --input gello mode."
            ) from exc

        self._rclpy = rclpy
        self._lock = threading.Lock()
        self._joint_pos: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._gripper: dict[str, float] = {"left": 1.0, "right": 1.0}
        self._last_update: dict[str, float] = {"left": 0.0, "right": 0.0}
        self._pedal_state = "NONE"
        self._pedal_update = 0.0

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("mujoco_gello_bridge")

        for side, ns in config.GELLO_NAMESPACES.items():
            self.node.create_subscription(
                JointState,
                f"/{ns}/{config.GELLO_JOINT_STATES_TOPIC}",
                self._make_joint_cb(side),
                10,
            )
            self.node.create_subscription(
                Float32,
                f"/{ns}/{config.GELLO_GRIPPER_TOPIC}",
                self._make_gripper_cb(side),
                10,
            )
        # USB foot pedal drives the mobile base in the GELLO workflow
        # (reference repo's pedal_state_publisher)
        self.node.create_subscription(
            String,
            config.PEDAL_STATE_TOPIC,
            self._pedal_cb,
            10,
        )

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._stop = False
        self._thread = threading.Thread(target=self._spin, daemon=True, name="gello-spin")
        self._thread.start()
        log(
            "[gello] subscribed: "
            + ", ".join(f"/{ns}/{config.GELLO_JOINT_STATES_TOPIC}" for ns in config.GELLO_NAMESPACES.values())
        )
        log("[gello] waiting for the franka_gello_state_publisher node to publish...")

    def _make_joint_cb(self, side: str):
        def cb(msg) -> None:
            with self._lock:
                self._joint_pos[side] = np.array(msg.position[:7], dtype=np.float64)
                self._last_update[side] = time.perf_counter()

        return cb

    def _make_gripper_cb(self, side: str):
        def cb(msg) -> None:
            with self._lock:
                self._gripper[side] = float(msg.data)

        return cb

    def _pedal_cb(self, msg) -> None:
        with self._lock:
            self._pedal_state = msg.data
            self._pedal_update = time.perf_counter()

    def _spin(self) -> None:
        while not self._stop and self._rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def get_state(self, side: str) -> tuple[np.ndarray | None, float]:
        """(7 joint angles or None if stale/never received, gripper fraction)."""
        with self._lock:
            pos = self._joint_pos[side]
            age = time.perf_counter() - self._last_update[side]
            gripper = self._gripper[side]
        if pos is None or age > config.GELLO_DATA_TIMEOUT:
            return None, gripper
        return pos.copy(), gripper

    def get_pedal_base_cmd(self) -> np.ndarray:
        """(local_x, local_y, spine, yaw) from the foot pedal; zeros when the
        pedal publisher is silent/stale or the state is unmapped."""
        with self._lock:
            state = self._pedal_state
            age = time.perf_counter() - self._pedal_update
        if age > config.PEDAL_DATA_TIMEOUT:
            return np.zeros(4)
        return np.array(config.PEDAL_BASE_COMMANDS.get(state, (0.0, 0.0, 0.0, 0.0)))

    def close(self) -> None:
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
        except Exception:
            pass
