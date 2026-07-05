"""Unified ROS 2 teleop bridge: subscribes to device-agnostic commands from
a standalone Teleop Node (keyboard/gamepad/VR, run as its own process/ROS
package - see teleop_state_publisher, intended for contribution to
github.com/EBiM-Benchmark/teleoperation) instead of reading local devices.

Deliberately dumb: this module does not know or care which physical device
produced the commands. The sim's own IK/grasp/base-drive code is identical
to the local keyboard/gamepad/VR modes - only the source of twist_cmd/
base_cmd/gripper intent changes, so teleop feel is unaffected by whether the
operator's device is local or fed in over ROS. GELLO is NOT part of this
contract; see input_gello.py for that separate, joint-space bridge.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from . import config, log


class RosTeleopBridge:
    """Background rclpy node; get_* methods return thread-safe snapshots,
    each None/zero once its topic has gone stale (publisher stopped)."""

    def __init__(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from std_msgs.msg import Float32, Float32MultiArray
        except ImportError as exc:
            raise RuntimeError(
                "rclpy is not available - ros_teleop needs a sourced ROS 2 environment "
                "(the eval Docker image, WSL2/Ubuntu with ROS 2, or a RoboStack conda env)."
            ) from exc

        self._rclpy = rclpy
        self._lock = threading.Lock()
        self._base_twist = np.zeros(4)  # local_x, local_y, spine, yaw
        self._arm_twist: dict[str, np.ndarray] = {
            "left": np.zeros(6),
            "right": np.zeros(6),
        }
        self._gripper: dict[str, float] = {"left": 0.0, "right": 0.0}
        self._last_update: dict[str, float] = {
            "base": 0.0,
            "left_arm": 0.0,
            "right_arm": 0.0,
            "left_gripper": 0.0,
            "right_gripper": 0.0,
        }

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("mujoco_ros_teleop_bridge")

        self.node.create_subscription(Twist, config.ROS_TELEOP_CMD_VEL_TOPIC, self._base_cb, 10)
        for side, ns in config.ROS_TELEOP_NAMESPACES.items():
            self.node.create_subscription(
                Twist,
                f"/{ns}/{config.ROS_TELEOP_ARM_TOPIC}",
                self._make_arm_cb(side),
                10,
            )
            self.node.create_subscription(
                Float32,
                f"/{ns}/{config.ROS_TELEOP_GRIPPER_TOPIC}",
                self._make_gripper_cb(side),
                10,
            )

        # frame feedback for screen-relative device mapping (keyboard/VR
        # publishers); pure information, carries no control authority
        self._Float32MultiArray = Float32MultiArray
        self._feedback_pub = self.node.create_publisher(
            Float32MultiArray,
            config.ROS_TELEOP_FEEDBACK_TOPIC,
            10,
        )
        self._next_feedback = 0.0

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._stop = False
        self._thread = threading.Thread(target=self._spin, daemon=True, name="ros-teleop-spin")
        self._thread.start()
        log(
            f"[ros_teleop] subscribed: {config.ROS_TELEOP_CMD_VEL_TOPIC}, "
            + ", ".join(f"/{ns}/{config.ROS_TELEOP_ARM_TOPIC}" for ns in config.ROS_TELEOP_NAMESPACES.values())
        )
        log("[ros_teleop] waiting for a Teleop Node (keyboard/gamepad/VR) to publish...")

    def _base_cb(self, msg) -> None:
        with self._lock:
            self._base_twist[:] = (
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z,
            )
            self._last_update["base"] = time.perf_counter()

    def _make_arm_cb(self, side: str):
        def cb(msg) -> None:
            with self._lock:
                self._arm_twist[side][:] = (
                    msg.linear.x,
                    msg.linear.y,
                    msg.linear.z,
                    msg.angular.x,
                    msg.angular.y,
                    msg.angular.z,
                )
                self._last_update[f"{side}_arm"] = time.perf_counter()

        return cb

    def _make_gripper_cb(self, side: str):
        def cb(msg) -> None:
            with self._lock:
                self._gripper[side] = float(msg.data)
                self._last_update[f"{side}_gripper"] = time.perf_counter()

        return cb

    def _spin(self) -> None:
        while not self._stop and self._rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _fresh(self, key: str) -> bool:
        return time.perf_counter() - self._last_update[key] <= config.ROS_TELEOP_DATA_TIMEOUT

    def get_base_cmd(self) -> np.ndarray:
        """(local_x, local_y, spine, yaw); zero once /cmd_vel goes stale."""
        with self._lock:
            if not self._fresh("base"):
                return np.zeros(4)
            return self._base_twist.copy()

    def get_arm_twist(self, side: str) -> np.ndarray | None:
        """6D Cartesian twist for one arm, or None once its topic is stale
        (caller should hold position, exactly like an unengaged VR clutch)."""
        with self._lock:
            if not self._fresh(f"{side}_arm"):
                return None
            return self._arm_twist[side].copy()

    def get_gripper_close(self, side: str) -> bool:
        """True if the latest fresh gripper_cmd requests closing."""
        with self._lock:
            if not self._fresh(f"{side}_gripper"):
                return False
            return self._gripper[side] > config.ROS_TELEOP_GRIPPER_CLOSE_ABOVE

    def publish_feedback(self, cam_azimuth_deg: float, robot_yaw_rad: float) -> None:
        """Rate-limited [azimuth, yaw] broadcast for screen-relative mapping."""
        now = time.perf_counter()
        if now < self._next_feedback:
            return
        self._next_feedback = now + 1.0 / config.ROS_TELEOP_FEEDBACK_HZ
        msg = self._Float32MultiArray()
        msg.data = [float(cam_azimuth_deg), float(robot_yaw_rad)]
        self._feedback_pub.publish(msg)

    def close(self) -> None:
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
        except Exception:
            pass
