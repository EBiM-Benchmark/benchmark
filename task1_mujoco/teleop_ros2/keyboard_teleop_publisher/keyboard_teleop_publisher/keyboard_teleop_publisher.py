"""Keyboard teleop publisher (full arm + base semantics of the EBiM task-1
MuJoCo simulator), standalone ROS 2 node.

Publishes device-agnostic Cartesian commands only — NO IK anywhere here;
the consumer (the MuJoCo sim's --input ros_teleop, or a real-robot
controller) solves motion with its own control stack against its own live
robot state. Held keys are polled straight from the OS (same mechanism as
the sim's own local keyboard mode, teleop/input_keyboard.py): GetAsyncKeyState
on Windows, the X server's query_keymap on Linux/X11 (`python-xlib`
required). Both work regardless of which window has focus.

Key map (identical to the sim's local keyboard mode):
  7/8/9        select base / left arm / right arm
  arrows       base and arm modes: drive in SCREEN directions (uses the
               sim's /mujoco/teleop_feedback [cam_azimuth_deg,
               robot_yaw_rad]; falls back to robot-frame if the sim
               isn't running)
  Home/End     base turn left/right
  PageUp/Down  base: spine up/down; arm: TCP Z up/down; rotate: roll
  R            toggle the ARM between translate and rotate
  G / V        close / open the active arm's gripper
  - / =        speed down / up

--pattern publishes a scripted left-arm twist + gripper-close for a few
seconds instead of reading the keyboard (self-test without any device).
"""

from __future__ import annotations

import argparse
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray

PUBLISH_HZ = 50.0
MOVE_SPEED = 4.0  # m/s at speed x1.00 - matches the sim's config.MOVE_SPEED
ROT_SPEED = math.radians(540.0)  # matches the sim's config.ROT_SPEED
FEEDBACK_TIMEOUT = 1.0
SIDES = ("left", "right")

# X11 keysym names for everything we track
_MOTION_KEYS = (
    "Up",
    "Down",
    "Left",
    "Right",
    "Page_Up",
    "Page_Down",
    "Home",
    "End",
)
_DISCRETE_KEYS = ("7", "8", "9", "r", "g", "v", "space", "minus", "equal")


class _X11Keys:
    """Held-key polling straight from the X server (no window needed)."""

    def __init__(self) -> None:
        from Xlib import XK
        from Xlib.display import Display

        self._display = Display()
        self._codes: dict[str, int] = {}
        for name in _MOTION_KEYS + _DISCRETE_KEYS:
            keysym = XK.string_to_keysym(name)
            code = self._display.keysym_to_keycode(keysym)
            if code:
                self._codes[name] = code

    def down(self) -> set[str]:
        keymap = self._display.query_keymap()
        return {name for name, code in self._codes.items() if keymap[code // 8] & (1 << (code % 8))}


# Windows virtual-key codes for every tracked key (standard VK_* constants;
# same values as teleop/input_keyboard.py's _WIN_KEY_CODES for the motion
# keys, extended here with the discrete keys since this node has no viewer
# window/callback to source them from).
_WIN_VK_CODES = {
    "Up": 0x26,
    "Down": 0x28,
    "Left": 0x25,
    "Right": 0x27,
    "Page_Up": 0x21,
    "Page_Down": 0x22,
    "Home": 0x24,
    "End": 0x23,
    "7": 0x37,
    "8": 0x38,
    "9": 0x39,
    "r": 0x52,
    "g": 0x47,
    "v": 0x56,
    "space": 0x20,
    "minus": 0xBD,  # VK_OEM_MINUS
    "equal": 0xBB,  # VK_OEM_PLUS (unshifted '=' on a US keyboard)
}


class _WinKeys:
    """Held-key polling via GetAsyncKeyState (same mechanism as the sim's
    own Windows keyboard backend, teleop/input_keyboard.py)."""

    def __init__(self) -> None:
        import ctypes

        if os.name != "nt":
            raise RuntimeError("not running on Windows")
        self._get_key_state = ctypes.windll.user32.GetAsyncKeyState

    def down(self) -> set[str]:
        return {name for name, vk in _WIN_VK_CODES.items() if self._get_key_state(vk) & 0x8000}


class KeyboardTeleopPublisher:
    def __init__(self, node) -> None:
        self.node = node
        self.cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pubs = {s: node.create_publisher(Twist, f"/{s}/teleop_cmd", 10) for s in SIDES}
        self.grip_pubs = {s: node.create_publisher(Float32, f"/{s}/gripper_cmd", 10) for s in SIDES}
        node.create_subscription(Float32MultiArray, "/mujoco/teleop_feedback", self._feedback_cb, 10)
        self._feedback = None  # (azimuth_deg, robot_yaw_rad)
        self._feedback_time = 0.0

        self.mode = "right"
        self.rotate_mode = {"left": False, "right": False}
        self.speed = 1.0
        self.grip_close = {"left": 0.0, "right": 0.0}
        self._prev_down: set[str] = set()

    def _feedback_cb(self, msg) -> None:
        if len(msg.data) >= 2:
            self._feedback = (float(msg.data[0]), float(msg.data[1]))
            self._feedback_time = time.perf_counter()

    # ------------------------------------------------------------- frames
    def _screen_to_base_local(self, sx: float, sy: float) -> tuple[float, float]:
        """Same math as the sim's screen_to_base_local, fed by the feedback
        topic; robot-frame passthrough when the sim isn't publishing."""
        if self._feedback is None or time.perf_counter() - self._feedback_time > FEEDBACK_TIMEOUT:
            return sy, -sx  # robot frame fallback: up=forward, left=left
        # MuJoCo free camera (measured via mjv_updateScene's mjvGLCamera):
        # into-screen = [+cos(az), +sin(az)], screen-right = [+sin(az), -cos(az)]
        az = math.radians(self._feedback[0])
        fwd_h = (math.cos(az), math.sin(az))
        right_h = (math.sin(az), -math.cos(az))
        wx = right_h[0] * sx + fwd_h[0] * sy
        wy = right_h[1] * sx + fwd_h[1] * sy
        yaw = self._feedback[1]
        fwd = (math.cos(yaw), math.sin(yaw))
        left = (-fwd[1], fwd[0])
        return wx * fwd[0] + wy * fwd[1], wx * left[0] + wy * left[1]

    def _arm_xy_to_world(self, fwd_in: float, right_in: float) -> tuple[float, float]:
        """Screen-relative frame -> world xy (matches the sim's --arm-frame
        camera default: camera_xy_to_world), fed by the live camera azimuth
        from the feedback topic; identity (no rotation) until it arrives."""
        az = 0.0
        if self._feedback is not None and time.perf_counter() - self._feedback_time <= FEEDBACK_TIMEOUT:
            az = math.radians(self._feedback[0])
        fwd = (math.cos(az), math.sin(az))
        right = (math.sin(az), -math.cos(az))
        return fwd_in * fwd[0] + right_in * right[0], fwd_in * fwd[1] + right_in * right[1]

    # --------------------------------------------------------------- tick
    def tick(self, down: set[str]) -> None:
        pressed = down - self._prev_down
        self._prev_down = set(down)

        if "7" in pressed:
            self.mode = "base"
            self.node.get_logger().info("mode=base")
        elif "8" in pressed:
            self.mode = "left"
            self.node.get_logger().info("mode=left")
        elif "9" in pressed:
            self.mode = "right"
            self.node.get_logger().info("mode=right")
        if "minus" in pressed:
            self.speed = max(0.25, self.speed / 1.5)
            self.node.get_logger().info(f"speed x{self.speed:.2f}")
        if "equal" in pressed:
            self.speed = min(30.0, self.speed * 1.5)
            self.node.get_logger().info(f"speed x{self.speed:.2f}")
        if "r" in pressed and self.mode in SIDES:
            self.rotate_mode[self.mode] = not self.rotate_mode[self.mode]
            state = "ROTATE" if self.rotate_mode[self.mode] else "TRANSLATE"
            self.node.get_logger().info(f"{self.mode} arm {state} mode")
        if "g" in pressed and self.mode in SIDES:
            self.grip_close[self.mode] = 1.0
            self.node.get_logger().info(f"{self.mode} gripper close")
        if ("v" in pressed or "space" in pressed) and self.mode in SIDES:
            self.grip_close[self.mode] = 0.0
            self.node.get_logger().info(f"{self.mode} gripper open")

        base = Twist()
        arm = {s: Twist() for s in SIDES}

        if self.mode == "base":
            sy = (1.0 if "Up" in down else 0.0) - (1.0 if "Down" in down else 0.0)
            sx = (1.0 if "Right" in down else 0.0) - (1.0 if "Left" in down else 0.0)
            if sx or sy:
                base.linear.x, base.linear.y = self._screen_to_base_local(sx, sy)
            base.linear.z = (1.0 if "Page_Up" in down else 0.0) - (1.0 if "Page_Down" in down else 0.0)
            base.angular.z = (1.0 if "End" in down else 0.0) - (1.0 if "Home" in down else 0.0)
        else:
            side = self.mode
            move = MOVE_SPEED * self.speed
            rot = ROT_SPEED * self.speed
            t = arm[side]
            if self.rotate_mode[side]:
                t.angular.z = rot * ((1.0 if "Left" in down else 0.0) - (1.0 if "Right" in down else 0.0))
                t.angular.x = rot * ((1.0 if "Up" in down else 0.0) - (1.0 if "Down" in down else 0.0))
                t.angular.y = rot * ((1.0 if "Page_Up" in down else 0.0) - (1.0 if "Page_Down" in down else 0.0))
            else:
                fwd_in = (1.0 if "Up" in down else 0.0) - (1.0 if "Down" in down else 0.0)
                right_in = (1.0 if "Right" in down else 0.0) - (1.0 if "Left" in down else 0.0)
                wx, wy = self._arm_xy_to_world(fwd_in, right_in)
                t.linear.x = move * wx
                t.linear.y = move * wy
                t.linear.z = move * ((1.0 if "Page_Up" in down else 0.0) - (1.0 if "Page_Down" in down else 0.0))

        self.cmd_vel_pub.publish(base)
        for s in SIDES:
            self.arm_pubs[s].publish(arm[s])
            self.grip_pubs[s].publish(Float32(data=self.grip_close[s]))


def _run_pattern(pub: KeyboardTeleopPublisher, node, seconds: float) -> None:
    """Scripted self-test: left-arm +x twist and gripper close, no device."""
    node.get_logger().info(f"--pattern: left arm linear.x=0.05 + close for {seconds:.0f}s")
    end = time.perf_counter() + seconds
    while rclpy.ok() and time.perf_counter() < end:
        t = Twist()
        t.linear.x = 0.05
        pub.arm_pubs["left"].publish(t)
        pub.grip_pubs["left"].publish(Float32(data=1.0))
        pub.cmd_vel_pub.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / PUBLISH_HZ)


def main(args=None) -> None:
    if os.name == "nt":
        try:
            import ctypes

            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pattern",
        type=float,
        nargs="?",
        const=5.0,
        default=None,
        help="publish a scripted test sequence for N seconds (default 5) and exit",
    )
    cli, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node("keyboard_teleop_publisher")
    pub = KeyboardTeleopPublisher(node)

    try:
        if cli.pattern is not None:
            _run_pattern(pub, node, cli.pattern)
            return
        try:
            keys = _WinKeys() if os.name == "nt" else _X11Keys()
        except Exception as exc:
            node.get_logger().error(
                f"held-key polling unavailable ({exc}); this node needs "
                "GetAsyncKeyState on Windows or python-xlib on Linux/X11. "
                "Use --pattern for a self-test."
            )
            return
        node.get_logger().info(
            "publishing /cmd_vel + /left|right/teleop_cmd + /left|right/gripper_cmd; "
            "7/8/9 mode, arrows move, R rotate, G/V gripper, -/= speed"
        )
        while rclpy.ok():
            pub.tick(keys.down())
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / PUBLISH_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
