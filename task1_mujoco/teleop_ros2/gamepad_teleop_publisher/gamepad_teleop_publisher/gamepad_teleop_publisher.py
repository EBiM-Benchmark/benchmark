"""Gamepad teleop publisher for the EBiM task-1 MuJoCo simulator,
standalone ROS 2 node.

Publishes device-agnostic Cartesian commands only — NO IK anywhere here;
the consumer solves motion with its own control stack. Uses SDL's
GameController mapping through pygame, so the physical layout is identical
on every OS and vendor (falls back to raw joystick indices for unmapped
pads, same as the simulator's local gamepad mode).

Mapping (identical to the sim's local gamepad mode):
  Share/Back      mobile base mode      L1 / R1   left / right arm mode
  left stick      base X/Y or TCP X/Y (robot heading frame via
                  /mujoco/teleop_feedback; falls back to yaw=0)
  right stick     base yaw or TCP yaw/pitch
  L2/R2           base: spine down/up; arm: TCP Z down/up
  D-pad left/right   TCP roll
  Circle/B close gripper   Cross/A open   click sticks = speed up/down

--pattern publishes a scripted left-arm twist + gripper-close for a few
seconds instead of reading a pad (self-test without any device).
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray

PUBLISH_HZ = 50.0
MOVE_SPEED = 4.0
ROT_SPEED = math.radians(540.0)
DEADZONE = 0.14
TRIGGER_DEAD = 0.08
FEEDBACK_TIMEOUT = 1.0
SIDES = ("left", "right")


def _dead(v: float, threshold: float = DEADZONE) -> float:
    return 0.0 if abs(v) < threshold else float(v)


class _Pad:
    """SDL GameController preferred; raw joystick fallback. Compact twin of
    the simulator's teleop/input_gamepad.py backends."""

    def __init__(self) -> None:
        import pygame

        self.pygame = pygame
        pygame.init()
        self.ctrl = None
        self.joy = None
        try:
            from pygame._sdl2 import controller as sdl2

            sdl2.init()
            for idx in range(sdl2.get_count()):
                if sdl2.is_controller(idx):
                    self.ctrl = sdl2.Controller(idx)
                    return
        except Exception:
            pass
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joy = pygame.joystick.Joystick(0)
            self.joy.init()

    @property
    def connected(self) -> bool:
        return self.ctrl is not None or self.joy is not None

    def pump(self) -> None:
        self.pygame.event.pump()

    def _axis(self, ctrl_axis, raw_idx, unsigned=False) -> float:
        if self.ctrl is not None:
            raw = float(self.ctrl.get_axis(ctrl_axis))
            return raw / 32767.0 if unsigned else raw / 32768.0
        if self.joy is not None and raw_idx < self.joy.get_numaxes():
            return float(self.joy.get_axis(raw_idx))
        return 0.0

    def sticks(self) -> tuple[float, float, float, float]:
        pg = self.pygame
        return (
            _dead(self._axis(pg.CONTROLLER_AXIS_LEFTX, 0)),
            _dead(self._axis(pg.CONTROLLER_AXIS_LEFTY, 1)),
            _dead(self._axis(pg.CONTROLLER_AXIS_RIGHTX, 2)),
            _dead(self._axis(pg.CONTROLLER_AXIS_RIGHTY, 3)),
        )

    def triggers(self) -> tuple[float, float]:
        pg = self.pygame
        if self.ctrl is not None:
            l2 = max(
                0.0,
                self._axis(pg.CONTROLLER_AXIS_TRIGGERLEFT, 4, unsigned=True),
            )
            r2 = max(
                0.0,
                self._axis(pg.CONTROLLER_AXIS_TRIGGERRIGHT, 5, unsigned=True),
            )
        else:
            l2 = (self._axis(0, 4) + 1.0) * 0.5
            r2 = (self._axis(0, 5) + 1.0) * 0.5
        return (
            0.0 if l2 < TRIGGER_DEAD else l2,
            0.0 if r2 < TRIGGER_DEAD else r2,
        )

    def dpad_x(self) -> int:
        pg = self.pygame
        if self.ctrl is not None:
            return int(self.ctrl.get_button(pg.CONTROLLER_BUTTON_DPAD_RIGHT)) - int(
                self.ctrl.get_button(pg.CONTROLLER_BUTTON_DPAD_LEFT)
            )
        if self.joy is not None and self.joy.get_numhats() > 0:
            return int(self.joy.get_hat(0)[0])
        return 0

    def buttons(self) -> set[str]:
        pg = self.pygame
        out = set()
        if self.ctrl is not None:
            table = {
                "mode_base": pg.CONTROLLER_BUTTON_BACK,
                "mode_left": pg.CONTROLLER_BUTTON_LEFTSHOULDER,
                "mode_right": pg.CONTROLLER_BUTTON_RIGHTSHOULDER,
                "close": pg.CONTROLLER_BUTTON_B,
                "open": pg.CONTROLLER_BUTTON_A,
                "speed_up": pg.CONTROLLER_BUTTON_LEFTSTICK,
                "speed_down": pg.CONTROLLER_BUTTON_RIGHTSTICK,
            }
            for name, btn in table.items():
                if self.ctrl.get_button(btn):
                    out.add(name)
        elif self.joy is not None:
            table = {
                "mode_base": (6,),
                "mode_left": (9, 4),
                "mode_right": (10, 5),
                "close": (1,),
                "open": (0,),
                "speed_down": (7,),
                "speed_up": (8,),
            }
            for name, ids in table.items():
                if any(i < self.joy.get_numbuttons() and self.joy.get_button(i) for i in ids):
                    out.add(name)
        return out


class GamepadTeleopPublisher:
    def __init__(self, node) -> None:
        self.node = node
        self.cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self.arm_pubs = {s: node.create_publisher(Twist, f"/{s}/teleop_cmd", 10) for s in SIDES}
        self.grip_pubs = {s: node.create_publisher(Float32, f"/{s}/gripper_cmd", 10) for s in SIDES}
        node.create_subscription(Float32MultiArray, "/mujoco/teleop_feedback", self._feedback_cb, 10)
        self._feedback = None
        self._feedback_time = 0.0
        self.mode = "right"
        self.speed = 1.0
        self.grip_close = {"left": 0.0, "right": 0.0}
        self._prev: set[str] = set()

    def _feedback_cb(self, msg) -> None:
        if len(msg.data) >= 2:
            self._feedback = (float(msg.data[0]), float(msg.data[1]))
            self._feedback_time = time.perf_counter()

    def _arm_xy_to_world(self, fwd_in: float, right_in: float) -> tuple[float, float]:
        yaw = 0.0
        if self._feedback is not None and time.perf_counter() - self._feedback_time <= FEEDBACK_TIMEOUT:
            yaw = self._feedback[1]
        fwd = (math.cos(yaw), math.sin(yaw))
        left = (-fwd[1], fwd[0])
        lx, ly = fwd_in, -right_in
        return lx * fwd[0] + ly * left[0], lx * fwd[1] + ly * left[1]

    def tick(self, pad: _Pad) -> None:
        pad.pump()
        held = pad.buttons()
        pressed = held - self._prev
        self._prev = set(held)

        if "mode_base" in pressed:
            self.mode = "base"
            self.node.get_logger().info("mode=base")
        elif "mode_left" in pressed:
            self.mode = "left"
            self.node.get_logger().info("mode=left")
        elif "mode_right" in pressed:
            self.mode = "right"
            self.node.get_logger().info("mode=right")
        if "speed_down" in pressed:
            self.speed = max(0.25, self.speed / 1.5)
            self.node.get_logger().info(f"speed x{self.speed:.2f}")
        if "speed_up" in pressed:
            self.speed = min(30.0, self.speed * 1.5)
            self.node.get_logger().info(f"speed x{self.speed:.2f}")
        if "close" in pressed and self.mode in SIDES:
            self.grip_close[self.mode] = 1.0
            self.node.get_logger().info(f"{self.mode} gripper close")
        if "open" in pressed and self.mode in SIDES:
            self.grip_close[self.mode] = 0.0
            self.node.get_logger().info(f"{self.mode} gripper open")

        lx, ly, rx, ry = pad.sticks()
        l2, r2 = pad.triggers()
        hx = pad.dpad_x()

        base = Twist()
        arm = {s: Twist() for s in SIDES}
        if self.mode == "base":
            base.linear.x = -ly
            base.linear.y = -lx  # stick left = robot's left, matching the sim
            base.linear.z = r2 - l2
            base.angular.z = rx
        else:
            side = self.mode
            move = MOVE_SPEED * self.speed
            rot = ROT_SPEED * self.speed
            wx, wy = self._arm_xy_to_world(-ly, lx)
            t = arm[side]
            t.linear.x = move * wx
            t.linear.y = move * wy
            t.linear.z = move * (r2 - l2)
            t.angular.z = -rx * rot
            t.angular.x = ry * rot
            t.angular.y = hx * rot

        self.cmd_vel_pub.publish(base)
        for s in SIDES:
            self.arm_pubs[s].publish(arm[s])
            self.grip_pubs[s].publish(Float32(data=self.grip_close[s]))


def _run_pattern(pub: GamepadTeleopPublisher, node, seconds: float) -> None:
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
    node = rclpy.create_node("gamepad_teleop_publisher")
    pub = GamepadTeleopPublisher(node)

    try:
        if cli.pattern is not None:
            _run_pattern(pub, node, cli.pattern)
            return
        try:
            pad = _Pad()
        except Exception as exc:
            node.get_logger().error(f"pygame unavailable ({exc}); use --pattern for a self-test")
            return
        if not pad.connected:
            node.get_logger().error("no gamepad connected; use --pattern for a self-test")
            return
        node.get_logger().info(
            "publishing /cmd_vel + /left|right/teleop_cmd + /left|right/gripper_cmd; "
            "Share=base L1/R1=arms Circle=close Cross=open stick-click=speed"
        )
        while rclpy.ok():
            pub.tick(pad)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / PUBLISH_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
