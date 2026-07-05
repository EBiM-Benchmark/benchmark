"""Unified ROS 2 teleop run loop (--input ros_teleop).

Consumes device-agnostic Cartesian commands from a standalone Teleop Node
(keyboard/gamepad/VR reading their own devices and publishing over ROS 2 —
see teleop_state_publisher, intended for contribution to
github.com/EBiM-Benchmark/teleoperation) instead of reading local devices.

By design this applies commands through the EXACT SAME functions the local
keyboard/gamepad/VR modes use (apply_twist_ik, clamp_tcp_twist_for_contact,
smooth_twist, hard_hold_arm/seed_arm, the close_ramp gripper state machine),
copied verbatim from run_desktop.py's per-arm tail — so teleop feel is
identical whether the operator's device is local or fed in over ROS; only
the SOURCE of twist_cmd/base_cmd/gripper intent changes. GELLO is handled
separately (input_gello.py/run_gello.py) since it is joint-space and
IK-free by nature, not part of this Cartesian contract.
"""

from __future__ import annotations

import math
import time

import glfw
import mujoco.viewer
import numpy as np

import mujoco

from . import config, log
from .grasping import open_gripper
from .maths import smooth_twist
from .mjutil import planar_body_axis
from .robot_arm import (
    apply_twist_ik,
    clamp_tcp_twist_for_contact,
    hard_hold_arm,
    seed_arm,
)
from .session import TeleopSession

HELP = """
Unified ROS 2 teleop (--input ros_teleop)
  Consumes /cmd_vel (base), /left|right/teleop_cmd (arm Cartesian twist),
  /left|right/gripper_cmd from a separately-running Teleop Node (keyboard,
  gamepad, or VR - see teleop_state_publisher). Applied through the exact
  same IK/grasp/base-drive code the local input modes use, so the feel is
  identical regardless of which device produced the commands.
  F / H : report task finished / skipped (--mnet eval mode)
"""


def main(args) -> None:
    log(HELP)
    try:
        from .input_ros_teleop import RosTeleopBridge

        bridge = RosTeleopBridge()
    except Exception as exc:
        log(f"[ros_teleop] unavailable: {exc}")
        return

    session = TeleopSession(args)
    model, data, arms = session.model, session.data, session.arms

    mnet = None
    if getattr(args, "mnet", False):
        try:
            from .mnet_bridge import MnetBridge

            mnet = MnetBridge(model, args)
        except Exception as exc:
            log(f"[mnet] eval bridge disabled: {exc}")

    from .code_display import CodeDisplay, stdin_command_listener

    code_display = CodeDisplay(model)
    if getattr(args, "display_code", None):
        code_display.show(args.display_code)
    code_queue = stdin_command_listener()

    # edge-triggered gripper: fire close_ramp/open_gripper only on the tick
    # gripper_cmd crosses the threshold, not every tick it stays past it
    # (update_grasp owns the close_ramp -> holding transition; re-arming it
    # every tick would undo that and the gripper would overdrive forever)
    was_closing = {"left": False, "right": False}

    def control_tick(dt: float) -> None:
        while not code_queue.empty():
            code_display.show(code_queue.get_nowait())

        base_cmd = bridge.get_base_cmd()
        session.base_driver.drive(base_cmd[0], base_cmd[1], base_cmd[2], base_cmd[3], dt)

        for side, arm in arms.items():
            incoming = bridge.get_arm_twist(side)
            if incoming is None:
                twist_cmd = np.zeros(6)
                command_active = False
            else:
                twist_cmd = incoming.copy()
                command_active = float(np.linalg.norm(twist_cmd)) > config.TWIST_DEAD

            # verbatim copy of run_desktop.py's per-arm tail: identical feel
            if arm.grasped_body is not None:
                if command_active:
                    twist_cmd[:3] *= config.GRASPED_SPEED_SCALE
                    twist_cmd[3:] *= max(config.GRASPED_SPEED_SCALE, 0.65)
                if arm.filtered_twist is None:
                    arm.filtered_twist = np.zeros(6, dtype=np.float64)
                arm.filtered_twist[:] = smooth_twist(
                    arm.filtered_twist,
                    twist_cmd,
                    dt,
                    config.GRASPED_TWIST_FILTER_TAU,
                )
                twist_cmd = arm.filtered_twist.copy()
                command_active = float(np.linalg.norm(twist_cmd)) > config.TWIST_DEAD
            elif arm.filtered_twist is not None:
                arm.filtered_twist[:] = 0.0

            if command_active:
                twist_cmd = clamp_tcp_twist_for_contact(model, twist_cmd, arm.grasped_body is not None)
                apply_twist_ik(model, data, arm, twist_cmd)
                arm.was_command_active = True
            else:
                if arm.was_command_active:
                    seed_arm(model, data, arm)
                hard_hold_arm(model, data, arm)
                arm.was_command_active = False

            closing_now = bridge.get_gripper_close(side)
            if closing_now and not was_closing[side]:
                arm.close_ramp = True
                log(f"{side} gripper closing (ros_teleop)...")
            if not closing_now and was_closing[side]:
                open_gripper(data, arm)
                log(f"{side} gripper open (ros_teleop)")
            was_closing[side] = closing_now

        session.step_once(dt)

    if args.no_viewer:
        for _ in range(400):
            control_tick(model.opt.timestep)
        bridge.close()
        if mnet is not None:
            mnet.close()
        log("init/smoke ok (no Teleop Node expected in --no-viewer mode)")
        return

    from .input_keyboard import KeyboardInput

    keyboard_discrete = KeyboardInput()

    loop_dt = 1.0 / config.LOOP_HZ
    render_dt = 0.0 if args.render_hz <= 0 else 1.0 / args.render_hz
    next_render = time.perf_counter()
    last = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard_discrete.key_callback) as viewer:
        session.setup_viewer_cam(viewer)
        if mnet is not None:
            mnet.ensure_renderer()
        while viewer.is_running():
            start = time.perf_counter()
            dt = min(max(start - last, model.opt.timestep), 1.0 / 60.0)
            last = start

            for key in keyboard_discrete.drain():
                if mnet is not None and key == glfw.KEY_F:
                    mnet.report_finished()
                elif mnet is not None and key == glfw.KEY_H:
                    mnet.report_skipped()

            control_tick(dt)

            # frame feedback for screen-relative device mapping in the
            # publishers (keyboard/VR); rate-limited inside the bridge
            fwd = planar_body_axis(data, session.base_body, args.robot_forward_axis)
            bridge.publish_feedback(float(viewer.cam.azimuth), math.atan2(fwd[1], fwd[0]))

            if mnet is not None:
                mnet.maybe_publish(data, viewer.cam)
                cfg = mnet.consume_board_config()
                if cfg is not None and getattr(args, "mnet_randomize", False):
                    from .mnet_board import apply_board_config

                    apply_board_config(session, cfg)

            if render_dt <= 0.0 or time.perf_counter() >= next_render:
                viewer.sync()
                next_render = time.perf_counter() + render_dt
            sleep = loop_dt - (time.perf_counter() - start)
            if sleep > 0:
                time.sleep(sleep)

    bridge.close()
    if mnet is not None:
        mnet.close()
