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
import sys
import time

import glfw
import mujoco.viewer
import numpy as np

import mujoco

from . import config, log
from .grasping import open_gripper
from .maths import mat_to_quat, rot_error, smooth_twist
from .mjutil import planar_body_axis
from .robot_arm import (
    apply_twist_ik,
    clamp_tcp_twist_for_contact,
    hard_hold_arm,
    seed_arm,
)
from .session import TeleopSession
from .vr_mapping import (
    ClutchState,
    screen_to_base_local,
    vr_to_screen_map,
    vr_to_world_map,
)

HELP = """
Unified ROS 2 teleop (--input ros_teleop)
  Consumes /cmd_vel (base), /left|right/teleop_cmd (arm Cartesian twist),
  /left|right/gripper_cmd from a separately-running Teleop Node (keyboard,
  gamepad - see teleop_ros2/). Applied through the exact same IK/grasp/
  base-drive code the local input modes use, so the feel is identical
  regardless of which device produced the commands.
  VR: /left|right/vr_hand (raw controller state from vr_teleop_publisher)
  drives the arms through local VR mode's own clutch/servo code - hold
  GRIP to move the mapped arm, TRIGGER close, A open, sticks drive the
  base. --facing / --vr-scale behave exactly like --input vr.
  F / H : report task finished / skipped (--mnet eval mode)
"""


def main(args) -> None:
    if sys.platform == "win32":
        try:
            import ctypes

            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

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

    # ---- VR-over-ROS state: raw controller poses arrive on <side>/vr_hand
    # and drive the arms through a verbatim copy of run_vr.py's clutch/servo
    # block (anchor at grip-engage, position servo against the LIVE sim TCP),
    # so the feel is identical to local VR mode; the publisher is a dumb
    # device reader. Hand->arm mirroring follows --facing like local VR.
    clutch = {"left": ClutchState(), "right": ClutchState()}
    prev_trigger = {"left": False, "right": False}
    prev_a = {"left": False, "right": False}
    facing_front = getattr(args, "facing", "front") == "front"
    vr_scale = float(getattr(args, "vr_scale", 1.4))
    hand_arm_pairs = (("left", "right"), ("right", "left")) if facing_front else (("left", "left"), ("right", "right"))
    vr_seen = [False]

    def vr_drive_arm(name: str, arm_name: str, hand, dt: float, cam) -> None:
        """One hand -> one arm, verbatim run_vr.py clutch/servo semantics."""
        arm = arms[arm_name]
        state = clutch[name]

        trig = hand.trigger > config.VR_TRIGGER_CLOSE
        if trig and not prev_trigger[name]:
            arm.close_ramp = True
            log(f"{arm_name} gripper closing (vr over ros)...")
        prev_trigger[name] = trig
        if hand.a and not prev_a[name]:
            open_gripper(data, arm)
            log(f"{arm_name} gripper open (vr over ros)")
        prev_a[name] = hand.a

        engage_now = hand.grip > (config.VR_GRIP_RELEASE if state.engaged else config.VR_GRIP_ENGAGE)
        if engage_now:
            if not state.engaged:
                state.engaged = True
                state.ctrl_pos = hand.pos.copy()
                state.ctrl_rot = hand.rot.copy()
                state.tcp_pos = data.xpos[arm.tcp_body].copy()
                state.tcp_rot = data.xmat[arm.tcp_body].reshape(3, 3).copy()
                if facing_front and cam is not None:
                    state.map = vr_to_screen_map(cam)
                else:
                    state.map = vr_to_world_map(data, session.base_body, args.robot_forward_axis)
                log(f"{arm_name} arm engaged (vr over ros)")
            m = state.map
            desired_pos = state.tcp_pos + m @ (hand.pos - state.ctrl_pos) * vr_scale
            rot_delta_w = m @ (hand.rot @ state.ctrl_rot.T) @ m.T
            desired_quat = mat_to_quat(rot_delta_w @ state.tcp_rot)
            twist = np.zeros(6)
            twist[:3] = config.VR_POS_GAIN * (desired_pos - data.xpos[arm.tcp_body])
            twist[3:] = config.VR_ROT_GAIN * rot_error(desired_quat, data.xmat[arm.tcp_body].reshape(3, 3))
            grasped = arm.grasped_body is not None
            if grasped:
                twist[:3] *= config.GRASPED_SPEED_SCALE
                twist[3:] *= max(config.GRASPED_SPEED_SCALE, 0.65)
                if arm.filtered_twist is None:
                    arm.filtered_twist = np.zeros(6)
                arm.filtered_twist[:] = smooth_twist(
                    arm.filtered_twist,
                    twist,
                    dt,
                    config.GRASPED_TWIST_FILTER_TAU,
                )
                twist = arm.filtered_twist.copy()
            twist = clamp_tcp_twist_for_contact(model, twist, grasped)
            apply_twist_ik(model, data, arm, twist)
            arm.was_command_active = True
        else:
            if state.engaged:
                state.engaged = False
                log(f"{arm_name} arm locked (vr over ros)")
            if arm.was_command_active:
                seed_arm(model, data, arm)
                arm.was_command_active = False
            hard_hold_arm(model, data, arm)

    def control_tick(dt: float, cam=None) -> None:
        while not code_queue.empty():
            code_display.show(code_queue.get_nowait())

        vr_hands = bridge.get_vr_hands()
        vr_active = {name: hand.valid for name, hand in vr_hands.items()}
        if any(vr_active.values()) and not vr_seen[0]:
            # same free-motion clamp loosening local VR mode applies, so
            # tracking keeps up with a real hand (grasped clamp unchanged)
            vr_seen[0] = True
            config.FREE_MAX_TCP_STEP = config.VR_FREE_MAX_TCP_STEP
            config.GRASPED_MAX_TCP_STEP = config.VR_GRASPED_MAX_TCP_STEP
            log("[ros_teleop] VR hand stream detected - VR motion clamps applied")

        # ---- base: VR sticks take over while a hand stream is live
        # (verbatim run_vr.py stick mapping); /cmd_vel otherwise
        if any(vr_active.values()):
            base_cmd = np.zeros(4)
            right, left = vr_hands["right"], vr_hands["left"]
            if right.valid:
                sx, sy = right.stick
                if abs(sx) > config.VR_STICK_DEAD or abs(sy) > config.VR_STICK_DEAD:
                    if facing_front and cam is not None:
                        lx, ly = screen_to_base_local(
                            cam, sx, sy, data, session.base_body, args.robot_forward_axis
                        )
                        base_cmd[0] += lx
                        base_cmd[1] += ly
                    else:
                        base_cmd[0] += sy
                        base_cmd[1] += -sx
            if left.valid:
                sx, sy = left.stick
                if abs(sx) > config.VR_STICK_DEAD:
                    base_cmd[3] += sx if facing_front else -sx
                if abs(sy) > config.VR_STICK_DEAD:
                    base_cmd[2] += sy
            if session.any_arm_grasped():
                base_cmd[0] *= config.GRASPED_BASE_SPEED_SCALE
                base_cmd[1] *= config.GRASPED_BASE_SPEED_SCALE
                base_cmd[3] *= config.GRASPED_BASE_SPEED_SCALE
                base_cmd[2] *= max(config.GRASPED_BASE_SPEED_SCALE, 0.35)
        else:
            base_cmd = bridge.get_base_cmd()
        session.base_driver.drive(base_cmd[0], base_cmd[1], base_cmd[2], base_cmd[3], dt)

        # arms driven by a live VR hand this tick (mapped via --facing)
        vr_driven_arms = set()
        for hand_name, arm_name in hand_arm_pairs:
            if vr_active[hand_name] or clutch[hand_name].engaged:
                vr_drive_arm(hand_name, arm_name, vr_hands[hand_name], dt, cam)
                vr_driven_arms.add(arm_name)

        for side, arm in arms.items():
            if side in vr_driven_arms:
                continue
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

        session.step_once(dt, follow_tcp_quat=bool(vr_driven_arms))

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

            control_tick(dt, viewer.cam)

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
