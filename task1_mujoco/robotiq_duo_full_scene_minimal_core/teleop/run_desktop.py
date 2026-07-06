"""Keyboard + gamepad run loop (the desktop input methods).

Mode-based teleop: one of {base, left arm, right arm} is active at a time;
idle arms are hard-held. Arm sticks/WASD are operator-relative (mirror
teleop); the frame is selectable with --arm-frame.
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
from .haptics import ContactPulser
from .input_gamepad import Gamepad
from .input_keyboard import KeyboardInput, held_key_backend
from .maths import mat_to_quat, rot_error, smooth_twist
from .mjutil import (
    camera_xy_to_world,
    contact_pair_summary,
    dump_contacts,
    robot_local_xy_to_world,
)
from .robot_arm import (
    Arm,
    apply_twist_ik,
    clamp_tcp_twist_for_contact,
    hard_hold_arm,
    seed_arm,
)
from .session import TeleopSession
from .vr_mapping import screen_to_base_local

HELP = """
Duo-FR3 IK + velocity-actuator teleop

Gamepad (same layout on every OS via the SDL controller mapping):
  Share/Back: mobile base mode    L1 / R1: left / right arm mode
  Left stick: base X/Y, or active TCP relative to your camera view
              (stick left = operator's left, mirror teleop)
  Right stick: base yaw or active TCP yaw/pitch
  L2/R2: spine down/up in base mode; TCP Z down/up in arm mode
  TCP rotation is always active: right stick X/Y = yaw/pitch, D-pad left/right = roll
  Circle/B: close active gripper with pad-cable contact stop
  Cross/A: open active gripper
  Triangle/Y: print rotation-control reminder
  Click left/right stick: faster/slower (matches VR)

Keyboard (motion = arrows + PageUp/PageDown only, letters stay free):
  7/8/9: base/left/right mode
  base mode: arrows = drive in SCREEN directions (up = away, left = left),
             Home/End = turn left/right, PageUp/PageDown = spine up/down
  arm modes: arrows = TCP X/Y, PageUp/PageDown = TCP Z up/down
  R: toggle the ARM between translate and rotate
     (rotate: arrows = yaw/pitch, PageUp/PageDown = roll)
  G: close with pad-cable contact stop
  V or Space: open
  -/=: slower/faster
  B: print all current contacts (debug)
  N: toggle collision-geometry display
  F / H: report task finished / skipped (--mnet eval mode)
"""


def main(args) -> None:
    # runtime tunable overrides (read as config.X at call time everywhere)
    config.MOVE_SPEED = float(args.move_speed)
    config.ROT_SPEED = math.radians(float(args.rot_speed_deg))
    config.JOINT_VEL_LIMIT = float(args.joint_vel_limit)
    config.ORIENTATION_LOCK_GAIN = float(args.ori_lock_gain)

    if sys.platform == "win32":
        try:
            import ctypes

            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    log(HELP)
    session = TeleopSession(args)
    model, data, arms = session.model, session.data, session.arms

    joy = Gamepad()
    if args.gamepad:
        joy.connect()
    log(f"[gamepad] {joy.message}")
    log(f"[keyboard] held-key backend: {held_key_backend()}")
    pulser = ContactPulser()
    if joy.connected:
        if joy.rest:
            log("[gamepad] axis rest " + " ".join(f"{k}:{v:+.2f}" for k, v in joy.rest.items()))
        log("[gamepad] mapping: Share=base L1/R1=arms Circle=close Cross=open Triangle=help stick-click=speed up/down")
    log(f"[base] control={args.base_control}")

    mode = ["right"]
    speed_scale = session.speed_scale
    viewer_ref: list = [None]

    def active_arm() -> Arm:
        return arms["right" if mode[0] == "right" else "left"]

    def set_mode(new_mode: str) -> None:
        if new_mode not in ("base", "left", "right"):
            return
        mode[0] = new_mode
        for arm in arms.values():
            hard_hold_arm(model, data, arm)
            seed_arm(model, data, arm)
        log(f"mode={new_mode}")

    def toggle_rotate(arm: Arm) -> None:
        arm.rotate_mode = not arm.rotate_mode
        if not arm.rotate_mode:
            mujoco.mj_forward(model, data)
            arm.translate_lock_quat = mat_to_quat(data.xmat[arm.tcp_body].reshape(3, 3).copy())
            arm.target_quat = arm.translate_lock_quat.copy()
        if arm.rotate_mode:
            log(f"{arm.name} arm ROTATE mode: arrows = yaw/pitch, PageUp/PageDown = roll (R again for translate)")
        else:
            log(f"{arm.name} arm TRANSLATE mode: arrows = X/Y, PageUp/PageDown = Z")

    def bump_speed(factor: float) -> None:
        speed_scale[0] = float(np.clip(speed_scale[0] * factor, 0.25, 30.0))
        log(f"speed x{speed_scale[0]:.2f}")

    def arm_xy_to_world(fwd: float, right: float) -> np.ndarray:
        """Map arm translation input (forward, right) to world xy.

        base frame: forward = robot heading, right = robot's right — stable
        no matter where the free camera looks. camera frame: screen axes.
        """
        if args.arm_frame == "camera" and viewer_ref[0] is not None:
            return camera_xy_to_world(viewer_ref[0].cam, np.array([fwd, right], dtype=np.float64))
        # robot_local_xy_to_world's second component is the robot's LEFT
        return robot_local_xy_to_world(
            data,
            session.base_body,
            np.array([fwd, -right], dtype=np.float64),
            args.robot_forward_axis,
        )

    def handle_discrete_key(key: int) -> bool:
        arm = active_arm()
        if key == glfw.KEY_7:
            set_mode("base")
        elif key == glfw.KEY_8:
            set_mode("left")
        elif key == glfw.KEY_9:
            set_mode("right")
        elif key == glfw.KEY_R:
            if mode[0] == "base":
                log("R toggles ARM rotation - switch to an arm first (8/9); base turning is Home/End")
            else:
                toggle_rotate(arm)
        elif key == glfw.KEY_G:
            arm.close_ramp = True
        elif key in (glfw.KEY_V, glfw.KEY_SPACE):
            open_gripper(data, arm)
        elif key in (glfw.KEY_MINUS, glfw.KEY_KP_SUBTRACT):
            bump_speed(1.0 / 1.5)
        elif key in (glfw.KEY_EQUAL, glfw.KEY_KP_ADD):
            bump_speed(1.5)
        elif key == glfw.KEY_C:
            seed_arm(model, data, arm)
            log(f"{arm.name} target synced")
        elif key == glfw.KEY_B:
            dump_contacts(model, data)
        elif key == glfw.KEY_N:
            if viewer_ref[0] is not None:
                viewer_ref[0].opt.geomgroup[3] = 0 if viewer_ref[0].opt.geomgroup[3] else 1
                log(f"collision geoms {'shown' if viewer_ref[0].opt.geomgroup[3] else 'hidden'}")
        elif key == glfw.KEY_F:
            if mnet is not None:
                mnet.report_finished()
            else:
                log("[mnet] not running (start with --mnet to report task status)")
        elif key == glfw.KEY_H:
            if mnet is not None:
                mnet.report_skipped()
            else:
                log("[mnet] not running (start with --mnet to report task status)")
        else:
            return False
        return True

    keyboard = KeyboardInput()

    # ManipulationNet eval bridge (optional): publishes a sim camera over ROS 2
    # and reports task finished/skipped to the mnet client (F / H keys)
    mnet = None
    if getattr(args, "mnet", False):
        try:
            from .mnet_bridge import MnetBridge

            mnet = MnetBridge(model, args)
        except Exception as exc:
            log(f"[mnet] eval bridge disabled: {exc}")

    # one-time-code plate (works with or without ROS)
    from .code_display import CodeDisplay, stdin_command_listener

    code_display = CodeDisplay(model)
    if getattr(args, "display_code", None):
        code_display.show(args.display_code)
    code_queue = stdin_command_listener()
    if mnet is not None:
        log("[code] when the client shows the one-time code, type: code <TEXT> here")

    if args.no_viewer:
        session.smoke()
        if mnet is not None:
            mnet.close()
        log("init/smoke ok")
        return

    loop_dt = 1.0 / config.LOOP_HZ
    render_dt = 0.0 if args.render_hz <= 0 else 1.0 / args.render_hz
    next_render = time.perf_counter()
    last = time.perf_counter()
    prof_last = time.perf_counter()
    prof_frames = 0
    prof_control = 0.0
    prof_step = 0.0
    prof_render = 0.0
    prof_mnet = 0.0
    prof_max_ncon = 0

    filtered_base_cmd = np.zeros(4, dtype=np.float64)

    with mujoco.viewer.launch_passive(model, data, key_callback=keyboard.key_callback) as viewer:
        viewer_ref[0] = viewer
        session.setup_viewer_cam(viewer)
        if mnet is not None:
            mnet.ensure_renderer()  # create GL contexts before the loop starts
        while viewer.is_running():
            start = time.perf_counter()
            dt = min(max(start - last, model.opt.timestep), 1.0 / 60.0)
            last = start
            control_start = time.perf_counter()

            # discrete keys queued by the viewer thread, handled here on the
            # main thread (MuJoCo state is only ever touched from this loop)
            for key in keyboard.drain():
                handle_discrete_key(key)
            while not code_queue.empty():
                code_display.show(code_queue.get_nowait())

            arm = active_arm()
            base_cmd = np.zeros(4, dtype=np.float64)
            twist_cmd = np.zeros(6, dtype=np.float64)
            translation_active = False
            rotation_active = False

            if joy.connected:
                joy.pump()
                # semantic one-shot events (edge detection lives in Gamepad,
                # so the physical layout is identical on every OS)
                for event in joy.pressed_events():
                    if event == "mode_left":
                        set_mode("left")
                    elif event == "mode_right":
                        set_mode("right")
                    elif event == "mode_base":
                        set_mode("base")
                    elif event == "close":
                        active_arm().close_ramp = True
                    elif event == "open":
                        open_gripper(data, active_arm())
                    elif event == "help":
                        log("gamepad rotation is always active: right stick yaw/pitch, D-pad roll")
                    elif event == "speed_down":
                        bump_speed(1.0 / 1.5)
                    elif event == "speed_up":
                        bump_speed(1.5)

                lx, ly = joy.left_stick()
                rx, ry = joy.right_stick()
                l2 = joy.trigger_left()
                r2 = joy.trigger_right()
                hx = joy.dpad_x()

                if mode[0] == "base":
                    # stick in screen axes (up = into the screen, right =
                    # screen-right) — same convention as the keyboard arrows,
                    # so base driving matches what the operator sees
                    bfwd, bleft = screen_to_base_local(
                        viewer.cam,
                        lx,
                        -ly,
                        data,
                        session.base_body,
                        args.robot_forward_axis,
                    )
                    base_cmd += np.array([bfwd, bleft, r2 - l2, -rx], dtype=np.float64)
                else:
                    move = config.MOVE_SPEED * speed_scale[0]
                    rot = config.ROT_SPEED * speed_scale[0]
                    xy_world = arm_xy_to_world(-ly, lx)
                    twist_cmd[:3] += (
                        np.array(
                            [xy_world[0], xy_world[1], r2 - l2],
                            dtype=np.float64,
                        )
                        * move
                    )
                    twist_cmd[5] += -rx * rot
                    twist_cmd[3] += ry * rot
                    twist_cmd[4] += hx * rot
                    translation_active = float(np.linalg.norm(twist_cmd[:3])) > config.TWIST_DEAD
                    rotation_active = float(np.linalg.norm(twist_cmd[3:])) > config.TWIST_DEAD

            poll_base_cmd, poll_key_twist, poll_translation, poll_rotation = keyboard.poll(
                getattr(viewer, "_window", None),
                mode[0],
                arm.rotate_mode,
                config.MOVE_SPEED * speed_scale[0],
                config.ROT_SPEED * speed_scale[0],
            )
            if float(np.linalg.norm(poll_base_cmd)) > config.TWIST_DEAD:
                kb_base = poll_base_cmd.copy()
                # keyboard arrows are screen axes: up = into the screen,
                # left = screen-left — convert to the robot's heading frame
                # so driving matches what the operator sees (same convention
                # as the VR base stick)
                if abs(kb_base[0]) > 0 or abs(kb_base[1]) > 0:
                    lx, ly = screen_to_base_local(
                        viewer.cam,
                        -kb_base[1],
                        kb_base[0],
                        data,
                        session.base_body,
                        args.robot_forward_axis,
                    )
                    kb_base[0], kb_base[1] = lx, ly
                base_cmd += kb_base
            if float(np.linalg.norm(poll_key_twist)) > config.TWIST_DEAD:
                key_twist = poll_key_twist.copy()
                xy_world = arm_xy_to_world(key_twist[0], key_twist[1])
                key_twist[0] = xy_world[0]
                key_twist[1] = xy_world[1]
                twist_cmd += key_twist
                translation_active = translation_active or poll_translation
                rotation_active = rotation_active or poll_rotation

            if mode[0] == "base":
                if session.any_arm_grasped():
                    # holding the cable: slow and low-pass the base so driving
                    # doesn't rip the grasp loose
                    base_cmd[:2] *= config.GRASPED_BASE_SPEED_SCALE
                    base_cmd[3] *= config.GRASPED_BASE_SPEED_SCALE
                    base_cmd[2] *= max(config.GRASPED_BASE_SPEED_SCALE, 0.35)
                    filtered_base_cmd[:] = smooth_twist(
                        filtered_base_cmd,
                        base_cmd,
                        dt,
                        config.GRASPED_BASE_FILTER_TAU,
                    )
                    base_cmd = filtered_base_cmd.copy()
                else:
                    filtered_base_cmd[:] = base_cmd
                session.base_driver.drive(base_cmd[0], base_cmd[1], base_cmd[2], base_cmd[3], dt)
                for hold_arm in arms.values():
                    hard_hold_arm(model, data, hold_arm)
            else:
                session.base_driver.drive(0.0, 0.0, 0.0, 0.0, dt)
                command_active = float(np.linalg.norm(twist_cmd)) > config.TWIST_DEAD
                if translation_active and not rotation_active and config.ORIENTATION_LOCK_GAIN > 0.0:
                    arm.target_quat = arm.translate_lock_quat.copy()
                    rot_hold = rot_error(arm.target_quat, data.xmat[arm.tcp_body].reshape(3, 3))
                    twist_cmd[3:] += np.clip(
                        config.ORIENTATION_LOCK_GAIN * rot_hold,
                        -config.ORIENTATION_LOCK_MAX,
                        config.ORIENTATION_LOCK_MAX,
                    )
                    command_active = float(np.linalg.norm(twist_cmd)) > config.TWIST_DEAD

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
                        seed_arm(model, data, arm)  # capture the new hold anchor
                    hard_hold_arm(model, data, arm)
                    arm.was_command_active = False

                for other_name, other_arm in arms.items():
                    if other_name != arm.name:
                        hard_hold_arm(model, data, other_arm)

            control_end = time.perf_counter()
            step_start = control_end
            session.step_once(dt)
            step_end = time.perf_counter()

            if joy.connected:
                # touch feedback: one pulse when a gripper FIRST makes
                # contact (silent while it persists, re-armed on release)
                amp = max(
                    pulser.update(side, session.gripper_contact_force(side), step_end) for side in ("left", "right")
                )
                if amp > 0.0:
                    joy.pulse(amp)

            if mnet is not None:
                mnet_start = time.perf_counter()
                mnet.maybe_publish(data, viewer.cam)
                cfg = mnet.consume_board_config()
                if cfg is not None and getattr(args, "mnet_randomize", False):
                    from .mnet_board import apply_board_config

                    apply_board_config(session, cfg)
                prof_mnet += time.perf_counter() - mnet_start

            if render_dt <= 0.0 or time.perf_counter() >= next_render:
                render_start = time.perf_counter()
                viewer.sync()
                prof_render += time.perf_counter() - render_start
                next_render = time.perf_counter() + render_dt
            prof_frames += 1
            prof_control += control_end - control_start
            prof_step += step_end - step_start
            prof_max_ncon = max(prof_max_ncon, int(data.ncon))
            if args.profile and time.perf_counter() - prof_last >= 1.0:
                elapsed = time.perf_counter() - prof_last
                msg = (
                    f"[profile] loop={prof_frames / elapsed:6.1f}Hz "
                    f"control={prof_control / max(prof_frames, 1) * 1000:6.2f}ms "
                    f"step={prof_step / max(prof_frames, 1) * 1000:6.2f}ms "
                    f"render={prof_render / max(prof_frames, 1) * 1000:6.2f}ms "
                    f"mnet={prof_mnet / max(prof_frames, 1) * 1000:6.2f}ms "
                    f"ncon={prof_max_ncon:3d} mode={mode[0]} "
                    f"grasped={session.any_arm_grasped()} speed={speed_scale[0]:.2f}"
                )
                if args.profile_contacts:
                    msg += f" contacts=[{contact_pair_summary(model, data)}]"
                print(msg, flush=True)
                prof_last = time.perf_counter()
                prof_frames = 0
                prof_control = prof_step = prof_render = prof_mnet = 0.0
                prof_max_ncon = 0
            sleep = loop_dt - (time.perf_counter() - start)
            if sleep > 0:
                time.sleep(sleep)

    if mnet is not None:
        mnet.close()
