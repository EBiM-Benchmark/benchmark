"""GELLO run loop (official EBiM competition input device via ROS 2).

Unlike keyboard/gamepad/VR, GELLO already gives ABSOLUTE joint-space targets
(it is a passive kinematic replica of the FR3 arm), so this loop skips the
twist/Jacobian IK path entirely: each arm joint is driven toward its GELLO
target with a simple velocity P-controller. The gripper's continuous width
fraction is translated into the same close_ramp/force-servo trigger every
other input mode uses (on threshold-crossing edges, not every tick), so
cable-grasp physics behave identically regardless of input device. The
mobile base has no GELLO equivalent — drive it with the keyboard (arrows +
Home/End), exactly as the reference teleoperation repo recommends.
"""

from __future__ import annotations

import time

import glfw
import mujoco.viewer
import numpy as np

import mujoco

from . import config, log
from .grasping import open_gripper
from .input_keyboard import KeyboardInput
from .robot_arm import hard_hold_arm
from .session import TeleopSession

HELP = """
GELLO teleop (official EBiM ROS 2 input device)
  Move the physical GELLO Duo arms - the sim arms follow 1:1 (joint space,
  no clutch: whatever GELLO reports is applied every tick; the arm holds
  still if the publisher stops sending data)
  Squeeze the GELLO gripper closed to grasp (force-servo + cable-retention
  physics are identical to every other input mode)
  Mobile base: USB foot pedal via the reference pedal_state_publisher
  (/pedal/state: A=turn left, B=turn right, C=forward, A+C / B+C = forward
  arcs - remap in teleop/config.py PEDAL_BASE_COMMANDS). Keyboard stays
  available as a fallback: arrows = drive, Home/End = turn,
  PageUp/PageDown = spine
  F / H : report task finished / skipped (--mnet eval mode)
Requires the official franka_gello_state_publisher node running and
publishing on /left and /right (github.com/EBiM-Benchmark/teleoperation).
"""


def _drive_arm_to_gello(model, data, arm, target_joints: np.ndarray, kp: float) -> None:
    """Per-joint velocity P-control toward GELLO's absolute joint target.
    No Jacobian/IK: GELLO already gives joint-space angles 1:1 with our
    7-dof FR3 arms, so this is a direct joint tracker, not a task-space one."""
    for i, (jid, act) in enumerate(zip(arm.joint_ids, arm.act_ids)):
        current = float(data.qpos[model.jnt_qposadr[jid]])
        lo, hi = model.jnt_range[jid]
        target = float(np.clip(target_joints[i], lo, hi))
        qvel_cmd = kp * (target - current)
        qvel_cmd = float(np.clip(qvel_cmd, -config.JOINT_VEL_LIMIT, config.JOINT_VEL_LIMIT))
        ctrl_lo, ctrl_hi = model.actuator_ctrlrange[act]
        data.ctrl[act] = float(np.clip(qvel_cmd, ctrl_lo, ctrl_hi))


def main(args) -> None:
    log(HELP)
    try:
        from .input_gello import GelloBridge

        gello = GelloBridge()
    except Exception as exc:
        log(f"[gello] unavailable: {exc}")
        return

    session = TeleopSession(args)
    model, data, arms = session.model, session.data, session.arms
    kp = float(getattr(args, "gello_joint_kp", config.GELLO_JOINT_KP))

    # ManipulationNet eval bridge (optional, see mnet_bridge.py)
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

    keyboard = KeyboardInput()
    # edge-triggered gripper thresholds: only fire open/close_ramp on the
    # tick the fraction CROSSES a threshold, not every tick it stays past it
    # (update_grasp already owns the close_ramp -> holding transition; if we
    # kept re-arming close_ramp every tick we'd undo that transition and the
    # gripper would overdrive forever instead of settling at the hold force)
    was_closing = {"left": False, "right": False}
    was_open = {"left": True, "right": True}

    def control_tick(dt: float, window) -> None:
        while not code_queue.empty():
            code_display.show(code_queue.get_nowait())

        # base: USB foot pedal is the primary driver in the GELLO workflow
        # (both hands are on GELLO); the keyboard stays live as a fallback
        # for rigs without a pedal - the two simply sum
        base_cmd, _twist, _tr, _rot = keyboard.poll(window, "base", False, 0.0, 0.0)
        base_cmd = base_cmd + gello.get_pedal_base_cmd()
        session.base_driver.drive(base_cmd[0], base_cmd[1], base_cmd[2], base_cmd[3], dt)

        for side, arm in arms.items():
            target_joints, gripper_fraction = gello.get_state(side)
            if target_joints is None:
                hard_hold_arm(model, data, arm)
                continue
            _drive_arm_to_gello(model, data, arm, target_joints, kp)

            closing_now = gripper_fraction < config.GELLO_GRIPPER_CLOSE_BELOW
            open_now = gripper_fraction > config.GELLO_GRIPPER_OPEN_ABOVE
            if closing_now and not was_closing[side]:
                arm.close_ramp = True
                log(f"{side} gripper closing (GELLO {gripper_fraction:.2f})...")
            if open_now and not was_open[side]:
                open_gripper(data, arm)
                log(f"{side} gripper open (GELLO {gripper_fraction:.2f})")
            was_closing[side] = closing_now
            was_open[side] = open_now

        session.step_once(dt, follow_tcp_quat=True)

    if args.no_viewer:
        for _ in range(400):
            control_tick(model.opt.timestep, None)
        gello.close()
        if mnet is not None:
            mnet.close()
        log("init/smoke ok (no GELLO publisher expected in --no-viewer mode)")
        return

    def key_callback(keycode: int) -> None:
        keyboard.key_callback(keycode)

    loop_dt = 1.0 / config.LOOP_HZ
    render_dt = 0.0 if args.render_hz <= 0 else 1.0 / args.render_hz
    next_render = time.perf_counter()
    last = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        session.setup_viewer_cam(viewer)
        if mnet is not None:
            mnet.ensure_renderer()
        while viewer.is_running():
            start = time.perf_counter()
            dt = min(max(start - last, model.opt.timestep), 1.0 / 60.0)
            last = start

            for key in keyboard.drain():
                if mnet is not None and key == glfw.KEY_F:
                    mnet.report_finished()
                elif mnet is not None and key == glfw.KEY_H:
                    mnet.report_skipped()

            control_tick(dt, getattr(viewer, "_window", None))

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

    gello.close()
    if mnet is not None:
        mnet.close()
