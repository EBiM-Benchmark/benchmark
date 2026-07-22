"""VR run loop (Quest 2 and other OpenXR headsets; SteamVR fallback).

Default backend is OpenXR — no Steam required. It talks to whatever OpenXR
runtime is active on the system:
  Quest 2/3       Meta Quest Link app (Link cable or Air Link)
  Pico            Pico Connect
  WMR/Index/Vive  their own runtimes (SteamVR also works)
Controller brands are handled by OpenXR interaction profiles (Touch/Index/
Vive/simple bindings live in vr_openxr.py).

Clutch-style mapping (per hand -> arm; mirrored when --facing front):
  hold GRIP        engage: controller motion drives the TCP 1:1 (scaled)
  release GRIP     arm freezes where you left it
  TRIGGER          close gripper (pad-contact force servo, same as gamepad)
  A / X            open gripper
  RIGHT stick      base translation      LEFT stick   yaw / spine
  CLICK stick      left = speed up, right = speed down
With --mnet, the desktop viewer window accepts F (task finished) and
H (task skipped) for ManipulationNet eval reporting.
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
from .maths import mat_to_quat, rot_error, smooth_twist
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
VR teleop
  hold GRIP   : move this hand's arm (clutch)
  TRIGGER     : close gripper on the cable
  A / X       : open gripper
  RIGHT stick : base X/Y   LEFT stick: yaw / spine
  CLICK stick : left = speed up, right = speed down
  B / Y       : print this reminder
openxr backend: --hmd-view floats the sim view inside the headset (mirrors
the desktop camera - aim it with the mouse); default is monitor-only.
steamvr backend: watch the monitor.
"""


def main(args) -> None:
    if sys.platform == "win32":
        try:
            import ctypes

            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    log(HELP)
    # VR loosens the free-motion clamp so tracking keeps up with a real hand;
    # the grasped clamp stays at the cable-safe default (see config)
    config.FREE_MAX_TCP_STEP = config.VR_FREE_MAX_TCP_STEP
    config.GRASPED_MAX_TCP_STEP = config.VR_GRASPED_MAX_TCP_STEP

    session = TeleopSession(args)
    model, data, arms = session.model, session.data, session.arms
    drive_base = session.base_driver.drive

    # ManipulationNet eval bridge (optional, see mnet_bridge.py)
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
        session.smoke(drive_base=True)
        if mnet is not None:
            mnet.close()
        log("init/smoke ok (VR not initialized in --no-viewer mode)")
        return

    clutch = {"left": ClutchState(), "right": ClutchState()}
    prev_trigger = {"left": False, "right": False}
    prev_a = {"left": False, "right": False}
    prev_b = {"left": False, "right": False}
    prev_stick_click = {"left": False, "right": False}

    # on-the-fly speed levels driven by stick clicks
    speed_idx = [config.VR_SPEED_LEVELS.index(1.0)]
    free_step_base = config.FREE_MAX_TCP_STEP
    grasped_step_base = config.GRASPED_MAX_TCP_STEP

    def bump_speed(delta: int) -> None:
        levels = config.VR_SPEED_LEVELS
        idx = min(max(speed_idx[0] + delta, 0), len(levels) - 1)
        if idx == speed_idx[0]:
            return
        speed_idx[0] = idx
        mult = levels[idx]
        config.FREE_MAX_TCP_STEP = free_step_base * mult
        config.GRASPED_MAX_TCP_STEP = grasped_step_base * min(mult, config.VR_GRASPED_SPEED_LEVEL_CAP)
        log(f"[speed] x{mult:g} (click left stick: faster, right stick: slower)")

    facing_front = args.facing == "front"
    # mirror teleop: the operator faces the robot, so their left hand drives
    # the arm on the LEFT OF THE SCREEN, which is the robot's right arm
    hand_arm_pairs = (("left", "right"), ("right", "left")) if facing_front else (("left", "left"), ("right", "right"))

    def control_tick(hands: dict, dt: float, cam) -> None:
        # one-time-code text typed into the sim terminal
        while not code_queue.empty():
            code_display.show(code_queue.get_nowait())

        # speed levels: rising edge of a stick click
        for hand_name, delta in (("left", 1), ("right", -1)):
            click = hands[hand_name].valid and hands[hand_name].stick_click
            if click and not prev_stick_click[hand_name]:
                bump_speed(delta)
            prev_stick_click[hand_name] = click

        # ---- base driving from the sticks
        base_cmd = np.zeros(4)  # local x, local y, spine, yaw
        right = hands["right"]
        left = hands["left"]
        if right.valid:
            sx, sy = right.stick
            if abs(sx) > config.VR_STICK_DEAD or abs(sy) > config.VR_STICK_DEAD:
                if facing_front and cam is not None:
                    lx, ly = screen_to_base_local(
                        cam,
                        sx,
                        sy,
                        data,
                        session.base_body,
                        args.robot_forward_axis,
                    )
                    base_cmd[0] += lx
                    base_cmd[1] += ly
                else:
                    base_cmd[0] += sy
                    base_cmd[1] += -sx
        if left.valid:
            sx, sy = left.stick
            if abs(sx) > config.VR_STICK_DEAD:
                # yaw sense flipped for the facing-front operator (user-tested)
                base_cmd[3] += sx if facing_front else -sx
            if abs(sy) > config.VR_STICK_DEAD:
                base_cmd[2] += sy
        if session.any_arm_grasped():
            # driving the base while holding the cable is the easiest way to
            # rip it loose — same slow-down the desktop loop applies
            base_cmd[0] *= config.GRASPED_BASE_SPEED_SCALE
            base_cmd[1] *= config.GRASPED_BASE_SPEED_SCALE
            base_cmd[3] *= config.GRASPED_BASE_SPEED_SCALE
            base_cmd[2] *= max(config.GRASPED_BASE_SPEED_SCALE, 0.35)
        drive_base(base_cmd[0], base_cmd[1], base_cmd[2], base_cmd[3], dt)

        # ---- clutch arm control per hand
        for name, arm_name in hand_arm_pairs:
            hand = hands[name]
            arm = arms[arm_name]
            state = clutch[name]
            if not hand.valid:
                if not state.engaged:
                    hard_hold_arm(model, data, arm)
                continue

            trig = hand.trigger > config.VR_TRIGGER_CLOSE
            if trig and not prev_trigger[name]:
                arm.close_ramp = True
                log(f"{arm_name} gripper closing...")
            prev_trigger[name] = trig
            if hand.a and not prev_a[name]:
                open_gripper(data, arm)
                log(f"{arm_name} gripper open")
            prev_a[name] = hand.a
            if hand.b and not prev_b[name]:
                log(HELP)
            prev_b[name] = hand.b

            engage_now = hand.grip > (config.VR_GRIP_RELEASE if state.engaged else config.VR_GRIP_ENGAGE)
            if engage_now:
                if not state.engaged:
                    # capture the clutch anchor and freeze the mapping frame
                    state.engaged = True
                    state.ctrl_pos = hand.pos.copy()
                    state.ctrl_rot = hand.rot.copy()
                    state.tcp_pos = data.xpos[arm.tcp_body].copy()
                    state.tcp_rot = data.xmat[arm.tcp_body].reshape(3, 3).copy()
                    if facing_front and cam is not None:
                        state.map = vr_to_screen_map(cam)
                    else:
                        state.map = vr_to_world_map(data, session.base_body, args.robot_forward_axis)
                    log(f"{arm_name} arm engaged")
                m = state.map
                desired_pos = state.tcp_pos + m @ (hand.pos - state.ctrl_pos) * float(args.vr_scale)
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
                    log(f"{arm_name} arm locked")
                if arm.was_command_active:
                    seed_arm(model, data, arm)
                    arm.was_command_active = False
                hard_hold_arm(model, data, arm)

        session.step_once(dt, follow_tcp_quat=True)

    if args.vr_backend == "steamvr":
        _run_steamvr(args, session, control_tick, mnet)
    else:
        _run_openxr(args, session, control_tick, hand_arm_pairs, mnet)
    if mnet is not None:
        mnet.close()


# --------------------------------------------------------------------------
# backends
# --------------------------------------------------------------------------


def _mnet_key_callback(mnet):
    """F/H eval-report keys on the desktop viewer window. Reports only touch
    ROS (never MuJoCo state), so handling them on the viewer thread is safe."""

    def callback(keycode: int) -> None:
        if mnet is None or keycode < 0:
            return
        if int(keycode) == glfw.KEY_F:
            mnet.report_finished()
        elif int(keycode) == glfw.KEY_H:
            mnet.report_skipped()

    return callback


def _run_steamvr(args, session: TeleopSession, control_tick, mnet=None) -> None:
    from .vr_steamvr import SteamVRInput

    model, data = session.model, session.data
    try:
        vr = SteamVRInput()
    except Exception as exc:
        log(f"[vr] cannot start steamvr backend: {exc}")
        log("[vr] checklist: 1) SteamVR running  2) headset connected  3) controllers awake (squeeze them once)")
        return
    log("[vr] SteamVR background session up; squeeze a grip to drive an arm")
    loop_dt = 1.0 / config.LOOP_HZ
    render_dt = 0.0 if args.render_hz <= 0 else 1.0 / args.render_hz
    next_render = time.perf_counter()
    last = time.perf_counter()
    with mujoco.viewer.launch_passive(model, data, key_callback=_mnet_key_callback(mnet)) as viewer:
        session.setup_viewer_cam(viewer, fallback_view=False)
        if mnet is not None:
            mnet.ensure_renderer()
        while viewer.is_running():
            start = time.perf_counter()
            dt = min(max(start - last, model.opt.timestep), 1.0 / 60.0)
            last = start
            control_tick(vr.poll(), dt, viewer.cam)
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
    vr.close()


def _run_openxr(args, session: TeleopSession, control_tick, hand_arm_pairs, mnet=None) -> None:
    """OpenXR: input + haptics; optionally a stereo sim view in the headset."""
    try:
        from .vr_openxr import OpenXRTeleop
    except Exception as exc:
        log(f"[vr] openxr modules unavailable: {exc}")
        log("[vr] pip install pyopenxr PyOpenGL, or use --vr-backend steamvr")
        return

    model, data = session.model, session.data
    scene_opt = mujoco.MjvOption()
    scene_opt.geomgroup[:] = 0
    for g in (0, 1, 2, 5):
        scene_opt.geomgroup[g] = 1
    scene_opt.sitegroup[5] = 1  # routing-direction hint (operator-only)
    hmd_cam = mujoco.MjvCamera()
    stereo_baseline = 0.06  # camera separation for the stereo screen (m)

    loop_dt = 1.0 / config.LOOP_HZ
    # hmd-view on: 40Hz stereo content for the in-headset screen; off: the
    # desktop viewer syncs at full render rate and the headset stays black
    render_dt = 1.0 / 40.0 if args.hmd_view else 1.0 / max(args.render_hz, 1.0)

    # strict GL bring-up order: every MuJoCo context (viewer thread fonts,
    # offscreen renderer) must be fully initialized BEFORE the OpenXR session
    # allocates its swapchains, otherwise mjr_makeContext intermittently dies
    # with 'Could not allocate font lists'
    with mujoco.viewer.launch_passive(model, data, key_callback=_mnet_key_callback(mnet)) as viewer:
        session.setup_viewer_cam(viewer, fallback_view=False)
        viewer.sync()
        renderer = None
        if args.hmd_view:
            renderer = mujoco.Renderer(
                model,
                height=config.HMD_VIEW_SIZE[1],
                width=config.HMD_VIEW_SIZE[0],
            )
        if mnet is not None:
            mnet.ensure_renderer()  # before the OpenXR session claims GL
        try:
            with OpenXRTeleop(config.HMD_VIEW_SIZE) as vr:
                log("[vr] OpenXR session up; squeeze a grip to drive an arm")
                vr.start()
                pulser = ContactPulser()
                next_render = time.perf_counter()
                last = time.perf_counter()
                prof_last = time.perf_counter()
                prof_n = 0
                prof_ctrl = 0.0
                prof_render = 0.0
                while viewer.is_running() and vr.alive():
                    start = time.perf_counter()
                    dt = min(max(start - last, model.opt.timestep), 1.0 / 60.0)
                    last = start
                    control_tick(vr.get_hands(), dt, viewer.cam)
                    # touch feedback: one pulse on the controller that drives
                    # that arm when its gripper (pads, fingers, knuckles,
                    # housing) FIRST makes contact; silent while it persists
                    for hand_name, arm_name in hand_arm_pairs:
                        amp = pulser.update(
                            arm_name,
                            session.gripper_contact_force(arm_name),
                            start,
                        )
                        if amp > 0.0:
                            vr.set_haptic(hand_name, amp)
                    if mnet is not None:
                        mnet.maybe_publish(data, viewer.cam)
                        cfg = mnet.consume_board_config()
                        if cfg is not None and getattr(args, "mnet_randomize", False):
                            from .mnet_board import apply_board_config

                            apply_board_config(session, cfg)
                    t_ctrl = time.perf_counter()
                    prof_ctrl += t_ctrl - start
                    if t_ctrl >= next_render:
                        viewer.sync()
                        if renderer is not None:
                            # in-headset stereo screen mirrors the desktop
                            # camera: one render per eye from offset cameras
                            az = math.radians(float(viewer.cam.azimuth))
                            right_h = np.array([-math.sin(az), math.cos(az), 0.0])
                            hmd_cam.distance = viewer.cam.distance
                            hmd_cam.azimuth = viewer.cam.azimuth
                            hmd_cam.elevation = viewer.cam.elevation
                            eyes = []
                            for side in (-0.5, 0.5):
                                hmd_cam.lookat[:] = viewer.cam.lookat + right_h * (side * stereo_baseline)
                                renderer.update_scene(
                                    data,
                                    camera=hmd_cam,
                                    scene_option=scene_opt,
                                )
                                eyes.append(renderer.render())
                            vr.submit(eyes[0], eyes[1])
                        next_render = time.perf_counter() + render_dt
                        prof_render += time.perf_counter() - t_ctrl
                    prof_n += 1
                    if time.perf_counter() - prof_last >= 2.0:
                        span = time.perf_counter() - prof_last
                        log(
                            f"[perf] loop={prof_n / span:5.1f}Hz ctrl+step={prof_ctrl / max(prof_n, 1) * 1000:5.1f}ms "
                            f"render={prof_render / max(prof_n, 1) * 1000:4.1f}ms "
                            f"rtf={model.opt.timestep * prof_n / span * 100:4.1f}%"
                        )
                        prof_last = time.perf_counter()
                        prof_n = 0
                        prof_ctrl = 0.0
                        prof_render = 0.0
                    # always yield at least 1ms so the XR thread can hit the
                    # headset's frame slots (GIL starvation reads as judder)
                    time.sleep(max(loop_dt - (time.perf_counter() - start), 0.001))
                vr.stop()
                if vr.error is not None:
                    log(f"[vr] xr thread ended with: {vr.error}")
        except Exception as exc:
            log(f"[vr] openxr session failed: {exc}")
            log(
                "[vr] checklist: 1) headset streaming app connected (Quest:"
                " Meta Quest Link; Pico: Pico Connect)  2) that app is set as"
                " the active OpenXR runtime  3) WEAR the headset while starting"
                " (or hold a finger on the inner proximity sensor) — Meta"
                " reports the device unavailable when it is not on a head"
                "  4) controllers awake. Fallback: --vr-backend steamvr"
            )
        if renderer is not None:
            renderer.close()
