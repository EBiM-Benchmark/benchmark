"""TeleopSession: everything the input methods share.

Owns the model/data, both arms, the base driver and the physics step, so a
run loop (keyboard/gamepad/VR) only has to translate device input into
per-tick commands. Construction order matters and mirrors the original demo:
load -> keyframe -> cable layout -> arms -> ready pose -> spawn -> seed.
"""

from __future__ import annotations

import numpy as np

import mujoco

from . import config, log
from .base_drive import BaseDriver
from .grasping import (
    apply_clip_guide,
    geoms_contact_force,
    gripper_geom_ids,
    update_grasp,
)
from .maths import mat_to_quat
from .mjutil import optional_obj_id
from .robot_arm import (
    Arm,
    hard_hold_arm,
    make_arm,
    pad_slot_center,
    seed_arm,
    set_arm_ready_pose,
    sync_target,
)
from .scene import (
    cable_body_ids,
    cable_geom_ids,
    initialize_cable_on_board,
    load_model,
    spawn_for_data_collection,
    teleport_base_near_cable,
)


class TeleopSession:
    def __init__(self, args) -> None:
        self.args = args
        base_control = getattr(args, "base_control", "actuator")
        self.model = load_model(
            timestep=args.timestep,
            noslip_iterations=args.noslip_iterations,
            wheel_collision=(base_control == "wheel"),
        )
        self.data = mujoco.MjData(self.model)
        key = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key)
        mujoco.mj_forward(self.model, self.data)
        initialize_cable_on_board(self.model, self.data)

        self.arms: dict[str, Arm] = {
            "left": make_arm(self.model, self.data, "left"),
            "right": make_arm(self.model, self.data, "right"),
        }
        # haptic source: every collidable geom of each gripper, pads included
        self.haptic_geoms: dict[str, set[int]] = {
            name: gripper_geom_ids(self.model, config.GRIPPER_BODY_PREFIX[name])
            | arm.pad_left_contact
            | arm.pad_right_contact
            for name, arm in self.arms.items()
        }

        self.cable_geoms = cable_geom_ids(self.model)
        self.cable_bodies = cable_body_ids(self.model)
        if not self.cable_geoms or not self.cable_bodies:
            raise RuntimeError("No cable geoms/bodies found.")
        self._cable_body_arr = np.asarray(self.cable_bodies, dtype=int)
        self.cable_dofs = np.asarray(
            [
                dof
                for b in self.cable_bodies
                for dof in range(self.model.body_dofadr[b], self.model.body_dofadr[b] + self.model.body_dofnum[b])
            ],
            dtype=int,
        )
        self.clip_body = optional_obj_id(self.model, mujoco.mjtObj.mjOBJ_BODY, config.CLIP_BODY)
        self.grasp_assist = bool(args.grasp_assist)

        for arm in self.arms.values():
            set_arm_ready_pose(self.model, self.data, arm)
        mujoco.mj_forward(self.model, self.data)

        # optional offline board randomization (same distribution as the mnet
        # client's test_coordinates) — applied before spawn placement so the
        # base is positioned relative to the relocated cable
        if getattr(args, "randomize_board", False):
            from .mnet_board import apply_local_random_config

            apply_local_random_config(self, getattr(args, "randomize_seed", None))

        # spawn: --start-at-board (explicit) takes priority over the default
        # data-collection spawn, which takes priority over the bare XML pose
        self.start_lookat: np.ndarray | None = None
        if args.start_at_board:
            if teleport_base_near_cable(self.model, self.data, self.arms["right"]):
                log("[start] base placed at board, right gripper over the cable (descend and close to grasp)")
                slot = pad_slot_center(
                    self.data,
                    self.arms["right"].pad_left,
                    self.arms["right"].pad_right,
                )
                self.start_lookat = np.array([slot[0], slot[1], 0.12], dtype=np.float64)
            else:
                log("[start] start-at-board placement failed; keeping spawn pose")
        elif getattr(args, "data_collection_spawn", True):
            if spawn_for_data_collection(self.model, self.data, args.robot_forward_axis):
                log("[start] data-collection spawn: base parked past the board, head_cam aimed at fixture 0")
            else:
                log("[start] data-collection spawn failed; keeping default pose")

        for arm in self.arms.values():
            seed_arm(self.model, self.data, arm)
            self.data.ctrl[arm.gripper_act] = config.GRIPPER_OPEN

        # shared speed multiplier; desktop mode changes it live from buttons
        self.speed_scale: list[float] = [1.0]
        self.base_driver = BaseDriver(
            self.model,
            self.data,
            mode=base_control,
            base_speed=args.base_speed,
            base_yaw_speed_deg=args.base_yaw_speed_deg,
            wheel_speed=getattr(args, "wheel_speed", 75.0),
            wheel_yaw_speed=getattr(args, "wheel_yaw_speed", 45.0),
            forward_axis=args.robot_forward_axis,
            speed_scale=self.speed_scale,
        )
        self.base_body = self.base_driver.base_body

        # routing-direction hint for the default (unrandomized) board; the
        # client / --randomize-board refresh it via apply_board_config
        from .mnet_board import apply_routing_overlay

        apply_routing_overlay(self)

    # ------------------------------------------------------------------ step
    def step_once(self, dt: float, *, follow_tcp_quat: bool = False) -> None:
        """One physics step: grasp servo + clip guide + mj_step + pose refresh.

        follow_tcp_quat=True (VR) keeps each arm's target orientation glued to
        the actual TCP; False (desktop) restores the translate-lock quat so
        pure translation cannot drift the orientation.
        """
        self.data.xfrc_applied[:, :] = 0.0
        for arm in self.arms.values():
            update_grasp(
                self.model,
                self.data,
                arm,
                self.cable_geoms,
                self.cable_bodies,
                self.grasp_assist,
                dt,
            )
        if self.clip_body is not None:
            apply_clip_guide(self.model, self.data, self.clip_body, self.cable_bodies)
        mujoco.mj_step(self.model, self.data)
        # ballistic safety valve: cap the cable's peak LINEAR segment speed
        # so whip-crack tips cannot tunnel through the table plates. Fires
        # only during blow-ups — normal manipulation is ~12x below the cap
        # (see config.CABLE_LINVEL_MAX for why this is NOT the reverted
        # angular limiter).
        lin = self.data.cvel[self._cable_body_arr, 3:6]
        peak = float(np.sqrt((lin * lin).sum(axis=1).max()))
        if peak > config.CABLE_LINVEL_MAX:
            self.data.qvel[self.cable_dofs] *= config.CABLE_LINVEL_MAX / peak
        # Physics is fully solved inside mj_step; downstream code only reads
        # poses (xpos/xmat/geom_xpos) and jacobian prerequisites, so refresh
        # just the kinematic caches instead of a second full dynamics pass.
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        for arm in self.arms.values():
            arm.target_pos = self.data.xpos[arm.tcp_body].copy()
            if follow_tcp_quat:
                arm.target_quat = mat_to_quat(self.data.xmat[arm.tcp_body].reshape(3, 3).copy())
            elif arm.rotate_mode:
                arm.target_quat = mat_to_quat(self.data.xmat[arm.tcp_body].reshape(3, 3).copy())
                arm.translate_lock_quat = arm.target_quat.copy()
            else:
                arm.target_quat = arm.translate_lock_quat.copy()
            sync_target(self.data, arm)

    # ------------------------------------------------------------------ misc
    def any_arm_grasped(self) -> bool:
        return any(arm.grasped_body is not None for arm in self.arms.values())

    def gripper_contact_force(self, arm_name: str) -> float:
        """Haptic signal: total contact force anywhere on this gripper."""
        return geoms_contact_force(self.model, self.data, self.haptic_geoms[arm_name])

    def setup_viewer_cam(self, viewer, *, fallback_view: bool = True) -> None:
        """Shared passive-viewer setup: visibility groups + start camera.
        Group 3 (collision geoms) and group 4 (TCP mocap-target debug
        markers) stay hidden; group 5 holds the board's visual meshes,
        which the viewer hides by default."""
        viewer.opt.geomgroup[0] = 1
        viewer.opt.geomgroup[1] = 1
        viewer.opt.geomgroup[3] = 0
        viewer.opt.geomgroup[4] = 0
        viewer.opt.geomgroup[5] = 1
        # routing-direction hint sites; the evidence-camera renderers never
        # enable site group 5, so the hint stays out of the benchmark video
        viewer.opt.sitegroup[5] = 1
        if self.start_lookat is not None:
            viewer.cam.lookat[:] = self.start_lookat
            viewer.cam.distance = 1.8
            viewer.cam.azimuth = 250
            viewer.cam.elevation = -20
        elif fallback_view:
            viewer.cam.distance = 1.6
            viewer.cam.azimuth = 145
            viewer.cam.elevation = -25

    def smoke(self, steps: int = 400, *, drive_base: bool = False) -> None:
        """--no-viewer: run a short headless burst to validate the setup."""
        for _ in range(steps):
            for arm in self.arms.values():
                hard_hold_arm(self.model, self.data, arm)
            if drive_base:
                self.base_driver.drive(0.0, 0.0, 0.0, 0.0, self.model.opt.timestep)
            self.step_once(self.model.opt.timestep)
