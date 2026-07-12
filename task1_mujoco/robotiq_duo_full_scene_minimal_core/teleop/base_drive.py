"""Mobile-base driving and the vertical spine.

Four control modes (--base-control):
  actuator   (default) servo the base planar-joint velocity actuators —
             smooth and obstacle-safe; idle pose is hard-anchored because the
             per-tick arm holds pump momentum into the free planar joints
  jointvel   inject joint velocities directly (harsher on collisions)
  kinematic  move the model root directly (no dynamics at all)
  wheel      drive the real wheel/steer actuators (needs ground traction)

All commands are (local_x, local_y, spine, yaw): local x/y in the robot's
heading frame, spine in up/down rate, yaw as turn rate.
"""

from __future__ import annotations

import math

import numpy as np

import mujoco

from . import config
from .maths import axis_angle_quat, quat_mul
from .mjutil import optional_obj_id, robot_local_xy_to_world


class BaseDriver:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        mode: str,
        base_speed: float,
        base_yaw_speed_deg: float,
        wheel_speed: float,
        wheel_yaw_speed: float,
        forward_axis: str,
        speed_scale: list[float],
    ) -> None:
        self.model = model
        self.data = data
        self.mode = mode
        self.base_speed = float(base_speed)
        self.base_yaw_speed = math.radians(float(base_yaw_speed_deg))
        self.wheel_speed = float(wheel_speed)
        self.wheel_yaw_speed = float(wheel_yaw_speed)
        self.forward_axis = forward_axis
        self.speed_scale = speed_scale  # shared 1-element list, live-updated by the runner

        self.base_body = optional_obj_id(model, mujoco.mjtObj.mjOBJ_BODY, config.BASE_BODY)
        joint_ids = {
            "x": optional_obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, config.BASE_X_JOINT),
            "y": optional_obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, config.BASE_Y_JOINT),
            "yaw": optional_obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, config.BASE_YAW_JOINT),
        }
        self.joint_ids = joint_ids
        self.dof_ids = {name: None if jid is None else int(model.jnt_dofadr[jid]) for name, jid in joint_ids.items()}
        self.qadrs = {name: None if jid is None else int(model.jnt_qposadr[jid]) for name, jid in joint_ids.items()}
        self.acts = {
            "base_x": optional_obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, config.BASE_VEL_X_ACT),
            "base_y": optional_obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, config.BASE_VEL_Y_ACT),
            "base_yaw": optional_obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, config.BASE_VEL_YAW_ACT),
            "steer_front": optional_obj_id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                config.BASE_STEER_FRONT_ACT,
            ),
            "steer_rear": optional_obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, config.BASE_STEER_REAR_ACT),
            "drive_front": optional_obj_id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                config.BASE_DRIVE_FRONT_ACT,
            ),
            "drive_rear": optional_obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, config.BASE_DRIVE_REAR_ACT),
            "caster_front_steer": optional_obj_id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                config.BASE_CASTER_FRONT_STEER_ACT,
            ),
            "caster_front_roll": optional_obj_id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                config.BASE_CASTER_FRONT_ROLL_ACT,
            ),
            "caster_rear_steer": optional_obj_id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                config.BASE_CASTER_REAR_STEER_ACT,
            ),
            "caster_rear_roll": optional_obj_id(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                config.BASE_CASTER_REAR_ROLL_ACT,
            ),
        }

        # spine (vertical lift) rides along with every base mode
        self.spine_act = optional_obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, config.SPINE_ACT)
        self.spine_dof: int | None = None
        self.spine_target = 0.0
        if self.spine_act is not None:
            j = int(model.actuator_trnid[self.spine_act, 0])
            if j >= 0:
                self.spine_target = float(data.qpos[int(model.jnt_qposadr[j])])
                self.spine_dof = int(model.jnt_dofadr[j])
                data.ctrl[self.spine_act] = self.spine_target

        self._idle_anchor: list | None = None  # captured planar qpos while idle

    # ------------------------------------------------------------------ util
    def _set_ctrl(self, act: int | None, value: float) -> None:
        if act is None:
            return
        low, high = self.model.actuator_ctrlrange[act]
        self.data.ctrl[act] = float(np.clip(value, low, high))

    def _update_spine(self, spine: float, dt: float) -> bool:
        if self.spine_act is None:
            return False
        low, high = self.model.actuator_ctrlrange[self.spine_act]
        if abs(spine) > config.TWIST_DEAD:
            self.spine_target = float(
                np.clip(
                    self.spine_target + spine * config.SPINE_SPEED * self.speed_scale[0] * dt,
                    low,
                    high,
                )
            )
            self.data.ctrl[self.spine_act] = self.spine_target
            return True
        self.spine_target = float(np.clip(self.spine_target, low, high))
        if self.spine_dof is not None:
            self.data.qvel[self.spine_dof] = 0.0
        self.data.ctrl[self.spine_act] = self.spine_target
        return False

    def _world_xy(self, local_x: float, local_y: float) -> np.ndarray:
        return robot_local_xy_to_world(
            self.data,
            self.base_body,
            np.array([local_x, local_y], dtype=np.float64),
            self.forward_axis,
        )

    def _joint_xy(self, world_xy: np.ndarray) -> tuple[float, float]:
        """Project a world-frame planar command onto the two slide joints'
        ACTUAL world axes. The planar stack is mounted under a rotated parent
        body (measured: base_planar_x drives world -Y, base_planar_y drives
        world +X), so writing world X/Y straight into the x/y channels turns
        every command by 90 degrees."""
        jx = self.joint_ids["x"]
        jy = self.joint_ids["y"]
        if jx is None or jy is None:
            return float(world_xy[0]), float(world_xy[1])
        return (
            float(world_xy[:2] @ self.data.xaxis[jx][:2]),
            float(world_xy[:2] @ self.data.xaxis[jy][:2]),
        )

    def _yaw_sign(self) -> float:
        """+1 if the yaw hinge spins about world +Z, -1 for -Z (the hinge
        lives in the same rotated stack as the slides)."""
        jz = self.joint_ids["yaw"]
        if jz is None:
            return 1.0
        s = float(np.sign(self.data.xaxis[jz][2]))
        return s if s != 0.0 else 1.0

    # ------------------------------------------------------------------ modes
    def drive(
        self,
        local_x: float,
        local_y: float,
        spine: float,
        yaw: float,
        dt: float,
    ) -> bool:
        """Dispatch one drive command to the configured mode. Returns True if
        anything is actively moving."""
        if self.mode == "jointvel":
            return self._drive_jointvel(local_x, local_y, spine, yaw, dt)
        if self.mode == "actuator":
            return self._drive_velocity_actuators(local_x, local_y, spine, yaw, dt)
        if self.mode == "wheel":
            return self._drive_wheels(local_x, local_y, spine, yaw, dt)
        # kinematic: park every dynamic base actuator, move the model root
        for name in (
            "drive_front",
            "drive_rear",
            "caster_front_roll",
            "caster_rear_roll",
            "base_x",
            "base_y",
            "base_yaw",
        ):
            self._set_ctrl(self.acts[name], 0.0)
        return self._drive_kinematic(local_x, local_y, spine, yaw, dt)

    def _drive_velocity_actuators(
        self,
        local_x: float,
        local_y: float,
        spine: float,
        yaw: float,
        dt: float,
    ) -> bool:
        active = False
        if float(np.hypot(local_x, local_y)) > config.TWIST_DEAD or abs(yaw) > config.TWIST_DEAD:
            self._idle_anchor = None
            jx_cmd, jy_cmd = self._joint_xy(self._world_xy(local_x, local_y))
            self._set_ctrl(
                self.acts["base_x"],
                jx_cmd * self.base_speed * self.speed_scale[0],
            )
            self._set_ctrl(
                self.acts["base_y"],
                jy_cmd * self.base_speed * self.speed_scale[0],
            )
            self._set_ctrl(
                self.acts["base_yaw"],
                yaw * self._yaw_sign() * self.base_yaw_speed * self.speed_scale[0],
            )
            active = True
        else:
            self._set_ctrl(self.acts["base_x"], 0.0)
            self._set_ctrl(self.acts["base_y"], 0.0)
            self._set_ctrl(self.acts["base_yaw"], 0.0)
            # anchor the virtual base while idle: the per-tick arm qvel holds
            # pump momentum into the free planar joints (reaction impulses)
            # and the capped brake alone lets the base wander visibly
            qadrs = [self.qadrs[name] for name in ("x", "y", "yaw")]
            if self._idle_anchor is None:
                self._idle_anchor = [None if qa is None else float(self.data.qpos[qa]) for qa in qadrs]
            for qa, anchor_q, name in zip(qadrs, self._idle_anchor, ("x", "y", "yaw")):
                if qa is not None and anchor_q is not None:
                    self.data.qpos[qa] = anchor_q
                dof = self.dof_ids[name]
                if dof is not None:
                    self.data.qvel[dof] = 0.0
        self._set_ctrl(self.acts["drive_front"], 0.0)
        self._set_ctrl(self.acts["drive_rear"], 0.0)
        self._set_ctrl(self.acts["caster_front_roll"], 0.0)
        self._set_ctrl(self.acts["caster_rear_roll"], 0.0)
        active = self._update_spine(spine, dt) or active
        return active

    def _drive_jointvel(
        self,
        local_x: float,
        local_y: float,
        spine: float,
        yaw: float,
        dt: float,
    ) -> bool:
        jx_cmd, jy_cmd = self._joint_xy(self._world_xy(local_x, local_y))
        vx = jx_cmd * self.base_speed * self.speed_scale[0]
        vy = jy_cmd * self.base_speed * self.speed_scale[0]
        wz = yaw * self._yaw_sign() * self.base_yaw_speed * self.speed_scale[0]
        if self.dof_ids["x"] is not None:
            self.data.qvel[self.dof_ids["x"]] = vx
        if self.dof_ids["y"] is not None:
            self.data.qvel[self.dof_ids["y"]] = vy
        if self.dof_ids["yaw"] is not None:
            self.data.qvel[self.dof_ids["yaw"]] = wz
        active = float(np.hypot(vx, vy)) > config.TWIST_DEAD or abs(wz) > config.TWIST_DEAD
        # sustain the commanded velocity through the physics step: with ctrl=0
        # the kv=200000 base velocity actuators brake the injected qvel inside
        # every step and the resulting jolts shake the arms
        self._set_ctrl(self.acts["base_x"], vx)
        self._set_ctrl(self.acts["base_y"], vy)
        self._set_ctrl(self.acts["base_yaw"], wz)
        self._set_ctrl(self.acts["drive_front"], 0.0)
        self._set_ctrl(self.acts["drive_rear"], 0.0)
        self._set_ctrl(self.acts["caster_front_roll"], 0.0)
        self._set_ctrl(self.acts["caster_rear_roll"], 0.0)
        active = self._update_spine(spine, dt) or active
        return active

    def _drive_wheels(
        self,
        local_x: float,
        local_y: float,
        spine: float,
        yaw: float,
        dt: float,
    ) -> bool:
        active = False
        trans = float(np.hypot(local_x, local_y))
        if trans > config.TWIST_DEAD:
            steer = math.atan2(local_y, local_x)
            drive = trans * self.wheel_speed * self.speed_scale[0]
            turn = yaw * self.wheel_yaw_speed * self.speed_scale[0]
            self._set_ctrl(self.acts["steer_front"], steer)
            self._set_ctrl(self.acts["steer_rear"], steer)
            self._set_ctrl(self.acts["caster_front_steer"], steer)
            self._set_ctrl(self.acts["caster_rear_steer"], steer)
            self._set_ctrl(self.acts["drive_front"], drive + turn)
            self._set_ctrl(self.acts["drive_rear"], drive - turn)
            self._set_ctrl(self.acts["caster_front_roll"], drive + turn)
            self._set_ctrl(self.acts["caster_rear_roll"], drive - turn)
            active = True
        elif abs(yaw) > config.TWIST_DEAD:
            # spin in place: wheels turned to a crab pattern
            self._set_ctrl(self.acts["steer_front"], math.pi / 2.0)
            self._set_ctrl(self.acts["steer_rear"], -math.pi / 2.0)
            self._set_ctrl(self.acts["caster_front_steer"], math.pi / 2.0)
            self._set_ctrl(self.acts["caster_rear_steer"], -math.pi / 2.0)
            turn = yaw * self.wheel_yaw_speed * self.speed_scale[0]
            self._set_ctrl(self.acts["drive_front"], turn)
            self._set_ctrl(self.acts["drive_rear"], -turn)
            self._set_ctrl(self.acts["caster_front_roll"], turn)
            self._set_ctrl(self.acts["caster_rear_roll"], -turn)
            active = True
        else:
            self._set_ctrl(self.acts["drive_front"], 0.0)
            self._set_ctrl(self.acts["drive_rear"], 0.0)
            self._set_ctrl(self.acts["caster_front_roll"], 0.0)
            self._set_ctrl(self.acts["caster_rear_roll"], 0.0)
        active = self._update_spine(spine, dt) or active
        return active

    def _drive_kinematic(
        self,
        local_x: float,
        local_y: float,
        spine: float,
        yaw: float,
        dt: float,
    ) -> bool:
        active = False
        if self.base_body is not None:
            local_xy = np.array([local_x, local_y], dtype=np.float64)
            if float(np.linalg.norm(local_xy)) > config.TWIST_DEAD:
                world_delta = robot_local_xy_to_world(self.data, self.base_body, local_xy, self.forward_axis)
                self.model.body_pos[self.base_body] += world_delta * self.base_speed * self.speed_scale[0] * dt
                active = True
            if abs(yaw) > config.TWIST_DEAD:
                delta_q = axis_angle_quat(
                    (0.0, 0.0, 1.0),
                    yaw * self.base_yaw_speed * self.speed_scale[0] * dt,
                )
                self.model.body_quat[self.base_body] = quat_mul(delta_q, self.model.body_quat[self.base_body].copy())
                active = True
        active = self._update_spine(spine, dt) or active
        return active
