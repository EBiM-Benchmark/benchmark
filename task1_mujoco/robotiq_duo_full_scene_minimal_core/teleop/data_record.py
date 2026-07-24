"""Optional LeRobotDataset recording bridge (train branch).

Logs (observation.state, one observation.images.<camera> per configured
camera, action) once per recorded tick while an episode is active, in the
format LeRobot's own training scripts (lerobot_train.py) already read -
no custom training code needed downstream. Only imported when
--record-dataset is passed: the practice/eval code paths never touch this
module or its `lerobot` import.

State/action layout follows task1_isaacsim's own data_contract.yaml
(fr3_duo_mobile_data_contract v1.0.0) instead of a MuJoCo-specific raw
qpos/ctrl dump, so recordings are structurally comparable to the Isaac Sim
side of Task 1 - see build_contract_state/build_contract_action below.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np

import mujoco

from . import log

DATASET_REPO_ID = "local/duo_fr3_cable_management"

# gripper joint/actuator range is [0, 0.8] (confirmed via the compiled
# model: qpos range == ctrl range == gear 1.0, i.e. qpos and ctrl share the
# same units), 0=open per config.GRIPPER_OPEN, 0.8=closed per
# config.GRIPPER_CLOSE. The contract wants the inverse: 1.0=open, 0.0=closed.
GRIPPER_CLOSED_VALUE = 0.8


def _gripper_open_fraction(value: float) -> float:
    return float(np.clip(1.0 - value / GRIPPER_CLOSED_VALUE, 0.0, 1.0))


def _base_local_velocity(model: mujoco.MjModel, data: mujoco.MjData, base_body: int | None) -> tuple[float, float, float]:
    """(vx, vy, omega) in the base's own current heading frame - mode-
    agnostic (works the same regardless of --base-control), via MuJoCo's
    own object-velocity query rather than reconstructing it by hand from
    whichever actuator/qvel representation the active drive mode uses."""
    if base_body is None:
        return 0.0, 0.0, 0.0
    res = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, base_body, res, 1)
    # res = [angular(3), linear(3)] in the body's own local frame
    return float(res[3]), float(res[4]), float(res[2])


def build_state_index(model: mujoco.MjModel, session) -> list[int]:
    """Fixed 19-dim qpos index layout shared by recording AND policy rollout
    (run_policy.py): left arm x7, left gripper, right arm x7, right gripper,
    base x/y/yaw. Must stay identical on both sides - it defines what
    "observation.state" means to a trained policy."""
    idx: list[int] = []
    for side in ("left", "right"):
        arm = session.arms[side]
        idx += [int(model.jnt_qposadr[jid]) for jid in arm.joint_ids]
        idx.append(int(model.jnt_qposadr[arm.gripper_joint]))
    for name in ("x", "y", "yaw"):
        qa = session.base_driver.qadrs.get(name)
        idx.append(qa if qa is not None else -1)
    return idx


def read_state(data: mujoco.MjData, qpos_idx: list[int]) -> np.ndarray:
    return np.array([data.qpos[i] if i >= 0 else 0.0 for i in qpos_idx], dtype=np.float32)


# ---------------------------------------------------------------- contract
# fr3_duo_mobile_data_contract v1.0.0 (task1_isaacsim/assets/embodiments/
# fr3duo_mobile/data_contract.yaml). Block layout, not interleaved - the
# contract doesn't pin an exact flat byte order for state (only for action,
# which this follows exactly), so state uses one clear convention:
# base(6) + left_arm(pos7,vel7,effort7) + right_arm(pos7,vel7,effort7) + gripper(2) = 50
# action follows the contract's own explicit order:
# base_targets(vx,vy,omega) + arm_targets(left0-6,right0-6) + gripper_targets(left,right) = 19
CONTRACT_STATE_DIM = 50
CONTRACT_ACTION_DIM = 19


def build_contract_state(model: mujoco.MjModel, data: mujoco.MjData, session) -> np.ndarray:
    base_body = session.base_driver.base_body
    out: list[float] = []

    # base: x_m, y_m, theta_rad, vx_m_s, vy_m_s, omega_rad_s
    if base_body is not None:
        out += [float(data.xpos[base_body][0]), float(data.xpos[base_body][1])]
        # yaw about world Z, from the body's own rotation matrix
        rmat = data.xmat[base_body].reshape(3, 3)
        out.append(float(np.arctan2(rmat[1, 0], rmat[0, 0])))
    else:
        out += [0.0, 0.0, 0.0]
    out += list(_base_local_velocity(model, data, base_body))

    # arms: position_rad x7, velocity_rad_s x7, effort_nm x7, per side
    for side in ("left", "right"):
        arm = session.arms[side]
        qpos_adr = [int(model.jnt_qposadr[jid]) for jid in arm.joint_ids]
        out += [float(data.qpos[i]) for i in qpos_adr]
        out += [float(data.qvel[i]) for i in arm.dof_ids]
        out += [float(data.actuator_force[i]) for i in arm.act_ids]

    # gripper: normalized open fraction, left then right
    for side in ("left", "right"):
        arm = session.arms[side]
        qa = int(model.jnt_qposadr[arm.gripper_joint])
        out.append(_gripper_open_fraction(float(data.qpos[qa])))

    return np.asarray(out, dtype=np.float32)


def build_contract_action(model: mujoco.MjModel, data: mujoco.MjData, session, dt: float) -> np.ndarray:
    """19-dim, contract order: base_targets(3) + arm_targets(14) +
    gripper_targets(2). Arms are velocity-controlled (IK writes joint
    velocity into ctrl each tick, see robot_arm.py), so there is no native
    "target angle" - one-step-integrate the commanded velocity
    (qpos + ctrl*dt) to get a semantically correct position target."""
    base_body = session.base_driver.base_body
    out: list[float] = list(_base_local_velocity(model, data, base_body))

    for side in ("left", "right"):
        arm = session.arms[side]
        qpos_adr = [int(model.jnt_qposadr[jid]) for jid in arm.joint_ids]
        out += [float(data.qpos[qa] + data.ctrl[ai] * dt) for qa, ai in zip(qpos_adr, arm.act_ids)]

    for side in ("left", "right"):
        arm = session.arms[side]
        out.append(_gripper_open_fraction(float(data.ctrl[arm.gripper_act])))

    return np.asarray(out, dtype=np.float32)


def apply_contract_action(model: mujoco.MjModel, data: mujoco.MjData, session, action: np.ndarray, dt: float) -> None:
    """Inverse of build_contract_action: write a policy's 19-dim contract
    action into data.ctrl. Arms/gripper invert cleanly (exact algebraic
    inverse of the forward integration/normalization, using the same dt).
    Base only inverts for --base-control=actuator (the default) - the
    other three modes (jointvel/wheel/kinematic) represent velocity in
    ways specific to that mode's own actuators and aren't handled here."""
    action = np.asarray(action, dtype=np.float64)
    vx, vy, omega = (float(action[0]), float(action[1]), float(action[2]))
    driver = session.base_driver
    if driver.mode == "actuator" and driver.base_body is not None:
        world_xy = driver._world_xy(vx, vy)
        jx_cmd, jy_cmd = driver._joint_xy(world_xy)
        driver._set_ctrl(driver.acts["base_x"], jx_cmd)
        driver._set_ctrl(driver.acts["base_y"], jy_cmd)
        driver._set_ctrl(driver.acts["base_yaw"], omega * driver._yaw_sign())
    else:
        log(f"[policy] base_control={driver.mode!r} not supported by apply_contract_action - base left as-is")

    idx = 3
    for side in ("left", "right"):
        arm = session.arms[side]
        qpos_adr = [int(model.jnt_qposadr[jid]) for jid in arm.joint_ids]
        for qa, ai in zip(qpos_adr, arm.act_ids):
            target = float(action[idx])
            idx += 1
            low, high = model.actuator_ctrlrange[ai]
            data.ctrl[ai] = float(np.clip((target - data.qpos[qa]) / dt, low, high))

    for side in ("left", "right"):
        arm = session.arms[side]
        open_fraction = float(np.clip(action[idx], 0.0, 1.0))
        idx += 1
        data.ctrl[arm.gripper_act] = (1.0 - open_fraction) * GRIPPER_CLOSED_VALUE


def visual_scene_option() -> mujoco.MjvOption:
    """Geom groups for a "what a human would see" render: 0/1/2 (robot/scene
    visuals) + 5 (the board's visual meshes, per session.py - the viewer
    shows these too, but mujoco.Renderer's own default does NOT, unlike
    the interactive viewer). No group 3 (collision debug) or 4 (mocap
    markers). Matches mnet_bridge.py's evidence-camera scene_option, so the
    recorded video and the real evidence camera look the same."""
    opt = mujoco.MjvOption()
    opt.geomgroup[:] = 0
    for g in (0, 1, 2, 5):
        opt.geomgroup[g] = 1
    return opt


def parse_camera_names(spec: str | list[str]) -> list[str]:
    """'mnet_overhead,right_wrist_cam' -> ['mnet_overhead', 'right_wrist_cam'].
    Already-a-list is passed through so callers don't have to care which
    form they got."""
    if isinstance(spec, list):
        return spec
    return [c.strip() for c in spec.split(",") if c.strip()]


# (width, height) per camera, matched to task1_isaacsim's own
# camera_sensors.yaml render_resolution for each real device - this is the
# exact resolution Isaac Sim itself renders at for the data contract, so
# matching it (not just picking a uniform default) is the most faithful
# sim2real choice available. This matters even though MuJoCo's camera fovy
# is already set to the correct vertical FOV: MuJoCo derives HORIZONTAL FOV
# from fovy + the render aspect ratio, so a mismatched aspect ratio (e.g.
# recording at 4:3 when the real device is 16:9) silently distorts
# horizontal FOV even though fovy itself is right.
CAMERA_RENDER_RESOLUTION: dict[str, tuple[int, int]] = {
    "left_wrist_cam": (848, 480),  # Intel RealSense D405
    "right_wrist_cam": (848, 480),  # Intel RealSense D405
    "head_cam": (1280, 720),  # Stereolabs ZED Mini
    "head_cam_right": (1280, 720),  # ZED Mini's 2nd eye - same device, not a separate contract camera
}
FALLBACK_CAMERA_RESOLUTION = (320, 240)  # cameras with no real-hardware spec (mnet_overhead, main)


def make_camera_renderers(
    model: mujoco.MjModel,
    camera_names: str | list[str],
    fallback_width: int = FALLBACK_CAMERA_RESOLUTION[0],
    fallback_height: int = FALLBACK_CAMERA_RESOLUTION[1],
) -> tuple[dict[str, tuple[mujoco.Renderer, int, int]], list[str], mujoco.MjvOption | None]:
    """Per-camera (Renderer, width, height), sized to CAMERA_RENDER_RESOLUTION
    for known real-hardware cameras and (fallback_width, fallback_height)
    otherwise. Cameras that need the same (width, height) share one
    Renderer - e.g. both D405 wrist cams share a single 848x480 buffer -
    one GL buffer per distinct size actually used, not one per camera name.
    Unknown camera names are skipped with a log line rather than failing
    the whole session."""
    names = parse_camera_names(camera_names)
    valid: list[str] = []
    size_by_name: dict[str, tuple[int, int]] = {}
    for name in names:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
            log(f"[record] camera '{name}' not found in the model - skipping it")
            continue
        valid.append(name)
        size_by_name[name] = CAMERA_RENDER_RESOLUTION.get(name, (fallback_width, fallback_height))
    if not valid:
        return {}, [], None

    renderer_by_size: dict[tuple[int, int], mujoco.Renderer] = {}
    cam_renderers: dict[str, tuple[mujoco.Renderer, int, int]] = {}
    for name in valid:
        size = size_by_name[name]
        if size not in renderer_by_size:
            renderer_by_size[size] = mujoco.Renderer(model, height=size[1], width=size[0])
        cam_renderers[name] = (renderer_by_size[size], size[0], size[1])

    return cam_renderers, valid, visual_scene_option()


class EpisodeRecorder:
    def __init__(
        self,
        model: mujoco.MjModel,
        session,
        root: str,
        *,
        task: str,
        record_fps: float = 10.0,
        camera_names: str | list[str] = "mnet_overhead",
        image_width: int = FALLBACK_CAMERA_RESOLUTION[0],
        image_height: int = FALLBACK_CAMERA_RESOLUTION[1],
    ) -> None:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        self.model = model
        self.task = task
        self.record_dt = 1.0 / record_fps
        self._sim_time_acc = 0.0
        self.active = False

        self.session = session
        # image_width/image_height only apply to cameras with no known
        # real-hardware resolution (see CAMERA_RENDER_RESOLUTION) - known
        # cameras (D405 wrists, ZED Mini head) always record at their real
        # device's own resolution regardless of these.
        self._cam_renderers, self.camera_names, self._scene_option = make_camera_renderers(
            model, camera_names, image_width, image_height
        )
        if not self.camera_names:
            log(f"[record] no valid cameras in {parse_camera_names(camera_names)!r} - recording without images")

        n_state = CONTRACT_STATE_DIM
        n_action = CONTRACT_ACTION_DIM
        features = {
            "observation.state": {"dtype": "float32", "shape": (n_state,), "names": None},
            "action": {"dtype": "float32", "shape": (n_action,), "names": None},
        }
        for cam in self.camera_names:
            _, cam_w, cam_h = self._cam_renderers[cam]
            features[f"observation.images.{cam}"] = {
                "dtype": "video",
                "shape": (cam_h, cam_w, 3),
                "names": ["height", "width", "channel"],
            }

        # LeRobotDataset.create() refuses to reuse an existing directory
        # (mkdir(exist_ok=False)), which would make every session after the
        # first one crash - resume/append instead if a dataset is already
        # there, so repeated --record-dataset runs accumulate episodes.
        # Just checking a file's existence isn't reliable (e.g. tasks.parquet
        # is written by the first save_episode(), but the meta/episodes
        # index is only flushed on finalize() - a session that recorded 4
        # real episodes and then crashed before finalize() still leaves that
        # index missing): actually try loading, and never silently delete a
        # directory that has real recorded frames in it.
        data_dir = Path(root) / "data"
        has_real_data = data_dir.is_dir() and any(data_dir.rglob("*.parquet"))
        loaded = False
        if Path(root).exists():
            try:
                self.dataset = LeRobotDataset(repo_id=DATASET_REPO_ID, root=root)
                loaded = True
            except Exception as exc:
                if has_real_data:
                    raise RuntimeError(
                        f"{root} has recorded episode data but its metadata index is broken "
                        f"(likely a crash before a previous session finished) and won't load: {exc!r}. "
                        "Not deleting it automatically - move it aside or pick a different "
                        "--record-dataset path if you don't need to recover it."
                    ) from exc
                shutil.rmtree(root)  # empty/stub directory only - safe to recreate
        if loaded:
            log(
                f"[record] resuming dataset at {root} ({self.dataset.meta.total_episodes} episodes, "
                f"{self.dataset.meta.total_frames} frames already there) - type 'record' to start/stop an episode"
            )
        else:
            self.dataset = LeRobotDataset.create(
                repo_id=DATASET_REPO_ID,
                fps=int(round(record_fps)),
                features=features,
                root=root,
                robot_type="duo_fr3_mobile",
                use_videos=bool(self._cam_renderers),
            )
            log(
                f"[record] dataset created at {root} (state={n_state}d, action={n_action}d, "
                f"cameras={self.camera_names or 'none'}, {record_fps:g} fps) - type 'record' to start/stop an episode"
            )

    def toggle(self) -> None:
        if self.active:
            self.save_episode()
        else:
            self.active = True
            self._sim_time_acc = 0.0
            log(f"[record] episode {self.dataset.meta.total_episodes} recording STARTED (type 'record' again to stop+save)")

    def save_episode(self) -> None:
        if not self.active:
            return
        self.active = False
        if self.dataset.episode_buffer is not None and self.dataset.episode_buffer.get("size", 0) > 0:
            self.dataset.save_episode()
            log(
                f"[record] episode saved ({self.dataset.meta.total_episodes} total episodes, "
                f"{self.dataset.meta.total_frames} total frames)"
            )
        else:
            log("[record] episode empty, nothing saved")

    def maybe_record(self, data: mujoco.MjData, dt: float) -> None:
        if not self.active:
            return
        self._sim_time_acc += dt
        if self._sim_time_acc < self.record_dt:
            return
        self._sim_time_acc = 0.0

        # target-angle integration uses the PHYSICS step, not this method's
        # `dt` (the recording interval, e.g. 1/10s) - ctrl is a velocity
        # that's only held for one physics tick before the next IK solve
        state = build_contract_state(self.model, data, self.session)
        action = build_contract_action(self.model, data, self.session, float(self.model.opt.timestep))
        frame = {
            "observation.state": state,
            "action": action,
            "task": self.task,
        }
        for cam in self.camera_names:
            renderer, _, _ = self._cam_renderers[cam]
            renderer.update_scene(data, camera=cam, scene_option=self._scene_option)
            frame[f"observation.images.{cam}"] = renderer.render()
        self.dataset.add_frame(frame)

    def close(self) -> None:
        self.save_episode()
        self.dataset.finalize()
        log(f"[record] dataset finalized at {self.dataset.root}")
