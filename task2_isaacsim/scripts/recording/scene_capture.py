# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Ground-truth publishers and the scene reset hotkey for Task 2 recording.

Both classes are run_teleop_loop tick callbacks: an optional bind(node) is
called once with the IsaacSimRosBridge node (to create publishers), then
tick(sim_time) runs once per loop iteration on the main thread.

GroundTruthPublisher publishes world poses of the task objects and the
deformed thermal-pad vertices so the recorder
(services/recording/record_task2.py) can store full ground truth alongside
the LeRobot dataset without a second file writer in the sim process.

SceneResetController resets (and optionally randomizes) the task objects and
the robot between episodes on the '5' key in the Isaac Sim window.
"""

from __future__ import annotations

import json
import math
import sys

import numpy as np
from std_msgs.msg import Float32MultiArray, String

from pxr import Gf, Usd, UsdGeom

try:
    from pxr import PhysxSchema
except Exception:  # pragma: no cover - ships with Isaac Sim
    PhysxSchema = None

# Shared topic contract from scripts/topics.py (scripts/ is on sys.path in
# every entry point that imports this package).
from topics import load_topics

_GROUND_TRUTH_TOPICS = load_topics()["ground_truth"]
OBJECT_POSES_TOPIC = _GROUND_TRUTH_TOPICS["object_poses"]
PAD_POINTS_TOPIC = _GROUND_TRUTH_TOPICS["pad_points"]
SCENE_RESET_TOPIC = _GROUND_TRUTH_TOPICS["scene_reset"]
SCENE_RESET_REQUEST_TOPIC = _GROUND_TRUTH_TOPICS["scene_reset_request"]

# Objects whose names start with this prefix are jittered as one rigid group
# (the deformable pad is attached to the sticker base).
PAD_GROUP_PREFIX = "thermalpad"


def _quat_wxyz(rotation: Gf.Quatd) -> list[float]:
    imaginary = rotation.GetImaginary()
    return [
        float(rotation.GetReal()),
        float(imaginary[0]),
        float(imaginary[1]),
        float(imaginary[2]),
    ]


class GroundTruthPublisher:
    """Publish task-object world poses (every tick) and thermal-pad mesh
    vertices (every pad_points_every ticks) on /isaac/task2/*.

    object_poses: std_msgs/String JSON
        {"sim_time": t, "objects": {name: [x, y, z, qw, qx, qy, qz]}}
    pad_points: std_msgs/Float32MultiArray
        data = [sim_time, n_points, x0, y0, z0, x1, ...] (world frame)
    """

    def __init__(
        self,
        stage,
        objects_root_path: str,
        *,
        pad_key: str = "thermalpad",
        pad_points_every: int = 6,
    ):
        self._stage = stage
        self._pad_points_every = max(int(pad_points_every), 0)
        self._tick_count = 0
        self._poses_pub = None
        self._pad_pub = None

        root_prim = stage.GetPrimAtPath(objects_root_path)
        if not root_prim.IsValid():
            raise RuntimeError(
                f"Task objects root prim not found: {objects_root_path}"
            )
        self._object_prims = {
            prim.GetName(): prim for prim in root_prim.GetChildren()
        }
        if not self._object_prims:
            raise RuntimeError(
                f"No task object prims under {objects_root_path}"
            )

        # Deformable thermal-pad meshes: prefer the PhysX deformable-body
        # API (works for both the room scene's per-object references and the
        # barebone scene's aggregate USD); fall back to all meshes under the
        # pad_key child.
        self._pad_meshes = []
        if PhysxSchema is not None:
            self._pad_meshes = [
                prim
                for prim in Usd.PrimRange(root_prim)
                if prim.IsA(UsdGeom.Mesh)
                and prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI)
            ]
        if not self._pad_meshes:
            pad_prim = stage.GetPrimAtPath(f"{objects_root_path}/{pad_key}")
            if pad_prim.IsValid():
                self._pad_meshes = [
                    prim
                    for prim in Usd.PrimRange(pad_prim)
                    if prim.IsA(UsdGeom.Mesh)
                ]
        if not self._pad_meshes:
            print(
                "Warning: no thermal-pad meshes found under "
                f"{objects_root_path}/{pad_key}; pad points will not be "
                "published",
                file=sys.stderr,
            )
        print(
            "Recording: ground truth for objects "
            f"{sorted(self._object_prims)} "
            f"({len(self._pad_meshes)} pad meshes)",
            flush=True,
        )

    def bind(self, node) -> None:
        self._poses_pub = node.create_publisher(String, OBJECT_POSES_TOPIC, 10)
        self._pad_pub = node.create_publisher(
            Float32MultiArray, PAD_POINTS_TOPIC, 10
        )

    def _publish_object_poses(self, sim_time: float, xform_cache) -> None:
        objects = {}
        for name, prim in self._object_prims.items():
            world = xform_cache.GetLocalToWorldTransform(prim)
            translation = world.ExtractTranslation()
            objects[name] = [
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
                *_quat_wxyz(world.ExtractRotationQuat()),
            ]
        msg = String()
        msg.data = json.dumps({"sim_time": sim_time, "objects": objects})
        self._poses_pub.publish(msg)

    def _publish_pad_points(self, sim_time: float, xform_cache) -> None:
        chunks = []
        for mesh_prim in self._pad_meshes:
            points_attr = UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get()
            if points_attr is None:
                continue
            points = np.asarray(points_attr, dtype=np.float64)
            if points.size == 0:
                continue
            matrix = np.asarray(
                xform_cache.GetLocalToWorldTransform(mesh_prim),
                dtype=np.float64,
            )
            # USD uses row-vector convention: p_world = p_local * M.
            chunks.append(points @ matrix[:3, :3] + matrix[3, :3])
        if not chunks:
            return
        world_points = np.concatenate(chunks, axis=0).astype(np.float32)
        msg = Float32MultiArray()
        msg.data = [
            float(sim_time),
            float(world_points.shape[0]),
            *world_points.reshape(-1).tolist(),
        ]
        self._pad_pub.publish(msg)

    def tick(self, sim_time: float) -> None:
        if self._poses_pub is None:
            return
        xform_cache = UsdGeom.XformCache()
        self._publish_object_poses(sim_time, xform_cache)
        self._tick_count += 1
        if (
            self._pad_meshes
            and self._pad_points_every > 0
            and self._tick_count % self._pad_points_every == 0
        ):
            self._publish_pad_points(sim_time, xform_cache)


class SceneResetController:
    """Reset the scene for the next episode on the '5' key or a ROS request.

    Stops the simulation (PhysX restores the object spawn poses), optionally
    jitters the authored object spawn transforms, plays again, and restores
    the robot ready pose plus the keyboard-teleop targets. Publishes a JSON
    event on /isaac/task2/scene_reset so the recorder can log the applied
    randomization.

    The thermal pad and its sticker base are jittered as one group about the
    sticker-base origin; boards and the target are jittered independently.
    """

    def __init__(
        self,
        world,
        robot,
        stage,
        objects_root_path: str,
        *,
        spine_controller=None,
        arm_teleop=None,
        randomize: bool = False,
        xy_jitter_m: float = 0.02,
        yaw_jitter_deg: float = 10.0,
        seed: int | None = None,
    ):
        self._world = world
        self._robot = robot
        self._stage = stage
        self._spine_controller = spine_controller
        self._arm_teleop = arm_teleop
        self._randomize = bool(randomize)
        self._xy_jitter_m = float(xy_jitter_m)
        self._yaw_jitter_deg = float(yaw_jitter_deg)
        self._rng = np.random.default_rng(seed)
        self._pending = False
        self._event_pub = None
        self._reset_count = 0

        root_prim = stage.GetPrimAtPath(objects_root_path)
        if not root_prim.IsValid():
            raise RuntimeError(
                f"Task objects root prim not found: {objects_root_path}"
            )
        # Spawn transforms as authored by reference_usd/set_xform: one
        # double-precision translate op and one float orient op per object
        # root Xform.
        self._spawn_ops: dict[str, tuple] = {}
        self._spawn_poses: dict[str, tuple] = {}
        for prim in root_prim.GetChildren():
            translate_op = None
            orient_op = None
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    orient_op = op
            if translate_op is None or orient_op is None:
                print(
                    "Warning: scene reset skips randomization of "
                    f"{prim.GetPath()}: expected translate+orient xform ops",
                    file=sys.stderr,
                )
                continue
            name = prim.GetName()
            self._spawn_ops[name] = (translate_op, orient_op)
            self._spawn_poses[name] = (
                Gf.Vec3d(translate_op.Get()),
                Gf.Quatf(orient_op.Get()),
            )

        self._keyboard_subscription = None
        try:
            import carb.input  # noqa: PLC0415
            import omni.appwindow  # noqa: PLC0415

            self._carb_input = carb.input
            input_iface = carb.input.acquire_input_interface()
            app_window = omni.appwindow.get_default_app_window()
            if app_window is None:
                raise RuntimeError("No Omniverse app window found")
            self._keyboard_subscription = (
                input_iface.subscribe_to_keyboard_events(
                    app_window.get_keyboard(), self._on_keyboard_event
                )
            )
            print(
                "Scene reset hotkey enabled: press 5 in the Isaac Sim "
                "window to reset "
                + ("and randomize " if self._randomize else "")
                + "the task objects for the next episode",
                flush=True,
            )
        except Exception as exc:  # noqa: BLE001 - keyboard optional headless
            self._carb_input = None
            print(
                f"Warning: scene reset hotkey unavailable: {exc}",
                file=sys.stderr,
            )

    def bind(self, node) -> None:
        self._event_pub = node.create_publisher(String, SCENE_RESET_TOPIC, 10)
        # Same effect as the '5' hotkey, but triggerable from the recorder
        # terminal (menu command 5). Runs on the bridge's spin_once, i.e.
        # the sim main-loop thread — no race with tick().
        self._reset_request_sub = node.create_subscription(
            String, SCENE_RESET_REQUEST_TOPIC, self._on_reset_request, 10
        )
        print(
            "Scene reset also available over ROS: "
            f"{SCENE_RESET_REQUEST_TOPIC}",
            flush=True,
        )

    def _on_reset_request(self, msg) -> None:
        del msg  # any message is a request
        print("Scene reset requested over ROS", flush=True)
        self._pending = True

    def _on_keyboard_event(self, event, *args, **kwargs):
        if self._carb_input is None:
            return True
        if (
            event.type == self._carb_input.KeyboardEventType.KEY_PRESS
            and event.input == self._carb_input.KeyboardInput.KEY_5
        ):
            self._pending = True
        return True

    def _sample_offsets(self) -> dict[str, dict[str, float]]:
        """One xy/yaw offset per jitter group, keyed by object name."""
        offsets: dict[str, dict[str, float]] = {}
        group_offsets: dict[str, dict[str, float]] = {}
        for name in self._spawn_poses:
            group = (
                PAD_GROUP_PREFIX if name.startswith(PAD_GROUP_PREFIX) else name
            )
            if group not in group_offsets:
                group_offsets[group] = {
                    "dx": float(
                        self._rng.uniform(
                            -self._xy_jitter_m, self._xy_jitter_m
                        )
                    ),
                    "dy": float(
                        self._rng.uniform(
                            -self._xy_jitter_m, self._xy_jitter_m
                        )
                    ),
                    "dyaw_deg": float(
                        self._rng.uniform(
                            -self._yaw_jitter_deg, self._yaw_jitter_deg
                        )
                    ),
                }
            offsets[name] = group_offsets[group]
        return offsets

    def _apply_offsets(self, offsets: dict[str, dict[str, float]]) -> None:
        # Pivot of the pad group: the sticker base spawn position, so pad
        # and base rotate together instead of about their own origins.
        pad_pivot = None
        for name, (position, _) in self._spawn_poses.items():
            if name.startswith(PAD_GROUP_PREFIX) and name.endswith("_base"):
                pad_pivot = position
        for name, (position, orientation) in self._spawn_poses.items():
            offset = offsets[name]
            yaw_rad = math.radians(offset["dyaw_deg"])
            rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), offset["dyaw_deg"])
            pivot = (
                pad_pivot
                if name.startswith(PAD_GROUP_PREFIX) and pad_pivot is not None
                else position
            )
            new_position = (
                rotation.TransformDir(position - pivot)
                + pivot
                + Gf.Vec3d(offset["dx"], offset["dy"], 0.0)
            )
            half = 0.5 * yaw_rad
            yaw_quat = Gf.Quatf(math.cos(half), 0.0, 0.0, math.sin(half))
            translate_op, orient_op = self._spawn_ops[name]
            translate_op.Set(Gf.Vec3d(new_position))
            orient_op.Set(yaw_quat * orientation)

    def _restore_spawn_poses(self) -> None:
        for name, (position, orientation) in self._spawn_poses.items():
            translate_op, orient_op = self._spawn_ops[name]
            translate_op.Set(position)
            orient_op.Set(orientation)

    def tick(self, sim_time: float) -> None:
        if not self._pending:
            return
        self._pending = False
        self._reset_count += 1
        print(
            f"Scene reset #{self._reset_count} starting (deformable "
            "re-initialization can take a few seconds)...",
            flush=True,
        )
        self._world.stop()
        offsets: dict[str, dict[str, float]] = {}
        if self._spawn_ops:
            if self._randomize:
                offsets = self._sample_offsets()
                self._apply_offsets(offsets)
            else:
                self._restore_spawn_poses()
        self._world.reset()

        import isaacsim_fr3duo_teleop_bridge_core as core  # noqa: PLC0415

        core._apply_ready_pose(self._robot, list(self._robot.dof_names))
        if self._spine_controller is not None:
            self._spine_controller.reset_target()
        if self._arm_teleop is not None and self._arm_teleop.available:
            self._arm_teleop.reset_targets()

        if self._event_pub is not None:
            msg = String()
            msg.data = json.dumps(
                {
                    "event": "scene_reset",
                    "reset_index": self._reset_count,
                    "sim_time": sim_time,
                    "randomized": bool(offsets),
                    "offsets": offsets,
                }
            )
            self._event_pub.publish(msg)
        print(
            f"Scene reset #{self._reset_count} done"
            + (f" (randomized {len(offsets)} objects)" if offsets else ""),
            flush=True,
        )
