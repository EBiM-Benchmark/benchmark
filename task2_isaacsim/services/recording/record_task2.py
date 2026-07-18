#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""LeRobot demonstration recorder for EBiM Task 2 (thermal-pad placement).

Subscribes to the Task 2 Isaac Sim bridge topics (task2_isaacsim/scripts/)
and writes a LeRobot dataset plus a task2_extras/ ground-truth sidecar:

* Gripper channels: both open-fractions are part of action and state.
* True action semantics: the action holds the post-arbitration joint
  position targets from /isaac/applied_joint_commands (whatever produced
  them: ROS/GELLO commands, keyboard RMPflow teleop, spine keys) plus the
  applied base twist -- not measured poses.
* Sim-time pacing: frames are sampled on the sim clock topic
  (topics.yaml "clock", /isaac/clock), so a deformable scene that
  runs below real time still yields uniformly spaced samples in sim time.
  Episode start latches on a fresh clock message (never the cached value,
  which goes stale around scene resets), and a clock jump in either
  direction while recording discards the episode.
* Ground truth: task-object world poses, deformed thermal-pad vertices,
  scene-reset/randomization events, and optional depth are stored as a
  task2_extras/ sidecar next to the LeRobot dataset (same process, same
  episode boundaries).
* Success labels: on save, an IoU suggestion is computed from the eval
  camera (scripts/evaluation/task2 logic) and confirmed on the console.

Run it in the task2 lerobot_recorder container (see
task2_isaacsim/docker-compose.yml, profile "record") while the sim runs
scene_room.py --record and the teleop stack is up.

Action layout (20, float32) -- indices 0..18 follow
task1_isaacsim/assets/embodiments/fr3duo_mobile/data_contract.yaml, the
spine is appended (see
task2_isaacsim/assets/embodiments/fr3duo_mobile_task2/data_contract_recording.yaml):
    [0:3]   base twist vx, vy, wz          (m/s, m/s, rad/s, body frame)
    [3:10]  left arm joint targets         (rad, absolute)
    [10:17] right arm joint targets        (rad, absolute)
    [17:19] gripper open-fraction targets  (left, right; 1 = open)
    [19]    spine height target            (m)

State layout (37, float32):
    [0:7]   left EE pose  x y z qx qy qz qw   (world, left_fr3v2_link8)
    [7:14]  right EE pose x y z qx qy qz qw   (world, right_fr3v2_link8)
    [14:21] left arm joint positions          (rad)
    [21:28] right arm joint positions         (rad)
    [28]    spine height                      (m)
    [29:31] gripper open-fractions            (left, right)
    [31:34] base odometry x, y, yaw           (m, m, rad, world)
    [34:37] base velocity vx, vy, wz          (body frame)
"""

import os

os.environ.setdefault("TORCH_SHOW_CPP_STACKTRACES", "0")
# Datasets are local-only (the <hub_namespace>/* repo ids do not exist on the
# hub); without this, lerobot falls back to hub lookups on any local load
# error.
os.environ.setdefault("HF_HUB_OFFLINE", "1")

import argparse  # noqa: E402
import json  # noqa: E402
import logging  # noqa: E402
import math  # noqa: E402
import select  # noqa: E402
import shutil  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import termios  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import tty  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402
from lerobot.utils.utils import init_logging  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import Float32MultiArray, String  # noqa: E402

try:
    from vision_msgs.msg import Detection2DArray
except ImportError:  # pragma: no cover - needed only for --suggest-success
    Detection2DArray = None

# The shared topic contract loader lives in task2_isaacsim/scripts/; it is
# import-safe outside Isaac Sim (stdlib + PyYAML only).
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from topics import camera_topic, load_topics  # noqa: E402

# ---------------------------------------------------------------------------
# Topic constants — all names come from config/topics.yaml (fail-hard load);
# schema constants below must match the bridge + eval config.
# ---------------------------------------------------------------------------
_TOPICS = load_topics()

CLOCK_TOPIC = _TOPICS["clock"]
FULL_STATES_TOPIC = _TOPICS["recording"]["joint_states_full"]
APPLIED_COMMANDS_TOPIC = _TOPICS["recording"]["applied_joint_commands"]
ODOM_TOPIC = _TOPICS["recording"]["odom"]
CMD_VEL_APPLIED_TOPIC = _TOPICS["recording"]["cmd_vel_applied"]
EE_POSE_TOPICS = dict(_TOPICS["recording"]["ee_pose"])
OBJECT_POSES_TOPIC = _TOPICS["ground_truth"]["object_poses"]
PAD_POINTS_TOPIC = _TOPICS["ground_truth"]["pad_points"]
SCENE_RESET_TOPIC = _TOPICS["ground_truth"]["scene_reset"]
SCENE_RESET_REQUEST_TOPIC = _TOPICS["ground_truth"]["scene_reset_request"]
EVAL_BBOX_TOPIC = _TOPICS["cameras"]["eval"]["bbox_2d_tight"]
EVAL_LABELS_TOPIC = _TOPICS["cameras"]["eval"]["semantic_labels"]
EVAL_SEGMENTATION_TOPIC = _TOPICS["cameras"]["eval"]["semantic_segmentation"]


def _build_camera_table(topics):
    """Recorder camera table (keys = --cameras / dataset video keys)."""
    entries = dict(topics["cameras"]["robot"])
    entries["eval_camera"] = topics["cameras"]["eval"]
    return {
        key: {
            "image_topic": camera_topic(topics, entry["namespace"], "image"),
            "depth_topic": camera_topic(topics, entry["namespace"], "depth"),
            "shape": tuple(entry["shape"]),
        }
        for key, entry in entries.items()
    }


CAMERAS = _build_camera_table(_TOPICS)

# Recording defaults ship next to this script; --config selects another file.
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "recording.yaml"
# Task2 evaluation modules (evaluation.py, image_utils.py, config.py).
EVAL_MODULE_DIR = (
    Path(__file__).resolve().parents[3] / "scripts" / "evaluation" / "task2"
)
# Keys that only make sense per-invocation and are rejected in the YAML.
CONFIG_CLI_ONLY_KEYS = {"help", "config", "resume", "resume_version"}

LEFT_JOINTS = [f"left_fr3v2_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"right_fr3v2_joint{i}" for i in range(1, 8)]
SPINE_JOINT = "franka_spine_vertical_joint"
LEFT_GRIPPER_DRIVER = "left_right_finger_joint"
RIGHT_GRIPPER_DRIVER = "right_right_finger_joint"
GRIPPER_CLOSED_RAD = 0.8

ACTION_DIM = 20
STATE_DIM = 37

# The clock topic carries world.current_time from the bridge main loop and
# rebases to ~0 on a scene reset. A gap beyond this tolerance ahead of the
# sampling schedule while recording means a reset happened mid-episode or
# the clock source misbehaved (the robot USD's embedded OmniGraph clock —
# a leaked publisher stuck at 0.0 — is why the topic is /isaac/clock, not
# /clock); catching it prevents a silent burst of duplicate catch-up
# frames.
CLOCK_FORWARD_JUMP_TOLERANCE_S = 2.0

ACTION_NAMES = (
    ["base.vx", "base.vy", "base.wz"]
    + [f"{name}.target" for name in LEFT_JOINTS + RIGHT_JOINTS]
    + [
        "left_gripper.open_fraction.target",
        "right_gripper.open_fraction.target",
    ]
    + ["spine.height.target"]
)
STATE_NAMES = (
    [f"left_ee.{axis}" for axis in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    + [f"right_ee.{axis}" for axis in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    + [f"{name}.pos" for name in LEFT_JOINTS + RIGHT_JOINTS]
    + ["spine.height"]
    + ["left_gripper.open_fraction", "right_gripper.open_fraction"]
    + ["base.odom.x", "base.odom.y", "base.odom.yaw"]
    + ["base.vel.vx", "base.vel.vy", "base.vel.wz"]
)


def _candidate_joint_names(name):
    """Robot-USD joint-name variants, mirroring the sim bridge's resolver."""
    yield name
    if "fr3v2_joint" in name:
        yield name.replace("fr3v2_joint", "fr3v2_1_joint")
    if name == "left_right_finger_joint":
        yield "left_fr3v2_finger_joint1"
    if name == "right_right_finger_joint":
        yield "right_fr3v2_finger_joint1"


def resolve_joint(joint_map: dict, name: str, default=math.nan) -> float:
    for candidate in _candidate_joint_names(name):
        value = joint_map.get(candidate)
        if value is not None and math.isfinite(value):
            return float(value)
    return default


def gripper_open_fraction(driver_position_rad: float) -> float:
    if not math.isfinite(driver_position_rad):
        return math.nan
    return float(
        np.clip(1.0 - driver_position_rad / GRIPPER_CLOSED_RAD, 0.0, 1.0)
    )


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
    )


def image_msg_to_array(msg) -> np.ndarray:
    channels = 3
    data = np.frombuffer(msg.data, dtype=np.uint8)
    array = data.reshape(msg.height, msg.step)[:, : msg.width * channels]
    array = array.reshape(msg.height, msg.width, channels)
    if msg.encoding.lower() in ("bgr8", "bgra8"):
        array = array[:, :, ::-1]
    return np.ascontiguousarray(array)


def depth_msg_to_array(msg) -> np.ndarray:
    data = np.frombuffer(msg.data, dtype=np.float32)
    return data.reshape(msg.height, msg.width).copy()


# ---------------------------------------------------------------------------
# ROS interface: latest-value cache over all recorded topics
# ---------------------------------------------------------------------------
class Task2RecorderNode(Node):
    def __init__(
        self, camera_keys, *, record_depth, suggest_success, qos_depth
    ):
        super().__init__("task2_lerobot_recorder")
        self.lock = threading.Lock()
        self.sim_time = None
        self.joint_states = {}
        self.applied_commands = {}
        self.odom = None  # (x, y, z, qx, qy, qz, qw, vx, vy, vz, wz)
        self.cmd_vel = (0.0, 0.0, 0.0)
        self.ee_poses = {"left": None, "right": None}
        self.images = {key: None for key in camera_keys}
        self.depths = {key: None for key in camera_keys}
        self.object_poses_raw = None
        self.pad_points_raw = None
        self.reset_events = []
        self.eval_bbox = None
        self.eval_labels = None
        self.eval_segmentation = None
        self.message_counts = {}

        self.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, qos_depth)
        self.create_subscription(
            JointState, FULL_STATES_TOPIC, self._on_full_states, qos_depth
        )
        self.create_subscription(
            JointState, APPLIED_COMMANDS_TOPIC, self._on_applied, qos_depth
        )
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, qos_depth
        )
        self.create_subscription(
            Twist, CMD_VEL_APPLIED_TOPIC, self._on_cmd_vel, qos_depth
        )
        for side, topic in EE_POSE_TOPICS.items():
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, side=side: self._on_ee_pose(side, msg),
                qos_depth,
            )
        for key in camera_keys:
            self.create_subscription(
                Image,
                CAMERAS[key]["image_topic"],
                lambda msg, key=key: self._on_image(key, msg),
                qos_profile_sensor_data,
            )
            if record_depth:
                self.create_subscription(
                    Image,
                    CAMERAS[key]["depth_topic"],
                    lambda msg, key=key: self._on_depth(key, msg),
                    qos_profile_sensor_data,
                )
        self.create_subscription(
            String, OBJECT_POSES_TOPIC, self._on_object_poses, qos_depth
        )
        self.create_subscription(
            Float32MultiArray, PAD_POINTS_TOPIC, self._on_pad_points, qos_depth
        )
        self.create_subscription(
            String, SCENE_RESET_TOPIC, self._on_scene_reset, qos_depth
        )
        self._reset_request_pub = self.create_publisher(
            String, SCENE_RESET_REQUEST_TOPIC, qos_depth
        )
        if suggest_success and Detection2DArray is not None:
            self.create_subscription(
                Detection2DArray,
                EVAL_BBOX_TOPIC,
                self._on_eval_bbox,
                qos_depth,
            )
            self.create_subscription(
                String, EVAL_LABELS_TOPIC, self._on_eval_labels, qos_depth
            )
            self.create_subscription(
                Image,
                EVAL_SEGMENTATION_TOPIC,
                self._on_eval_segmentation,
                qos_profile_sensor_data,
            )

    def _count(self, key):
        self.message_counts[key] = self.message_counts.get(key, 0) + 1

    def _on_clock(self, msg):
        with self.lock:
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
            self._count("clock")

    def _on_full_states(self, msg):
        with self.lock:
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self.joint_states[name] = float(msg.position[idx])
            self._count("joint_states_full")

    def _on_applied(self, msg):
        with self.lock:
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self.applied_commands[name] = float(msg.position[idx])
            self._count("applied_commands")

    def _on_odom(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        with self.lock:
            self.odom = (
                pos.x,
                pos.y,
                pos.z,
                ori.x,
                ori.y,
                ori.z,
                ori.w,
                lin.x,
                lin.y,
                lin.z,
                ang.z,
            )
            self._count("odom")

    def _on_cmd_vel(self, msg):
        with self.lock:
            self.cmd_vel = (msg.linear.x, msg.linear.y, msg.angular.z)
            self._count("cmd_vel")

    def _on_ee_pose(self, side, msg):
        p, o = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.ee_poses[side] = (p.x, p.y, p.z, o.x, o.y, o.z, o.w)
            self._count(f"ee_{side}")

    def _on_image(self, key, msg):
        with self.lock:
            self.images[key] = msg
            self._count(f"image_{key}")

    def _on_depth(self, key, msg):
        with self.lock:
            self.depths[key] = msg
            self._count(f"depth_{key}")

    def _on_object_poses(self, msg):
        with self.lock:
            self.object_poses_raw = msg.data
            self._count("object_poses")

    def _on_pad_points(self, msg):
        with self.lock:
            self.pad_points_raw = list(msg.data)
            self._count("pad_points")

    def _on_scene_reset(self, msg):
        with self.lock:
            self.reset_events.append(msg.data)
            self._count("scene_reset")

    def _on_eval_bbox(self, msg):
        with self.lock:
            self.eval_bbox = msg
            self._count("eval_bbox")

    def _on_eval_labels(self, msg):
        with self.lock:
            self.eval_labels = msg
            self._count("eval_labels")

    def _on_eval_segmentation(self, msg):
        with self.lock:
            self.eval_segmentation = msg
            self._count("eval_segmentation")

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "sim_time": self.sim_time,
                "joint_states": dict(self.joint_states),
                "applied_commands": dict(self.applied_commands),
                "odom": self.odom,
                "cmd_vel": tuple(self.cmd_vel),
                "ee_poses": dict(self.ee_poses),
                "images": dict(self.images),
                "depths": dict(self.depths),
                "object_poses_raw": self.object_poses_raw,
                "pad_points_raw": self.pad_points_raw,
            }

    def drain_reset_events(self) -> list:
        with self.lock:
            events, self.reset_events = self.reset_events, []
        return events

    def message_count(self, key: str) -> int:
        with self.lock:
            return self.message_counts.get(key, 0)

    def publish_scene_reset_request(self) -> None:
        self._reset_request_pub.publish(String())

    def eval_snapshot(self) -> tuple:
        with self.lock:
            return self.eval_bbox, self.eval_labels, self.eval_segmentation


# ---------------------------------------------------------------------------
# Frame assembly
# ---------------------------------------------------------------------------
def build_action(snap: dict) -> np.ndarray:
    action = np.full(ACTION_DIM, np.nan, dtype=np.float32)
    action[0:3] = snap["cmd_vel"]
    applied = snap["applied_commands"]
    measured = snap["joint_states"]
    for i, name in enumerate(LEFT_JOINTS + RIGHT_JOINTS):
        value = resolve_joint(applied, name)
        if not math.isfinite(value):
            value = resolve_joint(measured, name)  # hold current position
        action[3 + i] = value
    action[17] = gripper_open_fraction(
        resolve_joint(applied, LEFT_GRIPPER_DRIVER, 0.0)
    )
    action[18] = gripper_open_fraction(
        resolve_joint(applied, RIGHT_GRIPPER_DRIVER, 0.0)
    )
    spine_target = resolve_joint(applied, SPINE_JOINT)
    if not math.isfinite(spine_target):
        spine_target = resolve_joint(measured, SPINE_JOINT, 0.0)
    action[19] = spine_target
    return action


def build_state(snap: dict) -> np.ndarray:
    state = np.full(STATE_DIM, np.nan, dtype=np.float32)
    for offset, side in ((0, "left"), (7, "right")):
        pose = snap["ee_poses"].get(side)
        if pose is not None:
            state[offset : offset + 7] = pose
    measured = snap["joint_states"]
    for i, name in enumerate(LEFT_JOINTS + RIGHT_JOINTS):
        state[14 + i] = resolve_joint(measured, name)
    state[28] = resolve_joint(measured, SPINE_JOINT, 0.0)
    state[29] = gripper_open_fraction(
        resolve_joint(measured, LEFT_GRIPPER_DRIVER, 0.0)
    )
    state[30] = gripper_open_fraction(
        resolve_joint(measured, RIGHT_GRIPPER_DRIVER, 0.0)
    )
    odom = snap["odom"]
    if odom is not None:
        x, y, _, qx, qy, qz, qw, vx, vy, _, wz = odom
        state[31:34] = (x, y, quat_to_yaw(qx, qy, qz, qw))
        state[34:37] = (vx, vy, wz)
    return state


def parse_pad_points(raw) -> tuple:
    """(sim_time, (N, 3) float32) from the Float32MultiArray payload."""
    if raw is None or len(raw) < 2:
        return None, None
    pad_time = float(raw[0])
    count = int(raw[1])
    points = np.asarray(raw[2 : 2 + count * 3], dtype=np.float32)
    if points.size != count * 3:
        return None, None
    return pad_time, points.reshape(count, 3)


# ---------------------------------------------------------------------------
# Ground-truth extras buffered per episode
# ---------------------------------------------------------------------------
class ExtrasBuffer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.sim_times = []
        self.wall_times_ns = []
        self.object_names = None
        self.object_poses = []
        self.pad_times = []
        self.pad_points = []
        self._last_pad_time = None
        self.depth_frames = {}  # cam -> list[(frame_idx, array)]
        self.reset_events = []

    def add_frame(self, frame_index: int, snap: dict, *, depth_this_frame):
        """Buffer one frame of ground truth; returns what was appended so
        the in-flight crash-recovery stream can mirror it."""
        self.sim_times.append(snap["sim_time"])
        self.wall_times_ns.append(time.time_ns())

        raw = snap["object_poses_raw"]
        pose_row = None
        if raw is not None:
            try:
                payload = json.loads(raw)
                objects = payload.get("objects", {})
                if self.object_names is None:
                    self.object_names = sorted(objects)
                pose_row = np.array(
                    [
                        objects.get(name, [np.nan] * 7)
                        for name in self.object_names
                    ],
                    dtype=np.float32,
                )
            except (ValueError, TypeError):
                pose_row = None
        if pose_row is None:
            width = len(self.object_names) if self.object_names else 0
            pose_row = np.full((width, 7), np.nan, dtype=np.float32)
        self.object_poses.append(pose_row)

        pad_time, points = parse_pad_points(snap["pad_points_raw"])
        new_pad_points = None
        if points is not None and pad_time != self._last_pad_time:
            self._last_pad_time = pad_time
            self.pad_times.append(pad_time)
            self.pad_points.append(points)
            new_pad_points = points

        if depth_this_frame:
            for key, msg in snap["depths"].items():
                if msg is None:
                    continue
                self.depth_frames.setdefault(key, []).append(
                    (frame_index, depth_msg_to_array(msg).astype(np.float16))
                )

        return {
            "object_names": self.object_names,
            "object_poses": pose_row,
            "pad_time": pad_time if new_pad_points is not None else None,
            "pad_points": new_pad_points,
        }

    def save(self, extras_dir: Path, episode_index: int) -> dict:
        extras_dir.mkdir(parents=True, exist_ok=True)
        arrays = {
            "sim_time": np.asarray(self.sim_times, dtype=np.float64),
            "wall_time_ns": np.asarray(self.wall_times_ns, dtype=np.int64),
        }
        if self.object_names:
            widths = {row.shape[0] for row in self.object_poses}
            if len(widths) > 1:  # names discovered mid-episode
                width = len(self.object_names)
                self.object_poses = [
                    row
                    if row.shape[0] == width
                    else np.full((width, 7), np.nan, dtype=np.float32)
                    for row in self.object_poses
                ]
            arrays["object_poses"] = np.stack(self.object_poses)
            arrays["object_names"] = np.array(self.object_names)
        if self.pad_points:
            counts = {points.shape[0] for points in self.pad_points}
            if len(counts) == 1:
                arrays["pad_points"] = np.stack(self.pad_points)
            else:  # topology changed (should not happen); store flattened
                arrays["pad_points_flat"] = np.concatenate(
                    [points.reshape(-1) for points in self.pad_points]
                )
                arrays["pad_points_counts"] = np.asarray(
                    [points.shape[0] for points in self.pad_points],
                    dtype=np.int64,
                )
            arrays["pad_sim_time"] = np.asarray(
                self.pad_times, dtype=np.float64
            )
        for key, frames in self.depth_frames.items():
            arrays[f"depth_{key}"] = np.stack([f[1] for f in frames])
            arrays[f"depth_{key}_frame_index"] = np.asarray(
                [f[0] for f in frames], dtype=np.int64
            )
        path = extras_dir / f"episode_{episode_index:06d}.npz"
        np.savez_compressed(path, **arrays)
        return {
            "extras_file": path.name,
            "frames": len(self.sim_times),
            "pad_snapshots": len(self.pad_points),
            "depth_frames": {
                key: len(frames) for key, frames in self.depth_frames.items()
            },
        }


# ---------------------------------------------------------------------------
# Success suggestion via the repo's task2 eval logic
# ---------------------------------------------------------------------------
def load_eval_modules(eval_dir: Path):
    if not eval_dir.is_dir():
        return None
    sys.path.insert(0, str(eval_dir))
    try:
        import evaluation  # noqa: PLC0415
        import image_utils  # noqa: PLC0415
        from config import SEMANTIC_RAW_ID_NAME_HINTS  # noqa: PLC0415

        return {
            "evaluate": evaluation.evaluate_thermalpad_target_iou,
            "to_label_array": image_utils.ros_image_to_label_array,
            "hints": SEMANTIC_RAW_ID_NAME_HINTS,
        }
    except Exception as exc:  # noqa: BLE001 - suggestion is best-effort
        logging.warning(
            "Task2 eval modules unavailable (%s): %s", eval_dir, exc
        )
        return None


def suggest_success(eval_modules, node: Task2RecorderNode):
    if eval_modules is None:
        return None
    bbox, labels, segmentation = node.eval_snapshot()
    if bbox is None or labels is None:
        return None
    label_array = None
    if segmentation is not None:
        try:
            label_array = eval_modules["to_label_array"](segmentation)
        except Exception:  # noqa: BLE001
            label_array = None
    try:
        return eval_modules["evaluate"](
            bbox,
            labels.data,
            thermalpad_label="thermalpad",
            liner_label="liner",
            target_label="target",
            semantic_hints=eval_modules["hints"],
            label_array=label_array,
        )
    except Exception as exc:  # noqa: BLE001
        logging.warning("Success suggestion failed: %s", exc)
        return None


# ---------------------------------------------------------------------------
# Dataset helpers
# ---------------------------------------------------------------------------
_FRAG_MP4_OPTIONS = {
    # Fragmented MP4 (OBS-style): ftyp+moov up front, then self-contained
    # moof/mdat fragments, so a killed recorder leaves a playable file.
    "movflags": "+frag_keyframe+empty_moov+default_base_moof",
    # Flush the muxer's IO buffer per packet; without this up to 32 KB of
    # tail (or the whole file, early in an episode) sits in memory and a
    # crash truncates the last fragment mid-write.
    "flush_packets": "1",
}


def install_fragmented_mp4_shim() -> bool:
    """Make lerobot's streaming encoder write fragmented temp MP4s.

    lerobot 0.6.0 opens the per-episode temp container with
    av.open(path, "w") and no muxer options, so the moov atom is written
    only at close and a killed recorder leaves an unplayable file.
    VideoEncoderConfig.extra_options are codec options, not muxer
    options, so there is no public hook; instead replace the
    module-level ``av`` reference in lerobot.datasets.video_utils with a
    proxy whose open() injects the fragmented-MP4 muxer options for
    '*_streaming.mp4' write targets and delegates everything else to the
    real av module. The options are muxer-level and codec-independent
    (libsvtav1, h264/hevc, NVENC alike). Final chunk files are mostly
    unaffected: episode appends re-mux to a standard MP4; a chunk file
    that received exactly one whole episode stays fragmented but valid.

    Returns True if installed; on any layout mismatch it logs a warning
    and returns False (recording still works, but interrupted episodes
    leave unplayable temp MP4s).
    """
    try:
        import av
        from lerobot.datasets import video_utils
    except ImportError as exc:
        logging.warning("Fragmented-MP4 shim not installed (%s)", exc)
        return False
    current = getattr(video_utils, "av", None)
    if getattr(current, "_task2_frag_shim", False):
        return True
    if current is not av or not hasattr(video_utils, "StreamingVideoEncoder"):
        logging.warning(
            "Fragmented-MP4 shim not installed: lerobot.datasets."
            "video_utils does not match the expected lerobot 0.6.0 "
            "layout; interrupted episodes will leave unplayable temp "
            "MP4s."
        )
        return False

    class _FragmentedAvProxy:
        _task2_frag_shim = True

        def __getattr__(self, name):
            return getattr(av, name)

        @staticmethod
        def open(file, mode="r", *open_args, **open_kwargs):
            if mode == "w" and str(file).endswith("_streaming.mp4"):
                options = dict(open_kwargs.pop("options", None) or {})
                for key, value in _FRAG_MP4_OPTIONS.items():
                    options.setdefault(key, value)
                open_kwargs["options"] = options
            return av.open(file, mode, *open_args, **open_kwargs)

    video_utils.av = _FragmentedAvProxy()
    logging.info(
        "Streaming temp MP4s will be fragmented (movflags=%s)",
        _FRAG_MP4_OPTIONS["movflags"],
    )
    return True


# Encoder name -> stream codec family, for comparing a requested encoder
# against the "video.codec" probed from files of an existing dataset.
_VCODEC_FAMILIES = {
    "h264_nvenc": "h264",
    "h264_vaapi": "h264",
    "h264_qsv": "h264",
    "h264_videotoolbox": "h264",
    "libx264": "h264",
    "hevc_nvenc": "hevc",
    "hevc_videotoolbox": "hevc",
    "libx265": "hevc",
    "libsvtav1": "av1",
    "libaom-av1": "av1",
}


def build_rgb_encoder(vcodec):
    """RGBEncoderConfig for --rgb-vcodec (None -> lerobot defaults)."""
    if not vcodec:
        return None
    from lerobot.configs.video import RGBEncoderConfig

    # The constructor resolves "auto" to a concrete encoder and raises on
    # unknown/unavailable codecs.
    encoder = RGBEncoderConfig(vcodec=vcodec)
    if encoder.vcodec.endswith("_nvenc"):
        # NVENC rejects lerobot's universal GOP g=2 with its default
        # B-frames ("Gop Length should be greater than number of B frames
        # + 1"); bf=0 also matches the fast-random-access intent of g=2.
        encoder = RGBEncoderConfig(
            vcodec=encoder.vcodec, extra_options={"bf": 0}
        )
    if encoder.vcodec != vcodec:
        logging.info("--rgb-vcodec %s resolved to %s", vcodec, encoder.vcodec)
    return encoder


def dataset_write_kwargs(args):
    """Writer kwargs shared by every LeRobotDataset.create/resume call."""
    return {
        "image_writer_processes": args.image_writer_processes,
        "image_writer_threads": args.image_writer_threads,
        "streaming_encoding": args.streaming_encoding,
        "encoder_queue_maxsize": args.encoder_queue_maxsize,
        "encoder_threads": args.encoder_threads,
        "rgb_encoder": build_rgb_encoder(args.rgb_vcodec),
    }


def check_rgb_vcodec_consistency(dataset, rgb_vcodec):
    """Refuse to append episodes with a different codec family.

    At recording time lerobot appends episodes to the chunked video files
    with a stream-copy concat and no compatibility check, so a codec
    switch would corrupt them silently."""
    encoder = build_rgb_encoder(rgb_vcodec)
    if encoder is None or dataset.num_episodes == 0:
        return
    requested = _VCODEC_FAMILIES.get(encoder.vcodec, encoder.vcodec)
    depth_keys = set(getattr(dataset.meta, "depth_keys", []) or [])
    for key, feature in dataset.meta.features.items():
        if feature.get("dtype") != "video" or key in depth_keys:
            continue
        stored = (feature.get("info") or {}).get("video.codec")
        if stored is None:
            continue
        stored_family = _VCODEC_FAMILIES.get(stored, stored)
        if stored_family != requested:
            raise SystemExit(
                f"--rgb-vcodec {rgb_vcodec} ({requested}) does not match "
                f"the existing dataset's codec ({key} is {stored}); "
                "appending would corrupt the chunked video files. Resume "
                "without --rgb-vcodec or keep it consistent."
            )


def get_next_dataset_version_path(output_dir, repo_name):
    output_root = Path(output_dir)
    version = 1
    while True:
        versioned_repo_name = f"{repo_name}_v{version}"
        candidate = output_root / versioned_repo_name
        if not candidate.exists():
            return candidate, versioned_repo_name
        if not candidate.is_dir():
            raise FileExistsError(
                f"Dataset path exists but is not a directory: {candidate}"
            )
        version += 1


def get_resume_dataset_version_path(output_dir, repo_name, version=None):
    """Path of an existing dataset version to append to (latest if None)."""
    output_root = Path(output_dir)
    if version is None:
        versions = []
        for candidate in output_root.glob(f"{repo_name}_v*"):
            suffix = candidate.name[len(repo_name) + 2 :]
            if candidate.is_dir() and suffix.isdigit():
                versions.append(int(suffix))
        if not versions:
            raise SystemExit(
                f"--resume: no existing {repo_name}_vN dataset under "
                f"{output_root}"
            )
        version = max(versions)
    versioned_repo_name = f"{repo_name}_v{version}"
    candidate = output_root / versioned_repo_name
    if not candidate.is_dir():
        raise SystemExit(f"--resume: dataset not found: {candidate}")
    return candidate, versioned_repo_name


def validate_resume_dataset(dataset_path, fps, camera_keys, robot_id):
    """Refuse to append with a recorder config that mismatches the dataset.

    A camera mismatch would fail loudly later, but an fps mismatch would
    silently record wrong frame timestamps, so both are checked up front
    against the dataset's meta/info.json.
    """
    info_path = Path(dataset_path) / "meta" / "info.json"
    if not info_path.exists():
        raise SystemExit(
            f"--resume: {info_path} missing; not a LeRobot dataset "
            "(or the previous session crashed before finalizing)"
        )
    info = json.loads(info_path.read_text(encoding="utf-8"))
    if info.get("fps") != fps:
        raise SystemExit(
            f"--resume: dataset was recorded at fps={info.get('fps')}, "
            f"recorder is configured with --fps {fps}"
        )
    existing_cams = {
        key.removeprefix("observation.images.")
        for key in info.get("features", {})
        if key.startswith("observation.images.")
    }
    if existing_cams != set(camera_keys):
        raise SystemExit(
            f"--resume: dataset cameras {sorted(existing_cams)} != "
            f"recorder cameras {sorted(camera_keys)}"
        )
    if info.get("robot_type") != robot_id:
        logging.warning(
            "--resume: dataset robot_type=%r differs from --robot_id %r",
            info.get("robot_type"),
            robot_id,
        )


def setup_recording_tmp(dataset_path) -> Path:
    """Create <dataset>.tmp/ and point the dataset's images/ scratch there.

    lerobot dumps camera frames as temporary PNGs under <root>/images/
    (hardcoded) before video encoding; symlinking that into the sibling
    tmp dir keeps the dataset directory itself limited to the actual
    dataset format (meta/, data/, videos/, task2_extras/).

    With --streaming-encoding no PNGs are written; a crashed run instead
    leaves per-camera tmp*/ dirs holding partial (fragmented, hence
    playable) *_streaming.mp4 files in the dataset root, which are swept
    into <tmp>/streaming_leftover/ here.
    """
    dataset_path = Path(dataset_path)
    tmp_dir = dataset_path.parent / (dataset_path.name + ".tmp")
    (tmp_dir / "images").mkdir(parents=True, exist_ok=True)

    dataset_path.mkdir(parents=True, exist_ok=True)
    stray_streaming = [
        p
        for p in dataset_path.glob("tmp*")
        if p.is_dir() and any(p.glob("*_streaming.mp4"))
    ]
    if stray_streaming:
        # Timestamped so repeated crashes cannot collide on the random
        # mkdtemp names.
        salvage = (
            tmp_dir / "streaming_leftover" / time.strftime("%Y%m%d-%H%M%S")
        )
        salvage.mkdir(parents=True, exist_ok=True)
        logging.warning(
            "Moving %d stray streaming temp dir(s) from a crashed run into %s",
            len(stray_streaming),
            salvage,
        )
        for p in stray_streaming:
            shutil.move(str(p), str(salvage / p.name))
    images_link = dataset_path / "images"
    if images_link.is_symlink():
        images_link.unlink()
    elif images_link.exists():
        # Real dir from an older run: empty scaffolding is deleted, stray
        # PNGs (crashed episode) are moved aside instead of destroyed.
        leftovers = [p for p in images_link.rglob("*") if p.is_file()]
        if leftovers:
            salvage = tmp_dir / "images_leftover"
            logging.warning(
                "Moving %d stray image file(s) from %s to %s",
                len(leftovers),
                images_link,
                salvage,
            )
            shutil.move(str(images_link), str(salvage))
        else:
            shutil.rmtree(images_link)
    images_link.symlink_to(os.path.relpath(tmp_dir / "images", dataset_path))
    return tmp_dir


def cleanup_recording_tmp(dataset_path, tmp_dir) -> None:
    """Remove the images/ symlink; drop the tmp dir unless it holds data."""
    images_link = Path(dataset_path) / "images"
    if images_link.is_symlink():
        images_link.unlink()
    tmp_dir = Path(tmp_dir)
    if not any(p.is_file() for p in tmp_dir.rglob("*")):
        shutil.rmtree(tmp_dir, ignore_errors=True)
    else:
        logging.warning("Keeping %s: it still contains salvage data", tmp_dir)


def build_features(camera_keys) -> dict:
    features = {
        "action": {
            "dtype": "float32",
            "shape": (ACTION_DIM,),
            "names": ACTION_NAMES,
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (STATE_DIM,),
            "names": STATE_NAMES,
        },
    }
    for key in camera_keys:
        features[f"observation.images.{key}"] = {
            "dtype": "video",
            "shape": list(CAMERAS[key]["shape"]),
            "names": ["height", "width", "channels"],
        }
    return features


def launch_dataset_visualization(repo_id, dataset_path, episode_index):
    # The recorder container is headless, so serve the Rerun web viewer
    # (--mode distant) instead of spawning a local window. With the
    # service on host networking the viewer is reachable from the host
    # browser directly.
    cmd = [
        sys.executable,
        "-m",
        "lerobot.scripts.lerobot_dataset_viz",
        "--repo-id",
        repo_id,
        "--root",
        str(dataset_path),
        "--episode-index",
        str(episode_index),
        "--mode",
        "distant",
        "--display-compressed-images",
    ]
    # The repo_id only exists locally; without this, any local load hiccup
    # makes lerobot fall back to the HuggingFace hub and die on a 401.
    env = {**os.environ, "HF_HUB_OFFLINE": "1"}
    print("   Open http://localhost:9090 in a browser to view the episode.")
    return subprocess.Popen(cmd, env=env)


# ---------------------------------------------------------------------------
# Console controls

# key -> (action, menu text); the printed menus and prompts are generated
# from these. "reset" deliberately shares its key with the Isaac Sim
# window's own scene-reset key.
IDLE_KEYBINDS = {
    "1": ("reset_record", "reset/randomize the scene, then start recording"),
    "2": (
        "record",
        "start recording without reset (episode starts at the current sim "
        "time; use after manually reposing)",
    ),
    "4": ("visualize", "visualize a saved episode"),
    "5": (
        "reset",
        "reset/randomize the scene (same key as in the sim window)",
    ),
    "q": ("quit", "quit"),
}
RECORDING_KEYBINDS = {
    "3": ("save", "stop + save episode"),
    "0": ("discard", "stop + discard episode"),
    "q": ("quit", "quit (discards the episode)"),
}


def key_for(keybinds: dict, action: str) -> str:
    return next(k for k, (bound, _) in keybinds.items() if bound == action)


class KeyInput:
    """Single-keypress console input with a line-mode fallback.

    On a TTY, activate() holds the terminal in cbreak mode so menu keys
    act without Enter (cbreak keeps ISIG, so Ctrl-C still raises
    KeyboardInterrupt); line_input() drops back to the saved cooked mode
    for full-line prompts. Without a TTY (piped stdin) reads stay
    line-buffered and EOF reads as the quit key, so automated runs keep
    working.
    """

    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        self._fd = sys.stdin.fileno() if self.is_tty else None
        self._saved = None

    def activate(self) -> None:
        if self.is_tty and self._saved is None:
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)

    def restore(self) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            self._saved = None

    def read_key(self, timeout=None):
        """One pressed key (lowercased), or None if the timeout expires."""
        if timeout is not None:
            if not select.select([sys.stdin], [], [], timeout)[0]:
                return None
        if self._saved is not None:
            key = os.read(self._fd, 1).decode(errors="replace")
            print(key)  # cbreak disables ECHO; show what was pressed
            return key.lower()
        line = sys.stdin.readline()
        if not line:  # EOF on piped stdin
            return key_for(IDLE_KEYBINDS, "quit")
        return line.strip()[:1].lower()

    def line_input(self, prompt: str) -> str:
        """input() with the terminal temporarily back in cooked mode."""
        if self._saved is None:
            return input(prompt)
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
        try:
            return input(prompt)
        finally:
            tty.setcbreak(self._fd)


CONSOLE = KeyInput()


def prompt_success(suggestion) -> bool:
    default = None
    if suggestion is not None:
        iou = suggestion.get("iou_thermalpad_vs_target_current", 0.0)
        orientation_ok = suggestion.get("is_orientation_correct", False)
        case = suggestion.get("orientation_case", "")
        default = bool(orientation_ok and iou > 0.0)
        print(
            f"   Auto-suggested success: {default} "
            f"(IoU={iou:.3f}, orientation_ok={orientation_ok}, case={case})"
        )
    while True:
        hint = f"[Enter={default}, y, n]" if default is not None else "[y, n]"
        answer = (
            CONSOLE.line_input(f"   Episode successful? {hint}: ")
            .strip()
            .lower()
        )
        if not answer and default is not None:
            return default
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False


def get_streaming_dropped_frames(dataset) -> dict:
    """Per-camera dropped-frame counts of the current episode.

    Reads private lerobot internals (the container pins lerobot 0.6.0;
    the counts are cleared at each episode start). Returns {} when
    streaming encoding is off or the internals moved."""
    writer = getattr(dataset, "writer", None)
    encoder = getattr(writer, "_streaming_encoder", None)
    dropped = getattr(encoder, "_dropped_frames", None)
    if not isinstance(dropped, dict):
        return {}
    return {key: int(count) for key, count in dropped.items() if count}


def confirm_save_despite_drops(dropped, *, interactive) -> bool:
    """Warn about encoder frame drops; ask (or refuse) to save anyway.

    A dropped frame leaves the episode's video shorter than its
    action/state rows, desyncing every later frame of that episode."""
    total = sum(dropped.values())
    detail = ", ".join(f"{key}: {n}" for key, n in sorted(dropped.items()))
    print(
        f"⚠️  Streaming encoder dropped {total} video frame(s) ({detail}).\n"
        "   Dropped frames desync video from the action/state rows; "
        "saving is NOT recommended.\n"
        "   (Increase --encoder-queue-maxsize or use --rgb-vcodec auto "
        "for hardware encoding.)"
    )
    if not interactive:
        print("   Refusing to save (--no-prompt-success).")
        return False
    while True:
        answer = CONSOLE.line_input("   Save anyway? [y, N]: ").strip().lower()
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no", ""):
            return False


def wait_for_streams(node: Task2RecorderNode, camera_keys, timeout_s):
    """Block until /clock, states, commands, and all cameras are alive."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snap = node.snapshot()
        missing = []
        if snap["sim_time"] is None:
            missing.append(CLOCK_TOPIC)
        if not snap["joint_states"]:
            missing.append(FULL_STATES_TOPIC)
        if not snap["applied_commands"]:
            missing.append(APPLIED_COMMANDS_TOPIC)
        if snap["odom"] is None:
            missing.append(ODOM_TOPIC)
        missing.extend(
            CAMERAS[key]["image_topic"]
            for key in camera_keys
            if snap["images"].get(key) is None
        )
        if not missing:
            return True
        time.sleep(0.25)
    print(
        "⚠️  Still waiting for topics: "
        + ", ".join(missing)
        + "\n   Is the sim running scene_room.py --record "
        "(and the eval camera for eval_camera)?"
    )
    return False


def wait_for_fresh_clock(node: Task2RecorderNode, timeout_s) -> bool:
    """Block until a /clock message arrives that is newer than the cache.

    The cached sim time can be stale for seconds around a scene reset
    (the clock stops publishing while the timeline is stopped), so an
    episode must not latch its start time on it.
    """
    baseline = node.message_count("clock")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if node.message_count("clock") > baseline:
            return True
        time.sleep(0.01)
    return False


def request_scene_reset(node: Task2RecorderNode, timeout_s) -> bool:
    """Ask the sim to reset the scene; wait for the reset-done event.

    The sim publishes the /isaac/task2/scene_reset event only after
    world.reset() and the ready pose are re-applied, so returning True
    means the scene is ready for the next episode (the event itself stays
    queued and is attached to the episode metadata at episode start).
    """
    baseline = node.message_count("scene_reset")
    node.publish_scene_reset_request()
    print(
        "↻ Scene reset requested (deformable re-init can take a few "
        "seconds) ...",
        flush=True,
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if node.message_count("scene_reset") > baseline:
            return True
        time.sleep(0.05)
    print(
        f"⚠️  No scene-reset event within {timeout_s:.0f}s — is the sim "
        "running scene_room.py --record (scene reset controller)?"
    )
    return False


def handle_visualize_command(
    dataset, dataset_repo_id, dataset_path, visualization_process, args
):
    """Menu command 4: serve an episode in the Rerun web viewer.

    Returns the (possibly reopened) dataset and the viewer process.
    """
    if dataset.num_episodes == 0:
        print("⚠️  No saved episodes to visualize yet.")
        return dataset, visualization_process
    if visualization_process and visualization_process.poll() is None:
        print("ℹ️  Visualization already running.")
        return dataset, visualization_process
    latest = dataset.num_episodes - 1
    raw = CONSOLE.line_input(
        f"   Episode to visualize [0-{latest}, Enter={latest}]: "
    ).strip()
    episode_to_view = latest
    if raw:
        try:
            episode_to_view = int(raw)
        except ValueError:
            print(f"⚠️  Not a number: {raw!r}")
            return dataset, visualization_process
        if not 0 <= episode_to_view <= latest:
            print(f"⚠️  Episode out of range: {episode_to_view}")
            return dataset, visualization_process
    # The episodes-metadata parquet is buffered in memory and only becomes
    # readable once its writer is closed, so finalize before visualizing
    # and reopen for appending (resume rolls over to a fresh metadata
    # file).
    print("   Flushing dataset to disk ...")
    dataset.finalize()
    visualization_process = launch_dataset_visualization(
        dataset_repo_id, dataset_path, episode_to_view
    )
    dataset = LeRobotDataset.resume(
        dataset_repo_id,
        root=dataset_path,
        **dataset_write_kwargs(args),
    )
    return dataset, visualization_process


def clear_episode_buffer(dataset) -> None:
    """Drop the in-progress episode (method name varies across lerobot
    versions)."""
    if hasattr(dataset, "clear_episode_buffer"):
        dataset.clear_episode_buffer()
    else:
        dataset.episode_buffer = dataset.create_episode_buffer()


def open_dataset(args, camera_keys):
    """Create a new dataset version or resume an existing one.

    Returns (dataset, dataset_path, dataset_repo_id).
    """
    resuming = args.resume or args.resume_version is not None
    if resuming:
        dataset_path, versioned_repo_name = get_resume_dataset_version_path(
            args.output_dir, args.repo_name, args.resume_version
        )
        validate_resume_dataset(
            dataset_path, args.fps, camera_keys, args.robot_id
        )
    else:
        dataset_path, versioned_repo_name = get_next_dataset_version_path(
            args.output_dir, args.repo_name
        )
    dataset_repo_id = f"{args.hub_namespace}/{versioned_repo_name}"
    if resuming:
        dataset = LeRobotDataset.resume(
            dataset_repo_id,
            root=dataset_path,
            **dataset_write_kwargs(args),
        )
        check_rgb_vcodec_consistency(dataset, args.rgb_vcodec)
        logging.info(
            "Resuming dataset at: %s (%d episode(s) already recorded)",
            dataset_path,
            dataset.num_episodes,
        )
    else:
        logging.info("Recording dataset to: %s", dataset_path)
        dataset = LeRobotDataset.create(
            repo_id=dataset_repo_id,
            fps=args.fps,
            root=dataset_path,
            robot_type=args.robot_id,
            features=build_features(camera_keys),
            use_videos=True,
            **dataset_write_kwargs(args),
        )
    return dataset, dataset_path, dataset_repo_id


def print_controls_menu(camera_keys, fps) -> None:
    key_hint = (
        "press once, no Enter needed"
        if CONSOLE.is_tty
        else "no TTY: type + Enter"
    )
    print("\n" + "=" * 66)
    print(" Task 2 LeRobot recorder (sim-time paced)")
    print(
        f"   action {ACTION_DIM}-dim / state {STATE_DIM}-dim, "
        f"cameras: {', '.join(camera_keys)}, fps={fps} (sim time)"
    )
    print(f" Idle keys ({key_hint}):")
    for key, (_, menu) in IDLE_KEYBINDS.items():
        print(f"   [{key}] {menu}")
    print(" While recording:")
    for key, (_, menu) in RECORDING_KEYBINDS.items():
        print(f"   [{key}] {menu}")
    print("   (reset between episodes, never while recording)")
    print("=" * 66 + "\n")


def save_episode_with_metadata(
    dataset,
    node,
    args,
    eval_modules,
    extras,
    extras_dir,
    meta_path,
    *,
    frame_count,
    dropped_stale,
    encoder_dropped,
    pre_reset_events,
    episode_start_sim,
    sim_time_end,
) -> None:
    """Label success, save the buffered episode, and append its extras
    sidecar metadata line."""
    suggestion = suggest_success(eval_modules, node)
    success = (
        prompt_success(suggestion)
        if args.prompt_success
        else bool(
            suggestion
            and suggestion.get("is_orientation_correct")
            and suggestion.get("iou_thermalpad_vs_target_current", 0.0) > 0.0
        )
    )
    episode_index = dataset.num_episodes
    print(f"💾 Saving episode {episode_index} ...")
    dataset.save_episode()
    extras_info = extras.save(extras_dir, episode_index)
    meta_line = {
        "episode_index": episode_index,
        "success": success,
        "frames": frame_count,
        "dropped_stale_frames": dropped_stale,
        "encoder_dropped_frames": encoder_dropped or None,
        "fps_sim": args.fps,
        "task": args.single_task,
        "sim_time_start": episode_start_sim,
        "sim_time_end": sim_time_end,
        "wall_time_saved": time.time(),
        "scene_reset_events": [
            json.loads(event)
            for event in (pre_reset_events + node.drain_reset_events())
        ],
        "success_suggestion": {
            key: suggestion[key]
            for key in (
                "iou_thermalpad_vs_target_current",
                "is_orientation_correct",
                "orientation_case",
            )
        }
        if suggestion
        else None,
        **extras_info,
    }
    extras_dir.mkdir(parents=True, exist_ok=True)
    with meta_path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(meta_line) + "\n")
    print(
        f"✔ Saved episode {episode_index} "
        f"({frame_count} frames, success={success}, "
        f"{dropped_stale} stale frames dropped)"
    )


# ---------------------------------------------------------------------------
# Main recording loop
# ---------------------------------------------------------------------------
def run_recording(args):
    init_logging()
    if args.streaming_encoding and args.fragmented_mp4:
        # Must happen before any LeRobotDataset is created so every
        # per-episode encoder goes through the proxy.
        install_fragmented_mp4_shim()
    camera_keys = [
        key.strip() for key in args.cameras.split(",") if key.strip()
    ]
    unknown = sorted(set(camera_keys) - set(CAMERAS))
    if unknown:
        raise SystemExit(
            f"Unknown cameras {unknown}; choose from {sorted(CAMERAS)}"
        )

    rclpy.init()
    node = Task2RecorderNode(
        camera_keys,
        record_depth=args.record_depth,
        suggest_success=args.suggest_success,
        qos_depth=args.qos_depth,
    )
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    eval_modules = (
        load_eval_modules(EVAL_MODULE_DIR) if args.suggest_success else None
    )

    dataset, dataset_path, dataset_repo_id = open_dataset(args, camera_keys)
    # After creation: LeRobotDataset.create requires a not-yet-existing
    # root, and no images are written before the first add_frame.
    tmp_dir = setup_recording_tmp(dataset_path)
    extras_dir = Path(dataset_path) / "task2_extras"
    meta_path = extras_dir / "episodes_task2.jsonl"
    extras = ExtrasBuffer()

    print_controls_menu(camera_keys, args.fps)

    wait_for_streams(node, camera_keys, timeout_s=args.stream_timeout_s)

    sample_period = 1.0 / args.fps
    recorded_episodes = 0
    visualization_process = None
    idle_prompt = ", ".join(
        f"{key}={action}" for key, (action, _) in IDLE_KEYBINDS.items()
    )

    CONSOLE.activate()
    try:
        while recorded_episodes < args.max_episodes:
            print(
                f"\n[{dataset.num_episodes} episode(s) in dataset, "
                f"{recorded_episodes} this session] "
                f"command ({idle_prompt}): ",
                end="",
                flush=True,
            )
            action, _ = IDLE_KEYBINDS.get(CONSOLE.read_key(), (None, None))
            if action is None:
                continue
            if action == "visualize":
                dataset, visualization_process = handle_visualize_command(
                    dataset,
                    dataset_repo_id,
                    dataset_path,
                    visualization_process,
                    args,
                )
                continue
            if action == "quit":
                break
            if action in ("reset", "reset_record"):
                if not request_scene_reset(
                    node, timeout_s=args.reset_timeout_s
                ):
                    continue
                if action == "reset":
                    continue
            # action is "record" or "reset_record": start an episode

            # -------------------------------------------------- episode ---
            if not wait_for_streams(
                node, camera_keys, timeout_s=args.stream_timeout_s
            ):
                continue
            if not wait_for_fresh_clock(node, timeout_s=args.stream_timeout_s):
                print(
                    "⚠️  No fresh /clock message within "
                    f"{args.stream_timeout_s:.0f}s (sim paused or "
                    "mid-reset?) — not starting the episode."
                )
                continue
            # Defensive: every save/discard path already leaves the buffer
            # empty, but stray frames from an abnormal abort would silently
            # prepend to this episode.
            clear_episode_buffer(dataset)
            extras.reset()
            pre_reset_events = node.drain_reset_events()
            snap = node.snapshot()
            episode_start_sim = snap["sim_time"]
            next_sample = episode_start_sim
            frame_count = 0
            dropped_stale = 0
            print(
                f"● Recording (sim t0={episode_start_sim:.2f}s). "
                f"Press {key_for(RECORDING_KEYBINDS, 'save')} to save, "
                f"{key_for(RECORDING_KEYBINDS, 'discard')} to discard, "
                f"{key_for(RECORDING_KEYBINDS, 'quit')} to quit.",
                flush=True,
            )
            stop_cmd = None
            while stop_cmd is None:
                # Pace on simulation time from /clock.
                while True:
                    snap = node.snapshot()
                    sim_time = snap["sim_time"]
                    if sim_time is None:
                        time.sleep(0.005)
                        continue
                    if sim_time + 1e-9 < episode_start_sim:
                        # Sim was reset mid-episode; rebase but flag it.
                        print(
                            "\n⚠️  Simulation clock jumped backwards "
                            "(scene reset while recording?) — discarding "
                            "this episode.",
                            flush=True,
                        )
                        stop_cmd = "discard"
                        break
                    if sim_time - next_sample > CLOCK_FORWARD_JUMP_TOLERANCE_S:
                        print(
                            "\n⚠️  Simulation clock jumped forwards by "
                            f"{sim_time - next_sample:.1f}s (scene reset "
                            "while recording, or recorder stall) — "
                            "discarding this episode.",
                            flush=True,
                        )
                        stop_cmd = "discard"
                        break
                    if sim_time >= next_sample:
                        break
                    pressed = CONSOLE.read_key(timeout=0.0)
                    if pressed in RECORDING_KEYBINDS:
                        stop_cmd = RECORDING_KEYBINDS[pressed][0]
                        break
                    time.sleep(0.001)
                if stop_cmd is not None:
                    break

                frame = {
                    "action": build_action(snap),
                    "observation.state": build_state(snap),
                    "task": args.single_task,
                }
                stale = False
                for key in camera_keys:
                    msg = snap["images"].get(key)
                    if msg is None:
                        stale = True
                        break
                    frame[f"observation.images.{key}"] = image_msg_to_array(
                        msg
                    )
                if stale:
                    dropped_stale += 1
                else:
                    dataset.add_frame(frame)
                    extras.add_frame(
                        frame_count,
                        snap,
                        depth_this_frame=(
                            args.record_depth
                            and frame_count % max(args.depth_every, 1) == 0
                        ),
                    )
                    frame_count += 1
                next_sample += sample_period

                if frame_count and frame_count % args.fps == 0:
                    print(
                        f"   {frame_count} frames "
                        f"({frame_count / args.fps:.0f}s sim time)...",
                        end="\r",
                        flush=True,
                    )
                if frame_count >= int(args.fps * args.max_episode_time_s):
                    print("\n⏱  Max episode length reached.")
                    stop_cmd = "save"

            save_requested = stop_cmd == "save" and frame_count > 0
            encoder_dropped = {}
            if save_requested and args.streaming_encoding:
                encoder_dropped = get_streaming_dropped_frames(dataset)
                if encoder_dropped:
                    save_requested = confirm_save_despite_drops(
                        encoder_dropped, interactive=args.prompt_success
                    )
            if save_requested:
                save_episode_with_metadata(
                    dataset,
                    node,
                    args,
                    eval_modules,
                    extras,
                    extras_dir,
                    meta_path,
                    frame_count=frame_count,
                    dropped_stale=dropped_stale,
                    encoder_dropped=encoder_dropped,
                    pre_reset_events=pre_reset_events,
                    episode_start_sim=episode_start_sim,
                    sim_time_end=snap["sim_time"],
                )
                recorded_episodes += 1
            else:
                clear_episode_buffer(dataset)
                extras.reset()
                node.drain_reset_events()
                print("🗑  Episode discarded.")
            if stop_cmd == "quit":
                break
    except KeyboardInterrupt:
        logging.warning("Interrupted; shutting down.")
    finally:
        CONSOLE.restore()
        # Flush buffered episode metadata; without this the dataset is only
        # readable if the interpreter happens to run __del__ on exit.
        dataset.finalize()
        cleanup_recording_tmp(dataset_path, tmp_dir)
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    print(
        f"\nDone: {recorded_episodes} episode(s) recorded this session, "
        f"{dataset.num_episodes} total in {dataset_path}"
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="EBiM Task 2 LeRobot demonstration recorder",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--repo_name", type=str, default="task2_thermalpad")
    parser.add_argument(
        "--hub-namespace",
        type=str,
        default="ebim",
        help="Namespace of the dataset repo_id "
        "(<hub_namespace>/<repo_name>_vN); ids are local-only, nothing is "
        "uploaded to the hub. Keep it constant when resuming a dataset.",
    )
    parser.add_argument(
        "--single_task",
        type=str,
        default=(
            "Pick up the thermal pad and place it on the target RAM board."
        ),
    )
    parser.add_argument("--output_dir", type=str, default="dataset/")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Append episodes to the latest existing <repo_name>_vN "
        "dataset instead of starting a new version.",
    )
    parser.add_argument(
        "--resume_version",
        type=int,
        default=None,
        help="Append to this specific version number (implies --resume).",
    )
    parser.add_argument("--robot_id", type=str, default="fr3duo_mobile_task2")
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Sampling rate in SIMULATION time (paced on /clock).",
    )
    parser.add_argument(
        "--cameras",
        type=str,
        default="head,wrist_left,wrist_right,eval_camera",
        help=f"Comma-separated subset of {sorted(CAMERAS)}.",
    )
    parser.add_argument(
        "--stream-timeout-s",
        type=float,
        default=15.0,
        help="Seconds to wait for /clock, states, and camera topics before "
        "warning that streams are missing.",
    )
    parser.add_argument(
        "--reset-timeout-s",
        type=float,
        default=30.0,
        help="Seconds to wait for the sim's scene-reset event after a "
        "reset menu command (deformable re-initialization takes a few "
        "seconds).",
    )
    parser.add_argument(
        "--qos-depth",
        type=int,
        default=10,
        help="QoS history depth for non-image subscriptions (image topics "
        "use the sensor-data QoS profile).",
    )
    parser.add_argument(
        "--image-writer-processes",
        type=int,
        default=0,
        help="LeRobot image-writer processes (0 = threads inside the "
        "recorder process).",
    )
    parser.add_argument(
        "--image-writer-threads",
        type=int,
        default=4,
        help="LeRobot image-writer threads (per process if processes > 0).",
    )
    parser.add_argument(
        "--streaming-encoding",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Encode camera frames into video on the fly (per-camera PyAV "
        "encoder threads) instead of dumping PNGs and batch-encoding at "
        "save; episode save becomes near-instant. The image-writer flags "
        "are unused in this mode.",
    )
    parser.add_argument(
        "--encoder-queue-maxsize",
        type=int,
        default=90,
        help="Per-camera frame queue of the streaming encoder. A full "
        "queue DROPS frames (desyncing video from action/state rows), so "
        "this is deliberately 3x lerobot's default of 30; 90 frames is "
        "~3 s at 30 fps and ~1 GB worst-case RAM across 4 720p cameras.",
    )
    parser.add_argument(
        "--encoder-threads",
        type=int,
        default=None,
        help="Encoder threads for the streaming encoder "
        "(default: lerobot's internal default).",
    )
    parser.add_argument(
        "--fragmented-mp4",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="With --streaming-encoding: write the in-progress temp MP4s "
        "as fragmented MP4 (OBS-style) so a killed recorder leaves "
        "playable video for salvage. No effect without streaming.",
    )
    parser.add_argument(
        "--rgb-vcodec",
        type=str,
        default=None,
        help="RGB video codec ('auto' picks a hardware encoder such as "
        "h264_nvenc; requires GPU access in the recorder service). "
        "Default: lerobot's libsvtav1. Must stay consistent for the "
        "lifetime of a dataset.",
    )
    parser.add_argument(
        "--record-depth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Buffer float16 depth frames into task2_extras/ (needs "
        "--robot-camera-depth on the sim side; watch RAM usage).",
    )
    parser.add_argument(
        "--depth-every",
        type=int,
        default=6,
        help="Store depth every N recorded frames (6 = 5 Hz at 30 fps).",
    )
    parser.add_argument(
        "--suggest-success",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compute an IoU-based success suggestion from the eval camera "
        "at episode save.",
    )
    parser.add_argument(
        "--prompt-success",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Confirm the success label on the console at episode save "
        "(otherwise the suggestion is stored directly).",
    )
    parser.add_argument("--max_episode_time_s", type=float, default=300.0)
    parser.add_argument("--max_episodes", type=int, default=200)
    parser.add_argument(
        "--config",
        type=str,
        default=str(DEFAULT_CONFIG_PATH),
        help="YAML file with recording defaults (services/recording/"
        "recording.yaml); explicit CLI flags override its values.",
    )
    return parser


def load_recording_config(
    config_path, parser: argparse.ArgumentParser
) -> dict:
    """Defaults from the recording config YAML.

    Layered as: argparse defaults < YAML < explicit CLI flags. Fails hard
    on a missing file, an unknown/CLI-only key, or a value the matching
    argparse action cannot represent.
    """
    import yaml

    config_path = Path(config_path)
    if not config_path.is_file():
        raise FileNotFoundError(f"Recording config not found: {config_path}")
    with open(config_path, encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        raise RuntimeError(
            f"Recording config must be a YAML mapping: {config_path}"
        )

    actions = {
        action.dest: action
        for action in parser._actions
        if action.dest not in CONFIG_CLI_ONLY_KEYS
    }
    defaults = {}
    for key, value in config.items():
        action = actions.get(key)
        if action is None:
            raise RuntimeError(
                f"Unknown or CLI-only key {key!r} in {config_path}; "
                f"valid keys: {sorted(actions)}"
            )
        if value is None:
            continue  # null keeps the built-in argparse default
        if isinstance(action, argparse.BooleanOptionalAction):
            if not isinstance(value, bool):
                raise RuntimeError(
                    f"Key {key!r} in {config_path} must be a boolean, "
                    f"got {value!r}"
                )
        else:
            if key == "cameras" and isinstance(value, list):
                value = ",".join(str(item) for item in value)
            try:
                value = action.type(value) if action.type else value
            except (TypeError, ValueError) as exc:
                raise RuntimeError(
                    f"Key {key!r} in {config_path} has invalid value "
                    f"{value!r} for type {action.type.__name__}"
                ) from exc
        defaults[key] = value
    return defaults


if __name__ == "__main__":
    parser = build_arg_parser()
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument(
        "--config", type=str, default=str(DEFAULT_CONFIG_PATH)
    )
    config_args, _ = config_parser.parse_known_args()
    parser.set_defaults(**load_recording_config(config_args.config, parser))
    run_recording(parser.parse_args())
