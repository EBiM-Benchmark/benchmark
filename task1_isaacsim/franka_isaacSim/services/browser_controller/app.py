#!/usr/bin/env python3
"""Standalone browser-based ROS2 joint controller.

This process runs in its own container and acts as an independent ROS2
controller. It listens to joint states, keeps commanded targets, and publishes
commands at a fixed rate. The HTTP frontend is handled by api/routes.py.
"""

import argparse
import os
import struct
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, JointState

try:
    from geometry_msgs.msg import WrenchStamped
except ImportError:  # pragma: no cover - stub injected in tests
    WrenchStamped = None  # type: ignore[assignment,misc]

try:
    from isaac_camera_service import camera_feed_configs as load_camera_feed_configs
except Exception:  # pragma: no cover
    load_camera_feed_configs = None

try:
    from stack_config import get_config_sections, load_stack_config
except Exception:  # pragma: no cover
    get_config_sections = None
    load_stack_config = None

from isaac_bridge_constants import LEFT_JOINTS, RIGHT_JOINTS

LEFT_GRIPPER_OPENING = ["left_robotiq_opening"]
RIGHT_GRIPPER_OPENING = ["right_robotiq_opening"]

JOINT_GROUPS = [
    {
        "label": "Left Arm",
        "state_topic": "/bridge/left_joint_states",
        "command_topic": "/bridge/left_joint_commands",
        "default_joints": LEFT_JOINTS,
        "mode": "joint_passthrough",
        "min": -3.14,
        "max": 3.14,
    },
    {
        "label": "Right Arm",
        "state_topic": "/bridge/right_joint_states",
        "command_topic": "/bridge/right_joint_commands",
        "default_joints": RIGHT_JOINTS,
        "mode": "joint_passthrough",
        "min": -3.14,
        "max": 3.14,
    },
    {
        "label": "Left Robotiq",
        "state_topic": "/bridge/left_robotiq_joint_states",
        "command_topic": "/bridge/left_robotiq_joint_commands",
        "default_joints": LEFT_GRIPPER_OPENING,
        "mode": "gripper_opening",
        "display_joint": "left_robotiq_opening",
        "driver_joint": "left_robotiq_85_left_knuckle_joint",
        "gripper_open_position": 0.0,
        "gripper_closed_position": 0.8,
        "min": 0.0,
        "max": 1.0,
    },
    {
        "label": "Right Robotiq",
        "state_topic": "/bridge/right_robotiq_joint_states",
        "command_topic": "/bridge/right_robotiq_joint_commands",
        "default_joints": RIGHT_GRIPPER_OPENING,
        "mode": "gripper_opening",
        "display_joint": "right_robotiq_opening",
        "driver_joint": "right_robotiq_85_left_knuckle_joint",
        "gripper_open_position": 0.0,
        "gripper_closed_position": 0.8,
        "min": 0.0,
        "max": 1.0,
    },
]

# FR3/FR3v2 limits from assets/franka_description/robots/fr3v2/joint_limits.yaml
FR3_JOINT_LIMITS = {
    "joint1": (-2.9007400166666666, 2.9007400166666666),
    "joint2": (-1.8360900166666667, 1.8360900166666667),
    "joint3": (-2.9007400166666666, 2.9007400166666666),
    "joint4": (-3.077020016666667, -0.11693708333333333),
    "joint5": (-2.87630335, 2.87630335),
    "joint6": (0.43982265, 4.62163335),
    "joint7": (-3.05083335, 3.05083335),
}

DEFAULT_ACTIVITY_WINDOW_S = 0.75

# ---------------------------------------------------------------------------
# Control modes
# ---------------------------------------------------------------------------

CONTROL_MODES = {
    "ui_control": {
        "label": "UI Control",
        "description": "Browser sliders publish joint commands directly.",
        "publish_commands": True,
    },
    "digital_twin": {
        "label": "Franka DigitalTwin",
        "description": "Real robot joint states are forwarded to the simulation via the real_to_sim_bridge.",
        "publish_commands": False,
    },
    "gello": {
        "label": "Franka Gello",
        "description": "Gello teleoperation device drives the simulation via the real_to_sim_bridge.",
        "publish_commands": False,
    },
}
DEFAULT_CONTROL_MODE = "digital_twin"

# ---------------------------------------------------------------------------
# Hz tracker
# ---------------------------------------------------------------------------

class _HzTracker:
    """Rolling-window message-rate counter.  Thread-safe."""

    def __init__(self, window_s: float = 2.0) -> None:
        self._timestamps: deque = deque()
        self._window_s = float(window_s)
        self._lock = threading.Lock()
        self._last_wall: float = 0.0

    def record(self) -> None:
        now_mono = time.monotonic()
        now_wall = time.time()
        with self._lock:
            self._last_wall = now_wall
            self._timestamps.append(now_mono)
            cutoff = now_mono - self._window_s
            while self._timestamps and self._timestamps[0] < cutoff:
                self._timestamps.popleft()

    def hz(self) -> float:
        now_mono = time.monotonic()
        with self._lock:
            cutoff = now_mono - self._window_s
            while self._timestamps and self._timestamps[0] < cutoff:
                self._timestamps.popleft()
            count = len(self._timestamps)
            if count < 2:
                return 0.0
            span = self._timestamps[-1] - self._timestamps[0]
            return (count - 1) / span if span > 0.0 else 0.0

    def last_wall(self) -> float:
        with self._lock:
            return self._last_wall


# ---------------------------------------------------------------------------
# Monitored topics — data-pipeline health
# ---------------------------------------------------------------------------
# Each entry defines one topic to subscribe to for Hz and contract checking.
# msg_class: "JointState" | "WrenchStamped"
# contract keys:
#   joint_count    — expected len(msg.name)
#   require_effort — True if msg.effort must have ≥ joint_count values

MONITORED_TOPICS = [
    # ── Raw output from Isaac Sim ─────────────────────────────────────────
    {
        "name": "/isaac/left_joint_states",
        "msg_class": "JointState",
        "label": "Isaac L Arm State",
        "contract": {"joint_count": 7, "require_effort": True},
    },
    {
        "name": "/isaac/right_joint_states",
        "msg_class": "JointState",
        "label": "Isaac R Arm State",
        "contract": {"joint_count": 7, "require_effort": True},
    },
    {
        "name": "/isaac/left_ee_wrench",
        "msg_class": "WrenchStamped",
        "label": "Isaac L EE Wrench",
        "contract": {},
    },
    {
        "name": "/isaac/right_ee_wrench",
        "msg_class": "WrenchStamped",
        "label": "Isaac R EE Wrench",
        "contract": {},
    },
    {
        "name": "/isaac/left_robotiq_joint_states",
        "msg_class": "JointState",
        "label": "Isaac L Gripper State",
        "contract": {},
    },
    {
        "name": "/isaac/right_robotiq_joint_states",
        "msg_class": "JointState",
        "label": "Isaac R Gripper State",
        "contract": {},
    },
    # ── Republished bridge topics ─────────────────────────────────────────
    {
        "name": "/bridge/left_joint_states",
        "msg_class": "JointState",
        "label": "Bridge L Arm State",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/bridge/right_joint_states",
        "msg_class": "JointState",
        "label": "Bridge R Arm State",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/bridge/left_robotiq_joint_states",
        "msg_class": "JointState",
        "label": "Bridge L Gripper State",
        "contract": {},
    },
    {
        "name": "/bridge/right_robotiq_joint_states",
        "msg_class": "JointState",
        "label": "Bridge R Gripper State",
        "contract": {},
    },
    # ── Command topics ────────────────────────────────────────────────────
    {
        "name": "/bridge/left_joint_commands",
        "msg_class": "JointState",
        "label": "Bridge L Arm Cmd",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/bridge/right_joint_commands",
        "msg_class": "JointState",
        "label": "Bridge R Arm Cmd",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/bridge/left_robotiq_joint_commands",
        "msg_class": "JointState",
        "label": "Bridge L Gripper Cmd",
        "contract": {},
    },
    {
        "name": "/bridge/right_robotiq_joint_commands",
        "msg_class": "JointState",
        "label": "Bridge R Gripper Cmd",
        "contract": {},
    },
    # ── Sim command topics (forwarded by Republisher to Isaac) ──────────
    {
        "name": "/isaac/left_joint_commands",
        "msg_class": "JointState",
        "label": "Sim L Arm Cmd",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/isaac/right_joint_commands",
        "msg_class": "JointState",
        "label": "Sim R Arm Cmd",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/isaac/left_robotiq_joint_commands",
        "msg_class": "JointState",
        "label": "Sim L Gripper Cmd",
        "contract": {},
    },
    {
        "name": "/isaac/right_robotiq_joint_commands",
        "msg_class": "JointState",
        "label": "Sim R Gripper Cmd",
        "contract": {},
    },
    # ── Browser override topics (Republisher → Isaac) ─────────────────────
    {
        "name": "/isaac/browser/left_joint_commands",
        "msg_class": "JointState",
        "label": "Browser L Arm Cmd",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/isaac/browser/right_joint_commands",
        "msg_class": "JointState",
        "label": "Browser R Arm Cmd",
        "contract": {"joint_count": 7},
    },
    {
        "name": "/isaac/browser/left_robotiq_joint_commands",
        "msg_class": "JointState",
        "label": "Browser L Gripper Cmd",
        "contract": {},
    },
    {
        "name": "/isaac/browser/right_robotiq_joint_commands",
        "msg_class": "JointState",
        "label": "Browser R Gripper Cmd",
        "contract": {},
    },
]

# ---------------------------------------------------------------------------
# Topology columns — multi-column pipeline overview
# ---------------------------------------------------------------------------

TOPOLOGY_COLUMNS = [
    {
        "key": "isaac_sim",
        "label": "IsaacSim",
        "description": "Simulation physics — raw joint states, sensor data, and command inputs",
        "topics": [
            "/isaac/left_joint_states",
            "/isaac/right_joint_states",
            "/isaac/left_robotiq_joint_states",
            "/isaac/right_robotiq_joint_states",
            "/isaac/left_ee_wrench",
            "/isaac/right_ee_wrench",
            "/isaac/left_joint_commands",
            "/isaac/right_joint_commands",
            "/isaac/left_robotiq_joint_commands",
            "/isaac/right_robotiq_joint_commands",
            "/isaac/browser/left_joint_commands",
            "/isaac/browser/right_joint_commands",
            "/isaac/browser/left_robotiq_joint_commands",
            "/isaac/browser/right_robotiq_joint_commands",
        ],
    },
    {
        "key": "republisher",
        "label": "Republisher",
        "description": "Single interface between IsaacSim and all external components",
        "topics": [
            "/bridge/left_joint_states",
            "/bridge/right_joint_states",
            "/bridge/left_robotiq_joint_states",
            "/bridge/right_robotiq_joint_states",
            "/bridge/left_joint_commands",
            "/bridge/right_joint_commands",
            "/bridge/left_robotiq_joint_commands",
            "/bridge/right_robotiq_joint_commands",
            "/isaac/browser/left_joint_commands",
            "/isaac/browser/right_joint_commands",
            "/isaac/browser/left_robotiq_joint_commands",
            "/isaac/browser/right_robotiq_joint_commands",
        ],
    },
    {
        "key": "ui_control",
        "label": "UI Control",
        "description": "Browser controller — joint sliders and monitoring (port 8090)",
        "topics": [
            "/bridge/left_joint_states",
            "/bridge/right_joint_states",
            "/bridge/left_robotiq_joint_states",
            "/bridge/right_robotiq_joint_states",
            "/bridge/left_joint_commands",
            "/bridge/right_joint_commands",
            "/bridge/left_robotiq_joint_commands",
            "/bridge/right_robotiq_joint_commands",
            "/isaac/left_joint_states",
            "/isaac/right_joint_states",
            "/isaac/left_ee_wrench",
            "/isaac/right_ee_wrench",
            "/isaac/left_joint_commands",
            "/isaac/right_joint_commands",
            "/isaac/left_robotiq_joint_commands",
            "/isaac/right_robotiq_joint_commands",
        ],
    },
    {
        "key": "real_to_sim_bridge",
        "label": "Real Robot Bridge",
        "description": "Cross-RMW bridge — real robot (domain 53) → bridge commands (domain 0)",
        "topics": [
            "/bridge/left_joint_commands",
            "/bridge/right_joint_commands",
            "/bridge/left_robotiq_joint_commands",
            "/bridge/right_robotiq_joint_commands",
        ],
    },
]


def _load_runtime_defaults():
    if load_stack_config is None or get_config_sections is None:
        return {}
    try:
        return get_config_sections(load_stack_config(), ("bridge", "controller"))
    except Exception:
        return {}


STACK_RUNTIME_DEFAULTS = _load_runtime_defaults()
DEFAULT_CONTROLLER_NAME = str(
    STACK_RUNTIME_DEFAULTS.get(
        "controller_name",
        "position_passthrough",
    )
)
DEFAULT_CONTROLLER_MANAGER = str(
    STACK_RUNTIME_DEFAULTS.get("controller_manager", "/isaac_controller_manager")
)
DEFAULT_CONTROLLER_ACTIVITY_TOPIC = str(
    STACK_RUNTIME_DEFAULTS.get(
        "controller_activity_topic",
        f"{DEFAULT_CONTROLLER_MANAGER}/activity",
    )
)
NON_PRIMARY_CONTROLLER_NODE_NAMES = {
    "isaac_controller_manager",
    "joint_state_broadcaster",
    "left_robotiq_gripper_controller",
    "right_robotiq_gripper_controller",
    "robot_state_publisher",
}
IMAGE_TOPIC_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
RAW_IMAGE_PREVIEW_ENCODINGS = {"rgb8", "rgba8", "bgr8", "bgra8", "mono8"}
DEFAULT_CAMERA_FEEDS = [
    {
        "key": "left_wrist_camera",
        "label": "Left Wrist Camera",
        "aliases": (
            "left_wrist_camera",
            "wrist_left",
            "left_wrist",
            "left_d405",
        ),
        "topic_name": "/isaac/left_wrist_camera/image_raw",
        "camera_info_topic": "/isaac/left_wrist_camera/camera_info",
        "contract_video_key": "wrist_left",
        "contract_dtype": "uint8",
        "contract_layout": "HWC",
        "contract_color_space": "RGB",
    },
    {
        "key": "right_wrist_camera",
        "label": "Right Wrist Camera",
        "aliases": (
            "right_wrist_camera",
            "wrist_right",
            "right_wrist",
            "right_d405",
        ),
        "topic_name": "/isaac/right_wrist_camera/image_raw",
        "camera_info_topic": "/isaac/right_wrist_camera/camera_info",
        "contract_video_key": "wrist_right",
        "contract_dtype": "uint8",
        "contract_layout": "HWC",
        "contract_color_space": "RGB",
    },
    {
        "key": "head_camera",
        "label": "Head Camera",
        "aliases": (
            "head_camera",
            "head",
            "zed_mini",
            "zed",
        ),
        "topic_name": "/isaac/head_camera/image_raw",
        "camera_info_topic": "/isaac/head_camera/camera_info",
        "contract_video_key": "head",
        "contract_dtype": "uint8",
        "contract_layout": "HWC",
        "contract_color_space": "RGB",
    },
]


def _stack_camera_config_path():
    if load_stack_config is None or get_config_sections is None:
        return None
    try:
        config = load_stack_config()
        merged = get_config_sections(config, ("cameras",))
    except Exception:
        return None
    camera_config = merged.get("camera_config")
    return str(camera_config) if camera_config else None


def _load_camera_feeds():
    if load_camera_feed_configs is not None:
        try:
            configured = load_camera_feed_configs(_stack_camera_config_path())
            if configured:
                return configured
        except Exception:
            pass
    return [dict(feed) for feed in DEFAULT_CAMERA_FEEDS]


CAMERA_FEEDS = _load_camera_feeds()
CAMERA_FEEDS_BY_KEY = {feed["key"]: feed for feed in CAMERA_FEEDS}

ISAAC_SOURCE_NODE_NAMES = {"isaac_sim_joint_bridge"}

BRIDGE_SOURCE_NODE_NAMES = {
    "browser_joint_controller",
    "isaac_joint_republisher",
}

SOURCE_CONFIG = {
    "isaac_sim": {
        "topic_prefixes": ("/isaac/",),
        "node_names": ISAAC_SOURCE_NODE_NAMES,
        "node_prefixes": (),
        "topic_names": set(),
    },
    "bridge": {
        "topic_prefixes": ("/bridge/",),
        "node_names": BRIDGE_SOURCE_NODE_NAMES,
        "node_prefixes": (),
        "topic_names": set(),
    },
}


class BrowserControllerBridge:
    """Thread-safe ROS2 bridge for browser requests."""

    def __init__(
        self,
        ros_node: Node,
        publish_rate_hz: float,
        activity_window_s: float = DEFAULT_ACTIVITY_WINDOW_S,
    ):
        self._node = ros_node
        self._lock = threading.Lock()
        self._pending_updates = deque()
        self._dirty_topics = set()
        self._graph_snapshot_cache = {
            "topic_overview": {"isaac_sim": [], "bridge": []},
            "all_topics": [],
            "visible_nodes": [],
        }
        self._graph_snapshot_next_refresh = 0.0
        self._graph_snapshot_period = 1.0
        self._publishers = {}
        self._subscriptions = []
        self._groups = {}
        self._camera_enabled = {feed["key"]: False for feed in CAMERA_FEEDS}
        self._camera_topic_subscriptions = {}
        self._camera_frames = {feed["key"]: None for feed in CAMERA_FEEDS}
        self._publish_period = max(1.0 / publish_rate_hz, 0.001)
        self._next_publish_at = time.monotonic()
        self._activity_window_s = max(float(activity_window_s), 0.05)
        self._control_mode = DEFAULT_CONTROL_MODE

        for group in JOINT_GROUPS:
            command_topic = group["command_topic"]
            state_topic = group["state_topic"]
            mode = str(group.get("mode", "joint_passthrough"))
            display_joint = str(group.get("display_joint", ""))
            driver_joint = str(group.get("driver_joint", ""))
            gripper_open_position = float(group.get("gripper_open_position", 0.0))
            gripper_closed_position = float(group.get("gripper_closed_position", 0.8))
            if gripper_closed_position <= gripper_open_position:
                gripper_closed_position = gripper_open_position + 1.0
            self._groups[command_topic] = {
                "label": group["label"],
                "state_topic": state_topic,
                "command_topic": command_topic,
                "default_joints": list(group["default_joints"]),
                "mode": mode,
                "display_joint": display_joint,
                "driver_joint": driver_joint,
                "gripper_open_position": gripper_open_position,
                "gripper_closed_position": gripper_closed_position,
                "min": float(group.get("min", -3.14)),
                "max": float(group.get("max", 3.14)),
                "names": [],
                "positions": [],
                "targets": {},
                "initial_targets": {},
                "last_update": 0.0,
                "last_command_publish": 0.0,
                "manual_override_active": False,
            }
            self._publishers[command_topic] = self._node.create_publisher(JointState, command_topic, 10)
            self._subscriptions.append(
                self._node.create_subscription(
                    JointState,
                    state_topic,
                    lambda msg, topic=command_topic: self._on_joint_state(msg, topic),
                    10,
                )
            )

        # ── Process-monitor subscriptions ─────────────────────────────────
        self._monitor_trackers: dict[str, _HzTracker] = {}
        self._monitor_violations: dict[str, list] = {}
        self._monitor_last_wall: dict[str, float] = {}
        self._monitor_subscriptions: list = []
        for topic_cfg in MONITORED_TOPICS:
            tname = topic_cfg["name"]
            self._monitor_trackers[tname] = _HzTracker(window_s=2.0)
            self._monitor_violations[tname] = []
            msg_class_name = topic_cfg.get("msg_class", "JointState")
            if msg_class_name == "WrenchStamped":
                if WrenchStamped is None:
                    continue  # geometry_msgs not available
                msg_type = WrenchStamped
            else:
                msg_type = JointState
            self._monitor_subscriptions.append(
                self._node.create_subscription(
                    msg_type,
                    tname,
                    lambda msg, cfg=topic_cfg: self._on_monitor_msg(msg, cfg),
                    10,
                )
            )

    @staticmethod
    def _extract_joint_position(names, positions, desired_name):
        by_name = {}
        for index, name in enumerate(names):
            if index >= len(positions):
                break
            by_name[name] = positions[index]

        if desired_name in by_name:
            return by_name[desired_name]

        desired_tail = desired_name.split("_", 1)[-1] if "_" in desired_name else desired_name
        for name, value in by_name.items():
            if name.endswith(desired_name) or name.endswith(desired_tail):
                return value
        return None

    @staticmethod
    def _canonical_joint_name(name):
        joint_name = str(name)
        for side_prefix in ("left_", "right_"):
            if joint_name.startswith(side_prefix):
                joint_name = joint_name[len(side_prefix):]
                break
        for robot_prefix in ("fr3v2_", "fr3_"):
            if joint_name.startswith(robot_prefix):
                joint_name = joint_name[len(robot_prefix):]
                break
        return joint_name

    def _joint_limits_for_name(self, group, joint_name):
        canonical_joint = self._canonical_joint_name(joint_name)
        if canonical_joint in FR3_JOINT_LIMITS:
            lower, upper = FR3_JOINT_LIMITS[canonical_joint]
            return float(lower), float(upper)

        lower = float(group["min"])
        upper = float(group["max"])
        if lower > upper:
            lower, upper = upper, lower
        return lower, upper

    @staticmethod
    def _full_node_name(node_name, node_namespace):
        namespace = str(node_namespace or "/")
        if not namespace.startswith("/"):
            namespace = "/" + namespace
        if namespace == "/":
            return "/" + str(node_name)
        if namespace.endswith("/"):
            return namespace + str(node_name)
        return namespace + "/" + str(node_name)

    @staticmethod
    def _matches_any_prefix(value, prefixes):
        for prefix in prefixes:
            if value.startswith(prefix):
                return True
        return False

    @staticmethod
    def _topic_has_prefix(topic_name, prefixes):
        for prefix in prefixes:
            if topic_name.startswith(prefix):
                return True
        return False

    def _topic_matches_source(
        self,
        topic_name,
        publisher_node_names,
        subscriber_node_names,
        source_name,
    ):
        config = SOURCE_CONFIG[source_name]
        if topic_name in config["topic_names"]:
            return True
        if self._topic_has_prefix(topic_name, config["topic_prefixes"]):
            return True
        observed_nodes = publisher_node_names.union(subscriber_node_names)
        if any(node_name in config["node_names"] for node_name in observed_nodes):
            return True
        return any(
            self._matches_any_prefix(node_name, config["node_prefixes"])
            for node_name in observed_nodes
        )

    def _collect_graph_snapshot(self):
        topic_entries = []
        visible_nodes = set()
        for topic_name, topic_types in self._node.get_topic_names_and_types():
            try:
                publisher_infos = self._node.get_publishers_info_by_topic(topic_name)
            except Exception:
                publisher_infos = []
            try:
                subscriber_infos = self._node.get_subscriptions_info_by_topic(topic_name)
            except Exception:
                subscriber_infos = []

            publisher_node_names = sorted(
                {
                    str(info.node_name)
                    for info in publisher_infos
                    if getattr(info, "node_name", None)
                }
            )
            subscriber_node_names = sorted(
                {
                    str(info.node_name)
                    for info in subscriber_infos
                    if getattr(info, "node_name", None)
                }
            )
            publisher_nodes = sorted(
                {
                    self._full_node_name(
                        getattr(info, "node_name", ""),
                        getattr(info, "node_namespace", "/"),
                    )
                    for info in publisher_infos
                    if getattr(info, "node_name", None)
                }
            )
            subscriber_nodes = sorted(
                {
                    self._full_node_name(
                        getattr(info, "node_name", ""),
                        getattr(info, "node_namespace", "/"),
                    )
                    for info in subscriber_infos
                    if getattr(info, "node_name", None)
                }
            )

            entry = {
                "name": str(topic_name),
                "types": sorted(str(topic_type) for topic_type in topic_types),
                "publishers": publisher_nodes,
                "subscribers": subscriber_nodes,
                "publisher_count": len(publisher_nodes),
                "subscriber_count": len(subscriber_nodes),
            }
            visible_nodes.update(publisher_nodes)
            visible_nodes.update(subscriber_nodes)
            topic_entries.append(
                (
                    entry,
                    set(publisher_node_names),
                    set(subscriber_node_names),
                )
            )

        isaac_topics = []
        bridge_topics = []

        for entry, publisher_node_names, subscriber_node_names in topic_entries:
            topic_name = entry["name"]
            has_isaac_source = self._topic_matches_source(
                topic_name,
                publisher_node_names,
                subscriber_node_names,
                "isaac_sim",
            )
            has_bridge_source = self._topic_matches_source(
                topic_name,
                publisher_node_names,
                subscriber_node_names,
                "bridge",
            )

            if has_isaac_source:
                isaac_topics.append(entry)
            if has_bridge_source:
                bridge_topics.append(entry)

        isaac_topics.sort(key=lambda item: item["name"])
        bridge_topics.sort(key=lambda item: item["name"])
        all_topics = [entry for entry, _, _ in topic_entries]
        all_topics.sort(key=lambda item: item["name"])
        return {
            "topic_overview": {"isaac_sim": isaac_topics, "bridge": bridge_topics},
            "all_topics": all_topics,
            "visible_nodes": sorted(visible_nodes),
        }

    def _get_graph_snapshot(self):
        now = time.monotonic()
        with self._lock:
            if now < self._graph_snapshot_next_refresh:
                return self._graph_snapshot_cache

        graph_snapshot = self._collect_graph_snapshot()
        with self._lock:
            self._graph_snapshot_cache = graph_snapshot
            self._graph_snapshot_next_refresh = now + self._graph_snapshot_period
            return self._graph_snapshot_cache

    def _get_topic_overview(self):
        return self._get_graph_snapshot()["topic_overview"]

    @staticmethod
    def _topic_aliases(topic_name):
        normalized = str(topic_name or "")
        if not normalized:
            return []
        aliases = {normalized}
        if normalized.startswith("/bridge/"):
            aliases.add("/isaac/" + normalized[len("/bridge/"):])
        return sorted(aliases)

    @staticmethod
    def _record_topic_activity(topic_activity, topic_name, is_active, last_activity):
        normalized = str(topic_name or "")
        if not normalized:
            return
        existing = topic_activity.get(normalized, {"is_active": False, "last_activity": 0.0})
        existing["is_active"] = bool(existing.get("is_active", False) or is_active)
        existing["last_activity"] = max(float(existing.get("last_activity", 0.0)), float(last_activity))
        topic_activity[normalized] = existing

    @staticmethod
    def _status_label(status, inactive_label="NO DATA"):
        if status == "active":
            return "ACTIVE"
        if status == "idle":
            return "IDLE"
        if status == "connected":
            return "CONNECTED"
        if status == "ready":
            return "READY"
        if status == "publishing":
            return "PUBLISHING"
        if status == "subscribed":
            return "SUBSCRIBED"
        return inactive_label

    @classmethod
    def _topic_status(cls, entry, activity):
        is_active = bool(activity.get("is_active", False))
        last_activity = float(activity.get("last_activity", 0.0))
        publisher_count = int(entry.get("publisher_count", 0))
        subscriber_count = int(entry.get("subscriber_count", 0))

        if is_active:
            return "active", cls._status_label("active"), ""
        if last_activity > 0.0:
            return "idle", cls._status_label("idle"), ""
        if publisher_count > 0 and subscriber_count > 0:
            return "connected", cls._status_label("connected"), "Topic is connected on the ROS graph."
        if publisher_count > 0:
            return "publishing", cls._status_label("publishing"), "Publishers are visible on the ROS graph."
        if subscriber_count > 0:
            return "subscribed", cls._status_label("subscribed"), "Subscribers are visible on the ROS graph."
        return "inactive", cls._status_label("inactive"), "No publishers or subscribers discovered yet."

    def get_control_mode(self):
        with self._lock:
            mode_key = self._control_mode
        config = CONTROL_MODES.get(mode_key, CONTROL_MODES[DEFAULT_CONTROL_MODE])
        return {
            "mode": mode_key,
            "label": config["label"],
            "description": config["description"],
            "publish_commands": config["publish_commands"],
            "available_modes": [
                {"mode": k, "label": v["label"], "description": v["description"]}
                for k, v in CONTROL_MODES.items()
            ],
        }

    def set_control_mode(self, mode):
        if mode not in CONTROL_MODES:
            return False, f"Unknown control mode: {mode}. Valid: {', '.join(CONTROL_MODES)}"
        with self._lock:
            old_mode = self._control_mode
            self._control_mode = mode
        config = CONTROL_MODES[mode]
        return True, f"Switched from {CONTROL_MODES[old_mode]['label']} to {config['label']}"

    def get_topic_overview(self):
        topic_overview = self._get_topic_overview()
        now_wall = time.time()
        with self._lock:
            topic_activity = {}
            for group in self._groups.values():
                state_topic = str(group["state_topic"])
                command_topic = str(group["command_topic"])
                last_state = float(group["last_update"])
                last_command = float(group.get("last_command_publish", 0.0))
                for topic_alias in self._topic_aliases(state_topic):
                    self._record_topic_activity(
                        topic_activity,
                        topic_alias,
                        self._is_recent_activity(last_state, now_wall),
                        last_state,
                    )
                for topic_alias in self._topic_aliases(command_topic):
                    self._record_topic_activity(
                        topic_activity,
                        topic_alias,
                        self._is_recent_activity(last_command, now_wall),
                        last_command,
                    )
        return self._decorate_topic_overview(topic_overview, topic_activity)

    # ── Process monitor ───────────────────────────────────────────────────

    @staticmethod
    def _check_joint_state_contract(msg, contract: dict) -> list:
        violations = []
        expected = contract.get("joint_count")
        if expected and len(msg.name) != expected:
            violations.append(f"{len(msg.name)} joints (expected {expected})")
        if contract.get("require_effort") and len(msg.effort) < len(msg.name):
            violations.append(f"effort field missing ({len(msg.effort)}/{len(msg.name)})")
        return violations

    def _on_monitor_msg(self, msg, topic_cfg: dict) -> None:
        tname = topic_cfg["name"]
        msg_class = topic_cfg.get("msg_class", "JointState")
        contract = topic_cfg.get("contract", {})
        if msg_class == "JointState":
            violations = self._check_joint_state_contract(msg, contract)
        else:
            violations = []  # WrenchStamped is always structurally valid
        tracker = self._monitor_trackers.get(tname)
        if tracker is not None:
            tracker.record()
        with self._lock:
            self._monitor_violations[tname] = violations

    def get_monitor_snapshot(self) -> list:
        now_wall = time.time()
        graph = self._get_graph_snapshot()
        topic_map = {entry["name"]: entry for entry in graph.get("all_topics", [])}
        result = []
        for topic_cfg in MONITORED_TOPICS:
            tname = topic_cfg["name"]
            tracker = self._monitor_trackers.get(tname)
            hz = tracker.hz() if tracker else 0.0
            last_w = tracker.last_wall() if tracker else 0.0
            with self._lock:
                violations = list(self._monitor_violations.get(tname, []))
            age = (now_wall - last_w) if last_w > 0.0 else None
            online = age is not None and age < 2.0
            graph_entry = topic_map.get(tname, {})
            pub_count = int(graph_entry.get("publisher_count", 0))
            contract_ok = online and len(violations) == 0
            if online:
                status = "active"
                status_label = "ACTIVE"
            elif pub_count > 0:
                status = "idle"
                status_label = "IDLE"
            else:
                status = "inactive"
                status_label = "NO DATA"
            result.append({
                "name": tname,
                "label": topic_cfg.get("label", ""),
                "expected_type": f"sensor_msgs/msg/{topic_cfg.get('msg_class', 'JointState')}"
                    if topic_cfg.get("msg_class") != "WrenchStamped"
                    else "geometry_msgs/msg/WrenchStamped",
                "hz": round(hz, 1),
                "online": online,
                "last_seen_age_s": round(age, 2) if age is not None else None,
                "publisher_count": pub_count,
                "status": status,
                "status_label": status_label,
                "contract_ok": contract_ok,
                "violations": violations,
            })
        return result

    def get_topology_snapshot(self) -> dict:
        now_wall = time.time()
        graph = self._get_graph_snapshot()
        topic_map = {entry["name"]: entry for entry in graph.get("all_topics", [])}
        monitor_lookup = {cfg["name"]: cfg for cfg in MONITORED_TOPICS}
        columns = []
        for col_def in TOPOLOGY_COLUMNS:
            topics = []
            for tname in col_def["topics"]:
                tracker = self._monitor_trackers.get(tname)
                hz = tracker.hz() if tracker else 0.0
                last_w = tracker.last_wall() if tracker else 0.0
                age = (now_wall - last_w) if last_w > 0.0 else None
                online = age is not None and age < 2.0
                graph_entry = topic_map.get(tname, {})
                pub_count = int(graph_entry.get("publisher_count", 0))
                sub_count = int(graph_entry.get("subscriber_count", 0))
                publishers = list(graph_entry.get("publishers", []))
                if online:
                    status = "active"
                    status_label = "ACTIVE"
                elif pub_count > 0:
                    status = "idle"
                    status_label = "IDLE"
                else:
                    status = "inactive"
                    status_label = "NO DATA"
                label = monitor_lookup.get(tname, {}).get("label", "")
                topics.append({
                    "name": tname,
                    "label": label,
                    "hz": round(hz, 1),
                    "online": online,
                    "status": status,
                    "status_label": status_label,
                    "publisher_count": pub_count,
                    "subscriber_count": sub_count,
                    "publishers": publishers,
                })
            columns.append({
                "key": col_def["key"],
                "label": col_def["label"],
                "description": col_def["description"],
                "topics": topics,
            })
        return {"columns": columns, "timestamp": now_wall}

    @staticmethod
    def _clamp(value, lower, upper):
        return min(max(float(value), float(lower)), float(upper))

    def _normalize_opening(self, group, driver_position):
        stroke = float(group["gripper_closed_position"]) - float(group["gripper_open_position"])
        if stroke <= 0.0:
            return 0.0
        normalized_close = (float(driver_position) - float(group["gripper_open_position"])) / stroke
        normalized_close = min(max(normalized_close, 0.0), 1.0)
        return 1.0 - normalized_close

    def _reduce_gripper_state(self, group, names, positions):
        display_joint = group["display_joint"] or (group["default_joints"][0] if group["default_joints"] else "")
        if not display_joint:
            return list(names), list(positions)

        opening = self._extract_joint_position(names, positions, display_joint)
        if opening is None and group["driver_joint"]:
            driver_position = self._extract_joint_position(names, positions, group["driver_joint"])
            if driver_position is not None:
                opening = self._normalize_opening(group, driver_position)
        if opening is None:
            opening = positions[0] if positions else 0.0

        lower, upper = self._joint_limits_for_name(group, display_joint)
        return [display_joint], [self._clamp(opening, lower, upper)]

    def _effective_joint_names(self, group, allow_defaults=False):
        names = list(group["names"])
        if not names and allow_defaults:
            names = list(group["default_joints"])
        if group["mode"] == "gripper_opening":
            display_joint = group["display_joint"] or (names[0] if names else "")
            return [display_joint] if display_joint else []
        return names

    def _is_recent_activity(self, last_activity, now_wall):
        return float(last_activity) > 0.0 and (now_wall - float(last_activity)) <= self._activity_window_s

    @classmethod
    def _decorate_topic_overview(cls, topic_overview, topic_activity):
        decorated = {}
        for source_name, entries in topic_overview.items():
            decorated_entries = []
            for entry in entries:
                topic_name = str(entry.get("name", ""))
                activity = topic_activity.get(topic_name, {})
                status, status_label, status_detail = cls._topic_status(entry, activity)
                enriched = dict(entry)
                enriched["is_active"] = bool(activity.get("is_active", False))
                enriched["last_activity"] = float(activity.get("last_activity", 0.0))
                enriched["status"] = status
                enriched["status_label"] = status_label
                enriched["status_detail"] = status_detail
                decorated_entries.append(enriched)
            decorated[source_name] = decorated_entries
        return decorated

    def _on_joint_state(self, msg: JointState, command_topic: str):
        names = list(msg.name)
        if not names:
            return

        positions = list(msg.position)
        if len(positions) < len(names):
            positions.extend([0.0] * (len(names) - len(positions)))
        elif len(positions) > len(names):
            positions = positions[:len(names)]

        with self._lock:
            group = self._groups.get(command_topic)
            if group is None:
                return
            if group["mode"] == "gripper_opening":
                names, positions = self._reduce_gripper_state(group, names, positions)
            group["names"] = names
            group["positions"] = positions
            group["last_update"] = time.time()
            if group["mode"] == "gripper_opening":
                if names:
                    opening_name = names[0]
                    current_target = group["targets"].get(opening_name, positions[0])
                    lower, upper = self._joint_limits_for_name(group, opening_name)
                    group["targets"] = {opening_name: self._clamp(current_target, lower, upper)}
                else:
                    group["targets"] = {}
            for index, name in enumerate(names):
                lower, upper = self._joint_limits_for_name(group, name)
                if name not in group["initial_targets"]:
                    group["initial_targets"][name] = self._clamp(positions[index], lower, upper)
                if name not in group["targets"]:
                    group["targets"][name] = self._clamp(positions[index], lower, upper)

    @staticmethod
    def _node_basename(node_name):
        normalized = str(node_name or "")
        if not normalized:
            return ""
        return normalized.rsplit("/", 1)[-1]

    def _detect_current_controller_name(self, visible_nodes):
        for visible_node in visible_nodes:
            basename = self._node_basename(visible_node)
            if not basename:
                continue
            if basename == DEFAULT_CONTROLLER_NAME:
                return basename, True
            if basename in NON_PRIMARY_CONTROLLER_NODE_NAMES:
                continue
        return DEFAULT_CONTROLLER_NAME, False

    @staticmethod
    def _is_image_topic(entry):
        return any(topic_type in IMAGE_TOPIC_TYPES for topic_type in entry.get("types", []))

    @staticmethod
    def _camera_topic_score(feed_config, entry):
        topic_name = str(entry.get("name", "")).lower()
        if not topic_name:
            return None
        configured_topic = str(feed_config.get("topic_name", "")).lower()
        if configured_topic and topic_name == configured_topic:
            return 1000
        score = None
        for index, alias in enumerate(feed_config.get("aliases", ())):
            if alias in topic_name:
                score = max(score or 0, 100 - index)
        if score is None:
            return None
        if topic_name.endswith("/image_raw"):
            score += 5
        elif topic_name.endswith("/compressed"):
            score += 3
        elif "/image" in topic_name:
            score += 2
        return score

    def _find_camera_topic(self, all_topics, feed_config):
        best_entry = None
        best_score = None
        for entry in all_topics:
            if not self._is_image_topic(entry):
                continue
            score = self._camera_topic_score(feed_config, entry)
            if score is None:
                continue
            if best_score is None or score > best_score:
                best_entry = entry
                best_score = score
        return best_entry

    @staticmethod
    def _image_topic_message_type(entry):
        topic_types = set(entry.get("types", []))
        if "sensor_msgs/msg/Image" in topic_types:
            return "sensor_msgs/msg/Image"
        if "sensor_msgs/msg/CompressedImage" in topic_types:
            return "sensor_msgs/msg/CompressedImage"
        return ""

    @staticmethod
    def _compressed_image_content_type(image_format):
        normalized = str(image_format or "").lower()
        if "png" in normalized:
            return "image/png"
        return "image/jpeg"

    @staticmethod
    def _raw_row_to_bgr(row_bytes, encoding):
        bgr_row = bytearray()
        if encoding == "rgb8":
            for index in range(0, len(row_bytes), 3):
                bgr_row.extend((row_bytes[index + 2], row_bytes[index + 1], row_bytes[index]))
            return bgr_row
        if encoding == "bgr8":
            return bytearray(row_bytes)
        if encoding == "rgba8":
            for index in range(0, len(row_bytes), 4):
                bgr_row.extend((row_bytes[index + 2], row_bytes[index + 1], row_bytes[index]))
            return bgr_row
        if encoding == "bgra8":
            for index in range(0, len(row_bytes), 4):
                bgr_row.extend((row_bytes[index], row_bytes[index + 1], row_bytes[index + 2]))
            return bgr_row
        if encoding == "mono8":
            for value in row_bytes:
                bgr_row.extend((value, value, value))
            return bgr_row
        return None

    @classmethod
    def _encode_bmp_preview(cls, frame_state):
        width = int(frame_state.get("width", 0))
        height = int(frame_state.get("height", 0))
        encoding = str(frame_state.get("encoding", "")).lower()
        step = int(frame_state.get("step", 0))
        data = bytes(frame_state.get("data", b""))
        if width <= 0 or height <= 0 or not data or encoding not in RAW_IMAGE_PREVIEW_ENCODINGS:
            return None

        channels = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }[encoding]
        row_width = width * channels
        row_stride = step if step >= row_width else row_width
        padding = (4 - ((width * 3) % 4)) % 4
        image_size = (width * 3 + padding) * height
        file_size = 54 + image_size

        header = b"BM" + struct.pack("<IHHI", file_size, 0, 0, 54)
        dib = struct.pack(
            "<IIIHHIIIIII",
            40,
            width,
            height,
            1,
            24,
            0,
            image_size,
            2835,
            2835,
            0,
            0,
        )
        body = bytearray()
        for row_index in range(height - 1, -1, -1):
            offset = row_index * row_stride
            row_bytes = data[offset:offset + row_stride]
            row_pixels = row_bytes[:row_width]
            if len(row_pixels) < row_width:
                return None
            converted = cls._raw_row_to_bgr(row_pixels, encoding)
            if converted is None:
                return None
            body.extend(converted)
            if padding:
                body.extend(b"\x00" * padding)
        return header + dib + bytes(body)

    def _store_camera_frame(self, camera_key, topic_name, topic_type, msg):
        feed_config = CAMERA_FEEDS_BY_KEY.get(camera_key, {})
        contract_mode = "unsupported"
        payload = {
            "topic_name": str(topic_name),
            "topic_type": str(topic_type),
            "last_update": time.time(),
            "frame_id": "",
            "preview_available": False,
            "contract_compliant": False,
            "contract_video_key": str(feed_config.get("contract_video_key", "")),
        }
        header = getattr(msg, "header", None)
        payload["frame_id"] = str(getattr(header, "frame_id", "") or "")

        if topic_type == "sensor_msgs/msg/Image":
            encoding = str(getattr(msg, "encoding", "") or "").lower()
            width = int(getattr(msg, "width", 0) or 0)
            height = int(getattr(msg, "height", 0) or 0)
            step = int(getattr(msg, "step", 0) or 0)
            data = bytes(getattr(msg, "data", b"") or b"")
            if encoding == "rgb8":
                contract_mode = "direct"
            elif encoding in RAW_IMAGE_PREVIEW_ENCODINGS:
                contract_mode = "lossless_conversion"
            payload.update(
                {
                    "encoding": encoding,
                    "width": width,
                    "height": height,
                    "step": step,
                    "data": data,
                    "preview_available": bool(width > 0 and height > 0 and encoding in RAW_IMAGE_PREVIEW_ENCODINGS and data),
                    "contract_compliant": contract_mode in {"direct", "lossless_conversion"},
                    "contract_mode": contract_mode,
                }
            )
        elif topic_type == "sensor_msgs/msg/CompressedImage":
            image_format = str(getattr(msg, "format", "") or "")
            data = bytes(getattr(msg, "data", b"") or b"")
            payload.update(
                {
                    "encoding": image_format.lower(),
                    "data": data,
                    "content_type": self._compressed_image_content_type(image_format),
                    "preview_available": bool(data),
                    "contract_compliant": False,
                    "contract_mode": "compressed_transport",
                }
            )

        with self._lock:
            previous = self._camera_frames.get(camera_key) or {}
            payload["sequence"] = int(previous.get("sequence", 0)) + 1
            self._camera_frames[camera_key] = payload

    def _camera_callback(self, msg, *, camera_key, topic_name, topic_type):
        self._store_camera_frame(camera_key, topic_name, topic_type, msg)

    def _destroy_camera_subscription(self, camera_key):
        with self._lock:
            active = self._camera_topic_subscriptions.pop(camera_key, None)
        if not active:
            return
        subscription = active.get("subscription")
        if subscription is None:
            return
        try:
            self._node.destroy_subscription(subscription)
        except Exception:
            pass

    def _sync_camera_subscriptions(self, graph_snapshot):
        all_topics = list(graph_snapshot.get("all_topics", []))
        desired = {}
        with self._lock:
            enabled_state = dict(self._camera_enabled)
            active_subscriptions = {
                key: dict(value) for key, value in self._camera_topic_subscriptions.items()
            }

        for feed_config in CAMERA_FEEDS:
            key = str(feed_config["key"])
            if not enabled_state.get(key, False):
                continue
            matched_topic = self._find_camera_topic(all_topics, feed_config)
            if matched_topic is None:
                continue
            topic_name = str(matched_topic.get("name", "") or "")
            topic_type = self._image_topic_message_type(matched_topic)
            if not topic_name or not topic_type:
                continue
            desired[key] = {"topic_name": topic_name, "topic_type": topic_type}

        for camera_key, active in active_subscriptions.items():
            desired_state = desired.get(camera_key)
            if desired_state is None or (
                active.get("topic_name") != desired_state["topic_name"]
                or active.get("topic_type") != desired_state["topic_type"]
            ):
                self._destroy_camera_subscription(camera_key)

        for feed_config in CAMERA_FEEDS:
            camera_key = str(feed_config["key"])
            desired_state = desired.get(camera_key)
            if desired_state is None:
                continue
            active = active_subscriptions.get(camera_key)
            if active is not None and (
                active.get("topic_name") == desired_state["topic_name"]
                and active.get("topic_type") == desired_state["topic_type"]
            ):
                continue

            topic_type = desired_state["topic_type"]
            message_type = Image if topic_type == "sensor_msgs/msg/Image" else CompressedImage
            subscription = self._node.create_subscription(
                message_type,
                desired_state["topic_name"],
                lambda msg, camera_key=camera_key, topic_name=desired_state["topic_name"], topic_type=topic_type: self._camera_callback(
                    msg,
                    camera_key=camera_key,
                    topic_name=topic_name,
                    topic_type=topic_type,
                ),
                1,
            )
            with self._lock:
                self._camera_topic_subscriptions[camera_key] = {
                    "topic_name": desired_state["topic_name"],
                    "topic_type": topic_type,
                    "subscription": subscription,
                }

    def get_camera_preview(self, camera_key: str):
        normalized_key = str(camera_key or "").strip()
        if normalized_key not in CAMERA_FEEDS_BY_KEY:
            return None
        with self._lock:
            frame_state = dict(self._camera_frames.get(normalized_key) or {})
        if not frame_state:
            return None

        topic_type = frame_state.get("topic_type")
        if topic_type == "sensor_msgs/msg/CompressedImage":
            content_type = str(frame_state.get("content_type", "image/jpeg"))
            data = bytes(frame_state.get("data", b"") or b"")
            if not data:
                return None
            return {"content_type": content_type, "data": data}

        bmp = self._encode_bmp_preview(frame_state)
        if bmp is None:
            return None
        return {"content_type": "image/bmp", "data": bmp}

    def _build_camera_overview(self, graph_snapshot, topic_activity):
        with self._lock:
            enabled_state = dict(self._camera_enabled)
            camera_frames = {
                key: (dict(value) if isinstance(value, dict) else None)
                for key, value in self._camera_frames.items()
            }

        cameras = []
        all_topics = list(graph_snapshot.get("all_topics", []))
        for feed_config in CAMERA_FEEDS:
            key = str(feed_config["key"])
            enabled = bool(enabled_state.get(key, False))
            matched_topic = self._find_camera_topic(all_topics, feed_config)
            frame_state = camera_frames.get(key) or {}
            topic_name = str(frame_state.get("topic_name", "")) or (
                str(matched_topic.get("name", "")) if matched_topic else ""
            )
            preview_available = (
                enabled
                and matched_topic is not None
                and bool(frame_state.get("preview_available", False))
            )
            frame_sequence = int(frame_state.get("sequence", 0) or 0)
            width = int(frame_state.get("width", feed_config.get("width", 0)) or 0)
            height = int(frame_state.get("height", feed_config.get("height", 0)) or 0)
            encoding = str(frame_state.get("encoding", "") or "")
            contract_compliant = bool(frame_state.get("contract_compliant", False))
            contract_mode = str(frame_state.get("contract_mode", "") or "")
            frame_age = float(frame_state.get("last_update", 0.0) or 0.0)

            if matched_topic is not None:
                topic_status, _, _ = self._topic_status(
                    matched_topic,
                    topic_activity.get(topic_name, {}),
                )
                if enabled and preview_available:
                    status = "active" if self._is_recent_activity(frame_age, time.time()) else "connected"
                    status_label = self._status_label(status)
                    detail = (
                        f"Feed enabled. Live preview attached to {topic_name}. "
                        f"Contract mapping: {feed_config.get('contract_video_key', 'unmapped')}."
                    )
                elif enabled:
                    status = "connected" if topic_status in {"connected", "publishing", "active"} else "idle"
                    status_label = self._status_label(status)
                    detail = f"Feed enabled. Waiting for a frame on {topic_name}."
                else:
                    status = "ready"
                    status_label = self._status_label("ready")
                    detail = f"Topic discovered on {topic_name}. Activate to attach the preview stream."
            elif enabled:
                status = "idle"
                status_label = "WAITING"
                detail = "Feed enabled. Waiting for a ROS image topic from simulation."
            else:
                status = "inactive"
                status_label = "OFF"
                detail = "No ROS image topic discovered yet."

            cameras.append(
                {
                    "key": key,
                    "label": str(feed_config["label"]),
                    "enabled": enabled,
                    "topic_name": topic_name,
                    "status": status,
                    "status_label": status_label,
                    "detail": detail,
                    "contract_video_key": str(feed_config.get("contract_video_key", "")),
                    "preview_url": f"/api/cameras/frame?camera_key={key}" if preview_available else "",
                    "preview_available": preview_available,
                    "frame_sequence": frame_sequence,
                    "width": width,
                    "height": height,
                    "encoding": encoding,
                    "contract_compliant": contract_compliant,
                    "contract_mode": contract_mode,
                }
            )
        return cameras

    def _build_status_overview(self, graph_snapshot, groups):
        isaac_topics = list(graph_snapshot.get("topic_overview", {}).get("isaac_sim", []))
        all_topics = list(graph_snapshot.get("all_topics", []))
        all_topic_names = {str(entry.get("name", "")) for entry in all_topics}
        visible_nodes = set(graph_snapshot.get("visible_nodes", []))

        total_groups = len(groups)
        available_groups = sum(1 for group in groups if group["available"])
        live_groups = sum(1 for group in groups if group["is_active"])
        live_state_groups = sum(1 for group in groups if group["state_active"])
        live_command_groups = sum(1 for group in groups if group["command_active"])

        simulation_status = "active" if isaac_topics and live_state_groups > 0 else ("connected" if isaac_topics else "inactive")
        bridge_status = "active" if live_command_groups > 0 else ("connected" if available_groups > 0 else "inactive")
        manager_visible = (
            DEFAULT_CONTROLLER_MANAGER in visible_nodes
            or DEFAULT_CONTROLLER_ACTIVITY_TOPIC in all_topic_names
            or any(topic_name.startswith(f"{DEFAULT_CONTROLLER_MANAGER}/") for topic_name in all_topic_names)
        )
        controller_name, controller_observed = self._detect_current_controller_name(visible_nodes)
        controller_status = (
            "active"
            if controller_observed and live_command_groups > 0
            else ("connected" if controller_observed or live_command_groups > 0 else "inactive")
        )

        cards = [
            {
                "key": "simulation",
                "label": "Simulation",
                "status": simulation_status,
                "status_label": self._status_label(simulation_status),
                "summary": f"{len(isaac_topics)} Isaac topics discovered",
                "detail": (
                    f"{live_state_groups}/{total_groups} groups have recent state traffic. "
                    f"{live_groups}/{total_groups} groups show any recent motion or command traffic."
                ),
            },
            {
                "key": "bridge",
                "label": "Bridge Path",
                "status": bridge_status,
                "status_label": self._status_label(bridge_status),
                "summary": f"{available_groups}/{total_groups} groups ready",
                "detail": f"{live_command_groups}/{total_groups} groups have recent browser command publishes.",
            },
            {
                "key": "controller_manager",
                "label": "Controller Manager",
                "status": "connected" if manager_visible else "inactive",
                "status_label": self._status_label("connected" if manager_visible else "inactive"),
                "summary": DEFAULT_CONTROLLER_MANAGER,
                "detail": (
                    "Controller manager topics are visible on the ROS graph."
                    if manager_visible
                    else "Controller manager is not directly visible from the browser container."
                ),
            },
            {
                "key": "controller",
                "label": "Current Controller",
                "status": controller_status,
                "status_label": self._status_label(controller_status),
                "summary": controller_name,
                "detail": (
                    "Observed on the ROS graph."
                    if controller_observed
                    else "Using the configured default because controller state is not exposed here."
                ),
            },
        ]

        headline = (
            f"Simulation {self._status_label(simulation_status)} | "
            f"Groups ready: {available_groups}/{total_groups} | "
            f"Controller: {controller_name}"
        )
        return {"headline": headline, "cards": cards}

    def get_snapshot(self):
        with self._lock:
            groups = []
            topic_activity = {}
            now_wall = time.time()
            for group in self._groups.values():
                names = self._effective_joint_names(group, allow_defaults=False)
                positions = list(group["positions"])
                targets = dict(group["targets"])
                last_update = float(group["last_update"])
                last_command_publish = float(group.get("last_command_publish", 0.0))
                state_active = self._is_recent_activity(last_update, now_wall)
                command_active = self._is_recent_activity(last_command_publish, now_wall)
                last_activity = max(last_update, last_command_publish)
                joints = []
                for index, name in enumerate(names):
                    pos = positions[index] if index < len(positions) else 0.0
                    lower, upper = self._joint_limits_for_name(group, name)
                    joints.append(
                        {
                            "name": name,
                            "position": pos,
                            "target_position": self._clamp(float(targets.get(name, pos)), lower, upper),
                            "min": lower,
                            "max": upper,
                        }
                    )

                groups.append(
                    {
                        "label": group["label"],
                        "mode": group.get("mode", "joint_passthrough"),
                        "state_topic": group["state_topic"],
                        "command_topic": group["command_topic"],
                        "available": bool(names),
                        "joints": joints,
                        "last_update": last_update,
                        "last_command_publish": last_command_publish,
                        "state_active": state_active,
                        "command_active": command_active,
                        "is_active": state_active or command_active,
                        "last_activity": last_activity,
                    }
                )
                for topic_alias in self._topic_aliases(group["state_topic"]):
                    self._record_topic_activity(topic_activity, topic_alias, state_active, last_update)
                for topic_alias in self._topic_aliases(group["command_topic"]):
                    self._record_topic_activity(
                        topic_activity,
                        topic_alias,
                        command_active,
                        last_command_publish,
                    )
            camera_frames = {
                key: (dict(value) if isinstance(value, dict) else None)
                for key, value in self._camera_frames.items()
            }
            camera_enabled = dict(self._camera_enabled)
            for camera_key, frame_state in camera_frames.items():
                if not frame_state:
                    continue
                topic_name = str(frame_state.get("topic_name", "") or "")
                last_update = float(frame_state.get("last_update", 0.0) or 0.0)
                if topic_name:
                    self._record_topic_activity(
                        topic_activity,
                        topic_name,
                        camera_enabled.get(camera_key, False)
                        and self._is_recent_activity(last_update, now_wall),
                        last_update,
                    )
        graph_snapshot = self._get_graph_snapshot()
        self._sync_camera_subscriptions(graph_snapshot)
        topics = self._decorate_topic_overview(graph_snapshot["topic_overview"], topic_activity)
        cameras = self._build_camera_overview(graph_snapshot, topic_activity)
        status = self._build_status_overview(graph_snapshot, groups)
        return {
            "groups": groups,
            "topics": topics,
            "status": status,
            "cameras": cameras,
            "control_mode": self.get_control_mode(),
            "timestamp": now_wall,
        }

    def set_camera_enabled(self, camera_key: str, enabled):
        normalized_key = str(camera_key or "").strip()
        matching_feed = next((feed for feed in CAMERA_FEEDS if feed["key"] == normalized_key), None)
        if matching_feed is None:
            return False, f"Unknown camera feed: {camera_key}"

        with self._lock:
            self._camera_enabled[normalized_key] = bool(enabled)
        if not enabled:
            self._destroy_camera_subscription(normalized_key)
        action = "enabled" if enabled else "disabled"
        return True, f"{matching_feed['label']} {action}"

    def set_targets(self, command_topic: str, updates):
        if command_topic not in self._groups:
            return False, f"Unknown command topic: {command_topic}"
        if not isinstance(updates, dict):
            return False, "positions must be an object"

        sanitized = {}
        for joint_name, value in updates.items():
            if not isinstance(joint_name, str):
                continue
            try:
                sanitized[joint_name] = float(value)
            except (TypeError, ValueError):
                continue

        if not sanitized:
            return False, "No valid joint targets"

        with self._lock:
            group = self._groups.get(command_topic)
            if group is None:
                return False, f"Unknown command topic: {command_topic}"
            known_names = set(self._effective_joint_names(group, allow_defaults=True))
            if not known_names:
                return False, "Joint states not available yet for this group"
            clamped = {}
            for joint_name, target in sanitized.items():
                normalized_joint_name = joint_name
                if group["mode"] == "gripper_opening":
                    gripper_joint = group["display_joint"] or (
                        group["default_joints"][0] if group["default_joints"] else joint_name
                    )
                    normalized_joint_name = gripper_joint
                elif known_names and normalized_joint_name not in known_names:
                    continue
                lower, upper = self._joint_limits_for_name(group, normalized_joint_name)
                clamped[normalized_joint_name] = self._clamp(target, lower, upper)
            if not clamped:
                return False, "No valid joint targets for this group"
            self._pending_updates.append((command_topic, clamped))
        return True, "queued"

    def reset_targets(self, command_topic=None):
        with self._lock:
            if command_topic is not None:
                topics = [command_topic]
            else:
                topics = list(self._groups.keys())
            topics_set = set(topics)

            reset_count = 0
            for topic in topics:
                group = self._groups.get(topic)
                if group is None:
                    if command_topic is not None:
                        return False, f"Unknown command topic: {command_topic}"
                    continue

                names = self._effective_joint_names(group, allow_defaults=False)
                if not names:
                    continue
                positions = list(group["positions"])
                if len(positions) < len(names):
                    positions.extend([0.0] * (len(names) - len(positions)))
                elif len(positions) > len(names):
                    positions = positions[:len(names)]

                targets = {}
                for index, joint_name in enumerate(names):
                    lower, upper = self._joint_limits_for_name(group, joint_name)
                    if index < len(positions):
                        reset_value = positions[index]
                    else:
                        reset_value = group["targets"].get(
                            joint_name,
                            group["initial_targets"].get(joint_name, 0.0),
                        )
                    targets[joint_name] = self._clamp(reset_value, lower, upper)
                group["targets"] = targets
                group["manual_override_active"] = True
                reset_count += 1

            if self._pending_updates:
                retained_updates = deque()
                while self._pending_updates:
                    topic, updates = self._pending_updates.popleft()
                    if topic in topics_set:
                        continue
                    retained_updates.append((topic, updates))
                self._pending_updates = retained_updates
            self._dirty_topics.update(topics_set)
        if reset_count == 0:
            return True, "No active joint-state groups yet. Waiting for first state update."
        return True, f"Reset targets to current values for {reset_count} joint groups"

    def process(self):
        self._apply_pending_updates()

        with self._lock:
            mode_config = CONTROL_MODES.get(self._control_mode, CONTROL_MODES[DEFAULT_CONTROL_MODE])
            if not mode_config["publish_commands"]:
                self._dirty_topics.clear()
                return
            topics_to_publish = set(self._dirty_topics)
            for command_topic, group in self._groups.items():
                if group.get("manual_override_active", False):
                    topics_to_publish.add(command_topic)
        if not topics_to_publish:
            return

        now = time.monotonic()
        if now < self._next_publish_at:
            return
        self._next_publish_at = now + self._publish_period
        self._publish_target_commands(topics_to_publish=topics_to_publish)

    def _apply_pending_updates(self):
        while True:
            with self._lock:
                if not self._pending_updates:
                    return
                command_topic, updates = self._pending_updates.popleft()
                group = self._groups.get(command_topic)
                if group is None:
                    continue
                targets = group["targets"]
                for joint_name, position in updates.items():
                    lower, upper = self._joint_limits_for_name(group, joint_name)
                    targets[joint_name] = self._clamp(position, lower, upper)
                group["manual_override_active"] = True
                self._dirty_topics.add(command_topic)

    def _publish_target_commands(self, topics_to_publish=None):
        with self._lock:
            snapshots = []
            if topics_to_publish is None:
                topics_to_publish = set(self._dirty_topics)
            else:
                topics_to_publish = set(topics_to_publish)
            self._dirty_topics.difference_update(topics_to_publish)
            publish_wall_time = time.time()
            for topic in topics_to_publish:
                group = self._groups.get(topic)
                if group is None:
                    continue
                names = self._effective_joint_names(group, allow_defaults=True)
                if not names:
                    continue
                positions = list(group["positions"])
                targets = dict(group["targets"])
                group["last_command_publish"] = publish_wall_time
                snapshots.append((topic, group, names, positions, targets))

        for topic, group, names, positions, targets in snapshots:
            if not names:
                continue

            if len(positions) != len(names):
                positions = [0.0] * len(names)

            target_positions = []
            for index, name in enumerate(names):
                lower, upper = self._joint_limits_for_name(group, name)
                position = float(targets.get(name, positions[index]))
                target_positions.append(self._clamp(position, lower, upper))

            msg = JointState()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.name = names
            msg.position = target_positions
            self._publishers[topic].publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(description="Standalone ROS2 browser joint controller")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8090, help="HTTP bind port")
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=60.0,
        help="Command publish rate in Hz to simulate a dedicated controller loop",
    )
    parser.add_argument(
        "--activity-window",
        type=float,
        default=DEFAULT_ACTIVITY_WINDOW_S,
        help="Seconds a topic/group should remain ACTIVE after last observed traffic",
    )
    return parser.parse_args()


def main():
    from api.routes import SliderServer

    args = parse_args()

    rclpy.init(args=None)
    node = Node("browser_joint_controller")
    bridge = BrowserControllerBridge(
        node,
        publish_rate_hz=args.publish_rate,
        activity_window_s=args.activity_window,
    )
    server = SliderServer(args.host, args.port, bridge)
    server.start()

    node.get_logger().info(
        f"Browser controller started on http://localhost:{args.port} "
        f"(publish rate: {args.publish_rate:.1f} Hz, activity window: {args.activity_window:.2f}s)"
    )

    try:
        while rclpy.ok():
            bridge.process()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
