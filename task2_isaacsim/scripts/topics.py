# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Loader for the shared Task 2 ROS topic contract (config/topics.yaml).

Shared module in scripts/ (same convention as task1's
isaac_bridge_constants.py), import-safe anywhere: stdlib + PyYAML only, no
Isaac Sim or rclpy imports. It is used both inside the Isaac Sim container
(bridge and sim-side recording publishers) and inside the recorder
container (services/recording/record_task2.py) — the YAML path is resolved
relative to this file, so it works under both repo mounts.

Loading fails hard when the file or a required key is missing; falling back
to stale baked-in defaults is exactly the drift this contract exists to
prevent.
"""

from pathlib import Path

TOPICS_YAML = Path(__file__).resolve().parents[1] / "config" / "topics.yaml"

_REQUIRED_KEYS = (
    "clock",
    "teleop.pedal_state",
    "bridge.joint_groups.left_arm.state",
    "bridge.joint_groups.left_arm.command",
    "bridge.joint_groups.left_arm.browser_command",
    "bridge.joint_groups.right_arm.state",
    "bridge.joint_groups.right_arm.command",
    "bridge.joint_groups.right_arm.browser_command",
    "bridge.joint_groups.left_gripper.state",
    "bridge.joint_groups.left_gripper.command",
    "bridge.joint_groups.left_gripper.browser_command",
    "bridge.joint_groups.right_gripper.state",
    "bridge.joint_groups.right_gripper.command",
    "bridge.joint_groups.right_gripper.browser_command",
    "recording.applied_joint_commands",
    "recording.joint_states_full",
    "recording.odom",
    "recording.cmd_vel_applied",
    "recording.ee_pose.left",
    "recording.ee_pose.right",
    "ground_truth.object_poses",
    "ground_truth.pad_points",
    "ground_truth.scene_reset",
    "ground_truth.scene_reset_request",
    "cameras.subtopics.image",
    "cameras.subtopics.camera_info",
    "cameras.subtopics.depth",
    "cameras.robot.head.namespace",
    "cameras.robot.head.sensors_key",
    "cameras.robot.head.shape",
    "cameras.robot.wrist_left.namespace",
    "cameras.robot.wrist_left.sensors_key",
    "cameras.robot.wrist_left.shape",
    "cameras.robot.wrist_right.namespace",
    "cameras.robot.wrist_right.sensors_key",
    "cameras.robot.wrist_right.shape",
    "cameras.eval.namespace",
    "cameras.eval.shape",
    "cameras.eval.bbox_2d_tight",
    "cameras.eval.semantic_labels",
    "cameras.eval.semantic_segmentation",
)

_topics_cache = None


def _lookup(tree, dotted):
    node = tree
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return None
        node = node[part]
    return node


def load_topics():
    """Load and validate config/topics.yaml (cached per process)."""
    global _topics_cache
    if _topics_cache is not None:
        return _topics_cache

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required to load the Task 2 topic contract "
            f"({TOPICS_YAML})"
        ) from exc

    if not TOPICS_YAML.is_file():
        raise FileNotFoundError(
            f"Task 2 topic contract not found: {TOPICS_YAML}"
        )
    with open(TOPICS_YAML, encoding="utf-8") as f:
        topics = yaml.safe_load(f) or {}

    missing = [key for key in _REQUIRED_KEYS if _lookup(topics, key) is None]
    if missing:
        raise RuntimeError(
            f"Task 2 topic contract {TOPICS_YAML} is missing required "
            f"keys: {', '.join(missing)}"
        )

    _topics_cache = topics
    return topics


def camera_topic(topics, namespace, kind):
    """Full camera topic name for a subtopic kind (image/camera_info/depth)."""
    return f"{namespace}/{topics['cameras']['subtopics'][kind]}"
