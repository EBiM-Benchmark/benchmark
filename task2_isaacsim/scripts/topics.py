# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Loader for the shared Task 2 ROS topic contract (config/topics.yaml).

Shared module in scripts/ (same convention as task1's
isaac_bridge_constants.py), import-safe anywhere: stdlib + PyYAML only, no
Isaac Sim or rclpy imports. It is used both inside the Isaac Sim container
(bridge and sim-side recording publishers) and inside the recorder
container (services/recording/record_task2.py) — the YAML path is resolved
relative to this file, so it works under both repo mounts.

Loading fails hard when the file is missing; a missing key surfaces as a
KeyError at the consumer's indexing site.
"""

from pathlib import Path

TOPICS_YAML = Path(__file__).resolve().parents[1] / "config" / "topics.yaml"

_topics_cache = None


def load_topics():
    """Load config/topics.yaml (cached per process)."""
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
        _topics_cache = yaml.safe_load(f) or {}
    return _topics_cache


def camera_topic(topics, namespace, kind):
    """Full camera topic name for a subtopic kind (image/camera_info/depth)."""
    return f"{namespace}/{topics['cameras']['subtopics'][kind]}"
