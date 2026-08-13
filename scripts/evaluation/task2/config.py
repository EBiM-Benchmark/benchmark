#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Runtime configuration for the task2 eval camera service.

Defaults live in ``APP_DEFAULTS`` and ``config.yaml``; both can be
overridden on the command line. ``load_runtime_config`` merges (in
increasing priority): APP_DEFAULTS < config.yaml < CLI args.
"""

import argparse
import os
from pathlib import Path
from typing import Any

import yaml

APP_DEFAULTS: dict[str, Any] = {
    "evaluate_service_name": "/isaac/eval_camera/evaluate",
    "image_topic": "/isaac/eval_camera/image_raw",
    "depth_topic": "/isaac/eval_camera/depth",
    "semantic_segmentation_topic": "/isaac/eval_camera/semantic_segmentation",
    "semantic_labels_topic": "/isaac/eval_camera/semantic_labels",
    "bbox_2d_tight_topic": "/isaac/eval_camera/bbox_2d_tight",
    # Each annotator publishes its OWN id->label map with its own ID
    # scheme; see task2_isaacsim/config/topics.yaml. The target resolves
    # through the loose stream because tight bboxes drop fully occluded
    # objects and a correct placement occludes the target exactly.
    "bbox_tight_labels_topic": "/isaac/eval_camera/bbox_2d_tight_labels",
    "bbox_2d_loose_topic": "/isaac/eval_camera/bbox_2d_loose",
    "bbox_loose_labels_topic": "/isaac/eval_camera/bbox_2d_loose_labels",
    "camera_info_topic": "/isaac/eval_camera/camera_info",
    "thermalpad_label": "thermalpad",
    "liner_label": "liner",
    "target_label": "target",
    # Default output dir is the persistent volume mounted into the container.
    "output_dir": "/output",
    "jpeg_quality": 95,
    "bbox_json_top_per_class_only": False,
    # EvalStreamSync tuning (see stream_sync.py). Default tolerance is
    # ~one render period at 60 Hz sim time.
    "sync_tolerance_s": 0.0167,
    "sync_timeout_s": 5.0,
    "sync_max_age_s": 0.5,
    "sync_rebase_epsilon_s": 0.05,
    "sync_image_buffer_len": 12,
    "sync_buffer_len": 120,
    # Audit topics exist only when the Isaac Sim scene runs with
    # --record.
    "audit_enabled": True,
    "audit_object_poses_topic": "/isaac/task2/object_poses",
    "audit_pad_points_topic": "/isaac/task2/pad_points",
    "audit_max_skew_s": 0.5,
}

# FALLBACK raw int32 semantic-mask pixel value -> class name. Isaac Sim
# assigns the mask IDs per session (verified: they permute between runs),
# so the live semantic_labels payload is authoritative and this map is
# only used when that payload is unavailable. It only resolves the
# both-pads-visible case via pixel ratios; an incorrect mapping silently
# breaks the both_liner_dominant / both_thermalpad_dominant / sideways
# decision.
SEMANTIC_RAW_ID_NAME_HINTS: dict[int, str] = {
    1: "unlabeled",
    2: "board",
    3: "thermalpad",
    4: "target",
    5: "liner",
}


def coerce_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "on"}:
            return True
        if text in {"0", "false", "no", "off"}:
            return False
    raise ValueError(f"Cannot parse boolean value: {value}")


def _looks_like_git_sha(text: str) -> bool:
    return len(text) == 40 and all(c in "0123456789abcdefABCDEF" for c in text)


def _resolve_git_dir(repo_root: Path) -> Path | None:
    """Return the real git metadata dir for ``repo_root``, or ``None``.

    Handles the common case (``.git`` is a directory). For worktrees,
    resolves only detached-HEAD checkouts (where ``.git`` is a file
    containing ``gitdir: <path>``). Branch-checkout worktrees have refs
    in the commondir and fall through to return ``None`` here.
    """
    git_path = repo_root / ".git"
    if git_path.is_dir():
        return git_path
    if git_path.is_file():
        content = git_path.read_text(encoding="utf-8").strip()
        prefix = "gitdir:"
        if content.startswith(prefix):
            target = Path(content[len(prefix) :].strip())
            if not target.is_absolute():
                target = (repo_root / target).resolve()
            if target.is_dir():
                return target
    return None


def _sha_from_packed_refs(git_dir: Path, ref_path: str) -> str | None:
    packed_refs = git_dir / "packed-refs"
    if not packed_refs.is_file():
        return None
    with packed_refs.open("r", encoding="utf-8") as file_obj:
        for raw_line in file_obj:
            line = raw_line.strip()
            if not line or line[0] in "#^":
                continue
            sha, _, path = line.partition(" ")
            if path == ref_path and _looks_like_git_sha(sha):
                return sha
    return None


def _read_git_commit_sha(repo_root: Path) -> str | None:
    """Parse the current commit sha out of ``repo_root/.git``.

    Pure Python, no ``git`` binary / subprocess. Returns ``None`` when
    nothing resolves; callers are expected to wrap this in a broad
    try/except since the many small filesystem reads here should
    never be allowed to raise out to the caller.
    """
    git_dir = _resolve_git_dir(repo_root)
    if git_dir is None:
        return None

    head_text = (git_dir / "HEAD").read_text(encoding="utf-8").strip()
    if not head_text:
        return None

    if head_text.startswith("ref:"):
        ref_path = head_text[len("ref:") :].strip()
        loose_ref = git_dir / ref_path
        if loose_ref.is_file():
            sha = loose_ref.read_text(encoding="utf-8").strip()
            if _looks_like_git_sha(sha):
                return sha
        return _sha_from_packed_refs(git_dir, ref_path)

    if _looks_like_git_sha(head_text):
        return head_text
    return None


def resolve_evaluator_version(repo_root: Path | None = None) -> str:
    """Resolve a short version string identifying this evaluator build.

    First hit wins:

    1. The ``EVAL_TASK2_VERSION`` env var (manual override).
    2. A pure-Python parse of ``<repo_root>/.git`` for the current
       commit sha, truncated to 7 chars. No ``git`` binary and no
       subprocess -- the eval container bind-mounts the repo but has
       neither installed. ``repo_root`` defaults to the checkout this
       file lives in (``<repo>/scripts/evaluation/task2/config.py``).
    3. The ``EVAL_TASK2_VERSION_BUILD`` env var, baked into the image
       at build time (see Dockerfile) -- covers running the image
       standalone, without the repo bind-mounted over it.
    4. ``"unknown"``.

    Never raises: any IO/parse failure at a given source falls
    through to the next one.
    """
    env_version = os.environ.get("EVAL_TASK2_VERSION", "").strip()
    if env_version:
        return env_version

    if repo_root is None:
        repo_root = Path(__file__).resolve().parents[3]

    try:
        sha = _read_git_commit_sha(repo_root)
    except Exception:
        # Best-effort diagnostics only: a missing/corrupt .git, an
        # unexpected layout, or a permissions error must never break
        # the caller -- fall through to the next version source.
        sha = None
    if sha:
        return sha[:7]

    build_version = os.environ.get("EVAL_TASK2_VERSION_BUILD", "").strip()
    if build_version:
        return build_version

    return "unknown"


def _default_config_path() -> Path:
    return Path(__file__).with_name("config.yaml")


def _load_yaml_config(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        return {}
    with config_path.open("r", encoding="utf-8") as file_obj:
        loaded = yaml.safe_load(file_obj) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Config file must be a YAML mapping: {config_path}")
    # Support an optional nested 'eval_task2' section.
    if "eval_task2" in loaded:
        nested = loaded["eval_task2"]
        if not isinstance(nested, dict):
            raise ValueError(
                "'eval_task2' section must be a mapping in config YAML"
            )
        return dict(nested)
    return dict(loaded)


def _build_arg_parser(defaults: dict[str, Any]) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Eval task2 camera capture and IoU evaluation service"
    )
    parser.add_argument(
        "--config",
        type=str,
        default=str(_default_config_path()),
        help=(
            "Path to YAML config file (default: this directory's config.yaml)"
        ),
    )
    parser.add_argument(
        "--image-topic", type=str, default=str(defaults["image_topic"])
    )
    parser.add_argument(
        "--depth-topic", type=str, default=str(defaults["depth_topic"])
    )
    parser.add_argument(
        "--semantic-segmentation-topic",
        type=str,
        default=str(defaults["semantic_segmentation_topic"]),
    )
    parser.add_argument(
        "--semantic-labels-topic",
        type=str,
        default=str(defaults["semantic_labels_topic"]),
    )
    parser.add_argument(
        "--bbox-2d-tight-topic",
        type=str,
        default=str(defaults["bbox_2d_tight_topic"]),
    )
    parser.add_argument(
        "--bbox-tight-labels-topic",
        type=str,
        default=str(defaults["bbox_tight_labels_topic"]),
    )
    parser.add_argument(
        "--bbox-2d-loose-topic",
        type=str,
        default=str(defaults["bbox_2d_loose_topic"]),
    )
    parser.add_argument(
        "--bbox-loose-labels-topic",
        type=str,
        default=str(defaults["bbox_loose_labels_topic"]),
    )
    parser.add_argument(
        "--camera-info-topic",
        type=str,
        default=str(defaults["camera_info_topic"]),
    )
    parser.add_argument(
        "--evaluate-service-name",
        type=str,
        default=str(defaults["evaluate_service_name"]),
    )
    parser.add_argument(
        "--output-dir", type=str, default=str(defaults["output_dir"])
    )
    parser.add_argument(
        "--jpeg-quality", type=int, default=int(defaults["jpeg_quality"])
    )
    parser.add_argument(
        "--thermalpad-label",
        type=str,
        default=str(defaults["thermalpad_label"]),
    )
    parser.add_argument(
        "--liner-label", type=str, default=str(defaults["liner_label"])
    )
    parser.add_argument(
        "--target-label", type=str, default=str(defaults["target_label"])
    )
    parser.add_argument(
        "--bbox-json-top-per-class-only",
        type=coerce_bool,
        default=coerce_bool(defaults["bbox_json_top_per_class_only"]),
    )
    parser.add_argument(
        "--sync-tolerance-s",
        type=float,
        default=float(defaults["sync_tolerance_s"]),
    )
    parser.add_argument(
        "--sync-timeout-s",
        type=float,
        default=float(defaults["sync_timeout_s"]),
    )
    parser.add_argument(
        "--sync-max-age-s",
        type=float,
        default=float(defaults["sync_max_age_s"]),
    )
    parser.add_argument(
        "--sync-rebase-epsilon-s",
        type=float,
        default=float(defaults["sync_rebase_epsilon_s"]),
    )
    parser.add_argument(
        "--sync-image-buffer-len",
        type=int,
        default=int(defaults["sync_image_buffer_len"]),
    )
    parser.add_argument(
        "--sync-buffer-len",
        type=int,
        default=int(defaults["sync_buffer_len"]),
    )
    parser.add_argument(
        "--audit-enabled",
        type=coerce_bool,
        default=coerce_bool(defaults["audit_enabled"]),
    )
    parser.add_argument(
        "--audit-object-poses-topic",
        type=str,
        default=str(defaults["audit_object_poses_topic"]),
    )
    parser.add_argument(
        "--audit-pad-points-topic",
        type=str,
        default=str(defaults["audit_pad_points_topic"]),
    )
    parser.add_argument(
        "--audit-max-skew-s",
        type=float,
        default=float(defaults["audit_max_skew_s"]),
    )
    return parser


def load_runtime_config(args=None) -> dict[str, Any]:
    # First pass: discover the config path so YAML can feed argparse defaults.
    bootstrap_parser = argparse.ArgumentParser(add_help=False)
    bootstrap_parser.add_argument(
        "--config", type=str, default=str(_default_config_path())
    )
    bootstrap_args, _ = bootstrap_parser.parse_known_args(args=args)

    config_path = Path(bootstrap_args.config)
    yaml_defaults = _load_yaml_config(config_path)
    merged_defaults = dict(APP_DEFAULTS)
    merged_defaults.update(yaml_defaults)

    parser = _build_arg_parser(merged_defaults)
    parsed = parser.parse_args(args=args)

    return {
        "image_topic": parsed.image_topic,
        "depth_topic": parsed.depth_topic,
        "semantic_segmentation_topic": parsed.semantic_segmentation_topic,
        "semantic_labels_topic": parsed.semantic_labels_topic,
        "bbox_2d_tight_topic": parsed.bbox_2d_tight_topic,
        "bbox_tight_labels_topic": parsed.bbox_tight_labels_topic,
        "bbox_2d_loose_topic": parsed.bbox_2d_loose_topic,
        "bbox_loose_labels_topic": parsed.bbox_loose_labels_topic,
        "camera_info_topic": parsed.camera_info_topic,
        "evaluate_service_name": parsed.evaluate_service_name,
        "output_dir": parsed.output_dir,
        "jpeg_quality": int(parsed.jpeg_quality),
        "thermalpad_label": parsed.thermalpad_label,
        "liner_label": parsed.liner_label,
        "target_label": parsed.target_label,
        "bbox_json_top_per_class_only": coerce_bool(
            parsed.bbox_json_top_per_class_only
        ),
        "sync_tolerance_s": float(parsed.sync_tolerance_s),
        "sync_timeout_s": float(parsed.sync_timeout_s),
        "sync_max_age_s": float(parsed.sync_max_age_s),
        "sync_rebase_epsilon_s": float(parsed.sync_rebase_epsilon_s),
        "sync_image_buffer_len": int(parsed.sync_image_buffer_len),
        "sync_buffer_len": int(parsed.sync_buffer_len),
        "audit_enabled": coerce_bool(parsed.audit_enabled),
        "audit_object_poses_topic": parsed.audit_object_poses_topic,
        "audit_pad_points_topic": parsed.audit_pad_points_topic,
        "audit_max_skew_s": float(parsed.audit_max_skew_s),
        "evaluator_version": resolve_evaluator_version(),
    }
