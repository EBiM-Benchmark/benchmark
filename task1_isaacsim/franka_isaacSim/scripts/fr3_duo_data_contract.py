#!/usr/bin/env python3
"""Helpers to project recorder output into the FR3 duo data contract."""

from __future__ import annotations

import os

import yaml

from stack_config import REPO_ROOT


DEFAULT_CONTRACT_PATH = os.path.join(
    REPO_ROOT,
    "assets",
    "isaac_assets",
    "config",
    "fr3_duo_ai_data_contract.yaml",
)
DEFAULT_EMBODIMENT_KEY = "franka_fr3_duo"


def load_data_contract(contract_path: str | None = None) -> dict:
    resolved_path = os.path.abspath(os.path.expanduser(contract_path or DEFAULT_CONTRACT_PATH))
    with open(resolved_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def validate_pose_test_contract_export(export: dict, contract_path: str | None = None) -> dict:
    contract = load_data_contract(contract_path)
    errors = []
    warnings = []

    if export.get("contract_name") != contract.get("contract_name"):
        errors.append("contract_name does not match the configured data contract")
    if export.get("contract_version") != contract.get("contract_version"):
        errors.append("contract_version does not match the configured data contract")

    metadata = export.get("dataset_metadata")
    if not isinstance(metadata, dict):
        errors.append("dataset_metadata must be an object")
        metadata = {}
    required_metadata = (
        contract.get("data_generation", {})
        .get("dataset_metadata", {})
        .get("required_fields", [])
    )
    for field_name in required_metadata:
        if field_name not in metadata:
            errors.append(f"dataset_metadata missing required field {field_name}")

    episode = export.get("episode")
    if not isinstance(episode, dict):
        errors.append("episode must be an object")
        episode = {}
    timesteps = episode.get("timesteps")
    if not isinstance(timesteps, list) or not timesteps:
        errors.append("episode.timesteps must be a non-empty list")
        timesteps = []

    horizon_steps = int(
        contract.get("data_generation", {})
        .get("action_horizon", {})
        .get("default_horizon_steps", 16)
    )
    for timestep in timesteps:
        if not isinstance(timestep, dict):
            errors.append("each timestep must be an object")
            continue
        if not isinstance(timestep.get("timestamp_utc_ns"), int):
            errors.append("timestamp_utc_ns must be an integer")
        if not isinstance(timestep.get("timestep_index"), int):
            errors.append("timestep_index must be an integer")

        state = timestep.get("state", {})
        for arm_key in ("left_arm", "right_arm"):
            values = state.get(arm_key)
            if not isinstance(values, list) or len(values) != 7:
                errors.append(f"state.{arm_key} must be a 7-element list")
        for gripper_key in ("left_gripper", "right_gripper"):
            if not isinstance(state.get(gripper_key), (int, float)):
                errors.append(f"state.{gripper_key} must be a scalar")

        action = timestep.get("action", {})
        delta_indices = action.get("delta_indices")
        if not isinstance(delta_indices, list) or len(delta_indices) != horizon_steps:
            errors.append("action.delta_indices must match the configured action horizon")
        for arm_key in ("left_arm", "right_arm"):
            horizon = action.get(arm_key)
            if not isinstance(horizon, list) or len(horizon) != horizon_steps:
                errors.append(f"action.{arm_key} must contain one target per horizon step")
                continue
            for frame in horizon:
                if not isinstance(frame, list) or len(frame) != 7:
                    errors.append(f"every action.{arm_key} entry must be a 7-element list")
                    break
        for gripper_key in ("left_gripper", "right_gripper"):
            horizon = action.get(gripper_key)
            if not isinstance(horizon, list) or len(horizon) != horizon_steps:
                errors.append(f"action.{gripper_key} must contain one target per horizon step")
                continue
            if not all(isinstance(value, (int, float)) for value in horizon):
                errors.append(f"every action.{gripper_key} entry must be a scalar")

        task_description = (
            timestep.get("language", {})
            .get("annotation", {})
            .get("human", {})
            .get("action", {})
            .get("task_description")
        )
        if not isinstance(task_description, str):
            errors.append("language.annotation.human.action.task_description must be a string")

    modality_availability = export.get("modality_availability", {})
    if not modality_availability.get("video", False):
        warnings.append(
            "Video modalities from the contract are not produced by the pose-test recorders; "
            "this export covers the state/action/language data handled by this repo flow."
        )

    return {
        "compliant": not errors,
        "errors": errors,
        "warnings": warnings,
    }


def build_pose_test_contract_export(
    report: dict,
    *,
    report_name: str,
    task_description: str,
    success: bool,
    left_gripper_target: float,
    right_gripper_target: float,
    contract_path: str | None = None,
) -> dict:
    contract = load_data_contract(contract_path)
    horizon_steps = int(
        contract.get("data_generation", {})
        .get("action_horizon", {})
        .get("default_horizon_steps", 16)
    )
    contract_name = str(contract.get("contract_name", "fr3_duo_data_contract"))
    contract_version = str(contract.get("contract_version", "1.0.0"))
    start_epoch = float(report["config"]["start_epoch"])
    episode_id = f"{report_name}_{int(round(start_epoch * 1000.0))}"
    target_positions = report["target_positions"]

    timesteps = []
    for index, sample in enumerate(report["samples"]):
        timestamp_utc_ns = int(round((start_epoch + float(sample["elapsed_s"])) * 1e9))
        timesteps.append(
            {
                "timestamp_utc_ns": timestamp_utc_ns,
                "timestep_index": int(index),
                "state": {
                    "left_arm": list(sample["left"]["position"]),
                    "left_gripper": float(sample["left_gripper"]),
                    "right_arm": list(sample["right"]["position"]),
                    "right_gripper": float(sample["right_gripper"]),
                },
                "action": {
                    "delta_indices": list(range(horizon_steps)),
                    "left_arm": [list(target_positions["left"]) for _ in range(horizon_steps)],
                    "left_gripper": [float(left_gripper_target) for _ in range(horizon_steps)],
                    "right_arm": [list(target_positions["right"]) for _ in range(horizon_steps)],
                    "right_gripper": [float(right_gripper_target) for _ in range(horizon_steps)],
                },
                "language": {
                    "annotation": {
                        "human": {
                            "action": {
                                "task_description": task_description,
                            }
                        }
                    }
                },
            }
        )

    export = {
        "contract_name": contract_name,
        "contract_version": contract_version,
        "embodiment_key": DEFAULT_EMBODIMENT_KEY,
        "dataset_metadata": {
            "dataset_id": report_name,
            "contract_name": contract_name,
            "contract_version": contract_version,
            "embodiment_key": DEFAULT_EMBODIMENT_KEY,
            "wrench_frame": "not_recorded_in_pose_tests",
            "gripper_semantics": "normalized_open_fraction(1.0=open,0.0=closed)",
            "sampling_hz": float(report["config"]["sample_hz"]),
        },
        "episode": {
            "episode_id": episode_id,
            "step_count": len(timesteps),
            "success": bool(success),
            "timesteps": timesteps,
        },
        "modality_availability": {
            "video": False,
            "state": True,
            "action": True,
            "language": True,
        },
    }
    export["validation"] = validate_pose_test_contract_export(export, contract_path)
    return export
