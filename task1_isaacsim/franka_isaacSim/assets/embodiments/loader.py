"""Embodiment loader and utilities (IsaacSim-focused).

Provides functions to discover, load, and validate robot embodiment configurations.
Embodiments contain IsaacSim-specific parameters (physics tuning, control settings).
Joint/kinematic truth is sourced from URDF/USD files referenced by each embodiment.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


EMBODIMENTS_ROOT = os.path.dirname(os.path.abspath(__file__))


def list_available_embodiments() -> list[str]:
    """List all available embodiments in the embodiments directory.

    Returns:
        Sorted list of embodiment names (subdirectory names).
    """
    embodiments = []
    for item in os.listdir(EMBODIMENTS_ROOT):
        item_path = os.path.join(EMBODIMENTS_ROOT, item)
        if os.path.isdir(item_path) and item not in ("__pycache__",):
            # Check if it has at least the core config file
            if os.path.exists(os.path.join(item_path, "embodiment_config.yaml")):
                embodiments.append(item)
    return sorted(embodiments)


def resolve_embodiment_path(embodiment_name: str) -> str:
    """Resolve the full path to an embodiment directory.

    Args:
        embodiment_name: Name of the embodiment (e.g., "fr3duo_m+v").

    Returns:
        Absolute path to the embodiment directory.

    Raises:
        FileNotFoundError: If the embodiment does not exist.
    """
    path = os.path.join(EMBODIMENTS_ROOT, embodiment_name)
    if not os.path.isdir(path):
        available = list_available_embodiments()
        raise FileNotFoundError(
            f"Embodiment '{embodiment_name}' not found. "
            f"Available: {available}"
        )
    return path


def load_yaml(file_path: str) -> dict:
    """Load a YAML file and return its contents as a dict.

    Args:
        file_path: Absolute path to the YAML file.

    Returns:
        Parsed YAML as a dictionary.

    Raises:
        FileNotFoundError: If the file does not exist.
        ValueError: If the YAML is malformed.
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"YAML file not found: {file_path}")
    try:
        with open(file_path, "r", encoding="utf-8") as handle:
            content = yaml.safe_load(handle) or {}
        if not isinstance(content, dict):
            raise ValueError(f"YAML root must be a mapping: {file_path}")
        return content
    except yaml.YAMLError as exc:
        raise ValueError(f"Failed to parse YAML {file_path}: {exc}") from exc


def resolve_embodiment_file_path(embodiment_name: str, file_path: str) -> str:
    """Resolve a file path that may be relative to the embodiment directory.
    
    Args:
        embodiment_name: Name of the embodiment.
        file_path: Path to resolve (absolute or relative to embodiment dir).
        
    Returns:
        Absolute path to the file.
        
    Examples:
        - "camera_sensors.yaml" → "/path/to/embodiments/fr3duo_m+v/camera_sensors.yaml"
        - "/workspace/assets/config/file.yaml" → "/workspace/assets/config/file.yaml"
    """
    # If already absolute or starts with /workspace, return as-is
    if os.path.isabs(file_path) or file_path.startswith("/workspace/"):
        return file_path
    
    # Otherwise, resolve relative to embodiment directory
    embodiment_path = resolve_embodiment_path(embodiment_name)
    return os.path.join(embodiment_path, file_path)


def load_embodiment_component(
    embodiment_name: str, component_name: str
) -> dict:
    """Load a single component YAML from an embodiment.

    Args:
        embodiment_name: Name of the embodiment.
        component_name: Name of the component file (without .yaml extension).
                        E.g., "asset_references", "joint_drive_config".

    Returns:
        Parsed component as a dictionary (empty dict if optional file missing).

    Raises:
        FileNotFoundError: If the embodiment does not exist.
        ValueError: If YAML is malformed.
    """
    embodiment_path = resolve_embodiment_path(embodiment_name)
    component_path = os.path.join(
        embodiment_path, f"{component_name}.yaml"
    )
    
    if not os.path.exists(component_path):
        return {}
    
    return load_yaml(component_path)


def load_embodiment(embodiment_name: str) -> dict[str, dict]:
    """Load all IsaacSim components of an embodiment (no URDF duplication).

    Loads IsaacSim-specific configuration files:
    - embodiment_config.yaml (metadata, physics settings, gripper types)
    - joint_drive_config.yaml (IsaacSim physics tuning: stiffness, damping, force)
    - data_contract.yaml (sampling rate, data structure for ML pipelines)

    Note: Asset paths are resolved from asset names in embodiment_config.

    Args:
        embodiment_name: Name of the embodiment.

    Returns:
        Dictionary with keys: embodiment_config, joint_drive_config, data_contract.

    Raises:
        FileNotFoundError: If the embodiment does not exist.
        ValueError: If any YAML is malformed.
    """
    # Verify embodiment exists
    resolve_embodiment_path(embodiment_name)
    
    components = [
        "embodiment_config",
        "joint_drive_config",
        "data_contract",
    ]
    result = {}
    for component in components:
        try:
            result[component] = load_embodiment_component(
                embodiment_name, component
            )
        except FileNotFoundError as exc:
            raise FileNotFoundError(
                f"Required component '{component}.yaml' missing "
                f"for embodiment '{embodiment_name}': {exc}"
            ) from exc
    return result


def get_embodiment_metadata(embodiment_name: str) -> dict:
    """Get metadata about an embodiment without loading all components.

    Convenience function that loads only embodiment_config.yaml.

    Args:
        embodiment_name: Name of the embodiment.

    Returns:
        The embodiment_config as a dictionary.
    """
    return load_embodiment_component(embodiment_name, "embodiment_config")


def get_joint_drive_config(embodiment_name: str) -> dict:
    """Get IsaacSim joint drive parameters.

    Args:
        embodiment_name: Name of the embodiment.

    Returns:
        The joint_drive_config as a dictionary with stiffness, damping, force scales.
    """
    return load_embodiment_component(embodiment_name, "joint_drive_config")


def validate_embodiment(embodiment_name: str) -> tuple[bool, str]:
    """Validate an embodiment configuration.

    Checks that all required IsaacSim files exist, YAML is well-formed, and
    key fields are present.

    Args:
        embodiment_name: Name of the embodiment.

    Returns:
        Tuple of (is_valid, message). If not valid, message contains
        the validation error.
    """
    try:
        embodiment = load_embodiment(embodiment_name)
        
        # Check required fields in embodiment_config
        config = embodiment.get("embodiment_config", {})
        if not config:
            return False, "embodiment_config is empty"
        
        if "embodiment_key" not in config:
            return False, "embodiment_config missing 'embodiment_key'"
        
        if "platform" not in config:
            return False, "embodiment_config missing 'platform' section"
        
        # Check consistency: all components reference same embodiment_key
        embodiment_key = config["embodiment_key"]
        
        for component_name in ["joint_drive_config", "data_contract"]:
            comp_data = embodiment.get(component_name, {})
            if comp_data and comp_data.get("embodiment_key") != embodiment_key:
                return False, f"{component_name} has mismatched embodiment_key"
        
        return True, "Embodiment is valid (IsaacSim configs only)"
    
    except (FileNotFoundError, ValueError) as exc:
        return False, str(exc)
