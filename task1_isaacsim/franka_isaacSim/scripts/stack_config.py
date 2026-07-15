"""Shared YAML-backed configuration helpers for the Isaac/ROS stack."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import sys
from typing import Iterable

import yaml


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
DEFAULT_STACK_CONFIG_PATH = os.path.join(
    REPO_ROOT,
    "assets",
    "isaac_assets",
    "config",
    "stack_defaults.yaml",
)
DEFAULT_TEST_CONFIG_PATH = os.path.join(
    REPO_ROOT,
    "tests",
    "test_config.yaml",
)
DEFAULT_EMBODIMENT = "fr3duo_m+v"


# Import embodiment loader
EMBODIMENTS_DIR = os.path.join(REPO_ROOT, "assets", "embodiments")
if EMBODIMENTS_DIR not in sys.path:
    sys.path.insert(0, EMBODIMENTS_DIR)

try:
    from loader import get_embodiment_metadata, get_joint_drive_config, resolve_embodiment_file_path
except ImportError:
    get_embodiment_metadata = None
    get_joint_drive_config = None
    resolve_embodiment_file_path = None


def load_embodiment_config(embodiment_name: str = None) -> dict:
    """Load configuration from an embodiment.
    
    Args:
        embodiment_name: Name of the embodiment (default: fr3duo_m+v)
        
    Returns:
        Dictionary with merged configuration from embodiment_config.yaml and joint_drive_config.yaml
    """
    if not embodiment_name:
        embodiment_name = DEFAULT_EMBODIMENT
        
    if get_embodiment_metadata is None or get_joint_drive_config is None or resolve_embodiment_file_path is None:
        raise ImportError("Embodiment loader not available. Check assets/embodiments/loader.py")
        
    embodiment = get_embodiment_metadata(embodiment_name)
    joint_drive_config = get_joint_drive_config(embodiment_name)
    
    # Map embodiment config to stack config format
    merged = {}
    
    # Physics settings
    physics = embodiment.get("physics", {})
    merged["physics_hz"] = physics.get("simulation_hz", 240.0)
    merged["render_hz"] = physics.get("render_hz", 60.0)
    merged["physics_substeps"] = physics.get("substeps", 2)
    
    # Control settings
    control = embodiment.get("control", {})
    merged["controller_mode"] = control.get("controller_mode", "effort")
    merged["ros_publish_rate"] = control.get("ros_publish_rate_hz", 60.0)
    merged["command_smoothing_alpha"] = control.get("command_smoothing_alpha", 1.0)
    merged["max_position_step_rad"] = control.get("max_position_step_rad", 0.0)
    merged["position_deadband_rad"] = control.get("position_deadband_rad", 0.0)
    merged["settle_position_window_rad"] = control.get("settle_position_window_rad", 0.0)
    merged["settle_velocity_threshold_rad_s"] = control.get("settle_velocity_threshold_rad_s", 0.0)
    merged["browser_command_hold_seconds"] = control.get("browser_command_hold_seconds", 0.25)
    merged["controller_activity_topic"] = control.get("controller_activity_topic", "/isaac_controller_manager/activity")
    merged["primary_effort_stale_after_s"] = control.get("primary_effort_stale_after_s", 0.25)
    
    # Isaac joint drives (from joint_drive_config.yaml)
    joint_drives = joint_drive_config.get("isaac_joint_drives", {})
    scaling = joint_drive_config.get("scaling_parameters", {})
    joint_drive_config_path = joint_drives.get("joint_drive_config", "isaac_joint_drives.yaml")
    merged["joint_drive_config"] = resolve_embodiment_file_path(embodiment_name, joint_drive_config_path)
    # Support both nested (arms/grippers) and flat formats; flat format is the fallback.
    arm_scaling = scaling.get("arms", scaling)
    gripper_scaling = scaling.get("grippers", {})
    merged["joint_drive_stiffness_scale"] = arm_scaling.get("stiffness_scale", 0.65)
    merged["joint_drive_damping_scale"] = arm_scaling.get("damping_scale", 1.0)
    merged["joint_drive_max_force_scale"] = arm_scaling.get("max_force_scale", 3.0)
    merged["joint_drive_gripper_stiffness_scale"] = gripper_scaling.get("stiffness_scale", 1.0)
    merged["joint_drive_gripper_damping_scale"] = gripper_scaling.get("damping_scale", 1.0)
    merged["joint_drive_gripper_max_force_scale"] = gripper_scaling.get("max_force_scale", 1.0)
    
    # Cameras
    cameras = embodiment.get("cameras") or {}
    camera_config_path = cameras.get("camera_config")
    if camera_config_path:
        merged["camera_config"] = resolve_embodiment_file_path(embodiment_name, camera_config_path)
    else:
        merged["camera_config"] = ""  # Empty string when cameras disabled
    
    # Controller manager (from joint_drive_config.yaml)
    controller_mgr = joint_drive_config.get("controller_manager", {})
    merged["controller_name"] = controller_mgr.get("controller_name", "position_passthrough")
    merged["controller_manager"] = controller_mgr.get("controller_manager_topic", "/isaac_controller_manager")
    merged["controller_manager_node_name"] = controller_mgr.get("controller_manager_node_name", "isaac_controller_manager")
    merged["update_rate"] = controller_mgr.get("update_rate", 240)
    merged["gripper_max_effort"] = controller_mgr.get("gripper_max_effort", 50.0)
    merged["gripper_stall_timeout"] = controller_mgr.get("gripper_stall_timeout", 0.05)
    merged["gripper_goal_tolerance"] = controller_mgr.get("gripper_goal_tolerance", 0.02)
    
    # Store embodiment metadata
    merged["_embodiment_name"] = embodiment_name
    merged["_embodiment_key"] = embodiment.get("embodiment_key")
    merged["_embodiment_version"] = embodiment.get("version")
    
    return merged


def resolve_config_path(config_path: str | None = None) -> str:
    if not config_path:
        config_path = DEFAULT_STACK_CONFIG_PATH
    return os.path.abspath(os.path.expanduser(config_path))


def load_stack_config(config_path: str | None = None) -> dict:
    resolved_path = resolve_config_path(config_path)
    if not os.path.exists(resolved_path):
        raise FileNotFoundError(f"Stack config file not found: {resolved_path}")
    with open(resolved_path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Stack config root must be a mapping: {resolved_path}")
    loaded["_config_path"] = resolved_path
    return loaded


def get_config_sections(config: dict, section_names: Iterable[str]) -> dict:
    """Get configuration sections from a config dict.
    
    For test-specific sections (probe, *_test, *_audit, *_optimization),
    also checks tests/test_config.yaml if section not found in main config.
    
    Args:
        config: Main configuration dictionary
        section_names: Section names to extract
        
    Returns:
        Merged configuration dictionary
    """
    merged = {}
    test_sections = []
    
    # Known test section patterns
    test_section_patterns = (
        'probe', '_test', '_audit', '_optimization',
        'current_target_hold', 'episode_replay', 'burn_test', 
        'camera_output_test', 'isaac_drive_optimization'
    )
    
    for section_name in section_names:
        if not section_name:
            continue
        section = config.get(section_name, {})
        
        # If section not found and looks like a test section, mark it
        if not section and any(pattern in section_name for pattern in test_section_patterns):
            test_sections.append(section_name)
            continue
            
        if section is None:
            section = {}
        if not isinstance(section, dict):
            raise ValueError(f"Config section '{section_name}' must be a mapping")
        merged.update(section)
    
    # Load missing test sections from test config
    if test_sections:
        try:
            test_config = load_stack_config(DEFAULT_TEST_CONFIG_PATH)
            for section_name in test_sections:
                section = test_config.get(section_name, {})
                if section and isinstance(section, dict):
                    merged.update(section)
        except FileNotFoundError:
            pass  # Test config not required
    
    return merged


def add_config_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--config",
        type=str,
        default=DEFAULT_STACK_CONFIG_PATH,
        help="Path to the shared stack YAML configuration file",
    )


def _coerce_defaults_for_parser(
    parser: argparse.ArgumentParser,
    defaults: dict,
) -> dict:
    actions = {
        action.dest: action
        for action in getattr(parser, "_actions", [])
        if getattr(action, "dest", None)
    }
    coerced = {}
    for key, value in defaults.items():
        action = actions.get(key)
        if value == "" and action is not None and getattr(action, "type", None) not in (None, str):
            coerced[key] = None
            continue
        coerced[key] = value
    return coerced


def _bootstrap_config_path(argv=None) -> tuple[str, str | None]:
    """Bootstrap config path and embodiment name from argv.
    
    Returns:
        Tuple of (config_path, embodiment_name)
    """
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument("--config", type=str, default=DEFAULT_STACK_CONFIG_PATH)
    bootstrap.add_argument("--embodiment", type=str, default=None)
    known, _ = bootstrap.parse_known_args(argv)
    return resolve_config_path(known.config), known.embodiment


def apply_config_defaults(
    parser: argparse.ArgumentParser,
    section_names: Iterable[str],
    argv=None,
) -> str:
    """Apply configuration defaults to argument parser.
    
    Supports two modes:
    1. Embodiment mode (--embodiment): Load config from embodiment files
    2. Legacy mode: Load config from stack_defaults.yaml
    
    Args:
        parser: Argument parser to configure
        section_names: Config sections to merge (legacy mode only)
        argv: Command line arguments to parse
        
    Returns:
        Path to config file or embodiment name used
    """
    config_path, embodiment_name = _bootstrap_config_path(argv)
    
    if embodiment_name:
        # Use embodiment configuration
        config = load_embodiment_config(embodiment_name)
        parser.set_defaults(**_coerce_defaults_for_parser(parser, config))
        return embodiment_name
    
    # Legacy: use stack_defaults.yaml
    config = load_stack_config(config_path)
    parser.set_defaults(
        **_coerce_defaults_for_parser(parser, get_config_sections(config, section_names))
    )
    return config_path


def _to_shell_value(value) -> str:
    if value is None:
        return "''"
    if isinstance(value, bool):
        return shlex.quote("true" if value else "false")
    return shlex.quote(str(value))


def build_runtime_shell_assignments(config_path: str | None = None, embodiment_name: str | None = None) -> list[str]:
    """Build shell variable assignments for runtime configuration.
    
    Args:
        config_path: Path to stack_defaults.yaml (legacy mode)
        embodiment_name: Name of embodiment to load (preferred mode)
        
    Returns:
        List of shell variable assignment strings
    """
    if embodiment_name:
        merged = load_embodiment_config(embodiment_name)
    else:
        config = load_stack_config(config_path)
        merged = get_config_sections(config, ("simulation", "bridge", "isaac_joint_drives", "cameras"))
    
    mapping = {
        "force_recreate": "FORCE_RECREATE",
        "start_republisher": "START_REPUBLISHER",
        "start_browser": "START_BROWSER",
        "stream": "ENABLE_STREAM",
        "stream_ui": "STREAM_UI",
        "embedded_app": "EMBEDDED_APP_MODE",
        "controller_mode": "CONTROLLER_MODE",
        "ros_publish_rate": "ROS_PUBLISH_RATE",
        "physics_hz": "PHYSICS_HZ",
        "render_hz": "RENDER_HZ",
        "physics_substeps": "PHYSICS_SUBSTEPS",
        "command_smoothing_alpha": "COMMAND_SMOOTHING_ALPHA",
        "max_position_step_rad": "MAX_POSITION_STEP_RAD",
        "position_deadband_rad": "POSITION_DEADBAND_RAD",
        "settle_position_window_rad": "SETTLE_POSITION_WINDOW_RAD",
        "settle_velocity_threshold_rad_s": "SETTLE_VELOCITY_THRESHOLD_RAD_S",
        "joint_drive_config": "JOINT_DRIVE_CONFIG",
        "joint_drive_stiffness_scale": "JOINT_DRIVE_STIFFNESS_SCALE",
        "joint_drive_damping_scale": "JOINT_DRIVE_DAMPING_SCALE",
        "joint_drive_max_force_scale": "JOINT_DRIVE_MAX_FORCE_SCALE",
        "joint_drive_gripper_stiffness_scale": "JOINT_DRIVE_GRIPPER_STIFFNESS_SCALE",
        "joint_drive_gripper_damping_scale": "JOINT_DRIVE_GRIPPER_DAMPING_SCALE",
        "joint_drive_gripper_max_force_scale": "JOINT_DRIVE_GRIPPER_MAX_FORCE_SCALE",
        "camera_config": "CAMERA_CONFIG",
        "browser_command_hold_seconds": "BROWSER_COMMAND_HOLD_SECONDS",
        "controller_activity_topic": "CONTROLLER_ACTIVITY_TOPIC",
        "primary_effort_stale_after_s": "PRIMARY_EFFORT_STALE_AFTER_S",
        "asset_name": "ASSET_NAME",
        "asset_index": "ASSET_INDEX",
        "usd_path": "USD_PATH",
        "robot_prim_path": "ROBOT_PRIM_PATH",
        "list_assets": "LIST_ASSETS",
        "portable_root": "PORTABLE_ROOT",
    }
    assignments = []
    
    # Add embodiment metadata if using embodiment mode
    if embodiment_name:
        assignments.append(f"EMBODIMENT_NAME={_to_shell_value(embodiment_name)}")
        if "_embodiment_key" in merged:
            assignments.append(f"EMBODIMENT_KEY={_to_shell_value(merged['_embodiment_key'])}")
    
    for config_key, shell_var in mapping.items():
        if config_key not in merged:
            continue
        assignments.append(f"{shell_var}={_to_shell_value(merged[config_key])}")
    return assignments


def main():
    parser = argparse.ArgumentParser()
    add_config_argument(parser)
    parser.add_argument(
        "--embodiment",
        type=str,
        default=None,
        help="Embodiment name to load configuration from (overrides --config)",
    )
    parser.add_argument(
        "--format",
        choices=("json", "shell-runtime"),
        default="json",
        help="Output format",
    )
    parser.add_argument(
        "--sections",
        nargs="*",
        default=[],
        help="Config sections to merge when using JSON output (legacy mode only)",
    )
    args = parser.parse_args()

    if args.format == "shell-runtime":
        print("\n".join(build_runtime_shell_assignments(
            config_path=args.config if not args.embodiment else None,
            embodiment_name=args.embodiment
        )))
        return 0

    if args.embodiment:
        # Embodiment mode: return full merged config
        config = load_embodiment_config(args.embodiment)
        print(json.dumps(config, indent=2, sort_keys=True))
        return 0

    # Legacy mode: load from stack_defaults.yaml
    config = load_stack_config(args.config)
    payload = (
        get_config_sections(config, args.sections)
        if args.sections
        else {key: value for key, value in config.items() if key != "_config_path"}
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
