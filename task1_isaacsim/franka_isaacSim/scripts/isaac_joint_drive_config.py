"""Helpers for loading and applying Isaac joint-drive settings."""

from __future__ import annotations

import os

import yaml

from stack_config import REPO_ROOT


DEFAULT_JOINT_DRIVE_CONFIG_PATH = os.path.join(
    REPO_ROOT,
    "assets",
    "isaac_assets",
    "config",
    "fr3_duo_isaac_joint_drives.yaml",
)


def resolve_joint_drive_config_path(config_path: str | None = None) -> str:
    raw_path = config_path or DEFAULT_JOINT_DRIVE_CONFIG_PATH
    expanded = os.path.expanduser(str(raw_path))
    if expanded.startswith("/workspace/"):
        expanded = os.path.join(REPO_ROOT, expanded[len("/workspace/") :])
    return os.path.abspath(expanded)


def load_joint_drive_config(config_path: str | None = None) -> dict:
    resolved_path = resolve_joint_drive_config_path(config_path)
    if not os.path.exists(resolved_path):
        raise FileNotFoundError(f"Joint drive config file not found: {resolved_path}")
    with open(resolved_path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"Joint drive config root must be a mapping: {resolved_path}")
    loaded["_config_path"] = resolved_path
    return loaded


def resolve_scaled_joint_drives(
    config: dict,
    *,
    stiffness_scale: float = 1.0,
    damping_scale: float = 1.0,
    max_force_scale: float = 1.0,
    gripper_stiffness_scale: float = 1.0,
    gripper_damping_scale: float = 1.0,
    gripper_max_force_scale: float = 1.0,
) -> dict[str, dict]:
    joint_drives = config.get("joint_drives", {})
    if not isinstance(joint_drives, dict):
        raise ValueError("joint_drives must be a mapping")

    # Import gripper joint lists to identify gripper joints
    try:
        from isaac_bridge_constants import LEFT_GRIPPER_JOINTS, RIGHT_GRIPPER_JOINTS
        gripper_joints = set(LEFT_GRIPPER_JOINTS + RIGHT_GRIPPER_JOINTS)
    except ImportError:
        gripper_joints = set()

    scaled = {}
    for joint_name, joint_config in joint_drives.items():
        if not isinstance(joint_config, dict):
            raise ValueError(f"Joint drive entry must be a mapping: {joint_name}")
        
        # Use gripper-specific scales for gripper joints, arm scales for arm joints
        is_gripper = str(joint_name) in gripper_joints
        effective_stiffness_scale = gripper_stiffness_scale if is_gripper else stiffness_scale
        effective_damping_scale = gripper_damping_scale if is_gripper else damping_scale
        effective_max_force_scale = gripper_max_force_scale if is_gripper else max_force_scale
        
        scaled[str(joint_name)] = {
            "prim_path": str(joint_config.get("prim_path", "")),
            "drive_axis": str(joint_config.get("drive_axis", "angular")),
            "drive_type": str(joint_config.get("drive_type", "acceleration")),
            "stiffness": float(joint_config.get("stiffness", 0.0)) * float(effective_stiffness_scale),
            "damping": float(joint_config.get("damping", 0.0)) * float(effective_damping_scale),
            "max_force": float(joint_config.get("max_force", 0.0)) * float(effective_max_force_scale),
        }
    return scaled


def _resolve_joint_prim(stage, joint_name: str, prim_path: str):
    prim = None
    if prim_path:
        try:
            prim = stage.GetPrimAtPath(prim_path)
        except Exception:
            prim = None
    if prim and prim.IsValid():
        return prim

    candidates = []
    try:
        for stage_prim in stage.Traverse():
            if stage_prim.GetName() == joint_name:
                return stage_prim
            if stage_prim.GetName().endswith(joint_name):
                candidates.append(stage_prim)
    except Exception:
        return None
    return candidates[0] if candidates else None


def _ensure_attr(schema_api, getter_name: str, creator_name: str):
    getter = getattr(schema_api, getter_name, None)
    attr = getter() if callable(getter) else None
    if attr and attr.IsValid():
        return attr
    creator = getattr(schema_api, creator_name, None)
    if callable(creator):
        return creator()
    return None


def apply_joint_drive_configuration(
    stage,
    *,
    config_path: str | None = None,
    stiffness_scale: float = 1.0,
    damping_scale: float = 1.0,
    max_force_scale: float = 1.0,
    gripper_stiffness_scale: float = 1.0,
    gripper_damping_scale: float = 1.0,
    gripper_max_force_scale: float = 1.0,
) -> dict:
    if stage is None or not config_path:
        return {
            "config_path": None,
            "applied_count": 0,
            "missing_joints": [],
            "scales": {
                "stiffness_scale": float(stiffness_scale),
                "damping_scale": float(damping_scale),
                "max_force_scale": float(max_force_scale),
                "gripper_stiffness_scale": float(gripper_stiffness_scale),
                "gripper_damping_scale": float(gripper_damping_scale),
                "gripper_max_force_scale": float(gripper_max_force_scale),
            },
        }

    config = load_joint_drive_config(config_path)
    scaled_drives = resolve_scaled_joint_drives(
        config,
        stiffness_scale=stiffness_scale,
        damping_scale=damping_scale,
        max_force_scale=max_force_scale,
        gripper_stiffness_scale=gripper_stiffness_scale,
        gripper_damping_scale=gripper_damping_scale,
        gripper_max_force_scale=gripper_max_force_scale,
    )

    from pxr import UsdPhysics

    applied_count = 0
    missing_joints = []
    for joint_name, joint_config in scaled_drives.items():
        prim = _resolve_joint_prim(stage, joint_name, joint_config["prim_path"])
        if prim is None or not prim.IsValid():
            missing_joints.append(joint_name)
            continue

        drive_api = UsdPhysics.DriveAPI.Get(prim, joint_config["drive_axis"])
        if not drive_api:
            drive_api = UsdPhysics.DriveAPI.Apply(prim, joint_config["drive_axis"])

        type_attr = _ensure_attr(drive_api, "GetTypeAttr", "CreateTypeAttr")
        stiffness_attr = _ensure_attr(drive_api, "GetStiffnessAttr", "CreateStiffnessAttr")
        damping_attr = _ensure_attr(drive_api, "GetDampingAttr", "CreateDampingAttr")
        max_force_attr = _ensure_attr(drive_api, "GetMaxForceAttr", "CreateMaxForceAttr")
        if type_attr is not None:
            type_attr.Set(joint_config["drive_type"])
        if stiffness_attr is not None:
            stiffness_attr.Set(float(joint_config["stiffness"]))
        if damping_attr is not None:
            damping_attr.Set(float(joint_config["damping"]))
        if max_force_attr is not None:
            max_force_attr.Set(float(joint_config["max_force"]))
        applied_count += 1

    return {
        "config_path": resolve_joint_drive_config_path(config_path),
        "applied_count": applied_count,
        "missing_joints": missing_joints,
        "scales": {
            "stiffness_scale": float(stiffness_scale),
            "damping_scale": float(damping_scale),
            "max_force_scale": float(max_force_scale),
            "gripper_stiffness_scale": float(gripper_stiffness_scale),
            "gripper_damping_scale": float(gripper_damping_scale),
            "gripper_max_force_scale": float(gripper_max_force_scale),
        },
    }
