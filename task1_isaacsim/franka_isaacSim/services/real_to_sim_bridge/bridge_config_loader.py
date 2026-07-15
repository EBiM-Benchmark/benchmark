"""
bridge_config_loader.py — Configuration loader for real-to-sim bridge topic mappings.

Loads YAML configuration defining topic mappings between real robot hardware and
the simulation bridge interface.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class DomainConfig:
    """ROS domain configuration."""
    rmw_implementation: str
    domain_id: int
    cyclonedds_uri: str | None = None


@dataclass
class TopicMapping:
    """Mapping between real robot topic and bridge output topic."""
    name: str
    input_topic: str
    output_topic: str
    type: str
    description: str
    num_joints: int = 7
    expected_joint_names: list[str] = field(default_factory=list)
    enabled: bool = True
    # Gripper-specific fields
    gripper_joint_index: int | None = None
    gripper_opening_joint: str | None = None

    @property
    def is_gripper(self) -> bool:
        """Check if this mapping is for a gripper."""
        return "gripper" in self.name.lower()

    @property
    def is_arm(self) -> bool:
        """Check if this mapping is for an arm."""
        return "arm" in self.name.lower()


@dataclass
class GripperConfig:
    """Gripper normalization configuration."""
    driver_joint_closed: float = 0.0
    driver_joint_open: float = 0.8
    invert: bool = False


@dataclass
class BridgeConfig:
    """Complete bridge configuration."""
    real_robot: DomainConfig
    simulation: DomainConfig
    mappings: list[TopicMapping]
    gripper: GripperConfig

    @classmethod
    def from_yaml(cls, path: str | Path) -> BridgeConfig:
        """Load configuration from YAML file."""
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Bridge config not found: {path}")

        with path.open("r") as f:
            data = yaml.safe_load(f)

        return cls(
            real_robot=DomainConfig(**data["real_robot"]),
            simulation=DomainConfig(**data["simulation"]),
            mappings=[TopicMapping(**m) for m in data["mappings"]],
            gripper=GripperConfig(**data.get("gripper", {})),
        )

    def get_enabled_mappings(self) -> list[TopicMapping]:
        """Get only enabled topic mappings."""
        return [m for m in self.mappings if m.enabled]

    def get_arm_mappings(self) -> list[TopicMapping]:
        """Get arm-specific mappings."""
        return [m for m in self.get_enabled_mappings() if m.is_arm]

    def get_gripper_mappings(self) -> list[TopicMapping]:
        """Get gripper-specific mappings."""
        return [m for m in self.get_enabled_mappings() if m.is_gripper]

    def get_mapping_by_name(self, name: str) -> TopicMapping | None:
        """Get mapping by name."""
        for m in self.mappings:
            if m.name == name:
                return m
        return None

    def get_mapping_by_input_topic(self, topic: str) -> TopicMapping | None:
        """Get mapping by input topic name."""
        for m in self.get_enabled_mappings():
            if m.input_topic == topic:
                return m
        return None


def get_default_config_path() -> Path:
    """Get the default bridge configuration file path."""
    return Path(__file__).parent / "bridge_config.yaml"


def load_bridge_config(config_path: str | Path | None = None) -> BridgeConfig:
    """
    Load bridge configuration from YAML file.

    Args:
        config_path: Path to config file. If None, uses default location.

    Returns:
        BridgeConfig object.
    """
    if config_path is None:
        config_path = get_default_config_path()
    return BridgeConfig.from_yaml(config_path)
