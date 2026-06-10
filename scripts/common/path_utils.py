"""Shared repository path helpers for workshop scripts."""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = REPO_ROOT / "assets"
THIRD_PARTY_DIR = REPO_ROOT / "third_party"
LEGACY_FRANKA_DIR = REPO_ROOT / "franka_description"
FRANKA_DESCRIPTION_DIR = THIRD_PARTY_DIR / "franka_description"


def get_repo_root() -> Path:
    return REPO_ROOT


def asset_path(*parts: str) -> Path:
    return ASSETS_DIR.joinpath(*parts)


def franka_description_dir() -> Path:
    if FRANKA_DESCRIPTION_DIR.exists():
        return FRANKA_DESCRIPTION_DIR
    return LEGACY_FRANKA_DIR


def franka_urdf_path(*parts: str) -> Path:
    return franka_description_dir().joinpath("urdfs", *parts)
