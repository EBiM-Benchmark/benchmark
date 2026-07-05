#!/usr/bin/env python3
"""Build the end-user release bundle.

    python release/build_release.py [--obfuscate [--cross]]

Default: PLAIN SOURCE bundle — inherently cross-platform, ready to attach
to a GitHub Release. With --obfuscate the python code is PyArmor-protected
instead (requires `pip install pyarmor`; --cross additionally targets both
windows.x86_64 and linux.x86_64 in one bundle and needs
`pip install pyarmor.cli.runtime`; buy a PyArmor license for public
distribution of obfuscated builds).

Steps:
  1. collect main.py + the shims + the whole teleop/ package
     (plain copy, or PyArmor output with --obfuscate)
  2. add the scene XML, assets, README, environment.yml, start.py
  3. zip: build/duo_teleop_<version>.zip  ->  attach to a GitHub Release

Users unpack and run `python start.py` (creates the conda env on first run).
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent  # the project folder
BUILD = ROOT / "build"

# what ships: obfuscated code + these data files/folders verbatim
# (start.py is the user's one-click launcher and stays readable on purpose)
DATA_FILES = [
    "duo_full_scene_grasp.xml",
    "README.md",
    "environment.yml",
    "start.py",
]
DATA_DIRS = ["assets"]
CODE_ENTRIES = [
    "main.py",
    "duo_full_scene_gamepad_demo.py",
    "duo_full_scene_vr_demo.py",
]


def run(cmd: list[str]) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True, cwd=ROOT)


def version() -> str:
    try:
        out = subprocess.run(
            ["git", "describe", "--always", "--dirty", "--tags"],
            capture_output=True,
            text=True,
            check=True,
            cwd=ROOT,
        ).stdout.strip()
        return out.replace("/", "-")
    except Exception:
        return "dev"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--obfuscate",
        action="store_true",
        help="PyArmor-protect the code instead of shipping plain source (needs: pip install pyarmor)",
    )
    parser.add_argument(
        "--cross",
        action="store_true",
        help="with --obfuscate: one bundle for windows.x86_64 "
        "+ linux.x86_64 (needs pyarmor.cli.runtime); plain "
        "source is cross-platform anyway",
    )
    args = parser.parse_args()

    ver = version()
    stage = BUILD / f"duo_teleop_{ver}"
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True)

    if args.obfuscate:
        cmd = [
            sys.executable,
            "-m",
            "pyarmor.cli",
            "gen",
            "-O",
            str(stage),
            "--recursive",
            "teleop",
            *CODE_ENTRIES,
        ]
        if args.cross:
            cmd[6:6] = ["--platform", "windows.x86_64,linux.x86_64"]
        run(cmd)
    else:
        for entry in CODE_ENTRIES:
            shutil.copy2(ROOT / entry, stage / entry)
        shutil.copytree(
            ROOT / "teleop",
            stage / "teleop",
            ignore=shutil.ignore_patterns("__pycache__"),
        )

    for name in DATA_FILES:
        shutil.copy2(ROOT / name, stage / name)
    for name in DATA_DIRS:
        shutil.copytree(ROOT / name, stage / name)

    zip_path = BUILD / f"duo_teleop_{ver}.zip"
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for p in sorted(stage.rglob("*")):
            if p.is_file():
                zf.write(p, p.relative_to(stage.parent))
    size_mb = zip_path.stat().st_size / 1e6
    print(f"\nrelease bundle: {zip_path}  ({size_mb:.1f} MB)")
    print("smoke it before shipping:")
    print(f"  cd {stage} && python main.py --no-viewer")


if __name__ == "__main__":
    main()
