#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent

SCENES = {
    (100, "mesh"): HERE / "final_scene_100_beans_mesh.xml",
    (300, "mesh"): HERE / "final_scene_300_beans_mesh.xml",
    (100, "primitives"): HERE / "final_scene_100_beans_primitives.xml",
    (300, "primitives"): HERE / "final_scene_300_beans_primitives.xml",
}

RUNTIME_SCENES = {
    (100, "mesh"): HERE / "final_scene_100_beans_mesh_spoon_mocap_runtime.xml",
    (300, "mesh"): HERE / "final_scene_300_beans_mesh_spoon_mocap_runtime.xml",
    (100, "primitives"): HERE / "final_scene_100_beans_primitives_spoon_mocap_runtime.xml",
    (300, "primitives"): HERE / "final_scene_300_beans_primitives_spoon_mocap_runtime.xml",
}


# EBiM integration addition (not upstream): the 20 visual meshes/textures above
# the repository's 2 MB file limit are fetched separately, so a fresh clone can
# be missing them. MuJoCo would otherwise fail deep in model compilation with a
# bare "Error opening file '...obj'". Fail fast with the fix instead.
DOWNLOAD_SCRIPT = HERE / "scripts" / "download_large_assets.sh"


def check_large_assets() -> None:
    manifest = HERE / "scripts" / "large_assets.txt"
    if not manifest.is_file():
        return
    missing = [
        rel
        for rel in manifest.read_text().split()
        if not (HERE / rel).is_file()
    ]
    if not missing:
        return
    listing = "\n".join(f"  {rel}" for rel in missing)
    raise SystemExit(
        f"Missing {len(missing)} large Task 3 asset(s) not tracked in git:\n"
        f"{listing}\n\n"
        f"Fetch them with:\n  {DOWNLOAD_SCRIPT}\n"
        "See task3_mujoco/README.md for details."
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run Task 3. Defaults: 300 beans, robot teleoperation, and the "
            "exact small-model fingertip STL grasp meshes and segmented mesh spoon on both arms, with stiff flange welds and fixed preload. Unknown arguments are "
            "forwarded to the selected controller."
        )
    )
    parser.add_argument("--beans", type=int, choices=(100, 300), default=300)
    parser.add_argument(
        "--control", "--mode", dest="control",
        choices=("robot", "spoon"), default="robot",
    )
    parser.add_argument(
        "--gripper-collision",
        choices=("mesh", "primitives"),
        default="mesh",
        help=(
            "both options are retained for compatibility; compatibility selector; both scene variants use the exact small-model mesh grasp contact"
        ),
    )
    parser.add_argument("--print-command", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args, extra = parser.parse_known_args()

    if not args.dry_run:
        check_large_assets()

    key = (args.beans, args.gripper_collision)
    scene = SCENES[key]
    if args.control == "robot":
        command = [
            sys.executable,
            str(HERE / "teleop_keyboard.py"),
            "--xml", str(scene),
            "--arm-frame", "base",
            "--arm-rotation-frame", "tcp",
            *extra,
        ]
    else:
        command = [
            sys.executable,
            str(HERE / "attach_spoon_to_mocap.py"),
            "--xml", str(scene),
            "--runtime-xml", str(RUNTIME_SCENES[key]),
            *extra,
        ]

    if args.print_command or args.dry_run:
        print("Running:", " ".join(map(str, command)))
    if not args.dry_run:
        raise SystemExit(subprocess.call(command, cwd=HERE))


if __name__ == "__main__":
    main()
