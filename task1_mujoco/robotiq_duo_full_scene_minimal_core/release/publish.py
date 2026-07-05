#!/usr/bin/env python3
"""One-click release update: run this after development to refresh EVERY
release artifact from the current source tree.

    python release/publish.py [--obfuscate [--cross]] [--skip-docker] [--skip-zip]

Steps (each skippable, failures stop the run):
  1. smoke gate  - headless checks of keyboard / VR / randomization paths
  2. user bundle - plain-source zip via build_release.py (default); with
                   --obfuscate a PyArmor-protected one instead (--cross =>
                   one obfuscated bundle for windows.x86_64 + linux.x86_64)
  3. docker      - rebuild the duo-teleop-eval image (skipped with a note if
                   docker is not installed)

Artifacts:
  build/duo_teleop_<git-describe>.zip     -> attach to a GitHub Release
  duo-teleop-eval:latest                  -> `docker tag` + push to a registry
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SMOKES = [
    ["--no-viewer"],
    ["--input", "vr", "--no-viewer"],
    ["--no-viewer", "--randomize-board", "--randomize-seed", "1"],
    ["--mnet", "--no-viewer"],
]


def run(cmd: list[str], cwd: Path = ROOT) -> None:
    print("+", " ".join(str(c) for c in cmd), flush=True)
    subprocess.run(cmd, check=True, cwd=cwd)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--obfuscate",
        action="store_true",
        help="PyArmor-protect the bundle (default: plain source)",
    )
    parser.add_argument(
        "--cross",
        action="store_true",
        help="with --obfuscate: windows+linux bundle (needs pyarmor.cli.runtime)",
    )
    parser.add_argument("--skip-docker", action="store_true")
    parser.add_argument("--skip-zip", action="store_true")
    args = parser.parse_args()

    print("=== 1/3 smoke gate ===", flush=True)
    for smoke in SMOKES:
        run([sys.executable, "main.py", *smoke])

    if not args.skip_zip:
        print("=== 2/3 user bundle ===", flush=True)
        cmd = [sys.executable, "release/build_release.py"]
        if args.obfuscate:
            cmd.append("--obfuscate")
            if args.cross:
                cmd.append("--cross")
        run(cmd)

    if not args.skip_docker:
        print("=== 3/3 docker eval image ===", flush=True)
        if shutil.which("docker") is None:
            print(
                "[publish] docker not installed here - skipped (run this step on the WSL2/Linux box)",
                flush=True,
            )
        else:
            run(["docker", "compose", "-f", "release/compose.yaml", "build"])

    print("\n[publish] done. artifacts:", flush=True)
    for z in sorted((ROOT / "build").glob("duo_teleop_*.zip")):
        print("  ", z, flush=True)


if __name__ == "__main__":
    main()
