#!/usr/bin/env python3
"""One-click start (Windows / WSL2 / Linux): creates the conda environment on
first run, then launches the teleop. Any flag is passed straight through:

    python start.py                          # keyboard
    python start.py --input gamepad
    python start.py --input vr
    python start.py --input vr --mnet

Only prerequisite: a conda/miniconda install on PATH (https://conda.io).
After the first run this is equivalent to `conda run -n duo-teleop python
main.py ...`.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ENV_NAME = "duo-teleop"

# import-name probes for the pip requirements in environment.yml; anything
# not listed probes under its requirement name (with - mapped to _)
_IMPORT_NAMES = {
    "pillow": "PIL",
    "pyopenxr": "xr",
    "pyopengl": "OpenGL",
    "python-xlib": "Xlib",
    "opencv-python": "cv2",
}


def _pip_requirements() -> list[str]:
    """The pip requirement lines from environment.yml (tiny purpose-built
    parser — the interpreter running this script may lack PyYAML)."""
    reqs: list[str] = []
    pip_indent = None
    for raw in (HERE / "environment.yml").read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        if line.strip() == "- pip:":
            pip_indent = indent
            continue
        if pip_indent is not None:
            if indent <= pip_indent:
                pip_indent = None
                continue
            item = line.strip()
            if item.startswith("- "):
                reqs.append(item[2:].strip())
    return reqs


def _platform_ok(marker: str) -> bool:
    # environment.yml only uses sys_platform markers
    if "sys_platform" not in marker:
        return True
    plat = sys.platform if sys.platform in ("win32", "darwin") else "linux"
    return f'"{plat}"' in marker or f"'{plat}'" in marker


def _heal_missing_deps(conda: str) -> None:
    """Probe every pip dependency of the EXISTING env (importlib.find_spec,
    no heavy imports — sub-second) and install whatever is missing. An env
    created before a dependency entered environment.yml would otherwise
    stay broken forever, since env creation is skipped when it exists."""
    wanted: dict[str, str] = {}  # import name -> full requirement line
    for req in _pip_requirements():
        spec, _, marker = req.partition(";")
        if marker and not _platform_ok(marker):
            continue
        name = re.split(r"[<>=!\[ ]", spec.strip(), maxsplit=1)[0]
        wanted[_IMPORT_NAMES.get(name.lower(), name.replace("-", "_"))] = req
    if not wanted:
        return
    probe = subprocess.run(
        [
            conda,
            "run",
            "-n",
            ENV_NAME,
            "python",
            "-c",
            "import importlib.util,sys;print(' '.join(m for m in sys.argv[1:] if importlib.util.find_spec(m) is None))",
            *wanted,
        ],
        capture_output=True,
        text=True,
    )
    missing = probe.stdout.split() if probe.returncode == 0 else []
    if not missing:
        return
    reqs = [wanted[m] for m in missing]
    print(
        f"[start] env is missing {', '.join(missing)} - installing ({', '.join(reqs)})...",
        flush=True,
    )
    subprocess.run(
        [conda, "run", "-n", ENV_NAME, "pip", "install", *reqs],
        check=False,
    )


def main() -> None:
    conda = shutil.which("conda")
    if conda is None:
        sys.exit(
            "conda not found on PATH. Install Miniconda first:\n"
            "  https://docs.conda.io/en/latest/miniconda.html\n"
            "then reopen this terminal and run start.py again."
        )

    envs = subprocess.run([conda, "env", "list"], capture_output=True, text=True).stdout
    if not any(line.split()[:1] == [ENV_NAME] for line in envs.splitlines() if line.strip()):
        print(
            f"[start] first run: creating conda env '{ENV_NAME}' (a few minutes)...",
            flush=True,
        )
        subprocess.run(
            [conda, "env", "create", "-f", str(HERE / "environment.yml")],
            check=True,
        )
    else:
        _heal_missing_deps(conda)

    cmd = [
        conda,
        "run",
        "--live-stream",
        "-n",
        ENV_NAME,
        "python",
        str(HERE / "main.py"),
        *sys.argv[1:],
    ]
    raise SystemExit(subprocess.run(cmd, cwd=HERE).returncode)


if __name__ == "__main__":
    main()
