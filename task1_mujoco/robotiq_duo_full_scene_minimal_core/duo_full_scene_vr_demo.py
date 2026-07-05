"""DEPRECATED entry shim — the code moved into the ``teleop/`` package.

Use ``python main.py --input vr ...`` instead. This file only keeps old
commands working; it forces VR mode and forwards every other flag.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from main import main  # noqa: E402

if __name__ == "__main__":
    argv = sys.argv[1:]
    if "--input" not in argv:
        argv = ["--input", "vr", *argv]
    print(
        "[deprecated] this shim forwards to: python main.py " + " ".join(argv),
        flush=True,
    )
    main(argv)
