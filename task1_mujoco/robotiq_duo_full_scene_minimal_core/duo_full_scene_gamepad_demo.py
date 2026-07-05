"""DEPRECATED entry shim — the code moved into the ``teleop/`` package.

Use ``python main.py [--input keyboard|gamepad|vr] ...`` instead. This file
only keeps old commands and READMEs working, e.g.::

    python duo_full_scene_gamepad_demo.py --gamepad
    python duo_full_scene_gamepad_demo.py --input vr
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from main import main  # noqa: E402

if __name__ == "__main__":
    print(
        "[deprecated] this shim forwards to: python main.py " + " ".join(sys.argv[1:]),
        flush=True,
    )
    main()
