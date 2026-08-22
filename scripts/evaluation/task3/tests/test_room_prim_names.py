#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Assert every prim name Task 3 grading looks up exists in the room USD.

The Python grading helpers address room objects by bare prim name. The room
USD is a Git-LFS binary that is re-exported from a modelling tool, so a
re-export can silently rename a prim and break scoring with no test failure:
the pure grading tests feed synthetic dictionaries keyed by the same names,
so they keep passing, and no CI job opens a USD.

That happened in commit e36119c, which renamed ``spoon2`` -> ``spoon2_01``
and ``plate2`` -> ``plate2_01``. Stages 1, 2 and 4 stopped scoring and the
Isaac Sim integration runner produced no output at all while still exiting 0.

This check closes that gap. It needs ``pxr`` but not a running simulator, so
it is cheap enough for CI:

    docker exec isaac-sim-5-1-0-workshop bash -lc \
      'cd /workspace/EBiM_Challenge && python -B \
       scripts/evaluation/task3/tests/test_room_prim_names.py'

Exits 0 when every name resolves, 1 otherwise, and suggests the likely
replacement for anything missing.
"""

from __future__ import annotations

import difflib
import sys
from pathlib import Path
from typing import Any

TASK3_EVAL_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = TASK3_EVAL_DIR.parents[2]
if str(TASK3_EVAL_DIR) not in sys.path:
    sys.path.insert(0, str(TASK3_EVAL_DIR))

from grading import (  # noqa: E402
    DEFAULT_STAGE1_OBJECTS,
    DEFAULT_UTENSIL_OBJECTS,
)

ROOM_USD = REPO_ROOT / "assets" / "robot_room.usd"

# Names resolved outside the graded-object lists: the fed person, the bean
# recycling container, and the sink region. eval_overlay.py raises on startup
# if any of these is missing.
INFRASTRUCTURE_PRIMS = ("head", "ikea_knock_box", "sink_boundary")


def room_child_names(usd_path: Path) -> list[str]:
    """Return the names of the room USD's top-level objects."""
    from pxr import Usd as pxr_usd

    # pxr.Usd builds its namespace at import time through
    # Tf.PreparePythonModule(), so static analysers see an empty module and
    # flag every attribute (Usd.Stage -> reportAttributeAccessIssue). Erasing
    # the type is the convention already used in scripts/scenes/.
    Usd: Any = pxr_usd

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"Could not open room USD: {usd_path}")

    root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    return [child.GetName() for child in root.GetChildren()]


def main() -> int:
    room_usd = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOM_USD
    if not room_usd.is_file():
        print(f"FAIL room USD not found: {room_usd}")
        print("     Git LFS may not be installed, or was not smudged.")
        return 1

    present = room_child_names(room_usd)
    required = (
        *INFRASTRUCTURE_PRIMS,
        *dict.fromkeys((*DEFAULT_STAGE1_OBJECTS, *DEFAULT_UTENSIL_OBJECTS)),
    )

    missing = []
    for name in required:
        resolved = any(
            c == name
            or (c.startswith(f"{name}_") and c[len(name) + 1 :].isdigit())
            for c in present
        )

        if resolved:
            print(f"  ok      {name}")
            continue
        missing.append(name)
        close = difflib.get_close_matches(name, present, n=1, cutoff=0.6)
        hint = f"  did you mean {close[0]!r}?" if close else ""
        print(f"  MISSING {name}{hint}")

    print()
    if missing:
        print(
            f"FAIL {len(missing)} of {len(required)} Task 3 prim names do not "
            f"resolve in {room_usd.name}."
        )
        print(
            "     Grading silently under-scores when this happens; the Isaac "
            "Sim integration runner emits nothing and still exits 0."
        )
        return 1

    print(
        f"PASS all {len(required)} Task 3 prim names resolve in "
        f"{room_usd.name}."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
