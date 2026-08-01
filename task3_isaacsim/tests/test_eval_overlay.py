# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Import-safe tests for the Task 3 evaluation overlay."""

import importlib.util
from pathlib import Path

MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "eval_overlay.py"
)
SPEC = importlib.util.spec_from_file_location(
    "task3_eval_overlay", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
overlay = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(overlay)


def test_stage3_anchor_follows_supplied_recovery_region():
    region = overlay.grading.SphereRegion(
        overlay.grading.Point3D(4.0, 5.0, 6.0),
        radius=0.4,
    )

    sink_region = overlay.grading.SinkRegion(
        overlay.grading.Bounds2D(10.0, 20.0, 12.0, 24.0),
        tabletop_z=0.75,
    )
    specs = overlay._stage_anchor_specs((1.0, 2.0, 3.0), region, sink_region)
    stage3 = next(spec for spec in specs if spec[0] == "stage3")
    stage4 = next(spec for spec in specs if spec[0] == "stage4")

    assert stage3[2] == (4.0, 5.0, 6.6)
    assert stage4[2] == (11.0, 22.0, 1.35)
