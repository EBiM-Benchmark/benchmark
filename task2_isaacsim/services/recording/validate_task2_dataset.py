#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Sanity-check a Task 2 LeRobot dataset recorded by record_task2.py.

Needs only numpy (pandas+pyarrow optional, for per-frame checks on the
parquet files). Run on the host:

    python task2_isaacsim/services/recording/validate_task2_dataset.py \
        task2_isaacsim/dataset/task2_thermalpad_v1
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ACTION_DIM = 20
STATE_DIM = 37
EXPECTED_VIDEO_KEYS = {
    "observation.images.head": (720, 1280, 3),
    "observation.images.wrist_left": (480, 848, 3),
    "observation.images.wrist_right": (480, 848, 3),
    "observation.images.eval_camera": (720, 1280, 3),
}


def fail(message: str) -> None:
    print(f"  ✗ {message}")
    fail.count += 1


fail.count = 0


def ok(message: str) -> None:
    print(f"  ✓ {message}")


def check_info(root: Path) -> dict:
    info_path = root / "meta" / "info.json"
    if not info_path.is_file():
        fail(f"missing {info_path}")
        return {}
    info = json.loads(info_path.read_text())
    features = info.get("features", {})

    action = features.get("action", {})
    if tuple(action.get("shape", ())) == (ACTION_DIM,):
        ok(f"action shape ({ACTION_DIM},)")
    else:
        fail(f"action shape {action.get('shape')} != ({ACTION_DIM},)")

    state = features.get("observation.state", {})
    if tuple(state.get("shape", ())) == (STATE_DIM,):
        ok(f"observation.state shape ({STATE_DIM},)")
    else:
        fail(f"observation.state shape {state.get('shape')} != ({STATE_DIM},)")

    for key, shape in EXPECTED_VIDEO_KEYS.items():
        feature = features.get(key)
        if feature is None:
            print(f"  - {key} not recorded (camera subset)")
            continue
        if tuple(feature.get("shape", ())) == shape:
            ok(f"{key} shape {shape}")
        else:
            fail(f"{key} shape {feature.get('shape')} != {shape}")

    print(
        f"  info: fps={info.get('fps')}, "
        f"episodes={info.get('total_episodes')}, "
        f"frames={info.get('total_frames')}, robot={info.get('robot_type')}"
    )
    return info


def check_extras(root: Path, info: dict) -> None:
    extras_dir = root / "task2_extras"
    meta_path = extras_dir / "episodes_task2.jsonl"
    if not meta_path.is_file():
        fail(f"missing {meta_path}")
        return
    lines = [
        json.loads(line)
        for line in meta_path.read_text().splitlines()
        if line.strip()
    ]
    total_episodes = info.get("total_episodes")
    if total_episodes is not None and len(lines) != total_episodes:
        fail(
            f"episodes_task2.jsonl has {len(lines)} entries, dataset has "
            f"{total_episodes} episodes"
        )
    else:
        ok(f"episode metadata entries: {len(lines)}")

    successes = sum(1 for line in lines if line.get("success"))
    print(f"  success labels: {successes}/{len(lines)} successful")

    for line in lines:
        index = line["episode_index"]
        npz_path = extras_dir / line.get(
            "extras_file", f"episode_{index:06d}.npz"
        )
        if not npz_path.is_file():
            fail(f"missing extras file {npz_path.name}")
            continue
        with np.load(npz_path, allow_pickle=False) as data:
            sim_time = data["sim_time"]
            if sim_time.size != line.get("frames"):
                fail(
                    f"{npz_path.name}: sim_time length {sim_time.size} != "
                    f"frames {line.get('frames')}"
                )
            deltas = np.diff(sim_time)
            if sim_time.size > 1:
                if np.any(deltas <= 0):
                    fail(f"{npz_path.name}: sim_time not monotonic")
                else:
                    fps = line.get("fps_sim") or 30
                    jitter = np.abs(deltas - 1.0 / fps) * fps * 100.0
                    worst = float(jitter.max())
                    if worst > 25.0:
                        fail(
                            f"{npz_path.name}: sim-time jitter up to "
                            f"{worst:.1f}% of the sample period"
                        )
                    else:
                        ok(
                            f"episode {index}: {sim_time.size} frames, "
                            f"max jitter {worst:.1f}%"
                        )
            if "object_poses" in data:
                poses = data["object_poses"]
                if poses.ndim != 3 or poses.shape[2] != 7:
                    fail(f"{npz_path.name}: object_poses shape {poses.shape}")
            if "pad_points" in data:
                points = data["pad_points"]
                if points.ndim != 3 or points.shape[2] != 3:
                    fail(f"{npz_path.name}: pad_points shape {points.shape}")
                else:
                    print(
                        f"    episode {index}: pad_points "
                        f"{points.shape[0]} snapshots x {points.shape[1]} "
                        "vertices"
                    )


def check_frames(root: Path) -> None:
    try:
        import pandas as pd  # noqa: PLC0415
    except ImportError:
        print("  - pandas/pyarrow not installed; skipping per-frame checks")
        return
    parquet_files = sorted(root.glob("data/**/*.parquet"))
    if not parquet_files:
        fail("no parquet data files found")
        return
    frame = pd.read_parquet(parquet_files[0])
    action = np.stack(frame["action"].to_numpy())
    state = np.stack(frame["observation.state"].to_numpy())
    if action.shape[1] != ACTION_DIM:
        fail(f"parquet action dim {action.shape[1]} != {ACTION_DIM}")
    if state.shape[1] != STATE_DIM:
        fail(f"parquet state dim {state.shape[1]} != {STATE_DIM}")
    nan_actions = int(np.isnan(action).any(axis=1).sum())
    nan_states = int(np.isnan(state).any(axis=1).sum())
    if nan_actions or nan_states:
        fail(
            f"NaNs present: {nan_actions} action rows, {nan_states} state "
            "rows (topic missing during recording?)"
        )
    else:
        ok(f"first episode: {len(frame)} frames, no NaNs")
    grippers = action[:, 17:19]
    if grippers.min() < -1e-6 or grippers.max() > 1.0 + 1e-6:
        fail("gripper action outside [0, 1]")
    else:
        ok(
            "gripper action range "
            f"[{grippers.min():.2f}, {grippers.max():.2f}]"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_root", type=Path)
    args = parser.parse_args()
    root = args.dataset_root
    if not root.is_dir():
        print(f"Dataset root not found: {root}")
        return 2

    print(f"Validating {root}")
    print("info.json:")
    info = check_info(root)
    print("task2_extras:")
    check_extras(root, info)
    print("frames:")
    check_frames(root)

    if fail.count:
        print(f"\n{fail.count} problem(s) found.")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
