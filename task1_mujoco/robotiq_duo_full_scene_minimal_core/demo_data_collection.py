"""Minimal, self-contained demo of the data-collection pipeline.

Drives the robot with a simple scripted motion (no human input device
needed) and records it into a LeRobotDataset, so the whole record path
can be exercised and verified without a keyboard/gamepad/VR session.
See DATA_COLLECTION.md for the full recording/training/rollout workflow;
this script only covers recording.

Requires the `lerobot` package (`pip install lerobot`) - not part of
environment.yml, see DATA_COLLECTION.md for why.

Usage:
    python demo_data_collection.py --output /path/to/dataset
    python demo_data_collection.py --output /path/to/dataset --episodes 5 --steps 50
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=str, required=True, help="dataset output directory")
    parser.add_argument("--episodes", type=int, default=3, help="number of demo episodes to record")
    parser.add_argument("--steps", type=int, default=30, help="physics steps per episode")
    parser.add_argument(
        "--fps", type=float, default=10.0, help="recorded frame rate (independent of the physics rate)"
    )
    parser.add_argument(
        "--cameras",
        type=str,
        default="head_cam,left_wrist_cam,right_wrist_cam",
        help="comma-separated camera names to record",
    )
    parser.add_argument("--overwrite", action="store_true", help="delete --output first if it exists")
    args = parser.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from teleop.cli import build_desktop_parser
    from teleop.data_record import EpisodeRecorder
    from teleop.robot_arm import hard_hold_arm
    from teleop.session import TeleopSession

    out = Path(args.output).resolve()
    if out.exists():
        if not args.overwrite:
            raise SystemExit(f"{out} already exists - pass --overwrite to replace it")
        shutil.rmtree(out)

    session_args = build_desktop_parser().parse_args(
        # ["--input", "keyboard", "--no-viewer", "--start-at-board"]
        ["--input", "keyboard"]
    )
    session = TeleopSession(session_args)
    model, data = session.model, session.data

    recorder = EpisodeRecorder(
        model,
        session,
        str(out),
        task="scripted data-collection demo",
        record_fps=args.fps,
        camera_names=args.cameras,
    )

    for episode in range(args.episodes):
        recorder.toggle()  # start
        for step in range(args.steps):
            # Placeholder motion so there is something to record - replace
            # this block with a real trajectory source (a planned motion,
            # a leader-arm reader, a policy, etc.) for actual use.
            for arm in session.arms.values():
                hard_hold_arm(model, data, arm)
            session.base_driver.drive(
                0.05 * np.sin(0.3 * step + episode), 0.0, 0.02 * np.cos(0.2 * step), 0.0, model.opt.timestep
            )
            session.step_once(model.opt.timestep)
            recorder.maybe_record(data, 1.0 / args.fps)
        recorder.toggle()  # stop + save
        print(f"episode {episode} recorded")

    recorder.close()
    print(f"dataset saved at {out}")


if __name__ == "__main__":
    main()
