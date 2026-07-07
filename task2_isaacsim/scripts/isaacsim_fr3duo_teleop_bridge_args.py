# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Scene-agnostic argparse options shared by the Task 2 teleop bridge scripts.

Import-safe before SimulationApp is created (no Isaac Sim imports here).
"""

from __future__ import annotations

import argparse


def add_common_bridge_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--embodiment",
        default="fr3duo_mobile",
        help="Embodiment key under task1_isaacsim/assets/embodiments.",
    )
    parser.add_argument(
        "--franka-root",
        default="/workspace/EBiM_Challenge/task1_isaacsim",
        help="Task 1 root (containing assets/embodiments) inside the container.",
    )
    parser.add_argument(
        "--disable-browser-command-topics",
        action="store_true",
        help="Do not subscribe to /isaac/browser/* command topics.",
    )
    parser.add_argument("--ros-publish-rate", type=float, default=60.0)
    parser.add_argument(
        "--pedal-linear-speed",
        type=float,
        default=0.5,
        help="Base lateral translation speed in m/s used for pedal A/B commands.",
    )
    parser.add_argument(
        "--pedal-angular-speed",
        type=float,
        default=1.2,
        help="Base yaw speed in rad/s used for pedal A+C/B+C commands.",
    )
    parser.add_argument(
        "--pedal-timeout",
        type=float,
        default=1.0,
        help="Seconds without a new /pedal/state message before forcing the base command to NONE.",
    )
    parser.add_argument(
        "--spine-keyboard-control",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use keyboard Up/Down arrows to command franka_spine_vertical_joint height.",
    )
    parser.add_argument(
        "--spine-keyboard-step",
        type=float,
        default=0.01,
        help="Height target increment in meters for each Up/Down key press or repeat.",
    )
    parser.add_argument(
        "--spine-keyboard-min",
        type=float,
        default=-0.05,
        help="Minimum franka_spine_vertical_joint target in meters for keyboard control.",
    )
    parser.add_argument(
        "--spine-keyboard-max",
        type=float,
        default=0.50,
        help="Maximum franka_spine_vertical_joint target in meters for keyboard control.",
    )
    parser.add_argument("--physics-hz", type=float, default=240.0)
    parser.add_argument("--render-hz", type=float, default=60.0)
    parser.add_argument(
        "--configure-base-drives",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Author Task 1 actuator gains on the base drives (steer 500/50, wheel 0/5). "
        "Wheel joints need zero position stiffness for velocity control.",
    )
    parser.add_argument(
        "--apply-gripper-coupled-targets",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Also command the coupled Robotiq linkage joints (driver target x multiplier). "
        "Not needed for the default robot USD: its linkage joints carry "
        "PhysxMimicJointAPI, so PhysX couples them to the driver natively.",
    )
    parser.add_argument(
        "--disable-embedded-omnigraph",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Deactivate OmniGraph action graphs embedded in the robot USD "
        "(ROS_JointStates / Steer_joint_Controller); they duplicate this bridge "
        "and their script node crashes plain Isaac Sim.",
    )
    parser.add_argument("--headless", action="store_true")
