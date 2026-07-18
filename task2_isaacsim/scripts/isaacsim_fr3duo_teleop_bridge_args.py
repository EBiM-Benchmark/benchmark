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
        help="Task 1 root (containing assets/embodiments) inside the "
        "container.",
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
        help="Base lateral translation speed in m/s used for pedal A/B "
        "commands.",
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
        help="Seconds without a new /pedal/state message before forcing "
        "the base command to NONE.",
    )
    parser.add_argument(
        "--spine-keyboard-control",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use keyboard Up/Down arrows to command "
        "franka_spine_vertical_joint height.",
    )
    parser.add_argument(
        "--spine-keyboard-step",
        type=float,
        default=0.01,
        help="Height target increment in meters for each Up/Down key "
        "press or repeat.",
    )
    parser.add_argument(
        "--spine-keyboard-min",
        type=float,
        default=-0.05,
        help="Minimum franka_spine_vertical_joint target in meters for "
        "keyboard control.",
    )
    parser.add_argument(
        "--spine-keyboard-max",
        type=float,
        default=0.50,
        help="Maximum franka_spine_vertical_joint target in meters for "
        "keyboard control.",
    )
    parser.add_argument(
        "--arm-keyboard-teleop",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Drive both arm end effectors with the Kit-window keyboard "
        "through dual RMPflow. While active, ROS arm and gripper "
        "commands are NOT applied (joint states are still published).",
    )
    parser.add_argument(
        "--arm-teleop-linear-speed",
        type=float,
        default=0.18,
        help="End-effector translation speed in m/s while a move key is held.",
    )
    parser.add_argument(
        "--arm-teleop-angular-speed-deg",
        type=float,
        default=60.0,
        help="End-effector rotation speed in deg/s while a rotate key is "
        "held.",
    )
    parser.add_argument(
        "--arm-teleop-gripper-open",
        type=float,
        default=0.0,
        help="Gripper driver joint position in radians for the open state "
        "of the keyboard gripper toggle.",
    )
    parser.add_argument(
        "--arm-teleop-gripper-closed",
        type=float,
        default=0.8,
        help="Gripper driver joint position in radians for the closed "
        "state of the keyboard gripper toggle.",
    )
    parser.add_argument("--physics-hz", type=float, default=240.0)
    parser.add_argument("--render-hz", type=float, default=60.0)
    parser.add_argument(
        "--configure-base-drives",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Author Task 1 actuator gains on the base drives "
        "(steer 500/50, wheel 0/5). "
        "Wheel joints need zero position stiffness for velocity control.",
    )
    parser.add_argument(
        "--apply-gripper-coupled-targets",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Also command the coupled Robotiq linkage joints "
        "(driver target x multiplier). "
        "Not needed for the default robot USD: its linkage joints carry "
        "PhysxMimicJointAPI, so PhysX couples them to the driver natively.",
    )
    parser.add_argument(
        "--disable-embedded-omnigraph",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Deactivate OmniGraph action graphs embedded in the robot USD "
        "(ROS_JointStates / Steer_joint_Controller); they duplicate this "
        "bridge "
        "and their script node crashes plain Isaac Sim.",
    )
    parser.add_argument("--headless", action="store_true")
    _add_recording_args(parser)


def _add_recording_args(parser: argparse.ArgumentParser) -> None:
    """Demonstration-recording options (see task2_isaacsim/README.md and
    services/recording/record_task2.py)."""
    parser.add_argument(
        "--record",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Convenience switch: enables --publish-recording-topics, "
        "--enable-robot-cameras, --publish-ground-truth, and "
        "--scene-reset-hotkey for a demonstration-recording session.",
    )
    parser.add_argument(
        "--publish-recording-topics",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Publish the recording streams (applied joint commands, "
        "odometry, applied base twist, EE poses — names from "
        "config/topics.yaml), all stamped with simulation time.",
    )
    parser.add_argument(
        "--enable-robot-cameras",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Publish the head + wrist robot cameras and /clock over ROS 2 "
        "(OmniGraph render products on the Camera prims authored in the "
        "robot USD).",
    )
    parser.add_argument(
        "--robot-camera-depth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Also publish a depth topic per robot camera.",
    )
    parser.add_argument(
        "--robot-camera-frame-skip",
        type=int,
        default=0,
        help="Render frames skipped between camera messages (0 publishes "
        "every render frame; 1 halves the publish rate).",
    )
    parser.add_argument(
        "--publish-ground-truth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Publish task-object world poses (/isaac/task2/object_poses) "
        "and deformed thermal-pad vertices (/isaac/task2/pad_points).",
    )
    parser.add_argument(
        "--ground-truth-pad-every",
        type=int,
        default=6,
        help="Publish the thermal-pad vertices every N loop iterations "
        "(6 = 10 Hz at the default 60 Hz render rate; 0 disables).",
    )
    parser.add_argument(
        "--scene-reset-hotkey",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable the '5' key in the Isaac Sim window to reset (and "
        "optionally randomize) the task objects between episodes.",
    )
    parser.add_argument(
        "--randomize-objects",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Randomize the task-object spawn poses on each scene reset "
        "(the thermal pad and its sticker base move as one group).",
    )
    parser.add_argument(
        "--randomize-xy-cm",
        type=float,
        default=2.0,
        help="Max +/- XY spawn jitter in centimeters for --randomize-objects.",
    )
    parser.add_argument(
        "--randomize-yaw-deg",
        type=float,
        default=10.0,
        help="Max +/- yaw spawn jitter in degrees for --randomize-objects.",
    )


def resolve_recording_flags(args) -> None:
    """Fold the --record convenience switch into the individual flags."""
    if getattr(args, "record", False):
        args.publish_recording_topics = True
        args.enable_robot_cameras = True
        args.publish_ground_truth = True
        args.scene_reset_hotkey = True
