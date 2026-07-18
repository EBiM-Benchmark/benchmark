# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Sim-side helpers for Task 2 demonstration recording.

camera_publishers -- robot camera (head + wrists) OmniGraph ROS publishers
                     (the sim clock topic is published by the bridge node
                     itself).
scene_capture     -- ground-truth object/pad publishers and the scene
                     reset/randomize hotkey, run as run_teleop_loop tick
                     callbacks.

Both modules import Isaac Sim modules at import time; import them only after
SimulationApp has been created and isaacsim.ros2.bridge is enabled.
Keep this __init__ free of Isaac imports.
"""
