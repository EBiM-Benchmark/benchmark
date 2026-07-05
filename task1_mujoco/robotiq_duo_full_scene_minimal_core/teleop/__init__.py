"""Duo-FR3 full-scene teleoperation package.

Module map (edit only the layer you are working on):

  config.py          every model name and tunable constant, grouped by topic
  cli.py             command-line arguments, one argparse group per module
  maths.py           pure quaternion / geometry helpers (no MuJoCo state)
  mjutil.py          generic MuJoCo id lookup + debug printing helpers
  robot_arm.py       Arm state, velocity-IK, idle hold, ready pose
  scene.py           model loading, cable layout, spawn placement
  grasping.py        gripper close servo, grasp assist, C-clip guide, haptics
  base_drive.py      mobile-base driving (actuator/jointvel/kinematic/wheel)
  session.py         TeleopSession: owns model/data/arms and the physics step
  input_keyboard.py  keyboard polling (Windows key-state aware)
  input_gamepad.py   pygame joystick wrapper incl. rumble
  run_desktop.py     keyboard + gamepad main loop
  vr_mapping.py      controller hand state, clutch, VR->world/screen frames
  vr_steamvr.py      SteamVR (openvr) input-only backend
  vr_openxr.py       OpenXR backend: threaded session, input, haptics, HMD view
  run_vr.py          VR main loop (both backends)

Entry point is ``main.py`` one directory up: ``python main.py --input vr``.
Input methods are isolated: run_vr/vr_* are only imported when ``--input vr``
is requested, so a broken VR module never takes down keyboard/gamepad mode.
"""

from __future__ import annotations


def log(message: str) -> None:
    """Print immediately (teleop runs with an unbuffered feel even when piped)."""
    print(message, flush=True)
