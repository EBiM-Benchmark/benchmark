# Deprecated Task 3 direct-keyboard teleop stack

This folder holds the pre-ROS direct-keyboard teleoperation stack for Task 3.
It read the Isaac Sim carb keyboard directly, accumulated Cartesian pose deltas
per arm, and drove the dual FR3 arms through RMPflow / Lula inverse kinematics
while steering the swerve base from the same key map.

It has been superseded by the ROS runtime in
`../scripts/scene_room.py`, which aligns Task 3 with Task 1's adapter pattern:
GELLO joint states flow over `/bridge/*` topics to command the arms, and the
keyboard/pedal input publishes `/pedal/state` to drive the swerve base.

The code is kept here for reference only. It is **not** launched by
`../scripts/run_isaacsim_teleop.sh`, and nothing in the supported runtime
imports it. Its tests remain runnable in place under `tests/` (they resolve
`../common` and `../scene_robot_room_rmpflow.py` relative to this folder).
