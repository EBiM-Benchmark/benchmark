# Teleop ROS 2 publishers (keyboard / gamepad / VR)

Standalone ROS 2 input publishers extending the official EBiM
[teleoperation](https://github.com/EBiM-Benchmark/teleoperation) framework's
pattern (one package per device, publisher/subscriber split) to the devices
the MuJoCo task-1 simulator supports. Intended for contribution to that
repository. GELLO and the foot pedal are NOT here — the official
`franka_gello_state_publisher` / `pedal_state_publisher` already cover them.

## Topic contract (all names centralized, easy to rename later)

| Topic | Type | Content |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | mobile base, REP-103 base_link frame (x fwd, y left, angular.z yaw); `linear.z` repurposed as the spine (vertical lift) rate. Matches the EBiM_Challenge Isaac test commands. |
| `/left/teleop_cmd`, `/right/teleop_cmd` | `geometry_msgs/Twist` | per-arm Cartesian TCP twist (m/s, rad/s, world frame) |
| `/left/gripper_cmd`, `/right/gripper_cmd` | `std_msgs/Float32` | >0.5 = close intent, <=0.5 = open |
| `/mujoco/teleop_feedback` | `std_msgs/Float32MultiArray` | published BY the simulator: `[viewer_cam_azimuth_deg, robot_yaw_rad]` for screen-relative device mapping. Information only — **all IK and physics stay in the simulator**; publishers never solve IK. |

The consumers (the MuJoCo sim's `--input ros_teleop`, or a real-robot
controller) apply these through their own control stack, so teleoperation
feel is identical to driving the devices locally.

## Packages

- `keyboard_teleop_publisher` — full arm+base keyboard semantics of the
  MuJoCo task-1 sim (arrow-key cluster, screen-relative base driving,
  `7/8/9` mode select, `R` rotate toggle, `G`/`V` gripper, `-`/`=` speed).
  Distinct from the official `keyboard_state_publisher`, which is the
  W/A/S/D/Q/E base-only keyboard of the GELLO workflow. Linux/X11 only
  (needs `python-xlib` for held-key polling).
- `gamepad_teleop_publisher` — SDL GameController layout (identical mapping
  on every OS/vendor), `Share`/`L1`/`R1` mode select, sticks/triggers,
  stick-click speed levels. Needs `pygame`.
- VR publisher: planned, not yet implemented — it additionally needs a
  haptic-feedback topic (sim contact forces -> controller rumble) designed
  and a headset to verify against, so it is deferred rather than shipped
  untested.

Both publishers accept `--pattern` (publish a scripted test sequence for a
few seconds, no device needed) for CI/self-testing, and both hold safe
zeros/stop when their device disappears.

## Build & run

```bash
cd teleop_ros2
colcon build
source install/setup.bash
ros2 run keyboard_teleop_publisher keyboard_teleop_publisher
ros2 run gamepad_teleop_publisher gamepad_teleop_publisher
```
