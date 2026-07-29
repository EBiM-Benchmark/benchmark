# Safer position control and dynamic spine

## Arm controller

Both seven-joint FR3 arms remain position-controlled. Damped IK still computes a desired joint velocity, which is integrated into persistent position references. The previous `kp=80000`, `kv=2200`, `±5000` servos were replaced by moderate joint-dependent gains and FR3-like torque limits:

| Joint | kp | kv | Force/torque limit |
|---|---:|---:|---:|
| 1–2 | 6000 | 180 | ±87 |
| 3–4 | 5000 | 160 | ±87 |
| 5 | 1800 | 70 | ±12 |
| 6 | 1400 | 60 | ±12 |
| 7 | 1000 | 50 | ±12 |

MuJoCo bias forces are applied as feed-forward gravity compensation at every arm DOF.

## Static-wall anti-windup

Static room walls are detected by the `collision_wall_mat` material. When the selected arm or its gripper contacts such a wall, `q_ref` is reset to the measured joint configuration and IK integration is blocked. A latch keeps the arm blocked while the key remains held, even if compliant contact briefly opens. The latch is released only after the key is released and contact clears.

## Dynamic spine

The vertical slide is now driven only by its position actuator. The controller updates the target at `0.18 m/s`, applies spine bias/gravity compensation, and never writes runtime spine `qpos` or `qvel`. The prior before/after-step gripper projection was removed. The stiff equality welds now transmit the dynamically integrated spine motion to both grippers, allowing a grasped spoon to rise with the spine.

Spine actuator: `kp=12000`, `kv=800`, force range `±10000`.

## Safer teleoperation defaults

- Arm translation: `0.25 m/s`
- Arm rotation: `45 deg/s`
- IK damping: `0.03`
- Rotation IK damping: `0.01`
- Seven-dimensional joint-velocity norm limit: `2 rad/s`
- Maximum TCP displacement per physics step: `0.5 mm`
- Maximum rotation per physics step: `1.5 deg`
- Wall-time compensation: disabled by default

## Regression results

- Table-supported arm lift: PASS — spoon lift `0.261220 m`, four 6D contacts per side.
- Dynamic spine lift: PASS — spine `0.215759 m`, spoon `0.215036 m`, relative error `0.000523 m`.
- Static-wall guard: PASS — max `|qacc|=4.123e2`, TCP post-contact span `2.591 mm`, collision latch clears after key release.
- Lowest-point TCP rotation: PASS.
- Dynamic dual-gripper weld: PASS — maximum flange position error `0.000167 mm`.
