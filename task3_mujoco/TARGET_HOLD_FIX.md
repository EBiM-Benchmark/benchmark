# Bounded arm targets and stable dynamic spine hold

## Spine target calculation

While a spine key is held, the position target is always computed from the
current measured spine position:

```python
spine_target = q_measured + command * spine_speed * spine_target_lookahead
```

With the defaults, the target can be at most 18 mm ahead of the measured
position. It does not integrate wall-clock loop time and cannot build up a
backlog as the key remains held.

When the key is released, the controller enters a short velocity-aware braking
phase. Its target remains relative to the measured position and velocity:

```python
spine_target = q_measured + qvel_measured * release_brake_lookahead
```

The controller detects the first zero-velocity crossing and latches a fixed
physical hold reference before the spine can reverse direction. While idle, a
bounded algebraic load offset is calculated from the current MuJoCo constraint
and passive generalized forces:

```python
load_offset = -(qfrc_constraint + qfrc_passive) / spine_kp
spine_target = spine_hold_reference + clip(load_offset, -0.010, 0.010)
```

This compensates the static load transmitted through the arm/gripper welds
without integrating position error or key-press duration.

## Arm target calculation

Cartesian IK returns a desired joint velocity `qdot`, but the arm uses position
actuators. The target is therefore a short bounded look-ahead from the current
measured joint positions:

```python
target_delta = clip(qdot * 0.030, -0.040, 0.040)
q_ref = q_measured + target_delta
```

The target is rebuilt from measured `q` every cycle, rather than from the
previous `q_ref`, so no command backlog accumulates. The 40 mrad per-joint cap
also prevents an IK transient or loaded joint from creating a large position
servo impulse.

Different joints can still move at different velocities because the Jacobian
solution distributes motion according to the current arm configuration. This
is required for correct TCP motion and is not target accumulation.
