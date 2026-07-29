# Non-accumulating arm and spine targets

## Arm

The arm remains position controlled. Damped IK calculates `qdot`, but the
position target is rebuilt from the current measured joint angles each cycle:

```python
q_ref = q_measured + qdot * arm_target_lookahead
```

The default look-ahead is 0.030 s. Repeated held-key cycles cannot make the
target move progressively farther ahead of a slow or loaded joint.

Joint velocities are not expected to be equal. A Cartesian TCP command requires
different velocities at the seven joints according to the Jacobian. The global
2 rad/s norm limit preserves the IK direction while limiting overall speed.

## Spine

The spine remains dynamically position actuated. Its target is rebuilt from the
current measured height and a bounded lead:

```python
spine_target = spine_q_measured + command * spine_speed * spine_target_lookahead
```

The default look-ahead is 0.100 s, corresponding to an 18 mm maximum lead at
the default 0.18 m/s speed. The target no longer integrates wall-clock loop
time, and releasing the key resets the target to the current measured height.
