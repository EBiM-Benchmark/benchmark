# Exact small-model mesh transfer

The package now uses the standalone model's `spoon-mesh / gripper-mesh` configuration rather than the temporary flat-pad and single-handle-box approximation.

## Transferred without geometric substitution

- Original spoon visual mesh.
- Fourteen original spoon-handle collision mesh sections.
- Original small-model spoon neck and bowl collision parts.
- Original Robotiq collision STL meshes on the complete gripper.
- Original fingertip STL meshes as the dedicated high-friction spoon-contact surfaces.
- No runtime rotation or replacement of the fingertip contact geometry.

## Exact internal gripper parameters

```text
joint armature:       0.01
joint damping:        1.0
joint frictionloss:   0.1
actuator kp/kv:       500 / 30
actuator force range: -100 to 100
closing target:       0.735 rad
mimic solref:         0.006 1
mimic solimp:         0.995 0.9999 0.0001 0.5 2
```

## Exact grasp contact parameters

```text
condim:     6
friction:   2.5 0.10 0.03
margin:     0.00025
solref:     0.010 1
solimp:     0.95 0.99 0.001 0.5 2
```

## Exact global solver parameters

```text
timestep:       0.001 s
integrator:     implicitfast
solver:         Newton
iterations:     100
ls_iterations:  20
friction cone:  elliptic
impratio:       10
```

## Full-robot adaptation

The standalone gripper is held by a mocap weld and therefore has no articulated-arm deflection. In the complete mobile robot, both FR3 arms use stiff persistent position servos so fingertip contact does not shift during closure. The gripper bases remain rigidly attached to the flanges, and their poses are synchronized exactly during kinematic spine motion.

The table, spoon, bowl, plate, and beans were relocated to reproduce the small model's working settled geometry while keeping the original room and task components.
