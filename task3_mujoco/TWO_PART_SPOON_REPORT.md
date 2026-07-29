# Two-Part Spoon Structure

The spoon is divided into two named collision bodies while remaining one exactly rigid object.

## Hierarchy

```xml
<body name="dynamic_spoon2" ...>
  <freejoint name="dynamic_spoon2_freejoint"/>
  <inertial .../>
  <geom name="visual_dynamic_spoon2" .../>

  <body name="spoon_handle_neck_part" pos="0 0 0" quat="1 0 0 0">
    <!-- 14 existing stem mesh geoms -->
    <!-- existing neck box -->
  </body>

  <body name="spoon_scoop_part" pos="0 0 0" quat="1 0 0 0">
    <!-- 139 fitted curved scoop boxes -->
  </body>
</body>
```

Neither child body contains a joint or freejoint. MuJoCo therefore treats both child bodies as rigidly fixed to `dynamic_spoon2`; there is no spring, weld compliance, hinge, or relative motion between the handle and scoop.

## Preserved definitions

- The 14 stem collision mesh definitions are unchanged.
- The neck collision box is unchanged.
- The corrected curved scoop contains 139 fitted boxes.
- The spoon parent retains the original free joint, total mass, inertia, visual mesh, initial position and orientation.
- The scoop boxes retain the proven working spoon contact settings:
  `contype=1`, `conaffinity=16`, `condim=6`, `friction="1.0 0.02 0.002"`, `margin=0`, `solref="0.008 1"`, `solimp="0.96 0.995 0.001 0.5 2"`.

## Controller compatibility

The teleoperation code now treats collision geoms on every descendant of `dynamic_spoon2` as spoon geoms. This is required because the collision geoms are no longer attached directly to the parent body.
