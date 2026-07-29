# Curved scoop collision update

The spoon stem and neck are unchanged from the previous working package:

- 14 original segmented stem meshes (`spoon_collision_original_stem_mesh_00` ... `_13`)
- Existing `spoon_collision_neck` box, with all original dimensions, pose, friction, filtering and contact parameters

Only the scoop was changed. The two coarse ellipsoids were removed:

- `spoon_collision_bowl_rear`
- `spoon_collision_bowl_front`

They were replaced with **139 thin fitted box geoms** copied from the uploaded `final_scene_100_beans(1).xml`:

- 110 curved surface boxes
- 24 side-rim boxes
- 5 thin leading-edge boxes

The boxes retain the uploaded local positions, sizes, orientations and contact parameters. They form a curved shell that follows and covers the visual scoop while leaving the existing stem and neck untouched.
