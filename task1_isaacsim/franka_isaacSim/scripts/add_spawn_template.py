#!/usr/bin/env /isaac-sim/python.sh
"""Add a simple cube template for object spawning."""

from pxr import Usd, UsdGeom, Gf, UsdPhysics
import omni.usd

# Get the current stage
stage = omni.usd.get_context().get_stage()

# Create a templates parent
templates_path = "/World/templates"
templates_xform = UsdGeom.Xform.Define(stage, templates_path)

# Create a simple cube as a template
cube_path = f"{templates_path}/test_cube"
cube = UsdGeom.Cube.Define(stage, cube_path)
cube.GetSizeAttr().Set(0.05)  # 5cm cube

# Position it off to the side (not visible)
xform = UsdGeom.Xformable(cube)
xform.AddTranslateOp().Set(Gf.Vec3d(2.0, 2.0, 0.5))

# Add physics
UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
mass_api.GetMassAttr().Set(0.1)

# Add color (blue)
cube.GetDisplayColorAttr().Set([(0.2, 0.5, 0.9)])

print(f"Created template cube at {cube_path}")
print("Update objects_config.yaml to use this prim_path")
