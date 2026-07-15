# Creating Template Objects for the Object Spawner

This guide shows you how to set up template objects in your Isaac Sim scene that the spawner will copy.

## Quick Setup

### Step 1: Create Template Objects in Your USD Scene

You can create template objects in Isaac Sim in several ways:

#### Option A: Using Isaac Sim GUI
1. Open your USD scene in Isaac Sim
2. In the Stage panel, create a new Xform under `/World/templates/`
3. Add primitives (Create → Mesh → Cube/Sphere/Cylinder)
4. Set materials and colors
5. Add physics (right-click → Add → Physics → Rigid Body)

#### Option B: Using Python in Isaac Sim
```python
from pxr import Gf, UsdGeom, UsdPhysics
from omni.isaac.core.utils.stage import get_current_stage

stage = get_current_stage()

# Create templates folder
templates_prim = stage.DefinePrim("/World/templates", "Scope")

# Create a cube template
cube_prim = stage.DefinePrim("/World/templates/cube_red", "Cube")
cube = UsdGeom.Cube(cube_prim)
cube.GetSizeAttr().Set(0.1)  # 10cm cube

# Add color
cube.CreateDisplayColorAttr([(0.8, 0.1, 0.1)])  # Red

# Add physics
UsdPhysics.RigidBodyAPI.Apply(cube_prim)
UsdPhysics.CollisionAPI.Apply(cube_prim)
mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
mass_api.GetMassAttr().Set(0.1)  # 100g

# Position it somewhere out of the way (won't be visible during spawning)
xform = UsdGeom.Xformable(cube_prim)
xform.AddTranslateOp().Set(Gf.Vec3d(10, 10, 0))  # Far away
```

### Step 2: Create Multiple Templates

Create several template objects for variety:

```python
# Blue sphere
sphere_prim = stage.DefinePrim("/World/templates/sphere_blue", "Sphere")
sphere = UsdGeom.Sphere(sphere_prim)
sphere.GetRadiusAttr().Set(0.05)  # 5cm radius
sphere.CreateDisplayColorAttr([(0.1, 0.1, 0.8)])  # Blue
UsdPhysics.RigidBodyAPI.Apply(sphere_prim)
UsdPhysics.CollisionAPI.Apply(sphere_prim)

# Green cylinder
cylinder_prim = stage.DefinePrim("/World/templates/cylinder_green", "Cylinder")
cylinder = UsdGeom.Cylinder(cylinder_prim)
cylinder.GetRadiusAttr().Set(0.03)  # 3cm radius
cylinder.GetHeightAttr().Set(0.08)  # 8cm height
cylinder.CreateDisplayColorAttr([(0.1, 0.8, 0.1)])  # Green
UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)
UsdPhysics.CollisionAPI.Apply(cylinder_prim)
```

### Step 3: Update objects_config.yaml

Reference your template prims:

```yaml
objects:
  - name: "cube_red"
    prim_path: "/World/templates/cube_red"
    scale: 1.0
    
  - name: "sphere_blue"
    prim_path: "/World/templates/sphere_blue"
    scale: 1.0
    
  - name: "cylinder_green"
    prim_path: "/World/templates/cylinder_green"
    scale: 1.0
```

## Advanced: Using Existing Objects

If you already have objects in your scene, you can reference them directly:

```yaml
objects:
  # Reference an existing object in your scene
  - name: "existing_box"
    prim_path: "/World/YourExistingBox"
    scale: 0.5  # Spawn at half size
    
  # Or from a robot gripper
  - name: "grasped_object"
    prim_path: "/World/GraspableObjects/Object1"
    scale: 1.0
```

## Tips

### Position Templates Off-Screen
Templates should be positioned away from the main scene so they're not visible:
```python
xform.AddTranslateOp().Set(Gf.Vec3d(100, 100, 0))  # Far away
```

### Use Different Sizes
Vary the `scale` parameter to spawn objects at different sizes:
```yaml
- name: "cube_small"
  prim_path: "/World/templates/cube"
  scale: 0.5   # 50% size
  
- name: "cube_large"
  prim_path: "/World/templates/cube"
  scale: 2.0   # 200% size
```

### Organize Templates
Keep templates organized in a dedicated folder:
```
/World/templates/
  ├── cube_red
  ├── cube_blue
  ├── sphere_yellow
  ├── cylinder_green
  └── custom_object
```

## Verification

To verify your templates are set up correctly:

1. Check they exist in Stage panel: `/World/templates/`
2. Verify they have physics: Right-click → Inspect → should show RigidBodyAPI
3. Run test: `exec(open('/workspace/services/object_spawner/test_spawner.py').read())`

## Troubleshooting

**"Source prim not found" error?**
- Check the `prim_path` in `objects_config.yaml` exactly matches the path in your stage
- Use Stage panel to verify the exact path

**Objects spawn but don't fall?**
- Ensure templates have RigidBodyAPI applied
- Check physics is enabled in the scene

**Objects spawn with wrong appearance?**
- Materials and colors come from the template prim
- Update the template prim's appearance, not the config
