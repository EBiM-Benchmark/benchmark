# Object Spawner Service - Setup Complete ✅

## What Has Been Created

### Service Files (in `services/object_spawner/`)
- ✅ **object_spawner.py** - Main spawner implementation
- ✅ **objects_config.yaml** - Define which objects to spawn (5 example objects included)
- ✅ **spawn_config.yaml** - Configure spawn location and behavior
- ✅ **test_spawner.py** - Test script to verify functionality
- ✅ **README.md** - Complete documentation
- ✅ **Dockerfile** - Service container definition

### Integration
- ✅ Environment variable added to `.env`: `OBJECT_SPAWNER_ENABLED="true"`
- ✅ Integrated into `isaac_bridge_session.py` - automatically runs during simulation

## 🎯 NEXT STEPS - YOU NEED TO DO THIS

### 1. Create Template Objects in Your Simulation

The spawner copies existing prims from your simulation. You need to:

**a) Create template objects** in your USD scene:
- Add prims under `/World/templates/` (or any path)
- Set up materials, colors, physics properties
- These are the objects that will be copied

**b) Update `objects_config.yaml`** with your template prim paths:
```yaml
objects:
  - name: "my_cube"
    prim_path: "/World/templates/my_cube"  # Path to your template
    scale: 1.0
```

Example templates you might create:
- `/World/templates/cube_red` - A red cube with physics
- `/World/templates/sphere_blue` - A blue sphere
- `/World/templates/cylinder` - A cylinder object

### 2. Provide Your Table Coordinates

Edit `services/object_spawner/spawn_config.yaml` and replace the placeholder coordinates:

```yaml
spawn_area:
  center: [X, Y, Z]  # <-- REPLACE WITH YOUR TABLE COORDINATES
  range: [0.2, 0.3, 0.0]
```

**Example:** If your table center is at X=0.6m, Y=0.0m, and the top surface is at Z=0.9m:
```yaml
spawn_area:
  center: [0.6, 0.0, 1.0]  # Spawn 10cm above table surface
  range: [0.2, 0.3, 0.0]   # Random ±20cm X, ±30cm Y
```

### 3. Verify Despawn Height

Objects that fall below this height will disappear and respawn:
```yaml
despawn_height: 0.6  # meters - adjust if your floor is different
```

### 4. Test the Spawner

Start Isaac Sim and run:
```bash
make digital-twin-up
# or your usual startup command
```

The spawner will automatically:
- Spawn objects every 3 seconds (configurable)
- Monitor object heights
- Remove objects that fall below 0.6m
- Spawn new random objects

## Configuration Options

### Template Objects (in `objects_config.yaml`)

You must create template prims in your simulation first! Examples:
```yaml
objects:
  - name: "cube_red"
    prim_path: "/World/templates/cube_red"  # Must exist in your USD scene
    scale: 1.0
    
  - name: "sphere"
    prim_path: "/World/templates/sphere"  # Must exist in your USD scene
    scale: 1.2  # Spawn at 120% size
```

**The spawner copies these templates to `/World/objects/` when spawning.**

### Spawn Behavior (in `spawn_config.yaml`)

- **spawn_interval**: Time between spawns (default: 3.0 seconds)
- **max_objects**: Maximum objects in scene (default: 5)
- **despawn_height**: Objects below this fall off (default: 0.6m)
Template prims exist in your USD scene │
│  (e.g., /World/templates/cube_red)     │
└─────────────────────────────────────────┘
            ↓
┌─────────────────────────────────────────┐
│  Every 3 seconds (spawn_interval)      │
│  ↓                                       │
│  Copy random template prim              │
│  ↓                                       │
│  Place copy at random position          │
│  within spawn_area (center ± range)    │
│  ↓                                       │
│  Copy placed in /World/objects/         │
└─────────────────────────────────────────┘
            ↓
┌─────────────────────────────────────────┐
│  Every simulation tick                  │
│  ↓                                       │
│  Check height of all spawned objects    │
│  ↓                                       │
│  If height < despawn_height (0.6m):     │
│    - Remove object from scene           │
│    - Ready to spawn new copy             │
│  Check height of all active objects     │
│  ↓                                       │
│  If height < despawn_height (0.6m):     │
│    - Remove object from scene           │
│    - Ready to spawn new one             │
└─────────────────────────────────────────┘
```

## Testing

### Option 1: Full Simulation Test
Start the simulation normally - spawner runs automatically if enabled.

### Option 2: Standalone Test
From Isaac Sim Python console:
```python
exec(open('/workspace/services/object_spawner/test_spawner.py').read())
```

## Disable the Spawner

If you want to turn it off:
```bash
# In .env file:
OBJECT_SPAWNER_ENABLED="false"
```

## Example Coordinates

Here are some example configurations:

**Table in front of robot:**
```yaml
center: [0.5, 0.0, 1.0]  # 50cm forward, centered, 1m height
range: [0.15, 0.25, 0.0]  # Small spawn area
```

**Table to the left:**
```yaml
center: [0.4, 0.5, 0.95]  # Forward-left, 95cm height
range: [0.2, 0.2, 0.0]    # Square spawn area
```

**Large workspace:**
```yaml
center: [0.6, 0.0, 1.0]
range: [0.3, 0.4, 0.0]    # Wider spawn area
```

## Questions?

See the full documentation in `services/object_spawner/README.md`
