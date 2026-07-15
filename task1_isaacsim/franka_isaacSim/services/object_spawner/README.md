# Object Spawner Service

Spawns objects above a table in Isaac Sim and automatically respawns them when they fall below a threshold height.

## Quick Start

### 1. Enable the Service

Edit `.env` in the project root:
```bash
OBJECT_SPAWNER_ENABLED="true"
```

### 2. Set Your Table Coordinates

**IMPORTANT**: Update `spawn_config.yaml` with your table coordinates:

```yaml
spawn_area:
  center: [X, Y, Z]  # <-- Replace with your table coordinates (meters)
  range: [0.2, 0.3, 0.0]  # Adjust spawn area size as needed
```

Example for a table at position [0.5, 0.0] with top at 1.0m height:
```yaml
spawn_area:
  center: [0.5, 0.0, 1.0]  # Objects spawn above this point
  range: [0.2, 0.3, 0.0]   # ±20cm in X, ±30cm in Y
```

### 3. Adjust Despawn Height

Objects falling below this height will be removed and respawned:
```yaml
despawn_height: 0.6  # meters - adjust based on your table/floor
```

## Configuration Files

### `objects_config.yaml` - What Objects to Spawn

Define available objects by referencing existing prims in your simulation:
- **name**: Unique identifier for the object type
- **prim_path**: Path to existing prim in the simulation (will be copied)
- **scale**: Size multiplier (1.0 = original size, 2.0 = double size, etc.)
- **physics**: Mass (kg), friction, restitution (bounciness)

**IMPORTANT**: The prims referenced in `prim_path` must already exist in your simulation.
Create template objects in your USD scene (e.g., under `/World/templates/`) that will be copied when spawning.

Example:
```yaml
objects:
  - name: "my_cube"
    prim_path: "/World/templates/my_cube"  # Must exist in simulation
    scale: 1.0
```

The spawner will **copy** these template prims to `/World/objects/` with unique names.

### `spawn_config.yaml` - Where and When to Spawn

- **spawn_area.center**: [x, y, z] center point in world frame (meters)
- **spawn_area.range**: [±x, ±y, ±z] random variation range (meters)
- **despawn_height**: Z height threshold for object removal (meters)
- **spawn_interval**: Time between spawns (seconds)
- **max_objects**: Maximum simultaneous objects in scene

## How It Works

The spawner automatically:
1. ✅ Copies template prims from your simulation (e.g., `/World/templates/`)
2. 🎯 Places copies at random positions in `/World/objects/`
3. 🔍 Monitors object heights every simulation tick
4. ❌ Despawns objects that fall below threshold (< 0.6m by default)
5. 🔄 Spawns new random object copies to maintain population

**Template Prims**: Create your object templates in the USD scene (with materials, physics, etc.)
and reference them in `objects_config.yaml`. The spawner copies these templates when needed.

## Testing

Run the test script inside Isaac Sim:

```bash
# From Isaac Sim Python console or Script Editor:
exec(open('/workspace/services/object_spawner/test_spawner.py').read())
```

Or from command line if Isaac Sim is running:
```bash
python services/object_spawner/test_spawner.py
```

## Integration

The Setup Template Objects in Your Scene

1. **Create a templates folder** in your USD scene:
   - Add prims under `/World/templates/` (or any path you prefer)
   - Set up materials, colors, and properties on these templates
   - Templates should have physics enabled if you want spawned copies to be dynamic

2. **Reference templates in config**:
```yaml
objects:
  - name: "my_custom_object"
    prim_path: "/World/templates/my_custom_object"  # Your template prim
    scale: 1.5  # Spawn at 150% siz
  - name: "my_custom_object"
    usd_path: "/path/to/your/object.usd"
    scale: 0.08
    color: [1.0, 0.5, 0.0]  # Orange
```

### Change Spawn Behavior

Edit `spawn_config.yaml`:
- Increase `max_objects` for more objects
- Decrease `spawn_interval` for faster spawning
- Adjust `range` for wider/narrower spawn area
- Change `despawn_height` based on your setup

## Troubleshooting

**Objects not spawning?**
- Check `OBJECT_SPAWNER_ENABLED=true` in `.env`
- Verify spawn coordinates in `spawn_config.yaml`
- Check Isaac Sim console for error messages

**Objects spawn but immediately disappear?**
- Check if spawn height is above `despawn_height`
- Ensure spawn Z coordinate > 0.6m (or your threshold)

**Objects spawn in wrong location?**
- Update `spawn_area.center` with correct table coordinates
- Verify coordinates are in world frame (not robot frame)
