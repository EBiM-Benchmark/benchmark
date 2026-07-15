#!/usr/bin/env python3
"""Object Spawner Service for Isaac Sim

Spawns objects above a table by copying existing prims, monitors their height,
and respawns them if they fall below a threshold.
"""

import os
import random
import time
import yaml
from typing import Dict, List, Optional, Tuple

# Isaac Sim imports (only available in Isaac Sim container)
try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
    from omni.isaac.core.utils.stage import get_current_stage
    import omni.usd
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("Warning: Isaac Sim modules not available. Running in mock mode.")


class ObjectSpawner:
    """Manages spawning and monitoring of objects in Isaac Sim."""

    def __init__(
        self,
        objects_config_path: str,
        spawn_config_path: str,
        stage=None,
        world=None,
    ):
        """Initialize the object spawner.
        
        Args:
            objects_config_path: Path to objects_config.yaml
            spawn_config_path: Path to spawn_config.yaml
            stage: USD stage (required for Isaac Sim)
            world: Isaac World instance (required for physics)
        """
        self.stage = stage
        self.world = world
        
        # Load configurations
        with open(objects_config_path, 'r') as f:
            self.objects_config = yaml.safe_load(f)
        
        with open(spawn_config_path, 'r') as f:
            self.spawn_config = yaml.safe_load(f)
        
        # Active objects tracking
        self.active_objects: Dict[str, dict] = {}  # prim_path -> object_info
        self.object_counter = 0
        self.last_spawn_time = 0.0
        self.enabled = True  # Spawner is enabled by default
        
        # Spawn configuration
        self.spawn_center = self.spawn_config['spawn_area']['center']
        self.spawn_range = self.spawn_config['spawn_area']['range']
        self.despawn_height = self.spawn_config['despawn_height']
        self.spawn_interval = self.spawn_config['spawn_interval']
        self.max_objects = self.spawn_config['max_objects']
        
        print(f"Object Spawner initialized:")
        print(f"  Spawn center: {self.spawn_center}")
        print(f"  Spawn range: ±{self.spawn_range}")
        print(f"  Despawn height: {self.despawn_height}m")
        print(f"  Max objects: {self.max_objects}")
        print(f"  Available object types: {len(self.objects_config['objects'])}")

    def _get_random_position(self) -> Tuple[float, float, float]:
        """Generate a random spawn position within the configured area."""
        x = self.spawn_center[0] + random.uniform(-self.spawn_range[0], self.spawn_range[0])
        y = self.spawn_center[1] + random.uniform(-self.spawn_range[1], self.spawn_range[1])
        z = self.spawn_center[2] + random.uniform(-self.spawn_range[2], self.spawn_range[2])
        return (x, y, z)

    def _get_random_object_config(self) -> dict:
        """Select a random object configuration from the available objects."""
        return random.choice(self.objects_config['objects'])

    def _apply_collision_approximation_recursive(self, prim):
        """Apply convex hull collision approximation to all mesh prims recursively.
        
        This prevents the triangle mesh collision errors for dynamic bodies.
        
        Args:
            prim: USD prim to process (will recursively process children)
        """
        if not ISAAC_SIM_AVAILABLE:
            return
        
        # Check if this prim is a mesh
        if prim.IsA(UsdGeom.Mesh):
            # Apply mesh collision API with convex hull approximation
            if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
                # Set approximation to convex hull (valid for dynamic bodies)
                mesh_collision.GetApproximationAttr().Set("convexHull")
            else:
                mesh_collision = UsdPhysics.MeshCollisionAPI(prim)
                mesh_collision.GetApproximationAttr().Set("convexHull")
        
        # Recursively process all children
        for child in prim.GetChildren():
            self._apply_collision_approximation_recursive(child)

    def spawn_object(self) -> Optional[str]:
        """Spawn a new random object at a random position by copying an existing prim.
        
        Returns:
            Prim path of spawned object, or None if spawn failed
        """
        if not ISAAC_SIM_AVAILABLE or self.stage is None:
            print("Cannot spawn: Isaac Sim not available or stage not set")
            return None
        
        # Check max objects limit
        if len(self.active_objects) >= self.max_objects:
            return None
        
        # Get random object and position
        obj_config = self._get_random_object_config()
        position = self._get_random_position()
        
        # Generate unique prim path under /World/objects
        self.object_counter += 1
        dest_prim_path = f"/World/objects/{obj_config['name']}_{self.object_counter:04d}"
        
        # Ensure /World/objects directory exists
        objects_dir = "/World/objects"
        objects_prim = get_prim_at_path(objects_dir)
        if not objects_prim or not objects_prim.IsValid():
            print(f"Creating {objects_dir} directory...")
            UsdGeom.Xform.Define(self.stage, objects_dir)
            objects_prim = get_prim_at_path(objects_dir)
            print(f"  Created: {objects_prim.IsValid() if objects_prim else 'FAILED'}")
        
        try:
            # Get the source prim to copy
            source_prim_path = obj_config['prim_path']
            source_prim = get_prim_at_path(source_prim_path)
            
            if not source_prim or not source_prim.IsValid():
                # Create a simple cube template if source doesn't exist
                # Use the object name for the template
                template_path = f"/World/templates/{obj_config['name']}"
                print(f"Warning: Source prim not found: {source_prim_path}, creating default cube template at {template_path}")
               
                # Create template parent if needed
                template_dir = "/World/templates"
                if not get_prim_at_path(template_dir).IsValid():
                    UsdGeom.Xform.Define(self.stage, template_dir)
                
                # Create a simple cube as template with the object name
                cube = UsdGeom.Cube.Define(self.stage, template_path)
                cube.GetSizeAttr().Set(0.2)  # 50cm HUGE cube - impossible to miss!
                
                # Position template off-screen (templates shouldn't be visible)
                xform = UsdGeom.Xformable(cube)
                xform.AddTranslateOp().Set(Gf.Vec3d(-10.0, -10.0, 0.5))
                
                # Add BRIGHT YELLOW color for maximum visibility
                cube.GetDisplayColorAttr().Set([(1.0, 0.0, 0.0)])
                
                # Apply all physics properties to template prim
                template_prim = cube.GetPrim()
                
                # Rigid body physics - ensure it's dynamic (not kinematic)
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(template_prim)
                rigid_body.GetRigidBodyEnabledAttr().Set(True)
                
                # Collision
                UsdPhysics.CollisionAPI.Apply(template_prim)
                
                # Mass
                mass_api = UsdPhysics.MassAPI.Apply(template_prim)
                mass_api.GetMassAttr().Set(
                    self.objects_config['physics'].get('mass', 0.1)
                )
                
                # Update source references to use the created template
                source_prim = template_prim
                source_prim_path = template_path
                print(f"Created default cube template '{obj_config['name']}' at {template_path}")
            
            # Copy the prim to the new location
            omni.usd.duplicate_prim(
                self.stage,
                source_prim_path,
                dest_prim_path
            )
            
            # Get the newly created prim
            prim = get_prim_at_path(dest_prim_path)
            if not prim or not prim.IsValid():
                print(f"Error: Failed to create prim at {dest_prim_path}")
                print(f"  Prim exists: {prim is not None}, Valid: {prim.IsValid() if prim else 'N/A'}")
                return None
            
            print(f"✓ Successfully created prim at {dest_prim_path}")
            print(f"  Prim type: {prim.GetTypeName()}")
            print(f"  Prim is active: {prim.IsActive()}")
            print(f"  Prim has children: {len(list(prim.GetChildren()))}")
            
            # Set transform
            xform = UsdGeom.Xformable(prim)
            random_angles = (0.0, 0.0, 0.0)
            if xform:
                scale = obj_config.get('scale', 1.0)
                
                # Clear and set new transform operations
                xform.ClearXformOpOrder()
                
                # Position
                translate_op = xform.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
                
                # Random rotation on all three axes (0-360 degrees each)
                random_x = random.uniform(0, 360)
                random_y = random.uniform(0, 360)
                random_z = random.uniform(0, 360)
                random_angles = (random_x, random_y, random_z)
                rotate_op = xform.AddRotateXYZOp()
                rotate_op.Set(Gf.Vec3f(random_x, random_y, random_z))
                
                # Scale (if different from 1.0)
                if scale != 1.0:
                    scale_op = xform.AddScaleOp()
                    scale_op.Set(Gf.Vec3f(scale, scale, scale))
            
            # Ensure all physics properties are applied to spawned prim
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
                rigid_body.GetRigidBodyEnabledAttr().Set(True)
            else:
                # Ensure it's enabled and dynamic
                rigid_body = UsdPhysics.RigidBodyAPI(prim)
                rigid_body.GetRigidBodyEnabledAttr().Set(True)
            
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(prim)
            
            # Apply convex hull approximation to all mesh children to fix triangle mesh errors
            self._apply_collision_approximation_recursive(prim)
            
            if not prim.HasAPI(UsdPhysics.MassAPI):
                mass_api = UsdPhysics.MassAPI.Apply(prim)
                mass_api.GetMassAttr().Set(
                    self.objects_config['physics'].get('mass', 0.1)
                )
            
            # Track the object
            self.active_objects[dest_prim_path] = {
                'name': obj_config['name'],
                'spawn_time': time.time(),
                'position': position,
                'rotation': random_angles,
                'source_prim': source_prim_path,
            }
            
            # Verify prim still exists after all operations
            final_check = get_prim_at_path(dest_prim_path)
            if final_check and final_check.IsValid():
                print(f"✓ Spawned {obj_config['name']} (from {source_prim_path}) at {position} rotation=({random_angles[0]:.1f}°, {random_angles[1]:.1f}°, {random_angles[2]:.1f}°) -> {dest_prim_path}")
                print(f"  Final verification: Prim is valid and active={final_check.IsActive()}")
            else:
                print(f"⚠ WARNING: Prim {dest_prim_path} was created but is no longer valid!")
                
            return dest_prim_path
            
        except Exception as exc:
            print(f"Failed to spawn object: {exc}")
            import traceback
            traceback.print_exc()
            return None

    def get_object_height(self, prim_path: str) -> Optional[float]:
        """Get the current Z height of an object.
        
        Args:
            prim_path: USD prim path
            
        Returns:
            Current Z height in meters, or None if object not found
        """
        if not ISAAC_SIM_AVAILABLE or self.stage is None:
            return None
        
        prim = get_prim_at_path(prim_path)
        if not prim or not prim.IsValid():
            return None
        
        xformable = UsdGeom.Xformable(prim)
        if not xformable:
            return None
        
        # Get world transform
        transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = transform.ExtractTranslation()
        
        return position[2]  # Z coordinate

    def despawn_object(self, prim_path: str) -> bool:
        """Remove an object from the scene.
        
        Args:
            prim_path: USD prim path
            
        Returns:
            True if successfully despawned
        """
        if not ISAAC_SIM_AVAILABLE or self.stage is None:
            return False
        
        try:
            delete_prim(prim_path)
            if prim_path in self.active_objects:
                obj_info = self.active_objects.pop(prim_path)
                print(f"Despawned {obj_info['name']} ({prim_path})")
            return True
        except Exception as exc:
            print(f"Failed to despawn {prim_path}: {exc}")
            return False

    def set_enabled(self, enabled: bool):
        """Enable or disable the spawner.
        
        Args:
            enabled: True to enable spawning, False to disable
        """
        self.enabled = enabled
        status = "enabled" if enabled else "disabled"
        print(f"Object spawner {status}")
    
    def is_enabled(self) -> bool:
        """Check if spawner is currently enabled."""
        return self.enabled
    
    def toggle_enabled(self) -> bool:
        """Toggle spawner enabled state.
        
        Returns:
            New enabled state
        """
        self.enabled = not self.enabled
        status = "enabled" if self.enabled else "disabled"
        print(f"Object spawner {status}")
        return self.enabled

    def update(self, current_time: float) -> None:
        """Update spawner - check object heights and spawn new objects.
        
        Should be called every simulation tick.
        
        Args:
            current_time: Current simulation time in seconds
        """
        if not ISAAC_SIM_AVAILABLE or self.stage is None:
            return
        
        # Check all active objects
        to_despawn = []
        for prim_path, obj_info in list(self.active_objects.items()):
            height = self.get_object_height(prim_path)
            
            if height is None:
                # Object disappeared
                to_despawn.append(prim_path)
            elif height < self.despawn_height:
                # Object fell below threshold
                print(f"Object {obj_info['name']} fell to {height:.2f}m (threshold: {self.despawn_height}m)")
                to_despawn.append(prim_path)
        
        # Despawn fallen objects
        despawned_count = len(to_despawn)
        for prim_path in to_despawn:
            self.despawn_object(prim_path)
        
        # Skip spawning logic if spawner is disabled
        if not self.enabled:
            return
        
        # Check current active count after despawning
        active_count = len(self.active_objects)
        time_since_last = current_time - self.last_spawn_time
        
        # Ensure at least 1 object is always active
        if active_count == 0:
            print("No objects active - spawning immediately to maintain at least 1 object")
            if self.spawn_object():
                self.last_spawn_time = current_time
        # If objects were despawned and we're below max, spawn replacements immediately
        elif despawned_count > 0 and active_count < self.max_objects:
            print(f"Objects despawned - spawning replacement (active={active_count}/{self.max_objects})")
            if self.spawn_object():
                self.last_spawn_time = current_time
        # Otherwise use normal time-based spawning
        elif time_since_last >= self.spawn_interval and active_count < self.max_objects:
            print(f"Spawning: time_since_last={time_since_last:.1f}s, active={active_count}/{self.max_objects}")
            if self.spawn_object():
                self.last_spawn_time = current_time
        elif active_count >= self.max_objects and time_since_last >= 10:
            # Log every 10 seconds when at max capacity
            print(f"At max capacity: {active_count}/{self.max_objects} objects")
            self.last_spawn_time = current_time  # Reset timer to avoid spamming

    def reset(self) -> None:
        """Clear all spawned objects."""
        print("Resetting object spawner - clearing all objects")
        for prim_path in list(self.active_objects.keys()):
            self.despawn_object(prim_path)
        self.last_spawn_time = 0.0


def main():
    """Standalone test of the spawner (requires running in Isaac Sim)."""
    print("Object Spawner - Standalone Mode")
    
    if not ISAAC_SIM_AVAILABLE:
        print("Error: Must run inside Isaac Sim environment")
        return
    
    # Get current stage
    stage = get_current_stage()
    if stage is None:
        print("Error: No USD stage loaded")
        return
    
    # Determine config paths
    service_dir = os.path.dirname(os.path.abspath(__file__))
    objects_config = os.path.join(service_dir, "objects_config.yaml")
    spawn_config = os.path.join(service_dir, "spawn_config.yaml")
    
    # Create spawner
    spawner = ObjectSpawner(
        objects_config_path=objects_config,
        spawn_config_path=spawn_config,
        stage=stage,
        world=None,
    )
    
    # Spawn initial objects
    print("\nSpawning initial objects...")
    for _ in range(3):
        spawner.spawn_object()
        time.sleep(0.5)
    
    print("\nObject spawner ready. Use spawner.update(time) in simulation loop.")


if __name__ == "__main__":
    main()
