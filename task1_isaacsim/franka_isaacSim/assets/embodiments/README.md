# Embodiments – Robot Configuration Framework

This directory defines distinct robot embodiments (hardware configurations and their simulation parameters). Each embodiment encapsulates the complete robot topology, joint parametrization, kinematic chains, asset references, and control contracts.

## Design Rationale

**Why Embodiments?**
- **Modularity**: Each embodiment is self-contained; adding a new robot variant requires only a new subdirectory and config files, no code changes.
- **Reusability**: Embodiment configs are loaded dynamically by bridges, tests, and optimization scripts, ensuring consistent robot definitions across all tools.
- **Scalability**: As new robot platforms or variants are added (e.g., FR3 Solo, FR3 Mobile with different base), the pattern scales without bloat.

## Embodiment Naming Convention

**Format**: `{base_model}_{configuration_variant}`

- `fr3duo_m+v` – Two FR3v2 arms (Mobile + tabletop variant) on a fixed pedestal base.
- `fr3duo_mobile` – Two FR3v2 arms on an omnidirectional mobile base platform.
- `fr3_solo` – Single FR3v2 arm (future).

## Directory Structure

```
assets/embodiments/
├── README.md                     (this file)
├── registry.py                   (embodiment loading/resolution logic)
├── loader.py                     (YAML loading utilities)
│
├── fr3duo_m+v/                   (canonical fixed-base configuration)
│   ├── embodiment_config.yaml    (metadata, platform flags)
│   ├── joint_parametrization.yaml (all joint DOF definitions)
│   ├── kinematic_chain.yaml      (link tree, frame transforms)
│   ├── asset_references.yaml     (USD, URDF file paths)
│   ├── joint_drive_config.yaml   (Isaac drive stiffness, damping, force)
│   └── data_contract.yaml        (sampling rate, state structure, semantics)
│
└── fr3duo_mobile/                (mobile-base variant)
    ├── embodiment_config.yaml
    ├── joint_parametrization.yaml (includes base 3-DOF planar + 7+7 arm DOF)
    ├── kinematic_chain.yaml
    ├── asset_references.yaml
    ├── joint_drive_config.yaml
    └── data_contract.yaml
```

## Configuration Files – Overview

### 1. **embodiment_config.yaml**
Metadata and topology flags. Used by loaders and bridges to determine behavior.

**Example (fr3duo_m+v):**
```yaml
embodiment:
  name: fr3duo_m+v
  display_name: "Dual FR3v2 Arms on Fixed Pedestal"
  version: "1.0"
  description: "Two stationary FR3v2 robotic arms on a fixed pedestal base."
  category: "fixed_base"

platform:
  has_mobile_base: false
  arm_count: 2
  base_mobility_dof: 0  # Fixed: no translation/rotation
  total_controllable_dof: 14  # 7 per arm

availability:
  isaac_sim: true
  real_hardware: true
  data_collection: true
  demo_capable: true

recommended_defaults:
  controller_mode: effort  # or "position"
  simulation_physics_hz: 240
  ros_publish_rate_hz: 60
  command_smoothing_alpha: 0.08
```

### 2. **joint_parametrization.yaml**
Exhaustive joint definitions. Each joint carries type, limits, home pose, semantics.

**Example excerpt (fr3duo_m+v):**
```yaml
embodiment_key: fr3duo_m+v
version: "1.0"

arm_groups:
  left:
    arm_name: Left FR3v2
    base_link: left_link0
    ee_link: left_link8
    controllable_joints:
      - name: left_fr3v2_joint1
        index: 0
        type: revolute
        axis: z
        limits:
          lower: -2.8973
          upper: 2.8973
        home_position: 0.0
        velocity_limit: 2.175
        effort_limit: 87.0
      # ... (7 joints per arm)
  right:
    # ... (mirrored structure)
```

### 3. **kinematic_chain.yaml**
Rigid-body tree: link names, parent-child relationships, static transforms (DH params or frame transforms).

**Example excerpt:**
```yaml
embodiment_key: fr3duo_m+v
version: "1.0"

base_link: world

links:
  world:
    inertia_mass: 0.0
    inertia_matrix: [0, 0, 0, 0, 0, 0, 0, 0, 0]

  left_link0:
    parent: world
    transform_xyz: [-0.3, 0.0, 0.0]
    transform_rpy: [0.0, 0.0, 0.0]
    inertia_mass: 2.5  # Pedestal segment
    
  left_link1:
    parent: left_link0
    transform_xyz: [0.0, 0.0, 0.333]
    transform_rpy: [0.0, 0.0, 0.0]
```

### 4. **asset_references.yaml**
Paths to USD, URDF, and 3D asset files used in simulation and real hardware setup.

**Example:**
```yaml
embodiment_key: fr3duo_m+v
version: "1.0"

assets:
  pedestal_base:
    description: "Fixed stainless steel pedestal base"
    usd_path: /workspace/assets/3D_assets/pedestal_base/pedestal.usd
    urdf_path: /workspace/assets/franka_description/pedestal/pedestal.urdf
    
  left_arm:
    description: "Left FR3v2 robot arm"
    usd_path: /workspace/assets/fr3_franka_hand.usd
    urdf_path: /workspace/assets/franka_description/robots/fr3v2_1/fr3v2.urdf
    prim_path_in_stage: /left_fr3v2
    
  left_gripper:
    description: "Left Robotiq 2F-85 adaptive gripper"
    usd_path: /workspace/assets/ai_cell_robotiq_2f85.usd
    
  right_arm: { /* mirrored */ }
  right_gripper: { /* mirrored */ }
```

### 5. **joint_drive_config.yaml**
Isaac Sim joint-drive physics parameters (stiffness, damping, max force). These are tuned per embodiment/platform.

**Example:**
```yaml
embodiment_key: fr3duo_m+v
version: "1.0"

joint_drives:
  left_fr3v2_joint1:
    prim_path: /left_fr3v2/left_fr3v2_joint1
    drive_axis: angular
    drive_type: acceleration
    stiffness: 625.0      # Base stiffness (will be scaled at runtime)
    damping: 0.003        # Base damping
    max_force: 87.0       # Base max effort

  # ... (all 14 controllable joints)
```

### 6. **data_contract.yaml**
Defines the data structure, sampling semantics, and contracts for logs, replays, and ML pipelines.

**Example:**
```yaml
embodiment_key: fr3duo_m+v
version: "1.0"
contract_name: fr3_duo_data_contract
contract_version: "1.1.0"

sampling:
  frequency_hz: 60
  deterministic: true

state_structure:
  arms:
    left:
      joint_count: 7
      joint_names:
        - left_fr3v2_joint1
        - left_fr3v2_joint2
        # ...
      fields: [position, velocity, effort]
    right: { /* mirrored */ }
  
  gripper:
    left:
      joint_name: left_robotiq_85_left_knuckle_joint
      normalized_semantics: "open_fraction"  # 1.0=open, 0.0=closed
    right: { /* mirrored */ }

action_structure:
  arm_targets: [14]  # 7 left + 7 right joint targets
  gripper_targets: [2]  # left opening + right opening
```

---

## Usage in Code

### Loading an Embodiment

```python
from assets.embodiments.loader import load_embodiment

# Load the canonical fixed-base config
embodiment = load_embodiment("fr3duo_m+v")

# Access components
joints = embodiment["joint_parametrization"]["arm_groups"]["left"]["controllable_joints"]
asset_urdf = embodiment["asset_references"]["left_arm"]["urdf_path"]
drive_config = embodiment["joint_drive_config"]["joint_drives"]
```

### Registering a New Embodiment

1. Create a new subdirectory: `assets/embodiments/my_new_embodiment/`
2. Copy and adapt all `.yaml` files from an existing embodiment (e.g., `fr3duo_m+v`).
3. Update all `embodiment_key` fields and customize parameters.
4. (Optional) Register in a central registry if needed:
   ```python
   from assets.embodiments.registry import register_embodiment
   register_embodiment("my_new_embodiment", "/path/to/embodiments/my_new_embodiment")
   ```

---

## Design Decisions

### Why Separate Files Instead of One Monolithic YAML?

- **Clarity**: Each file has a single responsibility (joints vs. kinematics vs. assets).
- **Reuse**: Tools can load only what they need (e.g., a planner might only need `kinematic_chain.yaml`).
- **Version Control**: Changes to one aspect (e.g., drive tuning) don't require touching other configs.

### Why YAML Over Python Classes?

- **Portability**: Non-Python tools (sim, ML pipelines, external tools) can load YAML easily.
- **Easy Edits**: Domain experts (not programmers) can tune parameters without rebuilding.
- **No Code Coupling**: Robot changes don't require code deployments.

### Scalability

- As the number of embodiments grows, this pattern remains flat and discoverable.
- Tools can dynamically enumerate available embodiments by listing subdirectories.
- No central registry required (though one can be added for advanced features like versioning or deprecation).

---

## Next Steps

1. **Implement `loader.py`**: Provide utilities to load, validate, and cache embodiments.
2. **Integrate with `isaac_bridge_constants.py`**: Make joint groups and data contracts embodiment-aware.
3. **Update `run_current_target_hold_test.py`**: Add `--embodiment` flag.
4. **Extend `optimize_isaac_joint_drives.py`**: Support per-embodiment optimization.
5. **Create `fr3duo_mobile/` variant**: Implement mobile-base embodiment with 3-DOF planar base.
