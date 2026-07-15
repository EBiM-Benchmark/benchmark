# Embodiments Integration Roadmap

## Overview
This document outlines how to integrate the new **embodiments framework** into existing bridges, tests, and optimization workflows.

## Current Status
✓ **Framework Complete**: Loader, registry, and two embodiments (fr3duo_m+v, fr3duo_mobile) ready.

## Phase 1: Loader Validation (Do First)

Create `scripts/test_embodiments_loader.py`:
```python
#!/usr/bin/env python3
"""Quick validation script for embodiments loader."""

from assets.embodiments.loader import (
    list_available_embodiments,
    load_embodiment,
    get_joint_names,
    validate_embodiment,
)

# Test 1: List available
print("Available embodiments:", list_available_embodiments())

# Test 2: Load each embodiment
for embodiment_name in list_available_embodiments():
    embodiment = load_embodiment(embodiment_name)
    print(f"\n{embodiment_name}: {len(embodiment)} components loaded")
    
    # Test 3: Get joint names
    joints = get_joint_names(embodiment_name)
    print(f"  Joints: {list(joints.keys())}")
    
    # Test 4: Validate
    is_valid, msg = validate_embodiment(embodiment_name)
    print(f"  Valid: {is_valid} ({msg})")
```

**Run**: `docker compose -f docker-compose.yml -f docker-compose.test.yml run --rm test-runner python3 /workspace/scripts/test_embodiments_loader.py`

---

## Phase 2: Integrate with Isaac Bridge Constants

**File**: `scripts/isaac_bridge_constants.py`

**Changes**:
```python
# OLD (hard-coded)
LEFT_JOINTS = [
    "left_fr3v2_joint1",
    # ...
]

# NEW (embodiment-aware)
def get_embodiment_joints(embodiment_name: str = "fr3duo_m+v", arm: str = "left"):
    """Get joint names from embodiment config."""
    from assets.embodiments.loader import get_joint_names
    return get_joint_names(embodiment_name, arm)

# Keep backward compatibility
LEFT_JOINTS = get_embodiment_joints("fr3duo_m+v", "left")
RIGHT_JOINTS = get_embodiment_joints("fr3duo_m+v", "right")
```

---

## Phase 3: Extend Test Runner (run_current_target_hold_test.py)

**Add Argument**:
```python
parser.add_argument(
    "--embodiment",
    type=str,
    default="fr3duo_m+v",
    help="Robot embodiment variant to use"
)
```

**Load Embodiment Config Early**:
```python
from assets.embodiments.loader import load_embodiment, get_joint_names

embodiment_config = load_embodiment(args.embodiment)
left_joints = get_joint_names(args.embodiment, "left")
right_joints = get_joint_names(args.embodiment, "right")

# Use for building test command with correct joint names
```

---

## Phase 4: Extend Optimization Script (optimize_isaac_joint_drives.py)

**Add Argument**:
```python
parser.add_argument(
    "--embodiment",
    type=str,
    default="fr3duo_m+v",
    help="Embodiment to optimize"
)
```

**Update Summary Output**:
```python
summary = {
    "embodiment": args.embodiment,
    "embodiment_version": embodiment["embodiment_config"]["version"],
    # ... existing fields
}
```

---

## Phase 5: Update Data Collection Pipeline

**In Episode Replay & Data Collection Scripts**:
```python
# Load embodiment data contract
embodiment = load_embodiment(args.embodiment)
data_contract = embodiment["data_contract"]

# Validate collected episode structure matches contract
def validate_episode_against_contract(episode_data, data_contract):
    expected_arm_dof = data_contract["state_structure"]["arms"]["left"]["joint_count"]
    actual_arm_dof = len(episode_data["left"]["position"])
    assert actual_arm_dof == expected_arm_dof, "Mismatch!"
```

---

## Phase 6: Documentation Updates

**Update README.md**:
```markdown
## Supported Embodiments

The system supports multiple robot configurations via the embodiments framework:
- **fr3duo_m+v**: Fixed-base dual-arm (default, 14 DOF)
- **fr3duo_mobile**: Mobile-base dual-arm (simulation, 17 DOF)

### Running with a Specific Embodiment

\`\`\`bash
bash scripts/run_native_stream.sh --embodiment fr3duo_mobile

# Or in tests:
python3 scripts/test_suites/run_current_target_hold_test.py --embodiment fr3duo_mobile
\`\`\`
```

---

## Phase 7: Add New Embodiment Template

For future platforms, create:
```bash
mkdir assets/embodiments/my_new_embodiment
cp assets/embodiments/fr3duo_m+v/* assets/embodiments/my_new_embodiment/

# Edit 6 YAML files with new configuration
```

---

## Validation Checklist

Before merging:

- [ ] Loader validation script passes
- [ ] All 6 YAML files per embodiment are syntactically valid
- [ ] Joint counts match across embodiment_config and joint_parametrization
- [ ] Embodiment key consistency (all files reference same embodiment_key)
- [ ] Asset paths point to existing files (or are clearly placeholders)
- [ ] Data contract structure matches state recorded in samples
- [ ] No Python code changes required to switch embodiments (YAML only)

---

## Future Enhancements

1. **Embodiment Registry with Versioning**: Track deprecated embodiments, migration notes
2. **Hardware-in-the-Loop Validation**: Sync embodiment configs with actual robot telemetry
3. **Embodiment Inheritance**: Base class configs for common platforms (FR3 variants)
4. **Automatic Asset Path Resolution**: Smart fallback if USD not found (use URDF, etc.)
5. **Schema Validation**: JSONSchema or Pydantic models for embodiment YAML files

---

**Questions?** See `assets/embodiments/README.md` for full design rationale.
