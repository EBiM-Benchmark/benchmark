#!/usr/bin/env python3
"""Verify exact fingertip STL table contact and wheel collision disabling."""
from __future__ import annotations
from pathlib import Path
import importlib.util
import sys
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('teleop_table_validation', ROOT / 'teleop_keyboard.py')
t = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = t
spec.loader.exec_module(t)

model = mujoco.MjModel.from_xml_path(str(ROOT / 'benchmark_task3_mesh_table.xml'))
data = mujoco.MjData(model)
t.apply_ready_pose(model, data)
mujoco.mj_forward(model, data)
arms = {side: t.create_arm(model, data, side) for side in ('left', 'right')}
for arm in arms.values():
    t.open_gripper(model, data, arm)
    t.synchronize_arm(model, data, arm)
right = arms['right']
base = t.BaseDriver(model, data, 2.0, np.deg2rad(120), 0.00064, '+x', [1.0], 'jointvel')

# The spoon is irrelevant to this test.
for gid in range(model.ngeom):
    if t.body_is_descendant(model, int(model.geom_bodyid[gid]), right.spoon_body):
        model.geom_contype[gid] = 0
        model.geom_conaffinity[gid] = 0

wheel_active = []
for gid in range(model.ngeom):
    body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])) or ''
    if ('caster_' in body or 'argo_drive_' in body) and (model.geom_contype[gid] or model.geom_conaffinity[gid]):
        wheel_active.append(gid)

env_geoms = []
for gid in range(model.ngeom):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ''
    if '_environment_collision' in name:
        env_geoms.append(gid)

initial_table_contacts = []
for ci in range(data.ncon):
    c = data.contact[ci]
    n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)) or ''
    n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)) or ''
    if 'collision_runtime_table_top' in (n1, n2):
        initial_table_contacts.append((n1, n2, float(c.dist)))

first_contact = None
contact_names: set[str] = set()
for step in range(1200):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    t.hard_hold_arm(model, data, arms['left'])
    if step <= 100:
        t.hard_hold_arm(model, data, right)
    else:
        t.apply_twist_ik(model, data, right, np.array([0, 0, -0.15, 0, 0, 0], dtype=float), 0.015, 120.0)
    mujoco.mj_step(model, data)
    base.post_step_hold()
    for ci in range(data.ncon):
        c = data.contact[ci]
        n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)) or ''
        n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)) or ''
        if 'collision_runtime_table_top' not in (n1, n2):
            continue
        other = n2 if n1 == 'collision_runtime_table_top' else n1
        if other.startswith('right_actual_') and other.endswith('_environment_collision'):
            contact_names.add(other)
            if first_contact is None:
                first_contact = (step, other, float(c.dist))

all_meshes = all(int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_MESH) for gid in env_geoms)
tip_contacts = {n for n in contact_names if '_tip_environment_collision' in n}
ok = (len(env_geoms) == 18 and all_meshes and not wheel_active
      and not initial_table_contacts and len(tip_contacts) >= 1)
print(f'original STL environment collision geoms: {len(env_geoms)} / 18, all_mesh={all_meshes}')
print(f'active wheel/caster collision geoms: {len(wheel_active)}')
print(f'initial table contacts: {initial_table_contacts}')
print(f'first table contact: {first_contact}')
print(f'right gripper table-contact geoms: {sorted(contact_names)}')
print(f'LOWER GRIPPER/TABLE CONTACT: {"PASS" if ok else "FAIL"}')
raise SystemExit(0 if ok else 1)
