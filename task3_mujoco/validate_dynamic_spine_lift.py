#!/usr/bin/env python3
"""Validate that dynamic spine motion physically lifts a grasped spoon."""
from __future__ import annotations
from pathlib import Path
import importlib.util
import sys
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('teleop_dynamic_spine', ROOT / 'teleop_keyboard.py')
t = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = t
spec.loader.exec_module(t)

model = mujoco.MjModel.from_xml_path(str(ROOT / 'benchmark_task3_mesh_table.xml'))
data = mujoco.MjData(model)
t.apply_ready_pose(model, data)
mujoco.mj_forward(model, data)
t.disable_wheel_ground_collision(model)
mujoco.mj_forward(model, data)
arms = {side: t.create_arm(model, data, side) for side in ('left', 'right')}
for arm in arms.values():
    t.open_gripper(model, data, arm)
    t.synchronize_arm(model, data, arm)
base = t.BaseDriver(model, data, 2.0, np.deg2rad(120), 0.18, '+x', [1.0], 'actuator')
right = arms['right']
spoon = right.spoon_body
gripper = right.gripper_base_body

# Dynamic settle.
for _ in range(500):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    mujoco.mj_step(model, data)
    base.post_step_hold()

# Fixed-preload close.
t.start_gripper_closing(right)
for _ in range(350):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    t.update_gripper(model, data, right, model.opt.timestep * 8.0,
                     1.0, 18.0, 0.2, 8, 0.5, 0.005, False)
    mujoco.mj_step(model, data)
    base.post_step_hold()

left0, right0, dims0, _, _ = t.spoon_pad_contact_summary(model, data, right)
spoon_z0 = float(data.xpos[spoon, 2])
gripper_z0 = float(data.xpos[gripper, 2])
spine_joint = base.spine_joint
spine_q0 = float(data.qpos[base.spine_qpos_address])
rel0 = data.xpos[spoon] - data.xpos[gripper]
max_qacc = 0.0

# Raise the dynamic spine for 1.2 simulated seconds, then hold for 0.3 s.
for _ in range(1200):
    base.drive(0, 0, 1.0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    t.update_gripper(model, data, right, model.opt.timestep,
                     1.0, 18.0, 0.2, 8, 0.5, 0.005, False)
    mujoco.mj_step(model, data)
    base.post_step_hold()
    max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

for _ in range(300):
    base.drive(0, 0, 0.0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    t.update_gripper(model, data, right, model.opt.timestep,
                     1.0, 18.0, 0.2, 8, 0.5, 0.005, False)
    mujoco.mj_step(model, data)
    base.post_step_hold()
    max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

spine_q1 = float(data.qpos[base.spine_qpos_address])
spoon_lift = float(data.xpos[spoon, 2] - spoon_z0)
gripper_lift = float(data.xpos[gripper, 2] - gripper_z0)
relative_error = float(np.linalg.norm((data.xpos[spoon] - data.xpos[gripper]) - rel0))
left1, right1, dims1, _, _ = t.spoon_pad_contact_summary(model, data, right)

ok = (
    left0 > 0 and right0 > 0 and 6 in dims0
    and left1 > 0 and right1 > 0 and 6 in dims1
    and spine_q1 - spine_q0 > 0.12
    and spoon_lift > 0.10 and gripper_lift > 0.10
    and relative_error < 0.008
    and np.isfinite(max_qacc) and max_qacc < 1e9
)
print(f'spine start/end: {spine_q0:.6f} -> {spine_q1:.6f} m')
print(f'spine displacement: {spine_q1-spine_q0:.6f} m')
print(f'initial contacts left/right={left0}/{right0}, dims={sorted(dims0)}')
print(f'final contacts left/right={left1}/{right1}, dims={sorted(dims1)}')
print(f'gripper lift={gripper_lift:.6f} m, spoon lift={spoon_lift:.6f} m')
print(f'relative spoon/gripper error={relative_error:.6f} m')
print(f'max |qacc|={max_qacc:.3e}')
print(f'DYNAMIC SPINE LIFT: {"PASS" if ok else "FAIL"}')
raise SystemExit(0 if ok else 1)
