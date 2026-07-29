#!/usr/bin/env python3
"""Runtime-equivalent table-supported grasp/lift regression.

This uses the exact small-model 1 ms Newton/elliptic solver, fixed 0.735 rad
preload, original Robotiq fingertip STL contact meshes, 14 segmented spoon-handle
meshes, and the interactive controller path.
"""
from __future__ import annotations
from pathlib import Path
import importlib.util
import sys
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('teleop_validation', ROOT / 'teleop_keyboard.py')
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
base = t.BaseDriver(model, data, 2.0, np.deg2rad(120), 0.00064, '+x', [1.0], 'jointvel')
right = arms['right']
spoon = right.spoon_body
gripper = right.gripper_base_body

for _ in range(500):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    mujoco.mj_step(model, data)
    base.post_step_hold()

spoon_z0 = float(data.xpos[spoon, 2])
gripper_p0 = data.xpos[gripper].copy()
t.start_gripper_closing(right)
for _ in range(350):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    t.update_gripper(
        model, data, right,
        model.opt.timestep * 8.0,
        1.0, 18.0, 0.2, 8, 0.5, 0.005, False,
    )
    mujoco.mj_step(model, data)
    base.post_step_hold()

left, right_count, dims, friction, by_geom = t.spoon_pad_contact_summary(model, data, right)
command = float(data.ctrl[right.gripper_actuator])
q = float(data.qpos[model.jnt_qposadr[right.gripper_joint]])
rel0 = data.xpos[spoon] - data.xpos[gripper]
target_q = t.mat_to_quat(data.xmat[right.tcp_body].reshape(3, 3))

# Safe runtime default: 0.25 m/s, no wall-time compensation.
for _ in range(1200):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    t.hard_hold_arm(model, data, arms['left'])
    # Exercise the actual interactive U/PageUp path in the default base frame.
    operator_twist = np.array([0.0, 0.0, 0.25, 0.0, 0.0, 0.0])
    twist = t.map_arm_operator_twist_to_world(
        None, data, right.tcp_body, base.base_body, operator_twist,
        "base", "tcp", "+x",
    )
    twist = t.clamp_tcp_twist(model, twist, 0.0005, np.deg2rad(1.5))
    twist[3:] += 4.0 * t.rotation_error(target_q, data.xmat[right.tcp_body].reshape(3, 3))
    t.apply_twist_ik(model, data, right, twist, 0.03, 2.0, 0.030)
    t.update_gripper(
        model, data, right,
        model.opt.timestep * 8.0,
        1.0, 18.0, 0.2, 8, 0.5, 0.005, False,
    )
    mujoco.mj_step(model, data)
    base.post_step_hold()

spoon_lift = float(data.xpos[spoon, 2] - spoon_z0)
gripper_lift = float(data.xpos[gripper, 2] - gripper_p0[2])
relative_error = float(np.linalg.norm((data.xpos[spoon] - data.xpos[gripper]) - rel0))
final_left, final_right, final_dims, _, _ = t.spoon_pad_contact_summary(model, data, right)
ok = (
    abs(command - 0.735) < 1e-10
    and left > 0 and right_count > 0 and 6 in dims
    and final_left > 0 and final_right > 0 and 6 in final_dims
    and spoon_lift > 0.18 and gripper_lift > 0.18
    and relative_error < 0.008
)
print(f'timestep={model.opt.timestep:.4f} s')
print(f'command={command:.6f} rad, measured_q={q:.6f} rad')
print(f'initial contacts left/right={left}/{right_count}, dimensions={sorted(dims)}, friction={friction}, by_geom={by_geom}')
print(f'final contacts left/right={final_left}/{final_right}')
print(f'spoon_lift={spoon_lift:.6f} m, gripper_lift={gripper_lift:.6f} m')
print(f'relative_error={relative_error:.9f} m')
print(f'TABLE-SUPPORTED RUNTIME GRASP/LIFT: {"PASS" if ok else "FAIL"}')
raise SystemExit(0 if ok else 1)
