#!/usr/bin/env python3
"""Validate bounded arm targets and non-accumulating, stable spine hold."""
from __future__ import annotations
from pathlib import Path
import importlib.util
import sys
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("teleop_no_accum", ROOT / "teleop_keyboard.py")
t = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = t
spec.loader.exec_module(t)

model = mujoco.MjModel.from_xml_path(str(ROOT / "benchmark_task3_mesh_table.xml"))
data = mujoco.MjData(model)
t.apply_ready_pose(model, data)
mujoco.mj_forward(model, data)
arms = {side: t.create_arm(model, data, side) for side in ("left", "right")}
for arm in arms.values():
    t.open_gripper(model, data, arm)
    t.synchronize_arm(model, data, arm)
right = arms["right"]

# Arm target is rebuilt from measured q, not from the previous target.
twist = np.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0], dtype=float)
qdot = t.apply_twist_ik(model, data, right, twist, 0.03, 2.0, 0.030, 0.040)
measured_q = np.array(
    [data.qpos[int(model.jnt_qposadr[int(jid)])] for jid in right.joint_ids],
    dtype=float,
)
first_arm_target = right.q_ref.copy()
for _ in range(100):
    t.apply_twist_ik(model, data, right, twist, 0.03, 2.0, 0.030, 0.040)
arm_target_drift = float(np.max(np.abs(right.q_ref - first_arm_target)))
arm_lead = right.q_ref - measured_q
arm_max_lead = float(np.max(np.abs(arm_lead)))
expected_unclipped = qdot * 0.030
expected_bounded = np.clip(expected_unclipped, -0.040, 0.040)
arm_equation_error = float(np.max(np.abs(arm_lead - expected_bounded)))

base = t.BaseDriver(
    model=model,
    data=data,
    base_speed=2.0,
    yaw_speed=np.deg2rad(120.0),
    spine_speed=0.18,
    forward_axis="+x",
    speed_scale=[1.0],
    control_mode="actuator",
    spine_target_lookahead=0.100,
    spine_release_brake_lookahead=0.025,
)

# Repeated held-key calculations without stepping must produce one identical
# bounded target, irrespective of variable wall-clock dt.
base._update_spine(1.0, 0.001)
first_spine_target = float(base.spine_target)
for dt in (0.016, 0.004, 0.010, 0.001) * 25:
    base._update_spine(1.0, dt)
spine_active_target_drift = abs(float(base.spine_target) - first_spine_target)
measured_spine = float(data.qpos[base.spine_qpos_address])
expected_lead = 0.18 * 0.100
actual_lead = float(base.spine_target - measured_spine)

# Dynamic release test: raise, release, brake without reversal, then hold a
# fixed physical reference under the current weld/contact load.
for _ in range(500):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    mujoco.mj_step(model, data)
    base.post_step_hold()
for _ in range(500):
    base.drive(0, 0, 1, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    mujoco.mj_step(model, data)
    base.post_step_hold()
release_q = float(data.qpos[base.spine_qpos_address])
positions = [release_q]
max_qacc = 0.0
for _ in range(1000):
    base.drive(0, 0, 0, 0, model.opt.timestep)
    for arm in arms.values():
        t.hard_hold_arm(model, data, arm)
    mujoco.mj_step(model, data)
    base.post_step_hold()
    positions.append(float(data.qpos[base.spine_qpos_address]))
    max_qacc = max(max_qacc, float(np.max(np.abs(data.qacc))))

peak_q = max(positions)
final_q = positions[-1]
peak_to_final_rollback = peak_q - final_q
hold_reference_error = abs(final_q - float(base.spine_hold_reference))

ok = (
    arm_target_drift < 1e-12
    and arm_max_lead <= 0.040 + 1e-12
    and arm_equation_error < 1e-12
    and spine_active_target_drift < 1e-12
    and abs(actual_lead - expected_lead) < 1e-12
    and peak_to_final_rollback < 0.0002
    and hold_reference_error < 0.0002
    and np.isfinite(max_qacc)
    and max_qacc < 1e9
)
print(f"arm repeated-command target drift: {arm_target_drift:.3e} rad")
print(f"arm maximum bounded lead: {arm_max_lead:.6f} rad")
print(f"arm q_ref equation error: {arm_equation_error:.3e} rad")
print(f"spine held-key target drift: {spine_active_target_drift:.3e} m")
print(f"spine target lead: {actual_lead:.6f} m (expected {expected_lead:.6f} m)")
print(f"spine release-to-peak travel: {peak_q-release_q:.6f} m")
print(f"spine peak-to-held rollback: {peak_to_final_rollback:.9f} m")
print(f"spine held-reference error: {hold_reference_error:.3e} m")
print(f"max |qacc|: {max_qacc:.3e}")
print(f"BOUNDED TARGETS AND RELEASE HOLD: {'PASS' if ok else 'FAIL'}")
raise SystemExit(0 if ok else 1)
