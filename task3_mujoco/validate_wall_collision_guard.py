#!/usr/bin/env python3
"""Drive the right arm into a fixed test wall and verify anti-windup stability."""
from __future__ import annotations
from pathlib import Path
import importlib.util
import sys
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('teleop_wall_guard',ROOT/'teleop_keyboard.py')
t=importlib.util.module_from_spec(spec);sys.modules[spec.name]=t;spec.loader.exec_module(t)
model=mujoco.MjModel.from_xml_path(str(ROOT/'benchmark_wall_guard.xml'))
data=mujoco.MjData(model)
t.apply_ready_pose(model,data);mujoco.mj_forward(model,data)
arms={side:t.create_arm(model,data,side) for side in ('left','right')}
for arm in arms.values():
    t.open_gripper(model,data,arm);t.synchronize_arm(model,data,arm)
base=t.BaseDriver(model,data,2.0,np.deg2rad(120),0.18,'+x',[1.0],'actuator')
right=arms['right']

for _ in range(300):
    base.drive(0,0,0,0,model.opt.timestep)
    for arm in arms.values(): t.hard_hold_arm(model,data,arm)
    mujoco.mj_step(model,data);base.post_step_hold()

first_contact_step=None
max_qacc=0.0
contact_count=0
qref_error_max=0.0
max_tcp_y_after_contact=-np.inf
min_tcp_y_after_contact=np.inf

# Hold a command into the wall. The latch must prevent continued integration.
for step in range(1000):
    base.drive(0,0,0,0,model.opt.timestep)
    t.hard_hold_arm(model,data,arms['left'])
    operator_active=True
    wall_contacts=t.arm_static_wall_contacts(model,data,right)
    command_active=True
    if right.wall_contact_active:
        t.synchronize_arm(model,data,right)
        t.hard_hold_arm(model,data,right)
        command_active=False
    elif wall_contacts:
        right.wall_contact_active=True
        t.synchronize_arm(model,data,right)
        t.hard_hold_arm(model,data,right)
        command_active=False
    if command_active:
        twist=np.array([0.0,0.20,0.0,0.0,0.0,0.0])
        twist=t.clamp_tcp_twist(model,twist,0.0005,np.deg2rad(1.5))
        t.apply_twist_ik(model,data,right,twist,0.03,2.0)
    mujoco.mj_step(model,data);base.post_step_hold()
    after=t.arm_static_wall_contacts(model,data,right)
    if after:
        contact_count=max(contact_count,len(after))
        if first_contact_step is None:
            first_contact_step=step
        right.wall_contact_active=True
    if right.wall_contact_active:
        t.synchronize_arm(model,data,right)
        t.hard_hold_arm(model,data,right)
    if first_contact_step is not None:
        y=float(data.site_xpos[right.tcp_site,1])
        max_tcp_y_after_contact=max(max_tcp_y_after_contact,y)
        min_tcp_y_after_contact=min(min_tcp_y_after_contact,y)
    q_meas=np.array([data.qpos[int(model.jnt_qposadr[j])] for j in right.joint_ids])
    if first_contact_step is not None:
        qref_error_max=max(qref_error_max,float(np.max(np.abs(right.q_ref-q_meas))))
    max_qacc=max(max_qacc,float(np.max(np.abs(data.qacc))))

# Release the key so the latch is allowed to clear after contact relaxation.
for _ in range(300):
    base.drive(0,0,0,0,model.opt.timestep)
    t.hard_hold_arm(model,data,arms['left'])
    wall_contacts=t.arm_static_wall_contacts(model,data,right)
    if right.wall_contact_active:
        t.synchronize_arm(model,data,right)
        t.hard_hold_arm(model,data,right)
        if not wall_contacts:
            right.wall_contact_active=False
    else:
        t.hard_hold_arm(model,data,right)
    mujoco.mj_step(model,data);base.post_step_hold()
    q_meas=np.array([data.qpos[int(model.jnt_qposadr[j])] for j in right.joint_ids])
    qref_error_max=max(qref_error_max,float(np.max(np.abs(right.q_ref-q_meas))))
    max_qacc=max(max_qacc,float(np.max(np.abs(data.qacc))))

post_contact_tcp_span=max_tcp_y_after_contact-min_tcp_y_after_contact if first_contact_step is not None else np.inf
ok=(first_contact_step is not None and contact_count>0
    and post_contact_tcp_span<0.01
    and qref_error_max<0.03
    and np.isfinite(max_qacc) and max_qacc<1e8)
print(f'first wall contact step={first_contact_step}')
print(f'max simultaneous wall contacts={contact_count}')
print(f'TCP Y span after first contact={post_contact_tcp_span:.6f} m')
print(f'max |q_ref-q|_inf={qref_error_max:.6f} rad')
print(f'max |qacc|={max_qacc:.3e}')
print(f'collision latch cleared after key release={not right.wall_contact_active}')
print(f'WALL COLLISION ANTI-WINDUP: {"PASS" if ok else "FAIL"}')
raise SystemExit(0 if ok else 1)
