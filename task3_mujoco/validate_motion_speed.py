#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path
import importlib.util
import sys
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('teleop_speed_validation',ROOT/'teleop_keyboard.py')
t=importlib.util.module_from_spec(spec);sys.modules[spec.name]=t;spec.loader.exec_module(t)
model=mujoco.MjModel.from_xml_path(str(ROOT/'benchmark_task3_mesh_table.xml'))
data=mujoco.MjData(model)
t.apply_ready_pose(model,data);mujoco.mj_forward(model,data)
base=t.BaseDriver(model,data,2.0,np.deg2rad(120),0.00064,'+x',[1.0],'jointvel')
q0=float(data.qpos[base.planar_qpos_addresses[0]])
for _ in range(100):
    base.drive(1.0,0.0,0.0,0.0,model.opt.timestep,2.0)
    mujoco.mj_step(model,data)
    base.post_step_hold()
dx=float(data.qpos[base.planar_qpos_addresses[0]]-q0)
v=float(data.qvel[base.planar_dof_addresses[0]])
expected=2.0*2.0*model.opt.timestep*100
ok=abs(dx-expected)<0.01 and abs(v-4.0)<0.05
print(f'timestep={model.opt.timestep:.4f} s')
print(f'base command: nominal 2.0 m/s x 2.0 wall-time scale = 4.0 m/s')
print(f'displacement over 100 steps: {dx:.6f} m (expected {expected:.6f} m)')
print(f'final planar velocity: {v:.6f} m/s')
print(f'MOTION SPEED: {"PASS" if ok else "FAIL"}')
raise SystemExit(0 if ok else 1)
