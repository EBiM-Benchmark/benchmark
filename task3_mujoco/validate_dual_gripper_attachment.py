#!/usr/bin/env python3
"""Validate both stiff flange welds during fully dynamic spine motion."""
from __future__ import annotations
from pathlib import Path
import numpy as np
import mujoco
import teleop_keyboard as teleop

HERE = Path(__file__).resolve().parent
SCENE = HERE / "benchmark_task3_mesh.xml"
MOUNT = np.array([0.92388, 0.0, 0.0, -0.382683], dtype=np.float64)


def quat_angle(a, b):
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    return 2.0 * np.arccos(np.clip(abs(float(np.dot(a, b))), -1.0, 1.0))


def errors(model, data, side):
    flange = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_fr3v2_1_link8")
    grip = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_fr3v2_1_robotiq_85_base_link")
    desired_q = teleop.quat_mul(data.xquat[flange], MOUNT)
    desired_q /= np.linalg.norm(desired_q)
    return float(np.linalg.norm(data.xpos[grip] - data.xpos[flange])), float(quat_angle(data.xquat[grip], desired_q))

m = mujoco.MjModel.from_xml_path(str(SCENE))
d = mujoco.MjData(m)
teleop.apply_ready_pose(m, d)
mujoco.mj_forward(m, d)
arms = {side: teleop.create_arm(m, d, side) for side in ("left", "right")}
for arm in arms.values():
    teleop.synchronize_arm(m, d, arm)

driver = teleop.BaseDriver(m, d, 2.0, np.deg2rad(120.0), 0.18, "x", [1.0], "actuator")
max_pos = 0.0
max_rot = 0.0
spine0 = float(d.qpos[driver.spine_qpos_address])

# Settle, then raise dynamically; never project gripper free joints manually.
for _ in range(300):
    driver.drive(0, 0, 0, 0, m.opt.timestep)
    for arm in arms.values(): teleop.hard_hold_arm(m, d, arm)
    mujoco.mj_step(m, d); driver.post_step_hold()
for _ in range(500):
    driver.drive(0, 0, 1, 0, m.opt.timestep)
    for arm in arms.values(): teleop.hard_hold_arm(m, d, arm)
    mujoco.mj_step(m, d); driver.post_step_hold()
    for side in ("left", "right"):
        pe, re = errors(m, d, side)
        max_pos = max(max_pos, pe); max_rot = max(max_rot, re)

spine1 = float(d.qpos[driver.spine_qpos_address])
# Equality welds are stiff but finite; sub-millimeter error is acceptable.
ok = spine1-spine0 > 0.07 and max_pos < 5e-4 and max_rot < np.deg2rad(0.1)
print(f"dynamic spine displacement: {spine1-spine0:.6f} m")
print(f"maximum flange position error: {max_pos*1000:.6f} mm")
print(f"maximum flange orientation error: {np.degrees(max_rot):.6f} deg")
print(f"DYNAMIC DUAL-GRIPPER WELD: {'PASS' if ok else 'FAIL'}")
raise SystemExit(0 if ok else 1)
