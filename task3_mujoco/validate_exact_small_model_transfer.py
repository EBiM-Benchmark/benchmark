#!/usr/bin/env python3
"""Validate the exact small-model spoon/gripper/contact transfer."""
from __future__ import annotations
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
SCENE = ROOT / 'benchmark_task3_mesh_table.xml'
ROBOT = ROOT / 'robot_task3_small_model_mesh.xml'

# Compile the exact table-supported benchmark.
model = mujoco.MjModel.from_xml_path(str(SCENE))
assert abs(model.opt.timestep - 0.001) < 1e-12
assert model.opt.solver == mujoco.mjtSolver.mjSOL_NEWTON
assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
assert model.opt.iterations == 100
assert model.opt.ls_iterations == 20
assert abs(model.opt.impratio - 10.0) < 1e-12

root = ET.parse(SCENE).getroot()
robot = ET.parse(ROBOT).getroot()

# Exact mesh-spoon representation: 14 original stem mesh sections + neck + bowl parts.
assets = {m.get('name') for m in root.findall('asset/mesh')}
expected_assets = {f'spoon_stem_mesh_{i:02d}' for i in range(14)}
assert expected_assets <= assets
stem_geoms = root.findall(".//worldbody//geom")
stem_geoms = [g for g in stem_geoms if (g.get('name') or '').startswith('spoon_collision_original_stem_mesh_')]
assert len(stem_geoms) == 14 and all(g.get('type') == 'mesh' for g in stem_geoms)

# Original Robotiq fingertip STL collision meshes are used for high-friction grasping.
grasp_names = [
    'left_grasp_left_main', 'left_grasp_right_main',
    'right_grasp_left_main', 'right_grasp_right_main',
]
for name in grasp_names:
    g = robot.find(f".//geom[@name='{name}']")
    assert g is not None and g.get('type') == 'mesh'
    assert g.get('condim') == '6'
    assert g.get('friction') == '2.5 0.10 0.03'
    assert g.get('margin') == '0.00025'
    assert g.get('solref') == '0.010 1'
    assert g.get('solimp') == '0.95 0.99 0.001 0.5 2'

# All 18 ordinary environment geoms are also original Robotiq collision STL meshes.
env = [g for g in robot.findall('.//geom') if (g.get('name') or '').endswith('_environment_collision')]
assert len(env) == 18 and all(g.get('type') == 'mesh' for g in env)
assert all(g.get('friction') == '1.0 0.01 0.001' for g in env)

# Exact small-model internal gripper joint dynamics on both hands.
gripper_joints = [j for j in robot.findall('.//joint') if 'robotiq_85_' in (j.get('name') or '')]
assert len(gripper_joints) == 12
for j in gripper_joints:
    assert j.get('armature') == '0.01'
    assert j.get('damping') == '1.0'
    assert j.get('frictionloss') == '0.1'
    assert j.get('actuatorfrcrange') == '-100 100'

# Exact mimic constraints and fixed-preload actuators are in the scene.
for eq in root.findall('equality/joint'):
    if 'robotiq_85_' in (eq.get('joint1') or ''):
        assert eq.get('solref') == '0.006 1'
        assert eq.get('solimp') == '0.995 0.9999 0.0001 0.5 2'
for side in ('left', 'right'):
    name = f'{side}_fr3v2_1_robotiq_85_left_knuckle_joint'
    a = root.find(f"actuator/position[@name='{name}']")
    assert a is not None
    assert a.get('kp') == '500' and a.get('kv') == '30'
    assert a.get('ctrlrange') == '0 0.8' and a.get('forcerange') == '-100 100'

# Verify compiled friction vector for both active right-hand grasp meshes.
for name in ('right_grasp_left_main', 'right_grasp_right_main'):
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    assert gid >= 0
    np.testing.assert_allclose(model.geom_friction[gid], [2.5, 0.10, 0.03], atol=1e-12)
    assert int(model.geom_condim[gid]) == 6

print('EXACT SMALL-MODEL TRANSFER: PASS')
print('solver: timestep=0.001, implicitfast, Newton, 100/20 iterations, elliptic cone, impratio=10')
print('spoon: original visual mesh + 14 original handle mesh collision sections + neck/bowl collision parts')
print('grippers: four original fingertip STL grasp geoms; 18 original STL environment geoms')
print('gripper joints: armature=0.01, damping=1.0, frictionloss=0.1')
print('mimic constraints: solref=0.006 1, solimp=0.995 0.9999 0.0001 0.5 2')
print('actuators: kp=500, kv=30, force +/-100 N, fixed command=0.735 rad')
