#!/usr/bin/env python3
"""Structural collision audit for all full coffee-scene XML variants."""
from __future__ import annotations
from pathlib import Path
import xml.etree.ElementTree as ET
from collections import Counter

ROOT = Path(__file__).resolve().parent
ROBOT = ROOT / "robot_task3_small_model_mesh.xml"
SCENES = [
    ROOT / "final_scene_100_beans_mesh.xml",
    ROOT / "final_scene_300_beans_mesh.xml",
    ROOT / "final_scene_100_beans_primitives.xml",
    ROOT / "final_scene_300_beans_primitives.xml",
]

def mask_contact(a, b):
    act, aca = int(a.get("contype", "1")), int(a.get("conaffinity", "1"))
    bct, bca = int(b.get("contype", "1")), int(b.get("conaffinity", "1"))
    return bool((act & bca) or (bct & aca))

def category(name, material=''):
    n = name.lower()
    if material == 'collision_wall_mat': return 'walls/doors'
    if material == 'collision_furniture_mat': return 'tables/furniture'
    if "coffee_bean" in n: return "coffee beans"
    if "bowl" in n and "spoon" not in n: return "bowl"
    if "plate" in n: return "plate"
    if "spoon" in n: return "spoon"
    if "wall" in n or "door" in n: return "walls/doors"
    if "floor" in n: return "floor"
    if "table" in n or "rectangle" in n or "furniture" in n: return "tables/furniture"
    return "other physical scene geoms"

rr = ET.parse(ROBOT).getroot()
env = [g for g in rr.findall('.//geom') if g.get('name','').startswith('right_actual_') and g.get('name','').endswith('_environment_collision')]
grasp = [rr.find(".//geom[@name='right_grasp_left_main']"), rr.find(".//geom[@name='right_grasp_right_main']")]
assert len(env) == 9 and all(g is not None for g in grasp)
assert all(g.get('type') == 'mesh' for g in env + grasp)

for scene in SCENES:
    root = ET.parse(scene).getroot()
    include = root.find(".//include")
    assert include is not None and include.get('file') == 'robot_task3_small_model_mesh.xml'
    weld = root.find("equality/weld[@name='right_gripper_flange_weld']")
    assert weld is not None
    opt = root.find('option')
    expected = {
        'timestep':'0.001', 'integrator':'implicitfast', 'solver':'Newton',
        'iterations':'100', 'ls_iterations':'20', 'cone':'elliptic', 'impratio':'10'
    }
    assert opt is not None and all(opt.get(k) == v for k,v in expected.items()), (scene.name, opt.attrib)
    act = root.find("actuator/position[@name='right_fr3v2_1_robotiq_85_left_knuckle_joint']")
    assert act is not None and act.get('kp') == '500' and act.get('kv') == '30' and act.get('forcerange') == '-100 100'

    mesh_assets = {m.get('name') for m in root.findall('asset/mesh')}
    assert {f'spoon_stem_mesh_{i:02d}' for i in range(14)} <= mesh_assets

    counts = Counter()
    failures = []
    spoon_collision_count = 0
    for geom in root.findall('.//worldbody//geom'):
        name = geom.get('name', '<unnamed>')
        if int(geom.get('contype','1')) == 0:
            continue
        cat = category(name, geom.get('material',''))
        counts[cat] += 1
        if cat == 'spoon':
            spoon_collision_count += 1
            grasp_ok = any(mask_contact(g, geom) for g in grasp)
            env_duplicate = any(mask_contact(g, geom) for g in env)
            if not grasp_ok or env_duplicate:
                failures.append((name, f'exact-mesh grasp={grasp_ok} ordinary-gripper-duplicate={env_duplicate}'))
        else:
            env_ok = any(mask_contact(g, geom) for g in env)
            grasp_duplicate = any(mask_contact(g, geom) for g in grasp)
            if not env_ok or grasp_duplicate:
                failures.append((name, f'environment contact={env_ok} grasp-mesh-duplicate={grasp_duplicate}'))
    assert spoon_collision_count >= 17, spoon_collision_count  # 14 stem meshes + neck + two bowl ellipsoids
    if failures:
        raise SystemExit(f'{scene.name}: collision audit failed: {failures[:20]}')
    print(scene.name)
    for cat, count in sorted(counts.items()):
        print(f'  {cat}: {count} collision geoms compatible')
    print('  segmented mesh spoon, original fingertip STL grasp, solver and preload: PASS')
print('COLLISION AUDIT: PASS')
