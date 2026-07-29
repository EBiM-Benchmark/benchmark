#!/usr/bin/env python3
"""Validate the rigid two-part spoon hierarchy in every scene XML."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
FILES = [
    'benchmark_task3_mesh.xml',
    'benchmark_task3_mesh_table.xml',
    'final_scene_100_beans_mesh.xml',
    'final_scene_100_beans_mesh_spoon_mocap_runtime.xml',
    'final_scene_100_beans_primitives.xml',
    'final_scene_100_beans_primitives_spoon_mocap_runtime.xml',
    'final_scene_300_beans_mesh.xml',
    'final_scene_300_beans_mesh_spoon_mocap_runtime.xml',
    'final_scene_300_beans_primitives.xml',
    'final_scene_300_beans_primitives_spoon_mocap_runtime.xml',
]
EXPECTED_SCOOP = {
    'contype':'1', 'conaffinity':'16', 'condim':'6',
    'friction':'1.0 0.02 0.002', 'margin':'0',
    'solref':'0.008 1', 'solimp':'0.96 0.995 0.001 0.5 2',
    'priority':'0',
}

for filename in FILES:
    root = ET.parse(ROOT / filename).getroot()
    spoon = next(b for b in root.iter('body') if b.get('name') == 'dynamic_spoon2')
    handle = spoon.find("./body[@name='spoon_handle_neck_part']")
    scoop = spoon.find("./body[@name='spoon_scoop_part']")
    assert handle is not None, filename
    assert scoop is not None, filename
    # No joints in either child means an exact rigid parent-child connection.
    assert not handle.findall('./joint') and handle.find('./freejoint') is None
    assert not scoop.findall('./joint') and scoop.find('./freejoint') is None
    handle_geoms = handle.findall('./geom')
    scoop_geoms = scoop.findall('./geom')
    handle_names = [g.get('name','') for g in handle_geoms]
    assert len([n for n in handle_names if n.startswith('spoon_collision_original_stem_mesh_')]) == 14
    assert handle_names.count('spoon_collision_neck') == 1
    assert len(handle_geoms) == 15
    assert len(scoop_geoms) == 139
    assert all(g.get('name','').startswith('collision_spoon_meshfit_') for g in scoop_geoms)
    for geom in scoop_geoms:
        for key, value in EXPECTED_SCOOP.items():
            assert geom.get(key) == value, (filename, geom.get('name'), key, geom.get(key), value)
    direct_collision = [g.get('name') for g in spoon.findall('./geom') if g.get('contype','1') != '0']
    assert not direct_collision, (filename, direct_collision)
    assert spoon.find('./freejoint') is not None
    assert spoon.find('./inertial') is not None
    print(f'{filename}: handle/neck=15, scoop=139, fixed child bodies — PASS')

print('TWO-PART SPOON STRUCTURE: PASS')
