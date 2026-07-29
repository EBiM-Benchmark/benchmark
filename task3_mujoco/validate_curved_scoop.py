#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parent
expected_files = [
'benchmark_task3_mesh.xml','benchmark_task3_mesh_table.xml',
'final_scene_100_beans_mesh.xml','final_scene_100_beans_mesh_spoon_mocap_runtime.xml',
'final_scene_100_beans_primitives.xml','final_scene_100_beans_primitives_spoon_mocap_runtime.xml',
'final_scene_300_beans_mesh.xml','final_scene_300_beans_mesh_spoon_mocap_runtime.xml',
'final_scene_300_beans_primitives.xml','final_scene_300_beans_primitives_spoon_mocap_runtime.xml']
expected = {
    'contype':'1', 'conaffinity':'16', 'condim':'6',
    'friction':'1.0 0.02 0.002', 'margin':'0',
    'solref':'0.008 1', 'solimp':'0.96 0.995 0.001 0.5 2',
    'priority':'0',
}
for fn in expected_files:
    p=ROOT/fn
    root=ET.parse(p).getroot()
    spoon=next(b for b in root.iter('body') if b.get('name')=='dynamic_spoon2')
    geoms=list(spoon.findall('.//geom'))
    names=[g.get('name','') for g in geoms]
    stems=[n for n in names if n.startswith('spoon_collision_original_stem_mesh_')]
    neck=[n for n in names if n=='spoon_collision_neck']
    scoop=[g for g in geoms if g.get('name','').startswith('collision_spoon_meshfit_')]
    old=[n for n in names if n in ('spoon_collision_bowl_rear','spoon_collision_bowl_front')]
    assert len(stems)==14, (fn,len(stems))
    assert len(neck)==1, (fn,len(neck))
    assert len(scoop)==139, (fn,len(scoop))
    assert not old, (fn,old)
    for g in scoop:
        for key,value in expected.items():
            assert g.get(key)==value, (fn,g.get('name'),key,g.get(key),value)
    print(f'{fn}: stem/neck retained; 139 curved scoop geoms use working contact settings — PASS')
print('All corrected curved-scoop structural checks passed.')
