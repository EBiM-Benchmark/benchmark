#!/usr/bin/env python3
"""Structural audit for safer FR3 position control and dynamic spine settings."""
from __future__ import annotations
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
SCENES = sorted(
    p for p in ROOT.glob('*.xml')
    if p.name.startswith(('final_scene_', 'benchmark_task3_'))
    and p.name != 'benchmark_wall_guard.xml'
)
EXPECTED = {
    1: ('6000', '180', '-87 87'),
    2: ('6000', '180', '-87 87'),
    3: ('5000', '160', '-87 87'),
    4: ('5000', '160', '-87 87'),
    5: ('1800', '70', '-12 12'),
    6: ('1400', '60', '-12 12'),
    7: ('1000', '50', '-12 12'),
}

for path in SCENES:
    root = ET.parse(path).getroot()
    for side in ('left', 'right'):
        for j, (kp, kv, fr) in EXPECTED.items():
            name = f'{side}_fr3v2_1_joint{j}'
            actuator = root.find(f"actuator/position[@name='{name}']")
            assert actuator is not None, (path.name, name, 'not position controlled')
            assert actuator.get('kp') == kp, (path.name, name, actuator.get('kp'))
            assert actuator.get('kv') == kv, (path.name, name, actuator.get('kv'))
            assert actuator.get('forcerange') == fr, (path.name, name, actuator.get('forcerange'))
            assert actuator.get('forcelimited') == 'true'
    spine = root.find("actuator/position[@name='franka_spine_vertical_joint']")
    assert spine is not None
    assert spine.get('kp') == '12000'
    assert spine.get('kv') == '800'
    assert spine.get('forcerange') == '-10000 10000'
    assert spine.get('forcelimited') == 'true'

source = (ROOT / 'teleop_keyboard.py').read_text()
# The only direct spine qpos write must be initialization in apply_ready_pose.
assert 'self.data.qpos[self.spine_qpos_address] =' not in source
assert 'self.data.qvel[self.spine_dof_address] =' not in source
assert 'arm_static_wall_contacts' in source
assert 'wall_contact_active' in source
assert 'data.qfrc_applied[arm.dof_ids] = data.qfrc_bias[arm.dof_ids]' in source
assert 'self.data.qfrc_applied[self.spine_dof_address]' in source

print('SAFE POSITION CONTROL STRUCTURE: PASS')
print(f'audited XML files: {len(SCENES)}')
print('arms: position actuators with moderate gains and FR3-like torque limits')
print('spine: dynamic position actuator; no runtime qpos/qvel overwrite')
print('controller: gravity compensation and static-wall q_ref anti-windup latch enabled')
