#!/usr/bin/env python3
"""Generate final_scene_300_beans.xml from the validated 100-bean scene."""
from __future__ import annotations
import argparse, math, random, re
from pathlib import Path

BOWL_CENTER=(-0.613405,0.118512)
FIRST_LAYER_Z=1.1116
LAYER_SPACING=0.0068
LAYERS=6
BEANS_PER_LAYER=50
BASE_RADIUS=0.0310
RADIUS_GROWTH=0.0011
MIN_DISTANCE=0.0061
SEED=300
BODY_RE=re.compile(r'\s*<body name="dynamic_coffee_bean_\d{4}".*?</body>\n',re.DOTALL)

def quat_xyz(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2); cp,sp=math.cos(pitch/2),math.sin(pitch/2); cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return (cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy)

def points(rng,radius,count):
    out=[]; d2=MIN_DISTANCE**2
    for _ in range(500000):
        if len(out)==count: return out
        r=radius*math.sqrt(rng.random()); a=2*math.pi*rng.random(); p=(r*math.cos(a),r*math.sin(a))
        if all((p[0]-q[0])**2+(p[1]-q[1])**2>=d2 for q in out): out.append(p)
    raise RuntimeError(f'Could place only {len(out)}/{count} beans')

def bean(i,x,y,z,q):
    n=f'{i:04d}'; qs=' '.join(f'{v:.8f}' for v in q)
    lines=[
      f'    <body name="dynamic_coffee_bean_{n}" pos="{x:.8f} {y:.8f} {z:.8f}">',
      f'      <freejoint name="dynamic_coffee_bean_{n}_freejoint"/>',
      '      <inertial pos="0 0 0" mass="0.00018000" diaginertia="1.0e-8 1.0e-8 6.0e-9"/>',
      f'      <geom name="visual_coffee_bean_{n}" type="mesh" mesh="coffee_bean_visual_mesh" quat="{qs}" material="coffee_bean_mat" group="1" contype="0" conaffinity="0" density="0"/>',
      f'      <geom name="collision_coffee_bean_{n}" type="capsule" quat="{qs}" size="0.002500 0.001600" group="3" contype="1" conaffinity="7" condim="3" friction="1.25 0.02 0.002" solref="0.009 1" solimp="0.90 0.98 0.001" rgba="0.35 0.16 0.05 0.25" density="0"/>',
      '    </body>',
    ]
    return '\n'.join(lines)+'\n'

def generate(source,output):
    text=Path(source).read_text(); matches=list(BODY_RE.finditer(text))
    if len(matches)!=100: raise RuntimeError(f'Expected 100 source beans, found {len(matches)}')
    rng=random.Random(SEED); blocks=[]; idx=0; cx,cy=BOWL_CENTER
    for layer in range(LAYERS):
        angle=layer*math.radians(17); ca,sa=math.cos(angle),math.sin(angle)
        for px,py in points(rng,BASE_RADIUS+RADIUS_GROWTH*layer,BEANS_PER_LAYER):
            rx,ry=ca*px-sa*py,sa*px+ca*py
            x=cx+rx+rng.uniform(-0.00018,0.00018); y=cy+ry+rng.uniform(-0.00018,0.00018); z=FIRST_LAYER_Z+layer*LAYER_SPACING+rng.uniform(-0.00016,0.00016)
            q=quat_xyz(math.pi/2+rng.uniform(-0.18,0.18),rng.uniform(-0.15,0.15),rng.uniform(-math.pi,math.pi))
            blocks.append(bean(idx,x,y,z,q)); idx+=1
    Path(output).write_text(text[:matches[0].start()]+''.join(blocks)+text[matches[-1].end():])

def main():
    h=Path(__file__).resolve().parent; p=argparse.ArgumentParser(); p.add_argument('--source',type=Path,default=h/'final_scene_100_beans.xml'); p.add_argument('--output',type=Path,default=h/'final_scene_300_beans.xml'); a=p.parse_args(); generate(a.source,a.output); print(f'Wrote {a.output} with 300 beans')
if __name__=='__main__': main()
