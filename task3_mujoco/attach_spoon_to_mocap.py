#!/usr/bin/env python3
"""Attach the dynamic spoon to a free MuJoCo mocap body and control it by keyboard.

This script does not permanently modify final_scene.xml.  At startup it creates a
runtime XML next to the scene file with:

  1. a free mocap body named ``spoon_mocap``
  2. a weld equality constraint from ``spoon_mocap`` to ``dynamic_spoon2``

The spoon remains a dynamic body with its collision geoms, but the mocap weld lets
us move it directly for debugging the spoon/bowl/bean contacts.

Run from inside the generated package folder:

    python attach_spoon_to_mocap.py --xml final_scene.xml --show-collisions

Controls:
    W/S or Up/Down       move mocap +x / -x
    A/D or Left/Right    move mocap -y / +y
    Q/E or PgUp/PgDown   move mocap +z / -z
    J/L                  yaw + / -
    I/K                  pitch + / -
    U/O                  roll + / -
    F                    toggle translation frame: world / spoon-local
    R                    reset mocap target from current spoon pose
    C                    show/hide collision geoms (group 3)
    M                    show/hide contact points and forces
    H                    print help
"""

from __future__ import annotations

import argparse
import math
import queue
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import glfw
import mujoco
import mujoco.viewer
import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_XML = HERE / "final_scene_300_beans.xml"
RUNTIME_XML_NAME = "final_scene_spoon_mocap_runtime.xml"

SPOON_BODY = "dynamic_spoon2"
MOCAP_BODY = "spoon_mocap"
WELD_NAME = "spoon_mocap_weld"

# Change these values to make spoon mocap increments smaller/larger.
TRANSLATION_STEP = 0.015          # meters per key press
ROTATION_STEP = math.radians(7.5) # radians per key press
SIM_STEPS_PER_FRAME = 3
UI_HZ = 60.0

HELP = f"""
Spoon mocap controller

The script creates a temporary XML with a free mocap body welded to {SPOON_BODY}.
Move the mocap body to move the spoon.

Translation:
  W/S or Up/Down       : +x / -x
  A/D or Left/Right    : -y / +y
  Q/E or PgUp/PgDown   : +z / -z

Rotation:
  J/L                  : yaw + / -
  I/K                  : pitch + / -
  U/O                  : roll + / -

Frame/debug:
  F                    : toggle translation frame: world / spoon-local
  R                    : reset mocap target from current spoon pose
  C                    : show/hide collision geom group 3
  M                    : show/hide contact points and forces
  H                    : print this help

Step sizes:
  TRANSLATION_STEP = {TRANSLATION_STEP:.3f} m per key press
  ROTATION_STEP    = {math.degrees(ROTATION_STEP):.1f} deg per key press
"""

COMMAND_KEYS = {
    glfw.KEY_W, glfw.KEY_S, glfw.KEY_A, glfw.KEY_D, glfw.KEY_Q, glfw.KEY_E,
    glfw.KEY_UP, glfw.KEY_DOWN, glfw.KEY_LEFT, glfw.KEY_RIGHT,
    glfw.KEY_PAGE_UP, glfw.KEY_PAGE_DOWN,
    glfw.KEY_J, glfw.KEY_L, glfw.KEY_I, glfw.KEY_K, glfw.KEY_U, glfw.KEY_O,
    glfw.KEY_F, glfw.KEY_R, glfw.KEY_C, glfw.KEY_M, glfw.KEY_H,
}


def name2id(model: mujoco.MjModel, objtype: mujoco.mjtObj, name: str) -> int:
    idx = mujoco.mj_name2id(model, objtype, name)
    if idx < 0:
        raise RuntimeError(f"Missing {objtype.name} named {name!r}")
    return int(idx)


class KeyboardQueue:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._commands: queue.SimpleQueue[int] = queue.SimpleQueue()

    def key_callback(self, keycode: int) -> None:
        key = abs(int(keycode))
        pressed = int(keycode) >= 0
        if not pressed:
            return
        if key in COMMAND_KEYS:
            with self._lock:
                self._commands.put(key)

    def get(self) -> list[int]:
        out: list[int] = []
        while True:
            try:
                out.append(self._commands.get_nowait())
            except queue.Empty:
                return out


def mat_to_quat(mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, mat.reshape(9))
    return quat / max(np.linalg.norm(quat), 1e-12)


def quat_to_mat(quat: np.ndarray) -> np.ndarray:
    mat = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(mat, quat)
    return mat.reshape(3, 3)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    q = np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], dtype=np.float64)
    return q / max(np.linalg.norm(q), 1e-12)


def axis_angle_quat(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    a /= max(np.linalg.norm(a), 1e-12)
    half = 0.5 * angle
    return np.array([math.cos(half), *(math.sin(half) * a)], dtype=np.float64)


def rotate_local(q: np.ndarray, axis: tuple[float, float, float], angle: float) -> np.ndarray:
    # Local rotation: post-multiply by axis-angle increment.
    return quat_mul(q, axis_angle_quat(axis, angle))


def ensure_spoon_mocap_xml(src_xml: Path, dst_xml: Path) -> None:
    """Write a runtime XML containing a mocap body welded to the spoon body."""
    tree = ET.parse(src_xml)
    root = tree.getroot()
    world = root.find('worldbody')
    if world is None:
        raise RuntimeError('XML has no <worldbody>')
    spoon = world.find(f".//body[@name='{SPOON_BODY}']")
    if spoon is None:
        raise RuntimeError(f"Cannot find spoon body {SPOON_BODY!r} in XML")

    # Initial mocap pose is the initial spoon body pose in world coordinates.  In
    # this scene dynamic_spoon2 is a direct child of worldbody, so its pos is a
    # world-frame pose.
    spoon_pos = spoon.get('pos', '0 0 0')
    spoon_quat = spoon.get('quat', '1 0 0 0')

    # Add/remove existing generated mocap body to avoid duplicates.
    for body in list(world.findall(f"body[@name='{MOCAP_BODY}']")):
        world.remove(body)

    mocap_body = ET.Element('body', {
        'name': MOCAP_BODY,
        'mocap': 'true',
        'pos': spoon_pos,
        'quat': spoon_quat,
    })
    ET.SubElement(mocap_body, 'geom', {
        'name': 'spoon_mocap_marker',
        'type': 'sphere',
        'size': '0.012',
        'rgba': '1 0 1 0.45',
        'contype': '0',
        'conaffinity': '0',
        'group': '4',
    })
    world.insert(0, mocap_body)

    equality = root.find('equality')
    if equality is None:
        equality = ET.Element('equality')
        root.append(equality)
    for weld in list(equality.findall(f"weld[@name='{WELD_NAME}']")):
        equality.remove(weld)
    ET.SubElement(equality, 'weld', {
        'name': WELD_NAME,
        'body1': MOCAP_BODY,
        'body2': SPOON_BODY,
        # Stiff but not infinitely hard; increase stiffness by lowering first solref value.
        'solref': '0.015 1',
        'solimp': '0.90 0.98 0.001',
    })

    tree.write(dst_xml, encoding='utf-8', xml_declaration=True)


def set_mocap_from_spoon(model: mujoco.MjModel, data: mujoco.MjData, mocap_id: int, spoon_body_id: int) -> None:
    mujoco.mj_forward(model, data)
    data.mocap_pos[mocap_id] = data.xpos[spoon_body_id].copy()
    data.mocap_quat[mocap_id] = data.xquat[spoon_body_id].copy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Control the spoon through a free MuJoCo mocap body')
    parser.add_argument('--xml', type=Path, default=DEFAULT_XML, help='Base MuJoCo XML file')
    parser.add_argument('--runtime-xml', type=Path, default=None, help='Path for generated runtime XML')
    parser.add_argument('--show-collisions', action='store_true', help='show collision geom group 3 at startup')
    parser.add_argument('--sim-steps', type=int, default=SIM_STEPS_PER_FRAME, help='MuJoCo steps per viewer frame')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    src_xml = args.xml.expanduser().resolve()
    if not src_xml.exists():
        raise FileNotFoundError(src_xml)
    runtime_xml = args.runtime_xml or (src_xml.parent / f'{src_xml.stem}_spoon_mocap_runtime.xml')
    ensure_spoon_mocap_xml(src_xml, runtime_xml)

    model = mujoco.MjModel.from_xml_path(str(runtime_xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    mocap_body_id = name2id(model, mujoco.mjtObj.mjOBJ_BODY, MOCAP_BODY)
    spoon_body_id = name2id(model, mujoco.mjtObj.mjOBJ_BODY, SPOON_BODY)
    mocap_id = int(model.body_mocapid[mocap_body_id])
    if mocap_id < 0:
        raise RuntimeError(f'{MOCAP_BODY} is not a mocap body')

    set_mocap_from_spoon(model, data, mocap_id, spoon_body_id)
    frame = 'world'
    input_queue = KeyboardQueue()

    print(HELP)
    print(f'Loaded runtime XML: {runtime_xml}')
    print('Translation frame:', frame)

    with mujoco.viewer.launch_passive(model, data, key_callback=input_queue.key_callback) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -45
        viewer.opt.geomgroup[3] = 1 if args.show_collisions else 0
        viewer.opt.geomgroup[4] = 1
        show_contacts = False
        last = time.perf_counter()

        while viewer.is_running():
            now = time.perf_counter()
            dt = now - last
            last = now

            for key in input_queue.get():
                if key == glfw.KEY_H:
                    print(HELP)
                    continue
                if key == glfw.KEY_C:
                    viewer.opt.geomgroup[3] = 0 if viewer.opt.geomgroup[3] else 1
                    continue
                if key == glfw.KEY_M:
                    show_contacts = not show_contacts
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = show_contacts
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = show_contacts
                    continue
                if key == glfw.KEY_F:
                    frame = 'spoon' if frame == 'world' else 'world'
                    set_mocap_from_spoon(model, data, mocap_id, spoon_body_id)
                    print('Translation frame:', frame)
                    continue
                if key == glfw.KEY_R:
                    set_mocap_from_spoon(model, data, mocap_id, spoon_body_id)
                    data.qvel[:] = 0.0
                    print('Reset mocap target from current spoon pose')
                    continue

                delta = np.zeros(3, dtype=np.float64)
                if key in (glfw.KEY_W, glfw.KEY_UP):
                    delta[0] += TRANSLATION_STEP
                elif key in (glfw.KEY_S, glfw.KEY_DOWN):
                    delta[0] -= TRANSLATION_STEP
                elif key in (glfw.KEY_A, glfw.KEY_LEFT):
                    delta[1] -= TRANSLATION_STEP
                elif key in (glfw.KEY_D, glfw.KEY_RIGHT):
                    delta[1] += TRANSLATION_STEP
                elif key in (glfw.KEY_Q, glfw.KEY_PAGE_UP):
                    delta[2] += TRANSLATION_STEP
                elif key in (glfw.KEY_E, glfw.KEY_PAGE_DOWN):
                    delta[2] -= TRANSLATION_STEP

                if np.linalg.norm(delta) > 0.0:
                    if frame == 'spoon':
                        delta = quat_to_mat(data.mocap_quat[mocap_id]) @ delta
                    data.mocap_pos[mocap_id] += delta
                    continue

                q = data.mocap_quat[mocap_id].copy()
                if key == glfw.KEY_J:
                    q = rotate_local(q, (0, 0, 1), +ROTATION_STEP)
                elif key == glfw.KEY_L:
                    q = rotate_local(q, (0, 0, 1), -ROTATION_STEP)
                elif key == glfw.KEY_I:
                    q = rotate_local(q, (0, 1, 0), +ROTATION_STEP)
                elif key == glfw.KEY_K:
                    q = rotate_local(q, (0, 1, 0), -ROTATION_STEP)
                elif key == glfw.KEY_U:
                    q = rotate_local(q, (1, 0, 0), +ROTATION_STEP)
                elif key == glfw.KEY_O:
                    q = rotate_local(q, (1, 0, 0), -ROTATION_STEP)
                data.mocap_quat[mocap_id] = q / max(np.linalg.norm(q), 1e-12)

            for _ in range(max(1, int(args.sim_steps))):
                mujoco.mj_step(model, data)

            viewer.sync()
            elapsed = time.perf_counter() - now
            sleep_time = max(0.0, (1.0 / UI_HZ) - elapsed)
            if sleep_time > 0.0:
                time.sleep(sleep_time)


if __name__ == '__main__':
    main()
