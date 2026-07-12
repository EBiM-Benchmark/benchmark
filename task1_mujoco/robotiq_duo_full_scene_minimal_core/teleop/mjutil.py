"""Generic MuJoCo helpers: name/id lookup, frame mapping, contact debugging.
Nothing here knows about arms, grippers or the teleop loop."""

from __future__ import annotations

import math

import numpy as np

import mujoco

from . import log


def obj_id(model: mujoco.MjModel, obj_type, name: str) -> int:
    idx = mujoco.mj_name2id(model, obj_type, name)
    if idx < 0:
        raise RuntimeError(f"Missing {name}")
    return int(idx)


def optional_obj_id(model: mujoco.MjModel, obj_type, name: str) -> int | None:
    idx = mujoco.mj_name2id(model, obj_type, name)
    return None if idx < 0 else int(idx)


def geom_family_ids(model: mujoco.MjModel, prefix: str) -> set[int]:
    """All geom ids whose name starts with ``prefix`` (e.g. one pad + its lips)."""
    ids: set[int] = set()
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith(prefix):
            ids.add(gid)
    return ids


def nearest_bodies_to_point(data: mujoco.MjData, bodies: list[int], point: np.ndarray, count: int = 3) -> list[int]:
    if not bodies:
        return []
    ordered = sorted(bodies, key=lambda bid: float(np.linalg.norm(data.xpos[bid] - point)))
    return ordered[:count]


# --------------------------------------------------------------------------
# frame mapping: operator input axes -> world xy
# --------------------------------------------------------------------------


def planar_body_axis(data: mujoco.MjData, body_id: int | None, axis_name: str) -> np.ndarray:
    """A body's local x/y axis (optionally '-x'/'-y'), projected to the ground
    plane and normalized — the robot's heading for planar driving."""
    if body_id is None:
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        sign = -1.0 if axis_name.startswith("-") else 1.0
        base_axis = axis_name[-1].lower()
        col = {"x": 0, "y": 1}.get(base_axis, 0)
        mat = data.xmat[body_id].reshape(3, 3)
        axis = sign * mat[:, col].copy()
    axis[2] = 0.0
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    return axis / norm


def robot_local_xy_to_world(
    data: mujoco.MjData,
    body_id: int | None,
    local_xy: np.ndarray,
    forward_axis: str,
) -> np.ndarray:
    """(forward, left) input in the robot's heading frame -> world xy."""
    forward = planar_body_axis(data, body_id, forward_axis)
    left = np.cross(np.array([0.0, 0.0, 1.0], dtype=np.float64), forward)
    left /= max(float(np.linalg.norm(left)), 1e-9)
    world = local_xy[0] * forward + local_xy[1] * left
    return np.array([world[0], world[1], 0.0], dtype=np.float64)


def camera_xy_to_world(cam, local_xy: np.ndarray) -> np.ndarray:
    """Map operator-view input (forward=screen-up/away, right=screen-right)
    to world xy using the current viewer camera azimuth, so stick-left always
    moves the TCP toward the operator's left no matter how the robot faces."""
    # MuJoCo free camera (measured via mjv_updateScene's mjvGLCamera):
    # into-screen = [+cos(az), +sin(az)], screen-right = [+sin(az), -cos(az)]
    az = math.radians(float(cam.azimuth))
    forward = np.array([math.cos(az), math.sin(az), 0.0], dtype=np.float64)
    right = np.array([math.sin(az), -math.cos(az), 0.0], dtype=np.float64)
    world = local_xy[0] * forward + local_xy[1] * right
    return np.array([world[0], world[1], 0.0], dtype=np.float64)


# --------------------------------------------------------------------------
# contact debugging
# --------------------------------------------------------------------------


def dump_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Print every active contact with geom names and distance (debug aid)."""
    log(f"[contacts] ncon={data.ncon} nefc={data.nefc}")
    for i in range(data.ncon):
        con = data.contact[i]
        g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(con.geom1)) or f"geom{int(con.geom1)}"
        g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(con.geom2)) or f"geom{int(con.geom2)}"
        log(f"  {i:3d} {g1} <-> {g2} dist={float(con.dist):+.5f}")


def contact_pair_summary(model: mujoco.MjModel, data: mujoco.MjData, top: int = 6) -> str:
    """Histogram of contacts by geom family ('cable-pad:12, ...') for --profile."""
    counts: dict[tuple[str, str], int] = {}
    for i in range(data.ncon):
        con = data.contact[i]
        names = []
        for geom_id in (int(con.geom1), int(con.geom2)):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom{geom_id}"
            if name.startswith("G"):
                family = "cable"
            elif "pad" in name:
                family = "pad"
            elif "board" in name:
                family = "board"
            elif "table" in name:
                family = "table"
            elif "peg" in name:
                family = "peg"
            elif "connector" in name or "harness" in name or "adapter" in name:
                family = "fixture"
            elif "floor" in name or "wall" in name or "room" in name:
                family = "room"
            elif "fr3" in name or "robotiq" in name or "tmr" in name or "spine" in name:
                family = "robot"
            else:
                family = name
            names.append(family)
        pair = tuple(sorted(names))
        counts[pair] = counts.get(pair, 0) + 1
    ranked = sorted(counts.items(), key=lambda item: item[1], reverse=True)[:top]
    return ", ".join(f"{a}-{b}:{n}" for (a, b), n in ranked)
