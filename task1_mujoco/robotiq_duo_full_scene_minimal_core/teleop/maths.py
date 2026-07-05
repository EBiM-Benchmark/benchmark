"""Pure quaternion / geometry helpers. No MuJoCo state is touched here
(except mju_mat2Quat as a converter), so everything is unit-testable.
Quaternions are wxyz, matching MuJoCo."""

from __future__ import annotations

import math

import numpy as np

import mujoco


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    q = np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )
    return q / max(np.linalg.norm(q), 1e-9)


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def axis_angle_quat(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    axis_v = np.asarray(axis, dtype=np.float64)
    axis_v /= max(np.linalg.norm(axis_v), 1e-9)
    half = 0.5 * angle
    return np.array([math.cos(half), *(math.sin(half) * axis_v)], dtype=np.float64)


def mat_to_quat(mat: np.ndarray) -> np.ndarray:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, mat.reshape(9))
    return quat / max(np.linalg.norm(quat), 1e-9)


def rot_error(target_quat: np.ndarray, current_mat: np.ndarray) -> np.ndarray:
    """Axis-angle rotation (rad) taking ``current_mat`` onto ``target_quat``."""
    current = mat_to_quat(current_mat)
    dq = quat_mul(target_quat, quat_conj(current))
    if dq[0] < 0:
        dq = -dq
    axis_norm = float(np.linalg.norm(dq[1:]))
    if axis_norm < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(axis_norm, max(1e-9, float(dq[0])))
    return dq[1:] / axis_norm * angle


def frame_from_y_axis(y_axis: np.ndarray) -> np.ndarray:
    """Right-handed frame whose y column is the given direction (used to
    orient cable capsule segments along an initialization path)."""
    y = np.asarray(y_axis, dtype=np.float64)
    y /= max(np.linalg.norm(y), 1e-9)
    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(y, up))) > 0.95:
        up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    x = np.cross(y, up)
    x /= max(np.linalg.norm(x), 1e-9)
    z = np.cross(x, y)
    return np.column_stack((x, y, z))


def sample_polyline(points: np.ndarray, count: int) -> np.ndarray:
    """Resample a polyline to ``count`` points at equal arc-length spacing."""
    seg = points[1:] - points[:-1]
    lengths = np.linalg.norm(seg, axis=1)
    total = float(np.sum(lengths))
    if total <= 0.0:
        raise RuntimeError("Bad cable initialization path.")
    distances = np.linspace(0.0, total, count)
    out = []
    cursor = 0.0
    idx = 0
    for distance in distances:
        while idx < len(lengths) - 1 and distance > cursor + lengths[idx]:
            cursor += float(lengths[idx])
            idx += 1
        local = 0.0 if lengths[idx] <= 0.0 else (distance - cursor) / lengths[idx]
        out.append(points[idx] + local * seg[idx])
    return np.asarray(out, dtype=np.float64)


def smooth_twist(prev: np.ndarray, target: np.ndarray, dt: float, tau: float) -> np.ndarray:
    """First-order low-pass toward ``target`` with time constant ``tau``."""
    alpha = 1.0 - math.exp(-dt / max(tau, 1e-6))
    return prev + alpha * (target - prev)
