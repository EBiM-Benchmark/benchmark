"""Apply the mnet client's randomized board configuration to the sim board.

For the cable_management task the client publishes, per tier, the nominal
fixture grid coordinates (``base_coordinates``, on the official 30x40 hole
grid) and the slightly randomized ones the board must actually be set up
with (``test_coordinates``, each fixture shifted by at most one grid cell).
On a real board a human moves the fixtures; here we move the sim fixture
bodies by the equivalent distance.

Our board implements the TIER-2 layout (2x F5 wire adapters, 1x F2 C-clip,
4x F1 round pegs) but is not a millimeter-exact replica of the official
grid, so instead of hardcoding a grid pitch we fit a similarity transform
(uniform scale + rotation) grid->board-local from the tier's OWN base
coordinates to our fixture positions (brute-forcing the per-type
assignment), and apply that map to the (test - base) offsets. That yields
uniformly scaled, correctly oriented per-fixture shifts (one grid cell ~=
a few cm) regardless of the layout's absolute inaccuracies.
"""

from __future__ import annotations

from collections import Counter
from itertools import permutations

import numpy as np

import mujoco

from . import log
from .grasping import release_grasp
from .scene import initialize_cable_on_board

# our board's fixture bodies, grouped by official fixture type code
FIXTURE_BODIES_BY_TYPE = {
    "F5": ("adapter_0", "adapter_1"),  # wire-to-base adapters (cable ends)
    "F2": ("cclip_0",),  # C-clip
    "F1": ("round_peg_0", "round_peg_1", "round_peg_2", "round_peg_3"),
}

# Tier-2 layout, verbatim from mnet_client/tasks/cable_management.py (kept
# local so --randomize-board works without ROS or the client installed)
TIER_2_BASE = {
    "routing_configuration": [0, -1, +2, +4, -3, -5, 6],
    "base_coordinates": [
        (1, 2),
        (4, 19),
        (4, 33),
        (13, 18),
        (12, 29),
        (20, 15),
        (30, 39),
    ],
    "fixture_types": ["F5", "-F2", "+F1", "-F1", "+F1", "-F1", "F5"],
    "cable_length": "2m",
}
_RANDOM_OFFSET = [
    (0, 1),
    (1, 0),
    (0, -1),
    (-1, 0),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
]
_MAX_X, _MAX_Y = 30, 40
# hard sanity cap: a single grid cell is ~2.6 cm, so anything beyond this
# means the fit or the config is wrong — refuse to move fixtures that far
MAX_FIXTURE_SHIFT = 0.08


def _fit_similarity(grid: np.ndarray, local: np.ndarray) -> tuple[np.ndarray, float]:
    """Umeyama similarity fit local ~= (s R) grid + t.

    A similarity (uniform scale + rotation) is used instead of a full affine
    map because the official board grid is square — a free affine fit would
    absorb our layout's inaccuracies into anisotropic cell pitches and make
    x-offsets tiny while y-offsets are large. Returns (M = s R mapping grid
    deltas to local deltas, rms residual)."""
    mg, ml = grid.mean(axis=0), local.mean(axis=0)
    g, loc = grid - mg, local - ml
    cov = g.T @ loc / len(grid)
    u, s, vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(u @ vt))
    scale = float((s * [1.0, d]).sum() / (g**2).sum() * len(grid))
    rot = (u @ np.diag([1.0, d]) @ vt).T
    m_fit = scale * rot
    residual = float(np.sqrt(np.mean(np.sum((g @ m_fit.T - loc) ** 2, axis=1))))
    return m_fit, residual


def make_random_tier2_config(seed: int | None = None) -> dict:
    """A Tier-2 board configuration with the same per-fixture randomization
    the mnet client applies (one of 8 unit/diagonal cell offsets each,
    clamped to the grid) — for testing without ROS (--randomize-board)."""
    rng = np.random.default_rng(seed)
    cfg = dict(TIER_2_BASE)
    offsets = [_RANDOM_OFFSET[int(rng.integers(0, len(_RANDOM_OFFSET)))] for _ in cfg["base_coordinates"]]
    cfg["coordinate_offsets"] = offsets
    cfg["test_coordinates"] = [
        (max(1, min(c[0] + o[0], _MAX_X)), max(1, min(c[1] + o[1], _MAX_Y)))
        for c, o in zip(cfg["base_coordinates"], offsets)
    ]
    return cfg


def apply_local_random_config(session, seed: int | None = None) -> bool:
    """--randomize-board: randomize the fixtures locally, no client needed."""
    log(f"[board] local Tier2 randomization (no ROS), seed={seed if seed is not None else 'random'}")
    return apply_board_config(session, make_random_tier2_config(seed))


def apply_board_config(session, cfg: dict) -> bool:
    """Move the board fixtures per the client's test_coordinates and re-lay
    the cable. Returns True when fixtures were moved."""
    base = cfg.get("base_coordinates")
    test = cfg.get("test_coordinates")
    types = cfg.get("fixture_types")
    if not base or not test or not types or len(base) != len(types) or len(test) != len(types):
        log("[mnet] board config incomplete (no test_coordinates?) — nothing applied")
        return False

    # +/- prefixes encode the routing side, not the fixture type
    clean_types = [t.lstrip("+-") for t in types]
    ours = Counter(t for ts in FIXTURE_BODIES_BY_TYPE for t in [ts] * len(FIXTURE_BODIES_BY_TYPE[ts]))
    if Counter(clean_types) != ours:
        log(
            f"[mnet] board config is for a different layout ({Counter(clean_types)}) — "
            f"this scene implements Tier2 ({dict(ours)}); nothing applied"
        )
        return False

    model, data = session.model, session.data
    body_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for names in FIXTURE_BODIES_BY_TYPE.values()
        for name in names
    }
    if any(bid < 0 for bid in body_ids.values()):
        log("[mnet] fixture bodies missing from the model — nothing applied")
        return False
    local_pos = {name: model.body_pos[bid][:2].copy() for name, bid in body_ids.items()}

    # indices in the config per type, in published order
    idx_by_type: dict[str, list[int]] = {}
    for i, t in enumerate(clean_types):
        idx_by_type.setdefault(t, []).append(i)

    base_arr = np.asarray(base, dtype=np.float64)
    test_arr = np.asarray(test, dtype=np.float64)

    # stage 1: fit the grid->board map on the uniquely-typed ANCHOR fixtures
    # (the two cable-end adapters spanning the board diagonal + the C-clip);
    # only the adapter order is ambiguous. Pegs are left out — their layout
    # is the least grid-faithful part of the board and would drag the fit.
    anchor_names = (
        *FIXTURE_BODIES_BY_TYPE["F5"],
        *FIXTURE_BODIES_BY_TYPE["F2"],
    )
    best = None
    for f5_perm in permutations(idx_by_type["F5"]):
        assign = dict(zip(FIXTURE_BODIES_BY_TYPE["F5"], f5_perm))
        assign.update(zip(FIXTURE_BODIES_BY_TYPE["F2"], idx_by_type["F2"]))
        grid = base_arr[[assign[n] for n in anchor_names]]
        local = np.array([local_pos[n] for n in anchor_names])
        m_fit, residual = _fit_similarity(grid, local)
        if best is None or residual < best[2]:
            best = (assign, m_fit, residual)
    assign, linear, residual = best  # linear: 2x2, local_delta = linear @ grid_delta
    cell = float(np.mean(np.linalg.norm(linear, axis=0)))
    log(f"[mnet] board fit: cell pitch ~{cell * 100:.1f} cm, anchor residual {residual * 100:.1f} cm")

    # stage 2: pegs receive the offset of the config F1 slot whose predicted
    # board position is nearest (best total-distance assignment over 4! = 24)
    anchor_grid = base_arr[[assign[n] for n in anchor_names]]
    anchor_local = np.array([local_pos[n] for n in anchor_names])
    mg, ml = anchor_grid.mean(axis=0), anchor_local.mean(axis=0)
    peg_names = FIXTURE_BODIES_BY_TYPE["F1"]
    predicted = {i: (linear @ (base_arr[i] - mg)) + ml for i in idx_by_type["F1"]}
    best_perm = min(
        permutations(idx_by_type["F1"]),
        key=lambda perm: sum(float(np.linalg.norm(predicted[i] - local_pos[n])) for n, i in zip(peg_names, perm)),
    )
    assign.update(zip(peg_names, best_perm))

    # per-fixture shift = linear map of the grid offset
    moved = []
    for name, cfg_idx in assign.items():
        delta_grid = test_arr[cfg_idx] - base_arr[cfg_idx]
        delta = linear @ delta_grid
        dist = float(np.linalg.norm(delta))
        if dist < 1e-6:
            continue
        if dist > MAX_FIXTURE_SHIFT:
            log(f"[mnet] refusing to move {name} by {dist * 100:.1f} cm (cap {MAX_FIXTURE_SHIFT * 100:.0f} cm)")
            continue
        model.body_pos[body_ids[name]][:2] += delta
        moved.append(f"{name} {delta_grid.astype(int).tolist()} -> {delta[0] * 100:+.1f}/{delta[1] * 100:+.1f} cm")

    if not moved:
        log("[mnet] board config matched but no fixture needed to move")
        return False

    # the cable start is welded next to adapter_0 — keep them together
    b_first = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "B_first")
    a0_idx = assign["adapter_0"]
    a0_delta = linear @ (test_arr[a0_idx] - base_arr[a0_idx])
    if b_first >= 0 and float(np.linalg.norm(a0_delta)) <= MAX_FIXTURE_SHIFT:
        model.body_pos[b_first][:2] += a0_delta

    # re-lay the cable from scratch over the shifted fixtures and drop any
    # grasp state — segments may have been teleported out of the fingers
    for bid in session.cable_bodies:
        adr = int(model.body_dofadr[bid])
        num = int(model.body_dofnum[bid])
        if adr >= 0:
            data.qvel[adr : adr + num] = 0.0
    initialize_cable_on_board(model, data)
    for arm in session.arms.values():
        release_grasp(data, arm)
    mujoco.mj_forward(model, data)

    log("[mnet] fixtures moved per client randomization:")
    for line in moved:
        log(f"        {line}")
    return True
