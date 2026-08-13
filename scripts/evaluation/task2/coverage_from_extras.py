#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Offline physical coverage/spill audit over recorded task2 episodes.

Reads ``task2_extras/episode_XXXXXX.npz`` sidecars (written by
``task2_isaacsim/services/recording/record_task2.py``; see
``coverage_metrics``'s module docstring for the exact array shapes)
and runs ``coverage_metrics.coverage_metrics`` on each episode's last
``pad_points`` sample against the nearest-in-time ``board_target``
pose. Purely additive/read-only: this never touches the recorded
dataset or its ``episodes_task2.jsonl`` (only reads it, to join
``success``/the recorded IoU suggestion in for correlation).

``load_episode``/``evaluate_episode`` are plain functions with no
argparse side effects, so they import cleanly for tests; the CLI
(argument parsing, file IO, stdout summary) only runs under
``if __name__ == "__main__":``.

Usage:
    coverage_from_extras.py --dataset <dir> [--episodes all|A-B|1,2,3]
        [--out coverage_audit.jsonl] [--grid-res-mm 1.0]
        [--close-radius-mm 4.0] [--z-band-mm "-3:8"] [--settle-eps-mm 1.0]
    coverage_from_extras.py --npz <episode_XXXXXX.npz> [...]
"""

import argparse
import json
import re
import statistics
import sys
from pathlib import Path
from typing import Any

import numpy as np
from coverage_metrics import coverage_metrics

# Reasons a per-episode record can carry metrics=null; shared between the
# loader (which detects them) and the summary line (which reports them).
REASON_NO_PAD_POINTS = "no_pad_points"
REASON_BOARD_TARGET_MISSING = "board_target_missing"

_EPISODE_FILENAME_RE = re.compile(r"episode_(\d{6})\.npz$")

DEFAULT_OUT_PATH = "./coverage_audit.jsonl"
DEFAULT_GRID_RES_MM = 1.0
DEFAULT_CLOSE_RADIUS_MM = 4.0
DEFAULT_Z_BAND_MM = "-3:8"
DEFAULT_SETTLE_EPS_MM = 1.0


def parse_episode_index_from_name(filename: str) -> int | None:
    """Episode index from an ``episode_XXXXXX.npz`` basename, or None."""
    match = _EPISODE_FILENAME_RE.search(filename)
    return int(match.group(1)) if match else None


def discover_episodes(extras_dir: Path) -> dict[int, Path]:
    """``{episode_index: path}`` for every ``episode_*.npz`` under
    ``extras_dir``.
    """
    found: dict[int, Path] = {}
    if not extras_dir.is_dir():
        return found
    for path in sorted(extras_dir.glob("episode_*.npz")):
        index = parse_episode_index_from_name(path.name)
        if index is not None:
            found[index] = path
    return found


def parse_episode_selector(spec: str, available: list[int]) -> list[int]:
    """``all|A-B|comma-list`` -> sorted episode indices, filtered to
    ``available``.

    Ranges are inclusive of both endpoints (``0-2`` selects 0, 1, 2).
    Indices not present in ``available`` are silently dropped rather
    than raising -- selecting a range wider than the dataset actually
    holds is a normal, harmless usage pattern.
    """
    available_set = set(available)
    if spec.strip() == "all":
        return sorted(available_set)
    selected: set[int] = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part[1:]:
            # part[1:] so a leading "-" (a negative episode index, which
            # cannot occur in practice) is never mistaken for a range dash.
            start_str, _, end_str = part.partition("-")
            selected.update(range(int(start_str), int(end_str) + 1))
        else:
            selected.add(int(part))
    return sorted(selected & available_set)


def parse_z_band_mm(spec: str) -> tuple[float, float]:
    """``"LOW:HIGH"`` (millimeters) -> ``(low, high)`` floats."""
    low_str, sep, high_str = spec.partition(":")
    if not sep:
        raise argparse.ArgumentTypeError(
            f"--z-band-mm must be LOW:HIGH (e.g. -3:8), got {spec!r}"
        )
    return float(low_str), float(high_str)


def load_episode(npz_path: Path) -> dict[str, Any]:
    """Load one ``episode_XXXXXX.npz`` sidecar into plain arrays.

    Returns a dict with whichever of these keys the file actually has
    (missing/optional arrays are simply absent, never faked):
    ``sim_time`` (T,), ``object_poses`` (T, K, 7), ``object_names``
    (list[str], length K), ``pad_sim_time`` (P,), and ``pad_points``
    (a list of P per-sample ``(N_p, 3)`` arrays -- reconstructed from
    the ``pad_points_flat`` (1-D, length ``3 * sum(pad_points_counts)``)
    + ``pad_points_counts`` (P,) fallback layout when the regular
    ``pad_points`` (P, N, 3) array is absent, per
    ``ExtrasBuffer.save``'s "topology changed" branch in
    ``record_task2.py``).
    """
    result: dict[str, Any] = {}
    with np.load(npz_path, allow_pickle=False) as data:
        if "sim_time" in data:
            result["sim_time"] = data["sim_time"]
        if "object_poses" in data:
            result["object_poses"] = data["object_poses"]
        if "object_names" in data:
            result["object_names"] = [
                str(name) for name in data["object_names"]
            ]
        if "pad_sim_time" in data:
            result["pad_sim_time"] = data["pad_sim_time"]
        if "pad_points" in data:
            result["pad_points"] = list(data["pad_points"])
        elif "pad_points_flat" in data and "pad_points_counts" in data:
            flat = data["pad_points_flat"]
            if flat.ndim == 1:
                flat = flat.reshape(-1, 3)
            counts = data["pad_points_counts"]
            samples = []
            offset = 0
            for count in counts:
                n = int(count)
                samples.append(flat[offset : offset + n])
                offset += n
            result["pad_points"] = samples
    return result


def _pad_settled(
    pad_points_samples: list[np.ndarray], settle_eps_m: float
) -> bool:
    """Mean per-point displacement of the last sample vs. the previous one.

    True when there is only one sample (nothing to compare against), or
    when the two samples' point counts differ (a topology change right
    at the end of the episode -- treated conservatively as "not
    settled" rather than comparing mismatched point sets).
    """
    if len(pad_points_samples) < 2:
        return True
    last = np.asarray(pad_points_samples[-1], dtype=np.float64)
    prev = np.asarray(pad_points_samples[-2], dtype=np.float64)
    if last.shape != prev.shape:
        return False
    displacement = np.linalg.norm(last - prev, axis=1)
    return bool(np.mean(displacement) < settle_eps_m)


def _find_board_pose(loaded: dict[str, Any], target_sim_time: float):
    """``board_target``'s pose row nearest ``target_sim_time``, or None.

    None covers every way the lookup can fail to resolve: no
    ``object_names``/``object_poses``/``sim_time`` arrays at all, an
    empty ``sim_time``, or ``object_names`` not containing
    ``"board_target"``.
    """
    object_names = loaded.get("object_names")
    object_poses = loaded.get("object_poses")
    sim_time = loaded.get("sim_time")
    if (
        object_names is None
        or object_poses is None
        or sim_time is None
        or len(sim_time) == 0
        or "board_target" not in object_names
    ):
        return None
    board_index = object_names.index("board_target")
    frame_index = int(
        np.argmin(np.abs(np.asarray(sim_time) - target_sim_time))
    )
    return np.asarray(object_poses[frame_index, board_index], dtype=np.float64)


def evaluate_episode(
    episode_index: int,
    loaded: dict[str, Any],
    *,
    grid_res_m: float,
    close_radius_m: float,
    z_band_m: tuple[float, float],
    settle_eps_m: float,
) -> dict[str, Any]:
    """One ``{episode_index, settled, pad_sim_time, metrics|null, reason?}``
    record for ``loaded`` (as returned by ``load_episode``).

    Uses the LAST pad sample only. Missing pad data or an
    unresolvable ``board_target`` pose never raises -- the record
    carries ``metrics: null`` and a ``reason`` string instead, so a
    bad/partial episode cannot abort the whole run.
    """
    pad_points_samples = loaded.get("pad_points")
    pad_sim_time = loaded.get("pad_sim_time")
    if (
        not pad_points_samples
        or pad_sim_time is None
        or len(pad_sim_time) == 0
    ):
        return {
            "episode_index": episode_index,
            "settled": None,
            "pad_sim_time": None,
            "metrics": None,
            "reason": REASON_NO_PAD_POINTS,
        }

    last_points = np.asarray(pad_points_samples[-1], dtype=np.float32)
    last_pad_time = float(pad_sim_time[-1])
    settled = _pad_settled(pad_points_samples, settle_eps_m)

    board_pose = _find_board_pose(loaded, last_pad_time)
    if board_pose is None:
        return {
            "episode_index": episode_index,
            "settled": settled,
            "pad_sim_time": last_pad_time,
            "metrics": None,
            "reason": REASON_BOARD_TARGET_MISSING,
        }

    metrics = coverage_metrics(
        last_points,
        board_pose,
        z_band_m=z_band_m,
        grid_res_m=grid_res_m,
        close_radius_m=close_radius_m,
    )
    return {
        "episode_index": episode_index,
        "settled": settled,
        "pad_sim_time": last_pad_time,
        "metrics": metrics,
    }


def load_episode_meta(extras_dir: Path) -> dict[int, dict[str, Any]]:
    """``{episode_index: parsed row}`` from ``episodes_task2.jsonl``.

    Empty (never raises) when the file is absent, unreadable, or holds
    unparsable lines -- the join in ``run_cli`` is best-effort
    correlation data, not required for the audit itself.
    """
    meta_path = extras_dir / "episodes_task2.jsonl"
    meta: dict[int, dict[str, Any]] = {}
    if not meta_path.is_file():
        return meta
    with meta_path.open("r", encoding="utf-8") as file_obj:
        for raw_line in file_obj:
            line = raw_line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            index = row.get("episode_index")
            if index is not None:
                meta[int(index)] = row
    return meta


def _join_meta(
    record: dict[str, Any], meta_row: dict[str, Any] | None
) -> None:
    if meta_row is None:
        return
    record["success"] = meta_row.get("success")
    suggestion = meta_row.get("success_suggestion") or {}
    record["recorded_iou"] = suggestion.get("iou_thermalpad_vs_target_current")


def _print_aggregate(records: list[dict[str, Any]], out_path: Path) -> None:
    print(f"Wrote {len(records)} episode record(s) to {out_path}")
    coverages = [
        r["metrics"]["coverage"]
        for r in records
        if r.get("metrics") is not None
    ]
    print(f"  usable metrics: {len(coverages)}/{len(records)}")
    if coverages:
        print(
            f"  coverage overall: mean={statistics.mean(coverages):.4f} "
            f"median={statistics.median(coverages):.4f}"
        )
    by_success: dict[Any, list[float]] = {}
    for record in records:
        if record.get("metrics") is None or "success" not in record:
            continue
        by_success.setdefault(record["success"], []).append(
            record["metrics"]["coverage"]
        )
    for label in sorted(by_success, key=lambda v: (v is None, v)):
        values = by_success[label]
        print(
            f"  coverage success={label}: n={len(values)} "
            f"mean={statistics.mean(values):.4f} "
            f"median={statistics.median(values):.4f}"
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Offline physical coverage/spill audit over recorded task2 "
            "episode ground truth (task2_extras/episode_XXXXXX.npz)."
        )
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--dataset",
        type=str,
        default=None,
        help="Dataset root containing task2_extras/ "
        "(episode_*.npz + episodes_task2.jsonl)",
    )
    source.add_argument(
        "--npz",
        type=str,
        default=None,
        help="A single episode_XXXXXX.npz file",
    )
    parser.add_argument(
        "--episodes",
        type=str,
        default="all",
        help="all|A-B|comma-list of episode indices (default: all)",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=DEFAULT_OUT_PATH,
        help=f"Output JSONL path (default: {DEFAULT_OUT_PATH}, in the "
        "current directory -- never inside the dataset)",
    )
    parser.add_argument(
        "--grid-res-mm", type=float, default=DEFAULT_GRID_RES_MM
    )
    parser.add_argument(
        "--close-radius-mm", type=float, default=DEFAULT_CLOSE_RADIUS_MM
    )
    parser.add_argument("--z-band-mm", type=str, default=DEFAULT_Z_BAND_MM)
    parser.add_argument(
        "--settle-eps-mm", type=float, default=DEFAULT_SETTLE_EPS_MM
    )
    return parser


def run_cli(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    z_band_low_mm, z_band_high_mm = parse_z_band_mm(args.z_band_mm)
    z_band_m = (z_band_low_mm / 1000.0, z_band_high_mm / 1000.0)
    grid_res_m = args.grid_res_mm / 1000.0
    close_radius_m = args.close_radius_mm / 1000.0
    settle_eps_m = args.settle_eps_mm / 1000.0

    if args.dataset:
        extras_dir = Path(args.dataset) / "task2_extras"
        episode_paths = discover_episodes(extras_dir)
    else:
        npz_path = Path(args.npz)
        index = parse_episode_index_from_name(npz_path.name)
        episode_paths = {index if index is not None else 0: npz_path}
        extras_dir = npz_path.parent

    selected = parse_episode_selector(args.episodes, sorted(episode_paths))
    episode_meta = load_episode_meta(extras_dir)

    # Default (and any relative --out) resolves against the CWD, never
    # the dataset directory -- the audit is read-only w.r.t. the dataset.
    out_path = Path(args.out)
    records: list[dict[str, Any]] = []
    with out_path.open("w", encoding="utf-8") as out_fh:
        for episode_index in selected:
            npz_path = episode_paths[episode_index]
            try:
                loaded = load_episode(npz_path)
                record = evaluate_episode(
                    episode_index,
                    loaded,
                    grid_res_m=grid_res_m,
                    close_radius_m=close_radius_m,
                    z_band_m=z_band_m,
                    settle_eps_m=settle_eps_m,
                )
            except Exception as exc:  # noqa: BLE001 - never abort the run
                record = {
                    "episode_index": episode_index,
                    "settled": None,
                    "pad_sim_time": None,
                    "metrics": None,
                    "reason": f"load_error: {exc}",
                }
            _join_meta(record, episode_meta.get(episode_index))
            out_fh.write(json.dumps(record) + "\n")
            records.append(record)

    _print_aggregate(records, out_path)
    return 0


def main(argv: list[str] | None = None) -> int:
    return run_cli(argv)


if __name__ == "__main__":
    sys.exit(main())
