#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Sim-time coherent-frame selection for the Isaac Sim eval-camera streams.

Task 2's evaluator subscribes to nine per-camera streams (image, depth,
camera_info, semantic segmentation, three ``*_labels`` JSON tables, and
two ``bbox_2d`` detection streams). Naively pairing the *latest* message
per topic mixes messages from different physical instants -- e.g. a
fresh mask paired with a stale label table -- because one topic's most
recent arrival is not guaranteed to share a render tick with another's.
This module instead selects a *coherent set*: the newest set of
per-stream messages whose ``header.stamp`` values (sim time, derived
from Isaac Sim's 60 Hz render clock) agree within a small tolerance.

The module is pure stdlib (``json``, ``threading``, ``time``,
``collections``, ``dataclasses``) with no ROS and no numpy imports, so
it can be imported unmodified inside both the ROS eval container and
the Isaac Sim container: callers in each environment parse their own
message types and pass plain floats/opaque objects in. ``observe()``
treats ``msg`` as opaque -- this module never inspects it.

Rebase handling: sim time resets to (approximately) 0 on every scene
reset, because every publisher sets ``resetSimulationTimeOnStop``. A
backward jump on any one stream's stamp is therefore evidence that
*all* streams are rebasing together, so ``observe()`` responds by
flushing every stream's buffer at once (a global epoch cut), not just
the stream where the jump was observed. A per-stream-only flush would
leave older, pre-reset items sitting in the other streams' buffers;
because both epochs start near 0, the post-reset clock could
coincidentally revisit a stamp one of those stale items already held,
letting a pre-reset item be paired with post-reset ones. The
``ever_received``/``ever_stamped``/``ever_parsed`` bookkeeping flags
survive the flush because they describe what has been seen this
*session*, not this epoch.

Tolerance semantics: ``tolerance_s`` bounds how far a candidate item's
stamp may sit from the anchor stamp and still count as "the same
render tick". Isaac Sim publishes all nine streams once per render
tick at 60 Hz sim time, so same-tick stamps are near-identical; the
default (~0.0167 s) is about one render period, wide enough to absorb
ordinary publish jitter without reaching into an adjacent tick.
"""

import json
import threading
import time
from collections import deque
from dataclasses import dataclass

STREAM_IMAGE = "image_raw"
STREAM_DEPTH = "depth"
STREAM_CAMERA_INFO = "camera_info"
STREAM_SEMANTIC = "semantic_segmentation"
STREAM_SEMANTIC_LABELS = "semantic_labels"
STREAM_BBOX_TIGHT = "bbox_2d_tight"
STREAM_BBOX_TIGHT_LABELS = "bbox_2d_tight_labels"
STREAM_BBOX_LOOSE = "bbox_2d_loose"
STREAM_BBOX_LOOSE_LABELS = "bbox_2d_loose_labels"

ALL_STREAMS = (
    STREAM_IMAGE,
    STREAM_DEPTH,
    STREAM_CAMERA_INFO,
    STREAM_SEMANTIC,
    STREAM_SEMANTIC_LABELS,
    STREAM_BBOX_TIGHT,
    STREAM_BBOX_TIGHT_LABELS,
    STREAM_BBOX_LOOSE,
    STREAM_BBOX_LOOSE_LABELS,
)

LABEL_STREAMS = (
    STREAM_SEMANTIC_LABELS,
    STREAM_BBOX_TIGHT_LABELS,
    STREAM_BBOX_LOOSE_LABELS,
)

BEST_EFFORT_STREAMS = (STREAM_DEPTH, STREAM_CAMERA_INFO)

# Big Image payloads get short buffers so a slow consumer can't pile up
# unbounded memory; the small JSON/detection streams can hold more
# history for cheap.
IMAGE_STREAMS = (STREAM_IMAGE, STREAM_DEPTH, STREAM_SEMANTIC)


def stamp_to_seconds(sec, nanosec) -> float:
    """Convert a ROS ``sec``/``nanosec`` stamp pair to float seconds."""
    return float(sec) + float(nanosec) / 1_000_000_000.0


def parse_label_stamp(payload: str) -> float | None:
    """Extract the embedded frame time from a ``*_labels`` JSON payload.

    Expects ``{"time_stamp": {"sec": S, "nanosec": N}}`` somewhere in
    the payload and returns it as float seconds. Returns ``None`` on
    any parse failure, a missing ``time_stamp`` (or ``sec``/
    ``nanosec``) key, or a non-dict/malformed ``time_stamp`` value.
    Some older bridges omit ``time_stamp`` entirely -- that is one of
    the ``None`` cases callers must handle gracefully.
    """
    try:
        obj = json.loads(payload)
        time_stamp = obj["time_stamp"]
        return stamp_to_seconds(time_stamp["sec"], time_stamp["nanosec"])
    except (TypeError, KeyError, ValueError):
        return None


def parse_label_payload_ok(payload: str) -> bool:
    """Check a ``*_labels`` payload has >=1 usable class entry.

    True iff ``payload`` is a JSON object with at least one
    integer-string key whose value yields a usable class name (a dict
    with a "class"/"label"/"name" string, or a plain non-empty
    string). Mirrors ``evaluation.parse_semantic_label_map``'s
    acceptance rule without importing that module, so this module
    stays pure. Does not look at ``time_stamp``.
    """
    try:
        obj = json.loads(payload)
    except (TypeError, ValueError):
        return False
    if not isinstance(obj, dict):
        return False
    for raw_key, raw_value in obj.items():
        try:
            int(raw_key)
        except (TypeError, ValueError):
            continue
        if isinstance(raw_value, dict):
            for key in ("class", "label", "name"):
                candidate = raw_value.get(key)
                if isinstance(candidate, str) and candidate.strip():
                    return True
        elif isinstance(raw_value, str) and raw_value.strip():
            return True
    return False


@dataclass(frozen=True)
class StampedItem:
    """One observed message plus the bookkeeping needed to select it."""

    stamp: float | None
    seq: int  # global monotone arrival counter
    msg: object


@dataclass
class SyncSelection:
    """A coherent set of per-stream items anchored on one sim-time tick."""

    anchor_stamp: float
    status: str  # "ok" | "ok_unstamped_labels"
    items: dict[str, object | None]  # ALL nine keys present
    stamps: dict[str, float | None]  # ALL nine keys present
    max_stamp_delta: float
    required: tuple[str, ...]
    loose_in_scene: bool


def _nearest_item(buf, target: float, tolerance: float):
    """Return the item in ``buf`` closest to ``target`` within tolerance.

    Ties (equal distance) are broken by the larger ``seq`` (the more
    recently arrived item wins). Items with ``stamp is None`` are
    never eligible. Returns ``None`` if nothing qualifies.
    """
    best = None
    best_delta = None
    for item in buf:
        if item.stamp is None:
            continue
        delta = abs(item.stamp - target)
        if delta > tolerance:
            continue
        if (
            best is None
            or delta < best_delta
            or (delta == best_delta and item.seq > best.seq)
        ):
            best = item
            best_delta = delta
    return best


class EvalStreamSync:
    """Buffers per-stream messages and selects stamp-coherent sets.

    Thread-safe: ``observe()`` is expected to be called from ROS
    executor threads (one per subscription callback) while
    ``try_select()``/``wait_for_selection()`` are called from a
    service-handler thread. A single lock (via ``threading.Condition``)
    guards all state; ``wait_for_selection()`` blocks on the condition
    instead of busy-waiting.
    """

    def __init__(
        self,
        *,
        required_core: tuple[str, ...] = (
            STREAM_IMAGE,
            STREAM_SEMANTIC,
            STREAM_BBOX_TIGHT,
        ),
        tolerance_s: float = 0.0167,
        max_age_s: float = 0.5,
        rebase_epsilon_s: float = 0.05,
        image_buffer_len: int = 12,
        buffer_len: int = 120,
    ) -> None:
        self._required_core = tuple(required_core)
        self._tolerance_s = tolerance_s
        self._max_age_s = max_age_s
        self._rebase_epsilon_s = rebase_epsilon_s

        self._condition = threading.Condition()
        self._seq = 0
        self._buffers = {
            stream: deque(
                maxlen=image_buffer_len
                if stream in IMAGE_STREAMS
                else buffer_len
            )
            for stream in ALL_STREAMS
        }
        self._ever_received = dict.fromkeys(ALL_STREAMS, False)
        self._ever_stamped = dict.fromkeys(ALL_STREAMS, False)
        self._ever_parsed = dict.fromkeys(ALL_STREAMS, False)

    @property
    def tolerance_s(self) -> float:
        """Coherence tolerance this instance was constructed with."""
        return self._tolerance_s

    def observe(
        self,
        stream: str,
        msg: object,
        *,
        stamp: float | None,
        parsed_ok: bool | None = None,
    ) -> None:
        """Record one arrived message for ``stream``.

        Detects backward stamp jumps (scene resets) and performs a
        global buffer flush before appending when one is seen; see
        the module docstring for why the flush is global rather than
        per-stream. Wakes any thread blocked in
        ``wait_for_selection()``.
        """
        with self._condition:
            self._seq += 1
            seq = self._seq

            buf = self._buffers[stream]
            newest_stamp = buf[-1].stamp if buf else None
            if (
                stamp is not None
                and newest_stamp is not None
                and stamp < newest_stamp - self._rebase_epsilon_s
            ):
                self._flush_locked()

            self._ever_received[stream] = True
            if stamp is not None:
                self._ever_stamped[stream] = True
            if parsed_ok:
                self._ever_parsed[stream] = True

            buf.append(StampedItem(stamp=stamp, seq=seq, msg=msg))
            self._condition.notify_all()

    def try_select(self) -> SyncSelection | None:
        """Attempt one coherent-set selection from currently buffered data.

        Returns ``None`` immediately if no candidate anchor satisfies
        every required stream; never blocks.
        """
        with self._condition:
            return self._try_select_locked()

    def wait_for_selection(self, timeout_s: float) -> SyncSelection | None:
        """Block until a selection is possible or ``timeout_s`` elapses.

        Re-attempts ``try_select()`` on every ``observe()`` wakeup
        (and once up front) until it succeeds or the deadline passes.
        """
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                selection = self._try_select_locked()
                if selection is not None:
                    return selection
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)

    def stream_report(self) -> dict[str, dict]:
        """Per-stream diagnostic snapshot, for failure messages."""
        with self._condition:
            report = {}
            for stream in ALL_STREAMS:
                buf = self._buffers[stream]
                report[stream] = {
                    "ever_received": self._ever_received[stream],
                    "ever_stamped": self._ever_stamped[stream],
                    "buffered": len(buf),
                    "newest_stamp": buf[-1].stamp if buf else None,
                }
            return report

    def semantic_labels_ever_parsed(self) -> bool:
        with self._condition:
            return self._ever_parsed[STREAM_SEMANTIC_LABELS]

    def bbox_tight_labels_ever_parsed(self) -> bool:
        with self._condition:
            return self._ever_parsed[STREAM_BBOX_TIGHT_LABELS]

    def flush(self) -> None:
        """Manually cut a new epoch (all buffers emptied, flags kept)."""
        with self._condition:
            self._flush_locked()

    def _flush_locked(self) -> None:
        for buf in self._buffers.values():
            buf.clear()

    def _compute_required(self):
        """Build the required-stream list for the current session state.

        Returns ``(required, unstamped_labels, loose_in_scene)``.
        ``required`` streams participate in tolerance-based anchor
        matching; ``unstamped_labels`` streams are attached separately
        (newest item, no tolerance check) because their payloads have
        never carried a usable stamp.
        """
        required = list(self._required_core)
        unstamped_labels = []

        for stream in (STREAM_SEMANTIC_LABELS, STREAM_BBOX_TIGHT_LABELS):
            if self._ever_stamped[stream]:
                required.append(stream)
            elif self._ever_received[stream]:
                unstamped_labels.append(stream)

        loose_in_scene = (
            self._ever_received[STREAM_BBOX_LOOSE]
            and self._ever_received[STREAM_BBOX_LOOSE_LABELS]
        )
        if loose_in_scene:
            required.append(STREAM_BBOX_LOOSE)
            if self._ever_stamped[STREAM_BBOX_LOOSE_LABELS]:
                required.append(STREAM_BBOX_LOOSE_LABELS)
            else:
                unstamped_labels.append(STREAM_BBOX_LOOSE_LABELS)

        return required, unstamped_labels, loose_in_scene

    def _find_anchor(self, required):
        """Pick the newest satisfiable anchor stamp, or ``None``."""
        candidate_stamps = set()
        for stream in required:
            for item in self._buffers[stream]:
                if item.stamp is not None:
                    candidate_stamps.add(item.stamp)

        if not candidate_stamps:
            return None, None

        # A stale message delivered after a flush (late DDS arrival with a
        # pre-reset stamp) can inflate this max and block selection until its
        # stream publishes again — the next real message is a backward jump,
        # which flushes globally and self-heals. Failure mode is a visible
        # sync-failure report, never silent cross-epoch mixing.
        newest_required_stamp = max(candidate_stamps)
        for t in sorted(candidate_stamps, reverse=True):
            if newest_required_stamp - t > self._max_age_s:
                break
            bound = {}
            satisfied = True
            for stream in required:
                item = _nearest_item(
                    self._buffers[stream], t, self._tolerance_s
                )
                if item is None:
                    satisfied = False
                    break
                bound[stream] = item
            if satisfied:
                return t, bound

        return None, None

    def _try_select_locked(self) -> SyncSelection | None:
        required, unstamped_labels, loose_in_scene = self._compute_required()

        anchor_stamp, bound = self._find_anchor(required)
        if anchor_stamp is None:
            return None

        items: dict[str, object | None] = dict.fromkeys(ALL_STREAMS)
        stamps: dict[str, float | None] = dict.fromkeys(ALL_STREAMS)

        max_delta = 0.0
        for stream in required:
            item = bound[stream]
            items[stream] = item.msg
            stamps[stream] = item.stamp
            delta = abs(item.stamp - anchor_stamp)
            if delta > max_delta:
                max_delta = delta

        status = "ok"
        for stream in unstamped_labels:
            buf = self._buffers[stream]
            if buf:
                newest_item = buf[-1]
                items[stream] = newest_item.msg
                stamps[stream] = newest_item.stamp
                status = "ok_unstamped_labels"

        for stream in BEST_EFFORT_STREAMS:
            item = _nearest_item(
                self._buffers[stream], anchor_stamp, float("inf")
            )
            if item is not None:
                items[stream] = item.msg
                stamps[stream] = item.stamp

        return SyncSelection(
            anchor_stamp=anchor_stamp,
            status=status,
            items=items,
            stamps=stamps,
            max_stamp_delta=max_delta,
            required=tuple(required),
            loose_in_scene=loose_in_scene,
        )
