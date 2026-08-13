#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for stream_sync (no ROS required).

Run: python3 scripts/evaluation/task2/tests/test_stream_sync.py
Pure stdlib; no numpy, no ROS -- matches the module under test.
"""

import json
import os
import sys
import threading
import time

# Make the flat eval modules importable when run directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stream_sync import (  # noqa: E402
    ALL_STREAMS,
    STREAM_BBOX_LOOSE,
    STREAM_BBOX_LOOSE_LABELS,
    STREAM_BBOX_TIGHT,
    STREAM_BBOX_TIGHT_LABELS,
    STREAM_IMAGE,
    STREAM_SEMANTIC,
    STREAM_SEMANTIC_LABELS,
    EvalStreamSync,
    parse_label_payload_ok,
    parse_label_stamp,
)

# Real example payload from a live semantic_labels topic (see the task
# brief): used verbatim so the stamp-parsing tests exercise the exact
# on-wire shape.
LABEL_PAYLOAD_EXAMPLE = json.dumps(
    {
        "0": {"class": "thermalpad"},
        "1": {"class": "board"},
        "2": {"class": "target"},
        "time_stamp": {"nanosec": 716667121, "sec": 8},
    }
)


def expect(name, cond):
    status = "PASS" if cond else "FAIL"
    print(f"[{status}] {name}")
    if not cond:
        raise AssertionError(name)


def make_default_sync(**kwargs):
    return EvalStreamSync(**kwargs)


def observe_required_core(sync, stamp, tag):
    """Observe the default required_core trio at one stamp."""
    sync.observe(STREAM_IMAGE, f"image_{tag}", stamp=stamp)
    sync.observe(STREAM_SEMANTIC, f"semantic_{tag}", stamp=stamp)
    sync.observe(STREAM_BBOX_TIGHT, f"bbox_tight_{tag}", stamp=stamp)


def test_nearest_selection():
    sync = make_default_sync()
    observe_required_core(sync, 1.0, "t1")
    observe_required_core(sync, 2.0, "t2")
    observe_required_core(sync, 3.0, "t3")

    sel = sync.try_select()
    expect("nearest: selection made", sel is not None)
    expect("nearest: anchors newest tick", sel.anchor_stamp == 3.0)
    expect(
        "nearest: required is core trio",
        sel.required == (STREAM_IMAGE, STREAM_SEMANTIC, STREAM_BBOX_TIGHT),
    )
    expect("nearest: image item is t3", sel.items[STREAM_IMAGE] == "image_t3")
    expect(
        "nearest: semantic item is t3",
        sel.items[STREAM_SEMANTIC] == "semantic_t3",
    )
    expect(
        "nearest: bbox_tight item is t3",
        sel.items[STREAM_BBOX_TIGHT] == "bbox_tight_t3",
    )
    expect("nearest: zero delta", sel.max_stamp_delta == 0.0)
    expect("nearest: status ok", sel.status == "ok")


def test_duplicate_stamp_prefers_latest_arrival():
    # Two items on the same required stream share an exact stamp; the
    # spec's tie-break (larger seq, i.e. the more-recently-arrived
    # item) must decide, not first-seen-wins.
    sync = make_default_sync()
    sync.observe(STREAM_IMAGE, "image_first", stamp=2.0)
    sync.observe(STREAM_IMAGE, "image_second", stamp=2.0)
    sync.observe(STREAM_SEMANTIC, "semantic", stamp=2.0)
    sync.observe(STREAM_BBOX_TIGHT, "bbox_tight", stamp=2.0)

    sel = sync.try_select()
    expect("tie-break: selection made", sel is not None)
    expect(
        "tie-break: prefers the later-arrived duplicate",
        sel.items[STREAM_IMAGE] == "image_second",
    )


def test_tolerance_rejection():
    sync = make_default_sync()
    sync.observe(STREAM_IMAGE, "image", stamp=2.0)
    sync.observe(STREAM_SEMANTIC, "semantic", stamp=2.0)
    # 50 ms away from the other two -- outside the default 16.7 ms
    # tolerance but nowhere near the 500 ms max_age budget, so this
    # must fail purely on tolerance, at every candidate.
    sync.observe(STREAM_BBOX_TIGHT, "bbox_tight", stamp=2.05)

    sel = sync.try_select()
    expect("tolerance: no selection", sel is None)


def test_anchor_falls_back_to_older_coherent():
    sync = make_default_sync()
    sync.observe(STREAM_IMAGE, "image_a", stamp=1.0)
    sync.observe(STREAM_IMAGE, "image_b", stamp=1.1)
    sync.observe(STREAM_SEMANTIC, "semantic_a", stamp=1.0)
    sync.observe(STREAM_SEMANTIC, "semantic_b", stamp=1.1)
    # bbox_tight never publishes the newest (1.1) tick, only a slightly
    # earlier one close to the 1.0 tick.
    sync.observe(STREAM_BBOX_TIGHT, "bbox_tight_a", stamp=0.995)

    sel = sync.try_select()
    expect("fallback: selection made", sel is not None)
    expect("fallback: anchors older coherent tick", sel.anchor_stamp == 1.0)
    expect(
        "fallback: image is t=1.0 item", sel.items[STREAM_IMAGE] == "image_a"
    )
    expect(
        "fallback: semantic is t=1.0 item",
        sel.items[STREAM_SEMANTIC] == "semantic_a",
    )
    expect(
        "fallback: bbox_tight is the only item",
        sel.items[STREAM_BBOX_TIGHT] == "bbox_tight_a",
    )
    expect(
        "fallback: max_stamp_delta from bbox_tight",
        abs(sel.max_stamp_delta - 0.005) < 1e-9,
    )


def test_max_age_rejection():
    sync = make_default_sync()
    sync.observe(STREAM_IMAGE, "image_old", stamp=1.0)
    sync.observe(STREAM_SEMANTIC, "semantic_old", stamp=1.0)
    sync.observe(STREAM_BBOX_TIGHT, "bbox_tight_old", stamp=1.0)
    # Only STREAM_IMAGE advances far enough (600 ms, past the 500 ms
    # max_age_s budget) that the only fully-coherent set (the t=1.0
    # trio) is now too old relative to it.
    sync.observe(STREAM_IMAGE, "image_new", stamp=1.6)

    sel = sync.try_select()
    expect("max_age: no selection", sel is None)


def test_backward_jump_flushes_all():
    sync = make_default_sync()
    observe_required_core(sync, 5.0, "pre")

    report = sync.stream_report()
    expect("flush: pre-reset buffered", report[STREAM_IMAGE]["buffered"] == 1)

    # A backward jump on one stream (a fresh sim after scene reset,
    # rebased near 0) must flush every stream's buffer, not just its
    # own.
    sync.observe(STREAM_IMAGE, "image_post", stamp=0.016)

    report = sync.stream_report()
    expect(
        "flush: jumped stream keeps its new item",
        report[STREAM_IMAGE]["buffered"] == 1,
    )
    expect(
        "flush: other streams emptied",
        report[STREAM_SEMANTIC]["buffered"] == 0
        and report[STREAM_BBOX_TIGHT]["buffered"] == 0,
    )
    expect(
        "flush: ever_received flags survive",
        report[STREAM_SEMANTIC]["ever_received"] is True
        and report[STREAM_BBOX_TIGHT]["ever_received"] is True,
    )
    expect("flush: no selection yet", sync.try_select() is None)

    # Once all streams have republished post-reset, selection must
    # return only the post-reset items.
    sync.observe(STREAM_SEMANTIC, "semantic_post", stamp=0.020)
    sync.observe(STREAM_BBOX_TIGHT, "bbox_tight_post", stamp=0.018)

    sel = sync.try_select()
    expect("flush: post-reset selection made", sel is not None)
    expect(
        "flush: post-reset items only",
        sel.items[STREAM_IMAGE] == "image_post"
        and sel.items[STREAM_SEMANTIC] == "semantic_post"
        and sel.items[STREAM_BBOX_TIGHT] == "bbox_tight_post",
    )


def test_rebase_stamp_tie_never_pairs_across_flush():
    sync = make_default_sync()
    sync.observe(STREAM_IMAGE, "image_pre", stamp=0.10)
    # Establish a "newest" stamp on STREAM_SEMANTIC so a later backward
    # jump on it is detectable.
    sync.observe(STREAM_SEMANTIC, "semantic_pre_high", stamp=5.0)
    # Backward jump on STREAM_SEMANTIC triggers a global flush; its own
    # post-jump item happens to land on the exact same stamp
    # (t=0.10) that STREAM_IMAGE's now-flushed pre-reset item had.
    sync.observe(STREAM_SEMANTIC, "semantic_post", stamp=0.10)

    report = sync.stream_report()
    expect("tie: image buffer flushed", report[STREAM_IMAGE]["buffered"] == 0)

    # A genuinely new post-reset STREAM_IMAGE frame, coincidentally at
    # the same stamp, must be what gets selected -- never the flushed
    # pre-reset item.
    sync.observe(STREAM_IMAGE, "image_post", stamp=0.10)
    sync.observe(STREAM_BBOX_TIGHT, "bbox_tight_post", stamp=0.10)

    sel = sync.try_select()
    expect("tie: selection made", sel is not None)
    expect(
        "tie: image item is the post-reset one",
        sel.items[STREAM_IMAGE] == "image_post",
    )
    expect(
        "tie: semantic item is the post-reset one",
        sel.items[STREAM_SEMANTIC] == "semantic_post",
    )


def test_label_stamp_binding():
    sync = make_default_sync()
    sync.observe(STREAM_SEMANTIC_LABELS, "labels_t1", stamp=1.0)
    sync.observe(STREAM_SEMANTIC_LABELS, "labels_t2", stamp=2.0)
    observe_required_core(sync, 2.0, "t2")
    # A later-arriving, later-stamped labels item that is outside
    # tolerance of the eventual anchor -- proves binding is by stamp
    # proximity, not simply "whatever arrived most recently".
    sync.observe(STREAM_SEMANTIC_LABELS, "labels_t2_4", stamp=2.4)

    sel = sync.try_select()
    expect("label bind: selection made", sel is not None)
    expect("label bind: anchors on mask tick", sel.anchor_stamp == 2.0)
    expect(
        "label bind: picks the t=2.0 labels item, not newest arrival",
        sel.items[STREAM_SEMANTIC_LABELS] == "labels_t2",
    )


def test_stale_table_rejection():
    sync = make_default_sync()
    # Labels stream only ever stamped once, long ago; this makes
    # ever_stamped True forever, so it stays in `required`.
    sync.observe(STREAM_SEMANTIC_LABELS, "labels_stale", stamp=1.0)
    observe_required_core(sync, 5.0, "t5")

    sel = sync.try_select()
    expect(
        "stale table: no selection "
        "(fails tolerance at 5.0, fails max_age at 1.0)",
        sel is None,
    )


def test_unstamped_labels_degraded():
    sync = make_default_sync()
    observe_required_core(sync, 2.0, "t2")
    # This bridge's semantic_labels payloads never carry a time_stamp.
    sync.observe(STREAM_SEMANTIC_LABELS, "labels_old", stamp=None)
    sync.observe(STREAM_SEMANTIC_LABELS, "labels_new", stamp=None)

    sel = sync.try_select()
    expect("unstamped: selection made", sel is not None)
    expect("unstamped: degraded status", sel.status == "ok_unstamped_labels")
    expect(
        "unstamped: excluded from required",
        STREAM_SEMANTIC_LABELS not in sel.required,
    )
    expect(
        "unstamped: newest payload attached",
        sel.items[STREAM_SEMANTIC_LABELS] == "labels_new",
    )
    expect(
        "unstamped: stamp is None",
        sel.stamps[STREAM_SEMANTIC_LABELS] is None,
    )


def test_loose_pair_both_or_neither():
    # Only bbox_2d_loose is ever seen; its labels partner never
    # arrives, so the pair must be treated as fully absent.
    only_bbox = make_default_sync()
    observe_required_core(only_bbox, 2.0, "t2")
    only_bbox.observe(STREAM_BBOX_LOOSE, "loose", stamp=2.0)

    sel = only_bbox.try_select()
    expect("loose pair: selection made without loose", sel is not None)
    expect("loose pair: not in scene", sel.loose_in_scene is False)
    expect(
        "loose pair: bbox_loose excluded from required",
        STREAM_BBOX_LOOSE not in sel.required,
    )
    expect(
        "loose pair: bbox_loose item is None",
        sel.items[STREAM_BBOX_LOOSE] is None,
    )

    # Both loose streams seen: the pair becomes required and is bound
    # within tolerance like any other required stream.
    both = make_default_sync()
    observe_required_core(both, 2.0, "t2")
    both.observe(STREAM_BBOX_LOOSE, "loose", stamp=2.0)
    both.observe(STREAM_BBOX_LOOSE_LABELS, "loose_labels", stamp=2.0)

    sel2 = both.try_select()
    expect("loose pair: selection made with loose", sel2 is not None)
    expect("loose pair: in scene", sel2.loose_in_scene is True)
    expect(
        "loose pair: both streams required",
        STREAM_BBOX_LOOSE in sel2.required
        and STREAM_BBOX_LOOSE_LABELS in sel2.required,
    )
    expect(
        "loose pair: both items bound",
        sel2.items[STREAM_BBOX_LOOSE] == "loose"
        and sel2.items[STREAM_BBOX_LOOSE_LABELS] == "loose_labels",
    )


def test_required_core_parameterization():
    # The recorder profile only cares about semantic + bbox_tight.
    sync = make_default_sync(
        required_core=(STREAM_SEMANTIC, STREAM_BBOX_TIGHT)
    )
    sync.observe(STREAM_SEMANTIC, "semantic", stamp=2.0)
    sync.observe(STREAM_BBOX_TIGHT, "bbox_tight", stamp=2.0)

    sel = sync.try_select()
    expect("recorder profile: selection made", sel is not None)
    expect(
        "recorder profile: no image_raw in required",
        STREAM_IMAGE not in sel.required,
    )
    expect(
        "recorder profile: image_raw item is None",
        sel.items[STREAM_IMAGE] is None,
    )


def test_tolerance_property():
    default_sync = make_default_sync()
    expect(
        "tolerance property: default matches 0.0167",
        default_sync.tolerance_s == 0.0167,
    )

    custom_sync = make_default_sync(tolerance_s=0.05)
    expect(
        "tolerance property: reflects a custom constructor value",
        custom_sync.tolerance_s == 0.05,
    )


def test_parse_label_stamp():
    val = parse_label_stamp(LABEL_PAYLOAD_EXAMPLE)
    expect("parse stamp: not None", val is not None)
    expect("parse stamp: value", abs(val - 8.716667121) < 1e-9)

    missing = json.dumps({"0": {"class": "board"}})
    expect(
        "parse stamp: missing key -> None", parse_label_stamp(missing) is None
    )

    expect(
        "parse stamp: malformed JSON -> None",
        parse_label_stamp("{not json") is None,
    )

    not_dict = json.dumps({"time_stamp": "not-a-dict"})
    expect(
        "parse stamp: time_stamp not dict -> None",
        parse_label_stamp(not_dict) is None,
    )


def test_parse_label_payload_ok():
    expect(
        "payload ok: real example",
        parse_label_payload_ok(LABEL_PAYLOAD_EXAMPLE) is True,
    )

    time_stamp_only = json.dumps({"time_stamp": {"sec": 8, "nanosec": 0}})
    expect(
        "payload ok: time_stamp only -> False",
        parse_label_payload_ok(time_stamp_only) is False,
    )

    expect(
        "payload ok: malformed -> False",
        parse_label_payload_ok("{not json") is False,
    )


def test_stream_report_never_received():
    sync = make_default_sync()
    report = sync.stream_report()

    expect("report: all streams present", set(report) == set(ALL_STREAMS))
    for stream in ALL_STREAMS:
        entry = report[stream]
        expect(
            f"report: {stream} ever_received False",
            entry["ever_received"] is False,
        )
        expect(
            f"report: {stream} ever_stamped False",
            entry["ever_stamped"] is False,
        )
        expect(f"report: {stream} buffered 0", entry["buffered"] == 0)
        expect(
            f"report: {stream} newest_stamp None",
            entry["newest_stamp"] is None,
        )


def test_wait_for_selection_unblocks():
    sync = make_default_sync()

    def publish_later():
        time.sleep(0.05)
        observe_required_core(sync, 2.0, "t2")

    thread = threading.Thread(target=publish_later)
    thread.start()
    sel = sync.wait_for_selection(2.0)
    thread.join()

    expect("wait: unblocked with a selection", sel is not None)
    expect(
        "wait: selection has the published item",
        sel.items[STREAM_IMAGE] == "image_t2",
    )

    empty_sync = make_default_sync()
    start = time.monotonic()
    sel2 = empty_sync.wait_for_selection(0.1)
    elapsed = time.monotonic() - start

    expect("wait: empty times out to None", sel2 is None)
    expect("wait: empty returns promptly", elapsed < 1.0)


def test_ever_parsed_flags():
    sync = make_default_sync()
    expect(
        "parsed flags: semantic false initially",
        sync.semantic_labels_ever_parsed() is False,
    )
    expect(
        "parsed flags: bbox_tight false initially",
        sync.bbox_tight_labels_ever_parsed() is False,
    )

    sync.observe(STREAM_SEMANTIC_LABELS, "bad", stamp=None, parsed_ok=False)
    expect(
        "parsed flags: still false after a failed parse",
        sync.semantic_labels_ever_parsed() is False,
    )

    sync.observe(STREAM_SEMANTIC_LABELS, "good", stamp=1.0, parsed_ok=True)
    expect(
        "parsed flags: true after one good parse",
        sync.semantic_labels_ever_parsed() is True,
    )
    expect(
        "parsed flags: bbox_tight still false",
        sync.bbox_tight_labels_ever_parsed() is False,
    )

    sync.observe(STREAM_BBOX_TIGHT_LABELS, "good", stamp=1.0, parsed_ok=True)
    expect(
        "parsed flags: bbox_tight true after one good parse",
        sync.bbox_tight_labels_ever_parsed() is True,
    )


def main():
    tests = [
        test_nearest_selection,
        test_duplicate_stamp_prefers_latest_arrival,
        test_tolerance_rejection,
        test_anchor_falls_back_to_older_coherent,
        test_max_age_rejection,
        test_backward_jump_flushes_all,
        test_rebase_stamp_tie_never_pairs_across_flush,
        test_label_stamp_binding,
        test_stale_table_rejection,
        test_unstamped_labels_degraded,
        test_loose_pair_both_or_neither,
        test_required_core_parameterization,
        test_tolerance_property,
        test_parse_label_stamp,
        test_parse_label_payload_ok,
        test_stream_report_never_received,
        test_wait_for_selection_unblocks,
        test_ever_parsed_flags,
    ]
    for t in tests:
        t()
    print(f"\nAll {len(tests)} stream_sync tests passed.")


if __name__ == "__main__":
    main()
