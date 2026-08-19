#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Organizer-side Task 2 evaluation audit recorder.

Passively watches an evaluation run and records, per episode:
  * one fragmented MP4 per camera, mutually aligned on sim time via
    ROS-stamp PTS;
  * per-frame timestamp JSONLs and low-rate ground-truth/joint dumps;
  * official evaluator calls: Trigger response + artifacts — triggered
    by the [e] key, and (auto_evaluate) automatically on a scene-reset
    REQUEST, so the ending episode's final scene state is scored before
    the reset wipes it;
  * a manifest with camera stats, scene-reset events and provenance.

Episodes segment automatically at /isaac/task2/scene_reset events
(the policy under evaluation may reset the scene itself); the keypress
menu is the manual fallback/override. Topic names come from the shared
contract (config/topics.yaml via scripts/topics.py). The console
mirrors the official demo recorder (services/recording/record_task2.py):
cbreak single-keypress on a TTY, line-mode fallback on piped stdin with
EOF acting as quit. Launch through scripts/run_eval_recorder.sh
(compose profile "eval_recording").
"""

import argparse
import contextlib
import json
import os
import queue
import re
import select
import shutil
import signal
import sys
import termios
import threading
import time
import tty
from datetime import datetime, timezone
from fractions import Fraction
from pathlib import Path

import av
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

# Shared Task 2 topic contract (config/topics.yaml via scripts/topics.py,
# resolved relative to this file so /repo and host checkouts both work).
TASK2_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TASK2_ROOT / "scripts"))
from topics import camera_topic, load_topics  # noqa: E402

# Muxer options from the official recorder's fragmented-MP4 shim
# (record_task2.py): ftyp+moov up front + self-contained fragments, so a
# killed recorder leaves a playable file; flush_packets so a crash loses
# at most the fragment being written. frag_duration (µs of track time =
# sim time here) is added on top: frag_keyframe alone only closes a
# fragment at the NEXT keyframe, and at the render-tied wire rates
# (~2-5 fps under RTF<1) a 60-frame GOP means tens of seconds sit in the
# muxer buffer — a kill in that window would lose them all.
_FRAG_MP4_OPTIONS = {
    "movflags": "+frag_keyframe+empty_moov+default_base_moof",
    "frag_duration": "1000000",
    "flush_packets": "1",
}
_TIME_BASE = Fraction(1, 1000)  # frame PTS in milliseconds of sim time

CONFIG_CLI_ONLY_KEYS = {"help", "config", "eval_name", "episode"}

IDLE_KEYBINDS = {
    "1": ("reset", "publish scene reset request (boundary via the event)"),
    "2": ("start", "force-start a new episode now (manual boundary)"),
    "e": ("evaluate", "run official evaluation, archive into last episode"),
    "s": ("status", "full status panel"),
    "q": ("quit", "quit"),
}
RECORDING_KEYBINDS = {
    "3": ("stop", "stop episode, back to watching"),
    "0": ("discard", "stop + mark episode discarded (data kept)"),
    "e": ("evaluate", "run official evaluation, archive into this episode"),
    "s": ("status", "full status panel"),
    "q": ("quit", "finalize episode and quit"),
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


# ---------------------------------------------------------------------------
# Console (adapted from record_task2.py KeyInput)
# ---------------------------------------------------------------------------
class KeyInput:
    """Single-keypress console input with a line-mode fallback.

    On a TTY, activate() holds the terminal in cbreak mode so menu keys
    act without Enter (cbreak keeps ISIG, so Ctrl-C still raises
    KeyboardInterrupt). Without a TTY (piped stdin) reads stay
    line-buffered and EOF reads as the quit key, so scripted runs work.
    """

    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        self._fd = sys.stdin.fileno() if self.is_tty else None
        self._saved = None

    def activate(self) -> None:
        if self.is_tty and self._saved is None:
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)

    def restore(self) -> None:
        if self._saved is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            self._saved = None

    def read_key(self, timeout=None):
        """One pressed key (lowercased), or None if the timeout expires."""
        if timeout is not None:
            try:
                if not select.select([sys.stdin], [], [], timeout)[0]:
                    return None
            except (OSError, ValueError):
                return "q"
        if self._saved is not None:
            key = os.read(self._fd, 1).decode(errors="replace")
            print(key)  # cbreak disables ECHO; show what was pressed
            return key.lower()
        line = sys.stdin.readline()
        if not line:  # EOF on piped stdin
            return "q"
        return line.strip()[:1].lower()


CONSOLE = KeyInput()


# ---------------------------------------------------------------------------
# Topic contract helpers (adapted from record_task2.py)
# ---------------------------------------------------------------------------
def build_camera_table(topics) -> dict:
    """Camera key -> {topic, shape}; keys mirror the official recorder's
    (--cameras): cameras.robot.*, eval_camera, plus any other top-level
    cameras.* contract entry."""
    entries = dict(topics["cameras"]["robot"])
    entries["eval_camera"] = topics["cameras"]["eval"]
    for key, entry in topics["cameras"].items():
        if key not in ("subtopics", "robot", "eval"):
            entries[key] = entry
    return {
        key: {
            "topic": camera_topic(topics, entry["namespace"], "image"),
            "shape": tuple(entry["shape"]),
        }
        for key, entry in entries.items()
    }


def image_msg_to_array(msg) -> np.ndarray:
    channels = 3
    data = np.frombuffer(msg.data, dtype=np.uint8)
    array = data.reshape(msg.height, msg.step)[:, : msg.width * channels]
    array = array.reshape(msg.height, msg.width, channels)
    if msg.encoding.lower() in ("bgr8", "bgra8"):
        array = array[:, :, ::-1]
    return np.ascontiguousarray(array)


# ---------------------------------------------------------------------------
# Episode bookkeeping
# ---------------------------------------------------------------------------
class Episode:
    """One audit episode directory; the manifest is written by the last
    camera worker to deposit its stats after the episode is rotated out."""

    def __init__(
        self,
        path: Path,
        name: str,
        eval_name: str,
        *,
        started_by,
        start_reset_event,
        start_sim,
        topics_yaml,
        codec_info,
    ):
        self.lock = threading.Lock()
        self.path = path
        self.name = name
        self.eval_name = eval_name
        self.started_by = started_by
        self.start_reset_event = start_reset_event
        self.start_wall = utc_now()
        self.start_sim = start_sim
        self.topics_yaml = topics_yaml
        self.codec_info = codec_info
        self.scene_reset_events = []
        self.camera_stats = {}
        self.evaluator_calls = 0
        self.discarded = False
        self.ended_by = None
        self.end_wall = None
        self.end_sim = None
        self._pending = None  # workers still to deposit after close

    def mark_closed(self, *, ended_by, end_sim, discarded, worker_count):
        with self.lock:
            self.ended_by = ended_by
            self.end_wall = utc_now()
            self.end_sim = end_sim
            self.discarded = discarded
            self._pending = worker_count

    def add_reset_event(self, event) -> None:
        with self.lock:
            self.scene_reset_events.append(event)

    def deposit(self, camera_key, stats) -> None:
        with self.lock:
            self.camera_stats[camera_key] = stats
            self._pending -= 1
            done = self._pending == 0
        if done:
            self.write_manifest()

    def write_manifest(self) -> None:
        with self.lock:
            manifest = {
                "eval_name": self.eval_name,
                "episode": self.name,
                "started_by": self.started_by,
                "ended_by": self.ended_by,
                "start_reset_event": self.start_reset_event,
                "wall_time_start": self.start_wall,
                "wall_time_end": self.end_wall,
                "sim_time_start": self.start_sim,
                "sim_time_end": self.end_sim,
                "topics_yaml": self.topics_yaml,
                "codec": self.codec_info,
                "cameras": self.camera_stats,
                "scene_reset_events": self.scene_reset_events,
                "evaluator_calls_at_close": self.evaluator_calls,
                "discarded": self.discarded,
                "clean_shutdown": True,
            }
        path = self.path / "manifest.json"
        with path.open("w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)
        print(f"\n✔ {self.name}: manifest written ({path})")


# ---------------------------------------------------------------------------
# Per-camera encoder worker
# ---------------------------------------------------------------------------
_STOP = object()


class _Rotate:
    def __init__(self, episode):
        self.episode = episode  # Episode to record into, or None


class CameraWorker:
    """Bounded queue + encoder thread writing one fragmented MP4 and one
    frame-timestamp JSONL per episode. Lazy-opens on the first frame, so
    a silent camera leaves no empty files (its stats say encoded=0)."""

    def __init__(self, key, topic, shape, args, clock_snapshot):
        self.key = key
        self.topic = topic
        self.shape = shape
        self._args = args
        self._clock_snapshot = clock_snapshot
        self.queue = queue.Queue()
        self.recording = False  # gate set by the session around rotates
        self.received = 0
        self.dropped = 0
        self.ep_encoded = 0
        self.last_recv_mono = None
        self._stride_count = 0
        # Worker-thread state
        self._episode = None
        self._container = None
        self._stream = None
        self._jsonl = None
        self._pts_base = None
        self._last_pts = None
        self._encoded = 0
        self._nonmono = 0
        self._dropped_at_open = 0
        self._actual_shape = None
        self._first_stamp = None
        self._last_stamp = None
        self.thread = threading.Thread(
            target=self._run, name=f"enc-{key}", daemon=True
        )

    # -- executor thread ----------------------------------------------------
    def on_image(self, msg) -> None:
        self.received += 1
        self.last_recv_mono = time.monotonic()
        if not self.recording:
            return
        self._stride_count += 1
        if self._args.frame_stride > 1 and (
            self._stride_count % self._args.frame_stride
        ):
            return
        # Manual cap (the queue itself is unbounded so control sentinels
        # from the reset callback never block the executor).
        if self.queue.qsize() >= self._args.queue_maxsize:
            self.dropped += 1
            return
        self.queue.put(("frame", msg, time.time()))

    # -- control (session thread(s)) ----------------------------------------
    def rotate(self, episode) -> None:
        self.queue.put(_Rotate(episode))

    def stop(self) -> None:
        self.queue.put(_STOP)

    # -- worker thread -------------------------------------------------------
    def _run(self) -> None:
        while True:
            item = self.queue.get()
            if item is _STOP:
                self._close_episode()
                return
            if isinstance(item, _Rotate):
                self._close_episode()
                self._episode = item.episode
                self._dropped_at_open = self.dropped
                self.ep_encoded = 0
                self._pts_base = None
                self._last_pts = None
                self._encoded = 0
                self._nonmono = 0
                self._first_stamp = None
                self._last_stamp = None
                continue
            _, msg, recv_wall = item
            if self._episode is None:
                continue
            try:
                self._encode(msg, recv_wall)
            except Exception as exc:  # noqa: BLE001
                print(
                    f"\n[FAIL] {self.key}: encoder error, camera disabled "
                    f"for this episode: {exc}"
                )
                self._teardown_files()

    def _open(self, msg) -> None:
        height, width = msg.height, msg.width
        self._actual_shape = [height, width, 3]
        if (height, width) != tuple(self.shape[:2]):
            print(
                f"\n[WARN] {self.key}: frame {width}x{height} != contract "
                f"{self.shape[1]}x{self.shape[0]}; recording actual size"
            )
        videos = self._episode.path / "videos"
        frames = self._episode.path / "frames"
        videos.mkdir(exist_ok=True)
        frames.mkdir(exist_ok=True)
        self._container = av.open(
            str(videos / f"{self.key}.mp4"), "w", options=_FRAG_MP4_OPTIONS
        )
        gop = max(1, round(self._args.gop_s * self._args.fps))
        self._stream = self._container.add_stream(
            self._args.vcodec,
            rate=self._args.fps,
            options={
                "preset": self._args.preset,
                "crf": str(self._args.crf),
                "g": str(gop),
            },
        )
        self._stream.width = width
        self._stream.height = height
        self._stream.pix_fmt = "yuv420p"
        self._stream.codec_context.time_base = _TIME_BASE
        self._jsonl = (frames / f"{self.key}.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )

    def _encode(self, msg, recv_wall) -> None:
        if self._container is None:
            self._open(msg)
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )
        if self._pts_base is None:
            self._pts_base = stamp_ns
            self._first_stamp = stamp_ns / 1e9
        self._last_stamp = stamp_ns / 1e9
        pts = (stamp_ns - self._pts_base) // 1_000_000
        if self._last_pts is not None and pts <= self._last_pts:
            pts = self._last_pts + 1
            self._nonmono += 1
        self._last_pts = pts
        frame = av.VideoFrame.from_ndarray(
            image_msg_to_array(msg), format="rgb24"
        ).reformat(format="yuv420p")
        frame.pts = pts
        frame.time_base = _TIME_BASE
        for packet in self._stream.encode(frame):
            self._container.mux(packet)
        self._jsonl.write(
            json.dumps(
                {
                    "i": self._encoded,
                    "pts_ms": int(pts),
                    "stamp": stamp_ns / 1e9,
                    "recv_wall": recv_wall,
                    "clock": self._clock_snapshot(),
                }
            )
            + "\n"
        )
        self._encoded += 1
        self.ep_encoded = self._encoded

    def _teardown_files(self) -> None:
        if self._container is not None:
            try:
                for packet in self._stream.encode(None):
                    self._container.mux(packet)
                self._container.close()
            except Exception as exc:  # noqa: BLE001
                print(f"\n[WARN] {self.key}: error closing video: {exc}")
            self._container = None
            self._stream = None
        if self._jsonl is not None:
            self._jsonl.close()
            self._jsonl = None

    def _close_episode(self) -> None:
        episode = self._episode
        if episode is None:
            return
        self._teardown_files()
        episode.deposit(
            self.key,
            {
                "topic": self.topic,
                "contract_shape": list(self.shape),
                "actual_shape": self._actual_shape,
                "encoded": self._encoded,
                "dropped": self.dropped - self._dropped_at_open,
                "nonmonotonic": self._nonmono,
                "first_stamp": self._first_stamp,
                "last_stamp": self._last_stamp,
                "status": "recorded" if self._encoded else "missing",
            },
        )
        self._episode = None
        self._actual_shape = None


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
class AuditNode(Node):
    def __init__(self, args, topics, workers):
        super().__init__("eval_audit_recorder")
        self.lock = threading.Lock()
        self.session = None  # wired up after Session construction
        self.sim_time = None
        self.sim_wall = None
        self.clock_count = 0
        self.joint_state_count = 0
        self.object_poses_count = 0
        self.pad_points_count = 0
        self.last_object_poses = None
        self.reset_count = 0
        self.last_reset_event = None

        depth = 10
        self.create_subscription(Clock, topics["clock"], self._on_clock, depth)
        for worker in workers.values():
            self.create_subscription(
                Image,
                worker.topic,
                worker.on_image,
                qos_profile_sensor_data,
            )
        ground_truth = topics["ground_truth"]
        self.create_subscription(
            String, ground_truth["scene_reset"], self._on_scene_reset, depth
        )
        # Any reset request on the wire (policy- or organizer-initiated,
        # including our own [1] key) — the auto-evaluate hook.
        self.create_subscription(
            String,
            ground_truth["scene_reset_request"],
            self._on_reset_request,
            depth,
        )
        self.create_subscription(
            String,
            ground_truth["object_poses"],
            self._on_object_poses,
            depth,
        )
        self.create_subscription(
            Float32MultiArray,
            ground_truth["pad_points"],
            self._on_pad_points,
            depth,
        )
        self.create_subscription(
            JointState,
            topics["recording"]["joint_states_full"],
            self._on_joint_states,
            depth,
        )
        self._reset_request_pub = self.create_publisher(
            String, ground_truth["scene_reset_request"], depth
        )
        self._eval_client = self.create_client(Trigger, args.evaluate_service)

    def _on_clock(self, msg) -> None:
        with self.lock:
            self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9
            self.sim_wall = time.monotonic()
            self.clock_count += 1

    def _on_scene_reset(self, msg) -> None:
        try:
            event = json.loads(msg.data)
        except (TypeError, ValueError):
            event = {"raw": msg.data}
        with self.lock:
            self.reset_count += 1
            self.last_reset_event = event
        if self.session is not None:
            self.session.on_scene_reset(event)

    def _on_reset_request(self, _msg) -> None:
        if self.session is not None:
            self.session.on_reset_request()

    def _on_object_poses(self, msg) -> None:
        with self.lock:
            self.object_poses_count += 1
            self.last_object_poses = msg.data
        if self.session is not None:
            self.session.on_dump("object_poses", msg.data)

    def _on_pad_points(self, msg) -> None:
        with self.lock:
            self.pad_points_count += 1

    def _on_joint_states(self, msg) -> None:
        with self.lock:
            self.joint_state_count += 1
        if self.session is not None:
            self.session.on_dump(
                "joint_states",
                {
                    "stamp": msg.header.stamp.sec
                    + msg.header.stamp.nanosec / 1e9,
                    "name": list(msg.name),
                    "position": [round(p, 6) for p in msg.position],
                },
            )

    def clock_snapshot(self):
        with self.lock:
            return self.sim_time

    def publish_scene_reset_request(self) -> None:
        self._reset_request_pub.publish(String())

    def call_evaluate(self, timeout_s):
        """Trigger the official evaluation service; returns (ok, message)
        where ok is None on transport failure (service absent/timeout).
        timeout_s (--evaluate-timeout-s) bounds the call itself, on top
        of a fixed 3 s service-discovery wait."""
        if not self._eval_client.wait_for_service(timeout_sec=3.0):
            return None, "evaluator service not available (eval_task2 up?)"
        future = self._eval_client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                return None, f"service call timed out after {timeout_s:.0f}s"
            time.sleep(0.1)
        response = future.result()
        return bool(response.success), response.message


# ---------------------------------------------------------------------------
# Session: episode lifecycle + evaluator capture + dumps
# ---------------------------------------------------------------------------
class Session:
    def __init__(self, args, node, workers, session_dir: Path, topics_yaml):
        self.args = args
        self.node = node
        self.workers = workers
        self.session_dir = session_dir
        self.topics_yaml = topics_yaml
        self.lock = threading.Lock()
        self.episode = None
        self.last_episode = None
        self.pending_reset_request = None  # monotonic time of last request
        # Set when the [1] key already scored the open episode; the echo of
        # our own request on the wire must not score it a second time.
        self._suppress_request_eval = False
        self.last_eval_summary = None
        self._eval_in_flight = threading.Lock()
        self._eval_thread = None  # last auto-evaluate thread, for shutdown
        # eval_out artifacts already attributed to a call record; only
        # ever touched under _eval_in_flight (serializes the external
        # watcher against the recorder's own call diffs).
        self._claimed_files = set()
        self._external_baseline = False
        self._dump_files = {}
        self._dump_counts = {}
        self._first_episode_name = args.episode
        self._codec_info = {
            "vcodec": args.vcodec,
            "preset": args.preset,
            "crf": args.crf,
            "fps_hint": args.fps,
            "gop_s": args.gop_s,
            "frame_stride": args.frame_stride,
            "pts": "ros_stamp_ms_rebased_per_episode",
        }

    # -- episode lifecycle ---------------------------------------------------
    def _next_episode_name(self) -> str:
        if self._first_episode_name:
            name = self._first_episode_name
            self._first_episode_name = None
            return name
        taken = [
            int(m.group(1))
            for d in self.session_dir.iterdir()
            if d.is_dir()
            for m in [re.fullmatch(r"ep_(\d+)", d.name)]
            if m
        ]
        return f"ep_{max(taken, default=0) + 1:03d}"

    def _open_episode_locked(self, started_by, reset_event):
        name = self._next_episode_name()
        path = self.session_dir / name
        path.mkdir(parents=True, exist_ok=True)
        episode = Episode(
            path,
            name,
            self.args.eval_name,
            started_by=started_by,
            start_reset_event=reset_event,
            start_sim=self.node.clock_snapshot(),
            topics_yaml=self.topics_yaml,
            codec_info=self._codec_info,
        )
        provenance = self.session_dir / ".session_provenance.json"
        if provenance.is_file():
            shutil.copyfile(provenance, path / "provenance.json")
        if self.args.dump_topics:
            dumps = path / "dumps"
            dumps.mkdir(exist_ok=True)
            for key in ("object_poses", "joint_states"):
                self._dump_files[key] = (dumps / f"{key}.jsonl").open(
                    "w", encoding="utf-8", buffering=1
                )
                self._dump_counts[key] = 0
        return episode

    def _close_episode_locked(self, ended_by, discarded=False) -> None:
        episode = self.episode
        if episode is None:
            return
        for handle in self._dump_files.values():
            handle.close()
        self._dump_files.clear()
        episode.mark_closed(
            ended_by=ended_by,
            end_sim=self.node.clock_snapshot(),
            discarded=discarded,
            worker_count=len(self.workers),
        )
        self.last_episode = episode
        self.episode = None

    def start_episode(self, started_by, reset_event=None) -> None:
        """Start a new episode; if one is running it is finalized first
        (a single rotate per worker closes old + opens new)."""
        with self.lock:
            if self.episode is not None:
                self._close_episode_locked(ended_by=started_by)
                for worker in self.workers.values():
                    worker.recording = False
            episode = self._open_episode_locked(started_by, reset_event)
            self.episode = episode
            for worker in self.workers.values():
                worker.rotate(episode)
                worker.recording = True
        print(f"\n▶ {episode.name}: recording ({started_by}) → {episode.path}")

    def stop_episode(self, ended_by, discarded=False) -> None:
        with self.lock:
            if self.episode is None:
                print("\n(no episode running)")
                return
            name = self.episode.name
            for worker in self.workers.values():
                worker.recording = False
            self._close_episode_locked(ended_by, discarded=discarded)
            for worker in self.workers.values():
                worker.rotate(None)
        print(
            f"\n■ {name}: stopped ({ended_by}"
            + (", discarded)" if discarded else ")")
        )

    def evaluate_before_reset(self) -> None:
        """Console [1] path: score the open episode's final state BEFORE
        the reset request goes out. The threaded on_reset_request hook
        races the bridge (which acts on the same message within a second
        or two) and can sample the already-wiped scene; here the request
        is not published yet, so the evaluation is guaranteed pre-reset.
        The request's echo (we subscribe to the topic we publish on) is
        suppressed so the state isn't scored twice."""
        with self.lock:
            episode = self.episode
        if not self.args.auto_evaluate or episode is None:
            return
        print(
            f"\n⚑ scoring {episode.name}'s final state before the reset "
            "request goes out"
        )
        self._suppress_request_eval = True
        self.evaluate(trigger="reset_request", episode=episode)

    def on_reset_request(self) -> None:
        """A scene reset was REQUESTED but has not happened yet. For
        external requesters (a policy that resets its own scene) this is
        the only moment to score the ending episode — best-effort: the
        evaluation runs threaded and races the bridge's reset. Our own
        [1] key evaluates synchronously in evaluate_before_reset instead
        and suppresses this hook for the request's echo."""
        self.pending_reset_request = time.monotonic()
        if self._suppress_request_eval:
            self._suppress_request_eval = False
            return
        with self.lock:
            episode = self.episode
        if not self.args.auto_evaluate or episode is None:
            return
        print(
            f"\n⚑ reset requested — evaluating {episode.name}'s last "
            "frame before the reset"
        )
        thread = threading.Thread(
            target=self.evaluate,
            kwargs={"trigger": "reset_request", "episode": episode},
            name="auto-evaluate",
            daemon=True,  # shutdown joins it bounded; daemon is the net
        )
        self._eval_thread = thread
        thread.start()

    def on_scene_reset(self, event) -> None:
        """Executor-thread hook: auto mode segments episodes here."""
        with self.lock:
            self.pending_reset_request = None
        summary = {
            k: event.get(k) for k in ("reset_index", "sim_time", "randomized")
        }
        print(f"\n↺ scene reset event: {summary}")
        if self.args.auto_episode:
            self.start_episode("scene_reset", reset_event=event)
        else:
            with self.lock:
                if self.episode is not None:
                    self.episode.add_reset_event(event)

    def on_dump(self, key, payload) -> None:
        with self.lock:
            handle = self._dump_files.get(key)
            if handle is None:
                return
            self._dump_counts[key] += 1
            if (self._dump_counts[key] - 1) % self.args.dump_stride:
                return
            if key == "object_poses":
                line = {"recv_wall": time.time(), "data": payload}
            else:
                line = {"recv_wall": time.time(), **payload}
            handle.write(json.dumps(line) + "\n")

    # -- evaluator capture ---------------------------------------------------
    def evaluate(self, trigger="manual", episode=None) -> None:
        if episode is None:
            with self.lock:
                episode = self.episode or self.last_episode
        if episode is None:
            print(
                "\n[FAIL] no episode to attach the evaluation to "
                "(start one first)"
            )
            return
        if not self._eval_in_flight.acquire(blocking=False):
            print("\n(evaluation already in flight — skipped)")
            return
        try:
            self._evaluate_locked(trigger, episode)
        finally:
            self._eval_in_flight.release()

    def _evaluate_locked(self, trigger, episode) -> None:
        eval_dir = (
            Path(self.args.eval_out_dir) if self.args.eval_out_dir else None
        )
        before = set()
        if eval_dir is None:
            print(
                "[WARN] eval_out_dir not configured "
                "(artifacts will not be archived)"
            )
        elif eval_dir.is_dir():
            before = {p.name for p in eval_dir.iterdir() if p.is_file()}
        else:
            print(
                f"[WARN] evaluator output dir not mounted/found: "
                f"{eval_dir} (artifacts will not be archived)"
            )
        print("… calling official evaluation service")
        ok, message = self.node.call_evaluate(self.args.evaluate_timeout_s)
        new_files = []
        if eval_dir is not None and eval_dir.is_dir():
            new_files = sorted(
                p.name
                for p in eval_dir.iterdir()
                if p.is_file() and p.name not in before
            )
            self._claimed_files.update(new_files)
        self._record_call(trigger, episode, ok, message, new_files, eval_dir)

    def _record_call(
        self, trigger, episode, ok, message, new_files, eval_dir
    ) -> None:
        """Archive one evaluator call (own or external) into the
        episode's evaluator/ dir: call_NNN/ + calls.jsonl."""
        calls_dir = episode.path / "evaluator"
        calls_dir.mkdir(exist_ok=True)
        call_index = 1 + sum(
            1
            for d in calls_dir.iterdir()
            if d.is_dir() and d.name.startswith("call_")
        )
        call_dir = calls_dir / f"call_{call_index:03d}"
        artifacts_dir = call_dir / "artifacts"
        artifacts_dir.mkdir(parents=True, exist_ok=True)
        for name in new_files:
            shutil.copy2(eval_dir / name, artifacts_dir / name)
        iou = None
        iou_files = [
            n
            for n in new_files
            if n.startswith("eval_camera_iou_") and n.endswith(".json")
        ]
        if iou_files:
            with (eval_dir / sorted(iou_files)[-1]).open() as f:
                iou = json.load(f)
        result = {
            "call_index": call_index,
            "trigger": trigger,
            "wall_time_utc": utc_now(),
            "sim_time": self.node.clock_snapshot(),
            "episode": episode.name,
            "success": ok,
            "message": message,
            "iou": iou,
            "artifacts": new_files,
            "note": "message paths are evaluator-container paths "
            "(/output/evaluate)",
        }
        with (call_dir / "evaluator_result.json").open("w") as f:
            json.dump(result, f, indent=2)
        with (calls_dir / "calls.jsonl").open("a") as f:
            f.write(
                json.dumps(
                    {
                        "call": call_index,
                        "trigger": trigger,
                        "wall_time_utc": result["wall_time_utc"],
                        "success": ok,
                        "iou": (iou or {}).get(
                            "iou_thermalpad_vs_target_current"
                        ),
                        "orientation_ok": (iou or {}).get(
                            "is_orientation_correct"
                        ),
                    }
                )
                + "\n"
            )
        with episode.lock:
            episode.evaluator_calls += 1
        if trigger == "external":
            summary = (
                f"external eval #{call_index}: "
                f"IoU={(iou or {}).get('iou_thermalpad_vs_target_current')} "
                f"orientation_ok={(iou or {}).get('is_orientation_correct')}"
            )
        elif ok is None:
            summary = f"[FAIL] evaluation not captured: {message}"
        elif iou is not None:
            summary = (
                f"eval #{call_index} ({trigger}): success={ok} "
                f"IoU={iou.get('iou_thermalpad_vs_target_current')} "
                f"orientation_ok={iou.get('is_orientation_correct')} "
                f"case={iou.get('orientation_case')}"
            )
        else:
            summary = f"eval #{call_index}: success={ok} ({message})"
        self.last_eval_summary = summary
        print(
            f"\n{summary}\n   archived {len(new_files)} artifact(s) → "
            f"{call_dir}"
        )

    # -- external evaluator calls (--watch-external-evals) -------------------
    def scan_external_evals(self) -> None:
        """Attribute evaluator artifacts written by Trigger calls the
        recorder did not make (e.g. a policy calling the service
        directly) to episodes. ROS 2 service calls are point-to-point
        and unobservable by third parties — only this artifact side
        effect in eval_out_dir is."""
        if not (self.args.watch_external_evals and self.args.eval_out_dir):
            return
        eval_dir = Path(self.args.eval_out_dir)
        if not eval_dir.is_dir():
            return
        if not self._eval_in_flight.acquire(blocking=False):
            return  # a recorder-initiated call owns the dir diff now
        try:
            entries = {p.name: p for p in eval_dir.iterdir() if p.is_file()}
            if not self._external_baseline:
                # First sight of the dir: whatever is already there
                # predates the session.
                self._external_baseline = True
                self._claimed_files.update(entries)
                return
            fresh = sorted(set(entries) - self._claimed_files)
            if not fresh:
                return
            self._claimed_files.update(fresh)
            for episode, names in self._attribute_external(fresh, entries):
                if episode is None:
                    print(
                        f"\n[WARN] external evaluator artifacts with no "
                        f"episode to attach: {', '.join(names)}"
                    )
                    continue
                print(
                    f"\n⚑ external evaluator call detected — archiving "
                    f"{len(names)} artifact(s) into {episode.name}"
                )
                self._record_call(
                    "external",
                    episode,
                    None,
                    "external Trigger call observed via artifacts",
                    names,
                    eval_dir,
                )
        finally:
            self._eval_in_flight.release()

    def _attribute_external(self, fresh, entries):
        """Group fresh artifact names by target episode: a file older
        than the current episode's start belongs to the previous one
        (an external call racing an episode rotation scored the
        previous scene)."""
        with self.lock:
            current, last = self.episode, self.last_episode
        if current is None and last is None:
            return [(None, fresh)]
        if current is None or last is None:
            return [(current or last, fresh)]
        try:
            cur_start = datetime.fromisoformat(current.start_wall).timestamp()
        except ValueError:
            return [(current, fresh)]
        groups = {}
        for name in fresh:
            try:
                older = entries[name].stat().st_mtime < cur_start
            except OSError:
                older = False
            groups.setdefault(last if older else current, []).append(name)
        return list(groups.items())

    # -- shutdown ------------------------------------------------------------
    def _finish_evaluations(self, ended_by) -> None:
        """Settle evaluator state before episodes close: join an
        in-flight auto-evaluate (bounded), then — with evaluate_on_quit —
        score a still-recording, never-evaluated episode, so a session's
        last attempt is not lost just because no further scene-reset
        request will arrive."""
        thread = self._eval_thread
        if thread is not None and thread.is_alive():
            bound = self.args.evaluate_timeout_s + 5.0
            print(f"waiting for the in-flight evaluation (≤{bound:.0f}s)…")
            thread.join(timeout=bound)
            if thread.is_alive():
                print(
                    "[WARN] in-flight evaluation still running at "
                    "shutdown — its archive may be incomplete"
                )
        # Late external artifacts both get archived and count as the
        # episode being scored (suppressing evaluate_on_quit below).
        self.scan_external_evals()
        if not (self.args.evaluate_on_quit and self.args.auto_evaluate):
            return
        with self.lock:
            episode = self.episode
        if episode is None:  # stopped/discarded episodes are deliberate
            return
        with episode.lock:
            calls = episode.evaluator_calls
        if calls:
            return
        print(
            f"\n⚑ quitting with {episode.name} unevaluated — scoring its "
            "final state first (--no-evaluate-on-quit disables this)"
        )
        self.evaluate(trigger=ended_by, episode=episode)

    def shutdown(self, ended_by) -> None:
        try:
            self._finish_evaluations(ended_by)
        except KeyboardInterrupt:
            print("\n(final evaluation aborted — continuing shutdown)")
        print("draining encoders…")
        with self.lock:
            if self.episode is not None:
                for worker in self.workers.values():
                    worker.recording = False
                self._close_episode_locked(ended_by)
                for worker in self.workers.values():
                    worker.rotate(None)
        for worker in self.workers.values():
            worker.stop()
        for worker in self.workers.values():
            worker.thread.join(timeout=30)
            if worker.thread.is_alive():
                print(f"[WARN] {worker.key}: encoder did not drain in 30s")


# ---------------------------------------------------------------------------
# Status display
# ---------------------------------------------------------------------------
class StatusReporter:
    def __init__(self, args, node, session, workers, dropped_cameras):
        self.args = args
        self.node = node
        self.session = session
        self.workers = workers
        self.dropped_cameras = dropped_cameras
        self.start_mono = time.monotonic()
        self.latched = set()
        self.timeout_warned = False
        self._prev_counts = {key: 0 for key in workers}
        self._prev_tick = time.monotonic()
        self._prev_sim = None
        self.fps = {key: 0.0 for key in workers}
        self.rtf = None

    def tick(self) -> None:
        now = time.monotonic()
        interval = max(now - self._prev_tick, 1e-6)
        for key, worker in self.workers.items():
            self.fps[key] = (
                worker.received - self._prev_counts[key]
            ) / interval
            self._prev_counts[key] = worker.received
            if key not in self.latched and worker.received:
                self.latched.add(key)
                print(f"● latched {key}: {worker.topic}")
        sim = self.node.clock_snapshot()
        if sim is not None and self._prev_sim is not None:
            self.rtf = (sim - self._prev_sim) / interval
        self._prev_sim = sim
        self._prev_tick = now
        if (
            not self.timeout_warned
            and now - self.start_mono > self.args.stream_timeout_s
        ):
            self.timeout_warned = True
            silent = [k for k, w in self.workers.items() if not w.received]
            if self.node.clock_snapshot() is None:
                print(
                    f"[WARN] no sim clock after "
                    f"{self.args.stream_timeout_s:.0f}s — is the scene "
                    "running with --record?"
                )
            if silent:
                optional = set(self.args.optional_cameras.split(","))
                print(
                    f"[WARN] silent cameras after "
                    f"{self.args.stream_timeout_s:.0f}s: "
                    + ", ".join(
                        f"{k}{' (optional)' if k in optional else ''}"
                        for k in silent
                    )
                )
        self.session.scan_external_evals()
        print(self.line())
        self.write_status_json()

    def line(self) -> str:
        with self.session.lock:
            episode = self.session.episode
            state = f"RECORDING {episode.name}" if episode else "WATCHING"
            started = episode.started_by if episode else None
        sim = self.node.clock_snapshot()
        live = sum(
            1
            for w in self.workers.values()
            if w.last_recv_mono is not None
            and time.monotonic() - w.last_recv_mono < 5.0
        )
        silent = [
            k
            for k, w in self.workers.items()
            if w.last_recv_mono is None
            or time.monotonic() - w.last_recv_mono >= 5.0
        ]
        parts = [f"[{state}" + (f" by {started}]" if started else "]")]
        parts.append(
            f"sim {sim:.1f}s" + (f" RTF~{self.rtf:.2f}" if self.rtf else "")
            if sim is not None
            else "sim: no clock"
        )
        cams = f"cams {live}/{len(self.workers)} live"
        if silent and live:
            cams += f" ({','.join(silent)} silent)"
        parts.append(cams)
        if episode:
            frames = sum(w.ep_encoded for w in self.workers.values())
            drops = sum(w.dropped for w in self.workers.values())
            parts.append(f"frames {frames} drop {drops}")
        with self.node.lock:
            resets = self.node.reset_count
            last = self.node.last_reset_event or {}
        if resets:
            parts.append(f"resets {resets} (last #{last.get('reset_index')})")
        if self.session.pending_reset_request is not None:
            parts.append("reset requested…")
        return " | ".join(parts)

    def panel(self) -> None:
        print("\n" + "-" * 66)
        print(self.line())
        print(
            f" out: {self.session.session_dir}   contract: "
            f"{self.session.topics_yaml}"
        )
        mode = (
            "AUTO (scene_reset segments episodes)"
            if self.args.auto_episode
            else "MANUAL"
        )
        print(
            f" mode: {mode}   codec: {self.args.vcodec}/"
            f"{self.args.preset} crf {self.args.crf}"
        )
        for key, worker in self.workers.items():
            age = (
                "never"
                if worker.last_recv_mono is None
                else f"{time.monotonic() - worker.last_recv_mono:4.1f}s ago"
            )
            print(
                f"   {key:12s} {worker.topic:38s} recv "
                f"{worker.received:7d}  {self.fps[key]:5.1f} fps  "
                f"ep_frames {worker.ep_encoded:6d}  "
                f"drop {worker.dropped:4d}  last {age}"
            )
        if self.dropped_cameras:
            print(
                f"   (dropped from contract: "
                f"{', '.join(self.dropped_cameras)})"
            )
        with self.node.lock:
            print(
                f"   clock msgs {self.node.clock_count}, joint_states "
                f"{self.node.joint_state_count}, object_poses "
                f"{self.node.object_poses_count}, pad_points "
                f"{self.node.pad_points_count}"
            )
            objects = None
            if self.node.last_object_poses:
                try:
                    payload = json.loads(self.node.last_object_poses)
                    objects = len(payload.get("objects", []))
                except (TypeError, ValueError):
                    pass
            print(
                f"   scene: objects={objects} resets={self.node.reset_count}"
                f" last_reset={self.node.last_reset_event}"
            )
        if self.session.last_eval_summary:
            print(f"   {self.session.last_eval_summary}")
        print("-" * 66)

    def write_status_json(self) -> None:
        with self.session.lock:
            episode = self.session.episode
            status = {
                "ts": utc_now(),
                "state": "recording" if episode else "watching",
                "episode": episode.name if episode else None,
            }
        status["sim_time"] = self.node.clock_snapshot()
        status["rtf"] = self.rtf
        status["cameras"] = {
            key: {
                "received": worker.received,
                "fps": round(self.fps[key], 2),
                "ep_encoded": worker.ep_encoded,
                "dropped": worker.dropped,
            }
            for key, worker in self.workers.items()
        }
        with self.node.lock:
            status["resets"] = self.node.reset_count
        path = self.session.session_dir / ".status.json"
        with path.open("w", encoding="utf-8") as f:
            json.dump(status, f)


# ---------------------------------------------------------------------------
# Console loop
# ---------------------------------------------------------------------------
def print_menu(args, camera_keys) -> None:
    hint = (
        "press once, no Enter needed"
        if CONSOLE.is_tty
        else "no TTY: type + Enter"
    )
    mode = (
        "AUTO — scene_reset events segment episodes"
        if args.auto_episode
        else "MANUAL — [2]/[3] drive episodes"
    )
    print("\n" + "=" * 66)
    print(f" Task 2 evaluation audit recorder — {args.eval_name}")
    print(f"   cameras: {', '.join(camera_keys)}")
    print(f"   mode: {mode}")
    print(
        "   auto-evaluate on reset request: "
        + ("on" if args.auto_evaluate else "off")
    )
    print(
        "   external eval watch: "
        + ("on" if args.watch_external_evals else "off")
    )
    print(f" Keys ({hint}):")
    seen = set()
    for keybinds, label in (
        (IDLE_KEYBINDS, "idle"),
        (RECORDING_KEYBINDS, "while recording"),
    ):
        print(f"  {label}:")
        for key, (_, description) in keybinds.items():
            if (key, description) not in seen:
                seen.add((key, description))
                print(f"   [{key}] {description}")
    print("=" * 66 + "\n")


def key_loop(args, session, reporter, stop_event) -> str:
    while not stop_event.is_set():
        try:
            key = CONSOLE.read_key(timeout=args.status_interval_s)
        except KeyboardInterrupt:
            return "quit"
        if stop_event.is_set():
            return "signal"
        if key is None:
            reporter.tick()
            continue
        if key == "q":
            return "quit"
        if key == "1":
            session.evaluate_before_reset()
            session.pending_reset_request = time.monotonic()
            session.node.publish_scene_reset_request()
            print("→ scene reset requested (waiting for the reset event)")
        elif key == "2":
            session.start_episode("manual")
        elif key == "3":
            session.stop_episode("manual")
        elif key == "0":
            session.stop_episode("discard", discarded=True)
        elif key == "e":
            session.evaluate()
        elif key == "s":
            reporter.tick()
            reporter.panel()
        elif key.strip():
            print(f"(unbound key {key!r} — [s] status, [q] quit)")
    return "signal"


# ---------------------------------------------------------------------------
# Config / args (precedence: argparse defaults < YAML < CLI flags,
# mirroring services/recording/record_task2.py)
# ---------------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="YAML config (defaults: eval_recording.yaml next to this script)",
    )
    parser.add_argument(
        "--out-root",
        type=str,
        default="services/eval_recording/eval_raw_out",
        help="audit output root; relative paths resolve "
        "against task2_isaacsim/",
    )
    parser.add_argument(
        "--eval_name",
        type=str,
        required=True,
        help="evaluation session name (output subdirectory)",
    )
    parser.add_argument(
        "--episode",
        type=str,
        default=None,
        help="name for the first episode (default ep_NNN)",
    )
    parser.add_argument(
        "--video-cameras",
        type=str,
        default="head,wrist_left,wrist_right,eval_camera",
    )
    parser.add_argument(
        "--optional-cameras",
        type=str,
        default="",
        help="cameras dropped with a warning when their "
        "contract entry or topic is absent",
    )
    parser.add_argument(
        "--auto-episode",
        default=True,
        action=argparse.BooleanOptionalAction,
        help="segment episodes at scene_reset events",
    )
    parser.add_argument(
        "--manual",
        dest="auto_episode",
        action="store_false",
        help="alias for --no-auto-episode",
    )
    parser.add_argument(
        "--record-preamble",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="also record from session start to the first reset",
    )
    parser.add_argument(
        "--auto-evaluate",
        default=True,
        action=argparse.BooleanOptionalAction,
        help="on a scene-reset request, evaluate the ending "
        "episode's final scene state before the reset",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="encoder rate hint (PTS comes from ROS stamps)",
    )
    parser.add_argument("--vcodec", type=str, default="libx264")
    parser.add_argument("--preset", type=str, default="ultrafast")
    parser.add_argument("--crf", type=int, default=23)
    parser.add_argument(
        "--gop-s",
        type=float,
        default=2.0,
        help="keyframe interval (fragment size), seconds",
    )
    parser.add_argument(
        "--frame-stride",
        type=int,
        default=1,
        help="encode every Nth received frame per camera",
    )
    parser.add_argument("--queue-maxsize", type=int, default=120)
    parser.add_argument("--stream-timeout-s", type=float, default=20.0)
    parser.add_argument(
        "--dump-topics",
        default=True,
        action=argparse.BooleanOptionalAction,
        help="dump object_poses/joint_states JSONLs",
    )
    parser.add_argument("--dump-stride", type=int, default=6)
    parser.add_argument("--status-interval-s", type=float, default=5.0)
    parser.add_argument(
        "--evaluate-service", type=str, default="/isaac/eval_camera/evaluate"
    )
    parser.add_argument(
        "--eval-out-dir",
        type=str,
        default=None,
        help="official evaluator output dir (ro mount); "
        "unset = skip artifact archiving",
    )
    parser.add_argument(
        "--evaluate-timeout-s",
        type=float,
        default=30.0,
        help="deadline (s) for ONE official-evaluator Trigger call — "
        "used by [e], auto-evaluate and evaluate-on-quit, on top of a "
        "fixed 3 s service-discovery wait (unrelated to "
        "--stream-timeout-s, the silent-stream warning threshold)",
    )
    parser.add_argument(
        "--evaluate-on-quit",
        default=True,
        action=argparse.BooleanOptionalAction,
        help="on quit/SIGTERM while an episode is recording with no "
        "evaluator calls yet (and auto-evaluate on), run one final "
        "evaluation before shutdown so the session's last attempt is "
        "scored",
    )
    parser.add_argument(
        "--watch-external-evals",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="poll eval-out-dir (each status tick) for evaluator "
        "artifacts written by Trigger calls the recorder did not make "
        "— e.g. a policy calling the service directly — and archive "
        "them into the matching episode as trigger=external call "
        "records; requires --eval-out-dir",
    )
    return parser


def load_config(config_path, parser) -> dict:
    """YAML defaults; fails hard on unknown/CLI-only keys or bad values."""
    import yaml

    config_path = Path(config_path)
    if not config_path.is_file():
        raise SystemExit(f"Config not found: {config_path}")
    with open(config_path, encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    if not isinstance(config, dict):
        raise SystemExit(f"Config must be a YAML mapping: {config_path}")
    actions = {}
    for action in parser._actions:
        if action.dest not in CONFIG_CLI_ONLY_KEYS:
            actions.setdefault(action.dest, action)
    defaults = {}
    for key, value in config.items():
        action = actions.get(key)
        if action is None or action.dest != key:
            raise SystemExit(
                f"Unknown or CLI-only key {key!r} in {config_path}; "
                f"valid keys: {sorted(actions)}"
            )
        if value is None:
            continue
        if isinstance(action, argparse.BooleanOptionalAction):
            if not isinstance(value, bool):
                raise SystemExit(
                    f"Key {key!r} in {config_path} must be a boolean"
                )
        elif isinstance(value, list):
            value = ",".join(str(item) for item in value)
        elif action.type is not None:
            try:
                value = action.type(value)
            except (TypeError, ValueError) as exc:
                raise SystemExit(
                    f"Key {key!r} in {config_path}: bad value {value!r}"
                ) from exc
        defaults[key] = value
    return defaults


# ---------------------------------------------------------------------------
def main() -> None:
    parser = build_arg_parser()
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument(
        "--config",
        type=str,
        default=str(Path(__file__).resolve().parent / "eval_recording.yaml"),
    )
    config_args, _ = bootstrap.parse_known_args()
    parser.set_defaults(**load_config(config_args.config, parser))
    args = parser.parse_args()

    topics = load_topics()
    import topics as topics_module

    topics_yaml = str(topics_module.TOPICS_YAML)
    table = build_camera_table(topics)
    requested = [k.strip() for k in args.video_cameras.split(",") if k.strip()]
    optional = {k.strip() for k in args.optional_cameras.split(",")}
    camera_keys, dropped = [], []
    for key in requested:
        if key in table:
            camera_keys.append(key)
        elif key in optional:
            dropped.append(key)
        else:
            raise SystemExit(
                f"Camera {key!r} not in the loaded contract "
                f"({topics_yaml}); available: {sorted(table)}"
            )
    if dropped:
        print(
            f"[WARN] optional cameras absent from {topics_yaml}: "
            f"{', '.join(dropped)} — recording without them"
        )
    if not camera_keys:
        raise SystemExit("No cameras left to record")

    out_root = Path(args.out_root)
    if not out_root.is_absolute():
        out_root = TASK2_ROOT / out_root
    session_dir = out_root / args.eval_name
    session_dir.mkdir(parents=True, exist_ok=True)
    if args.episode and (session_dir / args.episode).exists():
        raise SystemExit(
            f"Episode dir already exists: {session_dir / args.episode}"
        )

    rclpy.init()
    node_holder = {}

    def clock_snapshot():
        node = node_holder.get("node")
        return node.clock_snapshot() if node else None

    workers = {
        key: CameraWorker(
            key,
            table[key]["topic"],
            table[key]["shape"],
            args,
            clock_snapshot,
        )
        for key in camera_keys
    }
    node = AuditNode(args, topics, workers)
    node_holder["node"] = node
    session = Session(args, node, workers, session_dir, topics_yaml)
    node.session = session
    session.scan_external_evals()  # baseline: claim pre-session artifacts
    reporter = StatusReporter(args, node, session, workers, dropped)

    for worker in workers.values():
        worker.thread.start()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin():
        # Shutdown races are expected here.
        with contextlib.suppress(Exception):
            executor.spin()

    spin_thread = threading.Thread(target=_spin, name="executor", daemon=True)
    spin_thread.start()

    stop_event = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stop_event.set())

    print_menu(args, camera_keys)
    print(f"contract: {topics_yaml}")
    print(f"output:   {session_dir}")
    if args.record_preamble:
        session.start_episode("session_start")
    elif args.auto_episode:
        print(
            "watching — recording starts at the first scene_reset event "
            "([2] to force-start now)\n"
        )

    CONSOLE.activate()
    try:
        ended_by = key_loop(args, session, reporter, stop_event)
    finally:
        CONSOLE.restore()
    print(f"\nshutting down ({ended_by})…")
    session.shutdown(ended_by)
    executor.shutdown(timeout_sec=2.0)
    node.destroy_node()
    rclpy.try_shutdown()
    spin_thread.join(timeout=5)
    print("done.")


if __name__ == "__main__":
    main()
