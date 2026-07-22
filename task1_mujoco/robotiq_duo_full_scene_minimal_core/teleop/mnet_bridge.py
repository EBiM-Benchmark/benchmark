"""ManipulationNet (mnet-client) eval bridge — enable with ``--mnet``.

The mnet-client (../mnet_client-ros_2, ``ros2 run mnet_client local_test`` or
``submission``) is the official ROS 2 middle layer between a robot system and
the ManipulationNet server: it records the camera stream as evidence video,
publishes the ongoing task, and scores finished/skipped reports. This bridge
makes the MuJoCo teleop look like that robot system:

  publish   <camera_image_topic>       sensor_msgs/Image, sim camera >=25fps
  publish   <camera_info_topic>        sensor_msgs/CameraInfo (intrinsics)
  subscribe /mnet_client/ongoing_task                std_msgs/String (Tier1..6)
  subscribe /mnet_client/board_configuration         std_msgs/String — for the
            cable_management task the client renames its instruction topic to
            this and publishes the tier dict incl. the RANDOMIZED fixture
            coordinates (test_coordinates) the board must be set up with;
            mnet_board.apply_board_config() moves the sim fixtures to match
  subscribe /mnet_client/connection_status           std_msgs/Bool
  call      /mnet_client/current_task_finished       std_srvs/Trigger (F key)
  call      /mnet_client/current_task_skipped        std_srvs/Trigger (H key)

The evidence camera is the FIXED overhead camera ``mnet_overhead`` defined in
the scene XML (ceiling mount above the board center, looking straight down,
lens below the room's pendant lamp); pass --mnet-camera viewer to publish the
operator's desktop view instead.

Topic names come from the mnet team_config.json when it is filled in
(--mnet-config, default: sibling mnet_client-ros_2/config/team_config.json);
otherwise the --mnet-camera-topic/--mnet-camera-info-topic defaults are used
— put the SAME topic names into team_config.json so the client finds the
stream.

Requires rclpy (a sourced ROS 2 environment). On this Windows box that means
running the sim inside WSL2/Ubuntu with ROS 2, or a RoboStack conda env; the
mnet client itself must run in the same ROS 2 domain. Without rclpy the
bridge reports itself unavailable and the teleop runs on unaffected.
"""

from __future__ import annotations

import array
import ast
import subprocess
import sys
import json
import math
import threading
import time
from multiprocessing import shared_memory
from pathlib import Path

import numpy as np

import mujoco

from . import log

ONGOING_TASK_TOPIC = "/mnet_client/ongoing_task"
LANGUAGE_INSTRUCTION_TOPIC = "/mnet_client/current_language_instruction"
BOARD_CONFIGURATION_TOPIC = "/mnet_client/board_configuration"
CONNECTION_STATUS_TOPIC = "/mnet_client/connection_status"
TASK_FINISHED_SERVICE = "/mnet_client/current_task_finished"
TASK_SKIPPED_SERVICE = "/mnet_client/current_task_skipped"

# default sim-camera topics; mirror them into team_config.json
DEFAULT_CAMERA_TOPIC = "/mujoco/camera/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/mujoco/camera/camera_info"


def _default_config_path() -> Path:
    # repo layout: d:/mujoco_test/{this folder, mnet_client-ros_2}
    return Path(__file__).resolve().parents[2] / "mnet_client-ros_2" / "config" / "team_config.json"


def _topics_from_config(path: Path) -> tuple[str | None, str | None]:
    """Camera topics from team_config.json, ignoring unfilled placeholders."""
    try:
        cfg = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None, None

    def valid(v) -> str | None:
        return v if isinstance(v, str) and v.startswith("/") else None

    return valid(cfg.get("camera_image_topic")), valid(cfg.get("camera_info_topic"))


class MnetBridge:
    """Owns the ROS 2 node, the offscreen sim camera, and the report calls."""

    def __init__(self, model: mujoco.MjModel, args) -> None:
        try:
            import rclpy
            from rclpy.node import Node  # noqa: F401
            from sensor_msgs.msg import CameraInfo, Image
            from std_msgs.msg import Bool, String
            from std_srvs.srv import Trigger
        except ImportError as exc:
            raise RuntimeError(
                "rclpy is not available - the mnet eval bridge needs a sourced "
                "ROS 2 environment (run the sim under WSL2/Ubuntu with ROS 2, "
                "or a RoboStack conda env). Teleop continues without eval."
            ) from exc

        self._rclpy = rclpy
        self._Image = Image
        self._CameraInfo = CameraInfo
        self._Trigger = Trigger

        self.model = model
        self.width = int(args.mnet_width)
        self.height = int(args.mnet_height)
        self.fps = float(args.mnet_fps)

        # evidence camera: a fixed model camera by default, "viewer" to follow
        # the operator's desktop view
        self.cam_name: str | None = getattr(args, "mnet_camera", "mnet_overhead")
        self._cam_fovy_deg = float(model.vis.global_.fovy)
        if self.cam_name and self.cam_name != "viewer":
            camid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, self.cam_name)
            if camid < 0:
                log(f"[mnet] camera '{self.cam_name}' not found in the model; falling back to the viewer camera")
                self.cam_name = None
            else:
                self._cam_fovy_deg = float(model.cam_fovy[camid])
        else:
            self.cam_name = None

        # resolve camera topics: explicit CLI > team_config.json > defaults
        cfg_path = Path(args.mnet_config) if args.mnet_config else _default_config_path()
        cfg_image, cfg_info = _topics_from_config(cfg_path)
        self.camera_topic = args.mnet_camera_topic or cfg_image or DEFAULT_CAMERA_TOPIC
        self.camera_info_topic = args.mnet_camera_info_topic or cfg_info or DEFAULT_CAMERA_INFO_TOPIC

        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("mujoco_mnet_bridge")
        self._image_pub = self.node.create_publisher(Image, self.camera_topic, 10)
        self._info_pub = self.node.create_publisher(CameraInfo, self.camera_info_topic, 10)

        self.ongoing_task: str | None = None
        self.instruction: str | None = None
        self.client_connected: bool | None = None
        # only this tier is played by hand; every other announced tier is
        # reported as skipped automatically (empty string disables)
        self.target_tier: str = (getattr(args, "mnet_tier", "") or "").strip()
        self._autoskipped: set[str] = set()
        self._board_config_raw: str | None = None
        self._board_config_pending: dict | None = None
        self._config_lock = threading.Lock()
        self.node.create_subscription(String, ONGOING_TASK_TOPIC, self._on_task, 10)
        self.node.create_subscription(String, LANGUAGE_INSTRUCTION_TOPIC, self._on_instruction, 10)
        self.node.create_subscription(String, BOARD_CONFIGURATION_TOPIC, self._on_board_config, 10)
        self.node.create_subscription(Bool, CONNECTION_STATUS_TOPIC, self._on_connection, 10)

        self._finished_cli = self.node.create_client(Trigger, TASK_FINISHED_SERVICE)
        self._skipped_cli = self.node.create_client(Trigger, TASK_SKIPPED_SERVICE)

        # spin in the background; all MuJoCo work stays on the caller's thread
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True, name="mnet-spin")
        self._stop = False
        self._spin_thread.start()

        self._renderer: mujoco.Renderer | None = None
        self._render_thread: threading.Thread | None = None
        self._frame_ready = threading.Event()  # main -> worker: scene snapshot taken
        self._worker_idle = threading.Event()  # worker -> main: ready for the next frame
        self._renderer_up = threading.Event()
        self._shm: shared_memory.SharedMemory | None = None
        self._seq_np = None
        self._hb_np = None
        self._cam_proc: subprocess.Popen | None = None
        self._scene_opt = mujoco.MjvOption()
        self._scene_opt.geomgroup[:] = 0
        for g in (
            0,
            1,
            2,
            5,
        ):  # visual groups incl. the board, no collision geoms
            self._scene_opt.geomgroup[g] = 1
        # never show the operator-only routing hint in the evidence video
        self._scene_opt.sitegroup[5] = 0
        self._next_pub = 0.0
        self._rate_t0: float | None = None
        self._rate_n = 0
        self._rate_warned = False
        self._info_msg = self._make_camera_info()

        log(
            f"[mnet] bridge up: camera '{self.cam_name or 'viewer'}' -> {self.camera_topic} "
            f"({self.width}x{self.height}@{self.fps:g}fps), reports: F=finished H=skipped"
        )
        log(
            f"[mnet] start the client with: ros2 run mnet_client local_test (or submission); "
            f"set camera_image_topic={self.camera_topic} in team_config.json"
        )

    # ------------------------------------------------------------- callbacks
    def _spin(self) -> None:
        while not self._stop and self._rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _on_task(self, msg) -> None:
        if msg.data != self.ongoing_task:
            self.ongoing_task = msg.data
            log(f"[mnet] ongoing task: {msg.data}")
            # the client announces e.g. "Current Task: Tier2" - match by
            # containment, not equality (first real run caught this)
            is_target = bool(self.target_tier) and self.target_tier in msg.data
            if self.target_tier and not is_target and msg.data not in self._autoskipped:
                self._autoskipped.add(msg.data)
                log(f"[mnet] auto-skipping {msg.data} (target tier: {self.target_tier})")
                self.report_skipped()

    def _on_instruction(self, msg) -> None:
        if msg.data and msg.data != self.instruction:
            self.instruction = msg.data
            log(f"[mnet] instruction: {msg.data}")

    def _on_board_config(self, msg) -> None:
        """cable_management: the client publishes the tier's board layout as
        str(dict) every 0.5s, including the randomized test_coordinates the
        board must be configured with. Parse and queue each NEW config once."""
        if not msg.data or msg.data == self._board_config_raw:
            return
        self._board_config_raw = msg.data
        try:
            cfg = ast.literal_eval(msg.data)
        except (ValueError, SyntaxError) as exc:
            log(f"[mnet] board_configuration unparsable: {exc}")
            return
        if not isinstance(cfg, dict):
            return
        with self._config_lock:
            self._board_config_pending = cfg
        n = len(cfg.get("fixture_types", []))
        log(
            f"[mnet] board configuration received: {n} fixtures, cable {cfg.get('cable_length')}, "
            f"offsets {cfg.get('coordinate_offsets')}"
        )

    def consume_board_config(self) -> dict | None:
        """Hand the newest unprocessed board configuration to the main loop."""
        with self._config_lock:
            cfg = self._board_config_pending
            self._board_config_pending = None
        return cfg

    def _on_connection(self, msg) -> None:
        if self.client_connected != bool(msg.data):
            self.client_connected = bool(msg.data)
            log(f"[mnet] server connection: {'up' if msg.data else 'DOWN'}")

    # --------------------------------------------------------------- camera
    def ensure_renderer(self) -> None:
        """Start the out-of-process camera renderer/publisher. In-process
        rendering contends with the viewer's GL contexts (3.5 ms alone ->
        20-24 ms measured) and with mj_step's GIL; the child owns its own
        model copy, GL context and interpreter. The sim side only memcpy's
        its state into shared memory (see maybe_publish)."""
        if self._cam_proc is not None or self._shm is not None:
            return
        from .scene import XML as SCENE_XML

        m = self.model
        qb, bpb, bqb, rgb_ = m.nq * 8, m.nbody * 24, m.nbody * 32, m.ngeom * 16
        self._slot_b = qb + bpb + bqb + rgb_
        self._shm = shared_memory.SharedMemory(create=True, size=64 + 2 * self._slot_b)
        self._seq_np = np.ndarray((1,), np.uint64, self._shm.buf, 0)
        self._seq_np[0] = 0
        self._hb_np = np.ndarray((1,), np.float64, self._shm.buf, 8)
        self._hb_np[0] = time.time()

        def _views(slot: int):
            off = 64 + slot * self._slot_b
            return (
                np.ndarray((m.nq,), np.float64, self._shm.buf, off),
                np.ndarray((m.nbody, 3), np.float64, self._shm.buf, off + qb),
                np.ndarray((m.nbody, 4), np.float64, self._shm.buf, off + qb + bpb),
                np.ndarray((m.ngeom, 4), np.float32, self._shm.buf, off + qb + bpb + bqb),
            )

        self._slot_views = (_views(0), _views(1))
        script = Path(__file__).resolve().parent / "mnet_cam_pub.py"
        cam_log = Path(script).parent.parent / "mnet_cam.log"
        self._cam_log_f = open(cam_log, "w", encoding="utf-8")
        self._cam_proc = subprocess.Popen(
            [
                sys.executable,
                str(script),
                self._shm.name,
                str(SCENE_XML),
                str(self.width),
                str(self.height),
                f"{self.fps:g}",
                self.cam_name or "mnet_overhead",
                self.camera_topic,
                self.camera_info_topic,
            ],
            stdout=self._cam_log_f,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
        )
        log(f"[mnet] camera process log: {cam_log}")
        log(f"[mnet] camera renderer process started (pid {self._cam_proc.pid}; it loads its own model copy, ~20 s)")

    def _make_camera_info(self):
        """Intrinsics of the offscreen render: square pixels, centered
        principal point, fovy from the fixed camera (or the model default
        when following the viewer)."""
        fovy = math.radians(self._cam_fovy_deg)
        fy = 0.5 * self.height / math.tan(0.5 * fovy)
        info = self._CameraInfo()
        info.header.frame_id = "mnet_sim_camera"
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            fy,
            0.0,
            self.width / 2.0,
            0.0,
            fy,
            self.height / 2.0,
            0.0,
            0.0,
            1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            fy,
            0.0,
            self.width / 2.0,
            0.0,
            0.0,
            fy,
            self.height / 2.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def maybe_publish(self, data: mujoco.MjData, viewer_cam=None) -> None:
        """Snapshot the sim state into shared memory (~20 KB memcpy). The
        camera process samples it at the target fps and does all rendering
        and publishing; syncing raw arrays means the code plate (geom_rgba),
        fixture randomization (body_pos/quat) and the cable (qpos) follow
        automatically with no replay logic."""
        self.ensure_renderer()
        if self._shm is None:
            return
        s = int(self._seq_np[0])
        q, bp, bq, rg = self._slot_views[(s + 1) & 1]
        np.copyto(q, data.qpos)
        np.copyto(bp, self.model.body_pos)
        np.copyto(bq, self.model.body_quat)
        np.copyto(rg, self.model.geom_rgba)
        self._hb_np[0] = time.time()
        self._seq_np[0] = s + 1

    # -------------------------------------------------------------- reports
    def _call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            log(f"[mnet] {label}: service not available — is the mnet client running?")
            return

        future = client.call_async(self._Trigger.Request())

        def done(fut) -> None:
            try:
                resp = fut.result()
                log(f"[mnet] {label}: success={resp.success} {resp.message}")
            except Exception as exc:
                log(f"[mnet] {label}: call failed: {exc}")

        future.add_done_callback(done)
        log(f"[mnet] reported: {label} (task: {self.ongoing_task or 'unknown'})")

    def report_finished(self) -> None:
        self._call_trigger(self._finished_cli, "task FINISHED")

    def report_skipped(self) -> None:
        self._call_trigger(self._skipped_cli, "task SKIPPED")

    # -------------------------------------------------------------- cleanup
    def close(self) -> None:
        self._stop = True
        if self._cam_proc is not None:
            try:
                self._cam_proc.terminate()
            except Exception:
                pass
        self._frame_ready.set()  # wake the render worker so it can exit
        if self._render_thread is not None and self._render_thread.is_alive():
            self._render_thread.join(timeout=2.0)
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self.node)
            self.node.destroy_node()
        except Exception:
            pass
        # the renderer is owned and closed by the render worker thread
        self._renderer = None
        if self._shm is not None:
            self._seq_np = None
            self._hb_np = None
            try:
                self._shm.close()
                self._shm.unlink()
            except Exception:
                pass
            self._shm = None
