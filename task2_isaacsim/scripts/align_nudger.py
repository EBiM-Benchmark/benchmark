#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Live room xform nudger — no Isaac relaunch.

Run this on the same machine as the Isaac container (host networking) while
scene_room.py is up with a NuRec room. Watch the WebRTC view and tap keys
here; the sim applies translate / rotate / scale on the next tick.

    python3 task2_isaacsim/scripts/align_nudger.py

Colon (``:``) opens a command line for exact values, e.g. ``:tz 1.24``,
``:rxyz 90 0 0``, or ``:scale 1.2``. ``p`` prints CLI flags you can paste
into the next launch.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import select
import socket
import sys
import termios
import threading
import time
import tty
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
ROOM = "room"
STEP_M = (0.01, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0)
STEP_DEG = (1.0, 5.0, 15.0, 45.0, 90.0)
STEP_S = (0.01, 0.05, 0.1, 0.25)
MIN_SCALE = 0.01
UNIT_SCALE = (1.0, 1.0, 1.0)

HELP = """
  wasd / arrows   translate X/Y     q / e        translate Z
  z / x           rotate Rx         c / v        rotate Ry
  b / n           rotate Rz         + / -        uniform scale
  [ ]  finer/coarser metres           { }  finer/coarser degrees
  ( )  finer/coarser scale
  m    toggle mesh visibility         p    print CLI flags
  0 / R  reset room
  :    type a command (xyz -0.5 0.7 2.2, rxyz -93 0 0, scale 5, …)
  h    this help                      esc / ctrl-c  quit
""".strip(
    "\n"
)


def wrap_deg(value: float) -> float:
    wrapped = (float(value) + 180.0) % 360.0 - 180.0
    return 0.0 if abs(wrapped) < 1e-9 else wrapped


def _v3(values: Iterable[float]) -> list[float]:
    out = [float(v) for v in values]
    if len(out) != 3:
        raise ValueError("expected 3 numbers")
    return out


def as_scale(values: Iterable[float] | None) -> tuple[float, float, float]:
    if not values:
        return UNIT_SCALE
    vals = [float(v) for v in values]
    if len(vals) == 1:
        s = max(MIN_SCALE, vals[0])
        return (s, s, s)
    if len(vals) == 3:
        return (
            max(MIN_SCALE, vals[0]),
            max(MIN_SCALE, vals[1]),
            max(MIN_SCALE, vals[2]),
        )
    raise ValueError("scale needs 1 or 3 numbers")


def _cli_scale(flag: str, scale: list[float]) -> str:
    sx, sy, sz = scale
    if abs(sx - sy) < 1e-9 and abs(sy - sz) < 1e-9:
        return f"{flag} {sx:g}"
    return f"{flag} {sx:g} {sy:g} {sz:g}"


def format_cli(prims: dict[str, dict[str, list[float]]]) -> str:
    room = prims[ROOM]
    return " ".join(
        [
            f"--xyz-deg {room['rxyz'][0]:g} {room['rxyz'][1]:g} {room['rxyz'][2]:g}",
            f"--xyz {room['pos'][0]:g} {room['pos'][1]:g} {room['pos'][2]:g}",
            _cli_scale("--scale", room["scale"]),
        ]
    )


def format_constants(prims: dict[str, dict[str, list[float]]]) -> str:
    return format_cli(prims)


@dataclass
class AlignState:
    target: str = ROOM
    step_m: float = 0.1
    step_deg: float = 1.0
    step_s: float = 0.05
    mesh_visible: bool = False
    prims: dict[str, dict[str, list[float]]] = field(default_factory=dict)

    def snapshot(self) -> dict[str, Any]:
        return {
            "target": self.target,
            "step_m": self.step_m,
            "step_deg": self.step_deg,
            "step_s": self.step_s,
            "mesh_visible": self.mesh_visible,
            "prims": deepcopy(self.prims),
            "cli": format_cli(self.prims),
            "constants": format_constants(self.prims),
        }


def default_prims(
    *,
    xyz: tuple[float, float, float],
    xyz_deg: tuple[float, float, float],
    scale: tuple[float, float, float],
) -> dict[str, dict[str, list[float]]]:
    return {
        ROOM: {
            "pos": list(xyz),
            "rxyz": list(xyz_deg),
            "scale": list(scale),
        },
    }


def _canonical_target(name: str) -> str | None:
    if str(name).lower() in {ROOM, "volume", "mesh", "env"}:
        return ROOM
    return None


def apply_command(state: AlignState, cmd: dict[str, Any]) -> str:
    """Mutate *state* from a JSON command. Returns a short status line."""
    kind = str(cmd.get("cmd", "")).lower()
    if kind in {"select", "target"}:
        target = _canonical_target(str(cmd.get("target", "")))
        if target is None:
            return f"unknown target {cmd.get('target')!r}"
        state.target = target
        return f"target {target}"
    if kind == "step":
        if "pos" in cmd:
            state.step_m = max(1e-4, float(cmd["pos"]))
        if "rot" in cmd:
            state.step_deg = max(0.1, float(cmd["rot"]))
        if "scale" in cmd:
            state.step_s = max(1e-4, float(cmd["scale"]))
        return (
            f"step {state.step_m:g} m / {state.step_deg:g} deg / "
            f"{state.step_s:g} scale"
        )
    if kind == "step_cycle":
        which = str(cmd.get("which", "pos"))
        direction = int(cmd.get("dir", 1))
        series = {"pos": STEP_M, "rot": STEP_DEG, "scale": STEP_S}[which]
        current = {
            "pos": state.step_m,
            "rot": state.step_deg,
            "scale": state.step_s,
        }[which]
        nearest = min(range(len(series)), key=lambda i: abs(series[i] - current))
        nxt = series[max(0, min(len(series) - 1, nearest + direction))]
        if which == "pos":
            state.step_m = float(nxt)
        elif which == "rot":
            state.step_deg = float(nxt)
        else:
            state.step_s = float(nxt)
        return (
            f"step {state.step_m:g} m / {state.step_deg:g} deg / "
            f"{state.step_s:g} scale"
        )
    if kind == "toggle_mesh":
        state.mesh_visible = not state.mesh_visible
        return f"mesh {'visible' if state.mesh_visible else 'hidden'}"
    if kind == "set_mesh":
        state.mesh_visible = bool(cmd.get("visible", True))
        return f"mesh {'visible' if state.mesh_visible else 'hidden'}"
    if kind in {"reset", "reset_target"}:
        return "reset"  # handled by controller (needs baseline)
    if kind == "reset_all":
        return "reset_all"
    if kind == "print":
        return "print"
    if kind == "ping":
        return "ok"
    target = _canonical_target(str(cmd.get("target", state.target)))
    if target is None:
        return f"unknown target {cmd.get('target')!r}"
    prim = state.prims[target]
    if kind == "nudge":
        dpos = _v3(cmd.get("dpos", (0.0, 0.0, 0.0)))
        drxyz = _v3(cmd.get("drxyz", (0.0, 0.0, 0.0)))
        prim.setdefault("scale", list(UNIT_SCALE))
        prim["pos"] = [a + b for a, b in zip(prim["pos"], dpos)]
        prim["rxyz"] = [wrap_deg(a + b) for a, b in zip(prim["rxyz"], drxyz)]
        if "dscale" in cmd:
            dscale = _v3(cmd["dscale"])
            prim["scale"] = [
                max(MIN_SCALE, a + b) for a, b in zip(prim["scale"], dscale)
            ]
        if "dscale_uniform" in cmd:
            delta = float(cmd["dscale_uniform"])
            prim["scale"] = [max(MIN_SCALE, a + delta) for a in prim["scale"]]
        state.target = target
        return _fmt_prim(target, prim)
    if kind == "set":
        prim.setdefault("scale", list(UNIT_SCALE))
        if "pos" in cmd:
            prim["pos"] = _v3(cmd["pos"])
        if "rxyz" in cmd:
            prim["rxyz"] = [wrap_deg(v) for v in _v3(cmd["rxyz"])]
        if "scale" in cmd:
            prim["scale"] = list(as_scale(cmd["scale"]))
        if "pos_x" in cmd:
            prim["pos"][0] = float(cmd["pos_x"])
        if "pos_y" in cmd:
            prim["pos"][1] = float(cmd["pos_y"])
        if "pos_z" in cmd:
            prim["pos"][2] = float(cmd["pos_z"])
        if "rx" in cmd:
            prim["rxyz"][0] = wrap_deg(float(cmd["rx"]))
        if "ry" in cmd:
            prim["rxyz"][1] = wrap_deg(float(cmd["ry"]))
        if "rz" in cmd:
            prim["rxyz"][2] = wrap_deg(float(cmd["rz"]))
        if "sx" in cmd:
            prim["scale"][0] = max(MIN_SCALE, float(cmd["sx"]))
        if "sy" in cmd:
            prim["scale"][1] = max(MIN_SCALE, float(cmd["sy"]))
        if "sz" in cmd:
            prim["scale"][2] = max(MIN_SCALE, float(cmd["sz"]))
        state.target = target
        return _fmt_prim(target, prim)
    return f"unknown cmd {kind!r}"


def _fmt_prim(name: str, prim: dict[str, list[float]]) -> str:
    x, y, z = prim["pos"]
    rx, ry, rz = prim["rxyz"]
    sx, sy, sz = prim.get("scale", list(UNIT_SCALE))
    if abs(sx - sy) < 1e-9 and abs(sy - sz) < 1e-9:
        scale_txt = f"s {sx:.3f}"
    else:
        scale_txt = f"s {sx:.3f} {sy:.3f} {sz:.3f}"
    return (
        f"{name}  t {x:+.3f} {y:+.3f} {z:+.3f}   "
        f"r {rx:+.1f} {ry:+.1f} {rz:+.1f}   {scale_txt}"
    )


def parse_typed_command(text: str, target: str) -> dict[str, Any] | None:
    raw = text.strip()
    if not raw:
        return None
    parts = raw.replace(",", " ").split()
    head = parts[0].lower()
    rest = parts[1:]
    if head == "mesh" and rest and rest[0] in {"on", "off", "show", "hide"}:
        return {
            "cmd": "set_mesh",
            "visible": rest[0] in {"on", "show"},
        }
    if head in {ROOM, "volume", "mesh", "env"}:
        return {"cmd": "select", "target": ROOM}
    if head in {"target", "t"} and rest:
        return {"cmd": "select", "target": rest[0].lower()}
    if head in {"print", "p", "cli"}:
        return {"cmd": "print"}
    if head in {"reset"}:
        return {"cmd": "reset_all" if rest == ["all"] else "reset"}
    if head == "step" and rest:
        if len(rest) == 1:
            return {"cmd": "step", "pos": float(rest[0])}
        if len(rest) == 2:
            return {"cmd": "step", "pos": float(rest[0]), "rot": float(rest[1])}
        return {
            "cmd": "step",
            "pos": float(rest[0]),
            "rot": float(rest[1]),
            "scale": float(rest[2]),
        }
    if head in {"xyz", "pos"} and len(rest) == 3:
        return {"cmd": "set", "target": target, "pos": [float(v) for v in rest]}
    if head in {"rxyz", "rot"} and len(rest) == 3:
        return {"cmd": "set", "target": target, "rxyz": [float(v) for v in rest]}
    if head in {"scale", "s"} and rest:
        return {
            "cmd": "set",
            "target": target,
            "scale": [float(v) for v in rest],
        }
    if head == "sx" and rest:
        return {"cmd": "set", "target": target, "sx": float(rest[0])}
    if head == "sy" and rest:
        return {"cmd": "set", "target": target, "sy": float(rest[0])}
    if head == "sz" and rest:
        return {"cmd": "set", "target": target, "sz": float(rest[0])}
    if head in {"tz", "z"} and rest:
        return {"cmd": "set", "target": target, "pos_z": float(rest[0])}
    if head in {"tx", "x"} and rest:
        return {"cmd": "set", "target": target, "pos_x": float(rest[0])}
    if head in {"ty", "y"} and rest:
        return {"cmd": "set", "target": target, "pos_y": float(rest[0])}
    if head == "rx" and rest:
        return {"cmd": "set", "target": target, "rx": float(rest[0])}
    if head == "ry" and rest:
        return {"cmd": "set", "target": target, "ry": float(rest[0])}
    if head in {"rz", "yaw"} and rest:
        return {"cmd": "set", "target": target, "rz": float(rest[0])}
    if head == "lift" and rest:
        return {"cmd": "set", "target": target, "pos_z": float(rest[0])}
    raise ValueError(
        f"cannot parse {raw!r} — try :tz 1.24  or  :rxyz 90 0 0  or  :scale 1.2"
    )


KEY_NUDGES = {
    "w": ("dpos", (0.0, 1.0, 0.0)),
    "s": ("dpos", (0.0, -1.0, 0.0)),
    "a": ("dpos", (-1.0, 0.0, 0.0)),
    "d": ("dpos", (1.0, 0.0, 0.0)),
    "q": ("dpos", (0.0, 0.0, 1.0)),
    "e": ("dpos", (0.0, 0.0, -1.0)),
    "z": ("drxyz", (1.0, 0.0, 0.0)),
    "x": ("drxyz", (-1.0, 0.0, 0.0)),
    "c": ("drxyz", (0.0, 1.0, 0.0)),
    "v": ("drxyz", (0.0, -1.0, 0.0)),
    "b": ("drxyz", (0.0, 0.0, 1.0)),
    "n": ("drxyz", (0.0, 0.0, -1.0)),
    "+": ("dscale_uniform", 1.0),
    "=": ("dscale_uniform", 1.0),
    "-": ("dscale_uniform", -1.0),
}


def command_for_key(key: str) -> dict[str, Any] | None:
    if key in {"\x1b[A", "UP"}:
        key = "w"
    elif key in {"\x1b[B", "DOWN"}:
        key = "s"
    elif key in {"\x1b[D", "LEFT"}:
        key = "a"
    elif key in {"\x1b[C", "RIGHT"}:
        key = "d"
    if key in KEY_NUDGES:
        kind, delta = KEY_NUDGES[key]
        return {"cmd": "nudge", kind: delta, "_scale": kind}
    if key == "[":
        return {"cmd": "step_cycle", "which": "pos", "dir": -1}
    if key == "]":
        return {"cmd": "step_cycle", "which": "pos", "dir": 1}
    if key == "{":
        return {"cmd": "step_cycle", "which": "rot", "dir": -1}
    if key == "}":
        return {"cmd": "step_cycle", "which": "rot", "dir": 1}
    if key == "(":
        return {"cmd": "step_cycle", "which": "scale", "dir": -1}
    if key == ")":
        return {"cmd": "step_cycle", "which": "scale", "dir": 1}
    if key in {"m"}:
        return {"cmd": "toggle_mesh"}
    if key in {"p"}:
        return {"cmd": "print"}
    if key in {"0"}:
        return {"cmd": "reset"}
    if key == "R":
        return {"cmd": "reset_all"}
    if key in {"h", "?"}:
        return {"cmd": "help"}
    return None


# ---------------------------------------------------------------------------
# Sim-side controller (imported by scene_room.py after SimulationApp).
# ---------------------------------------------------------------------------


class AlignController:
    """Queue TCP/keyboard commands and apply them on the Isaac tick thread."""

    def __init__(
        self,
        *,
        prims: dict[str, Any],
        set_xform: Callable,
        euler_to_quat: Callable,
        initial: dict[str, dict[str, list[float]]],
        mesh_visible: bool,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
    ) -> None:
        self._usd_prims = prims
        self._set_xform = set_xform
        self._euler_to_quat = euler_to_quat
        self._baseline = deepcopy(initial)
        self.state = AlignState(
            mesh_visible=mesh_visible, prims=deepcopy(initial)
        )
        self._queue: queue.Queue = queue.Queue()
        self._clients: list[socket.socket] = []
        self._clients_lock = threading.Lock()
        self._dirty = True
        self._mesh_dirty = True
        self._last_print = ""
        self.host = host
        self.port = port
        self._server_sock: socket.socket | None = None
        self._send_lock = threading.Lock()
        try:
            self._start_server()
            print(
                f"Room aligner on {host}:{port} — "
                "python3 task2_isaacsim/scripts/align_nudger.py",
                flush=True,
            )
        except OSError as exc:
            print(
                f"Warning: room aligner TCP {host}:{port} failed: {exc}",
                file=sys.stderr,
            )

    def bind(self, node) -> None:  # noqa: ARG002 — tick_callback contract
        return

    def tick(self, sim_time: float) -> None:  # noqa: ARG002
        had = False
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            had = True
            sock, cmd = item
            self._handle(sock, cmd)
        if had or self._dirty or self._mesh_dirty:
            self._flush_usd()

    def enqueue(self, cmd: dict[str, Any], sock: socket.socket | None = None) -> None:
        self._queue.put((sock, cmd))

    def _handle(self, sock: socket.socket | None, cmd: dict[str, Any]) -> None:
        kind = str(cmd.get("cmd", "")).lower()
        if kind == "cycle_target":
            kind = "select"
            cmd = {"cmd": "select", "target": ROOM}
        if kind == "nudge" and cmd.get("_scale") == "dpos":
            scale = self.state.step_m
            cmd = {
                "cmd": "nudge",
                "target": ROOM,
                "dpos": [scale * float(v) for v in cmd.get("dpos", (0, 0, 0))],
            }
        elif kind == "nudge" and cmd.get("_scale") == "drxyz":
            scale = self.state.step_deg
            cmd = {
                "cmd": "nudge",
                "target": ROOM,
                "drxyz": [scale * float(v) for v in cmd.get("drxyz", (0, 0, 0))],
            }
        elif kind == "nudge" and cmd.get("_scale") == "dscale_uniform":
            cmd = {
                "cmd": "nudge",
                "target": ROOM,
                "dscale_uniform": self.state.step_s
                * float(cmd.get("dscale_uniform", 0.0)),
            }
        if kind == "help":
            self._reply(sock, "ok", help_text=HELP)
            return
        if kind in {"reset", "reset_all"}:
            self.state.prims = deepcopy(self._baseline)
            self._dirty = True
            self._reply(sock, _fmt_prim(ROOM, self.state.prims[ROOM]))
            return
        if kind == "print":
            text = format_cli(self.state.prims)
            self._last_print = text
            print("\nRoom align snapshot\n" + text, flush=True)
            self._reply(sock, "print", extra={"print": text})
            return
        status = apply_command(self.state, cmd)
        if str(cmd.get("cmd", "")).lower() in {
            "nudge",
            "set",
            "reset",
            "reset_all",
        }:
            self._dirty = True
        if str(cmd.get("cmd", "")).lower() in {"toggle_mesh", "set_mesh"}:
            self._mesh_dirty = True
        self._reply(sock, status)

    def _apply_spec(self, spec: dict[str, list[float]]) -> None:
        prim = self._usd_prims.get("root")
        if prim is None:
            return
        quat = self._euler_to_quat(tuple(spec["rxyz"]))
        pos = tuple(spec["pos"])
        scale = tuple(spec.get("scale", list(UNIT_SCALE)))
        self._set_xform(prim, pos, quat)
        from pxr import Gf, UsdGeom  # noqa: PLC0415

        UsdGeom.Xformable(prim).AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*scale)
        )

    def _flush_usd(self) -> None:
        if self._dirty:
            spec = self.state.prims.get(ROOM)
            if spec is not None:
                try:
                    self._apply_spec(spec)
                except Exception as exc:  # noqa: BLE001 — keep the aligner alive
                    print(f"Warning: room align set_xform: {exc}", file=sys.stderr)
            self._dirty = False
        if self._mesh_dirty:
            mesh = self._usd_prims.get("mesh")
            if mesh is not None:
                try:
                    from pxr import UsdGeom  # noqa: PLC0415

                    imageable = UsdGeom.Imageable(mesh)
                    if self.state.mesh_visible:
                        imageable.MakeVisible()
                    else:
                        imageable.MakeInvisible()
                except Exception as exc:  # noqa: BLE001
                    print(f"Warning: room align mesh vis: {exc}", file=sys.stderr)
            self._mesh_dirty = False

    def _reply(
        self,
        sock: socket.socket | None,
        status: str,
        extra: dict[str, Any] | None = None,
        help_text: str | None = None,
    ) -> None:
        payload = {"ok": True, "status": status, **self.state.snapshot()}
        if extra:
            payload.update(extra)
        if help_text:
            payload["help"] = help_text
        line = json.dumps(payload) + "\n"
        targets = [sock] if sock is not None else []
        with self._clients_lock:
            if sock is None:
                targets = list(self._clients)
            elif sock not in self._clients:
                targets = [sock]
        dead = []
        with self._send_lock:
            for client in targets:
                try:
                    client.sendall(line.encode("utf-8"))
                except OSError:
                    dead.append(client)
        if dead:
            with self._clients_lock:
                self._clients = [c for c in self._clients if c not in dead]

    def _start_server(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(4)
        sock.setblocking(False)
        self._server_sock = sock
        thread = threading.Thread(target=self._accept_loop, daemon=True)
        thread.start()

    def _accept_loop(self) -> None:
        assert self._server_sock is not None
        while True:
            try:
                readable, _, _ = select.select([self._server_sock], [], [], 0.25)
            except OSError:
                return
            if not readable:
                continue
            try:
                client, _addr = self._server_sock.accept()
            except OSError:
                continue
            client.settimeout(0.0)
            with self._clients_lock:
                self._clients.append(client)
            self._reply(client, "connected")
            threading.Thread(
                target=self._client_loop, args=(client,), daemon=True
            ).start()

    def _client_loop(self, client: socket.socket) -> None:
        buf = b""
        try:
            while True:
                try:
                    chunk = client.recv(4096)
                except BlockingIOError:
                    time.sleep(0.02)
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    try:
                        cmd = json.loads(line)
                    except json.JSONDecodeError:
                        self._reply(client, f"bad json: {line[:80]}")
                        continue
                    if not isinstance(cmd, dict):
                        self._reply(client, "json must be an object")
                        continue
                    self.enqueue(cmd, client)
        finally:
            with self._clients_lock:
                self._clients = [c for c in self._clients if c is not client]
            try:
                client.close()
            except OSError:
                pass


# ---------------------------------------------------------------------------
# Host TUI (stdlib only).
# ---------------------------------------------------------------------------


class AlignClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.sock = socket.create_connection((host, port), timeout=3.0)
        self.sock.settimeout(5.0)
        self.state: dict[str, Any] = {}
        self._buf = b""
        self._recv_hello()

    def _recv_hello(self) -> None:
        self.state = self._readline()

    def _readline(self) -> dict[str, Any]:
        while b"\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("aligner disconnected")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    def send(self, cmd: dict[str, Any]) -> dict[str, Any]:
        self.sock.sendall((json.dumps(cmd) + "\n").encode("utf-8"))
        self.state = self._readline()
        return self.state

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


def _render(state: dict[str, Any], status: str, typed: str | None) -> str:
    prims = state.get("prims") or {}
    spec = prims.get(ROOM) or {
        "pos": [0, 0, 0],
        "rxyz": [0, 0, 0],
        "scale": [1, 1, 1],
    }
    rows = [
        f"  room aligner   {state.get('step_m', 0):g} m  "
        f"{state.get('step_deg', 0):g} deg  "
        f"{state.get('step_s', 0):g} scale   "
        f"mesh {'ON ' if state.get('mesh_visible') else 'off'}",
        "  " + "─" * 62,
        f"  ▶ {_fmt_prim(ROOM, spec)}",
        "  " + "─" * 62,
        f"  {status}",
    ]
    if typed is not None:
        rows.append(f"  :{typed}")
    else:
        rows.append("  wasd XY  qe Z  +- scale  zx/cv/bn Rxyz  :scale 1.2  p print")
    return "\n".join(rows)


def _read_key(fd: int) -> str:
    ch = os.read(fd, 1)
    if ch == b"\x1b":
        ready, _, _ = select.select([fd], [], [], 0.03)
        if not ready:
            return "\x1b"
        extra = os.read(fd, 2)
        seq = (ch + extra).decode("latin1", errors="replace")
        return seq
    if ch == b"\x03":
        raise KeyboardInterrupt
    return ch.decode("latin1", errors="replace")


def run_interactive(client: AlignClient) -> int:
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        return run_repl(client)
    old = termios.tcgetattr(fd)
    status = client.state.get("status", "connected")
    typed: str | None = None
    try:
        tty.setcbreak(fd)
        while True:
            sys.stdout.write("\033[H\033[J")
            sys.stdout.write(_render(client.state, status, typed) + "\n")
            sys.stdout.flush()
            ready, _, _ = select.select([fd], [], [], 0.25)
            if not ready:
                continue
            key = _read_key(fd)
            if typed is not None:
                if key in {"\r", "\n"}:
                    try:
                        cmd = parse_typed_command(
                            typed, client.state.get("target", ROOM)
                        )
                    except ValueError as exc:
                        status = str(exc)
                        typed = None
                        continue
                    typed = None
                    if cmd is None:
                        continue
                    reply = client.send(cmd)
                    status = reply.get("status", "")
                    if cmd.get("cmd") == "print":
                        status = "printed — also in Isaac log"
                        sys.stdout.write(
                            "\n" + reply.get("print", reply.get("cli", "")) + "\n"
                        )
                        sys.stdout.flush()
                        time.sleep(0.8)
                    continue
                if key in {"\x7f", "\b"}:
                    typed = typed[:-1]
                    continue
                if key == "\x1b":
                    typed = None
                    continue
                if len(key) == 1 and key.isprintable():
                    typed += key
                continue
            if key in {"\x1b"}:
                return 0
            if key == ":":
                typed = ""
                continue
            cmd = command_for_key(key)
            if cmd is None:
                continue
            if cmd.get("cmd") == "help":
                status = HELP.replace("\n", " | ")
                continue
            reply = client.send(cmd)
            status = reply.get("status", "")
            if cmd.get("cmd") == "print":
                sys.stdout.write(
                    "\n" + reply.get("print", reply.get("cli", "")) + "\n"
                )
                sys.stdout.flush()
                time.sleep(1.2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write("\n")
        if client.state.get("cli"):
            sys.stdout.write(client.state["cli"] + "\n")
        sys.stdout.flush()
    return 0


def run_repl(client: AlignClient) -> int:
    print(_render(client.state, client.state.get("status", ""), None))
    print("line mode — type: tz 1.24 | rxyz 90 0 0 | w | help | quit")
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            return 0
        if not line or line in {"quit", "exit", "q"}:
            return 0
        if line in {"help", "h", "?"}:
            print(HELP)
            continue
        cmd = command_for_key(line) if len(line) == 1 else None
        if cmd is None:
            try:
                cmd = parse_typed_command(
                    line, client.state.get("target", ROOM)
                )
            except ValueError as exc:
                print(exc)
                continue
        if cmd is None:
            continue
        reply = client.send(cmd)
        print(_render(reply, reply.get("status", ""), None))
        if cmd.get("cmd") == "print":
            print(reply.get("print", reply.get("cli", "")))
    return 0


def build_client_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default=os.environ.get("ALIGN_HOST", DEFAULT_HOST))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("ALIGN_PORT", DEFAULT_PORT)),
    )
    parser.add_argument(
        "command",
        nargs="*",
        help="Optional one-shot command (e.g. xyz -3 -5.1 4.2) instead of the TUI.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_client_parser().parse_args(argv)
    try:
        client = AlignClient(args.host, args.port)
    except OSError as exc:
        dest = Path(__file__).name
        print(
            f"Cannot connect to room aligner at {args.host}:{args.port}: {exc}\n"
            "Start the room scene first (NuRec .usdz). The sim prints\n"
            f"  Room aligner on {DEFAULT_HOST}:{DEFAULT_PORT}\n"
            f"then run:  python3 task2_isaacsim/scripts/{dest}",
            file=sys.stderr,
        )
        return 1
    try:
        if args.command:
            cmd = parse_typed_command(
                " ".join(args.command), client.state.get("target", ROOM)
            )
            if cmd is None:
                return 0
            reply = client.send(cmd)
            print(reply.get("status", ""))
            print(reply.get("cli", ""))
            return 0
        return run_interactive(client)
    except KeyboardInterrupt:
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
