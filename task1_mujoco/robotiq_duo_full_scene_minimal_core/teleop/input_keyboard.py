"""Keyboard input for the desktop run loop.

The MuJoCo passive viewer's key_callback only fires for PRESS/RELEASE, not
the OS's key-repeat events — so a key held down otherwise looks like a
single tap. Held keys are instead read straight from the OS: GetAsyncKeyState
on Windows, an X11 query_keymap() poll on Linux (both bypass the viewer
callback entirely, so held state is correct regardless of what the callback
forwards). Elsewhere (no Xlib, e.g. macOS) a press-timestamp timeout plus
glfw.get_key polling approximates the same.
Discrete actions (mode switches, gripper, speed) come through the viewer's
key callback but are only QUEUED there: the callback runs on the viewer's
thread, and touching MjData/MjModel from it (mj_forward, seed poses) races
with the main loop's mj_step — an intermittent native crash. The runner
drains the queue on the main thread every frame instead.
"""

from __future__ import annotations

import ctypes
import os
import sys
import time
from collections import deque

import glfw
import numpy as np

from . import config

# Only the CONTINUOUS motion keys are OS-polled; discrete actions arrive via
# the viewer key callback. Motion deliberately avoids letter keys (the viewer
# claims many letters for its own visualization toggles): arrows + PageUp/
# PageDown for motion, Home/End for base turning.
_WIN_KEY_CODES = {
    glfw.KEY_UP: 0x26,
    glfw.KEY_DOWN: 0x28,
    glfw.KEY_LEFT: 0x25,
    glfw.KEY_RIGHT: 0x27,
    glfw.KEY_PAGE_UP: 0x21,
    glfw.KEY_PAGE_DOWN: 0x22,
    glfw.KEY_HOME: 0x24,
    glfw.KEY_END: 0x23,
}

_GET_ASYNC_KEY_STATE = None
if os.name == "nt":
    try:
        _GET_ASYNC_KEY_STATE = ctypes.windll.user32.GetAsyncKeyState
    except Exception:
        _GET_ASYNC_KEY_STATE = None


def windows_key_down(glfw_key: int) -> bool:
    """Physical Windows key state (prevents stale held keys on this platform)."""
    if _GET_ASYNC_KEY_STATE is None:
        return False
    vk = _WIN_KEY_CODES.get(glfw_key)
    if vk is None:
        return False
    return bool(_GET_ASYNC_KEY_STATE(vk) & 0x8000)


# X11 keysym names (core "MISCELLANY" group, always available) for the same
# continuous-motion keys tracked above.
_X11_KEYSYM_NAMES = {
    glfw.KEY_UP: "Up",
    glfw.KEY_DOWN: "Down",
    glfw.KEY_LEFT: "Left",
    glfw.KEY_RIGHT: "Right",
    glfw.KEY_PAGE_UP: "Page_Up",
    glfw.KEY_PAGE_DOWN: "Page_Down",
    glfw.KEY_HOME: "Home",
    glfw.KEY_END: "End",
}

_x11_display = None
_x11_error: str | None = None
_x11_keycodes: dict[int, int] = {}


def _init_x11_polling() -> None:
    """Open a raw connection to the X server so held keys can be queried
    directly (query_keymap), the same way GetAsyncKeyState is used on
    Windows — independent of whether the viewer forwards key-repeat events."""
    global _x11_display, _x11_error
    try:
        from Xlib import XK
        from Xlib.display import Display

        disp = Display()
        for glfw_key, name in _X11_KEYSYM_NAMES.items():
            keysym = XK.string_to_keysym(name)
            keycode = disp.keysym_to_keycode(keysym)
            if keycode:
                _x11_keycodes[glfw_key] = keycode
        _x11_display = disp
    except Exception as exc:
        _x11_display = None
        _x11_error = repr(exc)


def held_key_backend() -> str:
    """Which held-key mechanism is active — logged at startup so a broken
    keyboard (keys cutting out right after the press) is diagnosable from
    the terminal output instead of guesswork."""
    if _GET_ASYNC_KEY_STATE is not None:
        return "GetAsyncKeyState (Windows)"
    if _x11_display is not None:
        return "X11 query_keymap"
    why = _x11_error if _x11_error else "non-Linux platform"
    return (
        f"press-timeout fallback ({why}) - held keys will cut out after "
        f"{config.KEY_HOLD_TIMEOUT:g}s; on Linux: pip install python-xlib "
        "and use an X11/XWayland session"
    )


def x11_key_down(glfw_key: int) -> bool:
    if _x11_display is None:
        return False
    keycode = _x11_keycodes.get(glfw_key)
    if keycode is None:
        return False
    keymap = _x11_display.query_keymap()
    return bool(keymap[keycode // 8] & (1 << (keycode % 8)))


if sys.platform.startswith("linux"):
    _init_x11_polling()


class KeyboardInput:
    """Tracks held keys and produces per-frame motion commands.

    One-shot keys (mode switch, gripper open/close, rotate toggle, speed,
    eval reports) are queued by the viewer-thread callback and must be
    consumed on the main thread via ``drain()``.
    """

    def __init__(self) -> None:
        self.held_keys: set[int] = set()
        self.key_press_time: dict[int, float] = {}
        self._pending: deque[int] = deque()

    def key_callback(self, keycode: int) -> None:
        """Wire this to mujoco.viewer.launch_passive(key_callback=...).
        Runs on the viewer thread — only records state, never touches MuJoCo."""
        key = abs(int(keycode))
        if keycode < 0:
            self.held_keys.discard(key)
            self.key_press_time.pop(key, None)
            return
        self.held_keys.add(key)
        self.key_press_time[key] = time.perf_counter()
        self._pending.append(key)

    def drain(self) -> list[int]:
        """One-shot key presses since the last drain (call from the main loop)."""
        out = []
        while self._pending:
            out.append(self._pending.popleft())
        return out

    def _down_keys(self, window) -> set[int]:
        keys = tuple(_WIN_KEY_CODES)
        if _GET_ASYNC_KEY_STATE is not None:
            return {k for k in keys if windows_key_down(k)}
        if _x11_display is not None:
            return {k for k in keys if x11_key_down(k)}
        now = time.perf_counter()
        down = {k for k in self.held_keys if now - self.key_press_time.get(k, 0.0) <= config.KEY_HOLD_TIMEOUT}
        if window is not None:
            down |= {k for k in keys if glfw.get_key(window, k) == glfw.PRESS}
        return down

    def poll(
        self,
        window,
        mode: str,
        rotate_mode: bool,
        move: float,
        rot: float,
    ) -> tuple[np.ndarray, np.ndarray, bool, bool]:
        """Poll held keys every frame so keyboard teleop behaves like gamepad
        teleop. Returns (base_cmd[4], twist[6], translating?, rotating?);
        twist x/y are still operator-frame (the runner maps them to world).
        """
        base = np.zeros(4, dtype=np.float64)
        twist = np.zeros(6, dtype=np.float64)
        translation = False
        rotation = False
        down = self._down_keys(window)

        def held(*codes: int) -> bool:
            return any(code in down for code in codes)

        if mode == "base":
            # arrows are SCREEN axes here (up = away/into the screen, left =
            # screen-left); the runner maps them into the robot's heading
            # frame via the camera, so driving matches what the operator sees
            if held(glfw.KEY_UP):
                base[0] += 1.0
            if held(glfw.KEY_DOWN):
                base[0] -= 1.0
            if held(glfw.KEY_LEFT):
                base[1] += 1.0
            if held(glfw.KEY_RIGHT):
                base[1] -= 1.0
            if held(glfw.KEY_PAGE_DOWN):
                base[2] -= 1.0
            if held(glfw.KEY_PAGE_UP):
                base[2] += 1.0
            if held(glfw.KEY_END):
                base[3] += 1.0  # End = turn right (matches the VR stick sense)
            if held(glfw.KEY_HOME):
                base[3] -= 1.0
            return base, twist, translation, rotation

        if rotate_mode:
            if held(glfw.KEY_LEFT):
                twist[5] += rot
            if held(glfw.KEY_RIGHT):
                twist[5] -= rot
            if held(glfw.KEY_UP):
                twist[3] += rot
            if held(glfw.KEY_DOWN):
                twist[3] -= rot
            if held(glfw.KEY_PAGE_UP):
                twist[4] += rot
            if held(glfw.KEY_PAGE_DOWN):
                twist[4] -= rot
            rotation = float(np.linalg.norm(twist[3:])) > config.TWIST_DEAD
            return base, twist, translation, rotation

        if held(glfw.KEY_UP):
            twist[0] += move
        if held(glfw.KEY_DOWN):
            twist[0] -= move
        # screen-relative (mirror teleop): left arrow is always the operator's left
        if held(glfw.KEY_LEFT):
            twist[1] -= move
        if held(glfw.KEY_RIGHT):
            twist[1] += move
        if held(glfw.KEY_PAGE_DOWN):
            twist[2] -= move
        if held(glfw.KEY_PAGE_UP):
            twist[2] += move
        translation = float(np.linalg.norm(twist[:3])) > config.TWIST_DEAD
        return base, twist, translation, rotation
