"""Gamepad input with a cross-platform layout guarantee.

Raw pygame joystick indices are NOT portable: the same Xbox pad exposes the
right stick and triggers on different axis numbers on Windows and Linux.
Two backends fix that:

- **SDL GameController** (preferred): pygame._sdl2.controller normalizes the
  layout across OS and vendors via SDL's controller database — the left
  stick is always LEFTX/LEFTY, triggers are always TRIGGERLEFT/RIGHT, and
  A/B/shoulders are the same physical buttons on Windows, WSL and Linux.
- **Raw joystick** (fallback for pads without an SDL mapping): per-platform
  index layout — the historical Windows one (right stick on axes 2/3,
  triggers on 4/5) or the Linux kernel xpad one (0=LX 1=LY 2=LT 3=RX
  4=RY 5=RT; reading the Windows indices there feeds the left trigger's
  -1 rest value into the right stick, and the arm drives itself).

The runner consumes SEMANTIC values only (sticks / triggers / dpad + named
button events), so mapping differences never leak out of this module.
"""

from __future__ import annotations

import sys
import time

import numpy as np

from . import config, log

try:
    import pygame
except ImportError:
    pygame = None

try:
    from pygame._sdl2 import controller as sdl2_controller
except Exception:
    sdl2_controller = None

# semantic one-shot buttons; physical placement per backend:
#   mode_base   = Share/Back          mode_left/right = R1 / L1
#   (operator-facing frame: the robot's own left arm sits on the
#   operator's right when facing it, so the physically-left shoulder
#   button selects the arm that appears on the operator's left)
#   close/open  = Circle(B) / Cross(A)   help = Triangle(Y)
#   speed_up/down = click left / right stick (same convention as VR)
_EVENTS = (
    "mode_base",
    "mode_left",
    "mode_right",
    "close",
    "open",
    "help",
    "speed_up",
    "speed_down",
)


def deadzone(value: float, threshold: float = config.GAMEPAD_DEAD) -> float:
    return 0.0 if abs(value) < threshold else float(value)


class Gamepad:
    """First connected pad; all reads are safe when absent."""

    def __init__(self) -> None:
        self.backend: str | None = None
        self.message = "disabled"
        self._ctrl = None
        self._joy = None
        self.rest: dict[int, float] = {}
        self._ax: dict[str, int] = {"rx": 2, "ry": 3, "lt": 4, "rt": 5}
        self._btn_map: dict[str, tuple[int, ...]] = {}
        self._prev: dict[str, bool] = {name: False for name in _EVENTS}

    # ------------------------------------------------------------- connect
    def connect(self) -> bool:
        if pygame is None:
            self.message = "pygame not installed"
            return False
        pygame.init()

        # 1) SDL GameController: identical layout on every OS
        if sdl2_controller is not None:
            try:
                sdl2_controller.init()
                for idx in range(sdl2_controller.get_count()):
                    if sdl2_controller.is_controller(idx):
                        self._ctrl = sdl2_controller.Controller(idx)
                        self.backend = "controller"
                        name = getattr(self._ctrl, "name", None) or "gamepad"
                        self.message = f"{name} [SDL mapping: layout identical on every OS]"
                        self._btn_map = {
                            "mode_base": (pygame.CONTROLLER_BUTTON_BACK,),
                            "mode_left": (pygame.CONTROLLER_BUTTON_RIGHTSHOULDER,),
                            "mode_right": (pygame.CONTROLLER_BUTTON_LEFTSHOULDER,),
                            "close": (pygame.CONTROLLER_BUTTON_B,),
                            "open": (pygame.CONTROLLER_BUTTON_A,),
                            "help": (pygame.CONTROLLER_BUTTON_Y,),
                            "speed_up": (pygame.CONTROLLER_BUTTON_LEFTSTICK,),
                            "speed_down": (pygame.CONTROLLER_BUTTON_RIGHTSTICK,),
                        }
                        return True
            except Exception as exc:
                log(f"[gamepad] SDL controller backend unavailable ({exc}); trying raw joystick")
                self._ctrl = None

        # 2) raw joystick fallback (project's historical Windows layout)
        pygame.joystick.init()
        if pygame.joystick.get_count() <= 0:
            self.message = "no gamepad"
            return False
        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()
        self.backend = "joystick"
        if sys.platform.startswith("linux"):
            # Linux kernel xpad order: 0=LX 1=LY 2=LT 3=RX 4=RY 5=RT
            self._ax = {"rx": 3, "ry": 4, "lt": 2, "rt": 5}
            layout = "linux-xpad"
        else:
            # historical Windows layout: right stick on 2/3, triggers on 4/5
            self._ax = {"rx": 2, "ry": 3, "lt": 4, "rt": 5}
            layout = "windows"
        self.message = f"{self._joy.get_name()} [raw indices, {layout} layout - verify]"
        self._btn_map = {
            "mode_base": (6,),
            "mode_left": (10, 5),
            "mode_right": (9, 4),
            "close": (1,),
            "open": (0,),
            "help": (3,),
            "speed_down": (7,),
            "speed_up": (8,),
        }
        self._calibrate()
        return True

    def _calibrate(self) -> None:
        """Raw backend only: record each axis' rest value — some drivers idle
        triggers at -1, others at 0, and sticks can rest off-center."""
        for _ in range(8):
            pygame.event.pump()
            time.sleep(0.002)
        self.rest = {i: float(self._joy.get_axis(i)) for i in range(self._joy.get_numaxes())}

    @property
    def connected(self) -> bool:
        return self.backend is not None

    def pump(self) -> None:
        if pygame is not None:
            pygame.event.pump()

    # ------------------------------------------------------- semantic reads
    def _ctrl_axis(self, axis: int, unsigned: bool = False) -> float:
        raw = float(self._ctrl.get_axis(axis))
        return raw / 32767.0 if unsigned else raw / 32768.0

    def _raw_axis(self, idx: int) -> float:
        if idx >= self._joy.get_numaxes():
            return 0.0
        return float(self._joy.get_axis(idx)) - self.rest.get(idx, 0.0)

    def left_stick(self) -> tuple[float, float]:
        if self.backend == "controller":
            return (
                deadzone(self._ctrl_axis(pygame.CONTROLLER_AXIS_LEFTX)),
                deadzone(self._ctrl_axis(pygame.CONTROLLER_AXIS_LEFTY)),
            )
        if self.backend == "joystick":
            return deadzone(self._raw_axis(0)), deadzone(self._raw_axis(1))
        return 0.0, 0.0

    def right_stick(self) -> tuple[float, float]:
        if self.backend == "controller":
            return (
                deadzone(self._ctrl_axis(pygame.CONTROLLER_AXIS_RIGHTX)),
                deadzone(self._ctrl_axis(pygame.CONTROLLER_AXIS_RIGHTY)),
            )
        if self.backend == "joystick":
            return deadzone(self._raw_axis(self._ax["rx"])), deadzone(self._raw_axis(self._ax["ry"]))
        return 0.0, 0.0

    def _trigger(self, ctrl_axis: int, raw_idx: int) -> float:
        if self.backend == "controller":
            value = float(np.clip(self._ctrl_axis(ctrl_axis, unsigned=True), 0.0, 1.0))
            return 0.0 if value < config.TRIGGER_DEAD else value
        if self.backend == "joystick":
            raw = float(self._joy.get_axis(raw_idx)) if raw_idx < self._joy.get_numaxes() else 0.0
            rest = self.rest.get(raw_idx, -1.0)
            if rest < -0.5:
                value = (raw + 1.0) * 0.5
            elif rest > 0.5:
                value = (rest - raw) * 0.5
            else:
                value = raw - rest
            value = float(np.clip(value, 0.0, 1.0))
            return 0.0 if value < config.TRIGGER_DEAD else value
        return 0.0

    def trigger_left(self) -> float:
        raw_idx = self._ax["lt"] if self.backend == "joystick" else 0
        return self._trigger(pygame.CONTROLLER_AXIS_TRIGGERLEFT if pygame else 0, raw_idx)

    def trigger_right(self) -> float:
        raw_idx = self._ax["rt"] if self.backend == "joystick" else 0
        return self._trigger(pygame.CONTROLLER_AXIS_TRIGGERRIGHT if pygame else 0, raw_idx)

    def dpad_x(self) -> int:
        if self.backend == "controller":
            right = int(self._ctrl.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT))
            left = int(self._ctrl.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT))
            return right - left
        if self.backend == "joystick":
            if self._joy.get_numhats() > 0:
                try:
                    hx = int(self._joy.get_hat(0)[0])
                    if hx:
                        return hx
                except Exception:
                    pass
            # some DS4 Windows drivers expose the D-pad as buttons
            if self._raw_button(14):
                return 1
            if self._raw_button(13):
                return -1
        return 0

    def _raw_button(self, idx: int) -> bool:
        return idx < self._joy.get_numbuttons() and bool(self._joy.get_button(idx))

    def _button(self, name: str) -> bool:
        ids = self._btn_map.get(name, ())
        if self.backend == "controller":
            return any(bool(self._ctrl.get_button(i)) for i in ids)
        if self.backend == "joystick":
            return any(self._raw_button(i) for i in ids)
        return False

    def pressed_events(self) -> list[str]:
        """Semantic one-shot events since the last call (rising edges)."""
        out = []
        for name in _EVENTS:
            cur = self._button(name)
            if cur and not self._prev[name]:
                out.append(name)
            self._prev[name] = cur
        return out

    # --------------------------------------------------------------- rumble
    def pulse(self, amplitude: float) -> None:
        """One short haptic pulse (0..1), fired when a gripper makes a new
        contact (edge-triggered by ContactPulser, not a continuous buzz)."""
        if self.backend is None or amplitude <= 0.0:
            return
        try:
            if self.backend == "controller":
                self._ctrl.rumble(0.5 * amplitude, amplitude, config.HAPTIC_PULSE_MS)
            else:
                self._joy.rumble(0.5 * amplitude, amplitude, config.HAPTIC_PULSE_MS)
        except Exception:
            pass  # pad/driver without rumble support
