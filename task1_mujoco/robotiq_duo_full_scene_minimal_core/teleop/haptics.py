"""Edge-triggered contact haptics, shared by the gamepad and VR front ends.

The raw signal (``session.gripper_contact_force``) stays high for as long as
a touch or grasp persists; driving the motor with it directly buzzes the
whole time and masks new events. ``ContactPulser`` turns it into one short
pulse per NEW contact: fire on the rising edge, stay silent while the
contact persists, and re-arm only after the gripper has been contact-free
for a debounce interval (so a chattering contact cannot machine-gun the
motor).
"""

from __future__ import annotations

from . import config


class ContactPulser:
    def __init__(self, sides: tuple[str, ...] = ("left", "right")) -> None:
        self._in_contact = {s: False for s in sides}
        self._quiet_since: dict[str, float | None] = {s: None for s in sides}

    def update(self, side: str, force: float, now: float) -> float:
        """Feed one tick of contact force (N); returns the pulse amplitude
        (0..1) to fire this tick — nonzero only on a new contact."""
        if force > config.HAPTIC_MIN_FORCE:
            self._quiet_since[side] = None
            if not self._in_contact[side]:
                self._in_contact[side] = True
                return min(
                    1.0,
                    max(config.HAPTIC_PULSE_MIN_AMP, force / config.HAPTIC_FULL_FORCE),
                )
            return 0.0
        if self._in_contact[side]:
            if self._quiet_since[side] is None:
                self._quiet_since[side] = now
            elif now - self._quiet_since[side] >= config.HAPTIC_REARM_S:
                self._in_contact[side] = False
                self._quiet_since[side] = None
        return 0.0
