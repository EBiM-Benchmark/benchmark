"""SteamVR (openvr) fallback backend: input only, no headset rendering.
Use --vr-backend steamvr when an OpenXR runtime is unavailable."""

from __future__ import annotations

import numpy as np

from .vr_mapping import HandState

try:
    import openvr
except ImportError:
    openvr = None


class SteamVRInput:
    """Reads both controllers through a SteamVR background session."""

    def __init__(self):
        if openvr is None:
            raise RuntimeError("openvr is not installed in this environment: pip install openvr")
        self.vr = openvr.init(openvr.VRApplication_Background)
        self.hands = {"left": HandState(), "right": HandState()}

    def close(self) -> None:
        openvr.shutdown()

    def poll(self) -> dict:
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,
            openvr.k_unMaxTrackedDeviceCount,
        )
        for hand in self.hands.values():
            hand.valid = False
        for idx in range(openvr.k_unMaxTrackedDeviceCount):
            if self.vr.getTrackedDeviceClass(idx) != openvr.TrackedDeviceClass_Controller:
                continue
            role = self.vr.getControllerRoleForTrackedDeviceIndex(idx)
            if role == openvr.TrackedControllerRole_LeftHand:
                hand = self.hands["left"]
            elif role == openvr.TrackedControllerRole_RightHand:
                hand = self.hands["right"]
            else:
                continue
            pose = poses[idx]
            if not pose.bPoseIsValid:
                continue
            m = pose.mDeviceToAbsoluteTracking
            hand.rot = np.array(
                [
                    [m[0][0], m[0][1], m[0][2]],
                    [m[1][0], m[1][1], m[1][2]],
                    [m[2][0], m[2][1], m[2][2]],
                ]
            )
            hand.pos = np.array([m[0][3], m[1][3], m[2][3]])
            ok, state = self.vr.getControllerState(idx)
            if ok:
                hand.trigger = float(state.rAxis[1].x)
                hand.grip = float(state.rAxis[2].x)
                hand.stick = np.array([state.rAxis[0].x, state.rAxis[0].y])
                pressed = state.ulButtonPressed
                hand.a = bool(pressed & (1 << openvr.k_EButton_A))
                hand.b = bool(pressed & (1 << openvr.k_EButton_ApplicationMenu))
                hand.stick_click = bool(pressed & (1 << openvr.k_EButton_SteamVR_Touchpad))
            hand.valid = True
        return self.hands
