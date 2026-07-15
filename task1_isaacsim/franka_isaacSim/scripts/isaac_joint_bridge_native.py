#!/usr/bin/env python3
"""Run the ROS joint bridge inside a native Isaac Kit app started via --exec."""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Add services to path for camera service import
_SERVICES_DIR = os.path.join(SCRIPT_DIR, "..", "services")
if _SERVICES_DIR not in sys.path:
    sys.path.insert(0, _SERVICES_DIR)

import omni.kit.app

from isaac_bridge_assets import _resolve_usd_path
from isaac_bridge_constants import (
    DEFAULT_LAYOUT_PATH,
    DEFAULT_COMMAND_SMOOTHING_ALPHA,
    DEFAULT_MAX_POSITION_STEP_RAD,
    DEFAULT_PHYSICS_HZ,
    DEFAULT_PHYSICS_SUBSTEPS,
    DEFAULT_PRIMARY_CONTROLLER_NAME,
    DEFAULT_POSITION_DEADBAND_RAD,
    DEFAULT_RENDER_HZ,
    DEFAULT_SETTLE_POSITION_WINDOW_RAD,
    DEFAULT_SETTLE_VELOCITY_THRESHOLD_RAD_S,
)
from isaac_camera_service.camera_config import DEFAULT_CAMERA_CONFIG_PATH
from isaac_joint_drive_config import DEFAULT_JOINT_DRIVE_CONFIG_PATH
from isaac_bridge_session import initialize_bridge_session
from stack_config import add_config_argument, apply_config_defaults


def _reapply_layout(layout_path):
    """Re-apply the UI layout after stage load so the streaming viewport rebinds."""
    if not layout_path or not os.path.exists(layout_path):
        return
    try:
        import omni.kit.window.layout as _layout_ext
        _layout_ext.load_layout(layout_path)
        print(f"Viewport layout refreshed: {layout_path}")
    except Exception as err:
        print(f"Warning: Could not refresh layout '{layout_path}': {err}")


class NativeBridgeRunner:
    """Attach the bridge to the currently running Kit app."""

    def __init__(self, args):
        self._args = args
        self._app = omni.kit.app.get_app()
        self._session = None
        self._update_sub = None
        self._shutdown_sub = None
        self._layout_refreshed = False

    def start(self):
        usd_path, should_exit = _resolve_usd_path(self._args)
        if should_exit:
            self._app.post_quit()
            return
        self._args.usd_path = usd_path
        print(f"Selected USD asset: {self._args.usd_path}")

        self._session = initialize_bridge_session(
            self._args,
            updater=self._app,
            render_enabled=True,
        )
        if self._session is None:
            self._app.post_quit()
            return

        self._update_sub = self._app.get_update_event_stream().create_subscription_to_pop(
            self._on_update,
            name="IsaacJointBridgeNative::update",
        )
        self._shutdown_sub = (
            self._app.get_shutdown_event_stream().create_subscription_to_pop_by_type(
                omni.kit.app.POST_QUIT_EVENT_TYPE,
                self._on_shutdown,
                name="IsaacJointBridgeNative::shutdown",
                order=0,
            )
        )

    def shutdown(self):
        self._update_sub = None
        self._shutdown_sub = None
        if self._session is not None:
            self._session.shutdown()
            self._session = None

    def _on_update(self, _event):
        if not self._layout_refreshed:
            self._layout_refreshed = True
            _reapply_layout(DEFAULT_LAYOUT_PATH)
        if self._session is None:
            return
        try:
            self._session.tick()
        except Exception as error:
            print(f"Error: Native bridge update loop failed: {error}")
            self.shutdown()
            self._app.post_quit()

    def _on_shutdown(self, _event):
        self.shutdown()


def _build_parser():
    parser = argparse.ArgumentParser()
    add_config_argument(parser)
    parser.add_argument(
        "--embodiment",
        type=str,
        default="fr3duo_m+v",
        help="Embodiment name to load configuration from (default: fr3duo_m+v)",
    )
    parser.add_argument(
        "--assets-dir",
        type=str,
        default="/workspace/assets",
        help="Directory to scan for USD assets",
    )
    parser.add_argument(
        "--usd-path",
        type=str,
        default=None,
        help="Path to robot USD to load",
    )
    parser.add_argument(
        "--asset-name",
        type=str,
        default=None,
        help="Select USD by filename (with or without .usd) from --assets-dir",
    )
    parser.add_argument(
        "--asset-index",
        type=int,
        default=None,
        help="Select USD by index from the discovered asset list",
    )
    parser.add_argument(
        "--select-asset",
        action="store_true",
        help="Force interactive asset selection menu",
    )
    parser.add_argument(
        "--list-assets",
        action="store_true",
        help="List discovered USD assets and exit",
    )
    parser.add_argument(
        "--ros-publish-rate",
        type=float,
        default=60.0,
        help="Joint-state publish rate in Hz for /isaac/*_joint_states",
    )
    parser.add_argument(
        "--physics-backend",
        choices=("physx", "newton"),
        default=os.getenv("ISAAC_PHYSICS_BACKEND", "physx"),
        help="Physics backend to activate inside Isaac before simulation starts",
    )
    parser.add_argument(
        "--physics-hz",
        type=float,
        default=DEFAULT_PHYSICS_HZ,
        help="Isaac physics stepping frequency in Hz",
    )
    parser.add_argument(
        "--render-hz",
        type=float,
        default=DEFAULT_RENDER_HZ,
        help="Render frequency in Hz (physics steps are decimated to this rate)",
    )
    parser.add_argument(
        "--physics-substeps",
        type=int,
        default=DEFAULT_PHYSICS_SUBSTEPS,
        help="Number of physics substeps requested from Isaac",
    )
    parser.add_argument(
        "--browser-command-hold-seconds",
        type=float,
        default=0.25,
        help="Time window to prioritize browser override command topics",
    )
    parser.add_argument(
        "--command-smoothing-alpha",
        type=float,
        default=DEFAULT_COMMAND_SMOOTHING_ALPHA,
        help="Low-pass factor for position commands (1.0 disables smoothing)",
    )
    parser.add_argument(
        "--max-position-step-rad",
        type=float,
        default=DEFAULT_MAX_POSITION_STEP_RAD,
        help="Maximum commanded joint position change per physics step in radians",
    )
    parser.add_argument(
        "--position-deadband-rad",
        type=float,
        default=DEFAULT_POSITION_DEADBAND_RAD,
        help="Ignore tiny target errors inside this joint-position deadband",
    )
    parser.add_argument(
        "--settle-position-window-rad",
        type=float,
        default=DEFAULT_SETTLE_POSITION_WINDOW_RAD,
        help="Snap targets to the current joint position once error is small enough",
    )
    parser.add_argument(
        "--settle-velocity-threshold-rad-s",
        type=float,
        default=DEFAULT_SETTLE_VELOCITY_THRESHOLD_RAD_S,
        help="Maximum joint speed allowed when applying the settle snap window",
    )
    parser.add_argument(
        "--robot-prim-path",
        type=str,
        default=None,
        help="Explicit robot articulation prim path (optional override)",
    )
    parser.add_argument(
        "--joint-drive-config",
        type=str,
        default=DEFAULT_JOINT_DRIVE_CONFIG_PATH,
        help="YAML file containing Isaac joint-drive settings for FR3 joints",
    )
    parser.add_argument(
        "--joint-drive-stiffness-scale",
        type=float,
        default=1.0,
        help="Global scale factor applied to configured Isaac joint-drive stiffness",
    )
    parser.add_argument(
        "--joint-drive-damping-scale",
        type=float,
        default=1.0,
        help="Global scale factor applied to configured Isaac joint-drive damping",
    )
    parser.add_argument(
        "--joint-drive-max-force-scale",
        type=float,
        default=1.0,
        help="Global scale factor applied to configured Isaac joint-drive max force (arm joints)",
    )
    parser.add_argument(
        "--joint-drive-gripper-stiffness-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to gripper joint-drive stiffness",
    )
    parser.add_argument(
        "--joint-drive-gripper-damping-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to gripper joint-drive damping",
    )
    parser.add_argument(
        "--joint-drive-gripper-max-force-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to gripper joint-drive max force",
    )
    parser.add_argument(
        "--camera-config",
        type=str,
        default=DEFAULT_CAMERA_CONFIG_PATH,
        help="YAML file containing Isaac camera sensor settings for FR3 duo",
    )
    parser.add_argument(
        "--primary-controller",
        type=str,
        default=DEFAULT_PRIMARY_CONTROLLER_NAME,
        help=(
            "Name of the ROS2 controller that gates primary arm commands. "
            "Leave empty to accept arm commands without external controller gating."
        ),
    )
    apply_config_defaults(parser, ("simulation", "bridge", "isaac_joint_drives", "cameras"))
    return parser


RUNNER = None


def main():
    global RUNNER
    parser = _build_parser()
    args, _ = parser.parse_known_args()
    RUNNER = NativeBridgeRunner(args)
    RUNNER.start()


if __name__ == "__main__":
    main()
