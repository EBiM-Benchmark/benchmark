#!/usr/bin/env python3
"""Isaac Sim service that exposes robot joints on raw `/isaac/*` ROS topics."""

import argparse
import os
import sys

# Add services to path for camera service import
_SERVICES_DIR = os.path.join(os.path.dirname(__file__), "..", "services")
if _SERVICES_DIR not in sys.path:
    sys.path.insert(0, _SERVICES_DIR)

from isaac_bridge_assets import _resolve_usd_path
from isaac_bridge_constants import (
    CONFIG,
    DEFAULT_COMMAND_SMOOTHING_ALPHA,
    DEFAULT_CONTROLLER_ACTIVITY_TOPIC,
    DEFAULT_LAYOUT_PATH,
    DEFAULT_MAX_POSITION_STEP_RAD,
    DEFAULT_PHYSICS_HZ,
    DEFAULT_PHYSICS_SUBSTEPS,
    DEFAULT_PRIMARY_EFFORT_STALE_AFTER_S,
    DEFAULT_POSITION_DEADBAND_RAD,
    DEFAULT_RENDER_HZ,
    DEFAULT_SETTLE_POSITION_WINDOW_RAD,
    DEFAULT_SETTLE_VELOCITY_THRESHOLD_RAD_S,
)
from isaac_camera_service.camera_config import DEFAULT_CAMERA_CONFIG_PATH
from isaac_joint_drive_config import DEFAULT_JOINT_DRIVE_CONFIG_PATH
from isaac_bridge_session import initialize_bridge_session
from isaac_bridge_runtime import (
    _configure_layout_defaults,
    _create_simulation_app,
    _enable_webrtc_extension,
    _has_available_display,
    _prepare_simulation_app_argv,
    _resolve_simulation_experience,
)
from stack_config import add_config_argument, apply_config_defaults


def main():
    parser = argparse.ArgumentParser()
    add_config_argument(parser)
    parser.add_argument(
        "--embodiment",
        type=str,
        default="fr3duo_m+v",
        help="Embodiment name to load configuration from (default: fr3duo_m+v)",
    )
    parser.add_argument("--headless", action="store_true", help="Run in headless mode")
    parser.add_argument("--stream", action="store_true", help="Enable WebRTC livestream")
    parser.add_argument(
        "--stream-ui",
        action="store_true",
        help="Run stream mode with full Kit UI (requires an available DISPLAY/X server)",
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
        help=(
            "Time window to prioritize browser override command topics "
            "after the latest browser update"
        ),
    )
    parser.add_argument(
        "--controller-activity-topic",
        type=str,
        default=DEFAULT_CONTROLLER_ACTIVITY_TOPIC,
        help="ControllerManagerActivity topic used to gate primary arm effort control",
    )
    parser.add_argument(
        "--primary-effort-stale-after-s",
        type=float,
        default=DEFAULT_PRIMARY_EFFORT_STALE_AFTER_S,
        help="Fallback hold timeout once primary effort commands stop changing",
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
    apply_config_defaults(parser, ("simulation", "bridge", "isaac_joint_drives", "cameras"))
    args, unknown = parser.parse_known_args()

    usd_path, should_exit = _resolve_usd_path(args)
    if should_exit:
        return
    args.usd_path = usd_path
    print(f"Selected USD asset: {args.usd_path}")

    # SimulationApp forwards unknown CLI args to Kit. Keep only caller-supplied
    # unknown args and strip this script's flags to avoid Kit-side misparsing.
    stream_ui_requested = bool(args.stream_ui)
    if args.stream and stream_ui_requested and not _has_available_display():
        print(
            "Warning: --stream-ui requested but no DISPLAY is available. "
            "Continuing in no-window streaming mode."
        )
        stream_ui_requested = False

    _prepare_simulation_app_argv(
        unknown,
        stream_enabled=args.stream,
        local_window=stream_ui_requested,
    )

    config = CONFIG.copy()
    run_headless = bool(args.headless)
    if args.stream and run_headless:
        print("Warning: --headless disables WebRTC UI mode switching.")
    if run_headless and stream_ui_requested:
        print("Warning: --headless overrides --stream-ui.")
    if run_headless:
        config["headless"] = True

    _configure_layout_defaults(DEFAULT_LAYOUT_PATH)
    experience_path = _resolve_simulation_experience(args.stream)
    if experience_path:
        print(f"Using Isaac native experience defaults from: {experience_path}")
    simulation_app = _create_simulation_app(config, experience_path)
    try:
        if args.stream:
            _enable_webrtc_extension()
            print("WebRTC livestream enabled. Connect at http://localhost:8211")

        render_enabled = bool(args.stream) or not bool(config.get("headless", False))
        bridge_session = initialize_bridge_session(
            args,
            updater=simulation_app,
            render_enabled=render_enabled,
        )
        if bridge_session is None:
            return

        try:
            step_count = 0
            while simulation_app.is_running():
                bridge_session.tick()
                step_count += 1
                should_render = (
                    render_enabled
                    and (step_count % bridge_session.render_every_n_steps) == 0
                )
                bridge_session.world.step(render=should_render)
        finally:
            bridge_session.shutdown()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
