"""Command-line arguments, one argparse group per module.

Each input method builds its own parser from shared groups, so
``main.py --input vr --help`` shows exactly the flags that apply to VR.
Defaults that differ between desktop and VR mode (timestep, base speed) are
passed in by the caller instead of being duplicated.
"""

from __future__ import annotations

import argparse
import math

from . import config


def add_viewer_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("viewer / debug")
    g.add_argument("--no-viewer", action="store_true", help="load model and run a short smoke test")
    g.add_argument("--render-hz", type=float, default=config.RENDER_HZ)
    g.add_argument("--profile", action="store_true", help="print loop/control/physics/render timing once per second")
    g.add_argument(
        "--profile-contacts", action="store_true", help="include top contact-pair families in --profile output"
    )


def add_physics_args(parser: argparse.ArgumentParser, *, timestep_default: float) -> None:
    g = parser.add_argument_group("physics (scene.py)")
    g.add_argument(
        "--timestep",
        type=float,
        default=timestep_default,
        help=f"physics timestep in seconds (default {timestep_default}; the XML's own value is 0.0005 - "
        "larger is faster but verify grasp feel)",
    )
    g.add_argument(
        "--noslip-iterations",
        type=int,
        default=None,
        help="override XML noslip_iterations (XML=20; 0 skips the noslip pass, "
        "~15%% faster physics, cable may slip slightly more)",
    )
    g.add_argument(
        "--start-at-board",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="spawn with the right gripper hovering over the cable (descend and close to grasp); "
        "--no-start-at-board keeps the original far-away spawn",
    )


def add_grasp_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("grasping (grasping.py)")
    g.add_argument(
        "--grasp-assist",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="capped-force spring keeping a grasped cable segment in the pad slot",
    )


def add_base_args(
    parser: argparse.ArgumentParser,
    *,
    base_speed_default: float,
    base_yaw_default: float,
    with_control_modes: bool,
) -> None:
    g = parser.add_argument_group("mobile base (base_drive.py)")
    if with_control_modes:
        g.add_argument(
            "--base-control",
            choices=("jointvel", "actuator", "kinematic", "wheel"),
            default="actuator",
            help="actuator (default) servos the base planar-joint velocity actuators - smooth and obstacle-safe; "
            "jointvel injects joint velocities directly (harsher on collisions); wheel drives wheel joints",
        )
        g.add_argument("--wheel-speed", type=float, default=75.0, help="wheel roll velocity scale")
        g.add_argument("--wheel-yaw-speed", type=float, default=45.0, help="wheel turn velocity scale")
    g.add_argument("--base-speed", type=float, default=base_speed_default, help="base translation speed in m/s")
    g.add_argument("--base-yaw-speed-deg", type=float, default=base_yaw_default, help="base yaw speed in deg/s")
    g.add_argument(
        "--robot-forward-axis",
        choices=("x", "-x", "y", "-y"),
        default="x",
        help="which base-body axis is 'forward' for local driving",
    )


def add_arm_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("arm control (robot_arm.py)")
    g.add_argument("--move-speed", type=float, default=config.MOVE_SPEED, help="TCP translation speed in m/s")
    g.add_argument(
        "--rot-speed-deg", type=float, default=math.degrees(config.ROT_SPEED), help="TCP rotation speed in deg/s"
    )
    g.add_argument(
        "--joint-vel-limit", type=float, default=config.JOINT_VEL_LIMIT, help="IK joint velocity clamp in rad/s"
    )
    g.add_argument("--ori-lock-gain", type=float, default=config.ORIENTATION_LOCK_GAIN)
    g.add_argument(
        "--arm-frame",
        choices=("base", "camera"),
        default="camera",
        help="frame for arm stick/arrow translation: camera = operator screen axes "
        "(matches the base and VR, default), base = robot heading (stable under "
        "camera orbits)",
    )


def add_gamepad_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("gamepad (input_gamepad.py)")
    g.add_argument("--gamepad", action="store_true", help="enable gamepad polling (implied by --input gamepad)")


def add_vr_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("vr (run_vr.py / vr_openxr.py)")
    g.add_argument(
        "--vr-backend",
        choices=("openxr", "steamvr"),
        default="openxr",
        help="openxr (default, no Steam needed) or steamvr (openvr background input only)",
    )
    g.add_argument("--vr-scale", type=float, default=1.4, help="controller-to-TCP motion scale")
    g.add_argument(
        "--facing",
        choices=("front", "behind"),
        default="front",
        help="front (default): operator faces the robot on screen - hands are mirrored to the"
        " opposite arm and motion follows the screen axes; behind: same-side hands, robot-frame motion",
    )
    g.add_argument(
        "--hmd-view",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="render the sim onto a stereo screen inside the headset (costs GPU and can"
        " flicker over Link). Default off: headset stays black as a pure input device and"
        " you watch the desktop viewer at full rate",
    )


def add_mnet_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("ManipulationNet eval (mnet_bridge.py)")
    g.add_argument(
        "--mnet",
        action="store_true",
        help="publish a sim camera over ROS 2 and report task finished/skipped "
        "(F/H keys) to the mnet client; needs rclpy",
    )
    g.add_argument(
        "--mnet-config",
        type=str,
        default=None,
        help="path to the mnet team_config.json (default: sibling "
        "mnet_client-ros_2/config/team_config.json); camera topics are read from it",
    )
    g.add_argument(
        "--mnet-camera-topic",
        type=str,
        default=None,
        help="override the sensor_msgs/Image topic to publish the sim camera on",
    )
    g.add_argument("--mnet-camera-info-topic", type=str, default=None, help="override the sensor_msgs/CameraInfo topic")
    g.add_argument("--mnet-width", type=int, default=640, help="published camera width")
    g.add_argument("--mnet-height", type=int, default=480, help="published camera height")
    g.add_argument("--mnet-fps", type=float, default=30.0, help="publish rate; the mnet client requires >=25")
    g.add_argument(
        "--mnet-tier",
        type=str,
        default="Tier2",
        help="the tier this scene plays; every other announced tier is auto-reported "
        "as SKIPPED so no manual H presses are needed (pass '' to disable)",
    )
    g.add_argument(
        "--mnet-camera",
        type=str,
        default="mnet_overhead",
        help="evidence camera: a fixed model camera name (default: the ceiling-mounted "
        "overhead camera above the board), or 'viewer' to follow the desktop view",
    )
    g.add_argument(
        "--mnet-randomize",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="apply the client's randomized fixture coordinates (test_coordinates from "
        "/mnet_client/board_configuration) to the sim board when the tier starts",
    )
    g.add_argument(
        "--randomize-board",
        action="store_true",
        help="randomize the Tier2 fixtures at startup with the same distribution the "
        "mnet client uses (offline test, no ROS/client needed)",
    )
    g.add_argument(
        "--randomize-seed", type=int, default=None, help="seed for --randomize-board (default: different every run)"
    )
    g.add_argument(
        "--display-code",
        type=str,
        default=None,
        help="show this text (A-Z 0-9, max 8 chars) on the board's code plate at "
        "startup - the one-time submission code must be visible in the camera view; "
        "at runtime type 'code <TEXT>' into the sim terminal instead",
    )


def add_gello_args(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("gello (input_gello.py / run_gello.py)")
    g.add_argument(
        "--gello-joint-kp",
        type=float,
        default=config.GELLO_JOINT_KP,
        help="P-gain driving the arm velocity actuators toward GELLO's "
        "joint-space target (GELLO gives absolute joint angles directly, no IK)",
    )


def build_desktop_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Duo-FR3 full-scene teleop - keyboard/gamepad mode",
    )
    parser.add_argument(
        "--input",
        choices=("keyboard", "gamepad", "vr", "gello", "ros_teleop"),
        default="keyboard",
        help="input method (dispatched by main.py)",
    )
    add_viewer_args(parser)
    add_physics_args(parser, timestep_default=0.001)
    add_grasp_args(parser)
    add_base_args(parser, base_speed_default=3.0, base_yaw_default=360.0, with_control_modes=True)
    add_arm_args(parser)
    add_gamepad_args(parser)
    add_mnet_args(parser)
    return parser


def build_vr_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Duo-FR3 full-scene teleop - VR mode",
    )
    parser.add_argument(
        "--input", choices=("keyboard", "gamepad", "vr"), default="vr", help="input method (dispatched by main.py)"
    )
    add_viewer_args(parser)
    # VR default timestep is 2x the desktop one: the VR loop shares its time
    # budget with headset frame submission, and grasp feel was verified at 2ms
    add_physics_args(parser, timestep_default=0.002)
    add_grasp_args(parser)
    add_base_args(parser, base_speed_default=1.2, base_yaw_default=120.0, with_control_modes=False)
    add_vr_args(parser)
    add_mnet_args(parser)
    return parser


def build_gello_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Duo-FR3 full-scene teleop - GELLO mode (official EBiM ROS 2 input device)",
    )
    parser.add_argument(
        "--input",
        choices=("keyboard", "gamepad", "vr", "gello", "ros_teleop"),
        default="gello",
        help="input method (dispatched by main.py)",
    )
    add_viewer_args(parser)
    add_physics_args(parser, timestep_default=0.001)
    add_grasp_args(parser)
    add_base_args(parser, base_speed_default=3.0, base_yaw_default=360.0, with_control_modes=True)
    add_gello_args(parser)
    add_mnet_args(parser)
    return parser


def build_ros_teleop_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Duo-FR3 full-scene teleop - unified ROS 2 teleop mode "
        "(consumes a standalone keyboard/gamepad/VR Teleop Node over ROS 2)",
    )
    parser.add_argument(
        "--input",
        choices=("keyboard", "gamepad", "vr", "gello", "ros_teleop"),
        default="ros_teleop",
        help="input method (dispatched by main.py)",
    )
    add_viewer_args(parser)
    add_physics_args(parser, timestep_default=0.001)
    add_grasp_args(parser)
    add_base_args(parser, base_speed_default=3.0, base_yaw_default=360.0, with_control_modes=True)
    add_mnet_args(parser)
    return parser
