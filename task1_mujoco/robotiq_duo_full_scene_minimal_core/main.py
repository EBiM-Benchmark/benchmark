"""Duo-FR3 full-scene teleop — single entry point for every input method.

    python main.py                       # keyboard (default)
    python main.py --input gamepad       # gamepad
    python main.py --input vr            # VR (OpenXR; --vr-backend steamvr)
    python main.py --input gello         # GELLO (official EBiM ROS 2 device)

All code lives in the ``teleop/`` package (see teleop/__init__.py for the
module map). Input methods are imported lazily and independently: a broken
VR module can never take down keyboard/gamepad mode.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# make ``import teleop`` work no matter where this is launched from
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Linux/Wayland: GLFW 3.4 would open a native Wayland window, which the X
# server never sees — held-key polling (X11 query_keymap) and OpenXR both
# need the window on X11/XWayland. Applied before any glfw.init(); export
# GLFW_PLATFORM=wayland to override deliberately.
if sys.platform.startswith("linux") and os.environ.get("XDG_SESSION_TYPE") == "wayland":
    if os.environ.setdefault("GLFW_PLATFORM", "x11") == "x11":
        print(
            "[main] Wayland session: forcing the viewer onto XWayland (GLFW_PLATFORM=x11) so held-key polling works",
            flush=True,
        )


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)

    # pre-parse only --input; the mode-specific parser handles the rest
    # (including --help, so help output matches the chosen mode)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument(
        "--input",
        choices=("keyboard", "gamepad", "vr", "gello", "ros_teleop"),
        default="keyboard",
    )
    ns, rest = pre.parse_known_args(argv)

    from teleop import cli

    if ns.input == "vr":
        try:
            from teleop import run_vr
        except Exception as exc:  # VR deps or module broken — other modes unaffected
            print(f"[main] VR mode unavailable: {exc!r}", flush=True)
            print(
                "[main] keyboard/gamepad still work: python main.py --input gamepad",
                flush=True,
            )
            raise SystemExit(1)
        args = cli.build_vr_parser().parse_args(rest)
        args.input = "vr"
        run_vr.main(args)
        return

    if ns.input == "gello":
        try:
            from teleop import run_gello
        except Exception as exc:  # rclpy or module broken — other modes unaffected
            print(f"[main] GELLO mode unavailable: {exc!r}", flush=True)
            print(
                "[main] keyboard/gamepad/vr still work: python main.py --input keyboard",
                flush=True,
            )
            raise SystemExit(1)
        args = cli.build_gello_parser().parse_args(rest)
        args.input = "gello"
        run_gello.main(args)
        return

    if ns.input == "ros_teleop":
        try:
            from teleop import run_ros_teleop
        except Exception as exc:  # rclpy or module broken — other modes unaffected
            print(f"[main] ros_teleop mode unavailable: {exc!r}", flush=True)
            print(
                "[main] keyboard/gamepad/vr still work: python main.py --input keyboard",
                flush=True,
            )
            raise SystemExit(1)
        args = cli.build_ros_teleop_parser().parse_args(rest)
        args.input = "ros_teleop"
        run_ros_teleop.main(args)
        return

    from teleop import run_desktop

    args = cli.build_desktop_parser().parse_args(rest)
    args.input = ns.input
    if ns.input == "gamepad":
        args.gamepad = True
    run_desktop.main(args)


if __name__ == "__main__":
    main()
