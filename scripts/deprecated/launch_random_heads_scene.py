#!/usr/bin/env python3
"""Launch a task scene with only two random head payloads enabled."""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path
from typing import Any

COMMON_DIR = Path(__file__).resolve().parents[1] / "common"
if str(COMMON_DIR) not in sys.path:
    sys.path.insert(0, str(COMMON_DIR))

from path_utils import asset_path

DEFAULT_SCENE_NAME = "tabletop_task_scene.usd"
DEFAULT_ENABLED_HEADS = 2
HEAD_ROOT_PATH = "/World/Scene"
HEAD_NAME_PREFIX = "Head_"
ISAACSIM_FULL_EXPERIENCE = "/isaac-sim/apps/isaacsim.exp.full.kit"
SCENE_ALIASES = {
    "default": DEFAULT_SCENE_NAME,
    "demo": "tabletop_task_scene_DEMO.usd",
    "tabletop": DEFAULT_SCENE_NAME,
    "tabletop_task_scene": DEFAULT_SCENE_NAME,
    "tabletop_task_scene.usd": DEFAULT_SCENE_NAME,
    "tabletop_task_scene_demo": "tabletop_task_scene_DEMO.usd",
    "tabletop_task_scene_demo.usd": "tabletop_task_scene_DEMO.usd",
}


def build_parser() -> argparse.ArgumentParser:
    bootstrap = argparse.ArgumentParser(add_help=False)
    bootstrap.add_argument(
        "--launcher",
        choices=("isaacsim", "isaaclab"),
        default="isaacsim",
    )
    bootstrap_args, _ = bootstrap.parse_known_args()

    parser = argparse.ArgumentParser(
        description=(
            "Open a bundled tabletop task scene and keep only two random "
            "head payloads loaded."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--scene",
        default=DEFAULT_SCENE_NAME,
        help=(
            "Bundled scene name, alias ('default' or 'demo'), or a USD path."
        ),
    )
    parser.add_argument(
        "--launcher",
        choices=("isaacsim", "isaaclab"),
        default=bootstrap_args.launcher,
        help="GUI runtime to launch.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed used to choose the two enabled heads.",
    )
    parser.add_argument(
        "--paused",
        action="store_true",
        help="Open the scene without starting the simulation timeline.",
    )

    if bootstrap_args.launcher == "isaaclab":
        try:
            from isaaclab.app import AppLauncher
        except ImportError as exc:
            raise SystemExit(
                "Could not import isaaclab. Run this inside an Isaac Lab "
                "runtime or use --launcher isaacsim."
            ) from exc

        AppLauncher.add_app_launcher_args(parser)

    return parser


def parse_args() -> argparse.Namespace:
    return build_parser().parse_args()


def resolve_scene_path(selection: str) -> Path:
    alias = SCENE_ALIASES.get(selection.lower())
    if alias is not None:
        candidate = asset_path(alias)
        if candidate.is_file():
            return candidate

    direct_candidate = Path(selection).expanduser()
    if direct_candidate.is_absolute() and direct_candidate.is_file():
        return direct_candidate

    bundled_candidate = asset_path(selection)
    if bundled_candidate.is_file():
        return bundled_candidate

    relative_candidate = direct_candidate.resolve()
    if relative_candidate.is_file():
        return relative_candidate

    raise FileNotFoundError(
        "Scene USD not found. Expected one of the bundled scenes "
        "'tabletop_task_scene.usd' or 'tabletop_task_scene_DEMO.usd', "
        f"or a valid USD path. Received: {selection}"
    )


def start_app(args: argparse.Namespace) -> Any:
    if args.launcher == "isaaclab":
        from isaaclab.app import AppLauncher

        app_launcher = AppLauncher(args)
        return app_launcher.app

    try:
        from isaacsim.simulation_app import SimulationApp
    except ImportError as exc:
        raise SystemExit(
            "Could not import isaacsim. Run this inside an Isaac Sim runtime "
            "or use --launcher isaaclab."
        ) from exc

    return SimulationApp(
        {"headless": False},
        experience=ISAACSIM_FULL_EXPERIENCE,
    )


def open_stage(app: Any, scene_path: Path) -> Any:
    import omni.usd

    context = omni.usd.get_context()
    result = context.open_stage(str(scene_path))

    if result is False:
        raise RuntimeError(f"Failed to open USD stage: {scene_path}")

    for _ in range(120):
        app.update()
        stage = context.get_stage()
        if stage is not None:
            return stage

    raise RuntimeError(f"Timed out while opening USD stage: {scene_path}")


def clear_selection() -> None:
    import omni.usd

    selection = omni.usd.get_context().get_selection()
    if selection is None:
        return

    clear_paths = getattr(selection, "clear_selected_prim_paths", None)
    if clear_paths is not None:
        clear_paths()

    # Isaac Sim 5.1 inside the container can leave an empty ALL-source
    # selection entry after payload load-state changes unless the USD
    # selection is explicitly normalized to an empty list.
    selection.set_selected_prim_paths([], False)


def head_payload_paths(stage: Any) -> list[Any]:
    from pxr import Sdf

    head_root = Sdf.Path(HEAD_ROOT_PATH)
    paths = []
    for prim in stage.Traverse():
        if prim.GetPath().GetParentPath() != head_root:
            continue
        if not prim.GetName().startswith(HEAD_NAME_PREFIX):
            continue
        paths.append(prim.GetPath())
    return sorted(paths, key=str)


def select_random_heads(
    head_paths: list[Any],
    seed: int | None,
) -> list[Any]:
    if len(head_paths) < DEFAULT_ENABLED_HEADS:
        raise RuntimeError(
            "Expected at least two head prims in the stage, but found "
            f"{len(head_paths)}. Rebuild the scene with --add-head first."
        )

    rng: random.Random | random.SystemRandom
    if seed is None:
        rng = random.SystemRandom()
    else:
        rng = random.Random(seed)

    return sorted(rng.sample(head_paths, DEFAULT_ENABLED_HEADS), key=str)


def set_head_payloads(app: Any, stage: Any, selected_paths: list[Any]) -> None:
    selected = {str(path) for path in selected_paths}
    all_heads = head_payload_paths(stage)
    unload_paths = [path for path in all_heads if str(path) not in selected]

    clear_selection()
    app.update()
    for path in unload_paths:
        stage.Unload(path)
        app.update()
    for path in selected_paths:
        stage.Load(path)
        app.update()
    clear_selection()
    app.update()


def start_timeline() -> None:
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()


def prim_name(path: Any) -> str:
    return str(path).rsplit("/", maxsplit=1)[-1]


def print_summary(
    scene_path: Path,
    launcher: str,
    seed: int | None,
    paused: bool,
    selected_paths: list[Any],
    all_head_paths: list[Any],
) -> None:
    print(f"Launcher: {launcher}")
    print(f"Scene: {scene_path}")
    print(f"Head payload prims found: {len(all_head_paths)}")
    print(f"Seed: {seed if seed is not None else 'system-random'}")
    print(f"Timeline: {'paused' if paused else 'playing'}")
    print(
        "Enabled head payloads: "
        + ", ".join(prim_name(path) for path in selected_paths)
    )
    print("Close the GUI window to exit.")


def run_until_exit(app: Any) -> None:
    while app.is_running():
        app.update()


def main() -> None:
    args = parse_args()
    scene_path = resolve_scene_path(args.scene)
    app = start_app(args)

    try:
        stage = open_stage(app, scene_path)
        clear_selection()
        all_head_paths = head_payload_paths(stage)
        selected_paths = select_random_heads(all_head_paths, args.seed)
        set_head_payloads(app, stage, selected_paths)
        if not args.paused:
            start_timeline()

        for _ in range(5):
            app.update()

        print_summary(
            scene_path=scene_path,
            launcher=args.launcher,
            seed=args.seed,
            paused=args.paused,
            selected_paths=selected_paths,
            all_head_paths=all_head_paths,
        )
        run_until_exit(app)
    finally:
        app.close()


if __name__ == "__main__":
    main()
