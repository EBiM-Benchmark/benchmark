"""USD asset discovery and selection helpers."""

import os
import sys


def _finalize_usd_path(usd_path):
    """Use the caller-selected USD exactly as requested."""
    if not usd_path:
        return usd_path
    return os.path.abspath(os.path.expanduser(usd_path))


def _discover_usd_assets(assets_dir):
    if not os.path.isdir(assets_dir):
        return []
    assets = []
    for name in sorted(os.listdir(assets_dir)):
        if not name.endswith(".usd"):
            continue
        full_path = os.path.join(assets_dir, name)
        if os.path.isfile(full_path):
            assets.append(full_path)
    return assets


def _read_last_robot_hint(assets_dir):
    hint_path = os.path.join(assets_dir, ".last_robot_name")
    if not os.path.exists(hint_path):
        return None
    try:
        with open(hint_path, "r", encoding="utf-8") as file:
            raw = file.read().strip()
            if not raw:
                return None
            if raw.endswith(".usd"):
                return raw
            return f"{raw}.usd"
    except Exception:
        return None


def _print_assets(assets):
    print("\nAvailable USD assets:")
    for index, path in enumerate(assets):
        print(f"  [{index}] {os.path.basename(path)}")


def _resolve_usd_path(args):
    assets = _discover_usd_assets(args.assets_dir)

    if args.list_assets:
        if not assets:
            print(f"No USD assets found in: {args.assets_dir}")
        else:
            _print_assets(assets)
        return None, True

    if args.usd_path:
        resolved = _finalize_usd_path(args.usd_path)
        if not os.path.isfile(resolved):
            print(f"Warning: specified --usd-path does not exist: {resolved}")
        return resolved, False

    if args.asset_name:
        for path in assets:
            base = os.path.basename(path)
            if base == args.asset_name or base == f"{args.asset_name}.usd":
                return _finalize_usd_path(path), False
        print(f"Error: asset name '{args.asset_name}' not found in {args.assets_dir}")
        return None, True

    if args.asset_index is not None:
        if args.asset_index < 0 or args.asset_index >= len(assets):
            print(
                f"Error: asset index {args.asset_index} out of range [0, {max(len(assets) - 1, 0)}]"
            )
            return None, True
        return _finalize_usd_path(assets[args.asset_index]), False

    if not assets:
        print(f"Error: No USD assets found in: {args.assets_dir}")
        return None, True

    hint_name = _read_last_robot_hint(args.assets_dir)
    if hint_name is not None:
        for path in assets:
            if os.path.basename(path) == hint_name:
                return _finalize_usd_path(path), False

    interactive = args.select_asset or sys.stdin.isatty()
    if not interactive:
        print(
            "Error: No USD selected in non-interactive mode. "
            "Use --usd-path, --asset-name, or --asset-index."
        )
        return None, True

    default_index = 0
    if hint_name is not None:
        for i, path in enumerate(assets):
            if os.path.basename(path) == hint_name:
                default_index = i
                break

    _print_assets(assets)
    print(f"\nPress Enter to use default [{default_index}] {os.path.basename(assets[default_index])}")

    while True:
        user_input = input("Select asset index: ").strip()
        if user_input == "":
            # Keep subsequent stdout on a fresh line so shell callers can parse it.
            print()
            return _finalize_usd_path(assets[default_index]), False
        try:
            index = int(user_input)
            if 0 <= index < len(assets):
                # Keep subsequent stdout on a fresh line so shell callers can parse it.
                print()
                return _finalize_usd_path(assets[index]), False
            print(f"Invalid index. Choose between 0 and {len(assets) - 1}.")
        except ValueError:
            print("Invalid input. Please enter a number.")
