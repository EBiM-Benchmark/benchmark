#!/usr/bin/env python3
"""Resolve the startup USD selection and print it for shell callers."""

import argparse
import sys

from isaac_bridge_assets import _resolve_usd_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--assets-dir",
        type=str,
        required=True,
        help="Directory to scan for USD assets",
    )
    parser.add_argument(
        "--usd-path",
        type=str,
        default=None,
        help="Explicit USD path to use",
    )
    parser.add_argument(
        "--asset-name",
        type=str,
        default=None,
        help="Select USD by filename (with or without .usd)",
    )
    parser.add_argument(
        "--asset-index",
        type=int,
        default=None,
        help="Select USD by index",
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
    args = parser.parse_args()

    usd_path, should_exit = _resolve_usd_path(args)
    if should_exit:
        return 0 if args.list_assets else 1
    if not usd_path:
        return 1

    print(f"SELECTED_USD={usd_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
