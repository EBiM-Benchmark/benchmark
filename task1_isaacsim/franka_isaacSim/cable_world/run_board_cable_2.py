# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import sys
from pathlib import Path

import run_board_cable


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "configs" / "board_cable_2.yaml"


def main() -> None:
    if not any(arg == "--config-path" or arg.startswith("--config-path=") for arg in sys.argv[1:]):
        sys.argv[1:1] = ["--config-path", str(DEFAULT_CONFIG_PATH)]
    run_board_cable.main()


if __name__ == "__main__":
    main()
