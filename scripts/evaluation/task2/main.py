#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Entrypoint for the task2 eval camera service.

Loads runtime config (APP_DEFAULTS < config.yaml < CLI), then starts the ROS2
node. See README.md for usage and the docker workflow.
"""

from config import load_runtime_config
from node import run


def main(args=None) -> None:
    config = load_runtime_config(args=args)
    run(config, args=args)


if __name__ == "__main__":
    main()
