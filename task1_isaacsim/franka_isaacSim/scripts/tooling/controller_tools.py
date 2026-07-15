#!/usr/bin/env python3
"""Helpers for controller-manager orchestration."""

from __future__ import annotations

import json
import shlex
import subprocess

from stack_config import REPO_ROOT
from tooling.runtime_tools import compose_exec


def switch_controller(
    *,
    controller_service: str,
    controller_manager: str,
    controller_name: str,
    activate: bool,
    check: bool = True,
):
    activate_controllers = [controller_name] if activate else []
    deactivate_controllers = [] if activate else [controller_name]
    request = json.dumps(
        {
            "activate_controllers": activate_controllers,
            "deactivate_controllers": deactivate_controllers,
            "strictness": 1,
            "activate_asap": True,
            "timeout": {"sec": 5, "nanosec": 0},
        }
    )
    command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /dependencies_ws/install/setup.bash && "
        "source /ros2_ws/install/setup.bash && "
        f"ros2 service call {controller_manager}/switch_controller "
        "controller_manager_msgs/srv/SwitchController "
        f"{shlex.quote(request)}"
    )
    completed = subprocess.run(
        compose_exec(controller_service, command),
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
    )
    if completed.stdout:
        print(completed.stdout.rstrip())
    if completed.stderr:
        print(completed.stderr.rstrip())
    if check and completed.returncode != 0:
        raise RuntimeError(
            f"Controller switch command failed ({completed.returncode}): {command}"
        )
    if check and not any(marker in completed.stdout for marker in ("ok: true", "ok=True", "ok: True")):
        raise RuntimeError(
            f"Controller switch request did not succeed for {controller_name}: {completed.stdout}"
        )
    return completed
