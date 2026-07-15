#!/usr/bin/env python3
"""Shared subprocess and Isaac bridge orchestration helpers."""

from __future__ import annotations

import os
import shlex
import subprocess
import time

from stack_config import REPO_ROOT


def host_path_from_container(path: str) -> str:
    if path.startswith("/workspace/"):
        return os.path.join(REPO_ROOT, path[len("/workspace/") :])
    return path


def container_path_from_host(path: str) -> str:
    resolved = os.path.abspath(os.path.expanduser(path))
    if resolved.startswith(REPO_ROOT + os.sep):
        return "/workspace/" + os.path.relpath(resolved, REPO_ROOT)
    return resolved


def run_command(args, *, check: bool = True, capture_output: bool = True, cwd=REPO_ROOT):
    completed = subprocess.run(
        args,
        cwd=cwd,
        text=True,
        capture_output=capture_output,
    )
    if completed.stdout:
        print(completed.stdout.rstrip())
    if completed.stderr:
        print(completed.stderr.rstrip())
    if check and completed.returncode != 0:
        raise RuntimeError(
            f"Command failed ({completed.returncode}): {' '.join(shlex.quote(part) for part in args)}"
        )
    return completed


def compose_exec(service: str, command: str) -> list[str]:
    return ["docker", "compose", "exec", "-T", service, "bash", "-lc", command]


def detect_running_kit(isaac_service: str) -> list[str]:
    completed = run_command(
        [
            "docker",
            "exec",
            isaac_service,
            "bash",
            "-lc",
            "ps -eo args | grep -F '/isaac-sim/kit/kit' | grep -v grep | head -n 1",
        ]
    )
    command_line = completed.stdout.strip()
    if not command_line:
        raise RuntimeError("No running Isaac kit process found in the isaac service")
    return shlex.split(command_line)


def value_after(tokens: list[str], flag: str, default: str = "") -> str:
    for index, token in enumerate(tokens):
        if token == flag and (index + 1) < len(tokens):
            return tokens[index + 1]
        if token.startswith(flag + "="):
            return token.split("=", 1)[1]
    return default


def launcher_from_app_path(app_path: str) -> str:
    if app_path.endswith(".streaming.kit"):
        return "/isaac-sim/isaac-sim.streaming.sh"
    return "/isaac-sim/isaac-sim.sh"


def kill_kit(isaac_service: str) -> None:
    run_command(
        ["docker", "exec", isaac_service, "bash", "-lc", "pkill -f '/isaac-sim/kit/kit' || true"],
        check=False,
    )


def wait_for_joint_states(
    recorder_service: str,
    ready_timeout: float,
    *,
    topic_name: str = "/isaac/left_joint_states",
) -> None:
    start_time = time.time()
    while True:
        try:
            completed = subprocess.run(
                compose_exec(
                    recorder_service,
                    f"source /opt/ros/jazzy/setup.bash && ros2 topic echo {topic_name} --once >/dev/null",
                ),
                cwd=REPO_ROOT,
                text=True,
                capture_output=True,
                timeout=3.0,
            )
        except subprocess.TimeoutExpired:
            completed = None

        if completed is not None and completed.returncode == 0:
            print(f"Isaac joint states ready after {time.time() - start_time:.1f} s")
            return
        if time.time() - start_time > ready_timeout:
            if completed is not None and completed.stderr:
                print(completed.stderr.rstrip())
            raise RuntimeError(f"Timed out waiting for {topic_name} after restart")
        time.sleep(2.0)


def build_native_bridge_exec_args(bridge_args: dict) -> list[str]:
    exec_args = [
        "isaac_joint_bridge_native.py",
        "--usd-path",
        str(bridge_args["usd_path"]),
        "--ros-publish-rate",
        str(bridge_args["ros_publish_rate"]),
        "--physics-hz",
        str(bridge_args["physics_hz"]),
        "--render-hz",
        str(bridge_args["render_hz"]),
        "--physics-substeps",
        str(bridge_args["physics_substeps"]),
        "--browser-command-hold-seconds",
        str(bridge_args["browser_command_hold_seconds"]),
        "--command-smoothing-alpha",
        str(bridge_args["command_smoothing_alpha"]),
        "--max-position-step-rad",
        str(bridge_args["max_position_step_rad"]),
        "--position-deadband-rad",
        str(bridge_args["position_deadband_rad"]),
        "--settle-position-window-rad",
        str(bridge_args["settle_position_window_rad"]),
        "--settle-velocity-threshold-rad-s",
        str(bridge_args["settle_velocity_threshold_rad_s"]),
    ]
    if bridge_args.get("camera_config"):
        exec_args.extend(["--camera-config", str(bridge_args["camera_config"])])
    if bridge_args.get("joint_drive_config"):
        exec_args.extend(["--joint-drive-config", str(bridge_args["joint_drive_config"])])
        exec_args.extend(
            [
                "--joint-drive-stiffness-scale",
                str(bridge_args.get("joint_drive_stiffness_scale", 1.0)),
                "--joint-drive-damping-scale",
                str(bridge_args.get("joint_drive_damping_scale", 1.0)),
                "--joint-drive-max-force-scale",
                str(bridge_args.get("joint_drive_max_force_scale", 1.0)),
                "--joint-drive-gripper-stiffness-scale",
                str(bridge_args.get("joint_drive_gripper_stiffness_scale", 1.0)),
                "--joint-drive-gripper-damping-scale",
                str(bridge_args.get("joint_drive_gripper_damping_scale", 1.0)),
                "--joint-drive-gripper-max-force-scale",
                str(bridge_args.get("joint_drive_gripper_max_force_scale", 1.0)),
            ]
        )
    if bridge_args.get("robot_prim_path"):
        exec_args.extend(["--robot-prim-path", str(bridge_args["robot_prim_path"])])
    if bridge_args.get("headless", False):
        exec_args.append("--headless")
    return exec_args


def launch_native_bridge(
    isaac_service: str,
    launcher_path: str,
    portable_root: str,
    bridge_args: dict,
) -> None:
    exec_args = build_native_bridge_exec_args(bridge_args)
    stage_path = str(bridge_args["usd_path"])
    run_command(
        [
            "docker",
            "exec",
            "-d",
            isaac_service,
            launcher_path,
            "--portable-root",
            portable_root,
            "--/app/content/emptyStageOnStart=false",
            f"--/app/content/stagePath={stage_path}",
            "--/app/python/scriptFolders/0=/workspace/scripts",
            "--exec",
            " ".join(exec_args),
        ]
    )


def restart_native_bridge(
    *,
    isaac_service: str,
    recorder_service: str,
    ready_timeout: float,
    restart_wait_seconds: float,
    launcher_path: str,
    portable_root: str,
    bridge_args: dict,
) -> None:
    kill_kit(isaac_service)
    time.sleep(max(restart_wait_seconds, 0.0))
    launch_native_bridge(
        isaac_service=isaac_service,
        launcher_path=launcher_path,
        portable_root=portable_root,
        bridge_args=bridge_args,
    )
    wait_for_joint_states(recorder_service, ready_timeout)
