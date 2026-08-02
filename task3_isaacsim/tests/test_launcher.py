# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""CLI smoke tests for the Task 3 Docker launcher."""

import ast
import os
import subprocess
from pathlib import Path

TASK3_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = TASK3_ROOT.parent
LAUNCHER = TASK3_ROOT / "scripts" / "run_isaacsim_teleop.sh"
SCENE = TASK3_ROOT / "scripts" / "scene_room.py"


def test_launcher_help_documents_both_gripper_profiles():
    assert LAUNCHER.is_file(), "Task 3 launcher is missing"

    result = subprocess.run(
        ["bash", str(LAUNCHER), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "--gripper robotiq|panda" in result.stdout
    assert "default: robotiq" in result.stdout


def test_shared_room_builder_accepts_dynamic_beans():
    scene_path = (
        REPO_ROOT / "scripts" / "scenes" / "scene_robot_room_keyboard.py"
    )
    tree = ast.parse(scene_path.read_text())
    build_stage = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "build_stage"
    )

    assert "dynamic_beans" in [
        argument.arg for argument in build_stage.args.args
    ]


def test_scene_configures_profile_drives_before_world_reset():
    tree = ast.parse(SCENE.read_text())
    main = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "main"
    )
    calls = [
        node
        for node in ast.walk(main)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    ]
    configure_line = next(
        node.lineno
        for node in calls
        if (
            isinstance(node.func.value, ast.Name)
            and node.func.value.id == "core"
            and node.func.attr == "_configure_drives"
        )
    )
    reset_line = next(
        node.lineno
        for node in calls
        if (
            isinstance(node.func.value, ast.Name)
            and node.func.value.id == "world"
            and node.func.attr == "reset"
        )
    )

    assert configure_line < reset_line


def test_helper_up_removes_services_disabled_by_new_options(tmp_path):
    helper = TASK3_ROOT / "scripts" / "run_helper_containers.sh"
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$DOCKER_CALLS"\n')
    docker.chmod(0o755)
    calls = tmp_path / "docker-calls.txt"
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    environment["DOCKER_CALLS"] = str(calls)

    result = subprocess.run(
        [
            "bash",
            str(helper),
            "up",
            "--controller-mode",
            "none",
            "--no-browser",
            "--no-republisher",
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )

    assert result.returncode == 0, result.stderr
    assert calls.read_text().splitlines() == [
        "compose --profile * rm -sf ros_republisher "
        "position_controller teleop_adapters browser_controller"
    ]
