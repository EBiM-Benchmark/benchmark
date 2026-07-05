#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Build, start, and validate the workshop Docker runtimes."""

from __future__ import annotations

import argparse
import os
import re
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
COMPOSE_FILE = REPO_ROOT / "docker" / "docker-compose.yaml"
ENV_FILE = REPO_ROOT / "docker" / ".env.base"
WORKSPACE_ROOT = "/workspace/EBiM_Challenge"


@dataclass(frozen=True)
class Runtime:
    profile: str
    service: str
    container_env: str
    mount_targets: tuple[str, ...]


RUNTIMES = (
    Runtime(
        profile="isaac-sim-5.1.0",
        service="isaac-sim-5-1-0",
        container_env="ISAAC_SIM_5_CONTAINER",
        mount_targets=(
            WORKSPACE_ROOT,
            "/isaac-sim/.cache",
            "/isaac-sim/kit/cache",
            "/isaac-sim/.cache/ov",
            "/isaac-sim/.cache/warp",
            "/isaac-sim/.nv/ComputeCache",
            "/isaac-sim/.nvidia-omniverse/logs",
            "/isaac-sim/kit/logs",
            "/isaac-sim/.nvidia-omniverse/config",
            "/isaac-sim/data/Kit",
            "/isaac-sim/kit/data/documents",
            "/isaac-sim/kit/data/Kit",
            "/isaac-sim/.local/share/ov/data/documents",
            "/isaac-sim/.local/share/ov/data/Kit",
            "/isaac-sim/.local/share/ov/pkg",
        ),
    ),
    Runtime(
        profile="isaac-sim-6.0.0",
        service="isaac-sim-6-0-0",
        container_env="ISAAC_SIM_6_CONTAINER",
        mount_targets=(
            WORKSPACE_ROOT,
            "/isaac-sim/.cache",
            "/isaac-sim/kit/cache",
            "/isaac-sim/.cache/ov",
            "/isaac-sim/.cache/warp",
            "/isaac-sim/.nv/ComputeCache",
            "/isaac-sim/.nvidia-omniverse/logs",
            "/isaac-sim/kit/logs",
            "/isaac-sim/.nvidia-omniverse/config",
            "/isaac-sim/data/Kit",
            "/isaac-sim/kit/data/documents",
            "/isaac-sim/kit/data/Kit",
            "/isaac-sim/.local/share/ov/data/documents",
            "/isaac-sim/.local/share/ov/data/Kit",
            "/isaac-sim/.local/share/ov/pkg",
        ),
    ),
    Runtime(
        profile="isaac-lab-2.3.2",
        service="isaac-lab-2-3-2",
        container_env="ISAAC_LAB_CONTAINER",
        mount_targets=(
            WORKSPACE_ROOT,
            "/isaac-sim/kit/cache",
            "/root/.cache/ov",
            "/root/.nvidia-omniverse/logs",
        ),
    ),
)

HOST_RUNTIME_DIRS = (
    "isaac-sim-5.1.0/cache/main/ov",
    "isaac-sim-5.1.0/cache/main/warp",
    "isaac-sim-5.1.0/cache/computecache",
    "isaac-sim-5.1.0/logs",
    "isaac-sim-5.1.0/config",
    "isaac-sim-5.1.0/data/documents",
    "isaac-sim-5.1.0/data/Kit",
    "isaac-sim-5.1.0/pkg",
    "isaac-sim-6.0.0/cache/main/ov",
    "isaac-sim-6.0.0/cache/main/warp",
    "isaac-sim-6.0.0/cache/computecache",
    "isaac-sim-6.0.0/logs",
    "isaac-sim-6.0.0/config",
    "isaac-sim-6.0.0/data/documents",
    "isaac-sim-6.0.0/data/Kit",
    "isaac-sim-6.0.0/pkg",
    "isaac-lab-2.3.2/cache/kit",
    "isaac-lab-2.3.2/cache/ov",
    "isaac-lab-2.3.2/cache/pip",
    "isaac-lab-2.3.2/cache/glcache",
    "isaac-lab-2.3.2/cache/computecache",
    "isaac-lab-2.3.2/logs",
    "isaac-lab-2.3.2/data",
    "isaac-lab-2.3.2/documents",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Prepare, build, start, and validate the three EBiM workshop "
            "Docker runtimes."
        )
    )
    parser.add_argument(
        "--prepare-dirs",
        action="store_true",
        help="Create host-side cache/data directories before starting.",
    )
    parser.add_argument(
        "--build",
        action="store_true",
        help="Build the three local runtime images in parallel.",
    )
    parser.add_argument(
        "--up",
        action="store_true",
        help="Start the three containers before validating them.",
    )
    parser.add_argument(
        "--down",
        action="store_true",
        help="Stop the containers after validation.",
    )
    parser.add_argument(
        "--skip-script-check",
        action="store_true",
        help="Skip Python/USD script checks inside the containers.",
    )
    parser.add_argument(
        "--external-network-check",
        action="store_true",
        help="Also test DNS and outbound HTTPS from inside each container.",
    )
    return parser.parse_args()


def expand_env_value(value: str, env: dict[str, str]) -> str:
    pattern = re.compile(r"\$\{([^}:]+)(?::-([^}]*))?\}")

    def replace(match: re.Match[str]) -> str:
        name = match.group(1)
        default = match.group(2)
        return env.get(name, default if default is not None else "")

    expanded = pattern.sub(replace, value)
    previous_environ = os.environ.copy()
    try:
        os.environ.update(env)
        return os.path.expandvars(expanded)
    finally:
        os.environ.clear()
        os.environ.update(previous_environ)


def read_env_file() -> dict[str, str]:
    env = dict(os.environ)
    for raw_line in ENV_FILE.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        value = value.strip().strip('"').strip("'")
        env[key.strip()] = expand_env_value(value, env)
    return env


def compose_prefix(profiles: tuple[str, ...] = ()) -> list[str]:
    command = [
        "docker",
        "compose",
        "--env-file",
        str(ENV_FILE.relative_to(REPO_ROOT)),
        "-f",
        str(COMPOSE_FILE.relative_to(REPO_ROOT)),
    ]
    for profile in profiles:
        command.extend(["--profile", profile])
    return command


def run(
    command: list[str],
    *,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    print(f"+ {shlex.join(command)}")
    return subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=check,
        text=True,
    )


def prepare_host_dirs(env: dict[str, str]) -> None:
    docker_root = Path(env["ISAAC_DOCKER_ROOT"]).expanduser()
    for relative_path in HOST_RUNTIME_DIRS:
        (docker_root / relative_path).mkdir(parents=True, exist_ok=True)

    xauthority = env.get("XAUTHORITY")
    if xauthority:
        Path(xauthority).expanduser().parent.mkdir(parents=True, exist_ok=True)
        Path(xauthority).expanduser().touch(exist_ok=True)

    print(f"Prepared host Docker storage under {docker_root}")
    print(
        "Isaac Sim containers run as HOST_UID/HOST_GID. "
        "If write checks fail, run:\n"
        '  sudo chown -R "${HOST_UID:-$(id -u)}:${HOST_GID:-$(id -g)}" '
        f"{docker_root / 'isaac-sim-5.1.0'} "
        f"{docker_root / 'isaac-sim-6.0.0'}"
    )


def build_images_in_parallel() -> None:
    processes: list[tuple[Runtime, subprocess.Popen[str]]] = []
    for runtime in RUNTIMES:
        command = compose_prefix((runtime.profile,)) + [
            "build",
            runtime.service,
        ]
        print(f"+ {shlex.join(command)}")
        processes.append(
            (
                runtime,
                subprocess.Popen(command, cwd=REPO_ROOT, text=True),
            )
        )

    failed = []
    for runtime, process in processes:
        return_code = process.wait()
        if return_code != 0:
            failed.append((runtime.service, return_code))

    if failed:
        details = ", ".join(f"{service}={code}" for service, code in failed)
        raise RuntimeError(f"Runtime image build failed: {details}")


def start_containers() -> None:
    profiles = tuple(runtime.profile for runtime in RUNTIMES)
    services = [runtime.service for runtime in RUNTIMES]
    run(compose_prefix(profiles) + ["up", "-d", *services])


def stop_containers() -> None:
    run(compose_prefix() + ["down"])


def docker_exec(container: str, shell_command: str) -> None:
    run(["docker", "exec", container, "bash", "-lc", shell_command])


def docker_output(command: list[str]) -> str:
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def python_runner_snippet() -> str:
    return (
        "USD_LIB=$(find /workspace/isaaclab/_isaac_sim/extscache "
        "/isaac-sim/extscache -maxdepth 1 -type d "
        "-name 'omni.usd.libs-*' 2>/dev/null | sort | tail -n 1); "
        'if [ -n "$USD_LIB" ]; then '
        'export PYTHONPATH="$USD_LIB${PYTHONPATH:+:$PYTHONPATH}"; '
        'export LD_LIBRARY_PATH="$USD_LIB/bin'
        '${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; '
        "fi; "
        "if command -v python >/dev/null 2>&1; then "
        "PY='python'; "
        "elif command -v ebim-python >/dev/null 2>&1; then "
        "PY='ebim-python'; "
        "elif [ -x /isaac-sim/python.sh ]; then "
        "PY='/isaac-sim/python.sh'; "
        "elif [ -x /isaac-lab/isaaclab.sh ]; then "
        "PY='/isaac-lab/isaaclab.sh -p'; "
        "elif [ -x /workspace/isaaclab/isaaclab.sh ]; then "
        "PY='/workspace/isaaclab/isaaclab.sh -p'; "
        "elif command -v python3 >/dev/null 2>&1; then "
        "PY='python3'; "
        "else "
        "PY='python'; "
        "fi"
    )


def validate_mounts(container: str, runtime: Runtime) -> None:
    mount_checks = " && ".join(
        f"test -d {shlex.quote(target)}" for target in runtime.mount_targets
    )
    write_targets = [
        target for target in runtime.mount_targets if target != WORKSPACE_ROOT
    ]
    write_checks = " && ".join(
        (
            f"touch {shlex.quote(target)}/.ebim_write_test && "
            f"rm {shlex.quote(target)}/.ebim_write_test"
        )
        for target in write_targets
    )
    docker_exec(
        container,
        (
            f"{mount_checks} && "
            f"{write_checks} && "
            f"test -f {WORKSPACE_ROOT}/README.md && "
            f"test -f {WORKSPACE_ROOT}/scripts/scenes/"
            "scene_robot_room_keyboard.py && "
            f"test -f {WORKSPACE_ROOT}/assets/robot_room.usd"
        ),
    )


def validate_x11(container: str) -> None:
    docker_exec(
        container,
        (
            'test -n "$DISPLAY" && '
            "test -d /tmp/.X11-unix && "
            'if [ -n "$XAUTHORITY" ]; then test -e "$XAUTHORITY"; fi'
        ),
    )


def validate_network(container: str, external: bool) -> None:
    network_mode = docker_output(
        [
            "docker",
            "inspect",
            "-f",
            "{{.HostConfig.NetworkMode}}",
            container,
        ]
    )
    if network_mode != "host":
        raise RuntimeError(
            f"{container} uses network mode {network_mode!r}, expected 'host'"
        )

    docker_exec(container, "getent hosts localhost >/dev/null")
    if external:
        code = (
            "import socket; "
            "socket.create_connection(('nvcr.io', 443), timeout=10).close(); "
            "print('external https ok')"
        )
        docker_exec(
            container,
            f"{python_runner_snippet()} && $PY -c {shlex.quote(code)}",
        )


def validate_scripts(container: str) -> None:
    syntax_check = (
        "from pathlib import Path; "
        "files = ["
        "'scripts/common/path_utils.py', "
        "'scripts/common/tmr_base_control.py', "
        "'scripts/tools/inspect_usd.py', "
        "'scripts/scenes/scene_robot_room_keyboard.py', "
        "]; "
        "[compile(Path(file).read_text(encoding='utf-8'), file, 'exec') "
        "for file in files]; "
        "print('syntax ok')"
    )
    path_check = (
        "from pathlib import Path; "
        "import sys; "
        "sys.path.insert(0, 'scripts/common'); "
        "from path_utils import asset_path, franka_urdf_path; "
        "assert asset_path('robot_room.usd').is_file(); "
        "assert franka_urdf_path("
        "'mobile_fr3_duo_v0_2_franka_hand.usd'"
        ").is_file(); "
        "print('repository paths ok')"
    )
    runtime_check = (
        "import os, sys; "
        "from pathlib import Path; "
        "print(sys.executable); "
        "assert os.environ.get('ISAAC_PATH') or "
        "Path('/workspace/isaaclab/_isaac_sim').exists()"
    )
    docker_exec(
        container,
        (
            f"cd {WORKSPACE_ROOT} && "
            "export PYTHONDONTWRITEBYTECODE=1 && "
            f"{python_runner_snippet()} && "
            'test "$PY" = python && '
            f"$PY -c {shlex.quote(runtime_check)} && "
            f"$PY -c {shlex.quote(syntax_check)} && "
            f"$PY -c {shlex.quote(path_check)} && "
            "$PY scripts/tools/inspect_usd.py "
            "assets/robot_room.usd >/tmp/ebim_robot_room.txt && "
            "test -s /tmp/ebim_robot_room.txt"
        ),
    )


def validate_containers(env: dict[str, str], args: argparse.Namespace) -> None:
    for runtime in RUNTIMES:
        container = env[runtime.container_env]
        running = docker_output(
            [
                "docker",
                "inspect",
                "-f",
                "{{.State.Running}}",
                container,
            ]
        )
        if running != "true":
            raise RuntimeError(f"{container} is not running")

        print(f"Validating {container}")
        validate_mounts(container, runtime)
        validate_x11(container)
        validate_network(container, args.external_network_check)
        if not args.skip_script_check:
            validate_scripts(container)


def main() -> int:
    args = parse_args()
    env = read_env_file()

    try:
        run(compose_prefix() + ["config", "--profiles"])
        if args.prepare_dirs:
            prepare_host_dirs(env)
        if args.build:
            build_images_in_parallel()
        if args.up:
            start_containers()
        validate_containers(env, args)
    finally:
        if args.down:
            stop_containers()

    print("All requested Docker runtime checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
