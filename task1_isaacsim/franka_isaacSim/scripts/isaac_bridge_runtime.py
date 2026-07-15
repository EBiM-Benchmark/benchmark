"""Isaac runtime, stage, viewport, and startup utility helpers."""

import ctypes
import os
import sys
import time

try:
    from isaacsim.simulation_app import SimulationApp
except Exception:
    from omni.isaac.kit import SimulationApp

from isaac_bridge_constants import DEFAULT_PORTABLE_ROOT

def _configure_world_timing(world, physics_hz, render_hz, physics_substeps):
    physics_hz = max(float(physics_hz), 1.0)
    render_hz = max(float(render_hz), 1.0)
    physics_substeps = max(int(physics_substeps), 1)
    physics_dt = 1.0 / physics_hz
    render_dt = 1.0 / render_hz
    configured = False

    set_simulation_dt = getattr(world, "set_simulation_dt", None)
    if callable(set_simulation_dt):
        attempts = (
            {"physics_dt": physics_dt, "rendering_dt": render_dt, "substeps": physics_substeps},
            {"physics_dt": physics_dt, "rendering_dt": render_dt},
            {"physics_dt": physics_dt},
        )
        for kwargs in attempts:
            try:
                set_simulation_dt(**kwargs)
                configured = True
                break
            except TypeError:
                continue
            except Exception:
                continue
        if not configured:
            try:
                set_simulation_dt(physics_dt, render_dt)
                configured = True
            except Exception:
                pass

    physics_context_factory = getattr(world, "get_physics_context", None)
    if callable(physics_context_factory):
        try:
            physics_context = physics_context_factory()
        except Exception:
            physics_context = None
        if physics_context is not None:
            for method_name in ("set_physics_dt", "set_timestep"):
                method = getattr(physics_context, method_name, None)
                if not callable(method):
                    continue
                try:
                    method(physics_dt)
                    configured = True
                    break
                except Exception:
                    continue
            set_substeps = getattr(physics_context, "set_substeps", None)
            if callable(set_substeps):
                try:
                    set_substeps(physics_substeps)
                    configured = True
                except Exception:
                    pass

    render_every_n_steps = max(int(round(physics_hz / render_hz)), 1)
    status = "ok" if configured else "fallback"
    print(
        "Simulation timing config "
        f"({status}): physics={physics_hz:.1f} Hz, render={render_hz:.1f} Hz, "
        f"substeps={physics_substeps}, render_every={render_every_n_steps}"
    )
    return render_every_n_steps

def _import_isaac_world_types():
    try:
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage

        return World, Robot, add_reference_to_stage
    except Exception:
        from omni.isaac.core import World
        from omni.isaac.core.robots import Robot
        from omni.isaac.core.utils.stage import add_reference_to_stage

        return World, Robot, add_reference_to_stage


def _enable_webrtc_extension():
    try:
        from isaacsim.core.utils.extensions import enable_extension
    except Exception:
        from omni.isaac.core.utils.extensions import enable_extension
    enable_extension("omni.kit.livestream.webrtc")


def _apply_default_scene_lighting():
    try:
        import omni.usd
        from franka_isaac import scene as franka_scene

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        franka_scene.setup_lighting(stage)
    except Exception as error:
        print(f"Warning: Failed to apply default lighting setup: {error}")


def _resolve_simulation_experience(stream_enabled):
    if not stream_enabled:
        return None

    candidates = [
        "/isaac-sim/apps/isaacsim.exp.full.streaming.kit",
        "/isaac-sim/apps/isaacsim.exp.full.kit",
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    return None


def _configure_layout_defaults(layout_path):
    if not layout_path:
        return
    if not os.path.exists(layout_path):
        return

    layout_arg = f"--/app/layout/file={layout_path}"
    persistent_layout_arg = f"--/persistent/app/window/layout={layout_path}"

    if layout_arg not in sys.argv:
        sys.argv.append(layout_arg)
    if persistent_layout_arg not in sys.argv:
        sys.argv.append(persistent_layout_arg)
    print(f"Using startup layout: {layout_path}")


def _prepare_simulation_app_argv(unknown_args, stream_enabled=False, local_window=False):
    if unknown_args is None:
        unknown_args = []
    forwarded_args = list(unknown_args)
    if (
        stream_enabled
        and not local_window
        and "--no-window" not in forwarded_args
    ):
        forwarded_args = ["--no-window", *forwarded_args]
    if not _contains_portable_root_arg(forwarded_args):
        try:
            os.makedirs(DEFAULT_PORTABLE_ROOT, exist_ok=True)
        except OSError as error:
            print(
                f"Warning: Failed to create default portable-root "
                f"'{DEFAULT_PORTABLE_ROOT}': {error}"
            )
        forwarded_args = ["--portable-root", DEFAULT_PORTABLE_ROOT, *forwarded_args]
    sys.argv[:] = [sys.argv[0], *forwarded_args]


def _contains_portable_root_arg(args):
    for arg in args:
        if arg == "--portable-root":
            return True
        if str(arg).startswith("--portable-root="):
            return True
    return False


def _create_simulation_app(config, experience_path):
    if not experience_path:
        return SimulationApp(config)

    try:
        return SimulationApp(config, experience=experience_path)
    except TypeError:
        return SimulationApp(config, experience_path)
    except Exception as error:
        print(
            "Warning: Failed to launch SimulationApp with explicit experience "
            f"'{experience_path}': {error}"
        )
        return SimulationApp(config)


def _open_stage_context(usd_path):
    try:
        import omni.usd

        context = omni.usd.get_context()
        if context is None:
            return None
        opened = context.open_stage(usd_path)
        if opened is False:
            return None
        return context.get_stage()
    except Exception as error:
        print(f"Warning: Failed to open stage directly: {error}")
        return None


def _get_current_stage():
    try:
        import omni.usd

        context = omni.usd.get_context()
        if context is None:
            return None
        return context.get_stage()
    except Exception:
        return None


def _apply_model_alias(stage, alias_name):
    if stage is None:
        return
    renamed_any = False
    try:
        for prim in stage.Traverse():
            if prim.GetName() != "ai_cell":
                continue
            prim.SetDisplayName(alias_name)
            renamed_any = True
    except Exception as error:
        print(f"Warning: Failed to set model display alias '{alias_name}': {error}")
        return

    if renamed_any:
        print(f"Applied display alias '{alias_name}' to ai_cell prims.")


def _find_articulation_paths(stage):
    if stage is None:
        return []
    try:
        from pxr import UsdPhysics
    except Exception:
        return []
    # Joint prim types that can carry ArticulationRootAPI but are not body prims.
    # PhysX requires instantiating the articulation view with the ROOT BODY path, not a
    # joint path, to get dof_names and working set_joint_position_targets calls.
    _joint_type_names = frozenset((
        "PhysicsFixedJoint", "PhysicsRevoluteJoint", "PhysicsPrismaticJoint",
        "PhysicsSphericalJoint", "PhysicsD6Joint",
    ))

    paths = []
    try:
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                continue
            path = str(prim.GetPath())
            # If ArticulationRootAPI is on a joint prim, resolve to body1 (the root body).
            if prim.GetTypeName() in _joint_type_names:
                body1 = prim.GetRelationship("physics:body1").GetTargets()
                if body1:
                    path = str(body1[0])
            paths.append(path)
    except Exception:
        return []
    return paths


def _stage_has_valid_prim(stage, prim_path):
    if stage is None or not prim_path:
        return False
    try:
        prim = stage.GetPrimAtPath(prim_path)
        return bool(prim and prim.IsValid())
    except Exception:
        return False


def _resolve_robot_prim_path(stage, preferred_path=None):
    articulation_paths = _find_articulation_paths(stage)
    if not articulation_paths:
        return None

    def find_under(path_prefix):
        if not path_prefix:
            return None
        normalized = path_prefix.rstrip("/")
        if normalized in articulation_paths:
            return normalized
        prefix = normalized + "/"
        for path in articulation_paths:
            if path.startswith(prefix):
                return path
        return None

    candidate_prefixes = [
        preferred_path,
        "/World/fr3duo_m_v",
        "/World/fr3_duo",
        "/World/ai_cell",
        "/World",
    ]
    for candidate in candidate_prefixes:
        match = find_under(candidate)
        if match:
            return match

    for path in articulation_paths:
        lowered = path.lower()
        if "fr3" in lowered or "ai_cell" in lowered:
            return path
    return articulation_paths[0]


def _resolve_robot_prim_path_with_retry(
    stage,
    preferred_path,
    simulation_app,
    timeout_s=8.0,
):
    if stage is None:
        return None

    timeout_s = max(float(timeout_s), 0.0)
    deadline = time.monotonic() + timeout_s
    attempts = 0
    last_stage = stage

    while True:
        candidate = _resolve_robot_prim_path(last_stage, preferred_path=preferred_path)
        if candidate:
            if attempts > 0:
                print(
                    f"Resolved robot articulation path after {attempts} retry attempts: {candidate}"
                )
            return candidate

        if time.monotonic() >= deadline:
            break

        attempts += 1
        try:
            simulation_app.update()
        except Exception:
            pass

        refreshed_stage = _get_current_stage()
        if refreshed_stage is not None:
            last_stage = refreshed_stage
        time.sleep(0.05)

    articulation_paths = _find_articulation_paths(last_stage)
    if articulation_paths:
        preview = ", ".join(articulation_paths[:5])
        if len(articulation_paths) > 5:
            preview += ", ..."
        print(
            "Warning: Could not resolve preferred articulation path. "
            f"Discovered articulation roots: {preview}"
        )
    else:
        print("Warning: No articulation roots discovered in stage after retry window.")
    return None


def _has_available_display():
    display = os.environ.get("DISPLAY", "").strip()
    if not display:
        return False
    if not os.path.isdir("/tmp/.X11-unix"):
        return False

    # Validate that we can actually open the X display; DISPLAY being set is not enough.
    try:
        x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XOpenDisplay.restype = ctypes.c_void_p
        x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    except Exception:
        return False

    handle = x11.XOpenDisplay(display.encode("utf-8"))
    if not handle:
        return False
    try:
        x11.XCloseDisplay(handle)
    except Exception:
        pass
    return True


def _set_startup_camera_view(stage, robot_prim_path):
    if stage is None or not robot_prim_path:
        return

    try:
        from pxr import UsdGeom
    except Exception:
        return

    try:
        from isaacsim.core.utils.viewports import set_camera_view
    except Exception:
        try:
            from omni.isaac.core.utils.viewports import set_camera_view
        except Exception:
            return

    def _compute_aligned_range(bound):
        for method_name in ("ComputeAlignedRange", "ComputeAlignedBox", "GetRange"):
            method = getattr(bound, method_name, None)
            if not callable(method):
                continue
            try:
                return method()
            except Exception:
                continue
        return None

    def _ancestor_paths(path):
        current = str(path or "").strip()
        if not current.startswith("/"):
            return []
        last_token = current.rsplit("/", 1)[-1]
        if last_token.endswith("_joint"):
            parent = current.rsplit("/", 1)[0]
            current = parent if parent else "/"
        paths = []
        while current and current != "/":
            paths.append(current)
            current = current.rsplit("/", 1)[0]
            if current == "":
                current = "/"
        # Avoid "/" to prevent framing the entire stage.
        return paths

    try:
        bbox_cache = UsdGeom.BBoxCache(
            0.0,
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            True,
            False,
        )
        aligned_range = None
        framed_prim_path = None
        for candidate_path in _ancestor_paths(robot_prim_path):
            candidate_prim = stage.GetPrimAtPath(candidate_path)
            if not candidate_prim or not candidate_prim.IsValid():
                continue
            bbox = bbox_cache.ComputeWorldBound(candidate_prim)
            candidate_range = _compute_aligned_range(bbox)
            if candidate_range is None:
                continue
            get_min = getattr(candidate_range, "GetMin", None)
            get_max = getattr(candidate_range, "GetMax", None)
            if not callable(get_min) or not callable(get_max):
                continue
            min_point = get_min()
            max_point = get_max()
            extents = [abs(float(max_point[i]) - float(min_point[i])) for i in range(3)]
            if max(extents) <= 1e-4:
                continue
            aligned_range = candidate_range
            framed_prim_path = candidate_path
            break

        if aligned_range is None:
            return

        get_min = getattr(aligned_range, "GetMin", None)
        get_max = getattr(aligned_range, "GetMax", None)
        if not callable(get_min) or not callable(get_max):
            return

        min_point = get_min()
        max_point = get_max()
        min_xyz = [float(min_point[i]) for i in range(3)]
        max_xyz = [float(max_point[i]) for i in range(3)]

        center = [
            0.5 * (min_xyz[0] + max_xyz[0]),
            0.5 * (min_xyz[1] + max_xyz[1]),
            0.5 * (min_xyz[2] + max_xyz[2]),
        ]
        extents = [max(max_xyz[i] - min_xyz[i], 1e-4) for i in range(3)]
        radius = max(extents) * 0.5
        distance = min(max(3.0 * radius, 1.0), 6.0)

        # Guard against extreme/invalid stage bounds by falling back to a known good AI-cell view.
        if (
            max(abs(value) for value in center) > 20.0
            or max(extents) > 20.0
        ):
            eye = [2.5, 2.5, 1.5]
            target = [0.0, 0.0, 0.4]
            print(
                "Warning: Computed stage bounds are extreme. "
                "Using fallback camera view."
            )
        else:
            eye = [
                center[0] + distance,
                center[1] + distance,
                center[2] + 0.6 * distance,
            ]
            target = [center[0], center[1], center[2] + 0.2 * radius]
        set_camera_view(eye=eye, target=target)
        print(
            "Applied startup camera framing "
            f"(source prim: {framed_prim_path or robot_prim_path})"
        )
    except Exception as error:
        print(f"Warning: Failed to set startup camera framing: {error}")


def _rclpy_is_ok(rclpy_module):
    try:
        return rclpy_module.ok()
    except Exception:
        return False
