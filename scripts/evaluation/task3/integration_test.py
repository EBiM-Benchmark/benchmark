#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Isaac Sim integration tests for Task 3 grading.

This runner launches a real Isaac Sim application, builds the Task3 room scene,
drives live scene prims through stage-specific validation motions, then
evaluates the result with the pure grading helpers.
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path
from typing import Any

TASK3_DIR = Path(__file__).resolve().parent
REPO_ROOT = TASK3_DIR.parents[2]
SCENES_DIR = REPO_ROOT / "scripts" / "scenes"
COMMON_DIR = REPO_ROOT / "scripts" / "common"
for import_path in (TASK3_DIR, SCENES_DIR, COMMON_DIR):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from grading import (  # noqa: E402
    DEFAULT_STAGE1_OBJECTS,
    DEFAULT_UTENSIL_OBJECTS,
    TASK3_BEAN_RECOVERY_REGION,
    TASK3_BEAN_SPAWN_POSITION,
    TASK3_SINK_REGION,
    Bounds2D,
    FeedHoldState,
    Point3D,
    bean_recovery_score,
    classify_table_area,
    count_points_in_sphere,
    feed_score,
    movement_is_smooth,
    score_stage1_table_setup,
    score_stage4_cleanup,
    update_feed_hold,
)
from path_utils import asset_path  # noqa: E402

STAGE_NAMES = ("stage1", "stage2", "stage3", "stage4")
IDENTITY_QUAT = (1.0, 0.0, 0.0, 0.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Task3 grading integration tests in Isaac Sim.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "stage",
        nargs="?",
        default="all",
        choices=(*STAGE_NAMES, "all"),
        help="Stage integration test to run.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac Sim without opening a GUI window.",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=12,
        help="App update frames after each deterministic manipulation.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.headless,
            "width": 1280,
            "height": 720,
        }
    )

    try:
        import scene_robot_room_keyboard as scene

        selected = STAGE_NAMES if args.stage == "all" else (args.stage,)
        results = []
        for stage_name in selected:
            stage = create_task3_stage(
                app,
                scene,
                args.frames,
                disable_utensil_rigid_bodies=stage_name == "stage1",
                include_beans=stage_name != "stage4",
            )
            print(f"STAGE_START {stage_name}", flush=True)
            result = INTEGRATION_TESTS[stage_name](app, stage, args.frames)
            results.append(result)
            print(
                "STAGE_RESULT " + json.dumps(result, sort_keys=True),
                flush=True,
            )

        failed = [result for result in results if not result["passed"]]
        exit_code = 1 if failed else 0
        if not args.headless:
            keep_gui_open(app)
        if exit_code:
            raise SystemExit(exit_code)
    finally:
        if args.headless:
            app.close()


def create_task3_stage(
    app: Any,
    scene: Any,
    frames: int,
    *,
    disable_utensil_rigid_bodies: bool = False,
    include_beans: bool = True,
) -> Any:
    import omni.usd

    context = omni.usd.get_context()
    context.new_stage()
    for _ in range(frames):
        app.update()

    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Could not create Isaac Sim stage.")

    scene.configure_robot_room_stage(
        app,
        stage,
        room_path=asset_path("robot_room.usd"),
        task="task3",
        head_placement="A",
        dynamic_beans=True,
    )
    if not include_beans:
        remove_coffee_beans(stage)
    if disable_utensil_rigid_bodies:
        for name in DEFAULT_STAGE1_OBJECTS:
            set_rigid_bodies_enabled_under(
                stage, resolve_prim_path(stage, name), False
            )
    for _ in range(frames):
        app.update()
    return stage


def keep_gui_open(app: Any) -> None:
    import signal

    shutdown_requested = False

    def request_shutdown(signum: int, _frame: Any) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True
        print(f"GUI_CLOSE signal={signum}", flush=True)
        request_kit_quit()
        raise KeyboardInterrupt

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    print(
        "GUI_OPEN Results printed. Close Isaac Sim or press Ctrl+C.",
        flush=True,
    )
    try:
        try:
            while app.is_running() and not shutdown_requested:
                app.update()
        except KeyboardInterrupt:
            shutdown_requested = True
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)

    if shutdown_requested:
        app.close()
        raise SystemExit(130)


def request_kit_quit() -> None:
    try:
        import omni.kit.app

        omni.kit.app.get_app().post_quit()
    except Exception:
        pass


def run_stage1(app: Any, stage: Any, frames: int) -> dict[str, Any]:
    import omni.timeline

    tray_path = resolve_prim_path(stage, "simple_tray")
    start = get_prim_position(stage, tray_path)
    target = Point3D(-2.85, 1.90, start.z)
    object_paths = {
        name: resolve_prim_path(stage, name) for name in DEFAULT_STAGE1_OBJECTS
    }
    bean_paths = sorted_bean_paths(stage)
    for index, bean_path in enumerate(bean_paths):
        set_rigid_bodies_enabled_under(stage, bean_path, True)
        set_rigid_bodies_kinematic_under(stage, bean_path, True)
        object_paths[f"bean_{index:04d}"] = bean_path
    start_positions = {
        name: get_prim_position(stage, path)
        for name, path in object_paths.items()
    }

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    drive_group_translation(
        app,
        stage,
        object_paths,
        start_positions,
        y_then_x_path(start, target, steps_per_axis=24),
        frames_per_step=max(1, frames // 4),
    )
    step_app(app, max(frames, 30))
    timeline.stop()

    positions = {
        name: get_named_prim_position(stage, name)
        for name in DEFAULT_STAGE1_OBJECTS
    }
    bean_positions = [
        get_prim_position(stage, bean_path) for bean_path in bean_paths
    ]
    beans_in_dining = sum(
        1
        for position in bean_positions
        if classify_task3_area(position) == "dining"
    )
    score = score_stage1_table_setup(positions)
    result = stage_result(
        "stage1", score.score, score.max_score, score.score == 5
    )
    result["objects_in_dining"] = score.score
    result["objects_total"] = score.max_score
    result["objects_in_dining_percent"] = percentage(
        score.score, score.max_score
    )
    result["objects_passed"] = score.passed
    result["objects_failed"] = score.failed
    result["beans_in_dining"] = beans_in_dining
    result["beans_total"] = len(bean_paths)
    result["beans_in_dining_percent"] = percentage(
        beans_in_dining, len(bean_paths)
    )
    return result


def run_stage2(app: Any, stage: Any, frames: int) -> dict[str, Any]:
    import omni.timeline

    bean_paths = sorted_bean_paths(stage)[:5]
    head_feed_pose = stage2_feed_pose(stage)
    spoon_start = Point3D(
        head_feed_pose.x, head_feed_pose.y - 0.20, head_feed_pose.z
    )
    insertion_pose = Point3D(
        head_feed_pose.x,
        head_feed_pose.y - 0.10,
        head_feed_pose.z,
    )
    retract_pose = spoon_start
    set_stage2_perspective_view(app, spoon_start, insertion_pose)
    spoon_path = resolve_prim_path(stage, "spoon2")
    spoon_rigid_body_paths = find_rigid_body_paths_under(stage, spoon_path)
    set_rigid_bodies_enabled_under(stage, spoon_path, True)
    set_rigid_bodies_kinematic_under(stage, spoon_path, True)
    for bean_path in bean_paths:
        set_rigid_bodies_enabled_under(stage, bean_path, True)
        set_rigid_bodies_kinematic_under(stage, bean_path, False)
    set_stage2_spoon_pose(stage, spoon_path, {}, spoon_start)
    spoon_body_offsets = capture_spoon_body_offsets(
        stage,
        spoon_rigid_body_paths,
        spoon_start,
    )
    set_stage2_spoon_pose(stage, spoon_path, spoon_body_offsets, spoon_start)
    place_beans_on_spoon(stage, bean_paths, spoon_start)
    beans_after_place = count_beans_on_spoon(stage, bean_paths, spoon_start)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    step_app(app, max(frames, 90))
    beans_after_settle = count_beans_on_spoon(stage, bean_paths, spoon_start)
    insertion_path = interpolate_path(spoon_start, insertion_pose, 180)
    hold = drive_spoon_feed_path(
        app,
        stage,
        spoon_path,
        spoon_body_offsets,
        bean_paths,
        insertion_path,
    )
    beans_after_insert = count_beans_on_spoon(
        stage, bean_paths, insertion_pose
    )

    retract_path = interpolate_path(insertion_pose, retract_pose, 90)
    drive_spoon_along_path(
        app,
        stage,
        spoon_path,
        spoon_body_offsets,
        retract_path,
    )
    final_spoon_position = retract_pose
    set_stage2_spoon_pose(
        stage, spoon_path, spoon_body_offsets, final_spoon_position
    )
    step_app(app, max(frames, 30))
    timeline.stop()

    beans_left = count_beans_on_spoon(stage, bean_paths, final_spoon_position)
    smooth = movement_is_smooth([*insertion_path, *retract_path], max_step=1.5)
    score = feed_score(
        beans_left=beans_left,
        hold_seconds=hold.hold_seconds,
        smooth=smooth,
    )
    result = stage_result("stage2", score, 4, score == 4)
    result["beans_left"] = beans_left
    result["beans_total"] = len(bean_paths)
    result["beans_left_percent"] = percentage(beans_left, len(bean_paths))
    result["beans_after_insert"] = beans_after_insert
    result["beans_after_place"] = beans_after_place
    result["beans_after_settle"] = beans_after_settle
    result["hold_seconds"] = round(hold.hold_seconds, 3)
    result["required_hold_seconds"] = 3.0
    result["smooth_motion"] = smooth
    result["initial_head_offset_m"] = 0.20
    result["insertion_distance_m"] = 0.10
    result["closest_head_offset_m"] = 0.10
    result["spoon_z_rotation_deg"] = 90.0
    result["spoon_rigid_body_count"] = len(spoon_rigid_body_paths)
    result["spoon_root_is_rigid"] = spoon_path in spoon_rigid_body_paths
    return result


def run_stage3(app: Any, stage: Any, frames: int) -> dict[str, Any]:
    import scene_robot_room_keyboard as scene

    import omni.timeline

    bean_paths = sorted_bean_paths(stage)
    random.seed(3)
    spawn_points = scene.bean_spawn_positions(
        len(bean_paths),
        (
            TASK3_BEAN_SPAWN_POSITION.x,
            TASK3_BEAN_SPAWN_POSITION.y,
            TASK3_BEAN_SPAWN_POSITION.z,
        ),
    )
    for bean_path, point in zip(bean_paths, spawn_points):
        set_prim_position(stage, bean_path, Point3D(*point))
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    step_app(app, max(frames, 180))
    timeline.stop()

    positions = [
        get_prim_position(stage, bean_path) for bean_path in bean_paths
    ]
    beans_inside = count_points_in_sphere(positions)
    score = bean_recovery_score(beans_inside, len(bean_paths))
    result = stage_result("stage3", score, 4, score >= 3)
    result["beans_inside_sphere"] = beans_inside
    result["beans_total"] = len(bean_paths)
    result["beans_inside_sphere_percent"] = percentage(
        beans_inside, len(bean_paths)
    )
    result["sphere_center"] = point_to_list(TASK3_BEAN_RECOVERY_REGION.center)
    result["sphere_radius"] = TASK3_BEAN_RECOVERY_REGION.radius
    return result


def run_stage4(app: Any, stage: Any, frames: int) -> dict[str, Any]:
    sink = TASK3_SINK_REGION.bounds
    sink_center = Point3D(
        0.5 * (sink.x_min + sink.x_max),
        0.5 * (sink.y_min + sink.y_max),
        TASK3_SINK_REGION.tabletop_z + 0.05,
    )
    object_paths = {
        name: resolve_prim_path(stage, name)
        for name in DEFAULT_UTENSIL_OBJECTS
    }
    for prim_path in object_paths.values():
        set_rigid_bodies_enabled_under(stage, prim_path, False)
    start_positions = {
        name: get_prim_position(stage, path)
        for name, path in object_paths.items()
    }
    target_positions = {
        name: Point3D(
            sink_center.x + (index - 2) * 0.02,
            sink_center.y,
            sink_center.z + index * 0.005,
        )
        for index, name in enumerate(DEFAULT_UTENSIL_OBJECTS)
    }
    drive_individual_y_then_x(
        app,
        stage,
        object_paths,
        start_positions,
        target_positions,
        steps_per_axis=24,
        frames_per_step=max(1, frames // 4),
    )
    step_app(app, frames)

    bounds = {
        name: get_named_prim_bounds_2d(stage, name)
        for name in DEFAULT_UTENSIL_OBJECTS
    }
    z_values = {
        name: get_named_prim_position(stage, name).z
        for name in DEFAULT_UTENSIL_OBJECTS
    }
    score = score_stage4_cleanup(bounds, z_values)
    result = stage_result(
        "stage4", score.score, score.max_score, score.score == 5
    )
    result["objects_in_sink"] = score.score
    result["objects_total"] = score.max_score
    result["objects_in_sink_percent"] = percentage(
        score.score, score.max_score
    )
    result["objects_passed"] = score.passed
    result["objects_failed"] = score.failed
    result["coffee_beans_spawned"] = bool(sorted_bean_paths(stage))
    result["sink_bounds"] = {
        "x_min": sink.x_min,
        "x_max": sink.x_max,
        "y_min": sink.y_min,
        "y_max": sink.y_max,
        "tabletop_z": TASK3_SINK_REGION.tabletop_z,
    }
    return result


def interpolate_path(
    start: Point3D, end: Point3D, steps: int
) -> list[Point3D]:
    points = []
    for index in range(max(1, steps) + 1):
        alpha = index / max(1, steps)
        points.append(
            Point3D(
                start.x + (end.x - start.x) * alpha,
                start.y + (end.y - start.y) * alpha,
                start.z + (end.z - start.z) * alpha,
            )
        )
    return points


def y_then_x_path(
    start: Point3D, end: Point3D, *, steps_per_axis: int
) -> list[Point3D]:
    midpoint = Point3D(start.x, end.y, end.z)
    return path_from_waypoints(
        [start, midpoint, end], steps_per_segment=steps_per_axis
    )


def path_from_waypoints(
    waypoints: list[Point3D],
    *,
    steps_per_segment: int,
) -> list[Point3D]:
    points = []
    for start, end in zip(waypoints, waypoints[1:]):
        segment = interpolate_path(start, end, steps_per_segment)
        points.extend(segment if not points else segment[1:])
    return points


def drive_group_translation(
    app: Any,
    stage: Any,
    object_paths: dict[str, str],
    start_positions: dict[str, Point3D],
    path: list[Point3D],
    *,
    frames_per_step: int = 1,
) -> None:
    if not path:
        return

    tray_start = path[0]
    for tray_point in path:
        delta = Point3D(
            tray_point.x - tray_start.x,
            tray_point.y - tray_start.y,
            tray_point.z - tray_start.z,
        )
        for name, prim_path in object_paths.items():
            start = start_positions[name]
            translate_prim_preserving_rotation(
                stage,
                prim_path,
                Point3D(
                    start.x + delta.x, start.y + delta.y, start.z + delta.z
                ),
            )
        step_app(app, frames_per_step)


def drive_individual_y_then_x(
    app: Any,
    stage: Any,
    object_paths: dict[str, str],
    start_positions: dict[str, Point3D],
    target_positions: dict[str, Point3D],
    *,
    steps_per_axis: int,
    frames_per_step: int = 1,
) -> None:
    paths = {
        name: y_then_x_path(
            start_positions[name],
            target_positions[name],
            steps_per_axis=steps_per_axis,
        )
        for name in object_paths
    }
    path_length = max(len(path) for path in paths.values())
    for index in range(path_length):
        for name, prim_path in object_paths.items():
            path = paths[name]
            point = path[min(index, len(path) - 1)]
            translate_prim_preserving_rotation(stage, prim_path, point)
        step_app(app, frames_per_step)


def drive_spoon_along_path(
    app: Any,
    stage: Any,
    spoon_path: str,
    spoon_body_offsets: dict[str, Point3D],
    path: list[Point3D],
    *,
    frames_per_step: int = 1,
) -> None:
    if not path:
        return

    for point in path:
        set_stage2_spoon_pose(stage, spoon_path, spoon_body_offsets, point)
        step_app(app, frames_per_step)


def drive_spoon_feed_path(
    app: Any,
    stage: Any,
    spoon_path: str,
    spoon_body_offsets: dict[str, Point3D],
    bean_paths: list[str],
    path: list[Point3D],
) -> FeedHoldState:
    hold = FeedHoldState()
    for point in path:
        set_stage2_spoon_pose(stage, spoon_path, spoon_body_offsets, point)
        bean_count = count_beans_on_spoon(stage, bean_paths, point)
        hold = update_feed_hold(
            hold,
            bean_count=bean_count,
            in_feed_zone=True,
            dt=1.0 / 60.0,
        )
        app.update()
    return hold


def place_beans_on_spoon(
    stage: Any,
    bean_paths: list[str],
    spoon_position: Point3D,
) -> None:
    set_initial_beans_on_spoon(stage, bean_paths, spoon_position)


def set_initial_beans_on_spoon(
    stage: Any,
    bean_paths: list[str],
    spoon_position: Point3D,
) -> None:
    for index, bean_path in enumerate(bean_paths):
        set_prim_position(
            stage, bean_path, spoon_bean_position(spoon_position, index)
        )


def spoon_bean_position(spoon_position: Point3D, index: int) -> Point3D:
    row = index // 3
    column = index % 3
    return Point3D(
        spoon_position.x + (column - 1) * 0.003,
        spoon_position.y + (row - 0.5) * 0.003,
        spoon_position.z + 0.010,
    )


def stage2_feed_pose(stage: Any) -> Point3D:
    head = get_prim_position(stage, resolve_prim_path(stage, "head"))
    return Point3D(head.x, head.y, head.z + 0.17)


def set_stage2_perspective_view(
    app: Any,
    spoon_start: Point3D,
    feed_pose: Point3D,
) -> None:
    try:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState
        from pxr import Gf

        camera_path = "/OmniverseKit_Persp"
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = camera_path
            try:
                camera_state = ViewportCameraState(camera_path, viewport)
            except TypeError:
                camera_state = ViewportCameraState(camera_path)
            camera_state.set_position_world(
                Gf.Vec3d(
                    -3.4481111251265144,
                    1.0918516227594448,
                    1.2729373463969458,
                ),
                True,
            )
            camera_state.set_target_world(
                Gf.Vec3d(
                    0.5 * (spoon_start.x + feed_pose.x),
                    0.5 * (spoon_start.y + feed_pose.y),
                    spoon_start.z,
                ),
                True,
            )
            app.update()
    except Exception:
        pass


def capture_spoon_body_offsets(
    stage: Any,
    spoon_rigid_body_paths: list[str],
    spoon_position: Point3D,
) -> dict[str, Point3D]:
    offsets = {}
    for body_path in spoon_rigid_body_paths:
        body_position = get_prim_position(stage, body_path)
        offsets[body_path] = Point3D(
            body_position.x - spoon_position.x,
            body_position.y - spoon_position.y,
            body_position.z - spoon_position.z,
        )
    return offsets


def set_stage2_spoon_pose(
    stage: Any,
    spoon_path: str,
    spoon_body_offsets: dict[str, Point3D],
    point: Point3D,
) -> None:
    set_spoon_feed_pose(stage, spoon_path, point)
    for body_path, offset in spoon_body_offsets.items():
        set_prim_world_translation_preserving_rotation(
            stage,
            body_path,
            Point3D(
                point.x + offset.x, point.y + offset.y, point.z + offset.z
            ),
        )


def set_spoon_feed_pose(stage: Any, spoon_path: str, point: Point3D) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(spoon_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {spoon_path}")

    xform = UsdGeom.Xformable(prim)
    translate_set = False
    rotate_set = False
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(point.x, point.y, point.z))
            translate_set = True
        elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
            op.Set(90.0)
            rotate_set = True

    if not translate_set:
        xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(point.x, point.y, point.z)
        )
    if not rotate_set:
        xform.AddRotateZOp(UsdGeom.XformOp.PrecisionFloat).Set(90.0)
    zero_rigid_body_velocity(prim)


def set_prim_world_translation_preserving_rotation(
    stage: Any,
    prim_path: str,
    point: Point3D,
) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {prim_path}")

    parent = prim.GetParent()
    target = Gf.Vec3d(point.x, point.y, point.z)
    if parent and parent.IsValid():
        cache = UsdGeom.XformCache()
        parent_world = cache.GetLocalToWorldTransform(parent)
        target = parent_world.GetInverse().Transform(target)

    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(target)
            zero_rigid_body_velocity(prim)
            return

    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(target)
    zero_rigid_body_velocity(prim)


def bean_is_on_spoon(
    stage: Any, bean_path: str, spoon_position: Point3D
) -> bool:
    bean = get_prim_position(stage, bean_path)
    dx = bean.x - spoon_position.x
    dy = bean.y - spoon_position.y
    dz = bean.z - spoon_position.z
    return dx * dx + dy * dy <= 0.060 * 0.060 and -0.020 <= dz <= 0.120


def count_beans_on_spoon(
    stage: Any,
    bean_paths: list[str],
    spoon_position: Point3D,
) -> int:
    return sum(
        1
        for bean_path in bean_paths
        if bean_is_on_spoon(stage, bean_path, spoon_position)
    )


def zero_rigid_body_velocity(prim: Any) -> None:
    set_rigid_body_velocity(prim, Point3D(0.0, 0.0, 0.0))


def set_rigid_body_velocity(prim: Any, velocity: Point3D) -> None:
    from pxr import Gf, UsdPhysics

    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return

    rigid_api = UsdPhysics.RigidBodyAPI(prim)
    rigid_api.CreateVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0)).Set(
        Gf.Vec3f(velocity.x, velocity.y, velocity.z)
    )
    rigid_api.CreateAngularVelocityAttr(Gf.Vec3f(0.0, 0.0, 0.0)).Set(
        Gf.Vec3f(0.0, 0.0, 0.0)
    )


INTEGRATION_TESTS = {
    "stage1": run_stage1,
    "stage2": run_stage2,
    "stage3": run_stage3,
    "stage4": run_stage4,
}


def stage_result(
    stage_name: str,
    score: int,
    max_score: int,
    passed: bool,
) -> dict[str, Any]:
    return {
        "stage": stage_name,
        "score": score,
        "max_score": max_score,
        "passed": passed,
    }


def percentage(count: int, total: int) -> float:
    if total <= 0:
        return 0.0
    return round(100.0 * count / total, 2)


def point_to_list(point: Point3D) -> list[float]:
    return [point.x, point.y, point.z]


def classify_task3_area(point: Point3D) -> str:
    return classify_table_area(point)


def step_app(app: Any, frames: int) -> None:
    for _ in range(max(1, frames)):
        app.update()


def resolve_prim_path(stage: Any, name: str) -> str:
    candidates = (
        f"/World/Environment/RobotRoom/Asset/{name}",
        f"/World/Environment/RobotRoom/Asset/root/{name}",
        f"/root/{name}",
    )
    for candidate in candidates:
        prim = stage.GetPrimAtPath(candidate)
        if prim and prim.IsValid():
            return candidate

    suffix = f"/{name}"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.endswith(suffix):
            return path

    raise RuntimeError(f"Could not resolve prim named '{name}'")


def find_prim_paths(stage: Any, prefix: str) -> list[str]:
    paths = []
    for prim in stage.Traverse():
        name = prim.GetName()
        if name.startswith(prefix):
            paths.append(str(prim.GetPath()))
    return paths


def remove_coffee_beans(stage: Any) -> None:
    bean_scope = stage.GetPrimAtPath("/World/Scene/CoffeeBeans")
    if bean_scope and bean_scope.IsValid():
        stage.RemovePrim(bean_scope.GetPath())


def sorted_bean_paths(stage: Any) -> list[str]:
    return sorted(find_prim_paths(stage, "Bean_"), key=bean_path_index)


def bean_path_index(path: str) -> int:
    try:
        return int(path.rsplit("_", 1)[1])
    except ValueError:
        return sys.maxsize


def find_rigid_body_paths_under(stage: Any, root_path: str) -> list[str]:
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Invalid prim path: {root_path}")

    paths = []
    for prim in Usd.PrimRange(root):
        attr = prim.GetAttribute("physics:rigidBodyEnabled")
        if (attr and attr.IsValid()) or prim.HasAPI(UsdPhysics.RigidBodyAPI):
            paths.append(str(prim.GetPath()))
    return paths


def set_rigid_bodies_enabled_under(
    stage: Any,
    root_path: str,
    enabled: bool,
) -> None:
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Invalid prim path: {root_path}")

    for prim in Usd.PrimRange(root):
        attr = prim.GetAttribute("physics:rigidBodyEnabled")
        if attr and attr.IsValid():
            attr.Set(enabled)
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(
                enabled
            ).Set(enabled)


def set_rigid_bodies_kinematic_under(
    stage: Any,
    root_path: str,
    enabled: bool,
) -> None:
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Invalid prim path: {root_path}")

    for prim in Usd.PrimRange(root):
        rigid_attr = prim.GetAttribute("physics:rigidBodyEnabled")
        if rigid_attr and rigid_attr.IsValid():
            rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            rigid_api.CreateKinematicEnabledAttr(enabled).Set(enabled)


def translate_prim_preserving_rotation(
    stage: Any,
    prim_path: str,
    point: Point3D,
) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {prim_path}")

    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(point.x, point.y, point.z))
            zero_rigid_body_velocity(prim)
            return

    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(point.x, point.y, point.z)
    )
    zero_rigid_body_velocity(prim)


def set_prim_position(stage: Any, prim_path: str, point: Point3D) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Invalid prim path: {prim_path}")

    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(point.x, point.y, point.z)
    )
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(
        Gf.Quatf(*IDENTITY_QUAT)
    )
    zero_rigid_body_velocity(prim)


def get_named_prim_position(stage: Any, name: str) -> Point3D:
    return get_prim_position(stage, resolve_prim_path(stage, name))


def get_prim_position(stage: Any, prim_path: str) -> Point3D:
    from pxr import UsdGeom

    cache = UsdGeom.XformCache()
    prim = stage.GetPrimAtPath(prim_path)
    translation = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return Point3D(
        float(translation[0]),
        float(translation[1]),
        float(translation[2]),
    )


def get_named_prim_bounds_2d(stage: Any, name: str) -> Bounds2D:
    return get_prim_bounds_2d(stage, resolve_prim_path(stage, name))


def get_prim_bounds_2d(stage: Any, prim_path: str) -> Bounds2D:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    purposes = [
        UsdGeom.Tokens.default_,
        UsdGeom.Tokens.render,
        UsdGeom.Tokens.proxy,
    ]
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    bbox_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
    bbox_min = bbox_range.GetMin()
    bbox_max = bbox_range.GetMax()
    return Bounds2D(
        x_min=float(bbox_min[0]),
        y_min=float(bbox_min[1]),
        x_max=float(bbox_max[0]),
        y_max=float(bbox_max[1]),
    )


if __name__ == "__main__":
    main()
