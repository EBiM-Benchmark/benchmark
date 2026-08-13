# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""Live per-stage eval indicators for the Task 3 teleop scene.

Draws a cluster of indicator spheres per grading stage (one sphere per point).
Spheres are red until their point is earned, then turn green. Scoring reuses
the pure helpers in ``scripts/evaluation/task3/grading.py`` fed with live prim
poses read each physics step.

Spheres are visual-only (no collider, no rigid body) so they never perturb the
simulation.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
GRADING_DIR = REPO_ROOT / "scripts" / "evaluation" / "task3"
if str(GRADING_DIR) not in sys.path:
    sys.path.insert(0, str(GRADING_DIR))

import grading  # noqa: E402

# build_stage() references the room USD under RobotRoom/Asset (see
# reference_usd), so objects resolve at ROOM_PRIM_PATH/<name>. Older/other
# layouts may drop the Asset wrapper, so the base is resolved at runtime.
ROOM_BASE_CANDIDATES = (
    "/World/Environment/RobotRoom/Asset",
    "/World/Environment/RobotRoom",
)
BEANS_SCOPE_PATH = "/World/Scene/CoffeeBeans"
OVERLAY_ROOT_PATH = "/World/EvalOverlay"

SPHERE_RADIUS = 0.1  # metres; "scale 0.2" diameter
SPHERE_SPACING = 0.3  # metres between adjacent spheres in a cluster
COLOR_RED = (0.85, 0.05, 0.05)
COLOR_GREEN = (0.05, 0.8, 0.1)

# Feed zone: stage 2 rewards holding the spoon near the fed person (the head)
# for a continuous interval. No feed-zone geometry ships in the asset, so
# define one here, centred on the head, as a first-cut proxy.
FEED_ZONE_RADIUS = 0.3
FEED_REQUIRED_HOLD_S = 3.0


def _stage_anchor_specs(
    head_world_pos: tuple[float, float, float],
    bean_recovery_region: grading.SphereRegion,
    sink_region: grading.SinkRegion,
):
    """Return the four stage clusters: (key, label, anchor_xyz, num_points)."""
    knock = bean_recovery_region.center
    sink = sink_region.bounds
    sink_center = (
        0.5 * (sink.x_min + sink.x_max),
        0.5 * (sink.y_min + sink.y_max),
        sink_region.tabletop_z,
    )
    return (
        ("stage1", "Table setup", (-5.0, 1.0, 2.0), 5),
        (
            "stage2",
            "Feed",
            (head_world_pos[0], head_world_pos[1], head_world_pos[2] + 0.6),
            4,
        ),
        (
            "stage3",
            "Bean recovery",
            (knock.x, knock.y, knock.z + 0.6),
            4,
        ),
        (
            "stage4",
            "Clean up",
            (sink_center[0], sink_center[1], sink_center[2] + 0.6),
            5,
        ),
    )


class EvalOverlay:
    """Creates and updates the per-stage indicator spheres."""

    def __init__(self, stage: Any):
        from pxr import Usd, UsdGeom

        self._UsdGeom = UsdGeom
        self._stage = stage

        self._xform_cache = UsdGeom.XformCache()
        self._bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            ["default", "render"],
            useExtentsHint=True,
        )

        self._room_base = self._resolve_room_base()
        self._head_world_pos = self._head_world_position()
        knock_bounds = self._world_bounds_3d("ikea_knock_box")
        if knock_bounds is None:
            raise RuntimeError(
                "Eval overlay: could not read ikea_knock_box bounds"
            )
        self._bean_recovery_region = grading.recovery_region_from_bounds(
            knock_bounds
        )
        sink_bounds = self._world_bounds_3d("sink_boundary")
        if sink_bounds is None:
            raise RuntimeError(
                "Eval overlay: could not read sink_boundary bounds"
            )
        self._sink_region = grading.sink_region_from_bounds(sink_bounds)

        # Per stage: list of Gprim colour-attribute handles, one per point.
        self._spheres: dict[str, list[Any]] = {}
        self._feed_state = grading.FeedHoldState()

        self._build_spheres()

    # ── stage resolution ────────────────────────────────────────────────
    def _resolve_room_base(self) -> str:
        for base in ROOM_BASE_CANDIDATES:
            prim = self._stage.GetPrimAtPath(f"{base}/head")
            if prim and prim.IsValid():
                return base
        raise RuntimeError(
            "Eval overlay: could not locate room objects. Tried: "
            + ", ".join(f"{b}/head" for b in ROOM_BASE_CANDIDATES)
        )

    def _head_world_position(self) -> tuple[float, float, float]:
        prim = self._stage.GetPrimAtPath(f"{self._room_base}/head")
        world = self._xform_cache.GetLocalToWorldTransform(prim)
        t = world.ExtractTranslation()
        return (float(t[0]), float(t[1]), float(t[2]))

    # ── construction ────────────────────────────────────────────────────
    def _build_spheres(self) -> None:
        UsdGeom = self._UsdGeom
        UsdGeom.Scope.Define(self._stage, OVERLAY_ROOT_PATH)

        for key, _label, anchor, count in _stage_anchor_specs(
            self._head_world_pos,
            self._bean_recovery_region,
            self._sink_region,
        ):
            handles = []
            # Centre the row on the anchor along +Y.
            start_y = anchor[1] - 0.5 * (count - 1) * SPHERE_SPACING
            for i in range(count):
                path = f"{OVERLAY_ROOT_PATH}/{key}_{i}"
                sphere = UsdGeom.Sphere.Define(self._stage, path)
                sphere.CreateRadiusAttr(SPHERE_RADIUS)
                xform = UsdGeom.Xformable(sphere.GetPrim())
                xform.AddTranslateOp().Set(
                    _vec3d(anchor[0], start_y + i * SPHERE_SPACING, anchor[2])
                )
                color_attr = sphere.CreateDisplayColorAttr()
                color_attr.Set([_vec3f(*COLOR_RED)])
                handles.append(color_attr)
            self._spheres[key] = handles

    # ── update ──────────────────────────────────────────────────────────
    def update(self, dt: float) -> None:
        self._xform_cache.Clear()
        self._bbox_cache.Clear()

        self._update_stage1()
        self._update_stage2(dt)
        self._update_stage3()
        self._update_stage4()

    def _set_points(self, key: str, passed_count: int) -> None:
        for i, color_attr in enumerate(self._spheres[key]):
            color = COLOR_GREEN if i < passed_count else COLOR_RED
            color_attr.Set([_vec3f(*color)])

    def _world_point(self, name: str) -> grading.Point3D | None:
        prim = self._stage.GetPrimAtPath(f"{self._room_base}/{name}")
        if not prim or not prim.IsValid():
            return None
        world = self._xform_cache.GetLocalToWorldTransform(prim)
        t = world.ExtractTranslation()
        return grading.Point3D(float(t[0]), float(t[1]), float(t[2]))

    def _world_bounds(self, name: str):
        bounds_3d = self._world_bounds_3d(name)
        if bounds_3d is None:
            return None, None
        bounds = grading.Bounds2D(
            bounds_3d.x_min,
            bounds_3d.y_min,
            bounds_3d.x_max,
            bounds_3d.y_max,
        )
        return bounds, bounds_3d.z_min

    def _world_bounds_3d(self, name: str) -> grading.Bounds3D | None:
        prim = self._stage.GetPrimAtPath(f"{self._room_base}/{name}")
        if not prim or not prim.IsValid():
            return None
        rng = self._bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            return None
        lo = rng.GetMin()
        hi = rng.GetMax()
        return grading.Bounds3D(
            x_min=float(lo[0]),
            y_min=float(lo[1]),
            z_min=float(lo[2]),
            x_max=float(hi[0]),
            y_max=float(hi[1]),
            z_max=float(hi[2]),
        )

    def _update_stage1(self) -> None:
        positions = {}
        for name in grading.DEFAULT_STAGE1_OBJECTS:
            point = self._world_point(name)
            if point is not None:
                positions[name] = point
        # Recolor per object rather than as an opaque count so each sphere maps
        # to a specific object in the DEFAULT_STAGE1_OBJECTS order.
        for i, name in enumerate(grading.DEFAULT_STAGE1_OBJECTS):
            point = positions.get(name)
            passed = (
                point is not None
                and grading.classify_table_area(point) == "dining"
            )
            self._spheres["stage1"][i].Set(
                [_vec3f(*(COLOR_GREEN if passed else COLOR_RED))]
            )

    def _update_stage2(self, dt: float) -> None:
        spoon = self._world_point("spoon2")
        beans_left = self._count_beans_outside_bowl()
        in_zone = False
        if spoon is not None:
            head = grading.Point3D(*self._head_world_pos)
            zone = grading.SphereRegion(head, FEED_ZONE_RADIUS)
            in_zone = zone.contains(spoon)
        self._feed_state = grading.update_feed_hold(
            self._feed_state,
            bean_count=beans_left,
            in_feed_zone=in_zone,
            dt=dt,
            required_hold_seconds=FEED_REQUIRED_HOLD_S,
        )
        # First cut: all-or-nothing once the continuous hold completes.
        passed = self._spheres["stage2"]
        count = len(passed) if self._feed_state.completed else 0
        self._set_points("stage2", count)

    def _update_stage3(self) -> None:
        beans = self._bean_points()
        total = len(beans)
        inside = grading.count_points_in_sphere(
            beans, self._bean_recovery_region
        )
        score = grading.bean_recovery_score(inside, total) if total else 0
        self._set_points("stage3", score)

    def _update_stage4(self) -> None:
        bounds_map = {}
        z_map = {}
        for name in grading.DEFAULT_UTENSIL_OBJECTS:
            bounds, z_min = self._world_bounds(name)
            if bounds is not None:
                bounds_map[name] = bounds
                z_map[name] = z_min
        for i, name in enumerate(grading.DEFAULT_UTENSIL_OBJECTS):
            passed = False
            if name in bounds_map:
                sink = self._sink_region
                passed = (
                    bounds_map[name].overlaps(sink.bounds)
                    and z_map[name] >= sink.tabletop_z
                )
            self._spheres["stage4"][i].Set(
                [_vec3f(*(COLOR_GREEN if passed else COLOR_RED))]
            )

    # ── bean helpers ────────────────────────────────────────────────────
    def _bean_points(self) -> list[grading.Point3D]:
        scope = self._stage.GetPrimAtPath(BEANS_SCOPE_PATH)
        if not scope or not scope.IsValid():
            return []
        points = []
        for prim in scope.GetChildren():
            world = self._xform_cache.GetLocalToWorldTransform(prim)
            t = world.ExtractTranslation()
            points.append(
                grading.Point3D(float(t[0]), float(t[1]), float(t[2]))
            )
        return points

    def _count_beans_outside_bowl(self) -> int:
        # Proxy for "beans available to feed": any bean lifted above the bowl
        # rim counts as picked up / on the spoon.
        beans = self._bean_points()
        if not beans:
            return 0
        bowl = self._world_point("bowl2")
        if bowl is None:
            return len(beans)
        return sum(1 for b in beans if b.z > bowl.z + 0.05)


def _vec3f(x: float, y: float, z: float):
    from pxr import Gf

    return Gf.Vec3f(x, y, z)


def _vec3d(x: float, y: float, z: float):
    from pxr import Gf

    return Gf.Vec3d(x, y, z)
