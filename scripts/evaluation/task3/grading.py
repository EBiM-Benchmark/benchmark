# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Pure scoring helpers for Task 3 assisted-living grading.

The functions in this file avoid Isaac Sim imports so they can be unit tested
directly. Isaac Sim scripts can adapt prim poses and bounds into these small
value objects, then call the same scoring functions used by the tests.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from math import sqrt


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Area2D:
    center_x: float
    center_y: float
    scale_x: float
    scale_y: float

    @property
    def x_min(self) -> float:
        return self.center_x - 0.5 * self.scale_x

    @property
    def x_max(self) -> float:
        return self.center_x + 0.5 * self.scale_x

    @property
    def y_min(self) -> float:
        return self.center_y - 0.5 * self.scale_y

    @property
    def y_max(self) -> float:
        return self.center_y + 0.5 * self.scale_y

    def contains_xy(self, point: tuple[float, float] | Point3D) -> bool:
        x, y = _xy(point)
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


@dataclass(frozen=True)
class Bounds2D:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @classmethod
    def from_point(cls, point: tuple[float, float] | Point3D) -> Bounds2D:
        x, y = _xy(point)
        return cls(x, y, x, y)

    def overlaps(self, other: Bounds2D) -> bool:
        return (
            self.x_min <= other.x_max
            and self.x_max >= other.x_min
            and self.y_min <= other.y_max
            and self.y_max >= other.y_min
        )


@dataclass(frozen=True)
class SphereRegion:
    center: Point3D
    radius: float

    def contains(self, point: Point3D) -> bool:
        dx = point.x - self.center.x
        dy = point.y - self.center.y
        dz = point.z - self.center.z
        distance_squared = dx * dx + dy * dy + dz * dz
        radius_squared = self.radius * self.radius
        return distance_squared <= radius_squared + 1e-12


@dataclass(frozen=True)
class SinkRegion:
    bounds: Bounds2D
    tabletop_z: float


@dataclass(frozen=True)
class StageScore:
    score: int
    max_score: int
    passed: list[str]
    failed: list[str]


@dataclass(frozen=True)
class FeedHoldState:
    hold_seconds: float = 0.0
    completed: bool = False


TASK3_KITCHEN_AREA = Area2D(
    center_x=-4.2,
    center_y=-1.8,
    scale_x=3.2,
    scale_y=4.1,
)
TASK3_DINING_AREA = Area2D(
    center_x=-2.85,
    center_y=1.9,
    scale_x=5.9,
    scale_y=3.4,
)
TASK3_BEAN_SPAWN_POSITION = Point3D(-3.94, -1.92, 0.8)
TASK3_BEAN_RECOVERY_REGION = SphereRegion(
    center=Point3D(
        -3.9436692037194394,
        -1.9169676477173505,
        0.8598584143807657,
    ),
    radius=0.2,
)
TASK3_SINK_REGION = SinkRegion(
    bounds=Bounds2D(
        x_min=-4.245322,
        y_min=-2.412793,
        x_max=-3.805322,
        y_max=-2.042793,
    ),
    tabletop_z=0.74699,
)

DEFAULT_STAGE1_OBJECTS = (
    "simple_tray",
    "bowl2",
    "spoon2",
    "plate2",
    "cup",
)
DEFAULT_UTENSIL_OBJECTS = DEFAULT_STAGE1_OBJECTS


def classify_table_area(
    point: tuple[float, float] | Point3D,
    area: Area2D | None = None,
    *,
    kitchen_area: Area2D = TASK3_KITCHEN_AREA,
    dining_area: Area2D = TASK3_DINING_AREA,
) -> str:
    """Classify a point against one area or the Task 3 table regions."""

    if area is not None:
        return "inside" if area.contains_xy(point) else "outside"

    if kitchen_area.contains_xy(point):
        return "kitchen"

    x, y = _xy(point)
    in_dining_x = dining_area.x_min <= x <= dining_area.x_max
    dining_y_min = max(kitchen_area.y_max, dining_area.y_min)
    in_dining_y = dining_y_min < y <= dining_area.y_max
    if in_dining_x and in_dining_y:
        return "dining"
    return "outside"


def score_stage1_table_setup(
    object_positions: Mapping[str, Point3D],
    object_names: Sequence[str] = DEFAULT_STAGE1_OBJECTS,
) -> StageScore:
    passed = [
        name
        for name in object_names
        if name in object_positions
        and classify_table_area(object_positions[name]) == "dining"
    ]
    return StageScore(
        score=len(passed),
        max_score=len(object_names),
        passed=passed,
        failed=[name for name in object_names if name not in passed],
    )


def movement_is_smooth(
    positions: Sequence[Point3D],
    *,
    max_step: float,
) -> bool:
    if len(positions) < 2:
        return False

    for previous, current in zip(positions, positions[1:]):
        if _distance(previous, current) > max_step:
            return False
    return True


def feed_score(
    *,
    beans_left: int,
    hold_seconds: float,
    smooth: bool,
    required_hold_seconds: float = 3.0,
    max_points: int = 4,
) -> int:
    if not smooth or hold_seconds < required_hold_seconds:
        return 0
    return min(max(0, int(beans_left)), max_points)


def update_feed_hold(
    state: FeedHoldState,
    *,
    bean_count: int,
    in_feed_zone: bool,
    dt: float,
    required_hold_seconds: float = 3.0,
) -> FeedHoldState:
    if bean_count <= 0 or not in_feed_zone or dt < 0.0:
        return FeedHoldState()

    hold_seconds = state.hold_seconds + dt
    return FeedHoldState(
        hold_seconds=hold_seconds,
        completed=hold_seconds >= required_hold_seconds - 1e-12,
    )


def count_points_in_sphere(
    points: Sequence[Point3D],
    region: SphereRegion = TASK3_BEAN_RECOVERY_REGION,
) -> int:
    return sum(1 for point in points if region.contains(point))


def bean_recovery_score(beans_inside: int, total_beans: int) -> int:
    if total_beans <= 0:
        return 0

    ratio = max(0.0, min(1.0, beans_inside / total_beans))
    if ratio >= 1.0:
        return 4
    if ratio >= 0.9:
        return 3
    if ratio >= 0.8:
        return 2
    return 0


def score_stage4_cleanup(
    object_bounds: Mapping[str, Bounds2D],
    object_z_values: Mapping[str, float],
    object_names: Sequence[str] = DEFAULT_UTENSIL_OBJECTS,
    sink_region: SinkRegion = TASK3_SINK_REGION,
) -> StageScore:
    passed = []
    for name in object_names:
        if name not in object_bounds or name not in object_z_values:
            continue

        bounds = object_bounds[name]
        if (
            bounds.overlaps(sink_region.bounds)
            and object_z_values[name] >= sink_region.tabletop_z
        ):
            passed.append(name)

    return StageScore(
        score=len(passed),
        max_score=len(object_names),
        passed=passed,
        failed=[name for name in object_names if name not in passed],
    )


def _xy(point: tuple[float, float] | Point3D) -> tuple[float, float]:
    if isinstance(point, Point3D):
        return point.x, point.y
    return float(point[0]), float(point[1])


def _distance(a: Point3D, b: Point3D) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return sqrt(dx * dx + dy * dy + dz * dz)
