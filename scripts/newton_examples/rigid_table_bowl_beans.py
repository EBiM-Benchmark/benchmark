# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""Standalone rigid bean container example.

Run from this repository checkout with:

    python scripts/newton_examples/example_rigid_table_bowl_beans.py --viewer gl

The file intentionally lives outside ``newton/`` so it can also be used with a
pip-installed Newton package.
"""

from __future__ import annotations

import math
import os
import time
from pathlib import Path

import newton.examples
import numpy as np
import warp as wp

import newton

BEAN_COLOR = (0.20, 0.12, 0.07)
# BEAN_COLOR = (0.10, 0.06, 0.03)
BEAN_COLLISION_GROUP_BASE = 1000

# Approximate real-world dry bean dimensions.
# Coffee bean: about 10-12 mm long, 6-8 mm wide.
# Black bean: about 12-16 mm long, 7-9 mm wide.
COFFEE_BEAN_RADIUS = 0.0025
COFFEE_BEAN_HALF_HEIGHT = 0.0016
BLACK_BEAN_RADIUS = 0.003
BLACK_BEAN_HALF_HEIGHT = 0.0020
CONTAINER_SIZE = 0.05
CONTAINER_WALL_THICKNESS = 0.004
TABLE_TOP_Z = 0.76
CONTAINER_CLEARANCE = 0.005
BEAN_SPAWN_HEIGHT = 0.10
STARTUP_PAUSE_SECONDS = 2.0
BEAN_SPAWN_SPACING_SCALE = 1.2
RIGID_CONTACT_MAX = 3200000
CONTAINER_MASS = 0.35
BEAN_MASS = 0.0001
BOWL_LINER_SEGMENTS = 24


class Example:
    def __init__(self, viewer, args):
        self.fps = args.fps
        self.frame_dt = 1.0 / self.fps
        self.sim_time = 0.0
        self.sim_substeps = args.substeps
        self.sim_dt = self.frame_dt / self.sim_substeps
        self.viewer = viewer
        self._startup_pause_seconds = args.startup_pause_seconds
        self._startup_pause_start_time = time.perf_counter()

        assets_dir = _resolve_assets_dir(args.assets_dir)
        table_path = assets_dir / args.table_asset
        _require_file(table_path)
        bowl_path = assets_dir / args.bowl_asset if args.use_bowl_usd else None
        if bowl_path is not None:
            _require_file(bowl_path)

        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        builder.default_shape_cfg.ke = args.contact_ke
        builder.default_shape_cfg.kd = args.contact_kd
        builder.default_shape_cfg.kf = args.contact_kf
        builder.default_shape_cfg.mu = args.collider_friction

        table_result = self._add_usd(
            builder,
            table_path,
            translation=args.table_pos,
            floating=False,
            skip_mesh_approximation=args.skip_mesh_approximation,
        )
        table_min_local, table_max_local = _usd_world_bounds(table_path)
        table_offset = np.array(args.table_pos, dtype=float)
        self._add_tabletop_support(
            builder,
            table_min_local + table_offset,
            table_max_local + table_offset,
            args.table_support_thickness,
            args.collider_friction,
        )
        for shape_id in table_result["path_shape_map"].values():
            builder.shape_flags[shape_id] |= int(
                newton.ShapeFlags.COLLIDE_SHAPES
            )

        if args.use_bowl_usd:
            bowl_result = self._add_usd(
                builder,
                bowl_path,
                translation=args.bowl_pos,
                floating=True,
                skip_mesh_approximation=args.skip_mesh_approximation,
            )
            bowl_bodies = list(bowl_result["path_body_map"].values())
            if not bowl_bodies:
                raise ValueError(
                    f"No rigid body was imported from bowl asset: {bowl_path}"
                )
            for shape_id in bowl_result["path_shape_map"].values():
                builder.shape_flags[shape_id] |= int(
                    newton.ShapeFlags.COLLIDE_SHAPES
                )

            bowl_min_local, bowl_max_local = _usd_world_bounds(bowl_path)
            self._add_bowl_bean_liner(
                builder,
                bowl_result,
                bowl_min_local,
                bowl_max_local,
                args.collider_friction,
                args.bean_radius,
            )
            bowl_offset = np.array(args.bowl_pos, dtype=float)
            container_min = bowl_min_local + bowl_offset
            container_max = bowl_max_local + bowl_offset
        else:
            container_center = np.array(
                [
                    args.container_pos[0],
                    args.container_pos[1],
                    (
                        TABLE_TOP_Z
                        + 0.5 * args.container_size
                        + args.container_clearance
                        if args.container_pos[2] is None
                        else args.container_pos[2]
                    ),
                ],
                dtype=float,
            )

            container_min, container_max = self._add_container(
                builder,
                center=container_center,
                size=args.container_size,
                wall_thickness=args.container_wall_thickness,
                friction=args.collider_friction,
                density=args.bowl_density,
                container_mass=args.container_mass,
            )
        container_inner_min = container_min
        container_inner_max = container_max
        container_center_xy = 0.5 * (
            container_inner_min[:2] + container_inner_max[:2]
        )
        container_inner_radius = 0.5 * min(
            container_inner_max[0] - container_inner_min[0],
            container_inner_max[1] - container_inner_min[1],
        )

        self._spawn_beans(
            builder,
            container_center_xy=container_center_xy,
            container_min=container_inner_min,
            container_max=container_inner_max,
            container_inner_radius=container_inner_radius,
            args=args,
        )

        self.model = builder.finalize()
        self.model.set_gravity(args.gravity)

        self.solver = newton.solvers.SolverXPBD(
            self.model, iterations=args.iterations
        )
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        self.contacts = newton.Contacts(
            args.rigid_contact_max, 0, device=self.model.device
        )

        newton.eval_fk(
            self.model, self.model.joint_q, self.model.joint_qd, self.state_0
        )

        self.viewer.set_model(self.model)
        if hasattr(self.viewer, "set_camera"):
            self.viewer.set_camera(
                pos=wp.vec3(0.35, -0.75, 1.35), pitch=-35.0, yaw=120.0
            )

    def _add_usd(
        self,
        builder: newton.ModelBuilder,
        path: Path,
        translation: list[float],
        floating: bool,
        skip_mesh_approximation: bool,
    ) -> dict:
        return builder.add_usd(
            str(path),
            xform=wp.transform(wp.vec3(*translation), wp.quat_identity()),
            floating=floating,
            collapse_fixed_joints=False,
            force_show_colliders=True,
            load_visual_shapes=True,
            skip_mesh_approximation=skip_mesh_approximation,
        )

    def _add_tabletop_support(
        self,
        builder: newton.ModelBuilder,
        table_min: np.ndarray,
        table_max: np.ndarray,
        thickness: float,
        friction: float,
    ) -> None:
        center = 0.5 * (table_min + table_max)
        support_z = table_max[2] - 0.5 * thickness
        builder.add_shape_box(
            body=-1,
            xform=wp.transform(
                wp.vec3(center[0], center[1], support_z), wp.quat_identity()
            ),
            hx=0.5 * (table_max[0] - table_min[0]),
            hy=0.5 * (table_max[1] - table_min[1]),
            hz=0.5 * thickness,
            cfg=newton.ModelBuilder.ShapeConfig(
                mu=friction,
                ke=1.0e6,
                kd=1.0e2,
                is_visible=False,
            ),
        )

    def _add_bowl_bean_liner(
        self,
        builder: newton.ModelBuilder,
        usd_result: dict,
        bowl_min: np.ndarray,
        bowl_max: np.ndarray,
        friction: float,
        bean_radius: float,
    ) -> None:
        """Add hidden primitive colliders for robust bean containment."""

        cfg = newton.ModelBuilder.ShapeConfig(
            density=0.0,
            mu=friction,
            ke=1.0e6,
            kd=1.0e2,
            is_visible=False,
            has_shape_collision=True,
            has_particle_collision=False,
        )

        bowl_bodies = list(usd_result["path_body_map"].values())
        if not bowl_bodies:
            return

        bowl_body = bowl_bodies[0]
        center_xy = 0.5 * (bowl_min[:2] + bowl_max[:2])
        outer_radius = 0.5 * min(
            bowl_max[0] - bowl_min[0], bowl_max[1] - bowl_min[1]
        )
        wall_thickness = max(0.006, 1.5 * bean_radius)
        inner_radius = max(
            bean_radius * 2.5, outer_radius - 2.5 * wall_thickness
        )
        wall_center_radius = inner_radius + 0.5 * wall_thickness

        bottom_thickness = max(0.006, 1.5 * bean_radius)
        bottom_center_z = bowl_min[2] + 0.5 * bottom_thickness
        wall_bottom_z = bowl_min[2] + bottom_thickness
        wall_top_z = bowl_max[2] - 0.006
        wall_half_height = max(
            0.5 * bottom_thickness, 0.5 * (wall_top_z - wall_bottom_z)
        )
        wall_center_z = wall_bottom_z + wall_half_height

        non_bean_shape_count = builder.shape_count

        bottom_shape_id = builder.add_shape_cylinder(
            body=bowl_body,
            xform=wp.transform(
                wp.vec3(center_xy[0], center_xy[1], bottom_center_z),
                wp.quat_identity(),
            ),
            radius=inner_radius,
            half_height=0.5 * bottom_thickness,
            cfg=cfg,
            label="bowl_bean_liner_bottom",
        )
        self._filter_shape_from_existing(
            builder, bottom_shape_id, non_bean_shape_count
        )

        segment_angle = 2.0 * math.pi / BOWL_LINER_SEGMENTS
        segment_half_length = wall_center_radius * math.tan(
            0.5 * segment_angle
        )
        for segment_index in range(BOWL_LINER_SEGMENTS):
            angle = segment_index * segment_angle
            radial = np.array([math.cos(angle), math.sin(angle)])
            position = np.array(
                [
                    center_xy[0] + wall_center_radius * radial[0],
                    center_xy[1] + wall_center_radius * radial[1],
                    wall_center_z,
                ]
            )
            rotation = wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), angle)
            wall_shape_id = builder.add_shape_box(
                body=bowl_body,
                xform=wp.transform(wp.vec3(*position), rotation),
                hx=0.5 * wall_thickness,
                hy=segment_half_length,
                hz=wall_half_height,
                cfg=cfg,
                label=f"bowl_bean_liner_wall_{segment_index}",
            )
            self._filter_shape_from_existing(
                builder, wall_shape_id, non_bean_shape_count
            )

    def _filter_shape_from_existing(
        self,
        builder: newton.ModelBuilder,
        shape_id: int,
        existing_shape_count: int,
    ) -> None:
        for other_shape_id in range(existing_shape_count):
            builder.add_shape_collision_filter_pair(shape_id, other_shape_id)

    def _add_container(
        self,
        builder: newton.ModelBuilder,
        center: np.ndarray,
        size: float,
        wall_thickness: float,
        friction: float,
        density: float,
        container_mass: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        outer_half = 0.5 * size
        inner_half = outer_half - wall_thickness
        if inner_half <= 0.0:
            raise ValueError(
                "Container wall thickness must be smaller than half the container size."
            )

        body = builder.add_link(
            xform=wp.transform(wp.vec3(*center), wp.quat_identity()),
            label="container",
        )
        builder.add_articulation(
            [builder.add_joint_free(body)], label="container"
        )

        bottom_center_z = -outer_half + 0.5 * wall_thickness
        wall_center_z = 0.0
        wall_half_height = outer_half

        cfg = newton.ModelBuilder.ShapeConfig(
            density=density,
            mu=friction,
            ke=1.0e6,
            kd=1.0e2,
        )

        shapes = [
            (
                (0.0, 0.0, bottom_center_z),
                outer_half,
                outer_half,
                0.5 * wall_thickness,
            ),
            (
                (-inner_half - 0.5 * wall_thickness, 0.0, wall_center_z),
                0.5 * wall_thickness,
                outer_half,
                wall_half_height,
            ),
            (
                (inner_half + 0.5 * wall_thickness, 0.0, wall_center_z),
                0.5 * wall_thickness,
                outer_half,
                wall_half_height,
            ),
            (
                (0.0, -inner_half - 0.5 * wall_thickness, wall_center_z),
                inner_half,
                0.5 * wall_thickness,
                wall_half_height,
            ),
            (
                (0.0, inner_half + 0.5 * wall_thickness, wall_center_z),
                inner_half,
                0.5 * wall_thickness,
                wall_half_height,
            ),
        ]

        for position, hx, hy, hz in shapes:
            builder.add_shape_box(
                body=body,
                xform=wp.transform(wp.vec3(*position), wp.quat_identity()),
                hx=hx,
                hy=hy,
                hz=hz,
                cfg=cfg,
            )

            builder.body_mass[body] = container_mass

        inner_min = np.array(
            [
                center[0] - inner_half,
                center[1] - inner_half,
                center[2] - outer_half + wall_thickness,
            ],
            dtype=float,
        )
        inner_max = np.array(
            [
                center[0] + inner_half,
                center[1] + inner_half,
                center[2] + outer_half,
            ],
            dtype=float,
        )
        return inner_min, inner_max

    def _spawn_beans(
        self,
        builder: newton.ModelBuilder,
        container_center_xy: np.ndarray,
        container_min: np.ndarray,
        container_max: np.ndarray,
        container_inner_radius: float,
        args,
    ) -> None:
        rng = np.random.default_rng(args.seed)
        radial_margin = max(
            1.25 * args.bean_radius, 0.60 * args.bean_half_height
        )
        usable_radius = max(
            args.bean_radius,
            container_inner_radius - args.spawn_wall_thickness - radial_margin,
        )
        bean_length = 2.0 * (args.bean_half_height + args.bean_radius)
        layer_height = max(2.4 * args.bean_radius, 0.9 * bean_length)
        spawn_bottom_z = (
            container_max[2] + args.particle_gap + args.bean_radius * 1.5
        )
        spawn_top_z = spawn_bottom_z + args.spawn_height
        if spawn_top_z <= spawn_bottom_z:
            raise ValueError("Bean spawn height must be positive.")

        z_coords = np.arange(
            spawn_bottom_z, spawn_top_z + 0.5 * layer_height, layer_height
        )
        positions: list[np.ndarray] = []
        ring_spacing = BEAN_SPAWN_SPACING_SCALE * max(
            2.8 * args.bean_radius, 0.92 * bean_length
        )
        angular_spacing = BEAN_SPAWN_SPACING_SCALE * max(
            2.6 * args.bean_radius, 0.8 * bean_length
        )

        layer_index = 0
        while len(positions) < args.bean_count:
            z = (
                z_coords[layer_index]
                if layer_index < len(z_coords)
                else spawn_top_z
                + (layer_index - len(z_coords) + 1) * layer_height
            )
            ring_phase = 0.5 * math.pi * (layer_index % 4)

            positions.append(
                np.array(
                    [
                        container_center_xy[0],
                        container_center_xy[1],
                        z,
                    ],
                    dtype=float,
                )
            )
            if len(positions) >= args.bean_count:
                break

            ring_radius = ring_spacing
            while (
                ring_radius <= usable_radius
                and len(positions) < args.bean_count
            ):
                circumference = 2.0 * math.pi * ring_radius
                count_on_ring = max(6, int(circumference / angular_spacing))
                angle_step = 2.0 * math.pi / count_on_ring
                for ring_index in range(count_on_ring):
                    angle = ring_phase + ring_index * angle_step
                    radial_jitter = rng.uniform(
                        -0.08 * ring_spacing, 0.08 * ring_spacing
                    )
                    theta_jitter = rng.uniform(-0.08, 0.08) * angle_step
                    current_radius = min(
                        usable_radius,
                        max(args.bean_radius, ring_radius + radial_jitter),
                    )
                    x = current_radius * math.cos(angle + theta_jitter)
                    y = current_radius * math.sin(angle + theta_jitter)
                    if x * x + y * y > usable_radius * usable_radius:
                        continue
                    positions.append(
                        np.array(
                            [
                                container_center_xy[0] + x,
                                container_center_xy[1] + y,
                                z
                                + rng.uniform(
                                    -0.08 * args.bean_radius,
                                    0.08 * args.bean_radius,
                                ),
                            ],
                            dtype=float,
                        )
                    )
                    if len(positions) >= args.bean_count:
                        break
                ring_radius += ring_spacing
            layer_index += 1

        spawn_count = min(len(positions), args.bean_count)

        bean_cfg = newton.ModelBuilder.ShapeConfig(
            density=args.bean_density,
            mu=args.bean_friction,
            restitution=args.bean_restitution,
            ke=args.contact_ke,
            kd=args.contact_kd,
            kf=args.contact_kf,
        )

        for index, position in enumerate(positions[:spawn_count]):
            roll = rng.uniform(-0.45, 0.45)
            pitch = rng.uniform(-0.45, 0.45)
            yaw = rng.uniform(0.0, 2.0 * math.pi)
            quat = _quat_from_euler_xyz(roll, pitch, yaw)
            body = builder.add_link(
                xform=wp.transform(wp.vec3(*position), quat),
                mass=args.bean_mass,
                label=f"bean_{index}",
            )
            cfg = bean_cfg.copy()
            cfg.collision_group = -(BEAN_COLLISION_GROUP_BASE + index)
            builder.add_shape_capsule(
                body,
                radius=args.bean_radius,
                half_height=args.bean_half_height,
                xform=wp.transform(
                    q=wp.quat_from_axis_angle(
                        wp.vec3(0.0, 1.0, 0.0), 0.5 * wp.pi
                    )
                ),
                cfg=cfg,
                color=BEAN_COLOR,
            )
            builder.add_articulation(
                [builder.add_joint_free(body)], label=f"bean_{index}"
            )

    def simulate(self):
        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)
            self.model.collide(self.state_0, self.contacts)
            self.solver.step(
                self.state_0,
                self.state_1,
                self.control,
                self.contacts,
                self.sim_dt,
            )
            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        if self._startup_pause_seconds > 0.0:
            elapsed = time.perf_counter() - self._startup_pause_start_time
            if elapsed < self._startup_pause_seconds:
                return
        self.simulate()
        self.sim_time += self.frame_dt

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.end_frame()

    def test_final(self):
        newton.examples.test_body_state(
            self.model,
            self.state_0,
            "beans stay above table",
            lambda q, qd: q[2] > -0.05,
            indices=list(range(1, min(self.model.body_count, 301))),
        )

    @staticmethod
    def create_parser():
        parser = newton.examples.create_parser()

        parser.add_argument("--assets-dir", type=str, default=None)
        parser.add_argument(
            "--table-asset", type=str, default="table_edit.usd"
        )
        parser.add_argument("--bowl-asset", type=str, default="bowl.usd")
        parser.add_argument(
            "--table-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0]
        )
        parser.add_argument(
            "--bowl-pos", type=float, nargs=3, default=[0.0, 0.0, 0.77]
        )
        parser.add_argument(
            "--container-pos",
            type=_parse_optional_float,
            nargs=3,
            default=[0.0, 0.0, None],
        )
        parser.add_argument("--use-bowl-usd", action="store_true")
        parser.add_argument("--bowl-density", type=float, default=300.0)
        parser.add_argument(
            "--container-size", type=float, default=CONTAINER_SIZE
        )
        parser.add_argument(
            "--container-wall-thickness",
            type=float,
            default=CONTAINER_WALL_THICKNESS,
        )
        parser.add_argument(
            "--container-clearance", type=float, default=CONTAINER_CLEARANCE
        )
        parser.add_argument(
            "--table-support-thickness", type=float, default=0.01
        )
        parser.add_argument(
            "--use-usd-mesh-approximation",
            action="store_false",
            dest="skip_mesh_approximation",
            default=True,
            help="Use authored USD mesh approximations such as boundingCube instead of the original mesh.",
        )

        parser.add_argument("--bean-count", type=int, default=150)
        parser.add_argument(
            "--bean-radius", type=float, default=BLACK_BEAN_RADIUS
        )
        parser.add_argument(
            "--bean-half-height", type=float, default=BLACK_BEAN_HALF_HEIGHT
        )
        parser.add_argument("--bean-mass", type=float, default=BEAN_MASS)
        parser.add_argument(
            "--container-mass", type=float, default=CONTAINER_MASS
        )
        parser.add_argument("--bean-density", type=float, default=850.0)
        parser.add_argument("--bean-friction", type=float, default=0.55)
        parser.add_argument("--bean-restitution", type=float, default=0.02)
        parser.add_argument(
            "--spawn-wall-thickness", type=float, default=0.016
        )
        parser.add_argument(
            "--spawn-height", type=float, default=BEAN_SPAWN_HEIGHT
        )
        parser.add_argument("--particle-gap", type=float, default=0.006)
        parser.add_argument("--seed", type=int, default=7)
        parser.add_argument(
            "--startup-pause-seconds",
            type=float,
            default=STARTUP_PAUSE_SECONDS,
        )
        parser.add_argument(
            "--rigid-contact-max", type=int, default=RIGID_CONTACT_MAX
        )

        parser.add_argument(
            "--gravity", type=float, nargs=3, default=[0.0, 0.0, -9.81]
        )
        parser.add_argument("--fps", type=float, default=120.0)
        parser.add_argument("--substeps", type=int, default=4)
        parser.add_argument("--iterations", type=int, default=24)
        parser.add_argument("--collider-friction", type=float, default=0.5)
        parser.add_argument("--contact-ke", type=float, default=1.0e6)
        parser.add_argument("--contact-kd", type=float, default=1.0e2)
        parser.add_argument("--contact-kf", type=float, default=1.0e3)
        return parser


def _resolve_assets_dir(raw_assets_dir: str | None) -> Path:
    candidates: list[Path] = []
    if raw_assets_dir:
        candidates.append(Path(raw_assets_dir).expanduser())
    for env_name in ("EBIM_CHALLENGE_ASSETS_DIR", "NEWTON_ASSETS_DIR"):
        env_value = os.environ.get(env_name)
        if env_value:
            candidates.append(Path(env_value).expanduser())

    cwd = Path.cwd()
    script_path = Path(__file__).resolve()
    candidates.extend(
        [
            cwd / "../benchmark/assets",
            cwd / "assets",
            script_path.parents[2] / "assets",
        ]
    )

    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()

    searched = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        f"Could not find asset directory. Searched:\n  {searched}"
    )


def _require_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"Required USD asset does not exist: {path}")


def _parse_optional_float(value: str) -> float | None:
    if value.lower() in {"none", "auto"}:
        return None
    return float(value)


def _usd_world_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise ValueError(f"Could not open USD stage: {path}")

    purposes = [
        UsdGeom.Tokens.default_,
        UsdGeom.Tokens.render,
        UsdGeom.Tokens.proxy,
    ]
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    bound_range = bbox_cache.ComputeWorldBound(
        stage.GetPseudoRoot()
    ).ComputeAlignedRange()
    return np.array(bound_range.GetMin(), dtype=float), np.array(
        bound_range.GetMax(), dtype=float
    )


def _quat_from_euler_xyz(roll: float, pitch: float, yaw: float) -> wp.quat:
    qx = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), roll)
    qy = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), pitch)
    qz = wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), yaw)
    return qz * qy * qx


if __name__ == "__main__":
    parser = Example.create_parser()
    viewer, args = newton.examples.init(parser)
    example = Example(viewer, args)
    newton.examples.run(example, args)
