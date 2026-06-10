# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""MPM particles settling in a small table-top container.

Run from this repository checkout with:

    python scripts/newton_examples/example_mpm_table_container.py --viewer gl

The example combines the USD table setup from ``rigid_table_bowl_beans.py``
with the MPM particle emission and solver flow from the viscous MPM example.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np
import warp as wp

import newton
import newton.examples
from newton.solvers import SolverImplicitMPM


@wp.kernel
def _copy_body_state(
    src_q: wp.array[wp.transform],
    src_qd: wp.array[wp.spatial_vector],
    dst_q: wp.array[wp.transform],
    dst_qd: wp.array[wp.spatial_vector],
):
    body_id = wp.tid()
    dst_q[body_id] = src_q[body_id]
    dst_qd[body_id] = src_qd[body_id]

CONTAINER_SIZE = 0.05
CONTAINER_WALL_THICKNESS = 0.004
TABLE_TOP_Z = 0.76
CONTAINER_CLEARANCE = 0.005
PARTICLE_COLOR = wp.vec3(0.20, 0.12, 0.07)
BOWL_LINER_SEGMENTS = 24


class Example:
    def __init__(self, viewer, args):
        self.fps = args.fps
        self.frame_dt = 1.0 / self.fps
        self.sim_time = 0.0
        self.sim_substeps = args.substeps
        self.sim_dt = self.frame_dt / self.sim_substeps
        self.viewer = viewer
        self.container_body = -1

        assets_dir = _resolve_assets_dir(args.assets_dir)
        table_path = assets_dir / args.table_asset
        _require_file(table_path)
        bowl_path = assets_dir / args.bowl_asset if args.use_bowl_usd else None
        if bowl_path is not None:
            _require_file(bowl_path)

        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)

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
                newton.ShapeFlags.COLLIDE_PARTICLES
            )
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
            for shape_id in bowl_result["path_shape_map"].values():
                builder.shape_flags[shape_id] |= int(
                    newton.ShapeFlags.COLLIDE_PARTICLES
                )
                builder.shape_flags[shape_id] |= int(
                    newton.ShapeFlags.COLLIDE_SHAPES
                )

            bowl_min_local, bowl_max_local = _usd_world_bounds(bowl_path)
            self._add_bowl_particle_liner(
                builder,
                bowl_result,
                bowl_min_local,
                bowl_max_local,
                args.collider_friction,
                args.voxel_size,
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
                mass=args.container_mass,
            )

        particle_builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        SolverImplicitMPM.register_custom_attributes(particle_builder)
        self.emit_particles(particle_builder, container_min, container_max, args)

        self.model = builder.finalize()
        self.particle_model = particle_builder.finalize()
        self.model.set_gravity(args.gravity)
        self.particle_model.set_gravity(args.gravity)

        mpm_options = SolverImplicitMPM.Config()
        for key, value in vars(args).items():
            if hasattr(mpm_options, key):
                setattr(mpm_options, key, value)
            if hasattr(self.particle_model.mpm, key):
                getattr(self.particle_model.mpm, key).fill_(value)

        self.rigid_solver = newton.solvers.SolverXPBD(self.model, iterations=args.rigid_iterations)
        self.mpm_solver = SolverImplicitMPM(self.particle_model, mpm_options)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.particle_state_0 = self.particle_model.state()
        self.particle_state_1 = self.particle_model.state()
        self.particle_state_0.body_q = wp.empty_like(self.state_0.body_q)
        self.particle_state_0.body_qd = wp.empty_like(self.state_0.body_qd)
        self.particle_state_1.body_q = wp.empty_like(self.state_0.body_q)
        self.particle_state_1.body_qd = wp.empty_like(self.state_0.body_qd)
        self._copy_rigid_body_state_to_particles()
        self.control = self.model.control()
        self.contacts = self.model.contacts()
        self.particle_render_colors = wp.full(
            self.particle_model.particle_count,
            value=PARTICLE_COLOR,
            dtype=wp.vec3,
            device=self.particle_model.device,
        )
        self.mpm_solver.setup_collider(
            model=self.model,
            body_mass=wp.zeros_like(self.model.body_mass),
            body_q=self.particle_state_0.body_q,
        )

        self.viewer.show_particles = True
        self.viewer.set_model(self.model)
        if hasattr(self.viewer, "set_camera"):
            self.viewer.set_camera(pos=wp.vec3(0.24, -0.38, 0.92), pitch=-28.0, yaw=135.0)

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

    def _add_bowl_particle_liner(
        self,
        builder: newton.ModelBuilder,
        usd_result: dict,
        bowl_min: np.ndarray,
        bowl_max: np.ndarray,
        friction: float,
        voxel_size: float,
    ) -> None:
        """Add hidden primitive particle colliders to make the USD bowl hold MPM particles."""
        bowl_bodies = list(usd_result["path_body_map"].values())
        if not bowl_bodies:
            raise ValueError("No rigid body was imported from the bowl asset.")

        bowl_body = bowl_bodies[0]
        center_xy = 0.5 * (bowl_min[:2] + bowl_max[:2])
        outer_radius = 0.5 * min(bowl_max[0] - bowl_min[0], bowl_max[1] - bowl_min[1])
        wall_thickness = max(0.006, voxel_size)
        inner_radius = max(2.5 * voxel_size, outer_radius - 2.5 * wall_thickness)
        wall_center_radius = inner_radius + 0.5 * wall_thickness

        bottom_thickness = max(0.006, voxel_size)
        bottom_center_z = bowl_min[2] + 0.5 * bottom_thickness
        wall_bottom_z = bowl_min[2] + bottom_thickness
        wall_top_z = bowl_max[2] - 0.006
        wall_half_height = max(0.5 * bottom_thickness, 0.5 * (wall_top_z - wall_bottom_z))
        wall_center_z = wall_bottom_z + wall_half_height

        cfg = newton.ModelBuilder.ShapeConfig(
            mu=friction,
            density=0.0,
            is_visible=False,
            has_shape_collision=True,
            has_particle_collision=True,
        )

        builder.add_shape_cylinder(
            body=bowl_body,
            xform=wp.transform(
                wp.vec3(center_xy[0], center_xy[1], bottom_center_z),
                wp.quat_identity(),
            ),
            radius=inner_radius,
            half_height=0.5 * bottom_thickness,
            cfg=cfg,
            label="bowl_particle_liner_bottom",
        )

        segment_angle = 2.0 * math.pi / BOWL_LINER_SEGMENTS
        segment_half_length = wall_center_radius * math.tan(0.5 * segment_angle)
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
            builder.add_shape_box(
                body=bowl_body,
                xform=wp.transform(wp.vec3(*position), rotation),
                hx=0.5 * wall_thickness,
                hy=segment_half_length,
                hz=wall_half_height,
                cfg=cfg,
                label=f"bowl_particle_liner_wall_{segment_index}",
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
            xform=wp.transform(wp.vec3(center[0], center[1], support_z), wp.quat_identity()),
            hx=0.5 * (table_max[0] - table_min[0]),
            hy=0.5 * (table_max[1] - table_min[1]),
            hz=0.5 * thickness,
            cfg=newton.ModelBuilder.ShapeConfig(
                mu=friction,
                is_visible=False,
                has_shape_collision=True,
                has_particle_collision=True,
            ),
        )

    def _add_container(
        self,
        builder: newton.ModelBuilder,
        center: np.ndarray,
        size: float,
        wall_thickness: float,
        friction: float,
        mass: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        outer_half = 0.5 * size
        inner_half = outer_half - wall_thickness
        if inner_half <= 0.0:
            raise ValueError("Container wall thickness must be smaller than half the container size.")

        inertia_value = (1.0 / 6.0) * mass * size * size
        self.container_body = builder.add_body(
            xform=wp.transform(wp.vec3(*center), wp.quat_identity()),
            mass=mass,
            inertia=wp.mat33(inertia_value, 0.0, 0.0, 0.0, inertia_value, 0.0, 0.0, 0.0, inertia_value),
            label="dynamic_container",
            lock_inertia=True,
        )

        bottom_center_z = -outer_half + 0.5 * wall_thickness
        wall_center_z = 0.0
        cfg = newton.ModelBuilder.ShapeConfig(
            mu=friction,
            density=0.0,
            has_shape_collision=True,
            has_particle_collision=True,
        )

        shapes = [
            ((0.0, 0.0, bottom_center_z), outer_half, outer_half, 0.5 * wall_thickness),
            ((-inner_half - 0.5 * wall_thickness, 0.0, wall_center_z), 0.5 * wall_thickness, outer_half, outer_half),
            ((inner_half + 0.5 * wall_thickness, 0.0, wall_center_z), 0.5 * wall_thickness, outer_half, outer_half),
            ((0.0, -inner_half - 0.5 * wall_thickness, wall_center_z), inner_half, 0.5 * wall_thickness, outer_half),
            ((0.0, inner_half + 0.5 * wall_thickness, wall_center_z), inner_half, 0.5 * wall_thickness, outer_half),
        ]

        for position, hx, hy, hz in shapes:
            builder.add_shape_box(
                body=self.container_body,
                xform=wp.transform(wp.vec3(*position), wp.quat_identity()),
                hx=hx,
                hy=hy,
                hz=hz,
                cfg=cfg,
            )

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

    def _copy_rigid_body_state_to_particles(self) -> None:
        wp.launch(
            _copy_body_state,
            dim=self.model.body_count,
            inputs=[
                self.state_0.body_q,
                self.state_0.body_qd,
                self.particle_state_0.body_q,
                self.particle_state_0.body_qd,
            ],
            device=self.model.device,
        )
        wp.launch(
            _copy_body_state,
            dim=self.model.body_count,
            inputs=[
                self.state_0.body_q,
                self.state_0.body_qd,
                self.particle_state_1.body_q,
                self.particle_state_1.body_qd,
            ],
            device=self.model.device,
        )

    @staticmethod
    def emit_particles(builder: newton.ModelBuilder, container_min: np.ndarray, container_max: np.ndarray, args) -> None:
        """Fill the container interior with MPM particles on a jittered grid."""
        voxel_size = args.voxel_size
        margin = max(args.particle_margin, voxel_size)
        particle_lo = container_min + np.array([margin, margin, margin], dtype=float)
        particle_hi = np.array(
            [
                container_max[0] - margin,
                container_max[1] - margin,
                min(container_max[2] - margin, container_min[2] + args.fill_height),
            ],
            dtype=float,
        )
        if np.any(particle_hi <= particle_lo):
            raise ValueError("Particle fill region is empty. Reduce voxel size, margin, or wall thickness.")

        if args.particle_spacing is None:
            particle_res = np.array(
                np.ceil(args.particles_per_cell * (particle_hi - particle_lo) / voxel_size),
                dtype=int,
            )
        else:
            particle_res = np.array(np.ceil((particle_hi - particle_lo) / args.particle_spacing), dtype=int)
        particle_res = np.maximum(particle_res, 1)
        cell_size = (particle_hi - particle_lo) / particle_res
        cell_volume = np.prod(cell_size)
        radius = args.particle_radius_scale * np.max(cell_size) * 0.5
        mass = cell_volume * args.density

        px = np.arange(particle_res[0] + 1) * cell_size[0]
        py = np.arange(particle_res[1] + 1) * cell_size[1]
        pz = np.arange(particle_res[2] + 1) * cell_size[2]
        points = np.stack(np.meshgrid(px, py, pz, indexing="ij")).reshape(3, -1).T

        rng = np.random.default_rng(args.seed)
        jitter = args.jitter * np.max(cell_size)
        points += (rng.random(points.shape) - 0.5) * jitter
        points += particle_lo

        inside = np.all((points > particle_lo) & (points < particle_hi), axis=1)
        points = points[inside]

        builder.add_particles(
            pos=points.tolist(),
            vel=np.zeros_like(points).tolist(),
            mass=[mass] * points.shape[0],
            radius=[radius] * points.shape[0],
        )

    def simulate(self):
        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)
            self.model.collide(self.state_0, self.contacts)
            self.rigid_solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
            self._copy_rigid_body_state_to_particles()

            self.mpm_solver.step(self.particle_state_0, self.particle_state_1, None, None, self.sim_dt)
            self.mpm_solver.project_outside(self.particle_state_1, self.particle_state_1, self.sim_dt)
            self.particle_state_0, self.particle_state_1 = self.particle_state_1, self.particle_state_0

    def step(self):
        self.simulate()
        self.sim_time += self.frame_dt

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.log_points(
            "/mpm_particles",
            points=self.particle_state_0.particle_q,
            radii=self.particle_model.particle_radius,
            colors=self.particle_render_colors,
            hidden=not self.viewer.show_particles,
        )
        self.viewer.end_frame()

    def test_final(self):
        voxel_size = self.mpm_solver.voxel_size
        newton.examples.test_particle_state(
            self.particle_state_0,
            "all particles stay above the table",
            lambda q, qd: q[2] > TABLE_TOP_Z - voxel_size,
        )

    @staticmethod
    def create_parser():
        parser = newton.examples.create_parser()

        parser.add_argument("--assets-dir", type=str, default=None)
        parser.add_argument("--table-asset", type=str, default="table_edit.usd")
        parser.add_argument("--bowl-asset", type=str, default="bowl.usd")
        parser.add_argument("--table-pos", type=float, nargs=3, default=[0.0, 0.0, 0.0])
        parser.add_argument("--bowl-pos", type=float, nargs=3, default=[0.0, 0.0, 0.77])
        parser.add_argument("--use-bowl-usd", action="store_true", help="Use the USD bowl as the MPM container instead of the box container.")
        parser.add_argument("--container-pos", type=_parse_optional_float, nargs=3, default=[0.0, 0.0, None])
        parser.add_argument("--container-size", type=float, default=CONTAINER_SIZE)
        parser.add_argument("--container-wall-thickness", type=float, default=CONTAINER_WALL_THICKNESS)
        parser.add_argument("--container-clearance", type=float, default=CONTAINER_CLEARANCE)
        parser.add_argument("--table-support-thickness", type=float, default=0.01)
        parser.add_argument(
            "--use-usd-mesh-approximation",
            action="store_false",
            dest="skip_mesh_approximation",
            default=True,
            help="Use authored USD mesh approximations such as boundingCube instead of the original mesh.",
        )

        parser.add_argument("--density", type=float, default=850.0)
        parser.add_argument("--viscosity", type=float, default=0.0)
        parser.add_argument("--tensile-yield-ratio", "-tyr", type=float, default=0.0)
        parser.add_argument("--friction", "-mu", type=float, default=0.68)
        parser.add_argument("--young-modulus", "-ym", type=float, default=1.0e15)
        parser.add_argument("--poisson-ratio", "-nu", type=float, default=0.3)
        parser.add_argument("--yield-pressure", "-yp", type=float, default=1.0e12)
        parser.add_argument("--yield-stress", "-ys", type=float, default=0.0)
        parser.add_argument("--hardening", type=float, default=0.0)
        parser.add_argument("--dilatancy", type=float, default=0.0)
        parser.add_argument("--damping", type=float, default=0.0)
        parser.add_argument("--critical-fraction", "-cf", type=float, default=0.0)
        parser.add_argument("--air-drag", type=float, default=1.0)
        parser.add_argument("--collider-friction", type=float, default=0.5)
        parser.add_argument("--container-mass", type=float, default=0.35)
        parser.add_argument("--fill-height", type=float, default=0.042)
        parser.add_argument("--particles-per-cell", type=float, default=2.0)
        parser.add_argument("--particle-spacing", type=float, default=0.0035)
        parser.add_argument("--particle-margin", type=float, default=0.004)
        parser.add_argument("--particle-radius-scale", type=float, default=1.0)
        parser.add_argument("--jitter", type=float, default=0.2)
        parser.add_argument("--seed", type=int, default=422)

        parser.add_argument("--gravity", type=float, nargs=3, default=[0.0, 0.0, -9.81])
        parser.add_argument("--fps", type=float, default=240.0)
        parser.add_argument("--substeps", type=int, default=1)
        parser.add_argument("--rigid-iterations", type=int, default=24)
        parser.add_argument("--max-iterations", "-it", type=int, default=80)
        parser.add_argument("--tolerance", "-tol", type=float, default=1.0e-6)
        parser.add_argument("--voxel-size", "-dx", type=float, default=0.008)
        parser.add_argument("--grid-type", "-gt", type=str, default="sparse", choices=["sparse", "fixed", "dense"])
        parser.add_argument("--transfer-scheme", "-ts", type=str, default="apic", choices=["apic", "pic"])
        parser.add_argument("--integration-scheme", "-is", type=str, default="pic", choices=["pic", "gimp"])
        parser.add_argument("--strain-basis", "-sb", type=str, default="P0")
        parser.add_argument("--collider-basis", "-cb", type=str, default="Q1")
        parser.add_argument(
            "--collider-velocity-mode",
            type=str,
            default="backward",
            choices=["forward", "backward"],
        )
        return parser


def _resolve_assets_dir(raw_assets_dir: str | None) -> Path:
    candidates: list[Path] = []
    if raw_assets_dir:
        candidates.append(Path(raw_assets_dir).expanduser())
    for env_name in ("IROS_WORKSHOP_ASSETS_DIR", "NEWTON_ASSETS_DIR"):
        env_value = os.environ.get(env_name)
        if env_value:
            candidates.append(Path(env_value).expanduser())

    cwd = Path.cwd()
    script_path = Path(__file__).resolve()
    candidates.extend(
        [
            cwd / "../IROS_Workshop/assets",
            cwd / "assets",
            script_path.parents[2] / "assets",
        ]
    )

    for candidate in candidates:
        if candidate.is_dir():
            return candidate.resolve()

    searched = "\n  ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f"Could not find asset directory. Searched:\n  {searched}")


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

    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    bound_range = bbox_cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    return np.array(bound_range.GetMin(), dtype=float), np.array(bound_range.GetMax(), dtype=float)


if __name__ == "__main__":
    parser = Example.create_parser()
    viewer, args = newton.examples.init(parser)
    example = Example(viewer, args)
    newton.examples.run(example, args)
