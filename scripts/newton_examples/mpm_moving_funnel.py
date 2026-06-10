# SPDX-FileCopyrightText: Copyright (c) 2026 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

"""Viscous fluid in a laterally oscillating funnel using MPM.

A funnel-shaped mesh collider moves back and forth along the X axis by
``+-0.1 m`` while viscous fluid drains through its aperture.  This is a stress
test for moving MPM mesh colliders and makes particle-wall penetration easy to
observe.
"""

import math

import numpy as np
import warp as wp

import newton
import newton.examples
from newton.solvers import SolverImplicitMPM


@wp.kernel
def _set_kinematic_funnel_motion(
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_id: int,
    center_z: float,
    amplitude: float,
    frequency: float,
    time: float,
):
    omega = 2.0 * wp.pi * frequency
    phase = omega * time
    x = amplitude * wp.sin(phase)
    vx = amplitude * omega * wp.cos(phase)
    body_q[body_id] = wp.transform(wp.vec3(x, 0.0, center_z), wp.quat_identity())
    body_qd[body_id] = wp.spatial_vector(wp.vec3(vx, 0.0, 0.0), wp.vec3(0.0, 0.0, 0.0))


class Example:
    def __init__(self, viewer, options):
        self.fps = options.fps
        self.frame_dt = 1.0 / self.fps

        self.sim_time = 0.0
        self.sim_substeps = options.substeps
        self.sim_dt = self.frame_dt / self.sim_substeps

        self.funnel_offset_z = options.funnel_offset_z
        self.funnel_height = options.funnel_height
        self.funnel_top_radius = options.funnel_top_radius
        self.funnel_aperture_radius = options.funnel_aperture / 2.0
        self.wall_thickness = options.wall_thickness
        self.wall_tolerance = options.wall_tolerance
        self.motion_amplitude = options.motion_amplitude
        self.motion_frequency = options.motion_frequency
        self.funnel_center_z = options.funnel_offset_z + 0.5 * options.funnel_height
        self.max_wall_penetration = 0

        self.viewer = viewer
        builder = newton.ModelBuilder()

        SolverImplicitMPM.register_custom_attributes(builder)

        self.funnel_body = builder.add_body(
            xform=wp.transform(wp.vec3(0.0, 0.0, self.funnel_center_z), wp.quat_identity()),
            mass=0.0,
            is_kinematic=True,
            label="moving_funnel",
        )

        # Create funnel mesh collider in body-local coordinates.
        vertices, indices = Example.create_funnel_mesh(
            aperture_radius=self.funnel_aperture_radius,
            top_radius=options.funnel_top_radius,
            height=options.funnel_height,
            thickness=options.wall_thickness,
            num_segments=options.funnel_segments,
        )
        mesh = newton.Mesh(vertices, indices, compute_inertia=False, is_solid=False)
        builder.add_shape_mesh(
            body=self.funnel_body,
            mesh=mesh,
            cfg=newton.ModelBuilder.ShapeConfig(
                mu=options.funnel_friction,
                has_particle_collision=True,
            ),
        )

        # Fill funnel with particles in the initial funnel position.
        Example.emit_particles(builder, options)

        builder.add_ground_plane(cfg=newton.ModelBuilder.ShapeConfig(mu=options.ground_friction))

        self.model = builder.finalize()
        self.model.set_gravity(options.gravity)

        # Set per-particle material properties.
        self.model.mpm.viscosity.fill_(options.viscosity)
        self.model.mpm.tensile_yield_ratio.fill_(options.tensile_yield_ratio)
        self.model.mpm.friction.fill_(options.friction)

        mpm_options = SolverImplicitMPM.Config()
        mpm_options.voxel_size = options.voxel_size
        mpm_options.tolerance = options.tolerance
        mpm_options.max_iterations = options.max_iterations
        mpm_options.strain_basis = options.strain_basis
        mpm_options.collider_basis = options.collider_basis
        mpm_options.collider_velocity_mode = options.collider_velocity_mode

        self.solver = SolverImplicitMPM(self.model, mpm_options)

        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self._update_funnel_motion(self.state_0, 0.0)
        self._update_funnel_motion(self.state_1, 0.0)

        # Treat the funnel as a kinematic dynamic collider so its body_q/body_qd
        # are used by the MPM rasterizer every step.
        self.solver.setup_collider(
            collider_body_ids=[-1, self.funnel_body],
            collider_margins=[0.0, options.collider_margin],
            collider_projection_threshold=[options.collider_projection_threshold, options.collider_projection_threshold],
            body_mass=wp.zeros_like(self.model.body_mass),
            body_q=self.state_0.body_q,
        )

        self.viewer.show_particles = True
        self.viewer.set_model(self.model)
        if hasattr(self.viewer, "set_camera"):
            self.viewer.set_camera(pos=wp.vec3(0.45, -0.35, 0.30), pitch=-20.0, yaw=140.0)

    def _update_funnel_motion(self, state: newton.State, time: float):
        wp.launch(
            _set_kinematic_funnel_motion,
            dim=1,
            inputs=[
                state.body_q,
                state.body_qd,
                self.funnel_body,
                self.funnel_center_z,
                self.motion_amplitude,
                self.motion_frequency,
                time,
            ],
            device=self.model.device,
        )

    def simulate(self):
        for _ in range(self.sim_substeps):
            step_start = self.sim_time
            step_end = step_start + self.sim_dt
            self._update_funnel_motion(self.state_0, step_start)
            self._update_funnel_motion(self.state_1, step_end)
            self.solver.step(self.state_0, self.state_1, None, None, self.sim_dt)
            self.solver.project_outside(self.state_1, self.state_1, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
            self.sim_time = step_end

    def step(self):
        self.simulate()

    def test_post_step(self):
        self.max_wall_penetration = max(self.max_wall_penetration, self.count_wall_penetrations())

    def test_final(self):
        voxel_size = self.solver.voxel_size
        newton.examples.test_particle_state(
            self.state_0,
            "all particles are above the ground",
            lambda q, qd: q[2] > -voxel_size,
        )
        if self.max_wall_penetration > 0:
            raise ValueError(f"Detected {self.max_wall_penetration} particles inside the moving funnel wall")

    def count_wall_penetrations(self) -> int:
        """Count particles that are inside the funnel wall material."""
        positions = self.state_0.particle_q.numpy()
        time = self.sim_time
        funnel_x = self.motion_amplitude * math.sin(2.0 * math.pi * self.motion_frequency * time)
        local = positions - np.array([funnel_x, 0.0, self.funnel_center_z], dtype=float)

        radial = np.linalg.norm(local[:, :2], axis=1)
        z = local[:, 2]
        z_frac = np.clip((z + 0.5 * self.funnel_height) / self.funnel_height, 0.0, 1.0)
        inner_radius = self.funnel_aperture_radius + z_frac * (self.funnel_top_radius - self.funnel_aperture_radius)
        inside_height = (z >= -0.5 * self.funnel_height) & (z <= 0.5 * self.funnel_height)
        inside_wall = (radial >= inner_radius - self.wall_tolerance) & (
            radial <= inner_radius + self.wall_thickness + self.wall_tolerance
        )
        return int(np.count_nonzero(inside_height & inside_wall))

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.end_frame()

    @staticmethod
    def create_parser():
        parser = newton.examples.create_parser()

        # Scene configuration.
        parser.add_argument("--funnel-aperture", type=float, default=0.02, help="Diameter of the narrow opening [m]")
        parser.add_argument("--funnel-top-radius", type=float, default=0.05, help="Radius of the wide opening [m]")
        parser.add_argument("--funnel-height", type=float, default=0.2, help="Vertical extent of the funnel [m]")
        parser.add_argument("--funnel-offset-z", type=float, default=0.2, help="Z position of the funnel bottom [m]")
        parser.add_argument("--wall-thickness", type=float, default=0.005, help="Radial wall thickness [m]")
        parser.add_argument("--funnel-segments", type=int, default=64)
        parser.add_argument("--gravity", type=float, nargs=3, default=[0, 0, -10])
        parser.add_argument("--fps", type=float, default=240.0)
        parser.add_argument("--substeps", type=int, default=1)

        # Funnel motion: x(t) = amplitude * sin(2*pi*frequency*t).
        parser.add_argument("--motion-amplitude", type=float, default=0.1, help="Funnel oscillation amplitude [m]")
        parser.add_argument("--motion-frequency", type=float, default=1.0, help="Funnel oscillation frequency [Hz]")
        parser.add_argument("--wall-tolerance", type=float, default=0.002, help="Tolerance for wall penetration test [m]")

        # Material parameters.
        parser.add_argument("--density", type=float, default=1000.0)
        parser.add_argument("--viscosity", type=float, default=50.0)
        parser.add_argument("--tensile-yield-ratio", "-tyr", type=float, default=1.0)
        parser.add_argument("--friction", "-mu", type=float, default=0.0)
        parser.add_argument("--ground-friction", type=float, default=0.5)
        parser.add_argument("--funnel-friction", type=float, default=0.0)

        # Solver parameters.
        parser.add_argument("--max-iterations", "-it", type=int, default=250)
        parser.add_argument("--tolerance", "-tol", type=float, default=1.0e-6)
        parser.add_argument("--voxel-size", "-dx", type=float, default=0.005)
        parser.add_argument("--strain-basis", "-sb", type=str, default="P0")
        parser.add_argument("--collider-basis", "-cb", type=str, default="S2")
        parser.add_argument("--collider-margin", type=float, default=0.0)
        parser.add_argument("--collider-projection-threshold", type=float, default=0.015)
        parser.add_argument(
            "--collider-velocity-mode",
            type=str,
            default="backward",
            choices=["forward", "backward"],
        )

        return parser

    @staticmethod
    def create_funnel_mesh(aperture_radius, top_radius, height, thickness=0.005, num_segments=64):
        """Generate a thick-walled funnel mesh centered about the body origin."""
        theta = np.linspace(0.0, 2.0 * np.pi, num_segments, endpoint=False)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        n = num_segments
        z_bottom = -0.5 * height
        z_top = 0.5 * height

        def ring(radius, z):
            return np.column_stack([radius * cos_t, radius * sin_t, np.full(n, z)])

        vertices = np.vstack(
            [
                ring(aperture_radius, z_bottom),
                ring(top_radius, z_top),
                ring(top_radius + thickness, z_top),
                ring(aperture_radius + thickness, z_bottom),
            ]
        ).astype(np.float32)

        indices = []
        for i in range(n):
            j = (i + 1) % n
            r0_i, r0_j = i, j
            r1_i, r1_j = i + n, j + n
            r2_i, r2_j = i + 2 * n, j + 2 * n
            r3_i, r3_j = i + 3 * n, j + 3 * n

            # Inner wall normals face inward, toward the fluid.
            indices.extend([r0_i, r1_i, r0_j])
            indices.extend([r0_j, r1_i, r1_j])

            # Outer wall normals face outward.
            indices.extend([r3_i, r3_j, r2_i])
            indices.extend([r2_i, r3_j, r2_j])

            # Top rim normals face up.
            indices.extend([r1_i, r2_i, r1_j])
            indices.extend([r1_j, r2_i, r2_j])

            # Bottom rim normals face down.
            indices.extend([r3_i, r0_i, r3_j])
            indices.extend([r3_j, r0_i, r0_j])

        return vertices, np.array(indices, dtype=np.int32)

    @staticmethod
    def emit_particles(builder: newton.ModelBuilder, args):
        """Fill the initial funnel interior with particles on a jittered grid."""
        voxel_size = args.voxel_size
        density = args.density
        particles_per_cell = 3.0

        aperture_radius = args.funnel_aperture / 2.0
        top_radius = args.funnel_top_radius
        height = args.funnel_height
        z_offset = args.funnel_offset_z

        particle_lo = np.array([-top_radius, -top_radius, z_offset])
        particle_hi = np.array([top_radius, top_radius, z_offset + height])

        particle_res = np.array(
            np.ceil(particles_per_cell * (particle_hi - particle_lo) / voxel_size),
            dtype=int,
        )

        cell_size = (particle_hi - particle_lo) / particle_res
        cell_volume = np.prod(cell_size)
        radius = np.max(cell_size) * 0.5
        mass = cell_volume * density

        dim_x = particle_res[0] + 1
        dim_y = particle_res[1] + 1
        dim_z = particle_res[2] + 1

        px = np.arange(dim_x) * cell_size[0]
        py = np.arange(dim_y) * cell_size[1]
        pz = np.arange(dim_z) * cell_size[2]
        points = np.stack(np.meshgrid(px, py, pz)).reshape(3, -1).T

        jitter = 2.0 * np.max(cell_size)
        rng = np.random.default_rng(422)
        points += (rng.random(points.shape) - 0.5) * jitter
        points += particle_lo

        margin = voxel_size
        z_frac = np.clip((points[:, 2] - z_offset) / height, 0.0, 1.0)
        r_max = aperture_radius + z_frac * (top_radius - aperture_radius) - margin
        r_xy = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
        inside = (r_xy < r_max) & (points[:, 2] > z_offset + margin) & (points[:, 2] < z_offset + height - margin)
        points = points[inside]

        builder.add_particles(
            pos=points.tolist(),
            vel=np.zeros_like(points).tolist(),
            mass=[mass] * points.shape[0],
            radius=[radius] * points.shape[0],
        )


if __name__ == "__main__":
    parser = Example.create_parser()

    viewer, args = newton.examples.init(parser)

    example = Example(viewer, args)

    newton.examples.run(example, args)
