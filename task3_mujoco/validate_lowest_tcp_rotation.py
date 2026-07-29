#!/usr/bin/env python3
"""Validate that arm rotation is defined about the lowest gripper TCP."""
from __future__ import annotations

from pathlib import Path
import numpy as np
import mujoco
import teleop_keyboard as teleop

ROOT = Path(__file__).resolve().parent
XML = ROOT / "benchmark_task3_mesh_table.xml"
EXPECTED_TCP_IN_GRIPPER = np.array([0.0, -0.000375, 0.163052783], dtype=np.float64)
ROTATION_DAMPING = 0.001


def validate_side(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> dict[str, float]:
    arm = teleop.create_arm(model, data, side)
    base_R = data.xmat[arm.gripper_base_body].reshape(3, 3)
    tcp_local = base_R.T @ (
        data.site_xpos[arm.tcp_site] - data.xpos[arm.gripper_base_body]
    )
    tcp_location_error = float(np.linalg.norm(tcp_local - EXPECTED_TCP_IN_GRIPPER))

    jac_p = np.zeros((3, model.nv), dtype=np.float64)
    jac_r = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacSite(model, data, jac_p, jac_r, arm.tcp_site)
    Jp = jac_p[:, arm.dof_ids]
    Jr = jac_r[:, arm.dof_ids]
    J = np.vstack((Jp, Jr))

    # Unit angular velocity around TCP-local Y, with zero requested linear speed.
    world_omega = data.site_xmat[arm.tcp_site].reshape(3, 3) @ np.array([0.0, 1.0, 0.0])
    requested_twist = np.concatenate((np.zeros(3), world_omega))
    qdot = J.T @ np.linalg.solve(
        J @ J.T + ROTATION_DAMPING**2 * np.eye(6),
        requested_twist,
    )
    tcp_linear_speed = float(np.linalg.norm(Jp @ qdot))
    angular_tracking_error = float(np.linalg.norm(Jr @ qdot - world_omega))

    flange_name = f"{side}_fr3v2_1_link8"
    flange_id = teleop.object_id(model, mujoco.mjtObj.mjOBJ_BODY, flange_name)
    flange_jac_p = np.zeros((3, model.nv), dtype=np.float64)
    flange_jac_r = np.zeros((3, model.nv), dtype=np.float64)
    mujoco.mj_jacBody(model, data, flange_jac_p, flange_jac_r, flange_id)
    flange_linear_speed = float(np.linalg.norm(flange_jac_p[:, arm.dof_ids] @ qdot))

    if tcp_location_error > 2e-6:
        raise AssertionError(f"{side}: TCP location error {tcp_location_error} m")
    if tcp_linear_speed > 1.1e-4:
        raise AssertionError(f"{side}: TCP pivot linear speed {tcp_linear_speed} m/s")
    if angular_tracking_error > 1.1e-4:
        raise AssertionError(f"{side}: angular tracking error {angular_tracking_error} rad/s")
    if flange_linear_speed < 0.05:
        raise AssertionError(f"{side}: flange did not move around TCP ({flange_linear_speed} m/s)")

    return {
        "tcp_location_error_m": tcp_location_error,
        "tcp_pivot_linear_speed_mps": tcp_linear_speed,
        "angular_tracking_error_radps": angular_tracking_error,
        "flange_linear_speed_mps": flange_linear_speed,
    }


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    teleop.apply_ready_pose(model, data)
    mujoco.mj_forward(model, data)

    results = {side: validate_side(model, data, side) for side in ("left", "right")}
    print("Lowest-point TCP rotation validation: PASS")
    for side, values in results.items():
        print(f"{side}:")
        for key, value in values.items():
            print(f"  {key}: {value:.9g}")


if __name__ == "__main__":
    main()
