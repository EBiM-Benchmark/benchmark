# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

import importlib
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

TASK3_ROOT = Path(__file__).resolve().parents[1]
COMMON_DIR = TASK3_ROOT / "scripts" / "common"
sys.path.insert(0, str(COMMON_DIR))


class FakeAction:
    def __init__(self, positions, indices):
        self.joint_positions = positions
        self.joint_indices = indices


class ScriptedSolver:
    def __init__(self, *responses):
        self.responses = list(responses)
        self.calls = []

    def compute_inverse_kinematics(self, position, orientation):
        self.calls.append((position, orientation))
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return response


class IsaacLikeSolver(ScriptedSolver):
    def compute_inverse_kinematics(self, position, orientation):
        position.astype(np.float64)
        if orientation is not None:
            orientation.astype(np.float64)
        return super().compute_inverse_kinematics(position, orientation)


class FakeRawLulaSolver:
    def __init__(self):
        self.base_pose_calls = []

    def set_robot_base_pose(self, position, orientation):
        position.astype(np.float64)
        orientation.astype(np.float64)
        self.base_pose_calls.append((position.copy(), orientation.copy()))


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


def _joint_names():
    return [
        "unowned",
        *(f"right_fr3v2_joint{i}" for i in range(1, 8)),
        "franka_spine_vertical_joint",
        *(f"left_fr3v2_joint{i}" for i in range(1, 8)),
    ]


def _action(joint_names, prefix, values):
    indices = [
        joint_names.index(f"{prefix}_fr3v2_joint{i}") for i in range(7, 0, -1)
    ]
    return FakeAction(list(reversed(values)), indices)


def test_module_imports_without_loading_isaac_sim():
    before = {name for name in sys.modules if name.startswith("isaacsim")}

    module = importlib.import_module("dual_arm_lula")

    assert module.LEFT_ARM_JOINTS[0] == "left_fr3v2_joint1"
    assert {
        name for name in sys.modules if name.startswith("isaacsim")
    } == before


def test_independent_successes_return_seven_targets_indexed_by_joint_name():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    left_values = [0.1 * i for i in range(1, 8)]
    right_values = [-0.1 * i for i in range(1, 8)]
    controller = DualArmLulaIK(
        ScriptedSolver((_action(names, "left", left_values), True)),
        ScriptedSolver((_action(names, "right", right_values), True)),
        names,
    )

    result = controller.solve(
        left_position=(0.5, 0.2, 0.8),
        right_position=(0.5, -0.2, 0.8),
        left_orientation=(1.0, 0.0, 0.0, 0.0),
        right_orientation=(1.0, 0.0, 0.0, 0.0),
    )

    assert result.left_succeeded is True
    assert result.right_succeeded is True
    assert result.left == pytest.approx(
        {f"left_fr3v2_joint{i}": left_values[i - 1] for i in range(1, 8)}
    )
    assert result.right == pytest.approx(
        {f"right_fr3v2_joint{i}": right_values[i - 1] for i in range(1, 8)}
    )
    assert result.combined == {**result.left, **result.right}


def test_solver_receives_float64_arrays_and_normalized_quaternions():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    left_solver = IsaacLikeSolver((_action(names, "left", [0.1] * 7), True))
    right_solver = IsaacLikeSolver((_action(names, "right", [0.2] * 7), True))
    controller = DualArmLulaIK(left_solver, right_solver, names)

    controller.solve(
        (0.5, 0.2, 0.8),
        (0.5, -0.2, 0.8),
        left_orientation=(2.0, 0.0, 0.0, 0.0),
        right_orientation=None,
    )

    left_position, left_orientation = left_solver.calls[0]
    right_position, right_orientation = right_solver.calls[0]
    assert left_position.dtype == np.float64
    assert right_position.dtype == np.float64
    assert left_orientation.dtype == np.float64
    assert left_orientation == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert right_orientation is None


@pytest.mark.parametrize(
    "position, orientation",
    [
        ((0.0, 0.0), None),
        ((0.0, 0.0, 0.0, 0.0), None),
        ((0.0, math.nan, 0.0), None),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (math.inf, 0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
    ],
)
def test_invalid_left_target_does_not_prevent_valid_right_target(
    position, orientation
):
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    controller = DualArmLulaIK(
        ScriptedSolver(),
        IsaacLikeSolver((_action(names, "right", [0.3] * 7), True)),
        names,
    )

    result = controller.solve(
        position,
        (0.5, -0.2, 0.8),
        left_orientation=orientation,
    )

    assert result.left_succeeded is False
    assert result.right_succeeded is True


def test_result_mappings_are_immutable_snapshots():
    from dual_arm_lula import DualArmIKResult

    source = {"left_fr3v2_joint1": 0.1}
    result = DualArmIKResult(source, {}, True, False)
    source["left_fr3v2_joint1"] = 9.9

    assert result.left["left_fr3v2_joint1"] == 0.1
    with pytest.raises(TypeError):
        result.left["left_fr3v2_joint1"] = 2.0


def test_failed_arm_retains_solution_without_blocking_other_arm():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    old_left = [0.2] * 7
    controller = DualArmLulaIK(
        ScriptedSolver(
            (_action(names, "left", old_left), True),
            (FakeAction(None, None), False),
        ),
        ScriptedSolver(
            (_action(names, "right", [0.3] * 7), True),
            (_action(names, "right", [0.4] * 7), True),
        ),
        names,
    )
    controller.solve((0, 0, 0), (0, 0, 0))

    result = controller.solve((1, 0, 0), (1, 0, 0))

    assert result.left_succeeded is False
    assert list(result.left.values()) == pytest.approx(old_left)
    assert result.right_succeeded is True
    assert list(result.right.values()) == pytest.approx([0.4] * 7)


@pytest.mark.parametrize(
    "bad_action",
    [
        FakeAction([0.1] * 6, list(range(9, 15))),
        FakeAction([0.1] * 6 + [math.nan], list(range(9, 16))),
        FakeAction([0.1] * 7, [9, 10, 11, 12, 13, 14, 14]),
    ],
    ids=("wrong-length", "non-finite", "duplicate-index"),
)
def test_invalid_arm_output_is_rejected_without_suppressing_other_arm(
    bad_action,
):
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    controller = DualArmLulaIK(
        ScriptedSolver((bad_action, True)),
        ScriptedSolver((_action(names, "right", [0.6] * 7), True)),
        names,
    )

    result = controller.solve((0, 0, 0), (0, 0, 0))

    assert result.left_succeeded is False
    assert result.left == {}
    assert result.right_succeeded is True
    assert list(result.right.values()) == pytest.approx([0.6] * 7)


def test_solver_exception_does_not_prevent_other_arm_from_solving():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    controller = DualArmLulaIK(
        ScriptedSolver(RuntimeError("left solver failed")),
        ScriptedSolver((_action(names, "right", [0.7] * 7), True)),
        names,
    )

    result = controller.solve((0, 0, 0), (0, 0, 0))

    assert result.left_succeeded is False
    assert result.right_succeeded is True
    assert list(result.right.values()) == pytest.approx([0.7] * 7)


def test_repeated_failures_warn_at_most_once_per_interval(caplog):
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    clock = FakeClock()
    failure = (FakeAction(None, None), False)
    controller = DualArmLulaIK(
        ScriptedSolver(failure, failure, failure, failure),
        ScriptedSolver(*[(_action(names, "right", [0.1] * 7), True)] * 4),
        names,
        monotonic=clock,
        warning_interval=1.0,
    )

    for now in (0.0, 0.1, 0.9, 1.0):
        clock.now = now
        controller.solve((0, 0, 0), (0, 0, 0))

    left_warnings = [
        record
        for record in caplog.records
        if "Left arm IK failed" in record.message
    ]
    assert len(left_warnings) == 2


def test_warning_rate_limits_are_independent_per_arm(caplog):
    from dual_arm_lula import DualArmLulaIK

    failure = (FakeAction(None, None), False)
    clock = FakeClock(4.0)
    controller = DualArmLulaIK(
        ScriptedSolver(failure, failure),
        ScriptedSolver(failure, failure),
        _joint_names(),
        monotonic=clock,
        warning_interval=1.0,
    )

    controller.solve((0, 0, 0), (0, 0, 0))
    clock.now = 4.1
    controller.solve((0, 0, 0), (0, 0, 0))

    messages = [record.message for record in caplog.records]
    assert sum("Left arm IK failed" in message for message in messages) == 1
    assert sum("Right arm IK failed" in message for message in messages) == 1


def test_spine_height_is_removed_from_both_world_target_z_values():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    left_solver = ScriptedSolver((_action(names, "left", [0.1] * 7), True))
    right_solver = ScriptedSolver((_action(names, "right", [0.2] * 7), True))
    controller = DualArmLulaIK(left_solver, right_solver, names)

    controller.solve(
        (0.5, 0.2, 1.2),
        (0.6, -0.2, 1.1),
        spine_position=0.35,
    )

    assert left_solver.calls[0][0] == pytest.approx((0.5, 0.2, 0.85))
    assert right_solver.calls[0][0] == pytest.approx((0.6, -0.2, 0.75))


def test_spine_offset_follows_tilted_robot_base_orientation():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    left_solver = ScriptedSolver((_action(names, "left", [0.1] * 7), True))
    right_solver = ScriptedSolver((_action(names, "right", [0.2] * 7), True))
    controller = DualArmLulaIK(left_solver, right_solver, names)
    half_sqrt_two = math.sqrt(0.5)

    controller.solve(
        (1.0, 0.2, 1.2),
        (0.9, -0.2, 1.1),
        spine_position=0.35,
        base_orientation_wxyz=(half_sqrt_two, 0.0, half_sqrt_two, 0.0),
    )

    assert left_solver.calls[0][0] == pytest.approx((0.65, 0.2, 1.2))
    assert right_solver.calls[0][0] == pytest.approx((0.55, -0.2, 1.1))


def test_raw_lula_base_poses_update_each_solve_even_when_one_arm_fails():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    failure = (FakeAction(None, None), False)
    left_raw = FakeRawLulaSolver()
    right_raw = FakeRawLulaSolver()
    controller = DualArmLulaIK(
        ScriptedSolver(failure, (_action(names, "left", [0.1] * 7), True)),
        ScriptedSolver(
            (_action(names, "right", [0.2] * 7), True),
            (_action(names, "right", [0.3] * 7), True),
        ),
        names,
        left_lula_solver=left_raw,
        right_lula_solver=right_raw,
    )
    half_sqrt_two = math.sqrt(0.5)

    controller.solve(
        (1.0, 0.2, 1.2),
        (0.9, -0.2, 1.1),
        base_position=(1.0, 2.0, 0.3),
        base_orientation_wxyz=(
            2.0 * half_sqrt_two,
            0.0,
            0.0,
            2.0 * half_sqrt_two,
        ),
    )
    controller.solve(
        (1.1, 0.2, 1.2),
        (1.0, -0.2, 1.1),
        base_position=(1.5, 2.5, 0.3),
        base_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
    )

    for raw in (left_raw, right_raw):
        assert len(raw.base_pose_calls) == 2
        first_position, first_orientation = raw.base_pose_calls[0]
        second_position, second_orientation = raw.base_pose_calls[1]
        assert first_position.dtype == np.float64
        assert first_orientation.dtype == np.float64
        assert first_position == pytest.approx((1.0, 2.0, 0.3))
        assert first_orientation == pytest.approx(
            (half_sqrt_two, 0.0, 0.0, half_sqrt_two)
        )
        assert second_position == pytest.approx((1.5, 2.5, 0.3))
        assert second_orientation == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_shared_raw_lula_solver_receives_one_base_update_per_solve():
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    shared_raw = FakeRawLulaSolver()
    controller = DualArmLulaIK(
        ScriptedSolver((_action(names, "left", [0.1] * 7), True)),
        ScriptedSolver((_action(names, "right", [0.2] * 7), True)),
        names,
        left_lula_solver=shared_raw,
        right_lula_solver=shared_raw,
    )

    controller.solve((0, 0, 0), (0, 0, 0), base_position=(1, 2, 3))

    assert len(shared_raw.base_pose_calls) == 1


@pytest.mark.parametrize("base_position", [(0.0, 0.0), (0.0, math.nan, 0.0)])
def test_base_position_must_contain_three_finite_values(base_position):
    from dual_arm_lula import DualArmLulaIK

    controller = DualArmLulaIK(
        ScriptedSolver(), ScriptedSolver(), _joint_names()
    )

    with pytest.raises(
        ValueError, match="base_position must contain 3 finite"
    ):
        controller.solve((0, 0, 0), (0, 0, 0), base_position=base_position)


@pytest.mark.parametrize("spine", [math.nan, math.inf, -math.inf, "invalid"])
def test_spine_height_must_be_a_finite_scalar(spine):
    from dual_arm_lula import DualArmLulaIK

    names = _joint_names()
    controller = DualArmLulaIK(ScriptedSolver(), ScriptedSolver(), names)

    with pytest.raises(ValueError, match="spine_position must be finite"):
        controller.solve((0, 0, 1), (0, 0, 1), spine_position=spine)


def test_factory_resolves_project_paths_and_builds_two_isaac_sim_solvers(
    tmp_path,
):
    from dual_arm_lula import create_dual_arm_lula

    config_dir = tmp_path / "assets" / "lula" / "mobile_fr3_duo"
    config_dir.mkdir(parents=True)
    (config_dir / "left_arm_description.yaml").touch()
    (config_dir / "right_arm_description.yaml").touch()
    urdf = config_dir / "mobile_fr3_duo_v0_2_franka_hand.urdf"
    urdf.touch()
    lula_calls = []
    wrapper_calls = []

    class FakeLulaSolver:
        def __init__(self, description_path, urdf_path):
            lula_calls.append((description_path, urdf_path))

    class FakeArticulationSolver:
        def __init__(self, articulation, lula_solver, end_effector):
            wrapper_calls.append((articulation, lula_solver, end_effector))

    articulation = object()
    controller = create_dual_arm_lula(
        articulation,
        joint_names=_joint_names(),
        project_root=tmp_path,
        lula_solver_cls=FakeLulaSolver,
        articulation_solver_cls=FakeArticulationSolver,
    )

    assert controller is not None
    assert lula_calls == [
        (str(config_dir / "left_arm_description.yaml"), str(urdf)),
        (str(config_dir / "right_arm_description.yaml"), str(urdf)),
    ]
    assert [call[2] for call in wrapper_calls] == [
        "left_fr3v2_hand_tcp",
        "right_fr3v2_hand_tcp",
    ]


def test_adapter_reads_current_end_effector_poses_as_normalized_wxyz():
    from dual_arm_lula import DualArmLulaIK

    class PoseSolver:
        def __init__(self, position, rotation):
            self.pose = (np.array(position), np.array(rotation))

        def compute_end_effector_pose(self):
            return self.pose

    left = PoseSolver(
        (0.5, 0.2, 0.8),
        ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    )
    right = PoseSolver(
        (0.5, -0.2, 0.8),
        ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0)),
    )
    controller = DualArmLulaIK(left, right, _joint_names())

    left_pose, right_pose = controller.current_end_effector_poses()

    half = math.sqrt(0.5)
    assert left_pose[0] == pytest.approx((0.5, 0.2, 0.8))
    assert left_pose[1] == pytest.approx((half, 0.0, 0.0, half))
    assert right_pose[0] == pytest.approx((0.5, -0.2, 0.8))
    assert right_pose[1] == pytest.approx((0.0, 1.0, 0.0, 0.0))
    assert np.linalg.norm(left_pose[1]) == pytest.approx(1.0)


def test_raw_arm_bridge_uses_numpy_warm_start_in_exact_arm_order():
    from dual_arm_lula import LEFT_ARM_JOINTS, RawLulaArmSolver

    names = _joint_names()
    current = np.arange(len(names), dtype=np.float32)

    class RawSolver:
        def __init__(self):
            self.calls = []

        def compute_inverse_kinematics(self, *args):
            self.calls.append(args)
            return np.arange(7, dtype=np.float64) + 0.25, True

    raw = RawSolver()
    bridge = RawLulaArmSolver(
        raw, "left_fr3v2_hand_tcp", names, LEFT_ARM_JOINTS, lambda: current
    )

    action, succeeded = bridge.compute_inverse_kinematics(
        (0.5, 0.2, 0.8), (1.0, 0.0, 0.0, 0.0)
    )

    frame, position, orientation, warm_start = raw.calls[0]
    expected_indices = [names.index(name) for name in LEFT_ARM_JOINTS]
    assert succeeded is True
    assert frame == "left_fr3v2_hand_tcp"
    assert isinstance(warm_start, np.ndarray)
    assert warm_start.dtype == np.float64
    assert warm_start == pytest.approx(current[expected_indices])
    assert position.dtype == np.float64
    assert orientation.dtype == np.float64
    assert action.joint_indices.tolist() == expected_indices
    assert action.joint_positions == pytest.approx(np.arange(7) + 0.25)


def test_raw_arm_bridge_forward_kinematics_uses_same_warm_start_order():
    from dual_arm_lula import RIGHT_ARM_JOINTS, RawLulaArmSolver

    names = _joint_names()
    current = np.arange(len(names), dtype=np.float64)

    class RawSolver:
        def __init__(self):
            self.calls = []

        def compute_forward_kinematics(self, frame, warm_start):
            self.calls.append((frame, warm_start))
            return np.array((1.0, 2.0, 3.0)), np.eye(3)

    raw = RawSolver()
    bridge = RawLulaArmSolver(
        raw, "right_fr3v2_hand_tcp", names, RIGHT_ARM_JOINTS, lambda: current
    )

    position, rotation = bridge.compute_end_effector_pose()

    expected = current[[names.index(name) for name in RIGHT_ARM_JOINTS]]
    assert raw.calls[0][0] == "right_fr3v2_hand_tcp"
    assert raw.calls[0][1] == pytest.approx(expected)
    assert position == pytest.approx((1.0, 2.0, 3.0))
    assert rotation == pytest.approx(np.eye(3))


def test_current_fk_synchronizes_spawn_pose_and_adds_tilted_spine_offset():
    from dual_arm_lula import DualArmLulaIK

    class PoseSolver:
        def compute_end_effector_pose(self):
            return np.array((5.0, 6.0, 1.0)), np.eye(3)

    left_raw = FakeRawLulaSolver()
    right_raw = FakeRawLulaSolver()
    controller = DualArmLulaIK(
        PoseSolver(),
        PoseSolver(),
        _joint_names(),
        left_lula_solver=left_raw,
        right_lula_solver=right_raw,
    )
    half = math.sqrt(0.5)

    left, right = controller.current_end_effector_poses(
        base_position=(4.0, 5.0, 0.2),
        base_orientation_wxyz=(half, 0.0, half, 0.0),
        spine_position=0.4,
    )

    assert left_raw.base_pose_calls[0][0] == pytest.approx((4.0, 5.0, 0.2))
    assert right_raw.base_pose_calls[0][1] == pytest.approx(
        (half, 0.0, half, 0.0)
    )
    assert left[0] == pytest.approx((5.4, 6.0, 1.0))
    assert right[0] == pytest.approx((5.4, 6.0, 1.0))


def test_repository_lula_configs_match_current_franka_hand_urdf():
    config_dir = TASK3_ROOT / "assets" / "lula" / "mobile_fr3_duo"

    urdf_path = config_dir / "mobile_fr3_duo_v0_2_franka_hand.urdf"
    root_element = ET.parse(urdf_path).getroot()
    joint_names = {
        joint.attrib["name"] for joint in root_element.findall("joint")
    }
    link_names = {link.attrib["name"] for link in root_element.findall("link")}

    for side in ("left", "right"):
        config_path = config_dir / f"{side}_arm_description.yaml"
        text = config_path.read_text()
        config = yaml.safe_load(text)
        defaults = config["default_q"]
        assert defaults == [0.0, -1.5, 0.0, -2.2, 0.0, 1.5, 0.785]
        for index in range(1, 8):
            joint = f"{side}_fr3v2_joint{index}"
            assert f"- {joint}" in text
            assert joint in joint_names
            limit = root_element.find(f"./joint[@name='{joint}']/limit")
            assert float(limit.attrib["lower"]) <= defaults[index - 1]
            assert defaults[index - 1] <= float(limit.attrib["upper"])
        assert f"{side}_fr3v2_hand_tcp" in link_names
        assert "api_version: 1.0" in text
        assert "collision_spheres: []" in text
        assert "NVIDIA CORPORATION" not in text
        assert "EBiM Benchmark Contributors" in text
        assert "Robotiq_DEMO commit 78c28ea" in text


def test_lula_configs_classify_every_movable_urdf_joint_once():
    config_dir = TASK3_ROOT / "assets" / "lula" / "mobile_fr3_duo"
    urdf_root = ET.parse(
        config_dir / "mobile_fr3_duo_v0_2_franka_hand.urdf"
    ).getroot()
    movable = {
        joint.attrib["name"]: joint
        for joint in urdf_root.findall("joint")
        if joint.attrib["type"] != "fixed"
    }
    home = [0.0, -1.5, 0.0, -2.2, 0.0, 1.5, 0.785]

    for side, opposite in (("left", "right"), ("right", "left")):
        config = yaml.safe_load(
            (config_dir / f"{side}_arm_description.yaml").read_text()
        )
        cspace = config["cspace"]
        rules = config["cspace_to_urdf_rules"]
        rule_names = [rule["name"] for rule in rules]

        assert len(cspace) == len(set(cspace))
        assert len(rule_names) == len(set(rule_names))
        assert set(cspace).isdisjoint(rule_names)
        assert set(cspace) | set(rule_names) == set(movable)
        assert config["default_q"] == home

        rule_values = {}
        for rule in rules:
            assert rule["rule"] == "fixed"
            value = float(rule["value"])
            assert math.isfinite(value)
            rule_values[rule["name"]] = value
            limit = movable[rule["name"]].find("limit")
            if limit is not None and "lower" in limit.attrib:
                assert float(limit.attrib["lower"]) <= value
                assert value <= float(limit.attrib["upper"])

        assert [
            rule_values[f"{opposite}_fr3v2_joint{index}"]
            for index in range(1, 8)
        ] == home


def test_tracked_lula_urdf_contains_only_portable_kinematics():
    urdf_path = (
        TASK3_ROOT
        / "assets"
        / "lula"
        / "mobile_fr3_duo"
        / "mobile_fr3_duo_v0_2_franka_hand.urdf"
    )
    text = urdf_path.read_text()
    robot = ET.fromstring(text)

    assert robot.findall(".//visual") == []
    assert robot.findall(".//collision") == []
    assert robot.findall(".//mesh") == []
    assert all(
        not Path(element.attrib["filename"]).is_absolute()
        for element in robot.iter()
        if "filename" in element.attrib
    )
    assert "/home/" not in text
    assert "autogenerated by xacro from /workspaces/" not in text
    assert "EBiM Benchmark Contributors" in text
    assert "Apache-2.0" in text
    assert robot.findall(".//inertial")
