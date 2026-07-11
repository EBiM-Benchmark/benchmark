import math
from pathlib import Path
import sys

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))

from teleop_commands import PoseDelta, TeleopCommand
from teleop_targets import (
    CartesianTargetTracker,
    DirectJointTargetLatch,
    JointGroups,
    Pose,
    TargetLimits,
    TeleopTargets,
    compose_position_targets,
    discover_joint_groups,
    clamp_arm_joint_positions,
    pose_base_to_world,
    pose_world_to_base,
)


def _targets() -> TeleopTargets:
    return TeleopTargets(
        left=Pose((0.5, 0.0, 0.7), (2.0, 0.0, 0.0, 0.0)),
        right=Pose((0.4, -0.1, 0.6), (1.0, 0.0, 0.0, 0.0)),
        left_gripper=0.02,
        right_gripper=0.03,
        spine=0.4,
    )


def _limits() -> TargetLimits:
    return TargetLimits(
        position_min=(0.2, -0.2, 0.3),
        position_max=(0.6, 0.2, 0.8),
        gripper_min=0.0,
        gripper_max=0.04,
        spine_min=0.0,
        spine_max=0.85,
    )


def _command(**values) -> TeleopCommand:
    return TeleopCommand(
        timestamp=1.0,
        source="keyboard",
        active=True,
        **values,
    )


def test_pose_tracker_applies_arm_deltas_independently_and_normalizes_wxyz():
    tracker = CartesianTargetTracker(_targets(), limits=_limits())

    result = tracker.apply(
        _command(
            left_pose=PoseDelta(
                translation=(0.05, 0.1, -0.2),
                rotation_rpy=(math.pi, 0.0, 0.0),
            )
        )
    )

    assert result.left.position == pytest.approx((0.55, 0.1, 0.5))
    assert result.left.orientation_wxyz == pytest.approx((0.0, 1.0, 0.0, 0.0))
    assert math.sqrt(sum(v * v for v in result.left.orientation_wxyz)) == pytest.approx(1.0)
    assert result.right == _targets().right


def test_pose_tracker_clamps_positions_grippers_and_spine():
    tracker = CartesianTargetTracker(_targets(), limits=_limits())

    result = tracker.apply(
        _command(
            left_pose=PoseDelta(translation=(1.0, -1.0, 1.0)),
            right_pose=PoseDelta(translation=(-1.0, 1.0, -1.0)),
            left_gripper_delta=1.0,
            right_gripper_delta=-1.0,
            spine_delta=1.0,
        )
    )

    assert result.left.position == (0.6, -0.2, 0.8)
    assert result.right.position == (0.2, 0.2, 0.3)
    assert result.left_gripper == 0.04
    assert result.right_gripper == 0.0
    assert result.spine == 0.85


def test_spine_motion_carries_both_base_relative_end_effector_targets():
    tracker = CartesianTargetTracker(_targets(), limits=_limits())

    raised = tracker.apply(_command(spine_delta=0.1))

    assert raised.spine == pytest.approx(0.5)
    assert raised.left.position == pytest.approx((0.5, 0.0, 0.8))
    assert raised.right.position == pytest.approx((0.4, -0.1, 0.7))


def test_clamped_spine_motion_only_carries_arms_by_applied_change():
    initial = _targets()
    tracker = CartesianTargetTracker(initial, limits=_limits())

    raised = tracker.apply(_command(spine_delta=1.0))

    assert raised.spine == pytest.approx(0.85)
    assert raised.left.position[2] == pytest.approx(0.8)
    assert raised.right.position[2] == pytest.approx(0.8)


def test_pose_world_base_round_trip_with_negative_ninety_degree_yaw():
    half = math.sqrt(0.5)
    root_position = (-4.6, 2.7, 0.0)
    root_orientation = (half, 0.0, 0.0, -half)
    world = Pose(
        position=(-4.6, 1.7, 0.8),
        orientation_wxyz=(half, 0.0, 0.0, -half),
    )

    relative = pose_world_to_base(world, root_position, root_orientation)
    reconstructed = pose_base_to_world(
        relative, root_position, root_orientation
    )

    assert relative.position == pytest.approx((1.0, 0.0, 0.8))
    assert relative.orientation_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert reconstructed.position == pytest.approx(world.position)
    assert reconstructed.orientation_wxyz == pytest.approx(
        world.orientation_wxyz
    )


def test_pose_tracker_accumulates_successive_commands():
    tracker = CartesianTargetTracker(_targets(), limits=_limits())

    tracker.apply(_command(left_pose=PoseDelta(translation=(0.02, 0.0, 0.0))))
    result = tracker.apply(_command(left_pose=PoseDelta(translation=(0.03, 0.0, 0.0))))

    assert result.left.position == pytest.approx((0.55, 0.0, 0.7))


def test_inactive_command_does_not_change_targets():
    tracker = CartesianTargetTracker(_targets(), limits=_limits())
    command = TeleopCommand(
        timestamp=1.0,
        source="keyboard",
        active=False,
        left_pose=PoseDelta(translation=(0.1, 0.1, 0.1)),
        left_gripper_delta=0.1,
        spine_delta=0.1,
    )

    assert tracker.apply(command) == tracker.targets
    assert tracker.targets.left.position == _targets().left.position


LEFT_ARM = tuple(f"left_fr3v2_joint{i}" for i in range(1, 8))
RIGHT_ARM = tuple(f"right_fr3v2_joint{i}" for i in range(1, 8))
LEFT_GRIPPER = (
    "left_fr3v2_finger_joint1",
    "left_fr3v2_finger_joint2",
)
RIGHT_GRIPPER = (
    "right_fr3v2_finger_joint1",
    "right_fr3v2_finger_joint2",
)
BASE = (
    "tmrv0_2_joint_0",
    "tmrv0_2_joint_1",
    "tmrv0_2_joint_2",
    "tmrv0_2_joint_3",
)


def _joint_names() -> list[str]:
    return [
        "unowned_joint",
        *RIGHT_ARM,
        BASE[3],
        *LEFT_GRIPPER,
        "franka_spine_vertical_joint",
        BASE[0],
        *LEFT_ARM,
        *RIGHT_GRIPPER,
        BASE[2],
        BASE[1],
    ]


def test_joint_discovery_finds_exact_mobile_fr3_duo_groups_in_runtime_order():
    names = _joint_names()

    groups = discover_joint_groups(names)

    assert groups.left_arm == tuple(names.index(name) for name in LEFT_ARM)
    assert groups.right_arm == tuple(names.index(name) for name in RIGHT_ARM)
    assert groups.left_gripper == tuple(names.index(name) for name in LEFT_GRIPPER)
    assert groups.right_gripper == tuple(names.index(name) for name in RIGHT_GRIPPER)
    assert groups.spine == (names.index("franka_spine_vertical_joint"),)
    assert groups.steering == tuple(names.index(name) for name in BASE[::2])
    assert groups.drive == tuple(names.index(name) for name in BASE[1::2])


def test_joint_discovery_reports_all_missing_required_names():
    names = _joint_names()
    names.remove("left_fr3v2_joint3")
    names.remove("right_fr3v2_finger_joint2")

    with pytest.raises(RuntimeError) as error:
        discover_joint_groups(names)

    message = str(error.value)
    assert "Missing required mobile FR3 Duo joints" in message
    assert "left_fr3v2_joint3" in message
    assert "right_fr3v2_finger_joint2" in message


def test_joint_discovery_reports_duplicate_required_names():
    names = [*_joint_names(), "tmrv0_2_joint_0", "left_fr3v2_joint1"]

    with pytest.raises(RuntimeError) as error:
        discover_joint_groups(names)

    message = str(error.value)
    assert "Duplicate required mobile FR3 Duo joints" in message
    assert "tmrv0_2_joint_0" in message
    assert "left_fr3v2_joint1" in message


def test_composer_clones_input_and_updates_only_supplied_groups():
    names = _joint_names()
    groups = discover_joint_groups(names)
    current = torch.arange(len(names), dtype=torch.float32).reshape(1, -1)

    result = compose_position_targets(
        current,
        groups,
        left_arm=[0.1] * 7,
        right_gripper=0.025,
        spine=0.7,
    )

    assert result is not current
    assert result[0, list(groups.left_arm)].tolist() == pytest.approx([0.1] * 7)
    assert result[0, list(groups.right_gripper)].tolist() == pytest.approx([0.025, 0.025])
    assert result[0, list(groups.spine)].tolist() == pytest.approx([0.7])
    preserved = set(range(len(names))) - set(groups.left_arm + groups.right_gripper + groups.spine)
    assert result[0, list(preserved)].tolist() == current[0, list(preserved)].tolist()
    assert current[0, list(groups.left_arm)].tolist() != pytest.approx([0.1] * 7)


def test_composer_can_update_both_arms_and_grippers_for_multiple_environments():
    groups = discover_joint_groups(_joint_names())
    current = torch.zeros((2, len(_joint_names())))

    result = compose_position_targets(
        current,
        groups,
        left_arm=torch.tensor([[1.0] * 7, [2.0] * 7]),
        right_arm=[3.0] * 7,
        left_gripper=[0.01, 0.02],
        right_gripper=0.03,
    )

    assert result[:, list(groups.left_arm)].tolist() == [[1.0] * 7, [2.0] * 7]
    assert result[:, list(groups.right_arm)].tolist() == [[3.0] * 7, [3.0] * 7]
    assert torch.allclose(
        result[:, list(groups.left_gripper)],
        torch.tensor([[0.01, 0.01], [0.02, 0.02]]),
    )
    assert torch.allclose(
        result[:, list(groups.right_gripper)],
        torch.tensor([[0.03, 0.03], [0.03, 0.03]]),
    )


def test_joint_groups_are_frozen_values():
    groups = discover_joint_groups(_joint_names())

    with pytest.raises((AttributeError, TypeError)):
        groups.left_arm = ()

    assert isinstance(groups, JointGroups)


def test_base_frame_rotation_delta_is_pre_multiplied_for_noncommuting_axes():
    root_half = math.sqrt(0.5)
    initial = TeleopTargets(
        left=Pose(
            (0.5, 0.0, 0.7),
            (root_half, root_half, 0.0, 0.0),
        ),
        right=_targets().right,
        left_gripper=0.02,
        right_gripper=0.03,
        spine=0.4,
    )
    tracker = CartesianTargetTracker(initial, limits=_limits())

    result = tracker.apply(
        _command(
            left_pose=PoseDelta(rotation_rpy=(0.0, 0.0, math.pi / 2.0))
        )
    )

    # Base-frame yaw followed by the existing local roll is q_yaw * q_roll.
    assert result.left.orientation_wxyz == pytest.approx(
        (0.5, 0.5, 0.5, 0.5)
    )


@pytest.mark.parametrize(
    "position",
    [(0.1, 0.2), (0.1, 0.2, 0.3, 0.4)],
)
def test_pose_rejects_malformed_position_vectors(position):
    with pytest.raises(ValueError, match="position.*3"):
        Pose(position, (1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_pose_rejects_nonfinite_position_or_orientation(bad):
    with pytest.raises(ValueError, match="position.*finite"):
        Pose((bad, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))

    with pytest.raises(ValueError, match="orientation.*finite"):
        Pose((0.5, 0.0, 0.7), (1.0, bad, 0.0, 0.0))


def test_pose_rejects_malformed_orientation_quaternion():
    with pytest.raises(ValueError, match="orientation.*4"):
        Pose((0.5, 0.0, 0.7), (1.0, 0.0, 0.0))


@pytest.mark.parametrize(
    ("lower", "upper"),
    [((0.0, 0.0), (1.0, 1.0)), ((0.0,) * 4, (1.0,) * 4)],
)
def test_target_limits_require_three_axis_position_bounds(lower, upper):
    with pytest.raises(ValueError, match="position.*3"):
        TargetLimits(position_min=lower, position_max=upper)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize(
    "field", ["left_gripper", "right_gripper", "spine"]
)
def test_tracker_rejects_nonfinite_initial_scalar_targets(field, bad):
    values = {
        "left": Pose((0.5, 0.0, 0.7), (1.0, 0.0, 0.0, 0.0)),
        "right": Pose((0.4, -0.1, 0.6), (1.0, 0.0, 0.0, 0.0)),
        "left_gripper": 0.02,
        "right_gripper": 0.03,
        "spine": 0.4,
    }
    values[field] = bad

    with pytest.raises(ValueError, match=f"{field}.*finite"):
        CartesianTargetTracker(TeleopTargets(**values), limits=_limits())


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize(
    "command_values",
    [
        lambda bad: {"left_pose": PoseDelta(translation=(bad, 0.0, 0.0))},
        lambda bad: {"right_pose": PoseDelta(rotation_rpy=(0.0, bad, 0.0))},
        lambda bad: {"left_gripper_delta": bad},
        lambda bad: {"right_gripper_delta": bad},
        lambda bad: {"spine_delta": bad},
    ],
)
def test_nonfinite_command_delta_is_atomic(command_values, bad):
    tracker = CartesianTargetTracker(_targets(), limits=_limits())
    before = tracker.targets

    with pytest.raises(ValueError, match="command.*finite"):
        tracker.apply(_command(**command_values(bad)))

    assert tracker.targets == before


@pytest.mark.parametrize(
    "pose_delta",
    [
        PoseDelta(translation=(0.1, 0.2)),
        PoseDelta(rotation_rpy=(0.1, 0.2, 0.3, 0.4)),
    ],
)
def test_malformed_command_pose_delta_is_atomic(pose_delta):
    tracker = CartesianTargetTracker(_targets(), limits=_limits())
    before = tracker.targets

    with pytest.raises(ValueError, match="command.*3"):
        tracker.apply(_command(left_pose=pose_delta))

    assert tracker.targets == before


@pytest.mark.parametrize(
    "dtype", [torch.int32, torch.int64, torch.bool]
)
def test_composer_rejects_non_floating_current_tensor(dtype):
    groups = discover_joint_groups(_joint_names())
    current = torch.zeros((1, len(_joint_names())), dtype=dtype)

    with pytest.raises(TypeError, match="floating-point"):
        compose_position_targets(current, groups, left_arm=[0.1] * 7)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_composer_rejects_nonfinite_update_without_mutating_input(bad):
    groups = discover_joint_groups(_joint_names())
    current = torch.arange(len(_joint_names()), dtype=torch.float32).reshape(
        1, -1
    )
    before = current.clone()

    with pytest.raises(ValueError, match="finite"):
        compose_position_targets(
            current,
            groups,
            left_arm=[0.1, 0.2, bad, 0.4, 0.5, 0.6, 0.7],
        )

    assert torch.equal(current, before)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_composer_rejects_nonfinite_unowned_current_joint_without_mutation(bad):
    groups = discover_joint_groups(_joint_names())
    current = torch.arange(len(_joint_names()), dtype=torch.float32).reshape(
        1, -1
    )
    current[0, 0] = bad
    before = current.clone()

    with pytest.raises(ValueError, match="current.*finite"):
        compose_position_targets(current, groups, left_arm=[0.1] * 7)

    assert torch.allclose(current, before, equal_nan=True)


def test_direct_joint_latch_holds_last_target_when_later_command_is_stale():
    latch = DirectJointTargetLatch()
    direct = tuple(0.1 * index for index in range(7))
    ik = type("IK", (), {"left": {name: 1.0 for name in LEFT_ARM}, "right": {}})()

    first_left, _ = latch.select(
        _command(left_joint_positions=direct), ik, LEFT_ARM, RIGHT_ARM
    )
    held_left, _ = latch.select(
        TeleopCommand.stop(timestamp=2.0, source="gello"),
        ik,
        LEFT_ARM,
        RIGHT_ARM,
    )

    assert first_left == direct
    assert held_left == direct


def test_direct_joint_latch_owns_each_arm_independently_and_release_restores_ik():
    latch = DirectJointTargetLatch()
    direct_left = (0.2,) * 7
    ik = type(
        "IK",
        (),
        {
            "left": {name: 1.0 for name in LEFT_ARM},
            "right": {name: 2.0 for name in RIGHT_ARM},
        },
    )()

    left, right = latch.select(
        _command(left_joint_positions=direct_left), ik, LEFT_ARM, RIGHT_ARM
    )
    latch.release("left")
    released_left, released_right = latch.select(
        _command(), ik, LEFT_ARM, RIGHT_ARM
    )

    assert left == direct_left
    assert right == [2.0] * 7
    assert released_left == [1.0] * 7
    assert released_right == [2.0] * 7


def test_direct_joint_positions_clamp_to_ordered_soft_limits():
    result = clamp_arm_joint_positions(
        (-2.0, -0.5, 0.0, 0.5, 2.0, 4.0, 9.0),
        (-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0),
        (1.0, 1.0, 1.0, 1.0, 1.0, 3.0, 8.0),
    )

    assert result == (-1.0, -0.5, 0.0, 0.5, 1.0, 3.0, 8.0)


@pytest.mark.parametrize(
    ("lower", "upper"),
    [((0.0,) * 6, (1.0,) * 7), ((0.0,) * 7, (1.0,) * 6), ((1.0,) * 7, (0.0,) * 7)],
)
def test_direct_joint_limit_validation_rejects_bad_shapes_or_order(lower, upper):
    with pytest.raises(ValueError, match="seven finite ordered"):
        clamp_arm_joint_positions((0.0,) * 7, lower, upper)
