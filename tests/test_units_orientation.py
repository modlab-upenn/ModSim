"""Unit-aware authoring and quaternion display conversion contracts."""

from __future__ import annotations

import math
import random
from typing import cast

import pytest

from modsim_robotics.core.transforms import (
    Quat,
    Transform,
    Vec3,
    quat_from_rpy,
    quat_rotate,
    quat_to_rpy,
)
from modsim_robotics.robot_packs.schema import PoseSpec
from modsim_robotics.units import UnitError


def _assert_same_rotation(first: Quat, second: Quat, *, tolerance: float = 1e-10) -> None:
    """Compare the action on a basis, independent of quaternion sign."""
    basis: tuple[Vec3, ...] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    for axis in basis:
        assert quat_rotate(first, axis) == pytest.approx(
            quat_rotate(second, axis), rel=0.0, abs=tolerance
        )


@pytest.mark.parametrize(
    "degrees",
    [
        (0.0, 0.0, 0.0),
        (90.0, 0.0, 0.0),
        (0.0, 90.0, 0.0),
        (0.0, 0.0, 90.0),
        (180.0, 0.0, 0.0),
        (0.0, 180.0, 0.0),
        (0.0, 0.0, 180.0),
        (23.0, -41.0, 117.0),
        (12.0, 90.0, 70.0),
        (12.0, -90.0, 70.0),
        (39.0, 90.0 - 1e-8, -74.0),
        (39.0, -90.0 + 1e-8, -74.0),
        (-720.0, 450.0, 1080.0),
    ],
)
def test_quaternion_euler_round_trip_preserves_orientation(degrees: Vec3) -> None:
    radians = (math.radians(degrees[0]), math.radians(degrees[1]), math.radians(degrees[2]))
    quaternion = quat_from_rpy(radians)
    displayed = quat_to_rpy(quaternion)
    assert -math.pi / 2 <= displayed[1] <= math.pi / 2
    _assert_same_rotation(quaternion, quat_from_rpy(displayed))


def test_quaternion_euler_round_trip_covers_arbitrary_rotations() -> None:
    generator = random.Random(4319)
    for _ in range(250):
        rpy = (
            generator.uniform(-4 * math.pi, 4 * math.pi),
            generator.uniform(-4 * math.pi, 4 * math.pi),
            generator.uniform(-4 * math.pi, 4 * math.pi),
        )
        quaternion = quat_from_rpy(rpy)
        _assert_same_rotation(quaternion, quat_from_rpy(quat_to_rpy(quaternion)))


@pytest.mark.parametrize("pitch", [-math.pi / 2, math.pi / 2])
def test_gimbal_lock_display_uses_a_deterministic_equivalent_rotation(pitch: float) -> None:
    quaternion = quat_from_rpy((0.7, pitch, -1.2))
    displayed = quat_to_rpy(quaternion)
    assert displayed[0] == 0.0
    assert displayed[1] == pytest.approx(pitch)
    _assert_same_rotation(quaternion, quat_from_rpy(displayed))


def test_quaternion_sign_does_not_change_the_displayed_orientation() -> None:
    quaternion = quat_from_rpy((0.2, -0.9, 2.7))
    negative = (-quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])
    assert quat_to_rpy(quaternion) == pytest.approx(quat_to_rpy(negative), abs=1e-14)


@pytest.mark.parametrize("scale", [1e-300, 1.0, 1e300])
def test_quaternion_display_normalizes_finite_nonunit_inputs(scale: float) -> None:
    quaternion = quat_from_rpy((0.2, -0.9, 2.7))
    scaled = (
        quaternion[0] * scale,
        quaternion[1] * scale,
        quaternion[2] * scale,
        quaternion[3] * scale,
    )
    _assert_same_rotation(quaternion, quat_from_rpy(quat_to_rpy(scaled)))


def test_quaternion_normalization_handles_extreme_finite_components() -> None:
    displayed = quat_to_rpy((1.7e308, 1.7e308, 1.7e308, 1.7e308))
    _assert_same_rotation((0.5, 0.5, 0.5, 0.5), quat_from_rpy(displayed))
    assert quat_to_rpy((5e-324, 0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize(
    "invalid",
    [
        (0.0, 0.0, 0.0, 0.0),
        (math.nan, 0.0, 0.0, 1.0),
        (1.0, math.inf, 0.0, 0.0),
        (1.0, 0.0, -math.inf, 0.0),
        (True, 0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, "invalid"),
    ],
)
def test_quaternion_display_rejects_invalid_input(invalid: object) -> None:
    with pytest.raises(ValueError):
        quat_to_rpy(cast(Quat, invalid))


def test_pose_authoring_converts_millimetres_and_degrees_without_changing_schema() -> None:
    pose = PoseSpec.from_rpy(
        (50.0, 0.0, 25.0), (0.0, 0.0, 90.0), length_unit="mm", angle_unit="deg"
    )
    assert pose.xyz_m == pytest.approx((0.05, 0.0, 0.025))
    assert pose.rpy_rad == pytest.approx((0.0, 0.0, math.pi / 2))
    assert set(pose.model_dump()) == {"xyz_m", "rpy_rad"}
    assert PoseSpec.model_validate_json(pose.model_dump_json()) == pose


def test_pose_authoring_retains_multiple_turns() -> None:
    pose = PoseSpec.from_rpy((0.0, 0.0, 0.0), (450.0, -720.0, 1080.0), angle_unit="deg")
    assert pose.rpy_rad == pytest.approx((2.5 * math.pi, -4 * math.pi, 6 * math.pi))


def test_pose_authoring_defaults_are_the_existing_canonical_units() -> None:
    xyz = (0.1, -0.2, 0.3)
    rpy = (0.4, -0.5, 0.6)
    assert PoseSpec.from_rpy(xyz, rpy) == PoseSpec(xyz_m=xyz, rpy_rad=rpy)


def test_transform_authoring_returns_si_translation_and_quaternion_rotation() -> None:
    transform = Transform.from_rpy(
        (50.0, 0.0, 25.0), (0.0, 0.0, 90.0), length_unit="mm", angle_unit="deg"
    )
    assert transform.translation == pytest.approx((0.05, 0.0, 0.025))
    assert sum(value * value for value in transform.rotation) == pytest.approx(1.0)
    assert transform.apply_direction((1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    assert transform.apply_point((0.1, 0.0, 0.0)) == pytest.approx((0.05, 0.1, 0.025))


def test_transform_unit_choices_produce_the_same_canonical_transform() -> None:
    millimetres = Transform.from_rpy(
        (50.0, -20.0, 25.0), (30.0, 45.0, 60.0), length_unit="mm", angle_unit="deg"
    )
    centimetres = Transform.from_rpy(
        (5.0, -2.0, 2.5), (30.0, 45.0, 60.0), length_unit="cm", angle_unit="deg"
    )
    canonical = Transform.from_rpy((0.05, -0.02, 0.025), (math.pi / 6, math.pi / 4, math.pi / 3))
    assert millimetres.is_close(centimetres)
    assert millimetres.is_close(canonical)


@pytest.mark.parametrize("runtime_transform", [False, True])
def test_pose_constructors_reject_incompatible_units(runtime_transform: bool) -> None:
    factory = Transform.from_rpy if runtime_transform else PoseSpec.from_rpy
    with pytest.raises(UnitError):
        factory((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), length_unit="deg")


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_pose_authoring_rejects_nonfinite_components(value: float) -> None:
    with pytest.raises(UnitError):
        PoseSpec.from_rpy((value, 0.0, 0.0), (0.0, 0.0, 0.0))
    with pytest.raises(UnitError):
        Transform.from_rpy((0.0, 0.0, 0.0), (0.0, value, 0.0))
