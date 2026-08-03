"""Tests for the dependency-free transform algebra."""

from __future__ import annotations

import math

import pytest

from modsim.core.transforms import (
    IDENTITY_QUAT,
    Transform,
    Vec3,
    angle_between,
    minimal_rotation,
    perpendicular,
    quat_from_axis_angle,
    quat_from_rpy,
    quat_rotate,
    signed_angle_about,
    vec_dot,
    vec_norm,
    vec_sub,
    wrap_angle,
)

X_AXIS: Vec3 = (1.0, 0.0, 0.0)
Y_AXIS: Vec3 = (0.0, 1.0, 0.0)
Z_AXIS: Vec3 = (0.0, 0.0, 1.0)


def test_rpy_follows_urdf_fixed_axis_convention() -> None:
    yaw = quat_from_rpy((0.0, 0.0, math.pi / 2))
    rotated = quat_rotate(yaw, X_AXIS)
    assert vec_norm(vec_sub(rotated, Y_AXIS)) == pytest.approx(0.0, abs=1e-12)

    pitch = quat_from_rpy((0.0, math.pi / 2, 0.0))
    assert vec_norm(vec_sub(quat_rotate(pitch, X_AXIS), (0.0, 0.0, -1.0))) == pytest.approx(
        0.0, abs=1e-12
    )


def test_compose_and_inverse_round_trip() -> None:
    transform = Transform(
        translation=(0.3, -0.2, 1.4),
        rotation=quat_from_rpy((0.4, -0.9, 2.1)),
    )
    identity = transform.compose(transform.inverse())
    assert identity.is_close(Transform.identity())
    assert transform.inverse().inverse().is_close(transform)


def test_relative_to_expresses_a_pose_in_another_frame() -> None:
    parent = Transform(translation=(1.0, 2.0, 3.0), rotation=quat_from_rpy((0.0, 0.0, math.pi / 2)))
    child = Transform(translation=(1.0, 3.0, 3.0))
    relative = child.relative_to(parent)
    # Child sits one metre along the parent's +y, which the parent's own frame
    # sees as its rotated -x direction.
    assert relative.translation[0] == pytest.approx(1.0, abs=1e-12)
    assert relative.translation[1] == pytest.approx(0.0, abs=1e-12)


def test_apply_point_and_direction_differ_by_translation() -> None:
    transform = Transform(translation=(5.0, 0.0, 0.0), rotation=IDENTITY_QUAT)
    assert transform.apply_point((1.0, 0.0, 0.0)) == (6.0, 0.0, 0.0)
    assert transform.apply_direction((1.0, 0.0, 0.0)) == (1.0, 0.0, 0.0)


def test_angle_between_is_accurate_near_zero() -> None:
    tiny = 1e-8
    nearly_parallel: Vec3 = (1.0, tiny, 0.0)
    measured = angle_between(X_AXIS, nearly_parallel)
    assert measured == pytest.approx(tiny, rel=1e-6)


def test_angle_between_handles_antiparallel_vectors() -> None:
    assert angle_between(X_AXIS, (-1.0, 0.0, 0.0)) == pytest.approx(math.pi)


def test_minimal_rotation_handles_the_antiparallel_case() -> None:
    rotation = minimal_rotation(X_AXIS, (-1.0, 0.0, 0.0))
    rotated = quat_rotate(rotation, X_AXIS)
    assert vec_norm(vec_sub(rotated, (-1.0, 0.0, 0.0))) == pytest.approx(0.0, abs=1e-9)


def test_minimal_rotation_is_identity_for_parallel_vectors() -> None:
    assert minimal_rotation(Z_AXIS, (0.0, 0.0, 2.0)) == IDENTITY_QUAT


@pytest.mark.parametrize("axis", [X_AXIS, Y_AXIS, Z_AXIS, (0.3, -0.5, 0.81)])
def test_perpendicular_is_orthogonal_and_unit_length(axis: Vec3) -> None:
    result = perpendicular(axis)
    assert vec_norm(result) == pytest.approx(1.0)
    assert vec_dot(result, axis) == pytest.approx(0.0, abs=1e-12)


def test_signed_angle_about_measures_roll_with_sign() -> None:
    rolled = quat_rotate(quat_from_axis_angle(X_AXIS, 0.7), Y_AXIS)
    assert signed_angle_about(Y_AXIS, rolled, X_AXIS) == pytest.approx(0.7)
    assert signed_angle_about(rolled, Y_AXIS, X_AXIS) == pytest.approx(-0.7)


def test_signed_angle_about_rejects_a_parallel_reference() -> None:
    with pytest.raises(ValueError, match="parallel reference"):
        signed_angle_about(X_AXIS, Y_AXIS, X_AXIS)


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, math.pi),
        (3 * math.pi, math.pi),
        (math.tau + 0.25, 0.25),
    ],
)
def test_wrap_angle_maps_into_the_half_open_turn(angle: float, expected: float) -> None:
    assert wrap_angle(angle) == pytest.approx(expected)


def test_normalising_a_zero_vector_is_an_error() -> None:
    with pytest.raises(ValueError, match="zero-length"):
        perpendicular((0.0, 0.0, 0.0))
