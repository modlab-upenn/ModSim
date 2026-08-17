"""Tests for connector compatibility and acceptance-region evaluation."""

from __future__ import annotations

import math

import pytest

from modsim.connectors.acceptance import (
    AXIS_ALIGNMENT,
    ORIENTATION,
    POSITION,
    RELATIVE_VELOCITY,
    effective_acceptance_region,
    evaluate_acceptance,
    nominal_relative_transform,
)
from modsim.connectors.compatibility import evaluate_compatibility, genders_match
from modsim.core.entities import ConnectorInstance
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.transforms import (
    IDENTITY_QUAT,
    Quat,
    Transform,
    Vec3,
    angle_between,
    quat_from_axis_angle,
    quat_rotate,
)
from modsim.robot_packs.schema import (
    AcceptanceRegion,
    AcceptanceShape,
    AllowedOrientations,
    ConnectorGender,
    ConnectorTypeSpec,
    OrientationMode,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
)

X_AXIS: Vec3 = (1.0, 0.0, 0.0)
NEGATIVE_X: Vec3 = (-1.0, 0.0, 0.0)
QUARTER_TURNS = (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)
TOLERANCE_RAD = 0.1396


def connector_type(
    identifier: str = "face",
    *,
    compatible_with: tuple[str, ...] = ("face",),
    gender: ConnectorGender = ConnectorGender.HERMAPHRODITIC,
    shape: AcceptanceShape = AcceptanceShape.BOX,
    position_tolerance_m: float = 0.006,
    orientation_tolerance_rad: float = TOLERANCE_RAD,
    max_relative_velocity_m_s: float = 0.05,
    orientations: tuple[float, ...] | None = QUARTER_TURNS,
    acceptance: bool = True,
) -> ConnectorTypeSpec:
    """Build a connector type for acceptance tests."""
    allowed = (
        AllowedOrientations(mode=OrientationMode.DISCRETE, values_rad=orientations)
        if orientations
        else AllowedOrientations(mode=OrientationMode.CONTINUOUS)
    )
    return ConnectorTypeSpec(
        id=identifier,
        gender=gender,
        compatible_with=compatible_with,
        allowed_orientations=allowed,
        acceptance_region=(
            AcceptanceRegion(
                shape=shape,
                position_tolerance_m=position_tolerance_m,
                orientation_tolerance_rad=orientation_tolerance_rad,
                max_relative_velocity_m_s=max_relative_velocity_m_s,
            )
            if acceptance
            else None
        ),
        physical_connection=PhysicalConnectionSpec(constraint=PhysicalConstraintType.FIXED),
        supports_undocking=True,
    )


def connector(
    name: str,
    *,
    translation: Vec3 = (0.0, 0.0, 0.0),
    rotation: Quat = IDENTITY_QUAT,
    docking_axis: Vec3 = X_AXIS,
    velocity: Vec3 = (0.0, 0.0, 0.0),
    type_id: str = "face",
) -> ConnectorInstance:
    """Build a resolved connector instance at a known world pose."""
    module = ModuleInstanceId(name)
    return ConnectorInstance(
        id=ConnectorInstanceId(f"{name}/face"),
        module_id=module,
        connector_id="face",
        connector_type_id=type_id,
        parent_link="base_link",
        world_pose=Transform(translation=translation, rotation=rotation),
        world_docking_axis=docking_axis,
        world_approach_axis=docking_axis,
        world_velocity_m_s=velocity,
        resolved=True,
    )


def facing_pair(
    *,
    offset: Vec3 = (0.0, 0.0, 0.0),
    roll_rad: float = 0.0,
    tilt_rad: float = 0.0,
    velocity_b: Vec3 = (0.0, 0.0, 0.0),
) -> tuple[ConnectorInstance, ConnectorInstance]:
    """Return two connectors nose to nose along the world x axis."""
    axis_b: Vec3 = (
        quat_rotate(quat_from_axis_angle((0.0, 0.0, 1.0), tilt_rad), NEGATIVE_X)
        if tilt_rad
        else NEGATIVE_X
    )
    return (
        connector("a"),
        connector(
            "b",
            translation=offset,
            rotation=quat_from_axis_angle(X_AXIS, roll_rad),
            docking_axis=axis_b,
            velocity=velocity_b,
        ),
    )


# ----------------------------------------------------------------------
# compatibility
# ----------------------------------------------------------------------


def test_mutually_declared_types_are_compatible() -> None:
    result = evaluate_compatibility(connector_type(), connector_type())
    assert result.compatible
    assert bool(result)


def test_one_sided_compatibility_is_rejected() -> None:
    left = connector_type("left", compatible_with=("right",))
    right = connector_type("right", compatible_with=())
    result = evaluate_compatibility(left, right)

    assert not result.compatible
    assert "mutually compatible" in (result.reason or "")


def test_mismatched_genders_are_rejected() -> None:
    left = connector_type("left", compatible_with=("right",), gender=ConnectorGender.MALE)
    right = connector_type("right", compatible_with=("left",), gender=ConnectorGender.MALE)
    result = evaluate_compatibility(left, right)

    assert not result.compatible
    assert "genders" in (result.reason or "")


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        (ConnectorGender.MALE, ConnectorGender.FEMALE, True),
        (ConnectorGender.FEMALE, ConnectorGender.MALE, True),
        (ConnectorGender.HERMAPHRODITIC, ConnectorGender.HERMAPHRODITIC, True),
        (ConnectorGender.GENDERLESS, ConnectorGender.GENDERLESS, True),
        (ConnectorGender.MALE, ConnectorGender.MALE, False),
        (ConnectorGender.GENDERLESS, ConnectorGender.HERMAPHRODITIC, False),
    ],
)
def test_gender_pairing_table(
    left: ConnectorGender,
    right: ConnectorGender,
    expected: bool,
) -> None:
    assert genders_match(left, right) is expected


# ----------------------------------------------------------------------
# acceptance
# ----------------------------------------------------------------------


def test_coincident_facing_connectors_are_accepted() -> None:
    first, second = facing_pair()
    result = evaluate_acceptance(first, second, connector_type(), connector_type())

    assert result.satisfied
    assert result.reason() == "acceptance satisfied"
    assert result.orientation_index == 0
    assert result.criterion(POSITION).measured == pytest.approx(0.0)


def test_position_beyond_tolerance_is_rejected() -> None:
    first, second = facing_pair(offset=(0.02, 0.0, 0.0))
    result = evaluate_acceptance(first, second, connector_type(), connector_type())

    assert not result.satisfied
    assert result.failures[0].name == POSITION
    assert "position out of tolerance" in result.reason()


def test_axis_misalignment_beyond_tolerance_is_rejected() -> None:
    first, second = facing_pair(tilt_rad=0.5)
    result = evaluate_acceptance(first, second, connector_type(), connector_type())

    assert not result.satisfied
    assert result.criterion(AXIS_ALIGNMENT).measured == pytest.approx(0.5, abs=1e-9)
    assert not result.criterion(AXIS_ALIGNMENT).satisfied


def test_roll_snaps_to_the_nearest_allowed_orientation() -> None:
    first, second = facing_pair(roll_rad=math.pi / 2 + 0.01)
    result = evaluate_acceptance(first, second, connector_type(), connector_type())

    assert result.satisfied
    assert result.orientation_index == 1
    assert result.orientation_rad == pytest.approx(math.pi / 2)
    assert result.criterion(ORIENTATION).measured == pytest.approx(0.01, abs=1e-9)


def test_roll_between_allowed_orientations_is_rejected() -> None:
    first, second = facing_pair(roll_rad=math.pi / 4)
    result = evaluate_acceptance(first, second, connector_type(), connector_type())

    assert not result.satisfied
    assert result.criterion(ORIENTATION).measured == pytest.approx(math.pi / 4)


def test_continuous_orientation_accepts_any_roll() -> None:
    first, second = facing_pair(roll_rad=math.pi / 4)
    continuous = connector_type(orientations=None)
    result = evaluate_acceptance(first, second, continuous, continuous)

    assert result.satisfied
    assert result.orientation_index is None


def test_relative_velocity_beyond_tolerance_is_rejected() -> None:
    first, second = facing_pair(velocity_b=(0.0, 0.2, 0.0))
    result = evaluate_acceptance(first, second, connector_type(), connector_type())

    assert not result.satisfied
    assert result.criterion(RELATIVE_VELOCITY).measured == pytest.approx(0.2)


def test_the_stricter_of_two_tolerances_governs() -> None:
    loose = connector_type(position_tolerance_m=0.05)
    tight = connector_type(position_tolerance_m=0.001)
    assert effective_acceptance_region(
        loose.acceptance_region,  # pyright: ignore[reportArgumentType]
        tight.acceptance_region,  # pyright: ignore[reportArgumentType]
    )[0] == pytest.approx(0.001)

    first, second = facing_pair(offset=(0.01, 0.0, 0.0))
    assert evaluate_acceptance(first, second, loose, loose).satisfied
    assert not evaluate_acceptance(first, second, loose, tight).satisfied


def test_sphere_and_box_shapes_disagree_on_a_diagonal_offset() -> None:
    diagonal = 0.005
    first, second = facing_pair(offset=(0.0, diagonal, diagonal))
    box = connector_type(shape=AcceptanceShape.BOX)
    sphere = connector_type(shape=AcceptanceShape.SPHERE)

    # Each component is inside the box half-extent, but the radius is not.
    assert evaluate_acceptance(first, second, box, box).satisfied
    assert not evaluate_acceptance(first, second, sphere, sphere).satisfied


def test_unresolved_connectors_are_never_accepted() -> None:
    first, second = facing_pair()
    first.resolved = False
    result = evaluate_acceptance(first, second, connector_type(), connector_type())

    assert not result.satisfied
    assert "have not been resolved" in result.reason()


def test_a_missing_acceptance_region_is_reported() -> None:
    first, second = facing_pair()
    result = evaluate_acceptance(
        first, second, connector_type(), connector_type("bare", acceptance=False)
    )

    assert not result.satisfied
    assert "does not declare an acceptance region" in result.reason()


# ----------------------------------------------------------------------
# nominal alignment
# ----------------------------------------------------------------------


def test_nominal_transform_makes_the_docking_axes_exactly_antiparallel() -> None:
    first, second = facing_pair(offset=(0.004, 0.001, 0.0), tilt_rad=0.05)
    nominal = nominal_relative_transform(first, second, orientation_rad=0.0)

    assert nominal.translation == (0.0, 0.0, 0.0)
    axis_in_a = first.world_pose.inverse().apply_direction(first.world_docking_axis)
    axis_in_b = second.world_pose.inverse().apply_direction(second.world_docking_axis)
    mapped = nominal.apply_direction(axis_in_b)
    assert angle_between(mapped, (-axis_in_a[0], -axis_in_a[1], -axis_in_a[2])) == pytest.approx(
        0.0, abs=1e-9
    )


def test_nominal_transform_applies_the_requested_roll() -> None:
    first = connector("a")
    second = connector("b")
    quarter = nominal_relative_transform(first, second, orientation_rad=math.pi / 2)
    straight = nominal_relative_transform(first, second, orientation_rad=0.0)

    assert not quarter.is_close(straight)

    second.world_pose = first.world_pose.compose(quarter)
    second.world_docking_axis = second.world_pose.apply_direction(X_AXIS)
    result = evaluate_acceptance(first, second, connector_type(), connector_type())
    assert result.satisfied
    assert result.orientation_rad == pytest.approx(math.pi / 2)
