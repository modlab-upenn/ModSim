"""Acceptance-region evaluation for a candidate connector pair.

Acceptance answers one question: are these two connectors currently close
enough, aligned enough, correctly rolled, and slow enough to latch? The result
is a structured record rather than a boolean, because both the Studio overlay
and the ``DockFailed`` event need to say *which* criterion failed and by how
much.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modsim.core.entities import ConnectorInstance
from modsim.core.transforms import (
    Transform,
    Vec3,
    angle_between,
    minimal_rotation,
    project_onto_plane,
    quat_from_axis_angle,
    quat_multiply,
    quat_normalize,
    quat_rotate,
    signed_angle_about,
    vec_dot,
    vec_norm,
    vec_scale,
    vec_sub,
    wrap_angle,
)
from modsim.robot_packs.schema import (
    AcceptanceRegion,
    AcceptanceShape,
    AllowedOrientations,
    ConnectorTypeSpec,
    OrientationMode,
)

_DEGENERATE_REFERENCE = 1e-6

POSITION = "position"
AXIS_ALIGNMENT = "axis_alignment"
ORIENTATION = "orientation"
RELATIVE_VELOCITY = "relative_velocity"


@dataclass(frozen=True, slots=True)
class AcceptanceCriterion:
    """One measured acceptance quantity and the tolerance it was checked against."""

    name: str
    satisfied: bool
    measured: float
    tolerance: float
    unit: str
    detail: str | None = None

@dataclass(frozen=True, slots=True)
class AcceptanceResult:
    """The full outcome of evaluating one candidate connector pair."""

    satisfied: bool
    criteria: tuple[AcceptanceCriterion, ...]
    relative_transform: Transform
    orientation_rad: float
    orientation_index: int | None
    detail: str | None = None

    def criterion(self, name: str) -> AcceptanceCriterion:
        """Return one criterion by name."""
        for item in self.criteria:
            if item.name == name:
                return item
        raise KeyError(f"no acceptance criterion named '{name}'")

    @property
    def failures(self) -> tuple[AcceptanceCriterion, ...]:
        """Return every unsatisfied criterion in evaluation order."""
        return tuple(item for item in self.criteria if not item.satisfied)

    def reason(self) -> str:
        """Return a short human-readable explanation of the outcome."""
        if self.detail is not None:
            return self.detail
        if self.satisfied:
            return "acceptance satisfied"
        first = self.failures[0]
        return (
            f"{first.name} out of tolerance: "
            f"{first.measured:.6g}{first.unit} > {first.tolerance:.6g}{first.unit}"
        )


def _unsatisfied(detail: str) -> AcceptanceResult:
    return AcceptanceResult(
        satisfied=False,
        criteria=(),
        relative_transform=Transform.identity(),
        orientation_rad=0.0,
        orientation_index=None,
        detail=detail,
    )


def _frame_reference(pose: Transform, axis: Vec3) -> Vec3:
    """Return a stable in-plane reference direction for measuring roll.

    The connector frame's own x axis is preferred so that roll is expressed
    against authored geometry. When x is parallel to the docking axis the
    projection degenerates, so y and then z are tried in turn.
    """
    for cardinal in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        projected = project_onto_plane(pose.apply_direction(cardinal), axis)
        if vec_norm(projected) > _DEGENERATE_REFERENCE:
            return projected
    raise ValueError("connector frame has no direction perpendicular to its docking axis")


def _position_satisfied(
    shape: AcceptanceShape,
    offset_in_frame: Vec3,
    axis_in_frame: Vec3,
    tolerance_m: float,
) -> bool:
    if shape is AcceptanceShape.SPHERE:
        return vec_norm(offset_in_frame) <= tolerance_m
    if shape is AcceptanceShape.BOX:
        return all(abs(component) <= tolerance_m for component in offset_in_frame)
    axial = vec_dot(offset_in_frame, axis_in_frame)
    radial = vec_norm(vec_sub(offset_in_frame, vec_scale(axis_in_frame, axial)))
    return abs(axial) <= tolerance_m and radial <= tolerance_m


def _snap_orientation(
    orientations: AllowedOrientations | None,
    roll_rad: float,
) -> tuple[int | None, float, float]:
    """Return the matched index, snapped angle, and angular error for a roll.

    A connector type without a declared orientation set is treated as
    continuous, which accepts any roll and reports zero error.
    """
    if orientations is None or orientations.mode is OrientationMode.CONTINUOUS:
        return None, wrap_angle(roll_rad), 0.0
    best_index = 0
    best_error = math.inf
    for index, value in enumerate(orientations.values_rad):
        error = abs(wrap_angle(roll_rad - value))
        if error < best_error:
            best_index, best_error = index, error
    return best_index, wrap_angle(orientations.values_rad[best_index]), best_error


def effective_acceptance_region(
    region_a: AcceptanceRegion,
    region_b: AcceptanceRegion,
) -> tuple[float, float, float]:
    """Return the stricter position, orientation, and velocity tolerances.

    When two connector types declare different tolerances, the tighter value
    governs. A connector cannot become more forgiving by being presented with a
    sloppier mate.
    """
    return (
        min(region_a.position_tolerance_m, region_b.position_tolerance_m),
        min(region_a.orientation_tolerance_rad, region_b.orientation_tolerance_rad),
        min(region_a.max_relative_velocity_m_s, region_b.max_relative_velocity_m_s),
    )


def evaluate_acceptance(
    connector_a: ConnectorInstance,
    connector_b: ConnectorInstance,
    type_a: ConnectorTypeSpec,
    type_b: ConnectorTypeSpec,
) -> AcceptanceResult:
    """Evaluate whether two connector instances may latch right now."""
    if not connector_a.resolved or not connector_b.resolved:
        return _unsatisfied("connector world frames have not been resolved from a backend snapshot")
    region_a, region_b = type_a.acceptance_region, type_b.acceptance_region
    if region_a is None or region_b is None:
        missing = type_a.id if region_a is None else type_b.id
        return _unsatisfied(f"connector type '{missing}' does not declare an acceptance region")

    position_tol, orientation_tol, velocity_tol = effective_acceptance_region(region_a, region_b)
    pose_a, pose_b = connector_a.world_pose, connector_b.world_pose
    relative = pose_b.relative_to(pose_a)

    # Position, evaluated in each connector's own frame so that the declared
    # acceptance shape means what it says relative to that connector.
    offset_in_a = relative.translation
    offset_in_b = pose_a.relative_to(pose_b).translation
    axis_in_a = pose_a.inverse().apply_direction(connector_a.world_docking_axis)
    axis_in_b = pose_b.inverse().apply_direction(connector_b.world_docking_axis)
    position_ok = _position_satisfied(
        region_a.shape, offset_in_a, axis_in_a, position_tol
    ) and _position_satisfied(region_b.shape, offset_in_b, axis_in_b, position_tol)
    distance = vec_norm(offset_in_a)
    position = AcceptanceCriterion(
        name=POSITION,
        satisfied=position_ok,
        measured=distance,
        tolerance=position_tol,
        unit="m",
        detail=(
            f"{region_a.shape.value}/{region_b.shape.value} acceptance shapes"
            if region_a.shape is not region_b.shape
            else f"{region_a.shape.value} acceptance shape"
        ),
    )

    # Axis alignment: two connectors mate when their docking axes are
    # antiparallel, that is, when each points into the other.
    axis_error = angle_between(
        connector_a.world_docking_axis,
        vec_scale(connector_b.world_docking_axis, -1.0),
    )
    alignment = AcceptanceCriterion(
        name=AXIS_ALIGNMENT,
        satisfied=axis_error <= orientation_tol,
        measured=axis_error,
        tolerance=orientation_tol,
        unit="rad",
    )

    # Roll about the shared docking axis, snapped to each side's orientation set.
    orientation_index: int | None = None
    orientation_rad = 0.0
    try:
        axis = connector_a.world_docking_axis
        roll = signed_angle_about(
            _frame_reference(pose_a, axis),
            _frame_reference(pose_b, axis),
            axis,
        )
        index_a, snapped_a, error_a = _snap_orientation(type_a.allowed_orientations, roll)
        _, _, error_b = _snap_orientation(type_b.allowed_orientations, -roll)
        orientation_error = max(error_a, error_b)
        orientation_index = index_a
        orientation_rad = snapped_a
        orientation_detail: str | None = None
    except ValueError as error:
        orientation_error = math.pi
        orientation_detail = str(error)
    orientation = AcceptanceCriterion(
        name=ORIENTATION,
        satisfied=orientation_error <= orientation_tol,
        measured=orientation_error,
        tolerance=orientation_tol,
        unit="rad",
        detail=orientation_detail,
    )

    relative_speed = vec_norm(
        vec_sub(connector_a.world_velocity_m_s, connector_b.world_velocity_m_s)
    )
    velocity = AcceptanceCriterion(
        name=RELATIVE_VELOCITY,
        satisfied=relative_speed <= velocity_tol,
        measured=relative_speed,
        tolerance=velocity_tol,
        unit="m/s",
    )

    criteria = (position, alignment, orientation, velocity)
    return AcceptanceResult(
        satisfied=all(item.satisfied for item in criteria),
        criteria=criteria,
        relative_transform=relative,
        orientation_rad=orientation_rad,
        orientation_index=orientation_index,
    )


def nominal_relative_transform(
    connector_a: ConnectorInstance,
    connector_b: ConnectorInstance,
    orientation_rad: float,
) -> Transform:
    """Return the ideal connector-frame relative pose for a mated pair.

    The nominal pose places the two connector frames coincident, with their
    docking axes exactly antiparallel and the roll set to the matched
    orientation value. Snapping to it keeps repeated reconfiguration from
    accumulating pose drift, at the cost of a small discontinuity at latch.
    """
    pose_a, pose_b = connector_a.world_pose, connector_b.world_pose
    axis_in_a = pose_a.inverse().apply_direction(connector_a.world_docking_axis)
    axis_in_b = pose_b.inverse().apply_direction(connector_b.world_docking_axis)
    align = minimal_rotation(axis_in_b, vec_scale(axis_in_a, -1.0))

    # A 180-degree axis alignment has infinitely many valid rotation axes. The
    # deterministic one chosen by ``minimal_rotation`` can already introduce a
    # half-turn of connector roll, so applying ``orientation_rad`` directly is
    # not sufficient. Measure that baseline in the connector-local frames and
    # apply only the correction needed to reach the authored orientation.
    reference_a = pose_a.inverse().apply_direction(
        _frame_reference(pose_a, connector_a.world_docking_axis)
    )
    reference_b = pose_b.inverse().apply_direction(
        _frame_reference(pose_b, connector_b.world_docking_axis)
    )
    aligned_reference_b = quat_rotate(align, reference_b)
    baseline_roll = signed_angle_about(reference_a, aligned_reference_b, axis_in_a)
    correction = wrap_angle(orientation_rad - baseline_roll)
    roll = quat_from_axis_angle(axis_in_a, correction)
    return Transform(rotation=quat_normalize(quat_multiply(roll, align)))
