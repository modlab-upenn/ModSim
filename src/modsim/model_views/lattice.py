"""Built-in cubic-lattice model-view generation."""

from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Hashable
from dataclasses import dataclass
from typing import Annotated, Self, cast

from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    StrictFloat,
    ValidationError,
    field_validator,
    model_validator,
)

from modsim.core.entities import ConnectionRuntime, ModuleInstance
from modsim.core.state import WorldState
from modsim.core.transforms import (
    Quat,
    Vec3,
    angle_between,
    quat_angle,
    quat_conjugate,
    quat_multiply,
    quat_normalize,
    quat_rotate,
    vec_cross,
    vec_dot,
    vec_norm,
    vec_normalize,
    vec_sub,
)
from modsim.model_views.base import (
    ModelViewBuilder,
    ModelViewBuildError,
    ModelViewContext,
    ModelViewUnavailableError,
)
from modsim.model_views.models import (
    CubicLatticeConnectionEdge,
    CubicLatticeNode,
    CubicLatticeOccupancyConflict,
    CubicLatticeOrientation,
    CubicLatticePoseResidual,
    CubicLatticeView,
    LatticeFace,
    ModelViewSourceStamp,
)
from modsim.robot_packs.schema import ModelViewMode, ModelViewSpec

_QUATERNION_TOLERANCE = 1e-6
_ZERO_TOLERANCE = 1e-12


def _json_array_as_tuple(value: object) -> object:
    """Accept JSON arrays while retaining an immutable validated configuration."""
    if isinstance(value, list):
        return tuple(cast(list[object], value))
    return value


_ConfigurationVector3 = Annotated[
    tuple[StrictFloat, StrictFloat, StrictFloat],
    BeforeValidator(_json_array_as_tuple),
]
_ConfigurationQuaternion = Annotated[
    tuple[StrictFloat, StrictFloat, StrictFloat, StrictFloat],
    BeforeValidator(_json_array_as_tuple),
]


class _CubicLatticeConfiguration(BaseModel):
    """Strict recipe configuration consumed by :class:`CubicLatticeBuilder`."""

    model_config = ConfigDict(
        allow_inf_nan=False,
        extra="forbid",
        frozen=True,
        strict=True,
        validate_default=True,
    )

    pitch_m: StrictFloat = Field(gt=0.0)
    position_tolerance_m: StrictFloat = Field(ge=0.0)
    orientation_tolerance_rad: StrictFloat = Field(ge=0.0, le=math.pi / 4.0)
    origin_world_m: _ConfigurationVector3 = (0.0, 0.0, 0.0)
    orientation_world_wxyz: _ConfigurationQuaternion = (1.0, 0.0, 0.0, 0.0)

    @field_validator("orientation_world_wxyz")
    @classmethod
    def require_unit_lattice_orientation(cls, value: Quat) -> Quat:
        magnitude = math.sqrt(sum(component * component for component in value))
        if not math.isclose(
            magnitude,
            1.0,
            rel_tol=_QUATERNION_TOLERANCE,
            abs_tol=_QUATERNION_TOLERANCE,
        ):
            raise ValueError("lattice orientation must be a unit quaternion")
        return _canonical_quaternion(value)

    @model_validator(mode="after")
    def require_unambiguous_position_tolerance(self) -> Self:
        if self.position_tolerance_m >= self.pitch_m / 2.0:
            raise ValueError("position_tolerance_m must be less than half of pitch_m")
        return self


@dataclass(frozen=True, slots=True)
class _CardinalDirection:
    face: LatticeFace
    vector: Vec3


_CARDINAL_DIRECTIONS = (
    _CardinalDirection("positive_x", (1.0, 0.0, 0.0)),
    _CardinalDirection("positive_y", (0.0, 1.0, 0.0)),
    _CardinalDirection("positive_z", (0.0, 0.0, 1.0)),
    _CardinalDirection("negative_x", (-1.0, 0.0, 0.0)),
    _CardinalDirection("negative_y", (0.0, -1.0, 0.0)),
    _CardinalDirection("negative_z", (0.0, 0.0, -1.0)),
)
_DIRECTION_BY_VECTOR = {direction.vector: direction for direction in _CARDINAL_DIRECTIONS}


def _cube_orientation_catalog() -> tuple[CubicLatticeOrientation, ...]:
    """Return the stable 24 proper rotations of an axis-aligned cube."""
    orientations: list[CubicLatticeOrientation] = []
    for local_x in _CARDINAL_DIRECTIONS:
        for local_y in _CARDINAL_DIRECTIONS:
            if vec_dot(local_x.vector, local_y.vector) != 0.0:
                continue
            local_z_vector = vec_cross(local_x.vector, local_y.vector)
            local_z = _DIRECTION_BY_VECTOR[local_z_vector]
            orientations.append(
                CubicLatticeOrientation(
                    index=len(orientations),
                    id=f"x_{local_x.face}_y_{local_y.face}",
                    local_x_face=local_x.face,
                    local_y_face=local_y.face,
                    local_z_face=local_z.face,
                    lattice_orientation_wxyz=_quaternion_from_basis(
                        local_x.vector,
                        local_y.vector,
                        local_z.vector,
                    ),
                )
            )
    if len(orientations) != 24:  # pragma: no cover - module invariant
        raise AssertionError("cube rotation catalog must contain exactly 24 orientations")
    return tuple(orientations)


class CubicLatticeBuilder(ModelViewBuilder[CubicLatticeView]):
    """Quantize module root poses into a spatial cubic-lattice multigraph."""

    @property
    def builder_id(self) -> str:
        return "cubic_lattice"

    @property
    def display_name(self) -> str:
        return "Cubic lattice"

    @property
    def view_type(self) -> str:
        return "cubic_lattice"

    @property
    def modes(self) -> tuple[ModelViewMode, ...]:
        return (ModelViewMode.RUNTIME,)

    @property
    def requires_world(self) -> bool:
        return True

    def cache_token(self, context: ModelViewContext) -> Hashable:
        world = context.world
        if world is None:
            raise ModelViewUnavailableError("builder 'cubic_lattice' requires a runtime WorldState")
        revision = world.revision
        return (revision.sample_sequence, revision.topology_revision)

    def build(
        self,
        recipe: ModelViewSpec,
        context: ModelViewContext,
        source: ModelViewSourceStamp,
    ) -> CubicLatticeView:
        world = context.world
        if world is None:
            raise ModelViewUnavailableError("builder 'cubic_lattice' requires a runtime WorldState")
        configuration = _parse_configuration(recipe)

        provisional_nodes = tuple(
            _module_node(module, world, configuration)
            for module in sorted(world.modules.values(), key=lambda item: str(item.id))
        )
        conflicts = _occupancy_conflicts(provisional_nodes)
        conflicted_ids = {module_id for conflict in conflicts for module_id in conflict.module_ids}
        nodes = tuple(
            node.model_copy(update={"occupancy_conflict": node.id in conflicted_ids})
            for node in provisional_nodes
        )
        edges = tuple(
            _connection_edge(connection, context, configuration.orientation_world_wxyz)
            for connection in sorted(world.connections.values(), key=lambda item: str(item.id))
        )
        return CubicLatticeView(
            id=recipe.id,
            name=recipe.name or self.display_name,
            builder=self.builder_id,
            source=source,
            nodes=nodes,
            edges=edges,
            pitch_m=configuration.pitch_m,
            origin_world_m=configuration.origin_world_m,
            orientation_world_wxyz=configuration.orientation_world_wxyz,
            position_tolerance_m=configuration.position_tolerance_m,
            orientation_tolerance_rad=configuration.orientation_tolerance_rad,
            orientation_catalog=_CUBE_ORIENTATIONS,
            occupancy_conflicts=conflicts,
        )


def _parse_configuration(recipe: ModelViewSpec) -> _CubicLatticeConfiguration:
    try:
        return _CubicLatticeConfiguration.model_validate(recipe.configuration)
    except ValidationError as error:
        details = "; ".join(
            f"{'.'.join(str(part) for part in item['loc'])}: {item['msg']}"
            for item in error.errors(include_url=False)
        )
        raise ModelViewBuildError(
            f"builder 'cubic_lattice' received invalid configuration for "
            f"recipe '{recipe.id}': {details}"
        ) from None


def _module_node(
    module: ModuleInstance,
    world: WorldState,
    configuration: _CubicLatticeConfiguration,
) -> CubicLatticeNode:
    try:
        lattice_position = quat_rotate(
            quat_conjugate(configuration.orientation_world_wxyz),
            vec_sub(module.pose.translation, configuration.origin_world_m),
        )
        _require_finite_vector(lattice_position)
        cell = (
            _nearest_integer(lattice_position[0] / configuration.pitch_m),
            _nearest_integer(lattice_position[1] / configuration.pitch_m),
            _nearest_integer(lattice_position[2] / configuration.pitch_m),
        )
        snapped_position = (
            cell[0] * configuration.pitch_m,
            cell[1] * configuration.pitch_m,
            cell[2] * configuration.pitch_m,
        )
        translation_residual = vec_sub(lattice_position, snapped_position)
        position_residual_m = vec_norm(translation_residual)

        _require_finite_quaternion(module.pose.rotation)
        observed_world_orientation = quat_normalize(module.pose.rotation)
        relative_orientation = quat_normalize(
            quat_multiply(
                quat_conjugate(configuration.orientation_world_wxyz),
                observed_world_orientation,
            )
        )
        orientation, orientation_residual_rad = min(
            (
                (
                    orientation,
                    _orientation_distance(
                        relative_orientation,
                        orientation.lattice_orientation_wxyz,
                    ),
                )
                for orientation in _CUBE_ORIENTATIONS
            ),
            key=lambda item: (item[1], item[0].index),
        )
    except (OverflowError, ValueError) as error:
        raise ModelViewBuildError(
            f"module '{module.id}' has a pose that cannot be quantized: {error}"
        ) from None

    position_within_tolerance = position_residual_m <= configuration.position_tolerance_m
    orientation_within_tolerance = (
        orientation_residual_rad <= configuration.orientation_tolerance_rad
    )
    return CubicLatticeNode(
        id=str(module.id),
        label=str(module.id),
        module_type_id=module.module_type_id,
        assembly_id=str(world.assemblies.assembly_of(module.id)),
        world_position_m=module.pose.translation,
        world_orientation_wxyz=_canonical_quaternion(observed_world_orientation),
        cell=cell,
        orientation_index=orientation.index,
        orientation_id=orientation.id,
        pose_residual=CubicLatticePoseResidual(
            translation_m=_clean_vector(translation_residual),
            position_m=_clean_scalar(position_residual_m),
            orientation_rad=_clean_scalar(orientation_residual_rad),
        ),
        position_within_tolerance=position_within_tolerance,
        orientation_within_tolerance=orientation_within_tolerance,
        off_lattice=not (position_within_tolerance and orientation_within_tolerance),
        occupancy_conflict=False,
    )


def _occupancy_conflicts(
    nodes: tuple[CubicLatticeNode, ...],
) -> tuple[CubicLatticeOccupancyConflict, ...]:
    occupants: defaultdict[tuple[int, int, int], list[str]] = defaultdict(list)
    for node in nodes:
        if node.position_within_tolerance:
            occupants[node.cell].append(node.id)
    return tuple(
        CubicLatticeOccupancyConflict(cell=cell, module_ids=tuple(sorted(module_ids)))
        for cell, module_ids in sorted(occupants.items())
        if len(module_ids) > 1
    )


def _connection_edge(
    connection: ConnectionRuntime,
    context: ModelViewContext,
    lattice_orientation: Quat,
) -> CubicLatticeConnectionEdge:
    world = context.world
    if world is None:  # pragma: no cover - builder invariant
        raise ModelViewUnavailableError("builder 'cubic_lattice' requires a runtime WorldState")
    endpoints: list[tuple[str, str, LatticeFace, float]] = []
    for module_id, connector_id in (
        (connection.module_a, connection.connector_a),
        (connection.module_b, connection.connector_b),
    ):
        connector = world.connector(connector_id)
        face, residual = _nearest_face(
            connector.world_docking_axis,
            lattice_orientation,
            connector_id=str(connector_id),
        )
        endpoints.append((str(module_id), str(connector_id), face, residual))
    (
        (source, connector_a, source_face, source_residual),
        (
            target,
            connector_b,
            target_face,
            target_residual,
        ),
    ) = sorted(endpoints, key=lambda endpoint: (endpoint[0], endpoint[1]))
    return CubicLatticeConnectionEdge(
        id=str(connection.id),
        connection_id=str(connection.id),
        source=source,
        target=target,
        connector_a=connector_a,
        connector_b=connector_b,
        constraint=connection.constraint,
        orientation_rad=connection.orientation_rad,
        orientation_index=connection.orientation_index,
        created_at_s=connection.created_at_s,
        source_face=source_face,
        target_face=target_face,
        source_face_residual_rad=_clean_scalar(source_residual),
        target_face_residual_rad=_clean_scalar(target_residual),
    )


def _nearest_face(
    world_axis: Vec3,
    lattice_orientation: Quat,
    *,
    connector_id: str,
) -> tuple[LatticeFace, float]:
    try:
        _require_finite_vector(world_axis)
        lattice_axis = vec_normalize(quat_rotate(quat_conjugate(lattice_orientation), world_axis))
    except ValueError as error:
        raise ModelViewBuildError(
            f"connector '{connector_id}' has an invalid docking axis: {error}"
        ) from None
    direction, residual = min(
        (
            (
                direction,
                angle_between(lattice_axis, direction.vector),
            )
            for direction in _CARDINAL_DIRECTIONS
        ),
        key=lambda item: (item[1], item[0].face),
    )
    return direction.face, residual


def _orientation_distance(first: Quat, second: Quat) -> float:
    return quat_angle(quat_multiply(quat_conjugate(second), first))


def _nearest_integer(value: float) -> int:
    """Round to the nearest integer, resolving exact half cells away from zero."""
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def _quaternion_from_basis(local_x: Vec3, local_y: Vec3, local_z: Vec3) -> Quat:
    """Return the quaternion whose rotation matrix has the three given columns."""
    r00, r01, r02 = local_x[0], local_y[0], local_z[0]
    r10, r11, r12 = local_x[1], local_y[1], local_z[1]
    r20, r21, r22 = local_x[2], local_y[2], local_z[2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = (
            0.25 * scale,
            (r21 - r12) / scale,
            (r02 - r20) / scale,
            (r10 - r01) / scale,
        )
    elif r00 > r11 and r00 > r22:
        scale = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        quaternion = (
            (r21 - r12) / scale,
            0.25 * scale,
            (r01 + r10) / scale,
            (r02 + r20) / scale,
        )
    elif r11 > r22:
        scale = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        quaternion = (
            (r02 - r20) / scale,
            (r01 + r10) / scale,
            0.25 * scale,
            (r12 + r21) / scale,
        )
    else:
        scale = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
        quaternion = (
            (r10 - r01) / scale,
            (r02 + r20) / scale,
            (r12 + r21) / scale,
            0.25 * scale,
        )
    return _canonical_quaternion(quaternion)


def _canonical_quaternion(quaternion: Quat) -> Quat:
    """Normalize a quaternion and choose one deterministic sign representation."""
    unit = quat_normalize(quaternion)
    for component in unit:
        if abs(component) <= _ZERO_TOLERANCE:
            continue
        if component < 0.0:
            unit = (-unit[0], -unit[1], -unit[2], -unit[3])
        break
    return (
        _clean_scalar(unit[0]),
        _clean_scalar(unit[1]),
        _clean_scalar(unit[2]),
        _clean_scalar(unit[3]),
    )


def _require_finite_vector(vector: Vec3) -> None:
    if not all(math.isfinite(component) for component in vector):
        raise ValueError("vector components must be finite")


def _require_finite_quaternion(quaternion: Quat) -> None:
    if not all(math.isfinite(component) for component in quaternion):
        raise ValueError("quaternion components must be finite")


def _clean_vector(vector: Vec3) -> Vec3:
    return (
        _clean_scalar(vector[0]),
        _clean_scalar(vector[1]),
        _clean_scalar(vector[2]),
    )


def _clean_scalar(value: float) -> float:
    return 0.0 if abs(value) <= _ZERO_TOLERANCE else value


_CUBE_ORIENTATIONS = _cube_orientation_catalog()


__all__ = ["CubicLatticeBuilder"]
