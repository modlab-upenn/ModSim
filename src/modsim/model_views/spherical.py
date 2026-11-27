"""Built-in discrete identification of attachments on spherical surfaces.

A continuous attachment is canonical state (docs/continuous_docking.md 6).
This builder derives a *discrete* identity from it, as the cubic lattice does
for module poses: each attachment is assigned to one face of a subdivided
icosahedron in the surface connector's frame, and to the nearest grid vertex.
The ``icosphere_2`` grid (frequency 2) has 42 vertices and 80 spherical
triangles, the division the FreeSN configuration-identification work uses.
Nothing here writes to ``WorldState``.
"""

from __future__ import annotations

import math
from collections.abc import Hashable
from typing import Annotated, cast

from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    StrictFloat,
    ValidationError,
    field_validator,
)

from modsim.core.ids import ConnectorInstanceId
from modsim.core.state import WorldState
from modsim.core.transforms import (
    Quat,
    Vec3,
    angle_between,
    quat_normalize,
    quat_rotate,
    vec_add,
    vec_cross,
    vec_dot,
    vec_normalize,
    vec_scale,
)
from modsim.model_views.base import (
    ModelViewBuilder,
    ModelViewBuildError,
    ModelViewContext,
    ModelViewUnavailableError,
)
from modsim.model_views.models import (
    ModelViewSourceStamp,
    SphericalAttachmentCell,
    SphericalAttachmentView,
    SphericalGridFace,
    SphericalSubdivision,
    SphericalSurfaceOccupancy,
)
from modsim.model_views.topology import surface_attachment_dto
from modsim.robot_packs.schema import ModelViewMode, ModelViewSpec, SurfaceShape

_QUATERNION_TOLERANCE = 1e-6
_ZERO_TOLERANCE = 1e-12
_GOLDEN = (1.0 + math.sqrt(5.0)) / 2.0

# The regular icosahedron, vertices (0, +-1, +-phi) and their cyclic
# permutations, faces wound counter-clockwise seen from outside.
_ICOSAHEDRON_VERTICES: tuple[Vec3, ...] = (
    (-1.0, _GOLDEN, 0.0),
    (1.0, _GOLDEN, 0.0),
    (-1.0, -_GOLDEN, 0.0),
    (1.0, -_GOLDEN, 0.0),
    (0.0, -1.0, _GOLDEN),
    (0.0, 1.0, _GOLDEN),
    (0.0, -1.0, -_GOLDEN),
    (0.0, 1.0, -_GOLDEN),
    (_GOLDEN, 0.0, -1.0),
    (_GOLDEN, 0.0, 1.0),
    (-_GOLDEN, 0.0, -1.0),
    (-_GOLDEN, 0.0, 1.0),
)
_ICOSAHEDRON_FACES: tuple[tuple[int, int, int], ...] = (
    (0, 11, 5),
    (0, 5, 1),
    (0, 1, 7),
    (0, 7, 10),
    (0, 10, 11),
    (1, 5, 9),
    (5, 11, 4),
    (11, 10, 2),
    (10, 7, 6),
    (7, 1, 8),
    (3, 9, 4),
    (3, 4, 2),
    (3, 2, 6),
    (3, 6, 8),
    (3, 8, 9),
    (4, 9, 5),
    (2, 4, 11),
    (6, 2, 10),
    (8, 6, 7),
    (9, 8, 1),
)
_SUBDIVISION_LEVELS: dict[str, int] = {"icosphere_2": 1}
"""Midpoint subdivisions of the icosahedron for each supported grid."""


def _json_array_as_tuple(value: object) -> object:
    if isinstance(value, list):
        return tuple(cast(list[object], value))
    return value


_ConfigurationQuaternion = Annotated[
    tuple[StrictFloat, StrictFloat, StrictFloat, StrictFloat],
    BeforeValidator(_json_array_as_tuple),
]


class _SphericalAttachmentConfiguration(BaseModel):
    """Strict recipe configuration consumed by :class:`SphericalAttachmentBuilder`."""

    model_config = ConfigDict(
        allow_inf_nan=False,
        extra="forbid",
        frozen=True,
        strict=True,
        validate_default=True,
    )

    subdivision: SphericalSubdivision = "icosphere_2"
    grid_orientation_local_wxyz: _ConfigurationQuaternion = (1.0, 0.0, 0.0, 0.0)
    """Rotation of the grid relative to each surface connector frame."""

    @field_validator("grid_orientation_local_wxyz")
    @classmethod
    def require_unit_orientation(cls, value: Quat) -> Quat:
        magnitude = math.sqrt(sum(component * component for component in value))
        if not math.isclose(
            magnitude, 1.0, rel_tol=_QUATERNION_TOLERANCE, abs_tol=_QUATERNION_TOLERANCE
        ):
            raise ValueError("grid orientation must be a unit quaternion")
        return quat_normalize(value)


class _Grid:
    """A geodesic triangulation of the unit sphere, in the surface connector frame."""

    __slots__ = ("edge_normals", "faces", "vertices")

    def __init__(self, subdivision: str, orientation: Quat) -> None:
        unit, faces = icosphere(_SUBDIVISION_LEVELS[subdivision])
        self.vertices = tuple(_clean_vector(quat_rotate(orientation, v)) for v in unit)
        self.faces = tuple(
            SphericalGridFace(
                index=index,
                vertices=face,
                centroid_local=_clean_vector(
                    vec_normalize(
                        vec_add(
                            vec_add(self.vertices[face[0]], self.vertices[face[1]]),
                            self.vertices[face[2]],
                        )
                    )
                ),
            )
            for index, face in enumerate(faces)
        )
        # The inward normal of each edge's great-circle plane. A direction is
        # inside a face when it is on the inner side of all three planes.
        self.edge_normals = tuple(
            tuple(
                vec_normalize(vec_cross(self.vertices[a], self.vertices[b]))
                for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0]))
            )
            for f in faces
        )

    def face_of(self, direction: Vec3) -> int:
        """Return the face containing ``direction``; ties go to the lowest index.

        Each face scores the smallest signed distance from ``direction`` to its
        three edge planes, which is non-negative exactly when the direction is
        inside. The highest score wins, so a direction on a shared edge or
        vertex, or one just outside every face by rounding, still gets a face.
        """
        best_index, best_score = 0, -math.inf
        for index, normals in enumerate(self.edge_normals):
            score = min(vec_dot(normal, direction) for normal in normals)
            if score > best_score + _ZERO_TOLERANCE:
                best_index, best_score = index, score
        return best_index

    def nearest_vertex(self, direction: Vec3) -> int:
        """Return the grid vertex closest to ``direction``; ties go to the lowest index."""
        best_index, best_dot = 0, -math.inf
        for index, vertex in enumerate(self.vertices):
            dot = vec_dot(vertex, direction)
            if dot > best_dot + _ZERO_TOLERANCE:
                best_index, best_dot = index, dot
        return best_index


def icosphere(levels: int) -> tuple[tuple[Vec3, ...], tuple[tuple[int, int, int], ...]]:
    """Return the unit vertices and outward-wound faces of a subdivided icosahedron.

    Each level splits every triangle into four at its edge midpoints. Vertex
    and face order is deterministic: the twelve icosahedron vertices first,
    then midpoints in the order they are first met; each parent face is
    replaced by its three corner children followed by its centre child.
    """
    vertices = [vec_normalize(vertex) for vertex in _ICOSAHEDRON_VERTICES]
    faces = list(_ICOSAHEDRON_FACES)
    for _ in range(levels):
        midpoints: dict[tuple[int, int], int] = {}
        children: list[tuple[int, int, int]] = []
        for a, b, c in faces:
            ab = _midpoint(vertices, midpoints, a, b)
            bc = _midpoint(vertices, midpoints, b, c)
            ca = _midpoint(vertices, midpoints, c, a)
            children.extend(((a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)))
        faces = children
    return tuple(vertices), tuple(faces)


def _midpoint(vertices: list[Vec3], midpoints: dict[tuple[int, int], int], a: int, b: int) -> int:
    """Return the index of the unit midpoint of edge ``ab``, adding it once."""
    key = (a, b) if a < b else (b, a)
    index = midpoints.get(key)
    if index is None:
        index = len(vertices)
        vertices.append(vec_normalize(vec_scale(vec_add(vertices[a], vertices[b]), 0.5)))
        midpoints[key] = index
    return index


class SphericalAttachmentBuilder(ModelViewBuilder[SphericalAttachmentView]):
    """Assign every surface attachment to a face and vertex of a geodesic grid."""

    @property
    def builder_id(self) -> str:
        return "spherical_attachment"

    @property
    def display_name(self) -> str:
        return "Spherical attachment"

    @property
    def view_type(self) -> str:
        return "spherical_attachment"

    @property
    def modes(self) -> tuple[ModelViewMode, ...]:
        return (ModelViewMode.RUNTIME,)

    @property
    def requires_world(self) -> bool:
        return True

    def cache_token(self, context: ModelViewContext) -> Hashable:
        world = context.world
        if world is None:
            raise ModelViewUnavailableError(
                "builder 'spherical_attachment' requires a runtime WorldState"
            )
        revision = world.revision
        return (revision.sample_sequence, revision.topology_revision)

    def build(
        self,
        recipe: ModelViewSpec,
        context: ModelViewContext,
        source: ModelViewSourceStamp,
    ) -> SphericalAttachmentView:
        world = context.world
        if world is None:
            raise ModelViewUnavailableError(
                "builder 'spherical_attachment' requires a runtime WorldState"
            )
        configuration = _parse_configuration(recipe)
        grid = _Grid(configuration.subdivision, configuration.grid_orientation_local_wxyz)
        surfaces = tuple(
            _surface_occupancy(world, surface_id, grid)
            for surface_id in world.surface_connectors
            if _is_sphere(world, surface_id)
        )
        return SphericalAttachmentView(
            id=recipe.id,
            name=recipe.name or self.display_name,
            builder=self.builder_id,
            source=source,
            subdivision=configuration.subdivision,
            grid_orientation_local_wxyz=configuration.grid_orientation_local_wxyz,
            vertices_local=grid.vertices,
            faces=grid.faces,
            surfaces=surfaces,
        )


def _parse_configuration(recipe: ModelViewSpec) -> _SphericalAttachmentConfiguration:
    try:
        return _SphericalAttachmentConfiguration.model_validate(recipe.configuration)
    except ValidationError as error:
        details = "; ".join(
            f"{'.'.join(str(part) for part in item['loc'])}: {item['msg']}"
            for item in error.errors(include_url=False)
        )
        raise ModelViewBuildError(
            f"builder 'spherical_attachment' received invalid configuration for "
            f"recipe '{recipe.id}': {details}"
        ) from None


def _is_sphere(world: WorldState, surface_id: ConnectorInstanceId) -> bool:
    spec = world.connector_type(surface_id).surface
    return spec is not None and spec.shape is SurfaceShape.SPHERE


def _surface_occupancy(
    world: WorldState,
    surface_id: ConnectorInstanceId,
    grid: _Grid,
) -> SphericalSurfaceOccupancy:
    surface = world.connector(surface_id)
    spec = world.connector_type(surface_id).surface
    if spec is None:  # pragma: no cover - filtered by the caller
        raise ModelViewBuildError(f"connector '{surface_id}' is not a surface connector")
    cells: list[SphericalAttachmentCell] = []
    for connection_id in surface.connection_ids:
        connection = world.connections[connection_id]
        attachment = surface_attachment_dto(world, connection)
        if attachment is None:
            # Undefined in this sample (an unresolved frame); nothing to place.
            continue
        normal = attachment.normal_local
        face_index = grid.face_of(normal)
        vertex_index = grid.nearest_vertex(normal)
        cells.append(
            SphericalAttachmentCell(
                connection_id=str(connection.id),
                point_module=str(connection.module_a),
                point_connector=str(connection.connector_a),
                normal_local=normal,
                face_index=face_index,
                face_residual_rad=_clean_scalar(
                    angle_between(normal, grid.faces[face_index].centroid_local)
                ),
                vertex_index=vertex_index,
                vertex_residual_rad=_clean_scalar(
                    angle_between(normal, grid.vertices[vertex_index])
                ),
                attachment=attachment,
            )
        )
    cells.sort(key=lambda cell: cell.connection_id)
    return SphericalSurfaceOccupancy(
        module_id=str(surface.module_id),
        connector_id=str(surface_id),
        capacity=spec.capacity,
        attachments=tuple(cells),
    )


def _clean_vector(vector: Vec3) -> Vec3:
    return (_clean_scalar(vector[0]), _clean_scalar(vector[1]), _clean_scalar(vector[2]))


def _clean_scalar(value: float) -> float:
    return 0.0 if abs(value) <= _ZERO_TOLERANCE else float(value)


__all__ = ["SphericalAttachmentBuilder", "icosphere"]
