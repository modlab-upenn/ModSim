"""Immutable, backend-neutral model-view result objects.

The DTOs in this module are the public boundary between model-view builders
and consumers such as algorithms, command-line tools, Studio, or a future
remote frontend.  They deliberately contain only JSON-safe scalar values and
tuples; no mutable runtime entities or renderer-specific objects escape into a
generated view.
"""

from __future__ import annotations

from typing import Generic, Literal, Self, TypeVar

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from modsim.robot_packs.schema import PhysicalConstraintType

MODEL_VIEW_SCHEMA_VERSION: Literal["0.1"] = "0.1"


class ModelViewDTO(BaseModel):
    """Strict immutable base for every serialized model-view object."""

    model_config = ConfigDict(
        allow_inf_nan=False,
        extra="forbid",
        frozen=True,
        strict=True,
        str_strip_whitespace=True,
        validate_default=True,
    )


class ModelViewSourceStamp(ModelViewDTO):
    """Exact source revisions from which a model view was generated."""

    pack_id: str = Field(min_length=1)
    pack_version: str = Field(min_length=1)
    world_time_s: float | None = None
    sample_sequence: int | None = Field(default=None, ge=0)
    topology_revision: int | None = Field(default=None, ge=0)
    docking_revision: int | None = Field(default=None, ge=0)
    event_revision: int | None = Field(default=None, ge=0)


class ModelView(ModelViewDTO):
    """Common envelope for one named, generated model view."""

    schema_version: Literal["0.1"] = MODEL_VIEW_SCHEMA_VERSION
    id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    builder: str = Field(min_length=1)
    view_type: str = Field(min_length=1)
    source: ModelViewSourceStamp


class GraphNode(ModelViewDTO):
    """Renderer-neutral base node in a graph model view."""

    id: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    label: str = Field(min_length=1)


class ModuleGraphNode(GraphNode):
    """One runtime module represented as a topology-graph node."""

    kind: str = Field(default="module", pattern=r"^module$")
    module_type_id: str = Field(min_length=1)
    assembly_id: str = Field(min_length=1)
    world_position_m: tuple[float, float, float]
    world_orientation_wxyz: tuple[float, float, float, float]


class GraphEdge(ModelViewDTO):
    """Renderer-neutral base edge in a graph model view."""

    id: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    source: str = Field(min_length=1)
    target: str = Field(min_length=1)


class DockedConnectionEdge(GraphEdge):
    """One active connector connection represented as an undirected edge."""

    kind: str = Field(default="docked_connection", pattern=r"^docked_connection$")
    connection_id: str = Field(min_length=1)
    connector_a: str = Field(min_length=1)
    connector_b: str = Field(min_length=1)
    constraint: PhysicalConstraintType = PhysicalConstraintType.FIXED
    orientation_rad: float
    orientation_index: int | None = Field(default=None, ge=0)
    created_at_s: float = Field(ge=0.0)


NodeT = TypeVar("NodeT", bound=GraphNode)
EdgeT = TypeVar("EdgeT", bound=GraphEdge)


class GraphModelView(ModelView, Generic[NodeT, EdgeT]):
    """Generic graph result with explicit graph semantics."""

    representation: Literal["graph"] = "graph"
    directed: bool
    allows_parallel_edges: bool
    nodes: tuple[NodeT, ...]
    edges: tuple[EdgeT, ...]


class ModuleTopologyGraphView(GraphModelView[ModuleGraphNode, DockedConnectionEdge]):
    """Module-level multigraph generated from current logical connections."""

    view_type: str = Field(default="module_topology_graph", pattern=r"^module_topology_graph$")
    directed: bool = False
    allows_parallel_edges: bool = True

    @field_validator("directed")
    @classmethod
    def require_undirected(cls, value: bool) -> bool:
        if value:
            raise ValueError("a module topology graph must be undirected")
        return value

    @field_validator("allows_parallel_edges")
    @classmethod
    def require_parallel_edges(cls, value: bool) -> bool:
        if not value:
            raise ValueError("a module topology graph must preserve parallel connections")
        return value


LatticeFace = Literal[
    "positive_x",
    "negative_x",
    "positive_y",
    "negative_y",
    "positive_z",
    "negative_z",
]


class CubicLatticeOrientation(ModelViewDTO):
    """One proper axis-aligned orientation in the 24-element cube rotation group."""

    index: int = Field(ge=0, le=23)
    id: str = Field(min_length=1)
    local_x_face: LatticeFace
    local_y_face: LatticeFace
    local_z_face: LatticeFace
    lattice_orientation_wxyz: tuple[float, float, float, float]


class CubicLatticePoseResidual(ModelViewDTO):
    """Difference between a measured module pose and its nearest lattice pose.

    Translation is expressed along the lattice axes. Orientation is the
    shortest angular distance to the selected cube-group orientation.
    """

    translation_m: tuple[float, float, float]
    position_m: float = Field(ge=0.0)
    orientation_rad: float = Field(ge=0.0)


class CubicLatticeNode(ModuleGraphNode):
    """One module quantized onto its nearest cubic-lattice pose."""

    cell: tuple[int, int, int]
    orientation_index: int = Field(ge=0, le=23)
    orientation_id: str = Field(min_length=1)
    pose_residual: CubicLatticePoseResidual
    position_within_tolerance: bool
    orientation_within_tolerance: bool
    off_lattice: bool
    occupancy_conflict: bool

    @model_validator(mode="after")
    def require_consistent_membership(self) -> Self:
        expected = not (self.position_within_tolerance and self.orientation_within_tolerance)
        if self.off_lattice != expected:
            raise ValueError("off_lattice must reflect the position and orientation tolerances")
        return self


class CubicLatticeConnectionEdge(DockedConnectionEdge):
    """A docked connection whose endpoints are labelled by lattice-facing sides."""

    source_face: LatticeFace
    target_face: LatticeFace
    source_face_residual_rad: float = Field(ge=0.0)
    target_face_residual_rad: float = Field(ge=0.0)


class CubicLatticeOccupancyConflict(ModelViewDTO):
    """Two or more position-valid modules quantized to the same lattice cell."""

    cell: tuple[int, int, int]
    module_ids: tuple[str, ...] = Field(min_length=2)

    @field_validator("module_ids")
    @classmethod
    def require_unique_sorted_module_ids(cls, value: tuple[str, ...]) -> tuple[str, ...]:
        if len(set(value)) != len(value):
            raise ValueError("occupancy-conflict module IDs must be unique")
        if value != tuple(sorted(value)):
            raise ValueError("occupancy-conflict module IDs must use deterministic order")
        return value


class CubicLatticeView(GraphModelView[CubicLatticeNode, CubicLatticeConnectionEdge]):
    """Spatial module graph quantized onto a configurable cubic lattice."""

    view_type: str = Field(default="cubic_lattice", pattern=r"^cubic_lattice$")
    directed: bool = False
    allows_parallel_edges: bool = True
    pitch_m: float = Field(gt=0.0)
    origin_world_m: tuple[float, float, float]
    orientation_world_wxyz: tuple[float, float, float, float]
    position_tolerance_m: float = Field(ge=0.0)
    orientation_tolerance_rad: float = Field(ge=0.0)
    orientation_catalog: tuple[CubicLatticeOrientation, ...] = Field(
        min_length=24,
        max_length=24,
    )
    occupancy_conflicts: tuple[CubicLatticeOccupancyConflict, ...]

    @field_validator("directed")
    @classmethod
    def require_undirected(cls, value: bool) -> bool:
        if value:
            raise ValueError("a cubic lattice view must be undirected")
        return value

    @field_validator("allows_parallel_edges")
    @classmethod
    def require_parallel_edges(cls, value: bool) -> bool:
        if not value:
            raise ValueError("a cubic lattice view must preserve parallel connections")
        return value

    @field_validator("orientation_catalog")
    @classmethod
    def require_complete_orientation_catalog(
        cls,
        value: tuple[CubicLatticeOrientation, ...],
    ) -> tuple[CubicLatticeOrientation, ...]:
        if tuple(item.index for item in value) != tuple(range(24)):
            raise ValueError("cube orientation catalog must contain ordered indices 0 through 23")
        if len({item.id for item in value}) != 24:
            raise ValueError("cube orientation catalog IDs must be unique")
        return value

    @model_validator(mode="after")
    def require_consistent_orientation_and_occupancy_references(self) -> Self:
        orientations = {item.index: item.id for item in self.orientation_catalog}
        for node in self.nodes:
            if orientations[node.orientation_index] != node.orientation_id:
                raise ValueError(
                    f"node '{node.id}' orientation ID does not match its catalog index"
                )

        conflicted = {
            module_id for conflict in self.occupancy_conflicts for module_id in conflict.module_ids
        }
        if any(node.occupancy_conflict != (node.id in conflicted) for node in self.nodes):
            raise ValueError("node occupancy flags must match the occupancy-conflict catalog")
        return self
