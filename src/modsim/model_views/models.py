"""Immutable, backend-neutral model-view result objects.

The DTOs in this module are the public boundary between model-view builders
and consumers such as algorithms, command-line tools, Studio, or a future
remote frontend.  They deliberately contain only JSON-safe scalar values and
tuples; no mutable runtime entities or renderer-specific objects escape into a
generated view.
"""

from __future__ import annotations

from typing import Generic, Literal, TypeVar

from pydantic import BaseModel, ConfigDict, Field, field_validator

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
