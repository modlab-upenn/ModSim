"""Built-in module-topology graph generation."""

from __future__ import annotations

from collections.abc import Hashable

from modsim.core.entities import ConnectionRuntime
from modsim.core.state import WorldState
from modsim.model_views.base import (
    ModelViewBuilder,
    ModelViewBuildError,
    ModelViewContext,
    ModelViewUnavailableError,
)
from modsim.model_views.models import (
    DockedConnectionEdge,
    ModelViewSourceStamp,
    ModuleGraphNode,
    ModuleTopologyGraphView,
    SurfaceAttachmentDTO,
)
from modsim.robot_packs.schema import ModelViewMode, ModelViewSpec


class ModuleTopologyGraphBuilder(ModelViewBuilder[ModuleTopologyGraphView]):
    """Build an undirected module multigraph from active world connections."""

    @property
    def builder_id(self) -> str:
        return "module_topology_graph"

    @property
    def display_name(self) -> str:
        return "Module topology graph"

    @property
    def view_type(self) -> str:
        return "module_topology_graph"

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
                "builder 'module_topology_graph' requires a runtime WorldState"
            )
        revision = world.revision
        return (revision.sample_sequence, revision.topology_revision)

    def build(
        self,
        recipe: ModelViewSpec,
        context: ModelViewContext,
        source: ModelViewSourceStamp,
    ) -> ModuleTopologyGraphView:
        world = context.world
        if world is None:
            raise ModelViewUnavailableError(
                "builder 'module_topology_graph' requires a runtime WorldState"
            )
        if recipe.configuration:
            names = ", ".join(sorted(recipe.configuration))
            raise ModelViewBuildError(
                f"builder 'module_topology_graph' does not support configuration fields: {names}"
            )

        nodes = tuple(
            ModuleGraphNode(
                id=str(module.id),
                label=str(module.id),
                module_type_id=module.module_type_id,
                assembly_id=str(world.assemblies.assembly_of(module.id)),
                world_position_m=module.pose.translation,
                world_orientation_wxyz=module.pose.rotation,
            )
            for module in sorted(world.modules.values(), key=lambda item: str(item.id))
        )
        edges = tuple(
            _connection_edge(connection, world)
            for connection in sorted(world.connections.values(), key=lambda item: str(item.id))
        )
        return ModuleTopologyGraphView(
            id=recipe.id,
            name=recipe.name or self.display_name,
            builder=self.builder_id,
            source=source,
            nodes=nodes,
            edges=edges,
        )


def surface_attachment_dto(
    world: WorldState,
    connection: ConnectionRuntime,
) -> SurfaceAttachmentDTO | None:
    """Return a connection's live surface attachment as a view DTO, if it has one."""
    if connection.realization is None:
        return None
    attachment = world.surface_attachment(connection.id)
    if attachment is None:
        return None
    normal = attachment.normal_local
    return SurfaceAttachmentDTO(
        normal_local=(float(normal[0]), float(normal[1]), float(normal[2])),
        polar_rad=attachment.polar_rad,
        azimuth_rad=attachment.azimuth_rad,
        spin_rad=attachment.spin_rad,
        spin_reference_degenerate=attachment.spin_reference_degenerate,
        seat_gap_m=attachment.seat_gap_m,
        magnet_height_m=attachment.magnet_height_m,
    )


def _connection_edge(connection: ConnectionRuntime, world: WorldState) -> DockedConnectionEdge:
    """Return an edge whose undirected endpoints have canonical ordering."""
    endpoints = (
        (str(connection.module_a), str(connection.connector_a)),
        (str(connection.module_b), str(connection.connector_b)),
    )
    (source, connector_a), (target, connector_b) = sorted(endpoints)
    return DockedConnectionEdge(
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
        realization=connection.realization,
        attachment=surface_attachment_dto(world, connection),
    )
