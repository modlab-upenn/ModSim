"""Canonical modular-robot world state.

``WorldState`` owns module instances, connector instances, logical connections,
and the derived assembly index. It is mutated only by applying events, so the
event log is a complete and replayable description of everything that happened.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass

from modsim.core.assemblies import AssemblyIndex
from modsim.core.entities import (
    ConnectionRuntime,
    ConnectorInstance,
    ConnectorLifecycleState,
    ModuleInstance,
)
from modsim.core.events import (
    AssemblyMerged,
    AssemblySplit,
    ConnectorOverloaded,
    DockCommitted,
    DockFailed,
    Event,
    EventLog,
    UndockCommitted,
)
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ModuleInstanceId,
    connector_instance_id,
)
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot
from modsim.core.transforms import Transform, quat_rotate
from modsim.robot_packs.schema import ConnectorSpec, ConnectorTypeSpec, RobotPack


class WorldStateError(RuntimeError):
    """Raised when an event cannot be applied to the world."""


@dataclass(frozen=True, slots=True)
class WorldStateRevision:
    """Immutable change stamp for incrementally derived world views.

    Each counter identifies an independent class of source changes so a
    consumer can invalidate only the projections affected by that change.
    """

    sample_sequence: int = 0
    topology_revision: int = 0
    docking_revision: int = 0
    event_revision: int = 0


class WorldState:
    """All modules, connectors, connections, and derived assemblies in one world."""

    __slots__ = (
        "_assemblies",
        "_connections",
        "_connectors",
        "_connectors_by_module",
        "_event_log",
        "_modules",
        "_pack",
        "_revision",
        "_time_s",
    )

    def __init__(self, pack: RobotPack) -> None:
        self._pack = pack
        self._time_s = 0.0
        self._modules: dict[ModuleInstanceId, ModuleInstance] = {}
        self._connectors: dict[ConnectorInstanceId, ConnectorInstance] = {}
        self._connectors_by_module: dict[ModuleInstanceId, tuple[ConnectorInstanceId, ...]] = {}
        self._connections: dict[ConnectionId, ConnectionRuntime] = {}
        self._assemblies = AssemblyIndex()
        self._event_log = EventLog()
        self._revision = WorldStateRevision()

    # ------------------------------------------------------------------
    # construction
    # ------------------------------------------------------------------

    @classmethod
    def from_scene(cls, pack: RobotPack, scene: SceneSpec) -> WorldState:
        """Instantiate every placement in ``scene`` as a free module."""
        scene.validate_against(pack)
        world = cls(pack)
        for placement in scene.placements:
            module_type = pack.hardware_catalog.module_types[placement.module_type_id]
            module = ModuleInstance(
                id=placement.instance_id,
                module_type_id=placement.module_type_id,
                pose=placement.pose,
                link_poses={module_type.root_link: placement.pose},
            )
            world._modules[module.id] = module
            connector_ids: list[ConnectorInstanceId] = []
            for connector_spec in module_type.connectors:
                instance = world._build_connector(module.id, connector_spec)
                world._connectors[instance.id] = instance
                connector_ids.append(instance.id)
            world._connectors_by_module[module.id] = tuple(connector_ids)
        world._assemblies.rebuild(world._modules, {})
        world._refresh_connector_frames()
        return world

    @staticmethod
    def _build_connector(
        module_id: ModuleInstanceId,
        spec: ConnectorSpec,
    ) -> ConnectorInstance:
        local_pose = (
            Transform.from_pose_spec(spec.local_pose)
            if spec.local_pose is not None
            else Transform.identity()
        )
        return ConnectorInstance(
            id=connector_instance_id(module_id, spec.id),
            module_id=module_id,
            connector_id=spec.id,
            connector_type_id=spec.connector_type,
            parent_link=spec.parent_link,
            local_pose=local_pose,
            world_docking_axis=spec.docking_axis or (1.0, 0.0, 0.0),
            world_approach_axis=spec.approach_axis or spec.docking_axis or (1.0, 0.0, 0.0),
        )

    # ------------------------------------------------------------------
    # accessors
    # ------------------------------------------------------------------

    @property
    def pack(self) -> RobotPack:
        """Return the Robot Pack backing this world."""
        return self._pack

    @property
    def time_s(self) -> float:
        """Return the most recently ingested backend time."""
        return self._time_s

    @property
    def modules(self) -> Mapping[ModuleInstanceId, ModuleInstance]:
        """Return every module instance."""
        return self._modules

    @property
    def connectors(self) -> Mapping[ConnectorInstanceId, ConnectorInstance]:
        """Return every connector instance."""
        return self._connectors

    @property
    def connections(self) -> Mapping[ConnectionId, ConnectionRuntime]:
        """Return every active logical connection."""
        return self._connections

    @property
    def assemblies(self) -> AssemblyIndex:
        """Return the derived assembly index."""
        return self._assemblies

    @property
    def event_log(self) -> EventLog:
        """Return the append-only event log."""
        return self._event_log

    @property
    def revision(self) -> WorldStateRevision:
        """Return immutable revision counters for the current world state."""
        return self._revision

    def connector(self, connector: ConnectorInstanceId) -> ConnectorInstance:
        """Return one connector instance."""
        try:
            return self._connectors[connector]
        except KeyError as error:
            raise KeyError(f"unknown connector '{connector}'") from error

    def connector_type(self, connector: ConnectorInstanceId) -> ConnectorTypeSpec:
        """Return the Robot Pack connector type backing one connector instance."""
        type_id = self.connector(connector).connector_type_id
        try:
            return self._pack.hardware_catalog.connector_types[type_id]
        except KeyError as error:
            raise KeyError(f"connector type '{type_id}' is not defined by the pack") from error

    def adjacency(
        self, *, exclude: ConnectionId | None = None
    ) -> dict[ModuleInstanceId, frozenset[ModuleInstanceId]]:
        """Return module-level adjacency implied by the active connections."""
        neighbours: dict[ModuleInstanceId, set[ModuleInstanceId]] = {
            module_id: set() for module_id in self._modules
        }
        for connection in self._connections.values():
            if connection.id == exclude or connection.module_a == connection.module_b:
                continue
            neighbours[connection.module_a].add(connection.module_b)
            neighbours[connection.module_b].add(connection.module_a)
        return {module_id: frozenset(values) for module_id, values in neighbours.items()}

    # ------------------------------------------------------------------
    # snapshot ingestion
    # ------------------------------------------------------------------

    def ingest(self, snapshot: BackendStateSnapshot) -> None:
        """Update module and connector kinematics from a backend observation."""
        self._time_s = snapshot.time_s
        for module_id, module in self._modules.items():
            links = snapshot.link_states.get(module_id)
            if links is None:
                continue
            module.link_poses = {name: body.pose for name, body in links.items()}
            module_type = self._pack.hardware_catalog.module_types[module.module_type_id]
            root = links.get(module_type.root_link)
            if root is not None:
                module.pose = root.pose
                module.linear_velocity_m_s = root.linear_velocity_m_s
                module.angular_velocity_rad_s = root.angular_velocity_rad_s
        self._refresh_connector_frames(snapshot)
        self._clear_transient_states()
        self._advance_revision(sample=True)

    def _refresh_connector_frames(self, snapshot: BackendStateSnapshot | None = None) -> None:
        for connector in self._connectors.values():
            module = self._modules[connector.module_id]
            module_type = self._pack.hardware_catalog.module_types[module.module_type_id]
            spec = self._connector_spec(module_type.connectors, connector.connector_id)
            link_pose = module.link_poses.get(connector.parent_link)
            if link_pose is None:
                connector.resolved = False
                continue

            measured = None
            if snapshot is not None:
                measured = snapshot.connector_frames.get(connector.module_id, {}).get(
                    connector.connector_id
                )
            connector.world_pose = (
                measured if measured is not None else link_pose.compose(connector.local_pose)
            )

            # Docking and approach axes are authored in the parent-link frame,
            # not the connector frame, so they rotate with the link.
            if spec.docking_axis is not None:
                connector.world_docking_axis = quat_rotate(link_pose.rotation, spec.docking_axis)
            approach = spec.approach_axis or spec.docking_axis
            if approach is not None:
                connector.world_approach_axis = quat_rotate(link_pose.rotation, approach)

            body = None
            if snapshot is not None:
                body = snapshot.body(connector.module_id, connector.parent_link)
            connector.world_velocity_m_s = (
                body.velocity_at(connector.world_pose.translation)
                if body is not None
                else module.linear_velocity_m_s
            )
            connector.resolved = True

    def _clear_transient_states(self) -> None:
        """Reset per-step lifecycle states so a stale detection cannot persist.

        ``CANDIDATE_DETECTED`` and ``IN_ACCEPTANCE_REGION`` describe the current
        step only. ``FAILED`` is also cleared here rather than requiring an
        explicit reset, so a rejected attempt cannot strand a connector.
        """
        transient = {
            ConnectorLifecycleState.CANDIDATE_DETECTED,
            ConnectorLifecycleState.IN_ACCEPTANCE_REGION,
            ConnectorLifecycleState.FAILED,
        }
        changed = False
        for connector in self._connectors.values():
            if connector.lifecycle_state in transient:
                connector.lifecycle_state = ConnectorLifecycleState.FREE
                changed = True
        if changed:
            self._advance_revision(docking=True)

    @staticmethod
    def _connector_spec(specs: Iterable[ConnectorSpec], connector_id: str) -> ConnectorSpec:
        for spec in specs:
            if spec.id == connector_id:
                return spec
        raise WorldStateError(f"module type no longer defines connector '{connector_id}'")

    # ------------------------------------------------------------------
    # event application
    # ------------------------------------------------------------------

    def apply(self, event: Event) -> Event:
        """Apply one event to the world and record it in the log."""
        topology_changed = False
        docking_changed = False
        if isinstance(event, DockCommitted):
            self._apply_dock_committed(event)
            topology_changed = True
            docking_changed = True
        elif isinstance(event, UndockCommitted):
            self._apply_undock_committed(event)
            topology_changed = True
            docking_changed = True
        elif isinstance(event, DockFailed):
            docking_changed = self._apply_dock_failed(event)
        elif isinstance(event, ConnectorOverloaded):
            docking_changed = self._apply_overload(event)
        elif not isinstance(event, AssemblyMerged | AssemblySplit):
            # Detection and assembly events are observational; the assembly
            # index is updated alongside the dock or undock that caused them.
            pass
        recorded = self._event_log.append(event)
        self._advance_revision(
            topology=topology_changed,
            docking=docking_changed,
            event=True,
        )
        return recorded

    def _apply_dock_committed(self, event: DockCommitted) -> None:
        first = self.connector(event.connector_a)
        second = self.connector(event.connector_b)
        if first.is_engaged or second.is_engaged:
            raise WorldStateError(
                f"cannot dock '{event.connector_a}' to '{event.connector_b}': "
                "at least one connector is already engaged"
            )
        connection = ConnectionRuntime(
            id=event.connection_id,
            connector_a=event.connector_a,
            connector_b=event.connector_b,
            module_a=first.module_id,
            module_b=second.module_id,
            relative_transform=event.relative_transform,
            orientation_rad=event.orientation_rad,
            orientation_index=event.orientation_index,
            constraint_handle=event.constraint_handle,
            created_at_s=event.time_s,
        )
        self._connections[connection.id] = connection
        for connector in (first, second):
            connector.lifecycle_state = ConnectorLifecycleState.DOCKED
            connector.connection_id = connection.id
        self._assemblies.merge_on_connection(first.module_id, second.module_id)

    def _apply_undock_committed(self, event: UndockCommitted) -> None:
        connection = self._connections.pop(event.connection_id, None)
        if connection is None:
            raise WorldStateError(f"unknown connection '{event.connection_id}'")
        for connector_id in connection.connectors:
            connector = self.connector(connector_id)
            connector.lifecycle_state = ConnectorLifecycleState.FREE
            connector.connection_id = None
            connector.available_at_s = event.time_s + self._cooldown_s(connector_id)
        if connection.module_a != connection.module_b:
            self._assemblies.split_on_disconnection(
                connection.module_a,
                connection.module_b,
                self.adjacency(),
            )

    def _apply_dock_failed(self, event: DockFailed) -> bool:
        changed = False
        for connector_id in (event.connector_a, event.connector_b):
            connector = self._connectors.get(connector_id)
            if connector is None or connector.is_engaged:
                continue
            available_at_s = event.time_s + self._cooldown_s(connector_id)
            if (
                connector.lifecycle_state is not ConnectorLifecycleState.FAILED
                or connector.available_at_s != available_at_s
            ):
                connector.lifecycle_state = ConnectorLifecycleState.FAILED
                connector.available_at_s = available_at_s
                changed = True
        return changed

    def _apply_overload(self, event: ConnectorOverloaded) -> bool:
        connection = self._connections.get(event.connection_id)
        if connection is None or connection.measured_force_n == event.measured_force_n:
            return False
        connection.measured_force_n = event.measured_force_n
        return True

    def _cooldown_s(self, connector_id: ConnectorInstanceId) -> float:
        try:
            policy = self.connector_type(connector_id).effective_docking_policy
        except KeyError:
            return 0.0
        return policy.redock_cooldown_s or 0.0

    # ------------------------------------------------------------------
    # lifecycle marking used by the docking engine
    # ------------------------------------------------------------------

    def mark_lifecycle(
        self,
        connector_id: ConnectorInstanceId,
        state: ConnectorLifecycleState,
    ) -> None:
        """Set a transient lifecycle state without recording an event.

        Only detection states may be set this way. States that imply a logical
        connection must arrive through :meth:`apply` so that the event log stays
        a complete description of the world.
        """
        allowed = {
            ConnectorLifecycleState.FREE,
            ConnectorLifecycleState.CANDIDATE_DETECTED,
            ConnectorLifecycleState.ALIGNING,
            ConnectorLifecycleState.IN_ACCEPTANCE_REGION,
        }
        if state not in allowed:
            raise WorldStateError(f"state '{state}' must be reached through an event")
        connector = self.connector(connector_id)
        if connector.is_engaged or connector.lifecycle_state is state:
            return
        connector.lifecycle_state = state
        self._advance_revision(docking=True)

    def _advance_revision(
        self,
        *,
        sample: bool = False,
        topology: bool = False,
        docking: bool = False,
        event: bool = False,
    ) -> None:
        """Advance selected revision domains as one atomic state stamp."""
        current = self._revision
        self._revision = WorldStateRevision(
            sample_sequence=current.sample_sequence + int(sample),
            topology_revision=current.topology_revision + int(topology),
            docking_revision=current.docking_revision + int(docking),
            event_revision=current.event_revision + int(event),
        )
