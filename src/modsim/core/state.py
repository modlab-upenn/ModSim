"""Canonical modular-robot world state.

``WorldState`` owns module instances, connector instances, logical connections,
and the derived assembly index. It is mutated only by applying events, so the
event log is a complete and replayable description of everything that happened.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass

from modsim.core.assemblies import AssemblyIndex
from modsim.core.entities import (
    ConnectionRuntime,
    ConnectorInstance,
    ConnectorLifecycleState,
    ModuleInstance,
    SampleClock,
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
    JointInstanceId,
    ModuleInstanceId,
    connector_instance_id,
    split_joint_instance_id,
)
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot, FrozenMapping, JointState, LinkPoseView
from modsim.core.surface import SurfaceAttachment, attachment_from_contact, surface_contact
from modsim.core.transforms import Transform, Vec3, quat_rotate
from modsim.robot_packs.schema import ConnectorSpec, ConnectorTypeSpec, JointSpec, RobotPack


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
        "_advancing_samples",
        "_assemblies",
        "_attachments",
        "_connections",
        "_connectors",
        "_connectors_by_module",
        "_event_log",
        "_frame_clock",
        "_module_clock",
        "_modules",
        "_neighbours",
        "_pack",
        "_parent_links_by_type",
        "_revision",
        "_sample_snapshot",
        "_time_s",
    )

    def __init__(self, pack: RobotPack) -> None:
        self._pack = pack
        self._time_s = 0.0
        self._advancing_samples = 0
        self._modules: dict[ModuleInstanceId, ModuleInstance] = {}
        self._connectors: dict[ConnectorInstanceId, ConnectorInstance] = {}
        self._connectors_by_module: dict[ModuleInstanceId, tuple[ConnectorInstanceId, ...]] = {}
        self._connections: dict[ConnectionId, ConnectionRuntime] = {}
        # Module adjacency maintained per dock/undock: neighbour -> number of
        # active connections between the pair (two modules may share several).
        self._neighbours: dict[ModuleInstanceId, dict[ModuleInstanceId, int]] = {}
        self._assemblies = AssemblyIndex()
        self._event_log = EventLog()
        self._revision = WorldStateRevision()
        # Module root states and connector world frames are derived lazily
        # from the latest ingested sample (see ``ingest``).
        self._sample_snapshot: BackendStateSnapshot | None = None
        self._parent_links_by_type: dict[str, tuple[str, ...]] = {}
        self._module_clock = SampleClock[ModuleInstance](self._resolve_module_root)
        self._frame_clock = SampleClock[ConnectorInstance](self._resolve_connector_frame)
        # Surface attachments are derived from connector frames, so they are
        # cached against the frame clock and recomputed only when read.
        self._attachments: dict[ConnectionId, tuple[int, SurfaceAttachment | None]] = {}

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
            module.bind_sample_clock(world._module_clock)
            world._modules[module.id] = module
            connector_ids: list[ConnectorInstanceId] = []
            for connector_spec in module_type.connectors:
                instance = world._build_connector(
                    module.id,
                    connector_spec,
                    pack.hardware_catalog.connector_types.get(connector_spec.connector_type),
                )
                instance.bind_frame_clock(world._frame_clock)
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
        connector_type: ConnectorTypeSpec | None = None,
    ) -> ConnectorInstance:
        surface = connector_type.surface if connector_type is not None else None
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
            surface=surface is not None,
            capacity=surface.capacity if surface is not None else 1,
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
    def advancing_samples(self) -> int:
        """Return how many ingested samples advanced time, i.e. physics steps observed.

        Counted at ingest, so it is the same however a caller drives the
        backend. Several backend steps between two ingests count once, which
        errs on the side of treating a recent connection change as pending.
        """
        return self._advancing_samples

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

    def joint_spec(self, joint: JointInstanceId) -> JointSpec:
        """Return the Robot Pack joint specification backing one joint instance."""
        module_id, joint_id = split_joint_instance_id(joint)
        try:
            module = self._modules[module_id]
        except KeyError as error:
            raise KeyError(f"unknown module '{module_id}'") from error
        module_type = self._pack.hardware_catalog.module_types[module.module_type_id]
        for spec in module_type.joints:
            if spec.id == joint_id:
                return spec
        raise KeyError(f"module '{module_id}' has no joint '{joint_id}'")

    def joint_state(self, joint: JointInstanceId) -> JointState:
        """Return the most recently ingested state for one known joint.

        Knowing a joint from the Robot Pack and having a measurement for it are
        separate conditions. A backend may omit passive or unsupported joints;
        callers get a distinct error until such a state has been reported.
        """
        module_id, joint_id = split_joint_instance_id(joint)
        self.joint_spec(joint)
        try:
            return self._modules[module_id].joint_states[joint_id]
        except KeyError as error:
            raise KeyError(f"backend has not reported state for joint '{joint}'") from error

    def link_pose(self, module_id: ModuleInstanceId, link: str) -> Transform | None:
        """Return one link's pose from the latest sample, or ``None`` if unreported."""
        return self.module(module_id).link_poses.get(link)

    def point_velocity(self, module_id: ModuleInstanceId, link: str, world_point: Vec3) -> Vec3:
        """Return the world velocity of a point rigidly attached to one link.

        The lever-arm term is included, so a point on a spinning body reports
        its own velocity rather than the body origin's. Without a measured
        twist for the link, the module root's linear velocity is returned, the
        same fallback connector frames use.
        """
        module = self.module(module_id)
        snapshot = self._sample_snapshot
        body = snapshot.body(module_id, link) if snapshot is not None else None
        if body is None:
            return module.linear_velocity_m_s
        return body.velocity_at(world_point)

    def surface_attachment(self, connection: ConnectionId) -> SurfaceAttachment | None:
        """Return where a point-to-surface connection currently sits on its surface.

        This is measured state, not event state: it is resolved lazily from
        the connector frames of the latest sample and cached until the next
        one. Sliding therefore changes it without writing any event. Returns
        ``None`` for a point-to-point connection, or when the attachment is
        undefined in this sample (an unresolved frame, or the point connector
        exactly at the sphere centre).
        """
        runtime = self._connections.get(connection)
        if runtime is None:
            raise KeyError(f"unknown connection '{connection}'")
        stamp = self._frame_clock.stamp
        cached = self._attachments.get(connection)
        if cached is not None and cached[0] == stamp:
            return cached[1]
        attachment = self._resolve_attachment(runtime.connector_a, runtime.connector_b)
        self._attachments[connection] = (stamp, attachment)
        return attachment

    def _resolve_attachment(
        self,
        point_id: ConnectorInstanceId,
        surface_id: ConnectorInstanceId,
    ) -> SurfaceAttachment | None:
        surface = self.connector(surface_id)
        spec = self.connector_type(surface_id).surface
        point = self.connector(point_id)
        if spec is None or point.surface or not (point.resolved and surface.resolved):
            return None
        link_pose = self.link_pose(surface.module_id, surface.parent_link)
        if link_pose is None:
            return None
        contact = surface_contact(point.world_pose, surface.world_pose, link_pose, spec)
        if contact is None:
            return None
        return attachment_from_contact(contact, surface.world_pose, self.magnet_height_m(point_id))

    def magnet_height_m(self, connector: ConnectorInstanceId) -> float | None:
        """Return a point connector's magnet height from its declared lift joint.

        ``h = magnet_height_offset_m - joint position`` (positive joint motion
        protrudes the magnet). ``None`` when no joint is declared or the
        backend has not reported its state.
        """
        instance = self.connector(connector)
        module = self.module(instance.module_id)
        module_type = self._pack.hardware_catalog.module_types[module.module_type_id]
        spec = self._connector_spec(module_type.connectors, instance.connector_id)
        if spec.magnet_height_joint is None:
            return None
        state = module.joint_states.get(spec.magnet_height_joint)
        if state is None:
            return None
        return spec.magnet_height_offset_m - state.position

    def has_capacity(self, connector: ConnectorInstanceId) -> bool:
        """Return whether a connector can accept one more connection now."""
        return self.connector(connector).has_capacity

    def module(self, module_id: ModuleInstanceId) -> ModuleInstance:
        """Return one module instance."""
        try:
            return self._modules[module_id]
        except KeyError as error:
            raise KeyError(f"unknown module '{module_id}'") from error

    def adjacency(
        self, *, exclude: ConnectionId | None = None
    ) -> dict[ModuleInstanceId, frozenset[ModuleInstanceId]]:
        """Return module-level adjacency implied by the active connections."""
        result = {
            module_id: frozenset(self._neighbours.get(module_id, ())) for module_id in self._modules
        }
        removed = self._connections.get(exclude) if exclude is not None else None
        if removed is not None and removed.module_a != removed.module_b:
            a, b = removed.module_a, removed.module_b
            if self._neighbours[a][b] == 1:
                result[a] = result[a] - {b}
                result[b] = result[b] - {a}
        return result

    def _link(self, module_a: ModuleInstanceId, module_b: ModuleInstanceId) -> None:
        for one, other in ((module_a, module_b), (module_b, module_a)):
            counts = self._neighbours.setdefault(one, {})
            counts[other] = counts.get(other, 0) + 1

    def _unlink(self, module_a: ModuleInstanceId, module_b: ModuleInstanceId) -> None:
        for one, other in ((module_a, module_b), (module_b, module_a)):
            counts = self._neighbours[one]
            if counts[other] == 1:
                del counts[other]
            else:
                counts[other] -= 1

    # ------------------------------------------------------------------
    # snapshot ingestion
    # ------------------------------------------------------------------

    def ingest(self, snapshot: BackendStateSnapshot) -> None:
        """Adopt a backend observation as the current kinematic sample.

        Joint states and link poses are taken by reference when the backend
        returns ``FrozenMapping`` views, and copied otherwise. Module root
        state and connector frames are not recomputed here: they are derived
        from this sample the first time they are read, and the values are the
        same as an eager copy would produce.
        """
        if snapshot.time_s > self._time_s:
            self._advancing_samples += 1
        self._time_s = snapshot.time_s
        module_types = self._pack.hardware_catalog.module_types
        for module_id, module in self._modules.items():
            joints = snapshot.joint_states.get(module_id)
            if joints is not None:
                # A FrozenMapping is an immutable, possibly lazy backend view:
                # keep it by reference instead of materialising every joint.
                module.joint_states = joints if isinstance(joints, FrozenMapping) else dict(joints)
            links = snapshot.link_states.get(module_id)
            if links is None or module_types[module.module_type_id].root_link not in links:
                # This sample does not report the root, so the module keeps its
                # current root state: resolve it against the outgoing sample now.
                module.settle()
            if links is None:
                continue
            parent_links = self._connector_parent_links(module.module_type_id)
            if any(link not in links for link in parent_links):
                # A frame whose parent link vanishes keeps its last derived
                # values; derive them from the outgoing sample first.
                for connector_id in self._connectors_by_module[module_id]:
                    connector = self._connectors[connector_id]
                    if connector.parent_link not in links:
                        connector.settle()
            module.link_poses = (
                LinkPoseView(links)
                if isinstance(links, FrozenMapping)
                else {name: body.pose for name, body in links.items()}
            )
        self._sample_snapshot = snapshot
        self._module_clock.stamp += 1
        self._frame_clock.stamp += 1
        self._clear_transient_states()
        self._advance_revision(sample=True)

    def _connector_parent_links(self, module_type_id: str) -> tuple[str, ...]:
        cached = self._parent_links_by_type.get(module_type_id)
        if cached is None:
            connectors = self._pack.hardware_catalog.module_types[module_type_id].connectors
            cached = tuple(dict.fromkeys(spec.parent_link for spec in connectors))
            self._parent_links_by_type[module_type_id] = cached
        return cached

    def _refresh_connector_frames(self) -> None:
        """Mark every connector frame stale against the current sample."""
        self._frame_clock.stamp += 1

    def _resolve_module_root(self, module: ModuleInstance) -> None:
        snapshot = self._sample_snapshot
        root = None
        if snapshot is not None:
            links = snapshot.link_states.get(module.id)
            if links is not None:
                root = links.get(
                    self._pack.hardware_catalog.module_types[module.module_type_id].root_link
                )
        if root is None:
            module.store_root_state()
            return
        module.store_root_state(root.pose, root.linear_velocity_m_s, root.angular_velocity_rad_s)

    def _resolve_connector_frame(self, connector: ConnectorInstance) -> None:
        snapshot = self._sample_snapshot
        module = self._modules[connector.module_id]
        link_pose = module.link_poses.get(connector.parent_link)
        if link_pose is None:
            connector.store_frame(resolved=False)
            return
        module_type = self._pack.hardware_catalog.module_types[module.module_type_id]
        spec = self._connector_spec(module_type.connectors, connector.connector_id)

        measured = None
        if snapshot is not None:
            measured = snapshot.connector_frames.get(connector.module_id, {}).get(
                connector.connector_id
            )
        world_pose = measured if measured is not None else link_pose.compose(connector.local_pose)

        # Docking and approach axes are authored in the parent-link frame,
        # not the connector frame, so they rotate with the link.
        docking_axis = None
        if spec.docking_axis is not None:
            docking_axis = quat_rotate(link_pose.rotation, spec.docking_axis)
        approach_axis = None
        approach = spec.approach_axis or spec.docking_axis
        if approach is not None:
            approach_axis = quat_rotate(link_pose.rotation, approach)

        body = None
        if snapshot is not None:
            body = snapshot.body(connector.module_id, connector.parent_link)
        velocity = (
            body.velocity_at(world_pose.translation)
            if body is not None
            else module.linear_velocity_m_s
        )
        connector.store_frame(
            resolved=True,
            world_pose=world_pose,
            world_docking_axis=docking_axis,
            world_approach_axis=approach_axis,
            world_velocity_m_s=velocity,
        )

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
        if not first.has_capacity or not second.has_capacity:
            raise WorldStateError(
                f"cannot dock '{event.connector_a}' to '{event.connector_b}': "
                "at least one connector is already engaged or full"
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
            constraint=event.constraint,
            realization=event.realization,
            committed_attachment=event.attachment,
        )
        self._connections[connection.id] = connection
        for connector in (first, second):
            connector.lifecycle_state = ConnectorLifecycleState.DOCKED
            connector.connection_ids = tuple(sorted((*connector.connection_ids, connection.id)))
        if first.module_id != second.module_id:
            self._link(first.module_id, second.module_id)
        self._assemblies.merge_on_connection(first.module_id, second.module_id)

    def _apply_undock_committed(self, event: UndockCommitted) -> None:
        connection = self._connections.pop(event.connection_id, None)
        if connection is None:
            raise WorldStateError(f"unknown connection '{event.connection_id}'")
        self._attachments.pop(event.connection_id, None)
        for connector_id in connection.connectors:
            connector = self.connector(connector_id)
            connector.connection_ids = tuple(
                held for held in connector.connection_ids if held != connection.id
            )
            if connector.surface:
                # A surface stays docked while other connections remain, and
                # takes no cooldown: one strut leaving must not lock out the
                # others. The point side's cooldown prevents re-latching.
                if not connector.connection_ids:
                    connector.lifecycle_state = ConnectorLifecycleState.FREE
                continue
            connector.lifecycle_state = ConnectorLifecycleState.FREE
            connector.available_at_s = event.time_s + self._cooldown_s(connector_id)
        if connection.module_a != connection.module_b:
            self._unlink(connection.module_a, connection.module_b)
            # Only the affected assembly is walked, through the maintained
            # adjacency, so an undock costs O(assembly) rather than O(world).
            self._assemblies.split_on_disconnection(
                connection.module_a,
                connection.module_b,
                _NeighbourView(self._neighbours),
            )

    def _apply_dock_failed(self, event: DockFailed) -> bool:
        changed = False
        for connector_id in (event.connector_a, event.connector_b):
            connector = self._connectors.get(connector_id)
            if connector is None or connector.is_engaged or connector.surface:
                # A failed attempt cools down the point side only (see above).
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


class _NeighbourView(Mapping[ModuleInstanceId, frozenset[ModuleInstanceId]]):
    """Read-only adjacency over the maintained neighbour counts, built per lookup."""

    __slots__ = ("_neighbours",)

    def __init__(self, neighbours: dict[ModuleInstanceId, dict[ModuleInstanceId, int]]) -> None:
        self._neighbours = neighbours

    def __getitem__(self, module_id: ModuleInstanceId) -> frozenset[ModuleInstanceId]:
        return frozenset(self._neighbours[module_id])

    def __iter__(self) -> Iterator[ModuleInstanceId]:
        return iter(self._neighbours)

    def __len__(self) -> int:
        return len(self._neighbours)
