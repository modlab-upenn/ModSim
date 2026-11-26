"""Docking and undocking execution.

The manager owns the per-step pipeline: broad phase, compatibility, acceptance,
guards, then a two-phase commit against the backend. A logical connection is
only ever created *after* the backend confirms the physical constraint exists,
so ModSim's semantic state can never claim a connection that no physics engine
is enforcing.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field, replace

from modsim.backends.base import (
    BackendAdapter,
    ConnectionOutcome,
    ConnectionRequest,
)
from modsim.connectors.acceptance import (
    AcceptanceResult,
    evaluate_acceptance,
    nominal_relative_transform,
)
from modsim.connectors.broadphase import candidate_pairs
from modsim.connectors.compatibility import CompatibilityResult, evaluate_compatibility
from modsim.connectors.guards import (
    GuardResult,
    evaluate_dock_guards,
    evaluate_undock_guards,
)
from modsim.connectors.surface import (
    candidate_surface_pairs,
    evaluate_surface_acceptance,
    virtual_attachment_connector,
)
from modsim.core.entities import ConnectorInstance, ConnectorLifecycleState
from modsim.core.events import (
    AssemblyMerged,
    AssemblySplit,
    DockCandidateDetected,
    DockCommitted,
    DockFailed,
    DockFailureReason,
    Event,
    UndockCommitted,
    UndockFailed,
)
from modsim.core.ids import ConnectionId, ConnectorInstanceId, connection_id
from modsim.core.state import WorldState
from modsim.core.surface import SurfaceContact, attachment_from_contact, surface_center
from modsim.core.transforms import Transform, Vec3
from modsim.robot_packs.schema import (
    AlignmentMode,
    BallConstraintSpec,
    ConnectorTypeSpec,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    SurfaceRealization,
)


@dataclass(frozen=True, slots=True)
class DockProposal:
    """One evaluated candidate pair and every reason it was or was not viable."""

    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId
    compatibility: CompatibilityResult
    acceptance: AcceptanceResult
    guard: GuardResult
    requested: bool = False
    surface_contact: SurfaceContact | None = None
    """Where the point side meets the surface, for a point-to-surface pair.

    For such a pair ``connector_a`` is always the point connector and
    ``connector_b`` the surface connector (docs/continuous_docking.md 5.3).
    """

    @property
    def viable(self) -> bool:
        """Whether this pair passed compatibility, acceptance, and guards."""
        return self.compatibility.compatible and self.acceptance.satisfied and self.guard.allowed

    @property
    def connection_id(self) -> ConnectionId:
        """Return the deterministic identifier this pair would produce."""
        return connection_id(self.connector_a, self.connector_b)

    def failure(self) -> tuple[DockFailureReason, str]:
        """Return the reason code and detail for a non-viable proposal."""
        if not self.compatibility.compatible:
            return (
                DockFailureReason.INCOMPATIBLE,
                self.compatibility.reason or "incompatible connector types",
            )
        if not self.guard.allowed:
            return DockFailureReason.GUARD_REJECTED, self.guard.reason or "guard rejected"
        if not self.acceptance.satisfied:
            return DockFailureReason.OUTSIDE_ACCEPTANCE_REGION, self.acceptance.reason()
        return DockFailureReason.GUARD_REJECTED, "proposal is viable"


@dataclass(slots=True)
class DockingManager:
    """Evaluates candidate connector pairs and commits docking decisions."""

    requested_pairs: set[tuple[ConnectorInstanceId, ConnectorInstanceId]] = field(
        default_factory=set[tuple[ConnectorInstanceId, ConnectorInstanceId]]
    )
    requested_releases: set[ConnectionId] = field(default_factory=set[ConnectionId])
    _detected: set[tuple[ConnectorInstanceId, ConnectorInstanceId]] = field(
        default_factory=set[tuple[ConnectorInstanceId, ConnectorInstanceId]]
    )
    """Pairs that were already in candidacy on the previous pass."""

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------

    def request_dock(
        self,
        connector_a: ConnectorInstanceId,
        connector_b: ConnectorInstanceId,
    ) -> None:
        """Queue an explicit dock command for the next evaluation pass."""
        self.requested_pairs.add(self._canonical(connector_a, connector_b))

    def request_undock(self, connection: ConnectionId) -> None:
        """Queue an explicit undock command for the next evaluation pass."""
        self.requested_releases.add(connection)

    @staticmethod
    def _canonical(
        connector_a: ConnectorInstanceId,
        connector_b: ConnectorInstanceId,
    ) -> tuple[ConnectorInstanceId, ConnectorInstanceId]:
        return (
            (connector_a, connector_b) if connector_a < connector_b else (connector_b, connector_a)
        )

    # ------------------------------------------------------------------
    # detection
    # ------------------------------------------------------------------

    def detection_radius(self, world: WorldState) -> float:
        """Return the broad-phase radius implied by the pack's acceptance regions."""
        radii = [
            connector_type.acceptance_region.position_tolerance_m
            for connector_type in world.pack.hardware_catalog.connector_types.values()
            if connector_type.acceptance_region is not None
        ]
        return max(radii) if radii else 0.0

    def detect(self, world: WorldState) -> tuple[DockProposal, ...]:
        """Evaluate every candidate pair in the world, in deterministic order.

        Explicitly requested pairs are always evaluated, even when they are far
        outside the broad-phase radius, so that a rejected command produces an
        informative ``DockFailed`` instead of silence.
        """
        radius = self.detection_radius(world)
        connector_types = world.pack.hardware_catalog.connector_types
        positions = [
            (connector.id, connector.world_pose.translation)
            for connector in world.connectors.values()
            if connector.resolved
            and not connector.is_engaged
            and not connector_types[connector.connector_type_id].is_surface
        ]
        pairs = set(candidate_pairs(positions, radius))
        pairs.update(self._surface_candidates(world, positions))
        pairs.update(self.requested_pairs)

        proposals: list[DockProposal] = []
        for connector_a_id, connector_b_id in sorted(pairs):
            requested = (connector_a_id, connector_b_id) in self.requested_pairs
            proposal = self._evaluate(world, connector_a_id, connector_b_id, requested=requested)
            if proposal is not None:
                proposals.append(proposal)
        return tuple(proposals)

    def _surface_candidates(
        self,
        world: WorldState,
        point_positions: list[tuple[ConnectorInstanceId, Vec3]],
    ) -> set[tuple[ConnectorInstanceId, ConnectorInstanceId]]:
        """Return canonical point-to-surface candidate pairs (section 4.3)."""
        connector_types = world.pack.hardware_catalog.connector_types
        surfaces: list[tuple[ConnectorInstanceId, Vec3, float]] = []
        tolerance = 0.0
        for connector in world.connectors.values():
            connector_type = connector_types[connector.connector_type_id]
            spec = connector_type.surface
            if spec is None or not connector.resolved or not world.has_capacity(connector.id):
                continue
            surfaces.append(
                (connector.id, surface_center(connector.world_pose, spec), spec.radius_m)
            )
            if connector_type.acceptance_region is not None:
                tolerance = max(tolerance, connector_type.acceptance_region.position_tolerance_m)
        if not surfaces:
            return set()
        surface_types = {
            connector_types[world.connector(surface_id).connector_type_id].id
            for surface_id, _, _ in surfaces
        }
        points = [
            (connector_id, position)
            for connector_id, position in point_positions
            if surface_types.intersection(
                connector_types[world.connector(connector_id).connector_type_id].compatible_with
            )
        ]
        return {
            self._canonical(point_id, surface_id)
            for point_id, surface_id in candidate_surface_pairs(points, surfaces, tolerance)
        }

    def _evaluate(
        self,
        world: WorldState,
        connector_a_id: ConnectorInstanceId,
        connector_b_id: ConnectorInstanceId,
        *,
        requested: bool,
    ) -> DockProposal | None:
        try:
            connector_a = world.connector(connector_a_id)
            connector_b = world.connector(connector_b_id)
            type_a = world.connector_type(connector_a_id)
            type_b = world.connector_type(connector_b_id)
        except KeyError:
            return None
        if type_a.is_surface and not type_b.is_surface:
            # Side A of a point-to-surface pair is always the point connector.
            connector_a, connector_b = connector_b, connector_a
            type_a, type_b = type_b, type_a
            connector_a_id, connector_b_id = connector_b_id, connector_a_id
        compatibility = evaluate_compatibility(type_a, type_b)
        contact: SurfaceContact | None = None
        if type_b.is_surface and not type_a.is_surface:
            surface = connector_b
            acceptance, contact = evaluate_surface_acceptance(
                connector_a,
                surface,
                type_a,
                type_b,
                surface_link_pose=world.link_pose(surface.module_id, surface.parent_link),
                surface_velocity_at=lambda point: world.point_velocity(
                    surface.module_id, surface.parent_link, point
                ),
            )
        elif type_b.is_surface:
            acceptance = _unsatisfied_pair("two surface connectors cannot mate")
        else:
            acceptance = evaluate_acceptance(connector_a, connector_b, type_a, type_b)
        guard = evaluate_dock_guards(
            world, connector_a, connector_b, requested=requested, surface_contact=contact
        )
        return DockProposal(
            connector_a=connector_a_id,
            connector_b=connector_b_id,
            compatibility=compatibility,
            acceptance=acceptance,
            guard=guard,
            requested=requested,
            surface_contact=contact,
        )

    # ------------------------------------------------------------------
    # commit
    # ------------------------------------------------------------------

    def commit(
        self,
        world: WorldState,
        adapter: BackendAdapter,
        proposal: DockProposal,
    ) -> tuple[Event, ...]:
        """Realise one proposal, or record why it could not be realised."""
        if proposal.requested:
            # A command is one attempt. Consume it before evaluating the
            # outcome so a guard, acceptance, or backend failure is reported
            # once instead of producing the same DockFailed event every step.
            self.requested_pairs.discard(
                self._canonical(proposal.connector_a, proposal.connector_b)
            )

        if not proposal.viable:
            if not proposal.requested:
                # Opportunistic near-misses are the normal case every step and
                # would swamp the event log. Only commanded attempts report.
                return ()
            reason, detail = proposal.failure()
            return (
                world.apply(
                    DockFailed(
                        time_s=world.time_s,
                        connector_a=proposal.connector_a,
                        connector_b=proposal.connector_b,
                        reason=reason,
                        detail=detail,
                    )
                ),
            )

        connector_a = world.connector(proposal.connector_a)
        connector_b = world.connector(proposal.connector_b)
        type_a = world.connector_type(proposal.connector_a)
        type_b = world.connector_type(proposal.connector_b)
        physical = proposal.guard.physical_connection
        if physical is None:  # pragma: no cover - guarded by evaluate_dock_guards
            raise ValueError("a viable proposal must carry a physical connection spec")

        snap = self._snaps_to_nominal(type_a, type_b)
        contact = proposal.surface_contact
        mate_b = (
            connector_b
            if contact is None
            else virtual_attachment_connector(connector_b, contact, connector_b.world_velocity_m_s)
        )
        # A surface snap keeps the measured attachment frame, spin included,
        # so it snaps to roll zero against A_q rather than to the measured roll.
        relative = (
            nominal_relative_transform(
                connector_a,
                mate_b,
                0.0 if contact is not None else proposal.acceptance.orientation_rad,
            )
            if snap
            else proposal.acceptance.relative_transform
        )
        base_request = ConnectionRequest(
            connection_id=proposal.connection_id,
            module_a=connector_a.module_id,
            link_a=connector_a.parent_link,
            connector_a=connector_a.id,
            connector_a_local=connector_a.local_pose,
            module_b=connector_b.module_id,
            link_b=connector_b.parent_link,
            connector_b=connector_b.id,
            connector_b_local=mate_b.local_pose,
            relative_transform=relative,
            physical_connection=physical,
            orientation_rad=proposal.acceptance.orientation_rad,
            orientation_index=proposal.acceptance.orientation_index,
            snap_to_nominal=snap,
            snap_side="a" if contact is not None else "b",
        )
        if contact is None:
            attempts: tuple[tuple[SurfaceRealization | None, ConnectionRequest], ...] = (
                (None, base_request),
            )
        else:
            attempts = self._surface_attempts(world, adapter, base_request, physical, contact)
            if not attempts:
                return (
                    world.apply(
                        DockFailed(
                            time_s=world.time_s,
                            connector_a=proposal.connector_a,
                            connector_b=proposal.connector_b,
                            reason=DockFailureReason.BACKEND_REFUSED,
                            detail=_no_realization_detail(adapter, physical),
                        )
                    ),
                )

        world.mark_lifecycle(connector_a.id, ConnectorLifecycleState.IN_ACCEPTANCE_REGION)
        world.mark_lifecycle(connector_b.id, ConnectorLifecycleState.IN_ACCEPTANCE_REGION)
        # Try each realisation the backend advertises, in authored order. A
        # refusal falls through to the next one; only the last refusal fails
        # the dock. Point-to-point pairs have exactly one attempt.
        refusals: list[str] = []
        realization: SurfaceRealization | None = None
        request = base_request
        outcome = ConnectionOutcome.refused("no attempt was made")
        for realization, request in attempts:
            outcome = self._create_connection(adapter, request)
            if outcome.success and outcome.handle is not None:
                break
            detail = outcome.detail or "backend refused the constraint"
            refusals.append(detail if realization is None else f"{realization.value}: {detail}")
        if not outcome.success or outcome.handle is None:
            return (
                world.apply(
                    DockFailed(
                        time_s=world.time_s,
                        connector_a=proposal.connector_a,
                        connector_b=proposal.connector_b,
                        reason=DockFailureReason.BACKEND_REFUSED,
                        detail="; ".join(refusals),
                    )
                ),
            )

        before = world.assemblies.assembly_of(connector_a.module_id)
        other_before = world.assemblies.assembly_of(connector_b.module_id)
        committed = world.apply(
            DockCommitted(
                time_s=world.time_s,
                connection_id=proposal.connection_id,
                connector_a=proposal.connector_a,
                connector_b=proposal.connector_b,
                constraint_handle=outcome.handle,
                relative_transform=relative,
                constraint=request.physical_connection.constraint,
                orientation_rad=proposal.acceptance.orientation_rad,
                orientation_index=proposal.acceptance.orientation_index,
                realization=realization,
                attachment=(
                    None
                    if contact is None
                    else attachment_from_contact(
                        contact, connector_b.world_pose, world.magnet_height_m(connector_a.id)
                    )
                ),
            )
        )
        events: list[Event] = [committed]
        if before != other_before:
            events.append(
                world.apply(
                    AssemblyMerged(
                        time_s=world.time_s,
                        assembly_id=world.assemblies.assembly_of(connector_a.module_id),
                        merged_from=tuple(sorted((before, other_before))),
                    )
                )
            )
        return tuple(events)

    @staticmethod
    def _surface_attempts(
        world: WorldState,
        adapter: BackendAdapter,
        request: ConnectionRequest,
        physical: PhysicalConnectionSpec,
        contact: SurfaceContact,
    ) -> tuple[tuple[SurfaceRealization, ConnectionRequest], ...]:
        """Return the realisations to try for a surface pair, each with its request.

        An authored list is honoured in order; a pair that declares none is
        realised as ``static_fixed``, which is exactly the authored ``fixed``.
        Realisations the backend does not advertise are skipped here, so the
        backend only ever sees requests it claims to support.
        """
        capabilities = adapter.capabilities()
        supported = capabilities.supported_constraint_types
        preferred = physical.surface_realizations or (SurfaceRealization.STATIC_FIXED,)
        link_b = world.link_pose(request.module_b, request.link_b)
        center_link_b = (
            None if link_b is None else link_b.inverse().apply_point(contact.center_world)
        )
        fixed = PhysicalConnectionSpec(constraint=PhysicalConstraintType.FIXED)
        attempts: list[tuple[SurfaceRealization, ConnectionRequest]] = []
        point_pose = world.connector(request.connector_a).world_pose
        ball = PhysicalConnectionSpec(
            constraint=PhysicalConstraintType.BALL,
            ball=BallConstraintSpec(
                anchor_m=point_pose.inverse().apply_point(contact.center_world)
            ),
        )
        realised: dict[SurfaceRealization, PhysicalConnectionSpec] = {}
        if PhysicalConstraintType.FIXED in supported:
            realised[SurfaceRealization.STATIC_FIXED] = fixed
            if capabilities.supports_connection_retarget:
                realised[SurfaceRealization.DRIVEN_FIXED] = fixed
        if PhysicalConstraintType.BALL in supported:
            realised[SurfaceRealization.BALL] = ball
        for realization in preferred:
            spec = realised.get(realization)
            # ``physical`` needs a link-wrench primitive no backend offers yet.
            if spec is not None:
                attempts.append(
                    (
                        realization,
                        replace(
                            request,
                            physical_connection=spec,
                            surface_realization=realization,
                            surface_center_link_b=center_link_b,
                        ),
                    )
                )
        return tuple(attempts)

    @staticmethod
    def _snaps_to_nominal(type_a: ConnectorTypeSpec, type_b: ConnectorTypeSpec) -> bool:
        """Return whether either side asks for nominal alignment.

        Snapping is asymmetric on purpose: if one connector type is authored for
        drift-free lattice reconfiguration, honouring that is more useful than
        requiring both sides to agree.
        """
        return AlignmentMode.NOMINAL in (
            type_a.effective_docking_policy.alignment,
            type_b.effective_docking_policy.alignment,
        )

    @staticmethod
    def _create_connection(
        adapter: BackendAdapter,
        request: ConnectionRequest,
    ) -> ConnectionOutcome:
        capabilities = adapter.capabilities()
        if not capabilities.supports_runtime_constraints:
            return ConnectionOutcome.refused(
                f"backend '{capabilities.name}' cannot create constraints at runtime"
            )
        return adapter.create_physical_connection(request)

    # ------------------------------------------------------------------
    # undocking
    # ------------------------------------------------------------------

    def undock(
        self,
        world: WorldState,
        adapter: BackendAdapter,
        connection: ConnectionId,
    ) -> tuple[Event, ...]:
        """Release one connection, or record why it could not be released."""
        self.requested_releases.discard(connection)
        runtime = world.connections.get(connection)
        if runtime is None:
            return (
                world.apply(
                    UndockFailed(
                        time_s=world.time_s,
                        connection_id=connection,
                        reason=DockFailureReason.UNKNOWN_CONNECTION,
                        detail=f"no active connection '{connection}'",
                    )
                ),
            )
        guard = evaluate_undock_guards(world, runtime)
        if not guard.allowed:
            return (
                world.apply(
                    UndockFailed(
                        time_s=world.time_s,
                        connection_id=connection,
                        reason=DockFailureReason.UNDOCKING_UNSUPPORTED,
                        detail=guard.reason or "undocking rejected",
                    )
                ),
            )
        if not adapter.remove_physical_connection(runtime.constraint_handle):
            return (
                world.apply(
                    UndockFailed(
                        time_s=world.time_s,
                        connection_id=connection,
                        reason=DockFailureReason.BACKEND_REFUSED,
                        detail=(f"backend does not hold constraint '{runtime.constraint_handle}'"),
                    )
                ),
            )

        source = world.assemblies.assembly_of(runtime.module_a)
        released = world.apply(
            UndockCommitted(
                time_s=world.time_s,
                connection_id=connection,
                connector_a=runtime.connector_a,
                connector_b=runtime.connector_b,
            )
        )
        events: list[Event] = [released]
        resulting = tuple(
            sorted(
                {
                    world.assemblies.assembly_of(runtime.module_a),
                    world.assemblies.assembly_of(runtime.module_b),
                }
            )
        )
        if len(resulting) > 1:
            events.append(
                world.apply(
                    AssemblySplit(
                        time_s=world.time_s,
                        source_assembly_id=source,
                        resulting=resulting,
                    )
                )
            )
        return tuple(events)

    # ------------------------------------------------------------------
    # one pass
    # ------------------------------------------------------------------

    def run(
        self,
        world: WorldState,
        adapter: BackendAdapter,
    ) -> tuple[Event, ...]:
        """Run one full docking pass: releases first, then detection and commits."""
        events: list[Event] = []
        for connection in sorted(self.requested_releases):
            events.extend(self.undock(world, adapter, connection))

        proposals = self.detect(world)
        events.extend(self._mark_detections(world, proposals))
        engaged: set[ConnectorInstanceId] = set()
        for proposal in proposals:
            if proposal.connector_a in engaged or proposal.connector_b in engaged:
                # A point connector can only commit once per pass. Sorted
                # evaluation order makes which pair wins deterministic.
                continue
            if proposal.surface_contact is not None and proposal.viable:
                # A surface accepts several commits per pass, so its capacity
                # and separation are re-checked against the commits made
                # earlier in this pass before this one is attempted.
                proposal = replace(proposal, guard=self._recheck_guard(world, proposal))
            committed = self.commit(world, adapter, proposal)
            events.extend(committed)
            if any(isinstance(event, DockCommitted) for event in committed):
                engaged.update(
                    connector
                    for connector in (proposal.connector_a, proposal.connector_b)
                    if not world.connector(connector).surface
                )

        stale = {pair for pair in self.requested_pairs if self._is_stale(world, pair)}
        self.requested_pairs -= stale
        return tuple(events)

    @staticmethod
    def _recheck_guard(world: WorldState, proposal: DockProposal) -> GuardResult:
        return evaluate_dock_guards(
            world,
            world.connector(proposal.connector_a),
            world.connector(proposal.connector_b),
            requested=proposal.requested,
            surface_contact=proposal.surface_contact,
        )

    @staticmethod
    def _is_stale(
        world: WorldState,
        pair: tuple[ConnectorInstanceId, ConnectorInstanceId],
    ) -> bool:
        """Whether a queued command can no longer be attempted.

        A command goes stale once its point end holds a connection or its
        surface end is full.
        """
        try:
            return any(not world.connector(connector).has_capacity for connector in pair)
        except KeyError:
            return True

    def _mark_detections(
        self,
        world: WorldState,
        proposals: Iterable[DockProposal],
    ) -> tuple[Event, ...]:
        """Set detection lifecycle states and log newly detected pairs.

        A pair that stays in range for a thousand steps is one detection, not a
        thousand. The event log records transitions, so an entry is written only
        when a pair enters candidacy, and the previous pass's set is what makes
        that decision.
        """
        events: list[Event] = []
        detected: set[tuple[ConnectorInstanceId, ConnectorInstanceId]] = set()
        for proposal in proposals:
            if not proposal.compatibility.compatible:
                continue
            state = (
                ConnectorLifecycleState.IN_ACCEPTANCE_REGION
                if proposal.acceptance.satisfied
                else ConnectorLifecycleState.CANDIDATE_DETECTED
            )
            for connector_id in (proposal.connector_a, proposal.connector_b):
                world.mark_lifecycle(connector_id, state)
            pair = (proposal.connector_a, proposal.connector_b)
            detected.add(pair)
            if pair in self._detected:
                continue
            events.append(
                world.apply(
                    DockCandidateDetected(
                        time_s=world.time_s,
                        connector_a=proposal.connector_a,
                        connector_b=proposal.connector_b,
                    )
                )
            )
        self._detected = detected
        return tuple(events)


def _no_realization_detail(adapter: BackendAdapter, physical: PhysicalConnectionSpec) -> str:
    preferred = physical.surface_realizations or (SurfaceRealization.STATIC_FIXED,)
    names = ", ".join(realization.value for realization in preferred)
    return (
        f"backend '{adapter.capabilities().name}' supports none of the surface "
        f"realizations [{names}]"
    )


def _unsatisfied_pair(detail: str) -> AcceptanceResult:
    return AcceptanceResult(
        satisfied=False,
        criteria=(),
        relative_transform=Transform.identity(),
        orientation_rad=0.0,
        orientation_index=None,
        detail=detail,
    )


def free_connectors(world: WorldState) -> tuple[ConnectorInstance, ...]:
    """Return every connector that can accept another connection.

    That is every point connector holding no connection, and every surface
    connector below its capacity.
    """
    return tuple(connector for connector in world.connectors.values() if connector.has_capacity)
