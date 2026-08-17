"""Docking and undocking execution.

The manager owns the per-step pipeline: broad phase, compatibility, acceptance,
guards, then a two-phase commit against the backend. A logical connection is
only ever created *after* the backend confirms the physical constraint exists,
so ModSim's semantic state can never claim a connection that no physics engine
is enforcing.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field

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
from modsim.robot_packs.schema import AlignmentMode, ConnectorTypeSpec


@dataclass(frozen=True, slots=True)
class DockProposal:
    """One evaluated candidate pair and every reason it was or was not viable."""

    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId
    compatibility: CompatibilityResult
    acceptance: AcceptanceResult
    guard: GuardResult
    requested: bool = False

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
        positions = [
            (connector.id, connector.world_pose.translation)
            for connector in world.connectors.values()
            if connector.resolved and not connector.is_engaged
        ]
        pairs = set(candidate_pairs(positions, radius))
        pairs.update(self.requested_pairs)

        proposals: list[DockProposal] = []
        for connector_a_id, connector_b_id in sorted(pairs):
            requested = (connector_a_id, connector_b_id) in self.requested_pairs
            proposal = self._evaluate(world, connector_a_id, connector_b_id, requested=requested)
            if proposal is not None:
                proposals.append(proposal)
        return tuple(proposals)

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
        compatibility = evaluate_compatibility(type_a, type_b)
        acceptance = evaluate_acceptance(connector_a, connector_b, type_a, type_b)
        guard = evaluate_dock_guards(world, connector_a, connector_b, requested=requested)
        return DockProposal(
            connector_a=connector_a_id,
            connector_b=connector_b_id,
            compatibility=compatibility,
            acceptance=acceptance,
            guard=guard,
            requested=requested,
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
        relative = (
            nominal_relative_transform(
                connector_a, connector_b, proposal.acceptance.orientation_rad
            )
            if snap
            else proposal.acceptance.relative_transform
        )
        request = ConnectionRequest(
            connection_id=proposal.connection_id,
            module_a=connector_a.module_id,
            link_a=connector_a.parent_link,
            connector_a=connector_a.id,
            connector_a_local=connector_a.local_pose,
            module_b=connector_b.module_id,
            link_b=connector_b.parent_link,
            connector_b=connector_b.id,
            connector_b_local=connector_b.local_pose,
            relative_transform=relative,
            physical_connection=physical,
            orientation_rad=proposal.acceptance.orientation_rad,
            orientation_index=proposal.acceptance.orientation_index,
            snap_to_nominal=snap,
        )

        world.mark_lifecycle(connector_a.id, ConnectorLifecycleState.IN_ACCEPTANCE_REGION)
        world.mark_lifecycle(connector_b.id, ConnectorLifecycleState.IN_ACCEPTANCE_REGION)
        outcome = self._create_connection(adapter, request)
        if not outcome.success or outcome.handle is None:
            return (
                world.apply(
                    DockFailed(
                        time_s=world.time_s,
                        connector_a=proposal.connector_a,
                        connector_b=proposal.connector_b,
                        reason=DockFailureReason.BACKEND_REFUSED,
                        detail=outcome.detail or "backend refused the constraint",
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
                orientation_rad=proposal.acceptance.orientation_rad,
                orientation_index=proposal.acceptance.orientation_index,
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
                # A connector can only commit once per pass. Sorted evaluation
                # order makes which pair wins deterministic.
                continue
            committed = self.commit(world, adapter, proposal)
            events.extend(committed)
            if any(isinstance(event, DockCommitted) for event in committed):
                engaged.update((proposal.connector_a, proposal.connector_b))

        stale = {pair for pair in self.requested_pairs if self._is_stale(world, pair)}
        self.requested_pairs -= stale
        return tuple(events)

    @staticmethod
    def _is_stale(
        world: WorldState,
        pair: tuple[ConnectorInstanceId, ConnectorInstanceId],
    ) -> bool:
        try:
            return any(world.connector(connector).is_engaged for connector in pair)
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


def free_connectors(world: WorldState) -> tuple[ConnectorInstance, ...]:
    """Return every connector that currently holds no connection."""
    return tuple(connector for connector in world.connectors.values() if not connector.is_engaged)
