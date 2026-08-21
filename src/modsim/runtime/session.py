"""The runtime loop that binds a Robot Pack, a world, and a backend together."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modsim.backends.base import BackendAdapter, BackendHandleRegistry
from modsim.backends.registry import create_backend
from modsim.connectors.docking import DockingManager, DockProposal
from modsim.core.events import ConnectorOverloaded, Event
from modsim.core.ids import ConnectionId, ConnectorInstanceId, ConstraintHandle
from modsim.core.scene import SceneSpec
from modsim.core.state import WorldState
from modsim.robot_packs.schema import LoadedRobotPack, RobotPack
from modsim.runtime.metrics import DockingMetrics, collect_metrics


@dataclass(slots=True)
class RuntimeSession:
    """One running world: semantic state in ModSim, physics in the backend."""

    world: WorldState
    adapter: BackendAdapter
    docking: DockingManager = field(default_factory=DockingManager)
    handles: BackendHandleRegistry = field(default_factory=BackendHandleRegistry)

    @classmethod
    def create(
        cls,
        source: LoadedRobotPack | RobotPack,
        scene: SceneSpec,
        adapter: BackendAdapter | str | None = None,
        *,
        root: Path | None = None,
        docking: DockingManager | None = None,
        **backend_options: Any,
    ) -> RuntimeSession:
        """Load a scene into a backend and build the matching world state.

        ``adapter`` may be an adapter instance, a registered backend name, or
        ``None`` to resolve one from ``MODSIM_BACKEND`` and fall back to the
        default. Passing a :class:`LoadedRobotPack` supplies the pack directory
        automatically, which backends that read mechanical assets require.
        """
        pack = source.pack if isinstance(source, LoadedRobotPack) else source
        pack_root = root if root is not None else getattr(source, "root", None)
        scene.validate_against(pack)
        resolved = (
            adapter
            if isinstance(adapter, BackendAdapter) and not isinstance(adapter, str)
            else create_backend(adapter, **backend_options)
        )
        handles = resolved.load(pack, scene, root=pack_root)
        world = WorldState.from_scene(pack, scene)
        session = cls(
            world=world,
            adapter=resolved,
            docking=docking if docking is not None else DockingManager(),
            handles=handles,
        )
        world.ingest(resolved.snapshot())
        return session

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------

    def request_dock(
        self,
        connector_a: ConnectorInstanceId,
        connector_b: ConnectorInstanceId,
    ) -> None:
        """Queue an explicit dock command."""
        self.docking.request_dock(connector_a, connector_b)

    def request_undock(self, connection: ConnectionId) -> None:
        """Queue an explicit undock command."""
        self.docking.request_undock(connection)

    def proposals(self) -> tuple[DockProposal, ...]:
        """Return the current candidate evaluations without committing any."""
        return self.docking.detect(self.world)

    def process_docking(self) -> tuple[Event, ...]:
        """Process queued docking commands against the current backend snapshot.

        This does not step physics or advance time. It exists for deterministic
        scenario staging: once connector frames are exact-aligned, a runtime
        may establish the physical constraint before an unconstrained contact
        step can push the pair apart. Compatibility, acceptance, guards,
        backend two-phase commit, canonical events, and assembly updates still
        go through the ordinary :class:`DockingManager` pipeline.
        """
        return self.docking.run(self.world, self.adapter)

    # ------------------------------------------------------------------
    # loop
    # ------------------------------------------------------------------

    def step(self, dt_s: float) -> tuple[Event, ...]:
        """Advance physics, ingest state, evaluate overloads, then run docking.

        Order matters. Docking decisions are made against the state the backend
        just reported, never against a stale snapshot, and overload releases are
        processed before new docks so a connection cannot break and re-form in
        the same step.
        """
        self.adapter.step(dt_s)
        snapshot = self.adapter.snapshot()
        self.world.ingest(snapshot)
        events: list[Event] = list(self._evaluate_overloads(snapshot.constraint_forces_n))
        events.extend(self.process_docking())
        return tuple(events)

    def _evaluate_overloads(
        self,
        forces_n: Mapping[ConstraintHandle, float],
    ) -> tuple[Event, ...]:
        events: list[Event] = []
        for connection in tuple(self.world.connections.values()):
            measured = forces_n.get(connection.constraint_handle)
            if measured is None:
                continue
            limit = self._break_force_n(connection.connector_a, connection.connector_b)
            if limit is None or measured <= limit:
                continue
            events.append(
                self.world.apply(
                    ConnectorOverloaded(
                        time_s=self.world.time_s,
                        connection_id=connection.id,
                        measured_force_n=measured,
                        limit_n=limit,
                        released=True,
                    )
                )
            )
            events.extend(self.docking.undock(self.world, self.adapter, connection.id))
        return tuple(events)

    def _break_force_n(
        self,
        connector_a: ConnectorInstanceId,
        connector_b: ConnectorInstanceId,
    ) -> float | None:
        """Return the lower of the two declared break forces, if any.

        The weaker connector governs: a joint is only as strong as its weakest
        half.
        """
        limits = [
            self.world.connector_type(connector).effective_docking_policy.break_force_n
            for connector in (connector_a, connector_b)
        ]
        declared = [value for value in limits if value is not None]
        return min(declared) if declared else None

    # ------------------------------------------------------------------
    # observation
    # ------------------------------------------------------------------

    def metrics(self) -> DockingMetrics:
        """Return the current modular-robot metric snapshot."""
        return collect_metrics(self.world)

    def shutdown(self) -> None:
        """Release backend resources."""
        self.adapter.shutdown()
