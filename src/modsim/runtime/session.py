"""The runtime loop that binds a Robot Pack, a world, and a backend together."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modsim.backends.base import (
    BackendAdapter,
    BackendCapabilities,
    BackendError,
    BackendHandleRegistry,
    SupportsJointCommands,
)
from modsim.backends.registry import create_backend
from modsim.connectors.docking import DockingManager, DockProposal
from modsim.core.entities import JointCommand
from modsim.core.events import ConnectorOverloaded, Event
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    JointInstanceId,
)
from modsim.core.scene import SceneSpec
from modsim.core.state import WorldState
from modsim.robot_packs.schema import ControlMode, JointSpec, JointType, LoadedRobotPack, RobotPack
from modsim.runtime.metrics import DockingMetrics, collect_metrics


class JointCommandError(ValueError):
    """Raised when a joint-command batch is invalid or unsupported."""


def _is_finite_command_number(value: object) -> bool:
    """Return whether a command value is a real finite scalar, excluding bool."""
    return not isinstance(value, bool) and isinstance(value, int | float) and math.isfinite(value)


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

    def set_joint_commands(self, commands: Iterable[JointCommand]) -> None:
        """Validate and submit one atomic batch of scalar joint commands.

        Every command is checked before the backend sees any of them. This
        prevents a bad second target from leaving the first target applied and
        makes Robot Pack modes and limits the single safety boundary shared by
        every physics engine.
        """
        batch = tuple(commands)
        if not batch:
            return

        capabilities = self.adapter.capabilities()
        seen: set[JointInstanceId] = set()
        for command in batch:
            if command.joint in seen:
                raise JointCommandError(
                    f"joint command batch contains duplicate target '{command.joint}'"
                )
            seen.add(command.joint)
            spec = self._resolve_joint_spec(command.joint)
            self._validate_joint_command(command, spec, capabilities)

        adapter = self._joint_command_adapter(capabilities)
        adapter.set_joint_commands(batch)

    def clear_joint_commands(
        self,
        joints: Iterable[JointInstanceId] | None = None,
    ) -> None:
        """Clear persistent commands for selected joints, or for all joints."""
        batch = None if joints is None else tuple(joints)
        if batch == ():
            return

        if batch is not None:
            seen: set[JointInstanceId] = set()
            for joint in batch:
                if joint in seen:
                    raise JointCommandError(
                        f"joint clear batch contains duplicate target '{joint}'"
                    )
                seen.add(joint)
                self._resolve_joint_spec(joint)

        capabilities = self.adapter.capabilities()
        adapter = self._joint_command_adapter(capabilities)
        adapter.clear_joint_commands(batch)

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

    def _resolve_joint_spec(self, joint: JointInstanceId) -> JointSpec:
        """Resolve one instance ID and translate lookup failures for callers."""
        try:
            return self.world.joint_spec(joint)
        except (KeyError, ValueError) as error:
            detail = str(error.args[0]) if error.args else str(error)
            raise JointCommandError(detail) from error

    def _validate_joint_command(
        self,
        command: JointCommand,
        spec: JointSpec,
        capabilities: BackendCapabilities,
    ) -> None:
        """Validate one command without mutating either runtime or backend."""
        if command.mode not in spec.control_modes:
            declared = ", ".join(mode.value for mode in spec.control_modes) or "none"
            raise JointCommandError(
                f"joint '{command.joint}' does not declare '{command.mode.value}' control; "
                f"declared modes: {declared}"
            )
        if not capabilities.supports_joint_commands:
            raise JointCommandError(
                f"backend '{capabilities.name}' does not support joint commands"
            )
        if command.mode not in capabilities.supported_joint_control_modes:
            supported = (
                ", ".join(
                    mode.value
                    for mode in sorted(
                        capabilities.supported_joint_control_modes,
                        key=lambda mode: mode.value,
                    )
                )
                or "none"
            )
            raise JointCommandError(
                f"backend '{capabilities.name}' does not support '{command.mode.value}' "
                f"joint commands; supported modes: {supported}"
            )
        if not _is_finite_command_number(command.value):
            raise JointCommandError(
                f"joint '{command.joint}' command value must be a finite number"
            )
        self._validate_joint_command_bounds(command, spec)

    @staticmethod
    def _validate_joint_command_bounds(command: JointCommand, spec: JointSpec) -> None:
        """Enforce the unit-bearing Robot Pack limit for a command's mode."""
        limits = spec.limits
        angular = spec.type in {JointType.REVOLUTE, JointType.CONTINUOUS}

        if command.mode is ControlMode.POSITION:
            if spec.type is JointType.CONTINUOUS:
                return
            if limits is None:
                raise JointCommandError(
                    f"joint '{command.joint}' has no limits for position control"
                )
            lower = limits.lower_position_rad if angular else limits.lower_position_m
            upper = limits.upper_position_rad if angular else limits.upper_position_m
            if lower is None or upper is None:
                unit = "rad" if angular else "m"
                raise JointCommandError(
                    f"joint '{command.joint}' requires finite lower and upper position "
                    f"limits in {unit}"
                )
            if command.value < lower or command.value > upper:
                unit = "rad" if angular else "m"
                raise JointCommandError(
                    f"joint '{command.joint}' position target {command.value} {unit} is outside "
                    f"[{lower}, {upper}] {unit}"
                )
            return

        if limits is None:
            raise JointCommandError(
                f"joint '{command.joint}' has no limits for {command.mode.value} control"
            )
        if command.mode is ControlMode.VELOCITY:
            maximum = limits.max_velocity_rad_per_s if angular else limits.max_velocity_m_per_s
            unit = "rad/s" if angular else "m/s"
        else:
            maximum = limits.max_effort_nm if angular else limits.max_effort_n
            unit = "N*m" if angular else "N"
        if maximum is None:
            raise JointCommandError(
                f"joint '{command.joint}' requires a maximum {command.mode.value} limit in {unit}"
            )
        if abs(command.value) > maximum:
            raise JointCommandError(
                f"joint '{command.joint}' {command.mode.value} target {command.value} {unit} "
                f"exceeds +/-{maximum} {unit}"
            )

    def _joint_command_adapter(
        self,
        capabilities: BackendCapabilities,
    ) -> SupportsJointCommands:
        """Return the optional command protocol or report an invalid backend."""
        if not capabilities.supports_joint_commands:
            raise JointCommandError(
                f"backend '{capabilities.name}' does not support joint commands"
            )
        if not isinstance(self.adapter, SupportsJointCommands):
            raise BackendError(
                f"backend '{capabilities.name}' advertises joint commands but does not "
                "implement SupportsJointCommands"
            )
        return self.adapter

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
