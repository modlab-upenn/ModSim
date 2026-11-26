"""The runtime loop that binds a Robot Pack, a world, and a backend together."""

from __future__ import annotations

import math
from collections.abc import Iterable
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
from modsim.core.events import ConnectorOverloaded, DockCommitted, Event, UndockCommitted
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    JointInstanceId,
)
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot
from modsim.core.state import WorldState
from modsim.core.transforms import vec_dot, vec_norm, vec_normalize, vec_scale, vec_sub
from modsim.robot_packs.schema import (
    ConnectorLimits,
    ControlMode,
    JointSpec,
    JointType,
    LoadedRobotPack,
    RobotPack,
)
from modsim.runtime.metrics import DockingMetrics, collect_metrics
from modsim.runtime.surface_slide import SlideReport, SurfaceSlideCommand, SurfaceSlideDriver

_AXIS_LIMITS = (
    ("normal_force", "max_normal_force_n"),
    ("shear_force", "max_shear_force_n"),
    ("bending_moment", "max_bending_moment_nm"),
    ("torsion_moment", "max_torsion_moment_nm"),
)


def _governing_break_limits(
    first: ConnectorLimits | None,
    second: ConnectorLimits | None,
) -> dict[str, float]:
    """Return the lower declared limit per quantity; a joint is as strong as its weaker half."""
    result: dict[str, float] = {}
    for quantity, field_name in _AXIS_LIMITS:
        declared = [
            value
            for limits in (first, second)
            if limits is not None
            for value in (getattr(limits, field_name),)
            if value is not None
        ]
        if declared:
            result[quantity] = min(declared)
    return result


def _check_break_limit_support(pack: RobotPack, capabilities: BackendCapabilities) -> None:
    """Refuse a pack whose moment break limits this backend cannot measure.

    Skipping the check silently would let a connection carry any moment, so a
    moment limit on a backend without moment telemetry is an explicit error.
    """
    if capabilities.supports_constraint_torques:
        return
    for type_id, connector_type in pack.hardware_catalog.connector_types.items():
        limits = connector_type.effective_docking_policy.break_limits
        if limits is None:
            continue
        if limits.max_bending_moment_nm is not None or limits.max_torsion_moment_nm is not None:
            raise BackendError(
                f"connector type '{type_id}' declares moment break limits, but backend "
                f"'{capabilities.name}' does not report constraint moments"
            )


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
    surface_slides: SurfaceSlideDriver = field(default_factory=SurfaceSlideDriver)
    _changed_at_step: dict[ConnectionId, int] = field(
        default_factory=dict[ConnectionId, int], init=False, repr=False
    )
    _change_delay_steps: int | None = field(default=None, init=False, repr=False)

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
        problems = resolved.capabilities().world_support_problems(scene.world)
        if problems:
            raise BackendError("; ".join(problems))
        _check_break_limit_support(pack, resolved.capabilities())
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

    def set_surface_slide(
        self,
        connection: ConnectionId,
        command: SurfaceSlideCommand,
    ) -> None:
        """Slide a ``driven_fixed`` point-to-surface connection until told otherwise.

        The command persists across steps. It is validated now: an unknown
        connection, a connection that is not ``driven_fixed`` (``ball`` and
        ``static_fixed`` cannot be driven), a backend that cannot retarget,
        or non-finite values raise :class:`SurfaceSlideError`.
        """
        self.surface_slides.set(self.world, self.adapter, connection, command)

    def clear_surface_slide(self, connections: Iterable[ConnectionId] | None = None) -> None:
        """Stop sliding selected connections, or all of them."""
        self.surface_slides.clear(connections)

    @property
    def surface_slide_report(self) -> SlideReport:
        """What the last step's slide update clamped or had refused."""
        return self.surface_slides.last_report

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
        return self.record_connection_changes(self.docking.run(self.world, self.adapter))

    # ------------------------------------------------------------------
    # physical effect of connection changes
    # ------------------------------------------------------------------

    def connection_change_pending(self, connection_id: ConnectionId) -> bool:
        """Return whether the next ``step`` may still be solved without the latest change.

        A dock or undock changes ModSim's logical topology at once, but a
        backend may enforce it only after a declared delay (rule 5 of the
        timing contract in ``modsim.backends.base``). While this returns True,
        a controller must not apply a load that relies on the change: braking
        against a released face that the engine still holds, for example.
        Steps are counted as time-advancing snapshots ingested by the world
        (``WorldState.advancing_samples``), so this holds however the backend
        is driven. Counting whole steps is conservative, because each covers at
        least one internal step.
        """
        changed = self._changed_at_step.get(connection_id)
        if changed is None:
            return False
        return self.world.advancing_samples < changed + self._connection_change_delay()

    def connection_changes_pending(self) -> bool:
        """Return whether any recent dock or undock may not act in the next ``step``."""
        return any(self.connection_change_pending(c) for c in tuple(self._changed_at_step))

    def _connection_change_delay(self) -> int:
        if self._change_delay_steps is None:
            self._change_delay_steps = self.adapter.capabilities().connection_change_delay_substeps
        return self._change_delay_steps

    def record_connection_changes(self, events: tuple[Event, ...]) -> tuple[Event, ...]:
        """Note the step at which each committed dock or undock happened.

        The session calls this for every docking pass it runs. A facade that
        runs docking through its own :class:`DockingManager` against this
        session's backend must pass the resulting events here too, or
        :meth:`connection_change_pending` will not know about them. Returns
        ``events`` unchanged.
        """
        delay = self._connection_change_delay()
        for event in events:
            if isinstance(event, DockCommitted | UndockCommitted):
                self._changed_at_step[event.connection_id] = self.world.advancing_samples
        if self._changed_at_step and delay >= 0:
            # Forget changes that can no longer be pending (keeps this O(active)).
            horizon = self.world.advancing_samples - delay
            for connection_id in [c for c, at in self._changed_at_step.items() if at < horizon]:
                del self._changed_at_step[connection_id]
        return events

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

    def step(
        self,
        dt_s: float,
        *,
        process_connectors: bool = True,
    ) -> tuple[Event, ...]:
        """Advance physics, ingest state, and evaluate the runtime lifecycle.

        Order matters. Docking decisions are made against the state the backend
        just reported, never against a stale snapshot, and overload releases are
        processed before new docks so a connection cannot break and re-form in
        the same step. Authored kinematic transit may set
        ``process_connectors=False`` to suppress passive capture while still
        retaining session-owned sampling and overload handling; queued connector
        commands remain pending until a later connector-processing pass.
        """
        self.adapter.step(dt_s)
        snapshot = self.adapter.snapshot()
        self.world.ingest(snapshot)
        events: list[Event] = list(self._evaluate_overloads(snapshot))
        if process_connectors:
            events.extend(self.process_docking())
        # Sliding runs after the docking pass, so connections released this
        # step are skipped and new ones start sliding at the next step.
        self.surface_slides.update(self.world, self.adapter, dt_s)
        return tuple(events)

    def _evaluate_overloads(self, snapshot: BackendStateSnapshot) -> tuple[Event, ...]:
        """Release every connection whose measured load exceeds a declared limit.

        The whole-force ``break_force_n`` check runs first, as before. Then
        the opt-in per-axis ``break_limits`` are checked against the wrench
        resolved about connector A's docking axis. The first exceeded
        quantity is reported and the connection is released.
        """
        forces_n = snapshot.constraint_forces_n
        events: list[Event] = []
        for connection in tuple(self.world.connections.values()):
            measured = forces_n.get(connection.constraint_handle)
            if measured is None:
                continue
            exceeded: tuple[str, float, float] | None = None
            limit = self._break_force_n(connection.connector_a, connection.connector_b)
            if limit is not None and measured > limit:
                exceeded = ("force_magnitude", measured, limit)
            else:
                exceeded = self._exceeded_axis_limit(
                    connection.connector_a,
                    connection.connector_b,
                    snapshot,
                    connection.constraint_handle,
                )
            if exceeded is None:
                continue
            quantity, value, limit_value = exceeded
            events.append(
                self.world.apply(
                    ConnectorOverloaded(
                        time_s=self.world.time_s,
                        connection_id=connection.id,
                        measured_force_n=value,
                        limit_n=limit_value,
                        released=True,
                        quantity=quantity,
                    )
                )
            )
            events.extend(self.docking.undock(self.world, self.adapter, connection.id))
        return self.record_connection_changes(tuple(events))

    def _exceeded_axis_limit(
        self,
        connector_a: ConnectorInstanceId,
        connector_b: ConnectorInstanceId,
        snapshot: BackendStateSnapshot,
        handle: ConstraintHandle,
    ) -> tuple[str, float, float] | None:
        limits = _governing_break_limits(
            self.world.connector_type(connector_a).effective_docking_policy.break_limits,
            self.world.connector_type(connector_b).effective_docking_policy.break_limits,
        )
        if not limits:
            # Read the wrench only when needed: a backend may compute moments lazily.
            return None
        wrench = snapshot.constraint_wrenches[handle]
        axis = vec_normalize(self.world.connector(connector_a).world_docking_axis)
        normal = vec_dot(wrench.force_n, axis)
        measured: dict[str, float | None] = {
            "normal_force": abs(normal),
            "shear_force": vec_norm(vec_sub(wrench.force_n, vec_scale(axis, normal))),
            "bending_moment": None,
            "torsion_moment": None,
        }
        if wrench.torque_nm is not None:
            twist = vec_dot(wrench.torque_nm, axis)
            measured["torsion_moment"] = abs(twist)
            measured["bending_moment"] = vec_norm(vec_sub(wrench.torque_nm, vec_scale(axis, twist)))
        for quantity, limit in limits.items():
            value = measured[quantity]
            if value is not None and value > limit:
                return quantity, value, limit
        return None

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
