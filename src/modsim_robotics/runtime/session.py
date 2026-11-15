"""The runtime loop that binds a Robot Pack, a world, and a backend together."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modsim_robotics.backends.base import (
    BackendAdapter,
    BackendCapabilities,
    BackendError,
    BackendHandleRegistry,
    SupportsJointCommands,
)
from modsim_robotics.backends.registry import create_backend
from modsim_robotics.connectors.docking import DockingManager, DockProposal
from modsim_robotics.core.entities import JointCommand
from modsim_robotics.core.events import ConnectorOverloaded, Event
from modsim_robotics.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    JointInstanceId,
)
from modsim_robotics.core.scene import SceneSpec
from modsim_robotics.core.state import WorldState
from modsim_robotics.robot_packs.schema import (
    ControlMode,
    JointSpec,
    JointType,
    LoadedRobotPack,
    RobotPack,
)
from modsim_robotics.runtime.metrics import DockingMetrics, collect_metrics


class JointCommandError(ValueError):
    """Raised when a joint-command batch is invalid or unsupported."""


def _is_finite_command_number(value: object) -> bool:
    """Return whether a command value is a real finite scalar, excluding bool."""
    return not isinstance(value, bool) and isinstance(value, int | float) and math.isfinite(value)


@dataclass(slots=True)
class RuntimeSession:
    """One running world: semantic state in ModSim, physics in the backend."""

    world: WorldState
    """Canonical measured state, committed connections, assemblies, and events."""

    adapter: BackendAdapter
    """Backend that advances mechanics and owns the physical constraints."""

    docking: DockingManager = field(default_factory=DockingManager)
    """Connector evaluation, queued requests, and connection lifecycle manager."""

    handles: BackendHandleRegistry = field(default_factory=BackendHandleRegistry)
    """Mapping of stable ModSim identifiers to backend-native handles."""

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

        The returned world contains the backend's initial measured snapshot.
        Call ``shutdown()`` when finished to release the backend resources.

        Args:
            source: Loaded pack with its asset directory, or an in-memory
                semantic pack. Asset-reading backends require a directory.
            scene: Module instances and their initial world-frame poses.
            adapter: Existing adapter or registered backend name. ``None``
                uses ``MODSIM_BACKEND``, falling back to ``mock``.
            root: Asset directory overriding the loaded pack's root. Supply
                this for an in-memory pack when the backend reads assets.
            docking: Docking manager for this session. ``None`` creates one.
            **backend_options: Constructor options forwarded when creating a
                named or default backend. Ignored for an existing adapter.

        Returns:
            A session owning the initialized world, backend, and docking manager.

        Raises:
            modsim_robotics.core.scene.SceneError: The scene is empty, repeats
                instance IDs, or names module types absent from the pack.
            BackendError: Backend selection, construction, or loading fails,
                including missing dependencies or a required asset directory.

        Examples:
            After installing Generic Cube with ``modsim-robotics setup``, create
            two touching cubes, request docking, and release the backend:

            .. code-block:: python

                from modsim_robotics import RobotPackLoader, RuntimeSession, SceneSpec
                from modsim_robotics.core.ids import ConnectorInstanceId
                from modsim_robotics.user_data import resolve_robot_pack

                loaded = RobotPackLoader().load(resolve_robot_pack("generic_cube"))
                scene = SceneSpec.grid("generic_cube", 2, spacing_m=0.1)
                session = RuntimeSession.create(loaded, scene, adapter="mock")
                try:
                    session.request_dock(
                        ConnectorInstanceId("generic_cube_0/front"),
                        ConnectorInstanceId("generic_cube_1/rear"),
                    )
                    events = session.step(0.01)
                    print([event.kind for event in events])
                finally:
                    session.shutdown()
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
        """Queue a connector pair for the next docking pass.

        This queues intent without moving modules or creating a constraint.
        ``process_docking()`` or a connector-enabled ``step()`` evaluates the
        pair against measured geometry and lifecycle rules. Inspect its returned
        events to see whether the attempt commits or fails.

        Args:
            connector_a: First runtime connector ID, in ``<module>/<connector>``
                form, such as ``generic_cube_0/front``.
            connector_b: Second runtime connector ID in the same form.
        """
        self.docking.request_dock(connector_a, connector_b)

    def request_undock(self, connection: ConnectionId) -> None:
        """Queue a connection release for the next docking pass.

        The constraint remains active until ``process_docking()`` or a
        connector-enabled ``step()`` processes the request. Release removes the
        constraint; it does not move the modules apart or apply a separating
        force. An unknown connection produces an ``UndockFailed`` event during
        processing rather than an immediate exception here.

        Args:
            connection: Runtime connection ID from ``world.connections`` or a
                ``DockCommitted`` event.
        """
        self.docking.request_undock(connection)

    def set_joint_commands(self, commands: Iterable[JointCommand]) -> None:
        """Validate and submit one atomic batch of scalar joint commands.

        Every command is checked before the backend sees any of them. This
        prevents a bad second target from leaving the first target applied and
        makes Robot Pack modes and limits the single safety boundary shared by
        every physics engine.

        Commands persist in the backend until replaced or cleared. Angular
        joints use radians, radians per second, or newton-metres for position,
        velocity, or effort; linear joints use metres, metres per second, or
        newtons. Supported modes depend on both the pack and backend; MuJoCo
        currently implements effort commands.

        Args:
            commands: Iterable of joint targets, consumed once. Each runtime
                joint ID may occur at most once. An empty iterable is a no-op.

        Raises:
            JointCommandError: A target is unknown or duplicated, its value is
                non-finite or outside declared limits, or its mode is unsupported.
            BackendError: The backend advertises command support without the
                required protocol, or rejects a target during submission.
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
        """Clear persistent commands for selected joints, or for all joints.

        Selected joint IDs are validated before the request reaches the backend.
        This clears command targets without resetting poses or joint velocities.
        The MuJoCo backend sets the corresponding effort targets to zero.

        Args:
            joints: Runtime joint IDs to clear. ``None`` clears all targets;
                an empty iterable is a no-op.

        Raises:
            JointCommandError: A selected joint is unknown or duplicated, or the
                backend does not support joint commands.
            BackendError: The backend lacks its advertised command protocol or
                cannot resolve a selected target to an actuator.
        """
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
        """Return the current candidate evaluations without committing any.

        Returns:
            Deterministically ordered connector proposals evaluated against the
            current measured world. No connection or lifecycle event is committed.
        """
        return self.docking.detect(self.world)

    def process_docking(self) -> tuple[Event, ...]:
        """Process queued docking commands against the current backend snapshot.

        This does not step physics or advance time. It exists for deterministic
        scenario staging: once connector frames are exact-aligned, a runtime
        may establish the physical constraint before an unconstrained contact
        step can push the pair apart. Compatibility, acceptance, guards,
        backend two-phase commit, canonical events, and assembly updates still
        go through the ordinary :class:`DockingManager` pipeline.

        Returns:
            Lifecycle events generated by this pass, in application order.

        Raises:
            BackendError: A backend operation fails while creating or removing
                a constraint. Ordinary docking refusals are returned as events.
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

        Args:
            dt_s: Requested advance in simulated seconds. The backend determines
                the actual interval; MuJoCo rounds to whole solver steps. Read
                ``world.time_s`` for the measured simulation time.
            process_connectors: Whether to process queued releases, detect
                connector candidates, and attempt docking after state ingestion.
                ``False`` defers those operations; overload handling still runs.

        Returns:
            Lifecycle events generated during this step, in application order.
            The backend and canonical world have also been advanced in place.

        Raises:
            BackendError: The backend rejects the interval, cannot step or
                sample its state, or fails a constraint operation. A negative
                interval is rejected by both shipped backends.
        """
        self.adapter.step(dt_s)
        snapshot = self.adapter.snapshot()
        self.world.ingest(snapshot)
        events: list[Event] = list(self._evaluate_overloads(snapshot.constraint_forces_n))
        if process_connectors:
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
        """Return the current modular-robot metric snapshot.

        Returns:
            Counts and docking metrics derived from the current canonical world.
        """
        return collect_metrics(self.world)

    def shutdown(self) -> None:
        """Release backend resources when the session is no longer needed.

        This delegates cleanup to the adapter. It does not emit undocking events
        or clear the retained canonical world, which can still be inspected.
        Do not continue stepping or submitting commands after cleanup.

        Raises:
            BackendError: The adapter reports a resource-cleanup failure.
        """
        self.adapter.shutdown()
