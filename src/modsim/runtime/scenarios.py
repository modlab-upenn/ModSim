"""Reusable setup helpers for small runtime demonstrations.

These helpers deliberately operate on connector frames reported by a loaded
backend.  That makes them work for connectors attached to articulated links as
well as root-link connectors, without duplicating URDF forward kinematics in
the ModSim core.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

from modsim.backends.base import BackendError, SupportsModuleKinematics
from modsim.connectors.acceptance import nominal_relative_transform
from modsim.core.events import DockFailed, Event, UndockFailed
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ModuleInstanceId,
    connection_id,
)
from modsim.core.transforms import Transform, Vec3, vec_scale
from modsim.runtime.session import RuntimeSession


class ScenarioSetupError(ValueError):
    """Raised when a runtime scene cannot be arranged as requested."""


@dataclass(frozen=True, slots=True)
class DockingPairSetup:
    """Result of arranging one moving connector in front of a fixed connector."""

    fixed_connector: ConnectorInstanceId
    moving_connector: ConnectorInstanceId
    moving_module: ModuleInstanceId
    approach_direction: Vec3
    gap_m: float


class DockingPairPhase(StrEnum):
    """Presentation-neutral phase of a scripted connector-pair scenario."""

    APPROACHING = "approaching"
    DOCKED = "docked"
    RELEASING = "releasing"
    RETRACTING = "retracting"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class DockingPairScenarioConfig:
    """Validated controls for one staged two-module docking scenario.

    Connector identifiers are full runtime IDs such as ``module_0/front``.
    ``release_after_s`` is relative to scenario creation rather than absolute
    backend time, which makes a configuration reusable after a warm-up or in a
    world whose clock did not start at zero.
    """

    fixed_connector: ConnectorInstanceId
    moving_connector: ConnectorInstanceId
    gap_m: float = 0.03
    orientation_rad: float = 0.0
    approach_speed_m_s: float = 0.03
    dt_s: float = 0.002
    release_after_s: float | None = None
    retract_speed_m_s: float | None = None

    def __post_init__(self) -> None:
        if self.fixed_connector == self.moving_connector:
            raise ValueError("fixed and moving connectors must be different")
        _require_finite_nonnegative(self.gap_m, "gap_m")
        _require_finite(self.orientation_rad, "orientation_rad")
        _require_finite_nonnegative(self.approach_speed_m_s, "approach_speed_m_s")
        _require_finite_positive(self.dt_s, "dt_s")
        if self.release_after_s is not None:
            _require_finite_nonnegative(self.release_after_s, "release_after_s")
        if self.retract_speed_m_s is not None:
            _require_finite_nonnegative(self.retract_speed_m_s, "retract_speed_m_s")


@dataclass(frozen=True, slots=True)
class DockingPairScenarioStatus:
    """Immutable status copied out of a running docking scenario."""

    phase: DockingPairPhase
    time_s: float
    target_connection_id: ConnectionId
    connected: bool
    release_requested: bool


@dataclass(slots=True)
class DockingPairScenario:
    """Drive, explicitly latch, and optionally release one connector pair.

    The controller is backend-neutral and deliberately targets only the pair in
    :class:`DockingPairScenarioConfig`. It never emits its own world events;
    connector, connection, and assembly events continue to come exclusively
    from :class:`RuntimeSession` and therefore remain canonical.
    """

    session: RuntimeSession
    config: DockingPairScenarioConfig
    setup: DockingPairSetup
    _started_at_s: float
    _target_connection_id: ConnectionId
    _phase: DockingPairPhase = DockingPairPhase.APPROACHING
    _release_requested: bool = False
    _undock_command_queued: bool = False
    _retraction_started: bool = False

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        config: DockingPairScenarioConfig,
    ) -> DockingPairScenario:
        """Stage ``config`` in ``session`` and start the moving module."""
        setup = stage_docking_pair(
            session,
            config.fixed_connector,
            config.moving_connector,
            gap_m=config.gap_m,
            orientation_rad=config.orientation_rad,
        )
        adapter = session.adapter
        if not isinstance(adapter, SupportsModuleKinematics):  # pragma: no cover - staged above
            raise ScenarioSetupError(
                f"backend '{adapter.capabilities().name}' cannot drive modules"
            )
        try:
            adapter.set_module_twist(
                setup.moving_module,
                linear_m_s=vec_scale(setup.approach_direction, config.approach_speed_m_s),
            )
        except BackendError as error:
            raise ScenarioSetupError(f"backend could not start the approach: {error}") from error
        return cls(
            session=session,
            config=config,
            setup=setup,
            _started_at_s=session.world.time_s,
            _target_connection_id=connection_id(
                config.fixed_connector,
                config.moving_connector,
            ),
        )

    @property
    def status(self) -> DockingPairScenarioStatus:
        """Return an immutable status safe to hand to an inspector client."""
        return DockingPairScenarioStatus(
            phase=self._phase,
            time_s=self.session.world.time_s,
            target_connection_id=self._target_connection_id,
            connected=self._target_connection_id in self.session.world.connections,
            release_requested=self._release_requested,
        )

    @property
    def target_connection_id(self) -> ConnectionId:
        """Return the deterministic connection ID for the selected pair."""
        return self._target_connection_id

    def step(self) -> tuple[Event, ...]:
        """Advance one configured step and update the presentation phase."""
        world = self.session.world
        connected_before = self._target_connection_id in world.connections
        if self._release_is_due() and not self._release_requested:
            self._release_requested = True
            self._phase = (
                DockingPairPhase.RELEASING if connected_before else DockingPairPhase.COMPLETE
            )
        if self._release_requested and connected_before and not self._undock_command_queued:
            self.session.request_undock(self._target_connection_id)
            self._undock_command_queued = True

        if not self._release_requested and not connected_before:
            self._request_target_if_acceptable()

        events = self.session.step(self.config.dt_s)
        connected_after = self._target_connection_id in world.connections
        if any(isinstance(event, DockFailed | UndockFailed) for event in events):
            self._phase = DockingPairPhase.FAILED
        elif connected_after:
            self._phase = (
                DockingPairPhase.RELEASING if self._release_requested else DockingPairPhase.DOCKED
            )
        elif self._release_requested:
            self._start_retraction()
        else:
            self._phase = DockingPairPhase.APPROACHING
        return events

    def _release_is_due(self) -> bool:
        release_after_s = self.config.release_after_s
        return (
            release_after_s is not None
            and self.session.world.time_s >= self._started_at_s + release_after_s
        )

    def _request_target_if_acceptable(self) -> None:
        for proposal in self.session.proposals():
            if proposal.connection_id != self._target_connection_id:
                continue
            if proposal.compatibility.compatible and proposal.acceptance.satisfied:
                self.session.request_dock(
                    self.config.fixed_connector,
                    self.config.moving_connector,
                )
            return

    def _start_retraction(self) -> None:
        if self._retraction_started:
            return
        speed = (
            self.config.approach_speed_m_s
            if self.config.retract_speed_m_s is None
            else self.config.retract_speed_m_s
        )
        if speed <= 0.0:
            self._phase = DockingPairPhase.COMPLETE
            self._retraction_started = True
            return
        adapter = self.session.adapter
        if not isinstance(adapter, SupportsModuleKinematics):  # pragma: no cover - staged above
            raise ScenarioSetupError(
                f"backend '{adapter.capabilities().name}' cannot retract modules"
            )
        try:
            adapter.set_module_twist(
                self.setup.moving_module,
                linear_m_s=vec_scale(self.setup.approach_direction, -speed),
            )
        except BackendError as error:
            raise ScenarioSetupError(f"backend could not start retraction: {error}") from error
        self._retraction_started = True
        self._phase = DockingPairPhase.RETRACTING


def stage_docking_pair(
    session: RuntimeSession,
    fixed_connector: ConnectorInstanceId,
    moving_connector: ConnectorInstanceId,
    *,
    gap_m: float,
    orientation_rad: float = 0.0,
) -> DockingPairSetup:
    """Place ``moving_connector`` directly in front of ``fixed_connector``.

    The moving module is rigidly repositioned so that the connector axes are
    antiparallel, the requested roll is applied, and the connector origins are
    separated by ``gap_m`` along the fixed connector's outward docking axis.
    The returned approach direction points from the moving connector toward
    the fixed connector.

    This is scenario setup, not a docking decision. Compatibility, acceptance,
    policy, and the physical-constraint two-phase commit are still evaluated by
    the normal runtime pipeline.
    """
    if gap_m < 0.0:
        raise ScenarioSetupError("connector gap must not be negative")
    if not isinstance(session.adapter, SupportsModuleKinematics):
        raise ScenarioSetupError(
            f"backend '{session.adapter.capabilities().name}' cannot reposition modules"
        )

    try:
        fixed = session.world.connector(fixed_connector)
        moving = session.world.connector(moving_connector)
    except KeyError as error:
        raise ScenarioSetupError(str(error)) from error
    if fixed.module_id == moving.module_id:
        raise ScenarioSetupError("a docking demo requires connectors on different modules")
    if not fixed.resolved or not moving.resolved:
        raise ScenarioSetupError("connector frames must be resolved before arranging a pair")

    moving_module = session.world.modules[moving.module_id]
    module_to_connector = moving.world_pose.relative_to(moving_module.pose)
    nominal = nominal_relative_transform(fixed, moving, orientation_rad)
    target_connector = fixed.world_pose.compose(nominal)
    target_connector = Transform(
        translation=(
            fixed.world_pose.translation[0] + fixed.world_docking_axis[0] * gap_m,
            fixed.world_pose.translation[1] + fixed.world_docking_axis[1] * gap_m,
            fixed.world_pose.translation[2] + fixed.world_docking_axis[2] * gap_m,
        ),
        rotation=target_connector.rotation,
    )
    target_module = target_connector.compose(module_to_connector.inverse())

    try:
        session.adapter.set_module_pose(moving.module_id, target_module)
        session.world.ingest(session.adapter.snapshot())
    except BackendError as error:
        raise ScenarioSetupError(
            f"backend could not arrange the connector pair: {error}"
        ) from error

    return DockingPairSetup(
        fixed_connector=fixed_connector,
        moving_connector=moving_connector,
        moving_module=moving.module_id,
        approach_direction=vec_scale(fixed.world_docking_axis, -1.0),
        gap_m=gap_m,
    )


def _require_finite(value: float, field_name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite")


def _require_finite_nonnegative(value: float, field_name: str) -> None:
    _require_finite(value, field_name)
    if value < 0.0:
        raise ValueError(f"{field_name} must not be negative")


def _require_finite_positive(value: float, field_name: str) -> None:
    _require_finite(value, field_name)
    if value <= 0.0:
        raise ValueError(f"{field_name} must be greater than zero")
