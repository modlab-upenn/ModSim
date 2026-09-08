"""Backend-neutral momentum-driven edge-pivot orchestration.

The controller in this module owns actuator timing and connector lifecycle,
not rigid-body motion.  It may stage the initial face bond at simulation time
zero; after creation it sends only bounded joint-effort commands and ordinary
dock/undock requests.  A physics backend remains responsible for gravity,
contact, flywheel reaction torque, and the temporary edge hinge.

The primitive replaces an initial fixed face bond with an authored hinge,
brakes a spinning flywheel, and commits a target face from measured connector
frames.  The target may belong to the same support (a convex half-turn) or the
next support (a quarter-turn surface traverse).  Multi-action orchestration is
kept separate so this controller remains one testable physical operation.  The
deterministic constraint transition stands in for an unresolved magnetic
force-versus-distance model; it is not a three-plane carrier implementation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from modsim.backends.base import BackendError
from modsim.core.entities import JointCommand
from modsim.core.events import DockFailed, Event, UndockFailed
from modsim.core.ids import (
    ConnectionId,
    JointInstanceId,
    ModuleInstanceId,
    connection_id,
    joint_instance_id,
    split_connector_instance_id,
)
from modsim.core.transforms import Vec3, quat_rotate, signed_angle_about
from modsim.core.validation import (
    require_finite,
    require_finite_nonnegative,
    require_finite_positive,
)
from modsim.robot_packs.schema import ControlMode, PhysicalConstraintType
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationPhase,
    ReconfigurationScenarioError,
    ReconfigurationStatus,
    stage_docking_assembly_pair,
)
from modsim.runtime.session import JointCommandError, RuntimeSession

PUBLISHED_FLYWHEEL_AXIAL_INERTIA_KG_M2 = 8.4e-6
PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S = 20_000.0 * math.tau / 60.0
PUBLISHED_SPINUP_EFFORT_CAP_NM = 0.03
PUBLISHED_BRAKE_EFFORT_CAP_NM = 2.6
MAX_MOMENTUM_TIMESTEP_S = 0.0005

_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class MomentumPivotConfig:
    """Connector, actuator, and timing inputs for one physical edge pivot.

    ``initial_face`` and ``edge_hinge`` are directed from the current support
    to the moving module. ``target_face`` is directed from the destination
    support to that same moving module; the destination may be a different
    module for a surface traverse. ``target_pivot_angle_rad`` documents the
    expected positive roll magnitude; target capture must satisfy both normal
    connector acceptance and ``capture_angle_tolerance_rad`` rather than
    forcing that angle. The initial face is staged only by
    :meth:`MomentumPivotScenario.create`; sequence owners use
    :meth:`create_from_existing` after staging their complete initial tree.
    """

    initial_face: ConnectorPairRef
    edge_hinge: ConnectorPairRef
    target_face: ConnectorPairRef
    flywheel_joint: str = "flywheel_spin"
    dt_s: float = 0.00025
    settle_s: float = 0.25
    spinup_effort_nm: float = PUBLISHED_SPINUP_EFFORT_CAP_NM
    target_flywheel_speed_rad_s: float = 9_000.0 * math.tau / 60.0
    spinup_speed_tolerance_rad_s: float = 1.0
    brake_effort_nm: float = PUBLISHED_BRAKE_EFFORT_CAP_NM
    flywheel_axial_inertia_kg_m2: float = PUBLISHED_FLYWHEEL_AXIAL_INERTIA_KG_M2
    flywheel_stop_speed_rad_s: float = 1.0
    capture_distance_m: float = 0.001
    capture_angle_tolerance_rad: float = math.radians(20.0)
    spinup_timeout_s: float = 2.0
    brake_timeout_s: float = 0.1
    pivot_timeout_s: float = 3.0
    connected_hold_s: float = 0.25
    spin_direction: int = 1
    pivot_axis_in_fixed_frame: Vec3 = (0.0, 1.0, 0.0)
    pivot_reference_in_moving_frame: Vec3 = (0.0, 0.0, 1.0)
    target_pivot_angle_rad: float = math.pi
    plan_id: str = "mblocks_two_module_momentum_pivot"
    plan_name: str = "M-Blocks two-module physical edge roll"

    def __post_init__(self) -> None:
        if not self.flywheel_joint.strip():
            raise ValueError("flywheel_joint must not be empty")
        if not self.plan_id.strip():
            raise ValueError("plan_id must not be empty")
        if not self.plan_name.strip():
            raise ValueError("plan_name must not be empty")
        if self.spin_direction not in (-1, 1):
            raise ValueError("spin_direction must be -1 or 1")

        for name, value in (
            ("dt_s", self.dt_s),
            ("spinup_effort_nm", self.spinup_effort_nm),
            ("target_flywheel_speed_rad_s", self.target_flywheel_speed_rad_s),
            ("spinup_speed_tolerance_rad_s", self.spinup_speed_tolerance_rad_s),
            ("brake_effort_nm", self.brake_effort_nm),
            ("flywheel_axial_inertia_kg_m2", self.flywheel_axial_inertia_kg_m2),
            ("flywheel_stop_speed_rad_s", self.flywheel_stop_speed_rad_s),
            ("capture_distance_m", self.capture_distance_m),
            ("capture_angle_tolerance_rad", self.capture_angle_tolerance_rad),
            ("spinup_timeout_s", self.spinup_timeout_s),
            ("brake_timeout_s", self.brake_timeout_s),
            ("pivot_timeout_s", self.pivot_timeout_s),
            ("target_pivot_angle_rad", self.target_pivot_angle_rad),
        ):
            require_finite_positive(value, name)
        for name, value in (
            ("settle_s", self.settle_s),
            ("connected_hold_s", self.connected_hold_s),
        ):
            require_finite_nonnegative(value, name)

        if self.dt_s > MAX_MOMENTUM_TIMESTEP_S:
            raise ValueError(
                f"dt_s must not exceed {MAX_MOMENTUM_TIMESTEP_S:g} for momentum braking"
            )
        if self.spinup_effort_nm > PUBLISHED_SPINUP_EFFORT_CAP_NM:
            raise ValueError("spinup_effort_nm must not exceed the 0.03 N m reference ceiling")
        if self.brake_effort_nm > PUBLISHED_BRAKE_EFFORT_CAP_NM:
            raise ValueError("brake_effort_nm must not exceed the 2.6 N m reference ceiling")
        if self.target_flywheel_speed_rad_s > PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S:
            raise ValueError(
                "target_flywheel_speed_rad_s must not exceed the 20,000 RPM reference ceiling"
            )
        if self.spinup_speed_tolerance_rad_s >= self.target_flywheel_speed_rad_s:
            raise ValueError("spinup_speed_tolerance_rad_s must be below the target speed")
        if self.capture_angle_tolerance_rad >= math.pi / 2.0:
            raise ValueError("capture_angle_tolerance_rad must be below pi/2")
        if self.target_pivot_angle_rad > math.pi:
            raise ValueError("target_pivot_angle_rad must not exceed pi")

        for name, vector in (
            ("pivot_axis_in_fixed_frame", self.pivot_axis_in_fixed_frame),
            ("pivot_reference_in_moving_frame", self.pivot_reference_in_moving_frame),
        ):
            for index, component in enumerate(vector):
                require_finite(component, f"{name}[{index}]")
            magnitude = math.sqrt(sum(component * component for component in vector))
            if not math.isclose(magnitude, 1.0, rel_tol=0.0, abs_tol=1e-6):
                raise ValueError(f"{name} must be a unit vector")
        alignment = sum(
            axis * reference
            for axis, reference in zip(
                self.pivot_axis_in_fixed_frame,
                self.pivot_reference_in_moving_frame,
                strict=True,
            )
        )
        if abs(alignment) >= 1.0 - 1e-6:
            raise ValueError("pivot reference must not be parallel to the pivot axis")

        fixed_module = self.fixed_module
        moving_module = self.moving_module
        if fixed_module == moving_module:
            raise ValueError("momentum pivot requires two different modules")
        hinge_fixed = split_connector_instance_id(self.edge_hinge.fixed_connector)[0]
        hinge_moving = split_connector_instance_id(self.edge_hinge.moving_connector)[0]
        if (hinge_fixed, hinge_moving) != (fixed_module, moving_module):
            raise ValueError(
                "edge_hinge must use the same directed support and moving modules as initial_face"
            )
        target_fixed = split_connector_instance_id(self.target_face.fixed_connector)[0]
        target_moving = split_connector_instance_id(self.target_face.moving_connector)[0]
        if target_moving != moving_module:
            raise ValueError("target_face must retain the initial moving module")
        if target_fixed == moving_module:
            raise ValueError("target_face support and moving modules must differ")
        connection_ids = {
            self.initial_face.connection_id,
            self.edge_hinge.connection_id,
            self.target_face.connection_id,
        }
        if len(connection_ids) != 3:
            raise ValueError("initial face, edge hinge, and target face connections must differ")

    @property
    def fixed_module(self) -> ModuleInstanceId:
        """Return the support-side module."""
        return split_connector_instance_id(self.initial_face.fixed_connector)[0]

    @property
    def moving_module(self) -> ModuleInstanceId:
        """Return the flywheel-actuated module."""
        return split_connector_instance_id(self.initial_face.moving_connector)[0]

    @property
    def target_module(self) -> ModuleInstanceId:
        """Return the destination support module."""
        return split_connector_instance_id(self.target_face.fixed_connector)[0]

    @property
    def flywheel(self) -> JointInstanceId:
        """Return the moving module's flywheel joint instance."""
        return joint_instance_id(self.moving_module, self.flywheel_joint)


@dataclass(frozen=True, slots=True)
class MomentumPivotTelemetry:
    """Controller measurements useful to tests and future runtime views."""

    flywheel_speed_rad_s: float
    commanded_effort_nm: float
    pivot_angle_rad: float
    maximum_pivot_angle_rad: float
    brake_impulse_nms: float
    maximum_brake_effort_nm: float
    brake_started_at_s: float | None
    brake_ended_at_s: float | None
    brake_end_flywheel_speed_rad_s: float | None
    capture_time_s: float | None


@dataclass(slots=True)
class MomentumPivotScenario:
    """Spin, brake, edge-pivot, and capture using backend-integrated physics."""

    session: RuntimeSession
    config: MomentumPivotConfig
    _phase: ReconfigurationPhase
    _phase_started_at_s: float
    _detail: str
    _initial_pivot_reference: Vec3
    _action_index: int
    _action_count: int
    _baseline_connections: frozenset[ConnectionId]
    _commanded_effort_nm: float = 0.0
    _pivot_angle_rad: float = 0.0
    _maximum_pivot_angle_rad: float = 0.0
    _brake_impulse_nms: float = 0.0
    _maximum_brake_effort_nm: float = 0.0
    _brake_started_at_s: float | None = None
    _brake_ended_at_s: float | None = None
    _brake_end_flywheel_speed_rad_s: float | None = None
    _capture_time_s: float | None = None

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        config: MomentumPivotConfig,
    ) -> MomentumPivotScenario:
        """Preflight effort/hinge semantics and stage the initial face at time zero."""
        if abs(session.world.time_s) > _EPSILON:
            raise ReconfigurationScenarioError(
                "momentum pivot must be created at simulation time zero"
            )
        scenario = cls(
            session=session,
            config=config,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _detail=("Settling the fixed face with its coincident edge magnets pre-engaged"),
            _initial_pivot_reference=config.pivot_reference_in_moving_frame,
            _action_index=0,
            _action_count=1,
            _baseline_connections=frozenset(),
        )
        scenario._preflight(require_fresh_world=True)
        try:
            stage_docking_assembly_pair(
                session,
                config.initial_face.fixed_connector,
                config.initial_face.moving_connector,
                gap_m=0.0,
                preserve_orientation=True,
            )
        except Exception as error:
            if isinstance(error, ReconfigurationScenarioError):
                raise
            raise ReconfigurationScenarioError(
                f"could not stage the initial momentum-pivot face bond: {error}"
            ) from error

        session.request_dock(*config.initial_face.connectors)
        events = session.process_docking()
        if config.initial_face.connection_id not in session.world.connections:
            failure = _dock_failure(events, config.initial_face.connection_id)
            detail = (
                f"{failure.reason.value}: {failure.detail}"
                if failure is not None
                else "runtime produced no committed connection"
            )
            raise ReconfigurationScenarioError(f"initial face bond failed: {detail}")

        scenario._activate_existing_face()
        return scenario

    @classmethod
    def create_from_existing(
        cls,
        session: RuntimeSession,
        config: MomentumPivotConfig,
        *,
        action_index: int,
        action_count: int,
    ) -> tuple[MomentumPivotScenario, tuple[Event, ...]]:
        """Start one pivot from an already committed initial face.

        This entry point never stages or writes a module root. It is intended
        for a sequence owner that established its full initial topology at
        time zero. The returned events are the transient hinge commit produced
        during activation and therefore belong to the caller's current step.
        """
        if action_count <= 0:
            raise ValueError("action_count must be positive")
        if action_index < 0 or action_index >= action_count:
            raise ValueError("action_index must be within action_count")
        scenario = cls(
            session=session,
            config=config,
            _phase=ReconfigurationPhase.HOLDING_INITIAL,
            _phase_started_at_s=session.world.time_s,
            _detail="Settling the committed face with its edge hinge pre-engaged",
            _initial_pivot_reference=config.pivot_reference_in_moving_frame,
            _action_index=action_index,
            _action_count=action_count,
            _baseline_connections=frozenset(),
        )
        scenario._preflight(require_fresh_world=False)
        events = scenario._activate_existing_face()
        return scenario, events

    def _activate_existing_face(self) -> tuple[Event, ...]:
        initial = self.session.world.connections.get(self.config.initial_face.connection_id)
        if initial is None:
            raise ReconfigurationScenarioError(
                f"initial face '{self.config.initial_face.connection_id}' is not committed"
            )
        if initial.constraint is not PhysicalConstraintType.FIXED:
            raise ReconfigurationScenarioError(
                f"initial face '{initial.id}' must be a fixed connection"
            )
        for label, connection in (
            ("edge hinge", self.config.edge_hinge.connection_id),
            ("target face", self.config.target_face.connection_id),
        ):
            if connection in self.session.world.connections:
                raise ReconfigurationScenarioError(
                    f"{label} '{connection}' is already committed before pivot activation"
                )
        self._baseline_connections = frozenset(self.session.world.connections)

        # Pre-engage the already coincident edge while the face weld still
        # holds exact geometry. Runtime spin-up can elastically load a solver
        # weld by millimetres; waiting until brake time would then ask the
        # narrow hinge capture gate to hide that drift. Real M-Blocks also have
        # edge magnets present while a face is bonded.
        events = self._engage_edge_hinge()
        fixed_pose = self.session.world.modules[self.config.fixed_module].pose
        moving_pose = self.session.world.modules[self.config.moving_module].pose
        relative = moving_pose.relative_to(fixed_pose)
        self._initial_pivot_reference = quat_rotate(
            relative.rotation,
            self.config.pivot_reference_in_moving_frame,
        )
        self._set_effort(0.0)
        return events

    @property
    def status(self) -> ReconfigurationStatus:
        """Return immutable progress for a runtime owner or inspector."""
        action_index = (
            None
            if self._phase is ReconfigurationPhase.HOLDING_INITIAL and self._action_count == 1
            else self._action_index
        )
        return ReconfigurationStatus(
            phase=self._phase,
            time_s=self.session.world.time_s,
            plan_id=self.config.plan_id,
            plan_name=self.config.plan_name,
            action_index=action_index,
            action_count=self._action_count,
            detail=self._detail,
        )

    @property
    def telemetry(self) -> MomentumPivotTelemetry:
        """Return the latest actuator and pivot measurements."""
        return MomentumPivotTelemetry(
            flywheel_speed_rad_s=self._flywheel_velocity(),
            commanded_effort_nm=self._commanded_effort_nm,
            pivot_angle_rad=self._pivot_angle_rad,
            maximum_pivot_angle_rad=self._maximum_pivot_angle_rad,
            brake_impulse_nms=self._brake_impulse_nms,
            maximum_brake_effort_nm=self._maximum_brake_effort_nm,
            brake_started_at_s=self._brake_started_at_s,
            brake_ended_at_s=self._brake_ended_at_s,
            brake_end_flywheel_speed_rad_s=self._brake_end_flywheel_speed_rad_s,
            capture_time_s=self._capture_time_s,
        )

    def step(self) -> tuple[Event, ...]:
        """Advance one physics step and update the momentum-pivot controller."""
        if self._phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            return self.session.step(self.config.dt_s, process_connectors=False)

        pivoting = self._phase is ReconfigurationPhase.PIVOTING
        if pivoting and self._brake_ended_at_s is None:
            self._brake_impulse_nms += abs(self._commanded_effort_nm) * self.config.dt_s
            self._maximum_brake_effort_nm = max(
                self._maximum_brake_effort_nm,
                abs(self._commanded_effort_nm),
            )
        events: list[Event] = list(
            self.session.step(
                self.config.dt_s,
                # This authored controller owns every constraint transition:
                # initial/hinge/target requests are evaluated explicitly at
                # their phase boundaries. Passive auto-latching here would
                # scan unrelated ports and could create topology not present
                # in the physical route.
                process_connectors=False,
            )
        )

        if self._phase is ReconfigurationPhase.HOLDING_INITIAL:
            if self._held_for(self.config.settle_s):
                self._enter_phase(
                    ReconfigurationPhase.APPROACHING,
                    "Spinning the one-plane flywheel with at most 0.03 N m effort",
                )
                self._set_spinup_effort()
        elif self._phase is ReconfigurationPhase.APPROACHING:
            events.extend(self._advance_spinup())
        elif self._phase is ReconfigurationPhase.PIVOTING:
            events.extend(self._advance_pivot())
        elif self._phase is ReconfigurationPhase.HOLDING_CONNECTED:
            if self.config.target_face.connection_id not in self.session.world.connections:
                self._fail("captured target face did not remain connected")
            elif self._held_for(self.config.connected_hold_s):
                self._complete()
        else:  # pragma: no cover - private phase invariant
            raise AssertionError(f"unhandled momentum-pivot phase '{self._phase}'")
        return tuple(events)

    def _advance_spinup(self) -> tuple[Event, ...]:
        projected_speed = self.config.spin_direction * self._flywheel_velocity()
        self._detail = (
            f"Flywheel spin-up: {projected_speed * 60.0 / math.tau:.0f} RPM / "
            f"{self.config.target_flywheel_speed_rad_s * 60.0 / math.tau:.0f} RPM; "
            f"effort {abs(self._commanded_effort_nm):.4g} N m"
        )
        if abs(self._flywheel_velocity()) > PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S + 1e-6:
            self._fail("flywheel exceeded the 20,000 RPM safety ceiling")
            return ()
        if (
            projected_speed + self.config.spinup_speed_tolerance_rad_s + _EPSILON
            >= self.config.target_flywheel_speed_rad_s
        ):
            self._set_effort(0.0)
            transition_events = self._release_initial_face_to_hinge()
            if self._phase is ReconfigurationPhase.FAILED:
                return transition_events
            self._brake_started_at_s = self.session.world.time_s
            self._enter_phase(
                ReconfigurationPhase.PIVOTING,
                "Initial face released onto the +Y edge hinge; applying bounded brake impulse",
            )
            self._set_brake_effort()
            return transition_events
        if self._held_for(self.config.spinup_timeout_s):
            self._fail(
                "timed out spinning the flywheel to "
                f"{self.config.target_flywheel_speed_rad_s:.6g} rad/s"
            )
            return ()
        self._set_spinup_effort()
        return ()

    def _advance_pivot(self) -> tuple[Event, ...]:
        events: list[Event] = []
        self._measure_pivot_angle()
        projected_speed = self.config.spin_direction * self._flywheel_velocity()
        mode = "braking" if self._brake_ended_at_s is None else "coasting"
        self._detail = (
            f"Edge pivot {mode}: flywheel {projected_speed:.1f} rad/s; "
            f"angle {math.degrees(self._pivot_angle_rad):.1f} deg / "
            f"{math.degrees(self.config.target_pivot_angle_rad):.1f} deg; "
            f"brake impulse {self._brake_impulse_nms:.5g} N m s"
        )
        if self._brake_ended_at_s is None:
            if projected_speed <= self.config.flywheel_stop_speed_rad_s + _EPSILON:
                self._finish_brake()
            elif (
                self._brake_started_at_s is not None
                and self.session.world.time_s - self._brake_started_at_s + _EPSILON
                >= self.config.brake_timeout_s
            ):
                self._fail("mechanical brake did not bring the flywheel to zero in time")
                return ()
            else:
                self._set_brake_effort()

        events.extend(self._capture_target_face())
        if self._phase is ReconfigurationPhase.FAILED:
            return tuple(events)
        if (
            self.config.target_face.connection_id in self.session.world.connections
            and self._brake_ended_at_s is not None
        ):
            release_events, released = self._release_edge_hinge()
            events.extend(release_events)
            if released:
                self._enter_phase(
                    ReconfigurationPhase.HOLDING_CONNECTED,
                    "Measured target face committed; transient edge hinge released",
                )
            return tuple(events)

        if self._held_for(self.config.pivot_timeout_s):
            self._fail(
                "timed out before the moving module entered measured target-face acceptance "
                f"(pivot angle {self._pivot_angle_rad:.6g} rad)"
            )
        return tuple(events)

    def _engage_edge_hinge(self) -> tuple[Event, ...]:
        """Commit the coincident edge while the initial face holds staging geometry."""
        self.session.request_dock(*self.config.edge_hinge.connectors)
        events = self.session.process_docking()
        if self.config.edge_hinge.connection_id not in self.session.world.connections:
            failure = _dock_failure(events, self.config.edge_hinge.connection_id)
            detail = (
                f"{failure.reason.value}: {failure.detail}"
                if failure is not None
                else "runtime produced no committed hinge"
            )
            raise ReconfigurationScenarioError(
                f"could not engage the transient edge hinge: {detail}"
            )
        expected = set(self._baseline_connections)
        expected.add(self.config.edge_hinge.connection_id)
        mismatch = self._topology_mismatch(expected)
        if mismatch is not None:
            raise ReconfigurationScenarioError(
                "transient hinge activation changed unrelated topology " + mismatch
            )
        return events

    def _release_initial_face_to_hinge(self) -> tuple[Event, ...]:
        """Release the face only after the pre-engaged edge hinge is active."""
        if self.config.edge_hinge.connection_id not in self.session.world.connections:
            self._fail("transient edge hinge disappeared before face release")
            return ()
        self.session.request_undock(self.config.initial_face.connection_id)
        events = self.session.process_docking()
        failure = _undock_failure(events, self.config.initial_face.connection_id)
        if failure is not None:
            self._fail(f"could not release the initial face bond: {failure.detail}")
        elif self.config.initial_face.connection_id in self.session.world.connections:
            self._fail("initial face bond remained active after release")
        else:
            expected = set(self._baseline_connections)
            expected.discard(self.config.initial_face.connection_id)
            expected.add(self.config.edge_hinge.connection_id)
            mismatch = self._topology_mismatch(expected)
            if mismatch is not None:
                self._fail("initial-face release changed unrelated topology " + mismatch)
        return events

    def _capture_target_face(self) -> tuple[Event, ...]:
        target = self.config.target_face.connection_id
        if target in self.session.world.connections:
            if self._capture_time_s is None:
                self._capture_time_s = self.session.world.time_s
            return ()
        proposal = next(
            (proposal for proposal in self.session.proposals() if proposal.connection_id == target),
            None,
        )
        if proposal is None or not (
            proposal.compatibility.compatible and proposal.acceptance.satisfied
        ):
            return ()
        position_error_m = proposal.acceptance.criterion("position").measured
        if position_error_m > self.config.capture_distance_m:
            return ()
        angle_error_rad = abs(abs(self._pivot_angle_rad) - self.config.target_pivot_angle_rad)
        half_turn = math.isclose(
            self.config.target_pivot_angle_rad,
            math.pi,
            rel_tol=0.0,
            abs_tol=_EPSILON,
        )
        direction_matches = half_turn or self._pivot_angle_rad > 0.0
        if not direction_matches or angle_error_rad > self.config.capture_angle_tolerance_rad:
            self._detail = (
                "Target face entered geometric acceptance, but pivot angle "
                f"{math.degrees(self._pivot_angle_rad):.1f} deg is outside the authored "
                f"+{math.degrees(self.config.target_pivot_angle_rad):.1f} +/- "
                f"{math.degrees(self.config.capture_angle_tolerance_rad):.1f} deg gate"
            )
            return ()

        self.session.request_dock(*self.config.target_face.connectors)
        events = self.session.process_docking()
        failure = _dock_failure(events, target)
        if failure is not None:
            self._fail(f"target-face capture failed ({failure.reason.value}): {failure.detail}")
        elif target in self.session.world.connections:
            self._capture_time_s = self.session.world.time_s
            expected = set(self._baseline_connections)
            expected.discard(self.config.initial_face.connection_id)
            expected.update((self.config.edge_hinge.connection_id, target))
            mismatch = self._topology_mismatch(expected)
            if mismatch is not None:
                self._fail("target-face capture changed unrelated topology " + mismatch)
            else:
                self._detail = (
                    "Measured target face captured; waiting for flywheel zero before hinge release"
                )
        else:
            self._fail("target-face dock request produced no committed connection")
        return events

    def _release_edge_hinge(self) -> tuple[tuple[Event, ...], bool]:
        hinge = self.config.edge_hinge.connection_id
        self.session.request_undock(hinge)
        events = self.session.process_docking()
        failure = _undock_failure(events, hinge)
        if failure is not None:
            self._fail(f"could not release the transient edge hinge: {failure.detail}")
            return events, False
        elif hinge in self.session.world.connections:
            self._fail("transient edge hinge remained active after release")
            return events, False
        expected = set(self._baseline_connections)
        expected.discard(self.config.initial_face.connection_id)
        expected.add(self.config.target_face.connection_id)
        mismatch = self._topology_mismatch(expected)
        if mismatch is not None:
            self._fail("edge-hinge release changed unrelated topology " + mismatch)
            return events, False
        return events, True

    def _set_spinup_effort(self) -> None:
        projected_speed = max(0.0, self.config.spin_direction * self._flywheel_velocity())
        remaining = max(0.0, self.config.target_flywheel_speed_rad_s - projected_speed)
        one_step_effort = self.config.flywheel_axial_inertia_kg_m2 * remaining / self.config.dt_s
        effort = min(self.config.spinup_effort_nm, one_step_effort)
        self._set_effort(self.config.spin_direction * effort)

    def _set_brake_effort(self) -> None:
        projected_speed = max(0.0, self.config.spin_direction * self._flywheel_velocity())
        one_step_effort = (
            self.config.flywheel_axial_inertia_kg_m2 * projected_speed / self.config.dt_s
        )
        effort = min(self.config.brake_effort_nm, one_step_effort)
        self._set_effort(-self.config.spin_direction * effort)

    def _finish_brake(self) -> None:
        self._brake_end_flywheel_speed_rad_s = self._flywheel_velocity()
        self._brake_ended_at_s = self.session.world.time_s
        self._set_effort(0.0)
        self._detail = (
            "Flywheel reached zero; coasting about the magnetic edge toward target capture"
        )

    def _set_effort(self, moving_effort_nm: float) -> None:
        fixed_joint = joint_instance_id(self.config.fixed_module, self.config.flywheel_joint)
        commands = (
            JointCommand(fixed_joint, ControlMode.EFFORT, 0.0),
            JointCommand(self.config.flywheel, ControlMode.EFFORT, moving_effort_nm),
        )
        try:
            self.session.set_joint_commands(commands)
        except (JointCommandError, BackendError) as error:
            raise ReconfigurationScenarioError(
                f"could not command momentum-pivot flywheel effort: {error}"
            ) from error
        self._commanded_effort_nm = moving_effort_nm

    def _preflight(self, *, require_fresh_world: bool) -> None:
        connector_pairs = (
            ("initial face", self.config.initial_face, PhysicalConstraintType.FIXED),
            ("edge hinge", self.config.edge_hinge, PhysicalConstraintType.HINGE),
            ("target face", self.config.target_face, PhysicalConstraintType.FIXED),
        )
        for label, pair, expected_constraint in connector_pairs:
            for connector_id in pair.connectors:
                try:
                    connector = self.session.world.connector(connector_id)
                    connector_type = self.session.world.connector_type(connector_id)
                except KeyError as error:
                    raise ReconfigurationScenarioError(
                        f"{label} references unknown connector '{connector_id}'"
                    ) from error
                if not connector.resolved:
                    raise ReconfigurationScenarioError(
                        f"{label} connector frame '{connector_id}' is unresolved"
                    )
                physical = connector_type.physical_connection
                if physical is None or physical.constraint is not expected_constraint:
                    actual = "undefined" if physical is None else physical.constraint.value
                    raise ReconfigurationScenarioError(
                        f"{label} connector '{connector_id}' requires "
                        f"'{expected_constraint.value}' physical connection, got '{actual}'"
                    )
        if require_fresh_world and self.session.world.connections:
            raise ReconfigurationScenarioError(
                "momentum pivot requires a fresh world with no active connections"
            )

        flywheel_ids = (
            joint_instance_id(self.config.fixed_module, self.config.flywheel_joint),
            self.config.flywheel,
        )
        for joint_id in flywheel_ids:
            try:
                state = self.session.world.joint_state(joint_id)
                spec = self.session.world.joint_spec(joint_id)
            except (KeyError, ValueError) as error:
                raise ReconfigurationScenarioError(
                    f"backend did not report required flywheel joint '{joint_id}'"
                ) from error
            if not all(math.isfinite(value) for value in (state.position, state.velocity)):
                raise ReconfigurationScenarioError(
                    f"backend reported non-finite flywheel state for '{joint_id}'"
                )
            limits = spec.limits
            if limits is None or limits.max_effort_nm is None:
                raise ReconfigurationScenarioError(
                    f"flywheel joint '{joint_id}' requires a finite effort limit"
                )
            if limits.max_effort_nm + _EPSILON < self.config.brake_effort_nm:
                raise ReconfigurationScenarioError(
                    f"flywheel joint '{joint_id}' effort limit {limits.max_effort_nm:g} N m "
                    f"is below requested brake effort {self.config.brake_effort_nm:g} N m"
                )
            if limits.max_velocity_rad_per_s is None:
                raise ReconfigurationScenarioError(
                    f"flywheel joint '{joint_id}' requires a finite velocity limit"
                )
            if limits.max_velocity_rad_per_s + _EPSILON < self.config.target_flywheel_speed_rad_s:
                raise ReconfigurationScenarioError(
                    f"flywheel joint '{joint_id}' velocity limit "
                    f"{limits.max_velocity_rad_per_s:g} rad/s is below target "
                    f"{self.config.target_flywheel_speed_rad_s:g} rad/s"
                )
        self._set_effort(0.0)

    def _flywheel_velocity(self) -> float:
        return self.session.world.joint_state(self.config.flywheel).velocity

    def _measure_pivot_angle(self) -> None:
        fixed = self.session.world.modules[self.config.fixed_module].pose
        moving = self.session.world.modules[self.config.moving_module].pose
        current_relative = moving.relative_to(fixed)
        current_reference = quat_rotate(
            current_relative.rotation,
            self.config.pivot_reference_in_moving_frame,
        )
        angle = signed_angle_about(
            self._initial_pivot_reference,
            current_reference,
            self.config.pivot_axis_in_fixed_frame,
        )
        self._pivot_angle_rad = angle
        self._maximum_pivot_angle_rad = max(self._maximum_pivot_angle_rad, abs(angle))

    def _held_for(self, duration_s: float) -> bool:
        return self.session.world.time_s - self._phase_started_at_s + _EPSILON >= duration_s

    def _topology_mismatch(self, expected: set[ConnectionId]) -> str | None:
        actual = set(self.session.world.connections)
        if actual == expected:
            return None
        return (
            f"(expected: {', '.join(sorted(expected)) or 'none'}; "
            f"active: {', '.join(sorted(actual)) or 'none'})"
        )

    def _enter_phase(self, phase: ReconfigurationPhase, detail: str) -> None:
        self._phase = phase
        self._phase_started_at_s = self.session.world.time_s
        self._detail = detail

    def _fail(self, detail: str) -> None:
        self._enter_phase(ReconfigurationPhase.FAILED, detail)
        self._set_effort(0.0)

    def _complete(self) -> None:
        initial = self.config.initial_face.connection_id
        hinge = self.config.edge_hinge.connection_id
        target = self.config.target_face.connection_id
        actual = set(self.session.world.connections)
        expected = set(self._baseline_connections)
        expected.discard(initial)
        expected.add(target)
        if actual != expected or hinge in actual:
            self._fail(
                "momentum pivot ended with the wrong connection topology "
                f"(expected: {', '.join(sorted(expected)) or 'none'}; "
                f"active: {', '.join(sorted(actual)) or 'none'})"
            )
            return
        self._enter_phase(
            ReconfigurationPhase.COMPLETE,
            "Completed one-plane physical face-to-hinge-to-face momentum pivot",
        )
        self._set_effort(0.0)


def _dock_failure(events: tuple[Event, ...], target: ConnectionId) -> DockFailed | None:
    return next(
        (
            event
            for event in events
            if isinstance(event, DockFailed)
            and connection_id(event.connector_a, event.connector_b) == target
        ),
        None,
    )


def _undock_failure(events: tuple[Event, ...], target: ConnectionId) -> UndockFailed | None:
    return next(
        (
            event
            for event in events
            if isinstance(event, UndockFailed) and event.connection_id == target
        ),
        None,
    )


__all__ = [
    "MAX_MOMENTUM_TIMESTEP_S",
    "PUBLISHED_BRAKE_EFFORT_CAP_NM",
    "PUBLISHED_FLYWHEEL_AXIAL_INERTIA_KG_M2",
    "PUBLISHED_FLYWHEEL_SPEED_CAP_RAD_S",
    "PUBLISHED_SPINUP_EFFORT_CAP_NM",
    "MomentumPivotConfig",
    "MomentumPivotScenario",
    "MomentumPivotTelemetry",
]
