"""Feedback execution of generated planar M-Blocks pivots.

Only the RuntimeSession owner reads WorldState or commands physics. Geometric
planning consumes immutable observations; a discrete move is committed only
after measured capture and settling. Attachment remains the documented ideal
face-weld/edge-hinge approximation, with no magnetic force-field claim.
"""

from __future__ import annotations

import math
from collections.abc import Generator
from dataclasses import dataclass, field
from time import perf_counter
from typing import cast

from modsim.core.entities import JointCommand
from modsim.core.events import Event
from modsim.core.ids import JointInstanceId, ModuleInstanceId
from modsim.core.transforms import Transform, quat_rotate, vec_norm
from modsim.planning.mblocks import (
    LatticeGoal,
    LatticePivot,
    LatticePlan,
    LatticePlanningError,
    LatticePlanningSnapshot,
    LatticeState,
    apply_pivot,
    plan_reconfiguration,
)
from modsim.planning.mblocks.models import LatticeActionRecord
from modsim.planning.mblocks.planner import recovery_search
from modsim.planning.models import PlannerDecision
from modsim.robot_packs.schema import ControlMode
from modsim.runtime.coordinated_pivot import CoordinatedMomentumPivotScenario
from modsim.runtime.mblocks_lattice import (
    PLAN_ID,
    PLAN_NAME,
    face_pairs,
    line_goal,
    make_pivot_plan,
    observation_frame,
    observe_lattice,
    starter_state,
)
from modsim.runtime.reconfiguration import (
    ReconfigurationPhase,
    ReconfigurationScenarioError,
    ReconfigurationStatus,
)
from modsim.runtime.session import RuntimeSession


@dataclass(slots=True)
class OnlineLatticeScenario:
    session: RuntimeSession
    plan: LatticePlan
    dt_s: float
    quarter_rpm: float = 14000.0
    half_rpm: float = 17000.0
    _state: LatticeState = field(init=False)
    _remaining: list[LatticePivot] = field(init=False)
    _controller: CoordinatedMomentumPivotScenario | None = None
    _active: LatticePivot | None = None
    _expected: LatticeState | None = None
    _phase: ReconfigurationPhase = ReconfigurationPhase.HOLDING_INITIAL
    _detail: str = "Settling the initial face-connected lattice"
    _wait_since: float = 0.0
    _completed: int = 0
    _revision: int = 1
    _replans: int = 0
    _planning_ms: float = 0.0
    _landing_error_m: float = 0.0
    _decisions: list[PlannerDecision] = field(default_factory=list[PlannerDecision])
    _history: list[LatticeActionRecord] = field(default_factory=list[LatticeActionRecord])
    _search: Generator[int, None, tuple[LatticePivot, ...]] | None = None
    _search_state: LatticeState | None = None
    _last_controller_phase: ReconfigurationPhase | None = None
    _landing_integral: float = 0.0
    _landing_effort_nm: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.dt_s) or not 0 < self.dt_s <= 0.0005:
            raise ValueError("tabletop planning requires 0 < dt_s <= 0.0005")
        for rpm in (self.quarter_rpm, self.half_rpm):
            if not math.isfinite(rpm) or not 0 < rpm <= 20000:
                raise ValueError("flywheel targets must be in (0, 20000] RPM")
        self._state = self.plan.initial
        self._remaining = list(self.plan.actions)

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        goal: LatticeGoal | None = None,
        *,
        initial: LatticeState | None = None,
        dt_s: float = 0.0005,
        quarter_rpm: float = 14000.0,
        half_rpm: float = 17000.0,
    ) -> OnlineLatticeScenario:
        if session.world.time_s != 0.0 or session.world.connections:
            raise ReconfigurationScenarioError(
                "online lattice initialization requires a fresh session"
            )
        if not session.adapter.capabilities().supports_joint_commands:
            raise ReconfigurationScenarioError(
                "online lattice execution requires joint/hinge dynamics"
            )
        requested = starter_state() if initial is None else initial
        selected_goal = line_goal(requested) if goal is None else goal
        if not math.isclose(selected_goal.pitch_m, 0.05, abs_tol=1e-9, rel_tol=0.0):
            raise ReconfigurationScenarioError("Physical M-Blocks require a 50 mm lattice pitch")
        if set(session.world.modules) != {b.id for b in requested.blocks}:
            raise ReconfigurationScenarioError("scene modules do not match the requested lattice")
        started = perf_counter()
        plan = plan_reconfiguration(requested, selected_goal)
        scenario = cls(session, plan, dt_s, quarter_rpm, half_rpm)
        scenario._planning_ms = (perf_counter() - started) * 1000
        observed, _ = observe_lattice(session, scenario.frame_transform, selected_goal.pitch_m)
        if observed != requested:
            raise ReconfigurationScenarioError(
                "initial measured poses differ from the requested lattice"
            )
        pairs = face_pairs(observed, scenario.frame_transform, selected_goal.pitch_m)
        for pair in pairs:
            session.request_dock(*pair.connectors)
        session.process_docking()
        if set(session.world.connections) != {pair.connection_id for pair in pairs}:
            raise ReconfigurationScenarioError(
                "could not establish the complete initial face topology"
            )
        scenario._decision(
            "planned", f"Sung 2015 planar construction: {len(plan.actions)} generated pivots"
        )
        return scenario

    @property
    def frame_transform(self) -> Transform:
        anchor = next(b for b in self.plan.initial.blocks if b.id == self.plan.anchor)
        return observation_frame(
            self.session,
            self.plan.anchor,
            self.plan.anchor_cell,
            anchor.quarter_turns,
            self.plan.goal.pitch_m,
        )

    @property
    def status(self) -> ReconfigurationStatus:
        return ReconfigurationStatus(
            phase=self._phase,
            time_s=self.session.world.time_s,
            plan_id=PLAN_ID,
            plan_name=PLAN_NAME,
            action_index=self._completed,
            action_count=self._completed + len(self._remaining) + int(self._active is not None),
            detail=self._detail,
        )

    @property
    def planning_snapshot(self) -> LatticePlanningSnapshot:
        revision = self.session.world.revision
        frame = self.frame_transform
        axis = frame.apply_direction((1.0, 0.0, 0.0))
        telemetry = self._controller.current_telemetry if self._controller is not None else None
        return LatticePlanningSnapshot(
            time_s=self.session.world.time_s,
            sample_sequence=revision.sample_sequence,
            topology_revision=revision.topology_revision,
            plan_revision=self._revision,
            goal=self.plan.goal,
            anchor=self.plan.anchor,
            origin_world_m=frame.translation,
            frame_yaw_rad=math.atan2(axis[1], axis[0]),
            blocks=self._state.blocks,
            active=self._active,
            remaining=tuple(self._remaining),
            boundary_order=self.plan.boundary_order,
            decisions=tuple(self._decisions[-256:]),
            history=tuple(self._history),
            completed_actions=self._completed,
            replans=self._replans,
            planning_ms=self._planning_ms,
            flywheel_rpm=(
                telemetry.flywheel_speeds_rad_s[0][1] * 60 / math.tau if telemetry else 0.0
            ),
            pivot_angle_deg=(math.degrees(telemetry.pivot_angle_rad) if telemetry else 0.0),
            landing_error_m=self._landing_error_m,
            landing_effort_nm=self._landing_effort_nm,
            phase="planning" if self._search is not None else self._phase.value,
            detail=self._detail,
        )

    def step(self) -> tuple[Event, ...]:
        if self._phase in (ReconfigurationPhase.COMPLETE, ReconfigurationPhase.FAILED):
            return self.session.step(self.dt_s, process_connectors=False)
        if self._controller is not None:
            events = self._controller.step()
            self._phase = self._controller.status.phase
            self._detail = self._controller.status.detail
            telemetry = self._controller.current_telemetry
            if telemetry.brake_ended_at_s is not None:
                if self._phase is ReconfigurationPhase.PIVOTING:
                    self._apply_landing_feedback()
                else:
                    self.session.clear_joint_commands()
                    self._landing_effort_nm = 0.0
            if (
                self._phase != self._last_controller_phase
                and self._phase is not ReconfigurationPhase.FAILED
            ):
                self._record_phase(self._phase.value)
                self._last_controller_phase = self._phase
            if self._phase is ReconfigurationPhase.FAILED:
                if not self._decisions or self._decisions[-1].kind != "failed":
                    self._fail(self._detail)
            elif self._phase is ReconfigurationPhase.COMPLETE:
                self._controller = None
                self._phase = ReconfigurationPhase.HOLDING_CONNECTED
                self._wait_since = self.session.world.time_s
                self._detail = "Verifying the settled measured lattice and all face captures"
            return events

        events = self.session.step(self.dt_s, process_connectors=False)
        if self._search is not None:
            self._advance_search()
            return events
        elapsed = self.session.world.time_s - self._wait_since
        if elapsed < 0.25:
            return events
        if not self._is_settled():
            if elapsed > 3.0:
                self._fail("The assembly did not settle within three seconds")
            return events
        try:
            observed, residual = observe_lattice(
                self.session,
                self.frame_transform,
                self.plan.goal.pitch_m,
            )
            expected_faces = face_pairs(observed, self.frame_transform, self.plan.goal.pitch_m)
            if set(self.session.world.connections) != {
                pair.connection_id for pair in expected_faces
            }:
                raise ReconfigurationScenarioError(
                    "Settled topology does not match measured face adjacency"
                )
            self._landing_error_m = residual
            self._state = observed
            if self._active is not None:
                self._decision(
                    "landed", f"{self._active.moving} captured; residual {residual * 1000:.2f} mm"
                )
                self._record_phase("verified")
                self._completed += 1
                self._active = None
                if observed != self._expected:
                    self._start_recovery(observed)
                    return events
                self._expected = None
            if observed.cells == frozenset(self.plan.goal.cells):
                self._phase = ReconfigurationPhase.COMPLETE
                self._detail = (
                    f"Target reached: {len(observed.blocks)} blocks, "
                    f"{self._completed} verified pivots"
                )
                self._decision("complete", self._detail)
                return events
            if not self._remaining:
                self._start_recovery(observed)
                return events
            action = self._remaining[0]
            self._expected = apply_pivot(observed, action)
            compiled = make_pivot_plan(
                self.session,
                observed,
                action,
                self.frame_transform,
                pitch_m=self.plan.goal.pitch_m,
                dt_s=self.dt_s,
                quarter_rpm=self.quarter_rpm,
                half_rpm=self.half_rpm,
            )
            self._controller = CoordinatedMomentumPivotScenario.create_from_existing(
                self.session, compiled, capture_ready=self._capture_ready
            )
            self._landing_integral = 0.0
            self._active = self._remaining.pop(0)
            self._phase = self._controller.status.phase
            self._last_controller_phase = None
            self._detail = self._controller.status.detail
            self._decision(
                "selected",
                f"{action.moving}: {action.source} → {action.destination}; "
                f"{abs(action.turns) * 90}° on {action.support}; {action.reason}",
            )
        except (ValueError, ReconfigurationScenarioError) as error:
            self._fail(str(error))
        return events

    def _landing_yaw_error(self) -> float:
        assert self._active is not None and self._expected is not None
        moving = self.session.world.modules[ModuleInstanceId(self._active.moving)]
        expected = next(b for b in self._expected.blocks if b.id == self._active.moving)
        axis = quat_rotate(moving.pose.relative_to(self.frame_transform).rotation, (1.0, 0.0, 0.0))
        return (
            expected.quarter_turns * math.pi / 2 - math.atan2(axis[1], axis[0]) + math.pi
        ) % math.tau - math.pi

    def _capture_ready(self) -> bool:
        # A relative pivot-angle gate alone can accumulate alignment error along
        # a line. Also check the moving cube against the anchor's measured frame.
        return abs(self._landing_yaw_error()) <= math.radians(0.75)

    def _apply_landing_feedback(self) -> None:
        """Simulation control extension: bounded motor reaction after the brake.

        This is not a controller reproduced from the geometric planning paper.
        It uses only the pack's flywheel effort interface, with no root control.
        """
        assert self._active is not None and self._controller is not None
        telemetry = self._controller.current_telemetry
        if any(abs(speed) > 20000 * math.tau / 60 for _, speed in telemetry.flywheel_speeds_rad_s):
            self._fail("Landing feedback exceeded the 20,000 RPM ceiling")
            return
        moving = self.session.world.modules[ModuleInstanceId(self._active.moving)]
        anchor = self.session.world.modules[ModuleInstanceId(self.plan.anchor)]
        error = self._landing_yaw_error()
        rate = moving.angular_velocity_rad_s[2] - anchor.angular_velocity_rad_s[2]
        self._landing_integral = max(-0.5, min(0.5, self._landing_integral + error * self.dt_s))
        effort = max(
            -0.03, min(0.03, -(0.12 * error + 0.04 * self._landing_integral - 0.008 * rate))
        )
        self._landing_effort_nm = effort
        self.session.set_joint_commands(
            (
                JointCommand(
                    JointInstanceId(f"{self._active.moving}/flywheel_spin"),
                    ControlMode.EFFORT,
                    effort,
                ),
            )
        )
        self._detail += f"; landing feedback {effort:.4f} N m"

    def _is_settled(self) -> bool:
        return all(
            vec_norm(module.linear_velocity_m_s) <= 0.01
            and vec_norm(module.angular_velocity_rad_s) <= 0.15
            for module in self.session.world.modules.values()
        )

    def _start_recovery(self, state: LatticeState) -> None:
        if self._replans >= 3:
            self._fail("Exhausted three settled-state replanning attempts")
            return
        self._replans += 1
        self._revision += 1
        self._remaining.clear()
        self._expected = None
        self._search_state = state
        self._search = recovery_search(state, self.plan.goal, self.plan.anchor)
        self._detail = "Replanning from the measured settled lattice"
        self._decision("replanning", self._detail)

    def _advance_search(self) -> None:
        assert self._search is not None
        start = perf_counter()
        try:
            expansions = next(self._search)
            self._detail = f"Recovery search: {expansions}/4000 expansions"
        except StopIteration as result:
            # Preserve the generator's declared return type across StopIteration.
            route = cast(tuple[LatticePivot, ...], result.value)
            self._search = None
            try:
                measured, _ = observe_lattice(
                    self.session, self.frame_transform, self.plan.goal.pitch_m
                )
            except ReconfigurationScenarioError as error:
                self._fail(str(error))
                return
            if measured != self._search_state:
                self._start_recovery(measured)
                return
            self._remaining = list(route)
            self._wait_since = self.session.world.time_s
            self._decision("replanned", f"Recovery found {len(self._remaining)} pivots")
        except LatticePlanningError as error:
            self._fail(str(error))
        finally:
            self._planning_ms = max(self._planning_ms, (perf_counter() - start) * 1000)

    def _record_phase(self, phase: str) -> None:
        if self._active is None:
            return
        now = self.session.world.time_s
        if self._history and self._history[-1].ended_at_s is None:
            self._history[-1] = self._history[-1].model_copy(update={"ended_at_s": now})
        self._history.append(
            LatticeActionRecord(
                index=self._completed,
                moving=self._active.moving,
                source=self._active.source,
                destination=self._active.destination,
                turns=self._active.turns,
                phase=phase,
                started_at_s=now,
                detail=self._detail,
                ended_at_s=now if phase in {"verified", "failed"} else None,
            )
        )

    def _decision(self, kind: str, detail: str) -> None:
        self._decisions.append(
            PlannerDecision(
                sequence=len(self._decisions),
                time_s=self.session.world.time_s,
                kind=kind,
                detail=detail,
            )
        )

    def _fail(self, detail: str) -> None:
        self._phase = ReconfigurationPhase.FAILED
        self._detail = detail
        self._search = None
        self._record_phase("failed")
        self.session.clear_joint_commands()
        self._landing_effort_nm = 0.0
        self._decision("failed", detail)
