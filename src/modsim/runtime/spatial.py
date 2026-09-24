"""ModSim-owned planning and measured execution of a supported spatial handoff.

The motion service provides only mechanical queries and motor commands. This
module owns search, command slew, the phase machine, docking/release decisions,
failure handling, and completion. It imports no simulator, NumPy, or GUI code.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Protocol

from modsim.core.events import Event
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, connection_id
from modsim.core.transforms import Vec3, vec_norm, vec_sub
from modsim.planning.inspection import (
    ActionInterval,
    PlannedTrace,
    PlannerDecision,
    SpatialAction,
    SpatialPlanningSnapshot,
    TargetBond,
    TargetModule,
)
from modsim.planning.spatial import (
    Bond,
    JointPoint,
    SpatialPlanningError,
    plan_joint_path,
    plan_support_changes,
    supported,
)
from modsim.runtime.reconfiguration import ReconfigurationPhase, ReconfigurationStatus
from modsim.runtime.session import RuntimeSession


@dataclass(frozen=True, slots=True)
class SpatialObservation:
    time_s: float
    penetration_m: float
    finite: bool


class SpatialMotionServices(Protocol):
    """Optional mechanical services for this experiment, beyond BackendAdapter."""

    @property
    def motor_keys(self) -> tuple[tuple[ModuleInstanceId, str], ...]: ...

    @property
    def target_positions(self) -> Mapping[str, Vec3]: ...

    def feasible(self, point: JointPoint, *, transferred: bool = False) -> bool:
        """Evaluate a candidate on isolated query state; never move live bodies."""
        ...

    def targets(self, point: JointPoint) -> tuple[float, ...]:
        """Map the benchmark's grid coordinates to named internal joint angles."""
        ...

    def configuration_positions(
        self, point: JointPoint, *, transferred: bool = False
    ) -> Mapping[str, Vec3]:
        """Return isolated query geometry for planner visualization."""
        ...

    def command_positions(self, positions: tuple[float, ...]) -> None:
        """Command provisioned, bounded-effort internal joint servos."""
        ...

    def observe_safety(self) -> SpatialObservation:
        """Read engine-specific contact diagnostics at the current world time."""
        ...


@dataclass(frozen=True, slots=True)
class SpatialHandoffProblem:
    modules: frozenset[str]
    anchors: frozenset[str]
    initial: frozenset[Bond]
    goal: frozenset[Bond]
    parked_helper: str
    start: tuple[int, ...] = (0, 0, 0)
    rendezvous: tuple[int, ...] = (6, 6, 6)
    final: tuple[int, ...] = (0, 6, 6)
    upper: tuple[int, ...] = (6, 6, 6)


class SpatialReconfigurationScenario:
    """Bounded two-stage search followed by a feedback-driven support transfer.

    This controller has one supplied rendezvous and a 3-joint coordinate mapping;
    it is not an arbitrary-topology task-and-motion planner. Actual connections
    and target geometry are always checked against the session's WorldState.
    """

    dt_s = 0.001

    def __init__(
        self,
        session: RuntimeSession,
        motion: SpatialMotionServices,
        problem: SpatialHandoffProblem,
        *,
        duration_s: float = 45.0,
    ) -> None:
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("duration must be finite and positive")
        self.session, self.motion, self.problem = session, motion, problem
        self.duration_s = duration_s
        self.started_at = session.world.time_s
        if self.bonds != problem.initial:
            raise SpatialPlanningError("staged WorldState differs from the initial support mode")
        self.actions = plan_support_changes(
            problem.modules, problem.initial, problem.goal, problem.anchors
        )
        if len(self.actions) != 2 or tuple(a.operation for a in self.actions) != (
            "dock",
            "release",
        ):
            raise SpatialPlanningError(
                "this handoff executor requires one capture followed by one release"
            )
        self.capture, self.release = self.actions[0].bond, self.actions[1].bond
        self.path = plan_joint_path(
            problem.start, problem.rendezvous, problem.upper, motion.feasible
        )
        self.return_path = plan_joint_path(
            problem.rendezvous,
            problem.final,
            problem.upper,
            lambda p: motion.feasible(p, transferred=True),
            subdivisions=15,
        )
        self.phase = "moving"
        self.detail = "Executing sampled joint-space plan with bounded effort"
        self.waypoint = self.return_waypoint = 1
        self.phase_started = session.world.time_s
        self.settled_since: float | None = None
        self.command = motion.targets(self.path.points[0])
        motion.command_positions(self.command)
        self.peak_effort_nm = self.peak_penetration_m = 0.0
        self.joint_error_rad = 0.0
        self._decisions: list[PlannerDecision] = []
        self._stages: list[tuple[float, str, str]] = []
        self._traces = self._build_traces()
        self._decision(
            "Support plan",
            f"Capture {self.capture.a} ↔ {self.capture.b} before releasing the helper",
        )
        self._decision(
            "Approach search",
            f"A*: {len(self.path.points) - 1} edges; "
            f"{self.path.expanded} states expanded; {self.path.rejected} edges rejected",
        )
        self._decision(
            "Withdrawal search",
            f"A*: {len(self.return_path.points) - 1} edges; "
            f"{self.return_path.expanded} states expanded; "
            f"{self.return_path.rejected} edges rejected; unfold receiver and withdraw helper",
        )
        self.history: list[dict[str, object]] = []
        self._record()

    @property
    def bonds(self) -> frozenset[Bond]:
        return frozenset(
            Bond(str(c.connector_a), str(c.connector_b))
            for c in self.session.world.connections.values()
        )

    @property
    def terminal(self) -> bool:
        return self.phase in {"complete", "failed", "stopped", "timeout"}

    @property
    def status(self) -> ReconfigurationStatus:
        phase = {
            "moving": ReconfigurationPhase.APPROACHING,
            "transfer": ReconfigurationPhase.HOLDING_CONNECTED,
            "retreat": ReconfigurationPhase.APPROACHING,
            "verify": ReconfigurationPhase.HOLDING_CONNECTED,
            "complete": ReconfigurationPhase.COMPLETE,
            "failed": ReconfigurationPhase.FAILED,
            "stopped": ReconfigurationPhase.STOPPED,
            "timeout": ReconfigurationPhase.TIMED_OUT,
        }[self.phase]
        detail = (
            f"{self.phase}: {self.detail} | target error {self.target_error_m() * 1000:.1f} mm"
            f" | peak effort {self.peak_effort_nm:.2f} Nm | two fixed supports"
        )
        return ReconfigurationStatus(
            phase=phase,
            time_s=self.session.world.time_s,
            plan_id="smores_spatial_handoff",
            plan_name="SMORES supported 3D handoff",
            action_index=0 if self.phase == "moving" else 1,
            action_count=2,
            detail=detail,
        )

    def _record(self) -> None:
        self._stages.append((self.session.world.time_s, self.phase, self.detail))
        self._decision(self.phase.capitalize(), self.detail)
        self.history.append(
            {
                "time_s": self.session.world.time_s,
                "phase": self.phase,
                "detail": self.detail,
                "connections": len(self.bonds),
            }
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

    def _build_traces(self) -> tuple[PlannedTrace, ...]:
        traces: list[PlannedTrace] = []
        for points, transferred in (
            (self.path.points, False),
            (self.return_path.points, True),
        ):
            poses = [
                self.motion.configuration_positions(p, transferred=transferred) for p in points
            ]
            for module in sorted(self.problem.modules - self.problem.anchors):
                traces.append(
                    PlannedTrace(
                        module_id=module,
                        stage="retreat" if transferred else "moving",
                        positions_m=tuple(p[module] for p in poses),
                    )
                )
        return tuple(traces)

    def planning_snapshot(self) -> SpatialPlanningSnapshot:
        """Copy planner intent without exposing mutable runtime or backend objects."""
        world = self.session.world
        actions: list[SpatialAction] = []
        for stage, label, detail, count, progress in (
            (
                "moving",
                "Lift & capture",
                "Reach the elevated docking pair; commit measured capture",
                len(self.path.points) - 1,
                self.waypoint,
            ),
            (
                "transfer",
                "Verify & release",
                "Hold both supports for 0.75 s, then release helper",
                1,
                0,
            ),
            (
                "retreat",
                "Unfold & withdraw",
                "Raise receiving chain to its target; lower and park helper",
                len(self.return_path.points) - 1,
                self.return_waypoint,
            ),
            (
                "verify",
                "Final hold",
                "Settle for 1 s; verify exact bonds and <4 mm position error",
                1,
                0,
            ),
        ):
            intervals: tuple[ActionInterval, ...] = ()
            state = "pending"
            for i, (started, observed, reason) in enumerate(self._stages):
                if observed != stage:
                    continue
                following = self._stages[i + 1] if i + 1 < len(self._stages) else None
                state = (
                    stage
                    if following is None
                    else (
                        following[1]
                        if following[1] in {"failed", "stopped", "timeout"}
                        else "complete"
                    )
                )
                detail = reason if following is None else detail
                intervals = (
                    ActionInterval(
                        phase=stage,
                        start_s=started,
                        end_s=None if following is None else following[0],
                    ),
                )
            actions.append(
                SpatialAction(
                    id=stage,
                    label=label,
                    phase=state,
                    detail=detail,
                    waypoint=count
                    if state == "complete"
                    else 0
                    if state == "pending"
                    else progress,
                    waypoint_count=count,
                    intervals=intervals,
                )
            )
        return SpatialPlanningSnapshot(
            time_s=world.time_s,
            sample_sequence=world.revision.sample_sequence,
            topology_revision=world.revision.topology_revision,
            phase=self.phase,
            detail=self.detail,
            targets=tuple(
                TargetModule(id=m, position_m=p, anchored=m in self.problem.anchors)
                for m, p in sorted(self.motion.target_positions.items())
            ),
            target_bonds=tuple(TargetBond(a=b.a, b=b.b) for b in sorted(self.problem.goal)),
            traces=self._traces,
            actions=tuple(actions),
            decisions=tuple(self._decisions),
            approach_expanded=self.path.expanded,
            approach_rejected=self.path.rejected,
            withdrawal_expanded=self.return_path.expanded,
            withdrawal_rejected=self.return_path.rejected,
            target_error_m=self.target_error_m(),
            joint_error_rad=self.joint_error_rad,
            peak_effort_nm=self.peak_effort_nm,
            peak_penetration_m=self.peak_penetration_m,
        )

    def _phase(self, phase: str, detail: str) -> None:
        self.phase, self.detail = phase, detail
        self.phase_started = self.session.world.time_s
        self.settled_since = None
        self._record()

    def stop(self) -> None:
        if not self.terminal:
            self._phase("stopped", "Stopped by user; target not verified")

    def step(self) -> tuple[Event, ...]:
        cursor = len(self.session.world.event_log)
        if not self.terminal:
            self._advance()
        return self.session.world.event_log.since(cursor)

    def _advance(self) -> None:
        if self.session.world.time_s - self.started_at >= self.duration_s - 1e-10:
            self._phase("timeout", "Time limit reached; target not achieved")
            return
        target = self.motion.targets(self.path.points[self.waypoint])
        if self.phase in {"retreat", "verify"}:
            point = self.return_path.points[self.return_waypoint]
            target = self.motion.targets(point)
        delta = 0.35 * self.dt_s
        self.command = tuple(
            old + max(-delta, min(delta, new - old))
            for old, new in zip(self.command, target, strict=True)
        )
        self.motion.command_positions(self.command)
        self.session.step(self.dt_s)
        sample = self.motion.observe_safety()
        if abs(sample.time_s - self.session.world.time_s) > 1e-9:
            self._phase("failed", "Mechanical observation does not match WorldState time")
            return
        self.peak_penetration_m = max(self.peak_penetration_m, sample.penetration_m)
        if (
            not sample.finite
            or not math.isfinite(sample.penetration_m)
            or sample.penetration_m > 0.004
        ):
            self._phase("failed", "Invalid motion or excessive collision proxy penetration")
            return
        error = speed = 0.0
        for (module_id, joint_id), desired in zip(self.motion.motor_keys, target, strict=True):
            measured = self.session.world.modules[module_id].joint_states.get(joint_id)
            if measured is None or not all(
                math.isfinite(x) for x in (measured.position, measured.velocity, measured.effort)
            ):
                self._phase(
                    "failed", f"Missing or invalid measured joint state: {module_id}/{joint_id}"
                )
                return
            error = max(error, abs(desired - measured.position))
            speed = max(speed, abs(measured.velocity))
            self.peak_effort_nm = max(self.peak_effort_nm, abs(measured.effort))
        live = self.bonds
        self.joint_error_rad = error
        if not supported(self.problem.modules, live, self.problem.anchors):
            self._phase("failed", "A module lost its path to a fixture")
            return
        if error < 0.018 and speed < 0.04:
            if self.settled_since is None:
                self.settled_since = self.session.world.time_s
        else:
            self.settled_since = None
        if self.settled_since is None or self.session.world.time_s - self.settled_since < 0.15:
            if self.session.world.time_s - self.phase_started > 15:
                self._phase(
                    "failed",
                    f"Joint tracking stalled: error={error:.3f} rad, speed={speed:.3f} rad/s",
                )
            return
        if self.phase == "moving":
            if self.waypoint + 1 < len(self.path.points):
                self._decision("Approach waypoint", f"Waypoint {self.waypoint} settled")
                self.waypoint += 1
                self.phase_started = self.session.world.time_s
                self.settled_since = None
            else:
                self.session.request_dock(
                    ConnectorInstanceId(self.capture.a), ConnectorInstanceId(self.capture.b)
                )
                self.session.process_docking()
                if self.capture not in self.bonds:
                    self._phase(
                        "failed", "Measured elevated docking refused; helper remains attached"
                    )
                else:
                    self._phase(
                        "transfer", "Elevated connection committed; verifying before release"
                    )
        elif self.phase == "transfer":
            if self.capture not in live:
                self._phase("failed", "Receiver capture lost; helper remains attached")
            elif self.session.world.time_s - self.phase_started >= 0.75:
                self.session.request_undock(
                    connection_id(
                        ConnectorInstanceId(self.release.a), ConnectorInstanceId(self.release.b)
                    )
                )
                self.session.process_docking()
                if self.release in self.bonds:
                    self._phase("failed", "Helper release refused; withdrawal cancelled")
                else:
                    self._phase("retreat", "Receiver unfolds toward target; helper lowers away")
        elif self.phase == "retreat":
            if self.return_waypoint + 1 < len(self.return_path.points):
                self._decision("Withdrawal waypoint", f"Waypoint {self.return_waypoint} settled")
                self.return_waypoint += 1
                self.phase_started = self.session.world.time_s
                self.settled_since = None
            else:
                self._phase("verify", "Holding final spatial assembly under gravity")
        elif self.phase == "verify" and self.session.world.time_s - self.phase_started >= 1:
            if live == self.problem.goal and self.target_error_m() < 0.004:
                self._phase("complete", "Target reached; elevated handoff and final hold verified")
            else:
                self._phase("failed", "Final geometry or topology differs from target")

    def target_error_m(self) -> float:
        return max(
            vec_norm(vec_sub(self.session.world.modules[ModuleInstanceId(m)].pose.translation, p))
            for m, p in self.motion.target_positions.items()
            if m != self.problem.parked_helper
        )

    def report(self) -> dict[str, object]:
        """Describe search and measured execution using only canonical state."""
        return {
            "phase": self.phase,
            "detail": self.detail,
            "time_s": self.session.world.time_s,
            "fixtures": sorted(self.problem.anchors),
            "topology_plan": [asdict(a) for a in self.actions],
            "joint_plan": asdict(self.path),
            "return_plan": asdict(self.return_path),
            "history": self.history,
            "peak_effort_nm": self.peak_effort_nm,
            "peak_penetration_m": self.peak_penetration_m,
            "target_error_m": self.target_error_m(),
            "target_positions": dict(self.motion.target_positions),
            "connections": sorted(str(c.id) for c in self.session.world.connections.values()),
            "events": [{"time_s": e.time_s, "kind": e.kind} for e in self.session.world.event_log],
        }
