"""Single-owner, feedback-driven execution of planar assembly plans.

The planner consumes copied poses. Only this coordinator submits efforts, requests
connections, and advances the session. Initial staging is the only root-pose write.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, replace
from typing import Literal, cast

from modsim.connectors.acceptance import evaluate_acceptance
from modsim.core.events import Event
from modsim.core.ids import ConnectionId, ConnectorInstanceId, ModuleInstanceId
from modsim.core.transforms import quat_rotate
from modsim.planning.assembly import plan_assembly
from modsim.planning.models import (
    ActionInterval,
    ActionObservation,
    AssemblyAction,
    AssemblyGoal,
    AssemblyPlan,
    Face,
    GoalEdge,
    PlannerDecision,
    PlanningSnapshot,
    Pose2,
    TimedPose,
)
from modsim.planning.routing import (
    Footprint,
    Obstacle,
    Reservation,
    RouteConfig,
    RouteUnavailable,
    angle_delta,
    plan_route,
    swept_clear,
)
from modsim.runtime.physical_reconfiguration import (
    DifferentialDriveReconfigurationConfig,
    PhysicalDockController,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationPhase,
    ReconfigurationPlan,
    ReconfigurationStatus,
    initialize_reconfiguration_plan,
)
from modsim.runtime.session import RuntimeSession

ActionPhase = Literal[
    "pending",
    "waiting",
    "navigating",
    "aligning",
    "approaching",
    "holding",
    "retreating",
    "complete",
    "failed",
]


def module_pose(session: RuntimeSession, module: str) -> Pose2:
    pose = session.world.modules[ModuleInstanceId(module)].pose
    forward = quat_rotate(pose.rotation, (1.0, 0.0, 0.0))
    return Pose2(
        x=pose.translation[0], y=pose.translation[1], yaw=math.atan2(forward[1], forward[0])
    )


def connector_pair(edge: GoalEdge) -> ConnectorPairRef:
    return ConnectorPairRef(
        ConnectorInstanceId(f"{edge.a}/{edge.face_a}"),
        ConnectorInstanceId(f"{edge.b}/{edge.face_b}"),
    )


def measured_faces(session: RuntimeSession) -> dict[Face, Pose2]:
    module = min(session.world.modules)
    root = module_pose(session, module)
    result: dict[Face, Pose2] = {}
    for face in ("pan", "bottom", "left", "right"):
        connector = session.world.connector(ConnectorInstanceId(f"{module}/{face}"))
        axis = connector.world_docking_axis
        xyz = connector.world_pose.translation
        result[face] = Pose2(x=xyz[0], y=xyz[1], yaw=math.atan2(axis[1], axis[0])).relative_to(root)
    return result


@dataclass
class _Execution:
    action: AssemblyAction
    controller: PhysicalDockController
    phase: ActionPhase = "pending"
    reason: str = "Waiting for dependencies"
    members: tuple[ModuleInstanceId, ...] = ()
    relatives: tuple[Pose2, ...] = ()
    path: tuple[TimedPose, ...] = ()
    index: int = 1
    started_s: float | None = None
    finished_s: float | None = None
    held_since: float = 0.0
    last_progress_s: float = 0.0
    best_error: float = math.inf
    attempts: int = 0
    target: Pose2 | None = None
    error_m: float | None = None
    angle_rad: float | None = None
    velocity_m_s: float | None = None
    intervals: list[ActionInterval] = field(default_factory=list[ActionInterval])
    clearing: bool = False
    recovery_until_s: float = 0.0
    next_route_s: float = 0.0


@dataclass
class OnlineAssemblyScenario:
    session: RuntimeSession
    plan: AssemblyPlan
    config: DifferentialDriveReconfigurationConfig
    executions: list[_Execution]
    footprint: Footprint = field(default_factory=Footprint)
    route_config: RouteConfig = field(default_factory=RouteConfig)
    planning_period_s: float = 0.5
    _released: bool = False
    _next_plan_s: float = 0.0
    _revision: int = 1
    _replans: int = 0
    _planning_ms: float = 0.0
    _distance_m: float = 0.0
    _decisions: list[PlannerDecision] = field(default_factory=list[PlannerDecision])
    _failed: str | None = None
    _complete: bool = False
    _last_poses: dict[str, Pose2] = field(default_factory=dict[str, Pose2])
    _preserved_connections: frozenset[ConnectionId] = frozenset()

    @classmethod
    def create(
        cls,
        session: RuntimeSession,
        goal: AssemblyGoal,
        config: DifferentialDriveReconfigurationConfig,
        initial: ReconfigurationPlan | None = None,
    ) -> OnlineAssemblyScenario:
        if session.world.time_s != 0:
            raise ValueError("online scenarios must be initialized at time zero")
        capabilities = session.adapter.capabilities()
        if not capabilities.supports_joint_commands:
            raise ValueError("online physical planning requires joint feedback and effort commands")
        if initial is not None:
            initialize_reconfiguration_plan(session, initial, preserve_orientation=True)
        poses = {str(m): module_pose(session, m) for m in session.world.modules}
        existing = tuple(
            GoalEdge(
                a=str(connection.connector_a).split("/")[0],
                face_a=cast(Face, str(connection.connector_a).split("/")[1]),
                b=str(connection.connector_b).split("/")[0],
                face_b=cast(Face, str(connection.connector_b).split("/")[1]),
                orientation_rad=connection.orientation_rad,
            )
            for connection in session.world.connections.values()
        )
        plan = plan_assembly(
            goal,
            poses,
            faces=measured_faces(session),
            existing=existing,
            fixed_mapping=None if initial is None else {n: n for n in goal.nodes},
        )
        executions = [
            _Execution(
                action,
                PhysicalDockController.for_action(session, connector_pair(action.edge), config),
            )
            for action in plan.actions
        ]
        if not executions:
            raise ValueError("online demo requires at least one docking action")
        offset = measured_faces(session)["left"].x
        result = cls(
            session,
            plan,
            config,
            executions,
            footprint=Footprint(center_x=offset),
            _last_poses=poses,
            _preserved_connections=frozenset(session.world.connections)
            - {connector_pair(edge).connection_id for edge in plan.releases},
        )
        for execution in executions:
            execution.controller.axle_offset_m = offset
            execution.controller.desired_orientation_rad = execution.action.orientation_rad
        result._record(
            "assignment",
            f"Root {plan.root_module}; distance cost {plan.assignment_cost_m:.3f} m; "
            f"{len(plan.actions)} actions",
        )
        result._submit_commands()
        return result

    @property
    def status(self) -> ReconfigurationStatus:
        completed = sum(e.phase == "complete" for e in self.executions)
        active = sum(
            e.phase in {"navigating", "aligning", "approaching", "holding", "retreating"}
            for e in self.executions
        )
        phase = (
            ReconfigurationPhase.FAILED
            if self._failed
            else ReconfigurationPhase.COMPLETE
            if self._complete
            else ReconfigurationPhase.APPROACHING
            if self._released
            else ReconfigurationPhase.HOLDING_INITIAL
        )
        return ReconfigurationStatus(
            phase=phase,
            time_s=self.session.world.time_s,
            plan_id=self.plan.goal.id,
            plan_name="SMORES-EP online parallel planning",
            action_index=None,
            action_count=len(self.executions),
            detail=self._failed
            or (
                f"{completed}/{len(self.executions)} docks committed · "
                f"{active} active · {self._replans} replans"
            ),
        )

    @property
    def planning_snapshot(self) -> PlanningSnapshot:
        revision = self.session.world.revision
        return PlanningSnapshot(
            time_s=self.session.world.time_s,
            sample_sequence=revision.sample_sequence,
            topology_revision=revision.topology_revision,
            plan_revision=self._revision,
            plan=self.plan,
            actions=tuple(
                ActionObservation(
                    action=e.action,
                    phase=e.phase,
                    reason=e.reason,
                    members=tuple(str(m) for m in e.members),
                    path=e.path,
                    waypoint_index=e.index,
                    started_s=e.started_s,
                    finished_s=e.finished_s,
                    position_error_m=e.error_m,
                    orientation_error_rad=e.angle_rad,
                    relative_velocity_m_s=e.velocity_m_s,
                    tracking_error_m=None
                    if not e.path or e.index >= len(e.path)
                    else module_pose(self.session, e.action.moving).distance(e.path[e.index].pose),
                    intervals=tuple(e.intervals),
                )
                for e in self.executions
            ),
            decisions=tuple(self._decisions),
            replans=self._replans,
            planning_ms=self._planning_ms,
            path_length_m=self._distance_m,
            footprint_half_m=(self.footprint.half_x, self.footprint.half_y),
            footprint_center_x_m=self.footprint.center_x,
        )

    def step(self) -> tuple[Event, ...]:
        now = self.session.world.time_s
        events: list[Event] = []
        if not self._failed and self._preserved_connections - self.session.world.connections.keys():
            self._fail("An already-correct connection being preserved was lost")
        if not self._failed and not self._complete:
            if not self._released and now >= self.config.settle_s:
                for edge in self.plan.releases:
                    self.session.request_undock(connector_pair(edge).connection_id)
                events.extend(self.session.process_docking())
                if any(
                    connector_pair(e).connection_id in self.session.world.connections
                    for e in self.plan.releases
                ):
                    self._fail("A required release was refused")
                else:
                    self._released = True
                    self._next_plan_s = now + self.config.release_delay_s
                    self._record(
                        "release",
                        f"Released {len(self.plan.releases)} connections; preserving correct bonds",
                    )
            if self._released and now >= self._next_plan_s:
                started = time.perf_counter()
                self._schedule()
                self._planning_ms = max(self._planning_ms, (time.perf_counter() - started) * 1000)
                self._next_plan_s = now + self.planning_period_s
            events.extend(self._advance())
        self._submit_commands()
        for execution in self.executions:
            if not execution.intervals or execution.intervals[-1].phase != execution.phase:
                if execution.intervals:
                    execution.intervals[-1] = execution.intervals[-1].model_copy(
                        update={"end_s": now}
                    )
                execution.intervals.append(ActionInterval(phase=execution.phase, start_s=now))
        events.extend(self.session.step(self.config.dt_s))
        for module in self._last_poses:
            pose = module_pose(self.session, module)
            self._distance_m += self._last_poses[module].distance(pose)
            self._last_poses[module] = pose
        return tuple(events)

    def _record(self, kind: str, detail: str) -> None:
        self._decisions.append(
            PlannerDecision(
                sequence=len(self._decisions),
                time_s=self.session.world.time_s,
                kind=kind,
                detail=detail,
            )
        )

    def _fail(self, reason: str) -> None:
        self._failed = reason
        self._record("failed", reason)
        for execution in self.executions:
            if execution.phase != "complete":
                execution.phase, execution.reason = "failed", reason

    def _obstacles(self, execution: _Execution, *, approach: bool = False) -> tuple[Obstacle, ...]:
        ignored = set(execution.members)
        if approach:
            ignored.add(ModuleInstanceId(execution.action.parent))
        return tuple(
            Obstacle(str(m), module_pose(self.session, m))
            for m in self.session.world.modules
            if m not in ignored
        )

    def _reservation(self, execution: _Execution) -> Reservation:
        now = self.session.world.time_s
        pose = module_pose(self.session, execution.action.moving)
        points = [TimedPose(pose=pose, time_s=now)]
        if execution.phase in {"navigating", "aligning"}:
            for point in execution.path[execution.index :]:
                previous = points[-1]
                duration = max(
                    previous.pose.distance(point.pose) / self.route_config.speed_m_s,
                    abs(angle_delta(point.pose.yaw, previous.pose.yaw))
                    / self.route_config.yaw_rate_rad_s,
                    0.2,
                )
                points.append(
                    TimedPose(
                        pose=point.pose,
                        time_s=previous.time_s + duration,
                        direction=point.direction,
                    )
                )
        target = execution.target
        if target is not None:
            points.append(
                TimedPose(
                    pose=target,
                    time_s=points[-1].time_s
                    + points[-1].pose.distance(target) / self.config.approach_speed_m_s
                    + 2.0,
                )
            )
        return Reservation(execution.action.id, tuple(points), execution.relatives)

    def _schedule(self) -> None:
        now = self.session.world.time_s
        done = {e.action.id for e in self.executions if e.phase == "complete"}
        for execution in self.executions:
            if execution.phase == "approaching" and execution.reason.startswith(
                "Measured clearance"
            ):
                x, y, yaw = execution.controller.target_pose()
                self._clear_corridor(
                    execution,
                    module_pose(self.session, execution.action.moving),
                    Pose2(x=x, y=y, yaw=yaw),
                )
        for execution in self.executions:
            if execution.phase not in {"pending", "waiting"}:
                continue
            if now < execution.next_route_s:
                continue
            if not set(execution.action.dependencies) <= done:
                execution.reason = "Waiting for earlier assembly depth / root-face group"
                continue
            assembly = self.session.world.assemblies.assembly_of(
                ModuleInstanceId(execution.action.moving)
            )
            execution.members = tuple(sorted(self.session.world.assemblies.members(assembly)))
            active = [
                e
                for e in self.executions
                if e.phase in {"navigating", "aligning", "approaching", "holding", "retreating"}
            ]
            if any(
                set(execution.members) & (set(e.members) | {ModuleInstanceId(e.action.parent)})
                or ModuleInstanceId(execution.action.parent) in e.members
                for e in active
            ):
                execution.reason = "Waiting for moving subassembly ownership"
                continue
            pose = module_pose(self.session, execution.action.moving)
            execution.relatives = tuple(
                module_pose(self.session, m).relative_to(pose) for m in execution.members
            )
            execution.controller.axle_offset_m = self.footprint.center_x + sum(
                p.x for p in execution.relatives
            ) / len(execution.relatives)
            x, y, yaw = execution.controller.target_pose()
            target = Pose2(x=x, y=y, yaw=yaw)
            direction = 1 if execution.action.moving_face == "pan" else -1
            staging = target.compose(Pose2(x=-direction * 0.12, y=0.0))
            reservations = tuple(self._reservation(e) for e in active)
            moving_ids = {str(m) for e in active for m in e.members}
            obstacles = tuple(o for o in self._obstacles(execution) if o.id not in moving_ids)
            try:
                # Reserve the final approach as well as the navigation route.
                if not swept_clear(
                    staging,
                    target,
                    members=execution.relatives,
                    footprint=replace(self.footprint, margin=0.0),
                    obstacles=self._obstacles(execution, approach=True),
                    departure=pose,
                ):
                    raise RouteUnavailable("final approach corridor is occupied")
                path = plan_route(
                    pose,
                    staging,
                    members=execution.relatives,
                    footprint=self.footprint,
                    obstacles=obstacles,
                    reservations=reservations,
                    start_s=now,
                    config=self.route_config,
                    pivot_offset_m=execution.controller.axle_offset_m,
                )
            except RouteUnavailable as error:
                reason = str(error)
                if execution.reason != reason:
                    self._record("waiting", f"{execution.action.id}: {reason}")
                execution.phase, execution.reason = "waiting", reason
                execution.next_route_s = now + 2.0
                self._clear_corridor(execution, staging, target)
                if execution.started_s is None:
                    execution.started_s = now
                if now - execution.started_s > self.config.action_timeout_s:
                    self._fail(f"{execution.action.id}: route remained blocked: {reason}")
                continue
            execution.path, execution.index, execution.target = path, 1, target
            execution.clearing = False
            execution.phase, execution.reason = "navigating", "Following generated route"
            execution.started_s = execution.started_s if execution.started_s is not None else now
            execution.last_progress_s, execution.best_error = now, math.inf
            self._revision += 1
            self._record(
                "route",
                f"{execution.action.id}: {len(path)} waypoints "
                f"for {len(execution.members)} module(s)",
            )

    def _clear_corridor(self, requester: _Execution, staging: Pose2, target: Pose2) -> None:
        """Move an unattached future participant aside when it blocks an approach.

        Clearance is an explicit, collision-checked motion, never a hidden pose write.
        Committed parent assemblies and active participants are not displaced.
        """
        for obstacle in self._obstacles(requester, approach=True):
            if swept_clear(
                staging,
                target,
                members=requester.relatives,
                footprint=self.footprint,
                obstacles=(obstacle,),
            ):
                continue
            blocker = next(
                (
                    e
                    for e in self.executions
                    if e.action.moving == obstacle.id
                    and e.phase in {"pending", "waiting"}
                    and e is not requester
                ),
                None,
            )
            if blocker is None:
                continue
            if any(
                e.action.parent == obstacle.id
                and e.phase in {"navigating", "aligning", "approaching", "holding", "retreating"}
                for e in self.executions
            ):
                continue
            assembly = self.session.world.assemblies.assembly_of(ModuleInstanceId(obstacle.id))
            members = tuple(sorted(self.session.world.assemblies.members(assembly)))
            if len(members) != 1:
                continue
            root = module_pose(self.session, self.plan.root_module)
            pose = module_pose(self.session, obstacle.id)
            headings = sorted(
                (0.0, math.pi / 2, math.pi, -math.pi / 2),
                key=lambda h: -((pose.x - root.x) * math.cos(h) + (pose.y - root.y) * math.sin(h)),
            )
            blocker.members, blocker.relatives = members, (Pose2(x=0.0, y=0.0),)
            blocker.controller.axle_offset_m = self.footprint.center_x
            for heading in headings:
                parking = Pose2(
                    x=pose.x + 0.35 * math.cos(heading),
                    y=pose.y + 0.35 * math.sin(heading),
                    yaw=pose.yaw,
                )
                try:
                    path = plan_route(
                        pose,
                        parking,
                        footprint=self.footprint,
                        obstacles=self._obstacles(blocker),
                        start_s=self.session.world.time_s,
                        config=replace(self.route_config, max_expansions=1200),
                    )
                except RouteUnavailable:
                    continue
                blocker.path, blocker.index, blocker.target = path, 1, None
                blocker.phase, blocker.reason, blocker.clearing = (
                    "navigating",
                    "Moving aside for approach clearance",
                    True,
                )
                blocker.started_s = blocker.last_progress_s = self.session.world.time_s
                blocker.best_error = math.inf
                self._revision += 1
                self._record("clearance", f"{obstacle.id} moves aside for {requester.action.id}")
                return

    def _retry(self, execution: _Execution, reason: str) -> None:
        execution.attempts += 1
        self._replans += 1
        self._revision += 1
        self._record("replan", f"{execution.action.id}: {reason}")
        if execution.attempts > 8:
            self._fail(f"{execution.action.id}: exhausted bounded recovery attempts ({reason})")
        elif execution.phase == "approaching":
            now = self.session.world.time_s
            execution.phase, execution.reason = (
                "retreating",
                "Backing away to retry connector alignment",
            )
            execution.recovery_until_s = now + 5.0
            x, y, yaw = execution.controller.target_pose()
            direction = 1 if execution.action.moving_face == "pan" else -1
            target = Pose2(x=x, y=y, yaw=yaw).compose(Pose2(x=-direction * 0.20, y=0.0))
            execution.path = (
                TimedPose(pose=module_pose(self.session, execution.action.moving), time_s=now),
                TimedPose(
                    pose=target,
                    time_s=execution.recovery_until_s,
                    direction=-1 if direction > 0 else 1,
                ),
            )
            execution.index = 1
        else:
            execution.phase, execution.reason, execution.path = "waiting", reason, ()

    def _advance(self) -> tuple[Event, ...]:
        now = self.session.world.time_s
        events: list[Event] = []
        for execution in self.executions:
            pair = connector_pair(execution.action.edge)
            if execution.phase in {"complete", "holding"}:
                if pair.connection_id not in self.session.world.connections:
                    self._fail(f"{execution.action.id}: a committed connection was lost")
                    break
                if (
                    execution.phase == "holding"
                    and now - execution.held_since >= self.config.connected_hold_s
                ):
                    execution.phase, execution.reason, execution.finished_s = (
                        "complete",
                        "Dock confirmed and settled",
                        now,
                    )
                    self._record("complete", execution.action.id)
                continue
            if execution.phase == "retreating":
                if now >= execution.recovery_until_s:
                    execution.phase, execution.reason, execution.path = (
                        "waiting",
                        "Backoff complete; regenerating route",
                        (),
                    )
                continue
            if execution.phase not in {"navigating", "aligning", "approaching"}:
                continue
            if (
                execution.started_s is not None
                and now - execution.started_s > self.config.action_timeout_s
            ):
                self._fail(f"{execution.action.id}: execution timed out")
                break
            pose = module_pose(self.session, execution.action.moving)
            x, y, yaw = execution.controller.target_pose()
            target = Pose2(x=x, y=y, yaw=yaw)
            if (
                execution.phase != "approaching"
                and execution.target is not None
                and (
                    target.distance(execution.target) > 0.025
                    or abs(angle_delta(target.yaw, execution.target.yaw)) > 0.25
                )
            ):
                self._retry(execution, "receiving assembly moved")
                continue
            if execution.phase in {"navigating", "aligning"}:
                waypoint = execution.path[execution.index]
                axle = Pose2(x=execution.controller.axle_offset_m, y=0.0)
                error = pose.compose(axle).distance(waypoint.pose.compose(axle))
                heading_error = abs(angle_delta(pose.yaw, waypoint.pose.yaw))
                progress_error = error + (heading_error * 0.05 if waypoint.direction == 0 else 0.0)
                if progress_error < execution.best_error - 0.002:
                    execution.best_error, execution.last_progress_s = progress_error, now
                if (
                    error < (0.06 if waypoint.direction == 0 else 0.03)
                    and (waypoint.direction != 0 or heading_error < 0.20)
                    and (waypoint.direction != 0 or now >= waypoint.time_s)
                ):
                    execution.index += 1
                    execution.best_error, execution.last_progress_s = math.inf, now
                    if execution.index == len(execution.path) and execution.clearing:
                        execution.phase, execution.reason = (
                            "pending",
                            "Clearance complete; waiting for dependencies",
                        )
                        execution.clearing, execution.started_s, execution.path = False, None, ()
                        self._record("clearance_complete", execution.action.id)
                    elif execution.index == len(execution.path):
                        execution.phase, execution.reason = (
                            "approaching",
                            "Aligning measured connector faces for capture",
                        )
                    elif execution.index >= len(execution.path) - 2:
                        execution.phase = "aligning"
                elif now - execution.last_progress_s > 18.0:
                    self._retry(execution, "route tracking stalled")
                continue
            world = self.session.world
            acceptance = evaluate_acceptance(
                world.connector(pair.fixed_connector),
                world.connector(pair.moving_connector),
                world.connector_type(pair.fixed_connector),
                world.connector_type(pair.moving_connector),
            )
            execution.error_m = acceptance.criterion("position").measured
            execution.angle_rad = acceptance.criterion("axis_alignment").measured
            execution.velocity_m_s = acceptance.criterion("relative_velocity").measured
            desired_roll = execution.action.orientation_rad
            roll_matches = (
                desired_roll is None
                or abs(angle_delta(acceptance.orientation_rad, desired_roll)) < 1e-5
            )
            if (
                acceptance.satisfied
                and execution.error_m <= self.config.latch_distance_m
                and roll_matches
            ):
                self.session.request_dock(pair.fixed_connector, pair.moving_connector)
                events.extend(self.session.process_docking())
                if pair.connection_id in self.session.world.connections:
                    execution.phase, execution.reason, execution.held_since = (
                        "holding",
                        "Dock committed; settling",
                        now,
                    )
                    self._record("dock", execution.action.id)
                else:
                    self._retry(execution, "backend or docking guard refused capture")
            error = pose.distance(target)
            if error < execution.best_error - 0.001:
                execution.best_error, execution.last_progress_s = error, now
            elif now - execution.last_progress_s > 25.0 and execution.phase == "approaching":
                self._retry(execution, "connector alignment stalled")
        if all(e.phase == "complete" for e in self.executions):
            mapping = {a.goal_node: a.module_id for a in self.plan.assignments}
            expected = {
                connector_pair(
                    GoalEdge(a=mapping[e.a], face_a=e.face_a, b=mapping[e.b], face_b=e.face_b)
                ).connection_id
                for e in self.plan.goal.edges
            }
            if set(self.session.world.connections) == expected:
                self._complete = True
            else:
                self._fail("Final committed topology differs from the requested goal")
        return tuple(events)

    def _submit_commands(self) -> None:
        # Brakes/holds for every module are overlaid with disjoint action commands.
        base = self.executions[0].controller
        commands = {c.joint: c for c in base.control_commands()}
        for execution in self.executions:
            if (
                self._failed
                or self._complete
                or execution.phase not in {"navigating", "aligning", "approaching", "retreating"}
            ):
                continue
            approach = execution.phase == "approaching"
            waypoint = None if approach else execution.path[execution.index]
            direction = 1 if waypoint is None or waypoint.direction == 0 else waypoint.direction
            if approach:
                x, y, yaw = execution.controller.target_pose()
                target = Pose2(x=x, y=y, yaw=yaw)
            else:
                assert waypoint is not None
                target = waypoint.pose
            pose = module_pose(self.session, execution.action.moving)
            # A short swept guard stops motion into measured occupancy, even when
            # a reservation was invalidated by contact or another module arriving late.
            lookahead = min(1.0, 0.025 / max(0.025, pose.distance(target)))
            from modsim.planning.routing import interpolate

            safe = swept_clear(
                pose,
                interpolate(pose, target, lookahead),
                members=execution.relatives,
                footprint=(replace(self.footprint, margin=0.0) if approach else self.footprint),
                obstacles=self._obstacles(
                    execution, approach=approach or execution.phase == "retreating"
                ),
                allow_escape=execution.phase == "retreating",
                departure=pose,
                pivot_offset_m=execution.controller.axle_offset_m,
            )
            execution.reason = (
                "Measured clearance blocked; holding"
                if not safe
                else "Moving aside for approach clearance"
                if execution.clearing
                else "Measured connector approach"
                if approach
                else "Following generated route"
            )
            for command in execution.controller.commands_to(
                execution.members,
                waypoint=(target.x, target.y, target.yaw),
                direction=direction,
                approach=approach,
                stopped=not safe,
                turn_only=waypoint is not None and waypoint.direction == 0,
            ):
                commands[command.joint] = command
            if approach:
                for command in execution.controller.parent_roll_commands():
                    commands[command.joint] = command
        self.session.set_joint_commands(commands.values())
