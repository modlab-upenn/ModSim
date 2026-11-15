"""Immutable planning inputs and observations; independent of physics and GUI libraries."""

from __future__ import annotations

import math
from typing import Literal, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator


class PlanningValue(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", allow_inf_nan=False, strict=True)


class Pose2(PlanningValue):
    x: float
    y: float
    yaw: float = 0.0

    def compose(self, other: Pose2) -> Pose2:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2(
            x=self.x + c * other.x - s * other.y,
            y=self.y + s * other.x + c * other.y,
            yaw=math.atan2(math.sin(self.yaw + other.yaw), math.cos(self.yaw + other.yaw)),
        )

    def relative_to(self, parent: Pose2) -> Pose2:
        c, s = math.cos(parent.yaw), math.sin(parent.yaw)
        dx, dy = self.x - parent.x, self.y - parent.y
        return Pose2(
            x=c * dx + s * dy,
            y=-s * dx + c * dy,
            yaw=math.atan2(math.sin(self.yaw - parent.yaw), math.cos(self.yaw - parent.yaw)),
        )

    def distance(self, other: Pose2) -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


Face = Literal["pan", "bottom", "left", "right"]


class GoalEdge(PlanningValue):
    a: str = Field(min_length=1)
    face_a: Face
    b: str = Field(min_length=1)
    face_b: Face
    orientation_rad: float | None = None

    @property
    def key(self) -> tuple[str, str]:
        first, second = sorted((f"{self.a}/{self.face_a}", f"{self.b}/{self.face_b}"))
        return first, second


class AssemblyGoal(PlanningValue):
    id: str = Field(min_length=1)
    nodes: tuple[str, ...] = Field(min_length=1)
    edges: tuple[GoalEdge, ...]

    @model_validator(mode="after")
    def validate_tree(self) -> Self:
        if len(set(self.nodes)) != len(self.nodes):
            raise ValueError("goal module IDs must be unique")
        if len(self.edges) != len(self.nodes) - 1:
            raise ValueError("online SMORES planning requires a tree goal")
        seen: set[str] = set()
        adjacency: dict[str, list[str]] = {node: [] for node in self.nodes}
        for edge in self.edges:
            if edge.a == edge.b or edge.a not in adjacency or edge.b not in adjacency:
                raise ValueError("goal edge references an invalid module")
            for endpoint in edge.key:
                if endpoint in seen:
                    raise ValueError("a goal connector cannot be used twice")
                seen.add(endpoint)
            adjacency[edge.a].append(edge.b)
            adjacency[edge.b].append(edge.a)
            quarter = (edge.orientation_rad or 0.0) / (math.pi / 2)
            if abs(quarter - round(quarter)) > 1e-6:
                raise ValueError("SMORES mating orientation must be a quarter turn")
            if edge.face_a == edge.face_b == "bottom" and abs(edge.orientation_rad or 0.0) > 1e-6:
                raise ValueError("twisted bottom-bottom goals are outside the planar baseline")
        pending, visited = [self.nodes[0]], set[str]()
        while pending:
            node = pending.pop()
            if node not in visited:
                visited.add(node)
                pending.extend(adjacency[node])
        if visited != set(self.nodes):
            raise ValueError("goal must be connected and acyclic")
        return self


class Assignment(PlanningValue):
    goal_node: str
    module_id: str
    target: Pose2


class AssemblyAction(PlanningValue):
    id: str
    moving: str
    moving_face: Face
    parent: str
    parent_face: Face
    depth: int = Field(ge=1)
    batch: int = Field(ge=0)
    dependencies: tuple[str, ...] = ()
    orientation_rad: float | None = None

    @property
    def edge(self) -> GoalEdge:
        return GoalEdge(
            a=self.parent,
            face_a=self.parent_face,
            b=self.moving,
            face_b=self.moving_face,
            orientation_rad=self.orientation_rad,
        )


class AssemblyPlan(PlanningValue):
    goal: AssemblyGoal
    root: str
    root_module: str
    assignments: tuple[Assignment, ...]
    actions: tuple[AssemblyAction, ...]
    releases: tuple[GoalEdge, ...] = ()
    assignment_cost_m: float = Field(ge=0.0)


class TimedPose(PlanningValue):
    pose: Pose2
    time_s: float = Field(ge=0.0)
    direction: Literal[-1, 0, 1] = 1


class PlannerDecision(PlanningValue):
    sequence: int
    time_s: float
    kind: str
    detail: str


class ActionInterval(PlanningValue):
    phase: str
    start_s: float
    end_s: float | None = None


class ActionObservation(PlanningValue):
    action: AssemblyAction
    phase: Literal[
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
    reason: str = ""
    members: tuple[str, ...] = ()
    path: tuple[TimedPose, ...] = ()
    waypoint_index: int = 0
    started_s: float | None = None
    finished_s: float | None = None
    position_error_m: float | None = None
    orientation_error_rad: float | None = None
    relative_velocity_m_s: float | None = None
    tracking_error_m: float | None = None
    intervals: tuple[ActionInterval, ...] = ()


class PlanningSnapshot(PlanningValue):
    """Intent is separate from canonical model-view nodes and committed edges."""

    time_s: float
    sample_sequence: int
    topology_revision: int
    plan_revision: int
    plan: AssemblyPlan
    actions: tuple[ActionObservation, ...]
    decisions: tuple[PlannerDecision, ...] = ()
    replans: int = 0
    planning_ms: float = 0.0
    path_length_m: float = 0.0
    footprint_half_m: tuple[float, float] = (0.044, 0.044)
    footprint_center_x_m: float = 0.033
