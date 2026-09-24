"""Immutable spatial planner observations for local and process-based inspectors."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from modsim.core.transforms import Vec3


class PlanningValue(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", allow_inf_nan=False, strict=True)


class TargetBond(PlanningValue):
    a: str
    b: str


class TargetModule(PlanningValue):
    id: str
    position_m: Vec3
    anchored: bool = False


class PlannedTrace(PlanningValue):
    module_id: str
    stage: Literal["moving", "retreat"]
    positions_m: tuple[Vec3, ...]


class PlannerDecision(PlanningValue):
    sequence: int = Field(ge=0)
    time_s: float = Field(ge=0)
    kind: str
    detail: str


class ActionInterval(PlanningValue):
    phase: str
    start_s: float
    end_s: float | None = None


class SpatialAction(PlanningValue):
    id: str
    label: str
    phase: str
    detail: str
    waypoint: int = Field(ge=0)
    waypoint_count: int = Field(ge=0)
    intervals: tuple[ActionInterval, ...] = ()


class SpatialPlanningSnapshot(PlanningValue):
    """Intent and diagnostics stamped with the canonical observation they describe."""

    time_s: float
    sample_sequence: int
    topology_revision: int
    phase: str
    detail: str
    targets: tuple[TargetModule, ...]
    target_bonds: tuple[TargetBond, ...]
    traces: tuple[PlannedTrace, ...]
    actions: tuple[SpatialAction, ...]
    decisions: tuple[PlannerDecision, ...]
    approach_expanded: int
    approach_rejected: int
    withdrawal_expanded: int
    withdrawal_rejected: int
    target_error_m: float
    joint_error_rad: float
    peak_effort_nm: float
    peak_penetration_m: float
