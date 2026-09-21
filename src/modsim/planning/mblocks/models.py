"""Immutable inputs and diagnostics for single-plane pivoting cubes."""

from __future__ import annotations

from typing import Literal, Self

from pydantic import Field, model_validator

from modsim.planning.models import PlannerDecision, PlanningValue

Cell = tuple[int, int]


class LatticeBlock(PlanningValue):
    id: str = Field(min_length=1)
    cell: Cell
    quarter_turns: int = Field(default=0, ge=0, le=3)


class LatticeState(PlanningValue):
    blocks: tuple[LatticeBlock, ...] = Field(min_length=2, max_length=32)

    @model_validator(mode="after")
    def unique_occupancy(self) -> Self:
        if len({block.id for block in self.blocks}) != len(self.blocks):
            raise ValueError("lattice module IDs must be unique")
        if len({block.cell for block in self.blocks}) != len(self.blocks):
            raise ValueError("two modules cannot occupy the same lattice cell")
        return self

    @property
    def cells(self) -> frozenset[Cell]:
        return frozenset(block.cell for block in self.blocks)

    def at(self, cell: Cell) -> LatticeBlock:
        return next(block for block in self.blocks if block.cell == cell)


class LatticeGoal(PlanningValue):
    """Unlabelled shape in an explicitly assembly-relative XY lattice frame."""

    id: str = Field(min_length=1)
    cells: tuple[Cell, ...] = Field(min_length=2, max_length=32)
    pitch_m: float = Field(default=0.05, gt=0.0)
    frame: Literal["assembly_relative"] = "assembly_relative"

    @model_validator(mode="after")
    def unique_cells(self) -> Self:
        if len(set(self.cells)) != len(self.cells):
            raise ValueError("goal cells must be unique")
        return self


class LatticePivot(PlanningValue):
    moving: str
    support: str
    source: Cell
    destination: Cell
    pivot_twice: Cell
    turns: Literal[-2, -1, 1, 2]
    clearance_cells: tuple[Cell, ...]
    reason: str = "Boundary traversal"


class LatticePlan(PlanningValue):
    algorithm: Literal["sung2015_planar", "bounded_recovery"] = "sung2015_planar"
    initial: LatticeState
    goal: LatticeGoal
    anchor: str
    anchor_cell: Cell
    actions: tuple[LatticePivot, ...]
    boundary_order: tuple[str, ...] = ()


class LatticeActionRecord(PlanningValue):
    index: int
    moving: str
    source: Cell
    destination: Cell
    turns: int
    phase: str
    started_at_s: float
    ended_at_s: float | None = None
    detail: str = ""


class LatticePlanningSnapshot(PlanningValue):
    kind: Literal["mblocks_lattice"] = "mblocks_lattice"
    time_s: float
    sample_sequence: int
    topology_revision: int
    plan_revision: int
    goal: LatticeGoal
    anchor: str
    origin_world_m: tuple[float, float, float]
    frame_yaw_rad: float
    blocks: tuple[LatticeBlock, ...]
    active: LatticePivot | None = None
    remaining: tuple[LatticePivot, ...] = ()
    boundary_order: tuple[str, ...] = ()
    decisions: tuple[PlannerDecision, ...] = ()
    history: tuple[LatticeActionRecord, ...] = ()
    completed_actions: int = 0
    replans: int = 0
    planning_ms: float = 0.0
    flywheel_rpm: float = 0.0
    pivot_angle_deg: float = 0.0
    landing_error_m: float = 0.0
    landing_effort_nm: float = 0.0
    phase: str = "planning"
    detail: str = ""
