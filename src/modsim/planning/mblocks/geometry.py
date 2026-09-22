"""Exact grid masks for the swept volume of 90/180 degree cube pivots.

Sung et al., ICRA 2015, Figures 2, 5 and 7. Cells have unit pitch. The
quarter-turn sweep intersects the destination and the two cells just outside
the traversed faces. A half turn adds the second quarter's three cells.
Touching the supporting edge is allowed; positive-volume overlap is not.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable
from typing import Literal

from modsim.planning.mblocks.models import Cell, LatticePivot, LatticeState

DIRECTIONS: tuple[Cell, ...] = ((1, 0), (0, 1), (-1, 0), (0, -1))


def add(a: Cell, b: Cell) -> Cell:
    return a[0] + b[0], a[1] + b[1]


def sub(a: Cell, b: Cell) -> Cell:
    return a[0] - b[0], a[1] - b[1]


def neighbors(cell: Cell) -> tuple[Cell, ...]:
    return tuple(add(cell, direction) for direction in DIRECTIONS)


def connected(cells: Iterable[Cell]) -> bool:
    remaining = set(cells)
    if not remaining:
        return False
    pending = [remaining.pop()]
    while pending:
        for cell in neighbors(pending.pop()):
            if cell in remaining:
                remaining.remove(cell)
                pending.append(cell)
    return not remaining


def inadmissible_reason(cells: frozenset[Cell]) -> str | None:
    """Recognize all rotations/reflections of the three planar forbidden rules."""
    if not connected(cells):
        return "The face-adjacent lattice must be connected"
    for p in sorted(cells):
        for u in DIRECTIONS:
            if add(add(p, u), u) in cells and add(p, u) not in cells:
                return f"Paper rule 1: one-cell gap at {add(p, u)}"
            for v in ((-u[1], u[0]), (u[1], -u[0])):
                if add(add(p, u), v) in cells and add(p, u) not in cells and add(p, v) not in cells:
                    return f"Paper rule 2: diagonal-only contact near {p}"
                far = add(add(add(p, u), u), v)
                empty = (add(p, u), add(add(p, u), u), add(p, v), add(add(p, u), v))
                if far in cells and not any(cell in cells for cell in empty):
                    return f"Paper rule 3: staggered gap near {p}"
    return None


def pivots(state: LatticeState, moving: str, *, direction: int = 0) -> tuple[LatticePivot, ...]:
    """Return legal supported pivots, stopping at the first face contact.

    Direction is +1 counterclockwise, -1 clockwise, or 0 for both. The
    stationary remainder stays face-connected; the mover retains an edge hinge.
    """
    if direction not in (-1, 0, 1):
        raise ValueError("pivot direction must be -1, 0, or 1")
    block = next(block for block in state.blocks if block.id == moving)
    p = block.cell
    remainder = state.cells - {p}
    if not connected(remainder):
        return ()
    result: list[LatticePivot] = []
    for s in sorted(set(neighbors(p)) & remainder):
        d = sub(p, s)
        for sign in (direction,) if direction else (-1, 1):
            tangent = (-sign * d[1], sign * d[0])
            quarter = add(p, tangent)
            clearance = (quarter, add(p, d), add(quarter, d))
            if any(cell in remainder for cell in clearance):
                continue
            turns: Literal[-2, -1, 1, 2]
            if set(neighbors(quarter)) & remainder:
                destination = quarter
                turns = -1 if sign < 0 else 1
            else:
                destination = add(s, tangent)
                clearance += (destination, add(quarter, tangent), add(destination, tangent))
                if any(cell in remainder for cell in clearance):
                    continue
                turns = -2 if sign < 0 else 2
            result.append(
                LatticePivot(
                    moving=moving,
                    support=state.at(s).id,
                    source=p,
                    destination=destination,
                    pivot_twice=add(add(p, s), tangent),
                    turns=turns,
                    clearance_cells=tuple(sorted(set(clearance))),
                )
            )
    return tuple(result)


def apply_pivot(state: LatticeState, action: LatticePivot) -> LatticeState:
    """Validate a transition before applying it; never accept arbitrary cell edits."""
    if not any(
        candidate.model_copy(update={"reason": action.reason}) == action
        for candidate in pivots(state, action.moving)
    ):
        raise ValueError(
            f"Illegal pivot of {action.moving}: {action.source} -> {action.destination}"
        )
    return LatticeState(
        blocks=tuple(
            block.model_copy(
                update={
                    "cell": action.destination,
                    "quarter_turns": (block.quarter_turns + action.turns) % 4,
                }
            )
            if block.id == action.moving
            else block
            for block in state.blocks
        )
    )


def boundary_order(cells: frozenset[Cell], start: Cell) -> tuple[Cell, ...]:
    """Trace the exterior counterclockwise, starting along the extreme top face."""
    # Doubled integer vertices avoid rounding and retain sharp concave corners.
    edges: dict[tuple[Cell, Cell], Cell] = {}
    outgoing: dict[Cell, list[Cell]] = defaultdict(list)
    for x, y in sorted(cells):
        vertices = (
            (2 * x - 1, 2 * y - 1),
            (2 * x + 1, 2 * y - 1),
            (2 * x + 1, 2 * y + 1),
            (2 * x - 1, 2 * y + 1),
        )
        for index, normal in enumerate(((0, -1), (1, 0), (0, 1), (-1, 0))):
            if add((x, y), normal) in cells:
                continue
            a, b = vertices[index], vertices[(index + 1) % 4]
            edges[a, b] = (x, y)
            outgoing[a].append(b)
    first = ((2 * start[0] + 1, 2 * start[1] + 1), (2 * start[0] - 1, 2 * start[1] + 1))
    if first not in edges:
        raise ValueError("boundary start must have an exposed top face")
    edge = first
    visited: set[tuple[Cell, Cell]] = set()
    ordered: list[Cell] = []
    while edge not in visited:
        visited.add(edge)
        owner = edges[edge]
        if owner not in ordered:
            ordered.append(owner)
        a, b = edge
        choices = outgoing[b]
        incoming = sub(b, a)
        # At a diagonal contact follow the sharpest left turn, keeping the
        # occupied component on the left. Valid admissible boundaries are unique.
        c = max(
            choices, key=lambda v: (incoming[0] * (v[1] - b[1]) - incoming[1] * (v[0] - b[0]), v)
        )
        edge = b, c
    return tuple(ordered)


def p3_triples(cells: frozenset[Cell], first: Cell) -> tuple[tuple[Cell, Cell], ...]:
    """Figure 7: three consecutive occupied cells with an empty parallel row."""
    result: list[tuple[Cell, Cell]] = []
    for u in DIRECTIONS:
        second, third = add(first, u), add(add(first, u), u)
        if second not in cells or third not in cells:
            continue
        for normal in ((-u[1], u[0]), (u[1], -u[0])):
            if all(add(cell, normal) not in cells for cell in (first, second, third)):
                result.append((second, third))
    return tuple(result)
