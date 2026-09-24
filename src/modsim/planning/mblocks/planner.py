"""Original implementation of Sung et al.'s 2015 planar line construction.

Algorithms 1/2, including the P3 removal queue. Each emitted pivot is checked
against the full swept-cell mask. Shape-to-shape construction reverses the
target's route through the same registered line, retaining actual module IDs.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Generator

from modsim.planning.mblocks.geometry import (
    add,
    apply_pivot,
    boundary_order,
    inadmissible_reason,
    p3_triples,
    pivots,
)
from modsim.planning.mblocks.models import (
    Cell,
    LatticeBlock,
    LatticeGoal,
    LatticePivot,
    LatticePlan,
    LatticeState,
)


class LatticePlanningError(ValueError):
    """Unsupported geometry, unavailable route, or exhausted search budget."""


def _walk_to_tail(
    state: LatticeState, moving: str, destination: Cell, max_moves: int
) -> tuple[LatticeState, tuple[LatticePivot, ...]]:
    actions: list[LatticePivot] = []
    visited: set[Cell] = set()
    while next(block.cell for block in state.blocks if block.id == moving) != destination:
        cell = next(block.cell for block in state.blocks if block.id == moving)
        if cell in visited or len(actions) >= max_moves:
            raise LatticePlanningError("Clockwise boundary traversal cannot reach the line tail")
        visited.add(cell)
        candidates = pivots(state, moving, direction=-1)
        if not candidates:
            raise LatticePlanningError(f"No clockwise pivot for {moving} at {cell}")
        action = candidates[0]
        actions.append(action)
        state = apply_pivot(state, action)
    return state, tuple(actions)


def _to_line(state: LatticeState) -> tuple[LatticeState, tuple[LatticePivot, ...], tuple[str, ...]]:
    problem = inadmissible_reason(state.cells)
    if problem:
        raise LatticePlanningError(problem)
    extreme = max(state.cells, key=lambda cell: (cell[1], cell[0]))
    anchor = state.at(extreme).id
    line = frozenset(add(extreme, (0, index)) for index in range(len(state.blocks)))
    actions: list[LatticePivot] = []
    selections: list[str] = []
    queue: deque[str] = deque()
    limit = 16 * len(state.blocks) ** 2
    while state.cells != line:
        top = max(state.cells, key=lambda cell: (cell[1], cell[0]))
        destination = add(top, (0, 1))
        tail = {cell for cell in state.cells if cell[0] == extreme[0] and cell[1] >= extreme[1]}
        moving = ""
        reason = "Boundary traversal"
        chosen: tuple[LatticeState, tuple[LatticePivot, ...]] | None = None
        if queue:
            reason = "P3 removal queue"
            moving = queue.popleft()
            chosen = _walk_to_tail(state, moving, destination, limit)
        else:
            for cell in boundary_order(state.cells, top):
                if cell in tail:
                    continue
                moving = state.at(cell).id
                if not pivots(state, moving, direction=-1):
                    continue
                remainder = state.cells - {cell}
                triples = p3_triples(state.cells, cell)
                if inadmissible_reason(remainder) and not triples:
                    continue
                try:
                    candidate = _walk_to_tail(state, moving, destination, limit)
                except LatticePlanningError:
                    continue
                if inadmissible_reason(remainder):
                    # Validate the whole special-case queue before committing its
                    # first choice, so an ambiguous rotated P3 cannot strand us.
                    for second, third in triples:
                        ids = (state.at(second).id, state.at(third).id)
                        if anchor in ids or second in tail or third in tail:
                            continue
                        probe = candidate[0]
                        try:
                            for offset, module in enumerate(ids, start=1):
                                probe, _ = _walk_to_tail(
                                    probe, module, add(destination, (0, offset)), limit
                                )
                        except LatticePlanningError:
                            continue
                        if inadmissible_reason(probe.cells) is None:
                            queue.extend(ids)
                            reason = f"P3 opening; then remove {ids[0]} and {ids[1]}"
                            break
                    else:
                        continue
                chosen = candidate
                break
        if chosen is None:
            raise LatticePlanningError("No supported admissible boundary removal was found")
        state, moves = chosen
        selections.append(moving)
        actions.extend(move.model_copy(update={"reason": reason}) for move in moves)
        if len(actions) > limit:
            raise LatticePlanningError(f"Line construction exceeded its {limit}-move bound")
    return state, tuple(actions), tuple(selections)


def plan_reconfiguration(initial: LatticeState, goal: LatticeGoal) -> LatticePlan:
    """Plan unlabelled shape reconfiguration with a common extreme anchor."""
    if len(initial.blocks) != len(goal.cells):
        raise LatticePlanningError("Initial and goal module counts differ")
    for label, cells in (("Initial", initial.cells), ("Goal", frozenset(goal.cells))):
        problem = inadmissible_reason(cells)
        if problem:
            raise LatticePlanningError(f"{label}: {problem}")
    extreme = max(initial.cells, key=lambda cell: (cell[1], cell[0]))
    anchor = initial.at(extreme).id
    if max(goal.cells, key=lambda cell: (cell[1], cell[0])) != extreme:
        # A canonical line itself extends above the original extreme.
        canonical = {add(extreme, (0, index)) for index in range(len(initial.blocks))}
        if set(goal.cells) != canonical:
            raise LatticePlanningError("Initial and goal shapes must share their extreme anchor")
    if initial.cells == frozenset(goal.cells):
        return LatticePlan(
            initial=initial, goal=goal, anchor=anchor, anchor_cell=extreme, actions=()
        )
    line_state, forward, order = _to_line(initial)
    if line_state.cells == frozenset(goal.cells):
        return LatticePlan(
            initial=initial,
            goal=goal,
            anchor=anchor,
            anchor_cell=extreme,
            actions=forward,
            boundary_order=order,
        )
    target = LatticeState(
        blocks=tuple(
            LatticeBlock(id=f"goal_{index}", cell=cell) for index, cell in enumerate(goal.cells)
        )
    )
    _, reverse, _ = _to_line(target)
    state = line_state
    result = list(forward)
    for step in reversed(reverse):
        moving = state.at(step.destination).id
        inverse = next(
            (
                action
                for action in pivots(state, moving)
                if action.destination == step.source
                and action.pivot_twice == step.pivot_twice
                and action.turns == -step.turns
            ),
            None,
        )
        if inverse is None:
            raise LatticePlanningError("Target route contains an unsupported inverse pivot")
        inverse = inverse.model_copy(update={"reason": "Reverse target-to-line construction"})
        result.append(inverse)
        state = apply_pivot(state, inverse)
    return LatticePlan(
        initial=initial,
        goal=goal,
        anchor=anchor,
        anchor_cell=extreme,
        actions=tuple(result),
        boundary_order=order,
    )


def recovery_search(
    initial: LatticeState,
    goal: LatticeGoal,
    anchor: str,
    *,
    max_expansions: int = 4000,
    batch_size: int = 20,
) -> Generator[int, None, tuple[LatticePivot, ...]]:
    """Cooperative bounded BFS for a settled deviation; no completeness claim.

    Yields expansion counts so the runtime owner can advance physics and accept
    cancellation between batches. Geometry is label-independent; returned moves
    retain the identities of the discovered path. The anchor remains fixed.
    """
    if max_expansions < 1 or batch_size < 1:
        raise ValueError("search bounds must be positive")
    pending: deque[tuple[LatticeState, tuple[LatticePivot, ...]]] = deque([(initial, ())])
    visited = {initial.cells}
    goal_cells = frozenset(goal.cells)
    for expanded in range(max_expansions):
        if not pending:
            raise LatticePlanningError("No route in the bounded recovery workspace")
        state, route = pending.popleft()
        if state.cells == goal_cells:
            return route
        for block in state.blocks:
            if block.id == anchor:
                continue
            for action in pivots(state, block.id):
                successor = apply_pivot(state, action)
                if successor.cells not in visited:
                    visited.add(successor.cells)
                    pending.append((successor, (*route, action)))
        if (expanded + 1) % batch_size == 0:
            yield expanded + 1
    raise LatticePlanningError(f"Recovery search exhausted {max_expansions} expansions")
