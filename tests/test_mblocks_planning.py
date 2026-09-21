"""Geometric planner checks independent of NumPy, MuJoCo and Qt."""

from __future__ import annotations

import math

import pytest

from modsim.planning.mblocks import (
    LatticeBlock,
    LatticeGoal,
    LatticePlanningError,
    LatticeState,
    apply_pivot,
    inadmissible_reason,
    pivots,
    plan_reconfiguration,
)
from modsim.planning.mblocks.geometry import connected
from modsim.planning.mblocks.planner import recovery_search
from modsim.runtime.mblocks_lattice import elbow_goal, line_goal, rectangle_state


def test_p3_queue_opens_an_admissible_ring_and_reaches_the_line() -> None:
    # A 5x5 perimeter surrounds a 3x3 hole. Removing its corner creates a
    # forbidden diagonal pattern, requiring the paper's queued triple removal.
    cells = tuple((x, y) for y in range(5) for x in range(5) if x in (0, 4) or y in (0, 4))
    initial = LatticeState(
        blocks=tuple(LatticeBlock(id=f"block_{i}", cell=cell) for i, cell in enumerate(cells))
    )
    plan = plan_reconfiguration(initial, line_goal(initial))
    assert any(action.reason.startswith("P3 opening") for action in plan.actions)
    assert any(action.reason == "P3 removal queue" for action in plan.actions)
    measured = initial
    for action in plan.actions:
        assert connected(measured.cells - {action.source})
        measured = apply_pivot(measured, action)
    assert measured.cells == frozenset(plan.goal.cells)


def test_already_reached_goal_still_requires_a_valid_connected_shape() -> None:
    state = LatticeState(
        blocks=(
            LatticeBlock(id="left", cell=(0, 0)),
            LatticeBlock(id="right", cell=(2, 0)),
        )
    )
    with pytest.raises(LatticePlanningError, match="connected"):
        plan_reconfiguration(state, LatticeGoal(id="invalid", cells=((0, 0), (2, 0))))


def _state(*cells: tuple[int, int]) -> LatticeState:
    return LatticeState(blocks=tuple(LatticeBlock(id=str(i), cell=c) for i, c in enumerate(cells)))


@pytest.mark.parametrize("width,height", [(2, 2), (3, 2), (2, 3), (4, 3)])
def test_generated_rectangle_to_line_preserves_geometry(width: int, height: int) -> None:
    state = rectangle_state(width, height)
    goal = line_goal(state)
    plan = plan_reconfiguration(state, goal)
    assert plan == plan_reconfiguration(state, goal)
    assert len({a.moving for a in plan.actions}) > 1
    for action in plan.actions:
        moving = state.at(action.source)
        assert moving.id == action.moving
        assert connected(state.cells - {action.source})
        assert not ((state.cells - {action.source}) & set(action.clearance_cells))
        # Independent continuous square-vs-square SAT checks; these do not use
        # the planner's grid masks or its pose-update implementation.
        for tick in range(1, 90):
            angle = action.turns * math.pi / 2 * tick / 90
            px, py = action.pivot_twice[0] / 2, action.pivot_twice[1] / 2
            dx, dy = action.source[0] - px, action.source[1] - py
            center = (
                px + math.cos(angle) * dx - math.sin(angle) * dy,
                py + math.sin(angle) * dx + math.cos(angle) * dy,
            )
            for obstacle in state.cells - {action.source}:
                assert not _square_overlap(center, angle, obstacle)
        state = apply_pivot(state, action)
        assert connected(state.cells)
    assert state.cells == frozenset(goal.cells)
    assert state.at(plan.anchor_cell).id == plan.anchor


def _square_overlap(center: tuple[float, float], angle: float, obstacle: tuple[int, int]) -> bool:
    u, v = (math.cos(angle), math.sin(angle)), (-math.sin(angle), math.cos(angle))
    delta = (obstacle[0] - center[0], obstacle[1] - center[1])
    for axis in ((1.0, 0.0), (0.0, 1.0), u, v):
        distance = abs(delta[0] * axis[0] + delta[1] * axis[1])
        moving_radius = 0.5 * (
            abs(u[0] * axis[0] + u[1] * axis[1]) + abs(v[0] * axis[0] + v[1] * axis[1])
        )
        stationary_radius = 0.5 * (abs(axis[0]) + abs(axis[1]))
        if distance >= moving_radius + stationary_radius - 1e-9:
            return False
    return True


def test_elbow_uses_registered_line_and_actual_module_identities() -> None:
    initial = rectangle_state()
    goal = elbow_goal(initial)
    plan = plan_reconfiguration(initial, goal)
    state = initial
    saw_line = False
    for action in plan.actions:
        state = apply_pivot(state, action)
        saw_line |= state.cells == frozenset(line_goal(initial).cells)
    assert saw_line
    assert state.cells == frozenset(goal.cells)
    assert {block.id for block in state.blocks} == {block.id for block in initial.blocks}
    assert {a.turns for a in plan.actions} == {-2, -1, 1, 2}


def test_quarter_turn_stops_at_first_face_contact_and_checks_sweep() -> None:
    state = _state((0, 0), (1, 0), (0, 1))
    move = next(a for a in pivots(state, "2", direction=-1) if a.support == "0")
    assert move.turns == -1
    assert move.destination == (1, 1)
    assert set(move.clearance_cells) == {(0, 2), (1, 2), (1, 1)}
    blocked = _state((0, 0), (1, 0), (0, 1), (1, 1), (1, 2))
    assert not any(a.destination == (1, 1) for a in pivots(blocked, "2", direction=-1))


def test_half_turn_and_inverse_restore_orientation() -> None:
    state = _state((0, 0), (0, 1))
    action = pivots(state, "1", direction=-1)[0]
    assert action.turns == -2
    after = apply_pivot(state, action)
    inverse = next(
        a for a in pivots(after, "1") if a.pivot_twice == action.pivot_twice and a.turns == 2
    )
    assert apply_pivot(after, inverse) == state


def test_articulation_module_cannot_leave_stationary_parts_disconnected() -> None:
    assert pivots(_state((0, 0), (1, 0), (2, 0)), "1") == ()


def test_rejects_gaps_conflicts_unregistered_goals_and_illegal_moves() -> None:
    assert inadmissible_reason(_state((0, 0), (0, 1), (1, 1), (2, 1), (2, 0)).cells)
    with pytest.raises(ValueError, match="same lattice cell"):
        _state((0, 0), (0, 0))
    initial = rectangle_state()
    with pytest.raises(LatticePlanningError, match="anchor"):
        plan_reconfiguration(
            initial, LatticeGoal(id="wrong_frame", cells=tuple((0, i) for i in range(6)))
        )
    action = plan_reconfiguration(initial, line_goal(initial)).actions[0]
    with pytest.raises(ValueError, match="Illegal pivot"):
        apply_pivot(initial, action.model_copy(update={"destination": (99, 99)}))


def test_bounded_recovery_yields_and_reaches_a_nearby_goal() -> None:
    state = _state((0, 0), (0, 1))
    goal = LatticeGoal(id="side", cells=((0, 0), (1, 0)))
    search = recovery_search(state, goal, "0", batch_size=1)
    assert next(search) == 1
    with pytest.raises(StopIteration) as result:
        next(search)
    final = state
    for action in result.value.value:
        final = apply_pivot(final, action)
    assert final.cells == frozenset(goal.cells)
    with pytest.raises(LatticePlanningError, match="exhausted"):
        list(recovery_search(state, goal, "0", max_expansions=1))
