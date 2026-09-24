"""Engine-independent support invariants and bounded motion search."""

from __future__ import annotations

import pytest

from modsim.planning.spatial import (
    Bond,
    SpatialPlanningError,
    plan_joint_path,
    plan_support_changes,
    supported,
)


def test_handoff_requires_capture_before_release() -> None:
    old = Bond("helper/pan", "payload/bottom")
    new = Bond("receiver/pan", "payload/left")
    modules = frozenset(("helper", "receiver", "payload"))
    anchors = frozenset(("helper", "receiver"))
    initial, goal = frozenset((old,)), frozenset((new,))
    actions = plan_support_changes(modules, initial, goal, anchors)
    assert [a.operation for a in actions] == ["dock", "release"]
    state = initial
    for action in actions:
        state ^= {action.bond}
        assert supported(modules, state, anchors)
    assert state == goal


def test_connector_reuse_cannot_fake_a_supported_handoff() -> None:
    modules = frozenset(("helper", "receiver", "payload"))
    anchors = frozenset(("helper", "receiver"))
    initial = frozenset((Bond("helper/pan", "payload/bottom"),))
    goal = frozenset((Bond("receiver/pan", "payload/bottom"),))
    with pytest.raises(SpatialPlanningError, match="no supported sequence"):
        plan_support_changes(modules, initial, goal, anchors)


def test_missing_support_is_refused() -> None:
    with pytest.raises(SpatialPlanningError, match="supported"):
        plan_support_changes(frozenset(("a", "b")), frozenset(), frozenset(), frozenset(("a",)))


def test_bonds_have_canonical_identity() -> None:
    assert Bond("a/x", "b/y") == Bond("b/y", "a/x")
    with pytest.raises(SpatialPlanningError):
        Bond("a/x", "a/y")


def test_motion_search_checks_between_grid_vertices() -> None:
    # The direct route has valid endpoints but a narrow obstacle between them.
    def feasible(point: tuple[float, ...]) -> bool:
        return not (0.4 <= point[0] <= 0.6 and point[1] < 0.8)

    path = plan_joint_path((0, 0), (1, 0), (1, 1), feasible)
    assert path.points == ((0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0))
    assert path.rejected > 0


def test_motion_search_failure_and_budget_are_explicit() -> None:
    with pytest.raises(SpatialPlanningError, match="no feasible"):
        plan_joint_path((0,), (2,), (2,), lambda p: p[0] != 1)
    with pytest.raises(SpatialPlanningError, match="budget"):
        plan_joint_path((0,), (2,), (2,), lambda _: True, max_states=1)
    with pytest.raises(SpatialPlanningError, match="start or goal"):
        plan_joint_path((0,), (2,), (2,), lambda _: False)


@pytest.mark.parametrize(
    "start,goal,upper", [((0,), (1, 2), (2,)), ((-1,), (1,), (2,)), ((), (), ())]
)
def test_bad_grids_are_refused(
    start: tuple[int, ...], goal: tuple[int, ...], upper: tuple[int, ...]
) -> None:
    with pytest.raises(SpatialPlanningError, match="invalid"):
        plan_joint_path(start, goal, upper, lambda _: True)
