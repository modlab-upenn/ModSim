"""Paper assignment, topology constraints, and independent motion-planning checks."""

from __future__ import annotations

import itertools
import math
import random

import pytest

from modsim.planning import (
    AssemblyGoal,
    GoalEdge,
    Pose2,
    minimum_assignment,
    plan_assembly,
    tree_root,
)
from modsim.planning.models import TimedPose
from modsim.planning.routing import (
    Footprint,
    Obstacle,
    Reservation,
    RouteConfig,
    RouteUnavailable,
    plan_route,
    swept_clear,
)
from modsim.planning.smores import (
    driver_to_snake_goal,
    mobile_manipulator_goal,
    paper_initial_poses,
)


def test_hungarian_matches_exhaustive_minimum_and_is_deterministic() -> None:
    rng = random.Random(17)
    for n in range(1, 7):
        costs = [[rng.randrange(30) / 10 for _ in range(n)] for _ in range(n)]
        assignment = minimum_assignment(costs)
        expected = min(
            sum(costs[i][p[i]] for i in range(n)) for p in itertools.permutations(range(n))
        )
        assert sum(costs[i][assignment[i]] for i in range(n)) == pytest.approx(expected)
        assert minimum_assignment(costs) == assignment
    assert minimum_assignment([[1.0] * 4 for _ in range(4)]) == (0, 1, 2, 3)
    assert minimum_assignment([]) == ()


def test_assignment_rejects_malformed_costs() -> None:
    for costs in ([[1.0, 2.0]], [[math.inf]], [[math.nan]]):
        with pytest.raises(ValueError):
            minimum_assignment(costs)


def test_paper_task_one_mapping_and_parallel_root_groups() -> None:
    plan = plan_assembly(mobile_manipulator_goal(), paper_initial_poses())
    assert plan.root == "goal_0"
    assert plan.root_module == "module_1"
    assert {a.goal_node: a.module_id for a in plan.assignments} == {
        "goal_0": "module_1",
        "goal_1": "module_5",
        "goal_2": "module_2",
        "goal_3": "module_0",
        "goal_4": "module_6",
        "goal_5": "module_4",
        "goal_6": "module_3",
    }
    groups = [
        {a.moving for a in plan.actions if a.batch == batch}
        for batch in sorted({a.batch for a in plan.actions})
    ]
    assert groups == [
        {"module_0", "module_5"},
        {"module_2", "module_6"},
        {"module_4"},
        {"module_3"},
    ]
    for action in plan.actions:
        assert all(
            next(a for a in plan.actions if a.id == d).batch < action.batch
            for d in action.dependencies
        )


def test_goal_validation_rejects_cycle_and_duplicate_connector() -> None:
    with pytest.raises(ValueError, match="tree"):
        AssemblyGoal(
            id="loop",
            nodes=("a", "b"),
            edges=(
                GoalEdge(a="a", face_a="pan", b="b", face_b="bottom"),
                GoalEdge(a="a", face_a="bottom", b="b", face_b="pan"),
            ),
        )
    with pytest.raises(ValueError, match="twice"):
        AssemblyGoal(
            id="conflict",
            nodes=("a", "b", "c"),
            edges=(
                GoalEdge(a="a", face_a="pan", b="b", face_b="bottom"),
                GoalEdge(a="a", face_a="pan", b="c", face_b="bottom"),
            ),
        )


def test_wheel_face_goal_requires_helper_instead_of_silent_approximation() -> None:
    goal = AssemblyGoal(
        id="helper",
        nodes=("a", "b"),
        edges=(GoalEdge(a="a", face_a="right", b="b", face_b="left"),),
    )
    with pytest.raises(ValueError, match="helping module"):
        plan_assembly(goal, {"one": Pose2(x=0.0, y=0.0), "two": Pose2(x=1.0, y=0.0)})


def test_driver_goal_preserves_correct_bonds_and_derives_changes() -> None:
    goal = driver_to_snake_goal()

    def edge(a: int, fa: str, b: int, fb: str) -> GoalEdge:
        return GoalEdge.model_validate(
            {"a": f"module_{a}", "face_a": fa, "b": f"module_{b}", "face_b": fb}
        )

    initial = (
        edge(2, "pan", 1, "bottom"),
        edge(2, "bottom", 3, "pan"),
        edge(2, "right", 4, "left"),
        edge(4, "right", 5, "left"),
        edge(5, "pan", 6, "bottom"),
        edge(5, "bottom", 7, "pan"),
    )
    poses = {node: Pose2(x=float(i), y=0.0) for i, node in enumerate(goal.nodes)}
    assert tree_root(goal) == "module_4"
    with pytest.raises(ValueError, match="explicit"):
        plan_assembly(goal, poses, existing=initial)
    plan = plan_assembly(goal, poses, existing=initial, fixed_mapping={n: n for n in goal.nodes})
    assert len(plan.releases) == len(plan.actions) == 4
    assert {a.moving for a in plan.actions if not a.dependencies} == {"module_2", "module_5"}
    assert {a.moving for a in plan.actions if a.dependencies} == {"module_1", "module_7"}
    assert initial[1] not in plan.releases and initial[4] not in plan.releases


def test_sweep_checks_interior_and_whole_subassembly() -> None:
    footprint = Footprint(half_x=0.02, half_y=0.02, center_x=0.0, margin=0.0)
    a, b = Pose2(x=0.0, y=0.0), Pose2(x=0.3, y=0.0)
    obstacle = (Obstacle("block", Pose2(x=0.15, y=0.0)),)
    assert not swept_clear(
        a, b, members=(Pose2(x=0.0, y=0.0),), footprint=footprint, obstacles=obstacle
    )
    obstacle = (Obstacle("block", Pose2(x=0.15, y=0.1)),)
    assert swept_clear(
        a, b, members=(Pose2(x=0.0, y=0.0),), footprint=footprint, obstacles=obstacle
    )
    assert not swept_clear(
        a,
        b,
        members=(Pose2(x=0.0, y=0.0), Pose2(x=0.0, y=0.1)),
        footprint=footprint,
        obstacles=obstacle,
    )


def test_generated_route_avoids_obstacle_and_reaches_heading() -> None:
    footprint = Footprint(half_x=0.025, half_y=0.025, center_x=0.0, margin=0.002)
    obstacles = (Obstacle("block", Pose2(x=0.15, y=0.0)),)
    path = plan_route(
        Pose2(x=0.0, y=0.0),
        Pose2(x=0.3, y=0.0, yaw=math.pi / 2),
        footprint=footprint,
        obstacles=obstacles,
    )
    assert path[-1].pose == Pose2(x=0.3, y=0.0, yaw=math.pi / 2)
    assert any(abs(point.pose.y) > 0.05 for point in path)
    for a, b in itertools.pairwise(path):
        assert b.time_s > a.time_s
        assert swept_clear(
            a.pose, b.pose, members=(Pose2(x=0.0, y=0.0),), footprint=footprint, obstacles=obstacles
        )


def test_reservation_blocks_crossing_only_during_occupancy() -> None:
    footprint = Footprint(half_x=0.02, half_y=0.02, center_x=0.0, margin=0.0)
    reservation = Reservation(
        "crossing",
        (
            TimedPose(pose=Pose2(x=0.1, y=-0.1), time_s=0.0),
            TimedPose(pose=Pose2(x=0.1, y=0.1), time_s=2.0),
        ),
    )
    for start_s, end_s, expected in ((0.0, 2.0, False), (3.0, 5.0, True)):
        assert (
            swept_clear(
                Pose2(x=0.0, y=0.0),
                Pose2(x=0.2, y=0.0),
                members=(Pose2(x=0.0, y=0.0),),
                footprint=footprint,
                obstacles=(),
                reservations=(reservation,),
                start_s=start_s,
                end_s=end_s,
            )
            is expected
        )


def test_no_route_is_a_bounded_failure() -> None:
    with pytest.raises(RouteUnavailable):
        plan_route(Pose2(x=0.0, y=0.0), Pose2(x=2.0, y=0.0), config=RouteConfig(max_expansions=1))


def test_fixed_mapping_preserves_orientation_as_well_as_connectors() -> None:
    edge = GoalEdge(a="a", face_a="bottom", b="b", face_b="pan", orientation_rad=0.0)
    goal = AssemblyGoal(id="orientation", nodes=("a", "b"), edges=(edge,))
    poses = {"a": Pose2(x=0.0, y=0.0), "b": Pose2(x=-0.08, y=0.0)}
    mapping = {"a": "a", "b": "b"}
    incorrect = edge.model_copy(update={"orientation_rad": math.pi / 2})
    plan = plan_assembly(goal, poses, fixed_mapping=mapping, existing=(incorrect,))
    assert plan.releases == (incorrect,)
    assert len(plan.actions) == 1
    preserved = plan_assembly(goal, poses, fixed_mapping=mapping, existing=(edge,))
    assert preserved.releases == preserved.actions == ()
