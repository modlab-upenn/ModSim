"""Liu et al. parallel planar assembly, with explicit fixed-mapping reconfiguration.

Reference: https://arxiv.org/abs/2104.00800, sections 4--6.
Navigation and online recovery are separate ModSim policies, not paper guarantees.
"""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Mapping, Sequence

from modsim.planning.models import (
    AssemblyAction,
    AssemblyGoal,
    AssemblyPlan,
    Assignment,
    Face,
    GoalEdge,
    Pose2,
)

NOMINAL_FACES: dict[Face, Pose2] = {
    "pan": Pose2(x=0.04, y=0.0),
    "bottom": Pose2(x=-0.04, y=0.0, yaw=math.pi),
    "left": Pose2(x=0.0, y=0.04, yaw=math.pi / 2),
    "right": Pose2(x=0.0, y=-0.04, yaw=-math.pi / 2),
}


def minimum_assignment(cost: Sequence[Sequence[float]]) -> tuple[int, ...]:
    """Deterministic O(n^3) Hungarian assignment; row -> column, no dependencies."""
    n = len(cost)
    if any(len(row) != n or any(not math.isfinite(v) for v in row) for row in cost):
        raise ValueError("assignment costs must be a finite square matrix")
    u, v = [0.0] * (n + 1), [0.0] * (n + 1)
    p, way = [0] * (n + 1), [0] * (n + 1)
    for i in range(1, n + 1):
        p[0], j0 = i, 0
        minimum, used = [math.inf] * (n + 1), [False] * (n + 1)
        while True:
            used[j0] = True
            i0, delta, j1 = p[j0], math.inf, 0
            for j in range(1, n + 1):
                if not used[j]:
                    current = cost[i0 - 1][j - 1] - u[i0] - v[j]
                    if current < minimum[j]:
                        minimum[j], way[j] = current, j0
                    if minimum[j] < delta:
                        delta, j1 = minimum[j], j
            for j in range(n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minimum[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0], j0 = p[j1], j1
    result = [0] * n
    for j in range(1, n + 1):
        result[p[j] - 1] = j - 1
    return tuple(result)


def tree_root(goal: AssemblyGoal) -> str:
    """Paper's branch-count root (tree centroid, not eccentricity center)."""
    adjacency: dict[str, list[str]] = {node: [] for node in goal.nodes}
    for edge in goal.edges:
        adjacency[edge.a].append(edge.b)
        adjacency[edge.b].append(edge.a)
    origin = min(goal.nodes)
    parent: dict[str, str | None] = {origin: None}
    order = [origin]
    for node in order:
        for neighbor in sorted(adjacency[node]):
            if neighbor != parent[node]:
                parent[neighbor] = node
                order.append(neighbor)
    sizes = dict.fromkeys(goal.nodes, 1)
    for node in reversed(order[1:]):
        ancestor = parent[node]
        assert ancestor is not None
        sizes[ancestor] += sizes[node]
    return min(
        node
        for node in goal.nodes
        if max(
            [len(goal.nodes) - sizes[node]]
            + [sizes[child] for child in adjacency[node] if parent.get(child) == node]
        )
        <= len(goal.nodes) / 2
    )


def plan_assembly(
    goal: AssemblyGoal,
    poses: Mapping[str, Pose2],
    *,
    faces: Mapping[Face, Pose2] = NOMINAL_FACES,
    fixed_mapping: Mapping[str, str] | None = None,
    existing: tuple[GoalEdge, ...] = (),
) -> AssemblyPlan:
    """Generate assignments and depth barriers; preserve already-correct bonds.

    Existing edges name physical modules. A mapping is required for reconfiguration;
    this deliberately does not claim the 2019 common-subconfiguration algorithm.
    """
    if len(poses) != len(goal.nodes) or not poses:
        raise ValueError("one physical module is required per goal node")
    if existing and fixed_mapping is None:
        raise ValueError("connected starts require an explicit module-to-goal mapping")
    root = tree_root(goal)
    adjacency: dict[str, list[tuple[str, Face, Face, float | None]]] = {n: [] for n in goal.nodes}
    for edge in goal.edges:
        adjacency[edge.a].append((edge.b, edge.face_a, edge.face_b, edge.orientation_rad))
        adjacency[edge.b].append(
            (
                edge.a,
                edge.face_b,
                edge.face_a,
                None if edge.orientation_rad is None else -edge.orientation_rad,
            )
        )
    targets = {root: Pose2(x=0.0, y=0.0)}
    depths = {root: 0}
    parents: dict[str, tuple[str, Face, Face, float | None]] = {}
    pending = deque([root])
    while pending:
        parent = pending.popleft()
        for child, parent_face, child_face, orientation in sorted(adjacency[parent]):
            if child in targets:
                continue
            pf, cf = faces[parent_face], faces[child_face]
            yaw = pf.yaw + math.pi - cf.yaw
            rotated = Pose2(x=0.0, y=0.0, yaw=yaw).compose(cf)
            targets[child] = targets[parent].compose(
                Pose2(x=pf.x - rotated.x, y=pf.y - rotated.y, yaw=yaw)
            )
            depths[child] = depths[parent] + 1
            parents[child] = (parent, parent_face, child_face, orientation)
            pending.append(child)
    target_list = list(targets.values())
    if any(a.distance(b) < 1e-5 for i, a in enumerate(target_list) for b in target_list[i + 1 :]):
        raise ValueError("target topology overlaps when unfolded onto the plane")
    centroid = Pose2(
        x=sum(p.x for p in poses.values()) / len(poses),
        y=sum(p.y for p in poses.values()) / len(poses),
    )
    if fixed_mapping is None:
        root_module = min(poses, key=lambda m: (poses[m].distance(centroid), m))
        mapping = {root: root_module}
        nodes, modules = sorted(set(goal.nodes) - {root}), sorted(set(poses) - {root_module})
        costs = [
            [poses[m].distance(poses[root_module].compose(targets[n])) for m in modules]
            for n in nodes
        ]
        mapping.update(
            {n: modules[j] for n, j in zip(nodes, minimum_assignment(costs), strict=True)}
        )
    else:
        mapping = dict(fixed_mapping)
        if set(mapping) != set(goal.nodes) or set(mapping.values()) != set(poses):
            raise ValueError("fixed mapping must be a bijection from goal nodes to modules")
        root_module = mapping[root]
    assignments = tuple(
        Assignment(goal_node=n, module_id=mapping[n], target=poses[root_module].compose(targets[n]))
        for n in sorted(targets)
    )
    existing_by_key = {edge.key: edge for edge in existing}
    preserved_keys: set[tuple[str, str]] = set()
    actions: list[AssemblyAction] = []
    for child in sorted(parents, key=lambda n: (depths[n], n)):
        parent, parent_face, child_face, orientation = parents[child]
        action = AssemblyAction(
            id=f"dock_{child}",
            moving=mapping[child],
            moving_face=child_face,
            parent=mapping[parent],
            parent_face=parent_face,
            depth=depths[child],
            batch=2 * depths[child] + int(depths[child] == 1 and parent_face in {"pan", "bottom"}),
            orientation_rad=orientation,
        )
        old = existing_by_key.get(action.edge.key)
        matching_orientation = action.orientation_rad is None
        if (
            old is not None
            and old.orientation_rad is not None
            and action.orientation_rad is not None
        ):
            old_angle = old.orientation_rad if old.a == action.parent else -old.orientation_rad
            delta = old_angle - action.orientation_rad
            matching_orientation = abs(math.atan2(math.sin(delta), math.cos(delta))) < 1e-6
        if old is not None and matching_orientation:
            preserved_keys.add(action.edge.key)
        else:
            if child_face in {"left", "right"}:
                raise ValueError(f"{action.id} requires a helping module for wheel-face docking")
            actions.append(action)
    actions = [
        a.model_copy(update={"dependencies": tuple(b.id for b in actions if b.batch < a.batch)})
        for a in actions
    ]
    return AssemblyPlan(
        goal=goal,
        root=root,
        root_module=root_module,
        assignments=assignments,
        actions=tuple(actions),
        releases=tuple(e for e in existing if e.key not in preserved_keys),
        assignment_cost_m=sum(poses[a.module_id].distance(a.target) for a in assignments),
    )
