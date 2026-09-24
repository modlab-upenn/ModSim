"""Bounded support-mode and joint-space search for spatial assembly experiments.

Support here means a graph path to a declared fixture, NOT force equilibrium.
Motion feasibility is supplied by an oracle. Neither search proves arbitrary
3D reconfigurability; see docs/smores_spatial_planning.md for the assumptions.
"""

from __future__ import annotations

import heapq
import math
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from itertools import count


class SpatialPlanningError(ValueError):
    """Invalid, infeasible, or budget-exhausted spatial planning problem."""


@dataclass(frozen=True, order=True, slots=True)
class Bond:
    """Two occupied connector resources, expressed as module/connector IDs."""

    a: str
    b: str

    def __post_init__(self) -> None:
        if any(len(c.split("/")) != 2 or not all(c.split("/")) for c in (self.a, self.b)):
            raise SpatialPlanningError("bonds require module/connector identifiers")
        if self.modules[0] == self.modules[1]:
            raise SpatialPlanningError("a bond must join different modules")
        if self.a > self.b:
            a, b = self.a, self.b
            object.__setattr__(self, "a", b)
            object.__setattr__(self, "b", a)

    @property
    def modules(self) -> tuple[str, str]:
        return self.a.split("/")[0], self.b.split("/")[0]


@dataclass(frozen=True, slots=True)
class TopologyAction:
    operation: str
    bond: Bond


def supported(modules: frozenset[str], bonds: frozenset[Bond], anchors: frozenset[str]) -> bool:
    """Check connector exclusivity and a fixture path for every module."""
    occupied: set[str] = set()
    adjacency = {module: set[str]() for module in modules}
    if not anchors or not anchors <= modules:
        return False
    for bond in bonds:
        a, b = bond.modules
        if a not in modules or b not in modules or occupied.intersection((bond.a, bond.b)):
            return False
        occupied.update((bond.a, bond.b))
        adjacency[a].add(b)
        adjacency[b].add(a)
    reached = set(anchors)
    queue = list(anchors)
    while queue:
        for neighbor in adjacency[queue.pop()] - reached:
            reached.add(neighbor)
            queue.append(neighbor)
    return frozenset(reached) == modules


def plan_support_changes(
    modules: frozenset[str],
    initial: frozenset[Bond],
    goal: frozenset[Bond],
    anchors: frozenset[str],
    *,
    max_states: int = 4096,
) -> tuple[TopologyAction, ...]:
    """Find a shortest supported sequence using bonds in initial union goal.

    Every intermediate mode is checked, including the moment after a release.
    Geometric docking and load feasibility must still be checked by the executor.
    """
    if max_states < 1:
        raise SpatialPlanningError("max_states must be positive")
    if not supported(modules, initial, anchors) or not supported(modules, goal, anchors):
        raise SpatialPlanningError("initial and target modes must have exclusive, supported bonds")
    queue: deque[tuple[frozenset[Bond], tuple[TopologyAction, ...]]] = deque([(initial, ())])
    seen = {initial}
    while queue:
        state, actions = queue.popleft()
        if state == goal:
            return actions
        for bond in sorted(initial | goal):
            candidate = state ^ {bond}
            if candidate in seen or not supported(modules, candidate, anchors):
                continue
            if len(seen) >= max_states:
                raise SpatialPlanningError("support search exhausted its state budget")
            seen.add(candidate)
            action = TopologyAction("release" if bond in state else "dock", bond)
            queue.append((candidate, (*actions, action)))
    raise SpatialPlanningError("no supported sequence exists in the supplied bond set")


JointPoint = tuple[float, ...]
GridPoint = tuple[int, ...]


@dataclass(frozen=True, slots=True)
class JointPath:
    points: tuple[JointPoint, ...]
    expanded: int
    rejected: int


def plan_joint_path(
    start: GridPoint,
    goal: GridPoint,
    upper: GridPoint,
    feasible: Callable[[JointPoint], bool],
    *,
    subdivisions: int = 5,
    max_states: int = 4096,
) -> JointPath:
    """A* on a bounded joint grid with sampled validation along EVERY edge.

    Coordinates are grid units; the caller maps them to physical joint angles.
    Single-axis edges and an L1 heuristic give a shortest grid path if search
    finishes. Sampling is not a continuous collision certificate.
    """
    if (
        not start
        or len(start) != len(goal)
        or len(start) != len(upper)
        or subdivisions < 1
        or max_states < 1
        or any(type(x) is not int for p in (start, goal, upper) for x in p)
        or any(
            not 0 <= a <= c or not 0 <= b <= c for a, b, c in zip(start, goal, upper, strict=False)
        )
    ):
        raise SpatialPlanningError("invalid bounded joint grid")
    if not feasible(tuple(map(float, start))) or not feasible(tuple(map(float, goal))):
        raise SpatialPlanningError("start or goal fails the motion feasibility oracle")
    serial = count()
    frontier: list[tuple[int, int, GridPoint]] = [(0, next(serial), start)]
    costs = {start: 0}
    parents: dict[GridPoint, GridPoint] = {}
    closed: set[GridPoint] = set()
    rejected = 0
    while frontier:
        _, _, point = heapq.heappop(frontier)
        if point in closed:
            continue
        if len(closed) >= max_states:
            raise SpatialPlanningError("joint search exhausted its state budget")
        closed.add(point)
        if point == goal:
            path = [point]
            while path[-1] != start:
                path.append(parents[path[-1]])
            return JointPath(
                tuple(tuple(map(float, p)) for p in reversed(path)), len(closed), rejected
            )
        for axis in range(len(point)):
            for sign in (1, -1):
                values = list(point)
                values[axis] += sign
                neighbor = tuple(values)
                if not 0 <= values[axis] <= upper[axis] or neighbor in closed:
                    continue
                cost = costs[point] + 1
                if cost >= costs.get(neighbor, math.inf):
                    continue
                if not all(
                    feasible(
                        tuple(
                            a + (b - a) * k / subdivisions
                            for a, b in zip(point, neighbor, strict=False)
                        )
                    )
                    for k in range(1, subdivisions + 1)
                ):
                    rejected += 1
                    continue
                costs[neighbor] = cost
                parents[neighbor] = point
                heuristic = sum(abs(a - b) for a, b in zip(neighbor, goal, strict=False))
                heapq.heappush(frontier, (cost + heuristic, next(serial), neighbor))
    raise SpatialPlanningError("no feasible joint path exists on this grid")
