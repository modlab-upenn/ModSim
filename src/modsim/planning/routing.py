"""Bounded space-time A* with oriented assembly footprints and swept checks.

Reservations are predictions, not a physics guarantee. The runtime checks measured
clearance and renews reservations before following further route segments.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from itertools import count, pairwise
from typing import Literal, TypeAlias, cast

from modsim.planning.models import Pose2, TimedPose

State: TypeAlias = tuple[int, int, int, int]


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def interpolate(a: Pose2, b: Pose2, fraction: float) -> Pose2:
    return Pose2(
        x=a.x + (b.x - a.x) * fraction,
        y=a.y + (b.y - a.y) * fraction,
        yaw=a.yaw + angle_delta(b.yaw, a.yaw) * fraction,
    )


def interpolate_about_axle(a: Pose2, b: Pose2, fraction: float, offset: float) -> Pose2:
    center = Pose2(x=offset, y=0.0)
    return interpolate(a.compose(center), b.compose(center), fraction).compose(
        Pose2(x=-offset, y=0.0)
    )


@dataclass(frozen=True)
class Footprint:
    half_x: float = 0.044
    half_y: float = 0.044
    center_x: float = 0.033
    margin: float = 0.002

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(v) for v in (self.half_x, self.half_y, self.center_x, self.margin)
        ):
            raise ValueError("footprint dimensions must be finite")
        if min(self.half_x, self.half_y) <= 0 or self.margin < 0:
            raise ValueError("footprint extents must be positive and margin nonnegative")


@dataclass(frozen=True)
class Obstacle:
    id: str
    pose: Pose2


@dataclass(frozen=True)
class Reservation:
    id: str
    path: tuple[TimedPose, ...]
    members: tuple[Pose2, ...] = (Pose2(x=0.0, y=0.0),)

    def pose_at(self, time_s: float) -> Pose2:
        if not self.path:
            raise ValueError("a reservation needs a path")
        for a, b in zip(self.path, self.path[1:], strict=False):
            if time_s <= b.time_s:
                fraction = max(0.0, (time_s - a.time_s) / max(1e-9, b.time_s - a.time_s))
                return interpolate(a.pose, b.pose, min(1.0, fraction))
        return self.path[-1].pose


def penetration(a: Pose2, b: Pose2, footprint: Footprint) -> float:
    """Signed rectangle penetration via separating axes; negative means clear."""
    ca, sa, cb, sb = math.cos(a.yaw), math.sin(a.yaw), math.cos(b.yaw), math.sin(b.yaw)
    dx = b.x + cb * footprint.center_x - a.x - ca * footprint.center_x
    dy = b.y + sb * footprint.center_x - a.y - sa * footprint.center_x
    hx, hy = footprint.half_x + footprint.margin, footprint.half_y + footprint.margin
    return min(
        hx * abs(ca * x + sa * y)
        + hy * abs(-sa * x + ca * y)
        + hx * abs(cb * x + sb * y)
        + hy * abs(-sb * x + cb * y)
        - abs(dx * x + dy * y)
        for x, y in ((ca, sa), (-sa, ca), (cb, sb), (-sb, cb))
    )


def swept_clear(
    start: Pose2,
    end: Pose2,
    *,
    members: tuple[Pose2, ...],
    footprint: Footprint,
    obstacles: tuple[Obstacle, ...],
    reservations: tuple[Reservation, ...] = (),
    start_s: float = 0.0,
    end_s: float = 1.0,
    departure: Pose2 | None = None,
    pivot_offset_m: float | None = None,
    allow_escape: bool = False,
) -> bool:
    """Check swept translation/rotation, including endpoints and timed occupancy.

    A just-released face can start inside the conservative padding (up to 6 mm).
    Departure may preserve or reduce that initial overlap, never deepen it.
    """
    radius = max((math.hypot(m.x, m.y) for m in members), default=0.0) + 0.08
    distance = start.distance(end) + radius * abs(angle_delta(end.yaw, start.yaw))
    steps = max(1, math.ceil(distance / 0.008), math.ceil((end_s - start_s) / 0.25))
    for index in range(steps + 1):
        fraction = index / steps
        pose = interpolate_about_axle(
            start, end, fraction, footprint.center_x if pivot_offset_m is None else pivot_offset_m
        )
        time_s = start_s + (end_s - start_s) * fraction
        for relative in members:
            body = pose.compose(relative)
            for obstacle in obstacles:
                allowance = 0.0
                if departure is not None:
                    initial = penetration(departure.compose(relative), obstacle.pose, footprint)
                    if initial > 0.0 and (initial <= 0.006 or allow_escape):
                        allowance = initial + 1e-6
                if penetration(body, obstacle.pose, footprint) > allowance:
                    return False
            for reservation in reservations:
                other = reservation.pose_at(time_s)
                if any(
                    penetration(body, other.compose(m), footprint) > 0 for m in reservation.members
                ):
                    return False
    return True


class RouteUnavailable(ValueError):
    """No route was found within the configured search bounds."""


@dataclass(frozen=True)
class RouteConfig:
    resolution_m: float = 0.05
    speed_m_s: float = 0.03
    yaw_rate_rad_s: float = 0.5
    max_expansions: int = 6000
    max_slots: int = 100

    def __post_init__(self) -> None:
        if any(
            not math.isfinite(v) or v <= 0
            for v in (self.resolution_m, self.speed_m_s, self.yaw_rate_rad_s)
        ):
            raise ValueError("route scales must be finite and positive")
        if self.max_expansions <= 0 or self.max_slots <= 0:
            raise ValueError("search bounds must be positive")


_DEFAULT_FOOTPRINT = Footprint()
_DEFAULT_ROUTE_CONFIG = RouteConfig()


def plan_route(
    start: Pose2,
    goal: Pose2,
    *,
    members: tuple[Pose2, ...] = (Pose2(x=0.0, y=0.0),),
    footprint: Footprint = _DEFAULT_FOOTPRINT,
    obstacles: tuple[Obstacle, ...] = (),
    reservations: tuple[Reservation, ...] = (),
    start_s: float = 0.0,
    config: RouteConfig = _DEFAULT_ROUTE_CONFIG,
    pivot_offset_m: float | None = None,
) -> tuple[TimedPose, ...]:
    """Search translation, in-place rotation and wait primitives with finite bounds."""
    step = config.resolution_m
    pivot = footprint.center_x if pivot_offset_m is None else pivot_offset_m
    axle = Pose2(x=pivot, y=0.0)
    to_root = Pose2(x=-pivot, y=0.0)
    reference = start.compose(axle)
    slot_s = max(step * math.sqrt(2) / config.speed_m_s, math.pi / 4 / config.yaw_rate_rad_s)
    serial = count()
    # Keys include time so waits can avoid moving reservations.
    heading_count = 16 if len(members) == 1 else 32
    heading_step = 2 * math.pi / heading_count
    first: State = (0, 0, 0, 0)
    poses: dict[State, Pose2] = {first: start}
    costs: dict[State, float] = {first: 0.0}
    best_spatial: dict[tuple[int, int, int], float] = {first[:3]: 0.0}
    previous: dict[State, tuple[State, int]] = {}
    queue: list[tuple[float, int, State]] = [(start.distance(goal) / step, next(serial), first)]
    for _ in range(config.max_expansions):
        if not queue:
            break
        _, _, state = heapq.heappop(queue)
        _, _, _, slot = state
        current = poses[state]
        t = start_s + slot * slot_s
        if current.distance(goal) <= step * 1.5:
            # Straight terminal segment followed by a checked in-place alignment.
            current_axle, goal_axle = current.compose(axle), goal.compose(axle)
            bearing = math.atan2(goal_axle.y - current_axle.y, goal_axle.x - current_axle.x)
            terminal_direction: Literal[-1, 1] = 1
            if abs(angle_delta(bearing, current.yaw)) > math.pi / 2:
                bearing += math.pi
                terminal_direction = -1
            terminal_heading = Pose2(x=current_axle.x, y=current_axle.y, yaw=bearing).compose(
                to_root
            )
            arrival = Pose2(x=goal_axle.x, y=goal_axle.y, yaw=bearing).compose(to_root)
            terminal = (current, terminal_heading, arrival, goal)
            modest_turn = len(members) == 1 or (
                abs(angle_delta(bearing, current.yaw)) < 0.3
                and abs(angle_delta(goal.yaw, bearing)) < 0.3
            )
            if modest_turn and all(
                swept_clear(
                    a,
                    b,
                    members=members,
                    footprint=footprint,
                    obstacles=obstacles,
                    reservations=reservations,
                    start_s=t + i * slot_s,
                    end_s=t + (i + 1) * slot_s,
                    departure=start,
                    pivot_offset_m=pivot,
                )
                for i, (a, b) in enumerate(pairwise(terminal))
            ):
                path = [
                    TimedPose(pose=goal, time_s=t + 3 * slot_s, direction=0),
                    TimedPose(pose=arrival, time_s=t + 2 * slot_s, direction=terminal_direction),
                    TimedPose(pose=terminal_heading, time_s=t + slot_s, direction=0),
                ]
                while state != first:
                    parent, direction = previous[state]
                    path.append(
                        TimedPose(
                            pose=poses[state],
                            time_s=start_s + state[3] * slot_s,
                            direction=cast(Literal[-1, 0, 1], direction),
                        )
                    )
                    state = parent
                path.append(TimedPose(pose=start, time_s=start_s))
                return tuple(reversed(path))
        if slot >= config.max_slots:
            continue
        center = current.compose(axle)
        candidates: list[tuple[Pose2, int]] = []
        for direction in (1, -1):
            for turn in (0, 1, -1):
                delta = turn * direction * heading_step
                distance = direction * step
                dx = distance if turn == 0 else distance * math.sin(delta) / delta
                dy = 0.0 if turn == 0 else distance * (1 - math.cos(delta)) / delta
                candidates.append(
                    (center.compose(Pose2(x=dx, y=dy, yaw=delta)).compose(to_root), direction)
                )
        # Long rigid differential-drive trains need rolling turns to overcome scrub.
        if len(members) == 1:
            for turn in (-1, 1):
                candidates.append(
                    (
                        center.compose(Pose2(x=0.0, y=0.0, yaw=turn * heading_step)).compose(
                            to_root
                        ),
                        0,
                    )
                )
        if reservations:
            candidates.append((current, 0))
        for target, direction in candidates:
            target_center = target.compose(axle)
            child: State = (
                round((target_center.x - reference.x) / (step / 2)),
                round((target_center.y - reference.y) / (step / 2)),
                round(angle_delta(target.yaw, start.yaw) / heading_step) % heading_count,
                slot + 1,
            )
            cost = costs[state] + 1.0 + (0.05 if direction < 0 else 0.0)
            if cost >= costs.get(child, math.inf):
                continue
            if not reservations and cost >= best_spatial.get(child[:3], math.inf):
                continue
            if not swept_clear(
                current,
                target,
                members=members,
                footprint=footprint,
                obstacles=obstacles,
                reservations=reservations,
                start_s=t,
                end_s=t + slot_s,
                departure=start,
                pivot_offset_m=pivot,
            ):
                continue
            costs[child], poses[child], previous[child] = cost, target, (state, direction)
            best_spatial[child[:3]] = cost
            heuristic = target.distance(goal) / (step * math.sqrt(2))
            heapq.heappush(queue, (cost + heuristic, next(serial), child))
    raise RouteUnavailable(
        f"no clear route within {config.max_expansions} expansions / {config.max_slots} slots"
    )
