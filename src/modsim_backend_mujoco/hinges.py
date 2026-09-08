"""Runtime hinge management over pre-allocated MuJoCo connect constraints.

MuJoCo cannot add equality constraints after compilation.  A ModSim hinge is
therefore represented by two reserved ``mjEQ_CONNECT`` equalities whose
anchors lie on the requested hinge line.  Fixing two separated points removes
translation and all rotation except rotation about the line through them.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from modsim.core.ids import ConstraintHandle
from modsim.core.transforms import (
    Transform,
    Vec3,
    vec_dot,
    vec_normalize,
    vec_scale,
)

CONNECT_ANCHOR_BODY_A = slice(0, 3)
CONNECT_ANCHOR_BODY_B = slice(3, 6)
CONNECTS_PER_HINGE = 2


class HingePoolExhaustedError(RuntimeError):
    """Raised when every reserved pair of connect constraints is occupied."""


@dataclass(frozen=True, slots=True)
class HingeAnchorPair:
    """Corresponding point anchors expressed in each constrained body."""

    body_a: Vec3
    body_b: Vec3


def hinge_anchor_pairs(
    body_a_world: Transform,
    connector_a_world: Transform,
    body_b_world: Transform,
    connector_b_world: Transform,
    axis_connector: Vec3,
    separation_m: float,
) -> tuple[HingeAnchorPair, HingeAnchorPair]:
    """Return two body-local point pairs defining a connector-centred hinge.

    ``axis_connector`` is interpreted in each connector's own frame.  Mating
    connector frames commonly point in opposite directions, so endpoints on
    side B are paired in whichever order makes the two measured world axes
    agree.  Each side remains centred on its own measured connector origin;
    any small accepted position or orientation error is then resolved by the
    equality solver instead of being hidden by backend geometry.
    """
    if separation_m <= 0.0:
        raise ValueError("hinge anchor separation must be positive")

    axis = vec_normalize(axis_connector)
    axis_a_world = vec_normalize(connector_a_world.apply_direction(axis))
    axis_b_world = vec_normalize(connector_b_world.apply_direction(axis))
    side_b = 1.0 if vec_dot(axis_a_world, axis_b_world) >= 0.0 else -1.0
    half = separation_m * 0.5
    world_to_a = body_a_world.inverse()
    world_to_b = body_b_world.inverse()

    pairs: list[HingeAnchorPair] = []
    for side_a in (-1.0, 1.0):
        endpoint_a = connector_a_world.apply_point(vec_scale(axis, side_a * half))
        endpoint_b = connector_b_world.apply_point(vec_scale(axis, side_a * side_b * half))
        pairs.append(
            HingeAnchorPair(
                body_a=world_to_a.apply_point(endpoint_a),
                body_b=world_to_b.apply_point(endpoint_b),
            )
        )
    return (pairs[0], pairs[1])


@dataclass(slots=True)
class HingeSlot:
    """Two reserved connect equalities and their current owner."""

    equality_ids: tuple[int, int]
    handle: ConstraintHandle | None = None

    @property
    def free(self) -> bool:
        """Whether this slot can be claimed."""
        return self.handle is None


@dataclass(slots=True)
class HingePool:
    """Allocator over pairs of equality constraints reserved for hinges."""

    slots: tuple[HingeSlot, ...] = ()
    _by_handle: dict[ConstraintHandle, HingeSlot] = field(
        default_factory=dict[ConstraintHandle, HingeSlot]
    )

    @classmethod
    def over(cls, equality_pairs: tuple[tuple[int, int], ...]) -> HingePool:
        """Build a pool over reserved pairs of connect equality ids."""
        return cls(slots=tuple(HingeSlot(pair) for pair in equality_pairs))

    @property
    def capacity(self) -> int:
        """Total number of hinges that can be active at once."""
        return len(self.slots)

    @property
    def in_use(self) -> int:
        """Number of slots currently holding a hinge."""
        return len(self._by_handle)

    @property
    def available(self) -> int:
        """Number of hinges that can still be created."""
        return self.capacity - self.in_use

    def claim(self, handle: ConstraintHandle) -> HingeSlot:
        """Reserve the lowest free pair for ``handle``."""
        if handle in self._by_handle:
            raise HingePoolExhaustedError(f"constraint '{handle}' already holds a hinge slot")
        for slot in self.slots:
            if slot.free:
                slot.handle = handle
                self._by_handle[handle] = slot
                return slot
        raise HingePoolExhaustedError(
            f"all {self.capacity} reserved hinge slots are in use; "
            "raise hinge_pool_size when constructing the backend"
        )

    def release(self, handle: ConstraintHandle) -> HingeSlot | None:
        """Free the slot held by ``handle``, or return ``None``."""
        slot = self._by_handle.pop(handle, None)
        if slot is None:
            return None
        slot.handle = None
        return slot

    def slot_for(self, handle: ConstraintHandle) -> HingeSlot | None:
        """Return the slot held by ``handle``, if any."""
        return self._by_handle.get(handle)

    def clear(self) -> None:
        """Release every slot."""
        for slot in self.slots:
            slot.handle = None
        self._by_handle.clear()
