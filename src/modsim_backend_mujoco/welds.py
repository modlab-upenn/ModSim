"""Runtime weld management over a pre-allocated equality-constraint pool.

MuJoCo fixes model topology at compile time, so docking cannot create a
constraint. It can only claim one of the inactive welds reserved when the scene
was compiled, point it at a new pair of bodies, and activate it.

The transform conversion lives here rather than in the adapter because it is
pure geometry and worth testing without a simulation running.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from modsim.core.ids import ConstraintHandle
from modsim.core.transforms import Transform

EQ_DATA_WIDTH = 11
"""``mjNEQDATA``: anchor (3), relpose position (3), relpose quaternion (4), torquescale (1)."""

ANCHOR = slice(0, 3)
RELPOSE_POSITION = slice(3, 6)
RELPOSE_ROTATION = slice(6, 10)
TORQUE_SCALE = 10

RIGID_TORQUE_SCALE = 1.0
"""Full rotational coupling. Zero would make the weld behave like a ball joint."""


class WeldPoolExhaustedError(RuntimeError):
    """Raised when every reserved weld slot is already in use."""


class ContactExclusionPoolExhaustedError(RuntimeError):
    """Raised when every reserved collision-exclusion slot is in use."""


def collision_signature(body_a: int, body_b: int) -> int:
    """Return MuJoCo's canonical collision-exclusion body-pair signature."""
    if body_a == body_b:
        raise ValueError("a contact exclusion requires two different bodies")
    first, second = sorted((body_a, body_b))
    if first < 0 or second >= 1 << 16:
        raise ValueError("MuJoCo contact exclusions require 16-bit non-negative body ids")
    return (first << 16) | second


def body_relative_transform(
    connector_a_local: Transform,
    connector_b_local: Transform,
    connector_relative: Transform,
) -> Transform:
    """Convert a connector-frame mate into the body-frame pose a weld needs.

    ModSim commits a relative transform between two *connector* frames, but a
    MuJoCo weld constrains two *bodies*. Composing outward from connector A to
    its body, across the mate, then inward from connector B to its body gives
    the pose of body B expressed in body A's frame::

        T_bodyA_bodyB = T_bodyA_connA.T_connA_connB.T_connB_bodyB

    ``connector_a_local`` and ``connector_b_local`` are each the pose of a
    connector in its own body's frame, so the last term is an inverse.
    """
    return connector_a_local.compose(connector_relative).compose(connector_b_local.inverse())


class LowestFreeIndex:
    """Free slot positions in a min-heap: the lowest free slot in O(log P).

    Every pool hands out its lowest free slot, so which constraint a connection
    occupies is the same as with a linear scan; only the search cost changes.
    """

    __slots__ = ("_heap",)

    def __init__(self, count: int = 0) -> None:
        self._heap = list(range(count))  # an ascending list is already a heap

    def pop(self) -> int | None:
        return heapq.heappop(self._heap) if self._heap else None

    def push(self, index: int) -> None:
        heapq.heappush(self._heap, index)

    def reset(self, count: int) -> None:
        self._heap = list(range(count))


@dataclass(slots=True)
class WeldSlot:
    """One reserved equality constraint and what currently occupies it."""

    equality_id: int
    handle: ConstraintHandle | None = None

    @property
    def free(self) -> bool:
        """Whether this slot can be claimed."""
        return self.handle is None


@dataclass(slots=True)
class WeldPool:
    """Allocator over the equality constraints reserved at compile time."""

    slots: tuple[WeldSlot, ...] = ()
    _by_handle: dict[ConstraintHandle, WeldSlot] = field(
        default_factory=dict[ConstraintHandle, WeldSlot]
    )
    _free: LowestFreeIndex = field(init=False)
    _position: dict[int, int] = field(init=False)

    def __post_init__(self) -> None:
        self._position = {id(slot): index for index, slot in enumerate(self.slots)}
        self._free = LowestFreeIndex(len(self.slots))

    @classmethod
    def over(cls, equality_ids: tuple[int, ...]) -> WeldPool:
        """Build a pool over the given reserved equality constraint ids."""
        return cls(slots=tuple(WeldSlot(equality_id=identifier) for identifier in equality_ids))

    @property
    def capacity(self) -> int:
        """Total number of reserved slots."""
        return len(self.slots)

    @property
    def in_use(self) -> int:
        """Number of slots currently holding a connection."""
        return len(self._by_handle)

    @property
    def available(self) -> int:
        """Number of slots that can still be claimed."""
        return self.capacity - self.in_use

    def claim(self, handle: ConstraintHandle) -> WeldSlot:
        """Reserve the lowest free slot for ``handle``."""
        if handle in self._by_handle:
            raise WeldPoolExhaustedError(f"constraint '{handle}' already holds a weld slot")
        index = self._free.pop()
        if index is not None:
            slot = self.slots[index]
            slot.handle = handle
            self._by_handle[handle] = slot
            return slot
        raise WeldPoolExhaustedError(
            f"all {self.capacity} reserved weld slots are in use; "
            "raise weld_pool_size when constructing the backend"
        )

    def release(self, handle: ConstraintHandle) -> WeldSlot | None:
        """Free the slot held by ``handle``, or return ``None`` if it holds none."""
        slot = self._by_handle.pop(handle, None)
        if slot is None:
            return None
        slot.handle = None
        self._free.push(self._position[id(slot)])
        return slot

    def slot_for(self, handle: ConstraintHandle) -> WeldSlot | None:
        """Return the slot held by ``handle``, if any."""
        return self._by_handle.get(handle)

    def clear(self) -> None:
        """Release every slot."""
        for slot in self.slots:
            slot.handle = None
        self._by_handle.clear()
        self._free.reset(len(self.slots))


@dataclass(slots=True)
class ContactExclusionSlot:
    """One reserved exclusion entry and the body pair currently occupying it."""

    reservation_id: int
    handle: ConstraintHandle | None = None
    signature: int = 0

    @property
    def free(self) -> bool:
        """Whether this slot can be claimed."""
        return self.handle is None


@dataclass(slots=True)
class ContactExclusionPool:
    """Allocator over mutable collision-exclusion signatures.

    MuJoCo searches the model's exclusion signatures in sorted order. The pool
    owns connection-to-signature bookkeeping; the adapter rewrites and sorts
    the compiled array atomically after every claim or release.
    """

    slots: tuple[ContactExclusionSlot, ...] = ()
    _by_handle: dict[ConstraintHandle, ContactExclusionSlot] = field(
        default_factory=dict[ConstraintHandle, ContactExclusionSlot]
    )
    _free: LowestFreeIndex = field(init=False)
    _position: dict[int, int] = field(init=False)

    def __post_init__(self) -> None:
        self._position = {id(slot): index for index, slot in enumerate(self.slots)}
        self._free = LowestFreeIndex(len(self.slots))

    @classmethod
    def over(cls, reservation_ids: tuple[int, ...]) -> ContactExclusionPool:
        """Build a pool over the given precompiled exclusion reservations."""
        return cls(
            slots=tuple(
                ContactExclusionSlot(reservation_id=identifier) for identifier in reservation_ids
            )
        )

    @property
    def capacity(self) -> int:
        """Total number of reserved entries."""
        return len(self.slots)

    @property
    def in_use(self) -> int:
        """Number of entries currently suppressing a body pair."""
        return len(self._by_handle)

    @property
    def active_signatures(self) -> tuple[int, ...]:
        """Return occupied signatures, ready for sorting into the model."""
        return tuple(slot.signature for slot in self.slots if not slot.free)

    def claim(
        self,
        handle: ConstraintHandle,
        body_a: int,
        body_b: int,
    ) -> ContactExclusionSlot:
        """Reserve the lowest free entry for one welded body pair."""
        if handle in self._by_handle:
            raise ContactExclusionPoolExhaustedError(
                f"constraint '{handle}' already holds a contact-exclusion slot"
            )
        signature = collision_signature(body_a, body_b)
        index = self._free.pop()
        if index is not None:
            slot = self.slots[index]
            slot.handle = handle
            slot.signature = signature
            self._by_handle[handle] = slot
            return slot
        raise ContactExclusionPoolExhaustedError(
            f"all {self.capacity} reserved contact-exclusion slots are in use; "
            "raise weld_pool_size when constructing the backend"
        )

    def release(self, handle: ConstraintHandle) -> ContactExclusionSlot | None:
        """Free the entry held by ``handle``, if any."""
        slot = self._by_handle.pop(handle, None)
        if slot is None:
            return None
        slot.handle = None
        slot.signature = 0
        self._free.push(self._position[id(slot)])
        return slot

    def signature_of(self, handle: ConstraintHandle) -> int | None:
        """Return the body-pair signature ``handle`` currently suppresses, if any."""
        slot = self._by_handle.get(handle)
        return None if slot is None else slot.signature

    def clear(self) -> None:
        """Release every entry."""
        for slot in self.slots:
            slot.handle = None
            slot.signature = 0
        self._by_handle.clear()
        self._free.reset(len(self.slots))
