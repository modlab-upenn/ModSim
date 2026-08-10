"""Runtime weld management over a pre-allocated equality-constraint pool.

MuJoCo fixes model topology at compile time, so docking cannot create a
constraint. It can only claim one of the inactive welds reserved when the scene
was compiled, point it at a new pair of bodies, and activate it.

The transform conversion lives here rather than in the adapter because it is
pure geometry and worth testing without a simulation running.
"""

from __future__ import annotations

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
        for slot in self.slots:
            if slot.free:
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
        return slot

    def slot_for(self, handle: ConstraintHandle) -> WeldSlot | None:
        """Return the slot held by ``handle``, if any."""
        return self._by_handle.get(handle)

    def clear(self) -> None:
        """Release every slot."""
        for slot in self.slots:
            slot.handle = None
        self._by_handle.clear()
