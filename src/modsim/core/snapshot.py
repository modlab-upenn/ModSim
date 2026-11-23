"""The read-only state contract a backend returns to ModSim each step.

A snapshot carries only what modular-robot semantics need: where links are, how
fast they are moving, optional connector frame poses, and optional constraint
wrenches. It deliberately does not carry contacts, solver internals, or renderer
state, because ModSim does not reason about those.

Timing contract (see ``docs/backends.md``): every pose, twist, joint state and
connector frame in one snapshot describes the single instant ``time_s``.
Constraint wrenches are interval quantities: they are what the solver applied
during the step that ended at ``time_s``.
"""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
from typing import TypeVar

from modsim.core.ids import ConstraintHandle, ModuleInstanceId
from modsim.core.transforms import (
    ZERO_VEC3,
    Transform,
    Vec3,
    quat_conjugate,
    quat_rotate,
    vec_add,
    vec_cross,
    vec_norm,
    vec_sub,
)

_K = TypeVar("_K")
_V = TypeVar("_V")


class FrozenMapping(Mapping[_K, _V]):
    """Marker base for read-only mappings that a backend promises never to mutate.

    A backend may return lazy, array-backed views instead of plain dictionaries
    so that a snapshot costs one bulk copy of engine arrays rather than one
    Python object per body, joint, and connector. ``WorldState`` keeps such
    mappings by reference instead of copying them. Values must still be the
    immutable types declared below, and must be the same on every lookup.
    """

    __slots__ = ()


class LinkPoseView(FrozenMapping[str, Transform]):
    """Link name to pose, read lazily from one module's ``BodyState`` mapping."""

    __slots__ = ("_links",)

    def __init__(self, links: Mapping[str, BodyState]) -> None:
        self._links = links

    def __getitem__(self, link: str) -> Transform:
        return self._links[link].pose

    def __iter__(self) -> Iterator[str]:
        return iter(self._links)

    def __len__(self) -> int:
        return len(self._links)


@dataclass(frozen=True, slots=True)
class BodyState:
    """World pose and twist of one rigid link."""

    pose: Transform = field(default_factory=Transform.identity)
    linear_velocity_m_s: Vec3 = ZERO_VEC3
    angular_velocity_rad_s: Vec3 = ZERO_VEC3

    def velocity_at(self, world_point: Vec3) -> Vec3:
        """Return the world velocity of a point rigidly attached to this link.

        A connector sits away from its link origin, so its velocity includes the
        ``omega x r`` term. Dropping that term makes a spinning module look
        stationary at its connectors and lets docks commit that should not.
        """
        lever = vec_sub(world_point, self.pose.translation)
        return vec_add(self.linear_velocity_m_s, vec_cross(self.angular_velocity_rad_s, lever))


@dataclass(frozen=True, slots=True)
class JointState:
    """Measured state of one actuated or passive joint."""

    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0


@dataclass(frozen=True, slots=True)
class ConstraintWrench:
    """Force and moment one physical connection transmits, as seen by module A.

    Both vectors are in the world frame. ``force_n`` is the force the connection
    applies to module A's connected link; ``torque_nm`` is the moment it applies
    to that link, taken about connector A's origin. Module B receives the equal
    and opposite wrench about the same point. "Module A" is the ``module_a`` of
    the :class:`~modsim.backends.base.ConnectionRequest` that created the
    connection.

    ``torque_nm`` is ``None`` when a backend measures force but not moment.
    """

    force_n: Vec3 = ZERO_VEC3
    torque_nm: Vec3 | None = ZERO_VEC3

    @property
    def force_magnitude_n(self) -> float:
        """Return the magnitude of the transmitted force in newtons."""
        return vec_norm(self.force_n)

    @property
    def torque_magnitude_nm(self) -> float | None:
        """Return the magnitude of the transmitted moment, or ``None`` if unmeasured."""
        return None if self.torque_nm is None else vec_norm(self.torque_nm)

    def resolved_in(self, frame: Transform) -> ConstraintWrench:
        """Return the same wrench with its vectors expressed in ``frame``'s axes.

        Only the axes change: the moment keeps connector A's origin as its
        reference point. Passing connector A's world frame gives the components
        a connector datasheet uses (normal and shear force, twist and bending
        moment about the connector's own axes).
        """
        inverse = quat_conjugate(frame.rotation)
        return ConstraintWrench(
            force_n=quat_rotate(inverse, self.force_n),
            torque_nm=None if self.torque_nm is None else quat_rotate(inverse, self.torque_nm),
        )


class ForceMagnitudeView(FrozenMapping[ConstraintHandle, float]):
    """Handle to force magnitude, read lazily from a wrench mapping."""

    __slots__ = ("_wrenches",)

    def __init__(self, wrenches: Mapping[ConstraintHandle, ConstraintWrench]) -> None:
        self._wrenches = wrenches

    def __getitem__(self, handle: ConstraintHandle) -> float:
        return self._wrenches[handle].force_magnitude_n

    def __iter__(self) -> Iterator[ConstraintHandle]:
        return iter(self._wrenches)

    def __len__(self) -> int:
        return len(self._wrenches)


@dataclass(frozen=True, slots=True)
class BackendStateSnapshot:
    """One immutable observation of backend state."""

    time_s: float = 0.0
    link_states: Mapping[ModuleInstanceId, Mapping[str, BodyState]] = field(
        default_factory=dict[ModuleInstanceId, Mapping[str, BodyState]]
    )
    joint_states: Mapping[ModuleInstanceId, Mapping[str, JointState]] = field(
        default_factory=dict[ModuleInstanceId, Mapping[str, JointState]]
    )
    connector_frames: Mapping[ModuleInstanceId, Mapping[str, Transform]] = field(
        default_factory=dict[ModuleInstanceId, Mapping[str, Transform]]
    )
    """Optional directly measured connector frames.

    A backend that materialises connector frames natively (MuJoCo sites, USD
    prims) should report them here. They take precedence over composing the
    parent link pose with the authored local pose, which avoids ModSim and the
    backend disagreeing about where a connector is.
    """

    constraint_wrenches: Mapping[ConstraintHandle, ConstraintWrench] = field(
        default_factory=dict[ConstraintHandle, ConstraintWrench]
    )
    """Wrench each active connection transmitted during the last step, where supported.

    A backend reports only connections that still exist when the snapshot is
    taken. See :class:`ConstraintWrench` for the frame and sign convention.
    """

    @property
    def constraint_forces_n(self) -> Mapping[ConstraintHandle, float]:
        """Return the force magnitude of every reported connection, in newtons."""
        return ForceMagnitudeView(self.constraint_wrenches)

    def body(self, module_id: ModuleInstanceId, link: str) -> BodyState | None:
        """Return the state of one link, or ``None`` when the backend omits it."""
        return self.link_states.get(module_id, {}).get(link)
