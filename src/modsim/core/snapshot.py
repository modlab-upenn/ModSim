"""The read-only state contract a backend returns to ModSim each step.

A snapshot carries only what modular-robot semantics need: where links are, how
fast they are moving, optional connector frame poses, and optional constraint
forces. It deliberately does not carry contacts, solver internals, or renderer
state, because ModSim does not reason about those.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

from modsim.core.ids import ConstraintHandle, ModuleInstanceId
from modsim.core.transforms import ZERO_VEC3, Transform, Vec3, vec_add, vec_cross, vec_sub


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

    constraint_forces_n: Mapping[ConstraintHandle, float] = field(
        default_factory=dict[ConstraintHandle, float]
    )
    """Measured magnitude of each active constraint force, where supported."""

    def body(self, module_id: ModuleInstanceId, link: str) -> BodyState | None:
        """Return the state of one link, or ``None`` when the backend omits it."""
        return self.link_states.get(module_id, {}).get(link)
