"""What each demonstration needs from a physics backend.

Demonstrations used to be gated on the backend's *name*. They are now gated on
declared :class:`~modsim.backends.base.BackendCapabilities`, so any adapter
that reports the right capabilities can run them. Engine-specific tuning that
has no backend-neutral meaning lives in :data:`ENGINE_TUNING`, keyed by the
engine it applies to, and is passed only to that engine.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any

from modsim.backends.base import BackendCapabilities, BackendError
from modsim.backends.registry import backend_capabilities
from modsim.core.scene import ConnectionBudget
from modsim.robot_packs.schema import ControlMode, PhysicalConstraintType
from modsim.runtime.demos import RuntimeDemo


@dataclass(frozen=True, slots=True)
class BackendRequirements:
    """Capabilities a scenario needs from whichever backend runs it."""

    gravity: bool = False
    ground_plane: bool = False
    contact_dynamics: bool = False
    runtime_constraints: bool = False
    constraint_removal: bool = False
    constraint_types: frozenset[PhysicalConstraintType] = field(
        default_factory=frozenset[PhysicalConstraintType]
    )
    joint_control_modes: frozenset[ControlMode] = field(default_factory=frozenset[ControlMode])
    surface_slide: bool = False
    """Point connectors are driven over a surface while attached.

    Only ``supports_connection_retarget`` satisfies this. A ``ball`` constraint
    lets a strut slide passively but cannot be driven, because nothing presses
    the wheels onto the surface (docs/continuous_docking.md, amendment A3).
    """

    def problems(self, capabilities: BackendCapabilities) -> tuple[str, ...]:
        """Return each requirement ``capabilities`` does not meet."""
        missing: list[str] = []
        if self.gravity and not capabilities.supports_gravity:
            missing.append("gravity")
        if self.ground_plane and not capabilities.supports_ground_plane:
            missing.append("a ground plane")
        if self.contact_dynamics and not capabilities.supports_contact_dynamics:
            missing.append("contact dynamics")
        if self.runtime_constraints and not capabilities.supports_runtime_constraints:
            missing.append("runtime connections")
        if self.constraint_removal and not capabilities.supports_constraint_removal:
            missing.append("connection removal")
        for kind in sorted(self.constraint_types - capabilities.supported_constraint_types):
            missing.append(f"{kind.value} connections")
        for mode in sorted(self.joint_control_modes - capabilities.supported_joint_control_modes):
            missing.append(f"{mode.value} joint control")
        if self.surface_slide and not capabilities.supports_connection_retarget:
            missing.append("driven surface sliding (connection retarget)")
        return tuple(missing)


_PHYSICAL_RECONFIGURATION = BackendRequirements(
    gravity=True,
    ground_plane=True,
    contact_dynamics=True,
    runtime_constraints=True,
    constraint_removal=True,
    constraint_types=frozenset({PhysicalConstraintType.FIXED}),
    joint_control_modes=frozenset({ControlMode.EFFORT}),
)
"""Wheel- or flywheel-driven modules that dock and undock under real contact."""

_PHYSICAL_PIVOTING = BackendRequirements(
    gravity=True,
    ground_plane=True,
    contact_dynamics=True,
    runtime_constraints=True,
    constraint_removal=True,
    constraint_types=frozenset({PhysicalConstraintType.FIXED, PhysicalConstraintType.HINGE}),
    joint_control_modes=frozenset({ControlMode.EFFORT}),
)
"""M-Blocks-style edge pivots: fixed face bonds plus a transient edge hinge."""

DEMO_REQUIREMENTS: Mapping[RuntimeDemo, BackendRequirements] = MappingProxyType(
    {
        RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT: _PHYSICAL_PIVOTING,
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE: _PHYSICAL_PIVOTING,
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE: _PHYSICAL_PIVOTING,
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE: _PHYSICAL_PIVOTING,
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE_LARGE: _PHYSICAL_PIVOTING,
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE_ELBOW: _PHYSICAL_PIVOTING,
        RuntimeDemo.SMORES_DIFF_DRIVE_DOCK_UNDOCK: _PHYSICAL_RECONFIGURATION,
        RuntimeDemo.SMORES_PHYSICAL_DRIVER_TO_SNAKE: _PHYSICAL_RECONFIGURATION,
        RuntimeDemo.SMORES_ONLINE_ASSEMBLY: _PHYSICAL_RECONFIGURATION,
        RuntimeDemo.SMORES_ONLINE_DRIVER_TO_SNAKE: _PHYSICAL_RECONFIGURATION,
    }
)
"""Demos absent from this table run on any backend (the kinematic demos)."""

DEMO_CONNECTION_BUDGETS: Mapping[RuntimeDemo, ConnectionBudget] = MappingProxyType(
    {
        RuntimeDemo.MBLOCKS_MOMENTUM_PIVOT: ConnectionBudget(fixed=2, hinge=2),
        # Eleven simultaneous face bonds and one transient hinge. Without a
        # budget a pool-based engine would reserve one slot per directed
        # connector pair (84 here).
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE: ConnectionBudget(fixed=12, hinge=2),
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE: ConnectionBudget(fixed=12, hinge=2),
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE_LARGE: ConnectionBudget(fixed=12, hinge=2),
        RuntimeDemo.MBLOCKS_ONLINE_LATTICE_ELBOW: ConnectionBudget(fixed=12, hinge=2),
        # The mat begins with 16 cyclic face bonds and finishes with 18; only
        # one edge hinge is active during a coordinated slab pivot.
        RuntimeDemo.MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE: ConnectionBudget(fixed=24, hinge=2),
        RuntimeDemo.MBLOCKS_TWELVE_MODULE_STAIRCASE: ConnectionBudget(fixed=24, hinge=2),
    }
)
"""How many connections each demo holds at once. Backend-neutral sizing hints."""

ENGINE_TUNING: Mapping[RuntimeDemo, Mapping[str, Mapping[str, Any]]] = MappingProxyType(
    {
        demo: MappingProxyType({"mujoco": MappingProxyType({"constraint_time_constant_s": 0.002})})
        for demo in (
            RuntimeDemo.MBLOCKS_ONLINE_LATTICE,
            RuntimeDemo.MBLOCKS_ONLINE_LATTICE_LARGE,
            RuntimeDemo.MBLOCKS_ONLINE_LATTICE_ELBOW,
        )
    }
)
"""Per-engine solver tuning a demo was validated with.

These are deliberately keyed by engine name: a solver time constant has no
meaning outside the engine it tunes, so another backend simply receives none.
"""


def demo_backend_problems(demo: RuntimeDemo | None, backend: str | None) -> tuple[str, ...]:
    """Return what ``backend`` lacks to run ``demo``; empty when it can.

    Raises :class:`BackendError` when the backend is unknown or not installed,
    since its capabilities cannot be known.
    """
    if demo is None or demo not in DEMO_REQUIREMENTS:
        return ()
    return DEMO_REQUIREMENTS[demo].problems(backend_capabilities(backend))


def describe_backend_problems(demo: RuntimeDemo, backend: str, problems: tuple[str, ...]) -> str:
    """Return one user-facing sentence for :func:`demo_backend_problems`."""
    return (
        f"{demo.value} needs a backend with {', '.join(problems)}; backend '{backend}' lacks them"
    )


def engine_tuning(demo: RuntimeDemo | None, backend: str) -> dict[str, Any]:
    """Return the engine-specific constructor options validated for ``demo``."""
    if demo is None:
        return {}
    return dict(ENGINE_TUNING.get(demo, {}).get(backend, {}))


__all__ = [
    "DEMO_CONNECTION_BUDGETS",
    "DEMO_REQUIREMENTS",
    "ENGINE_TUNING",
    "BackendError",
    "BackendRequirements",
    "demo_backend_problems",
    "describe_backend_problems",
    "engine_tuning",
]
