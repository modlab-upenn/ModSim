"""The contract between ModSim semantics and a physics backend.

The boundary rule is narrow on purpose. ModSim asks a backend to load a scene,
step, report state, and create or remove a physical connection. The backend
never decides what a modular connector *means*: it does not evaluate
compatibility, acceptance, or lifecycle, and it does not create logical
connections. It only reports whether the constraint it was asked for exists.

Everything engine-specific stays in the adapter. World properties (gravity,
ground, timestep) and the connection budget arrive in the :class:`SceneSpec`;
what an engine can do is declared in :class:`BackendCapabilities`, and callers
check those flags rather than a backend's name.

Timing contract
---------------
ModSim drives a backend as a sequence of ``step`` calls with commands issued
between them. Every adapter must honour these rules, so that controllers and
docking logic behave the same on every engine:

1. **One instant per snapshot.** After ``load``, ``step`` or a pose/twist
   write returns, ``snapshot()`` describes the single instant ``time_s``:
   every link pose, twist, joint state and connector frame is the state at
   that time. A backend must not mix the state before and after its last
   internal step.
2. **Wrenches cover the last step.** ``constraint_wrenches`` are what the
   solver applied during the step that ended at ``time_s``. Before the first
   step they may be empty or come from a static solve.
3. **Commands act over the whole next step.** Joint commands set between
   steps are held for the entire next ``step`` call.
4. **Pose and twist writes are immediate.** They are visible in the next
   ``snapshot()`` without stepping.
5. **Connection changes have a declared delay.** A connection created or
   removed between steps is enforced, or released, no later than
   ``BackendCapabilities.connection_change_delay_substeps`` internal steps
   into the next ``step`` call. Zero means "from the first internal step".
   The logical connection in ModSim changes immediately either way.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Protocol, runtime_checkable

from modsim.core.entities import JointCommand
from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    JointInstanceId,
    ModuleInstanceId,
)
from modsim.core.scene import SceneSpec, WorldSettings
from modsim.core.snapshot import BackendStateSnapshot
from modsim.core.transforms import ZERO_VEC3, Transform, Vec3
from modsim.robot_packs.schema import (
    ControlMode,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    RobotPack,
    SurfaceRealization,
)


class BackendError(RuntimeError):
    """Raised when a backend cannot satisfy a request."""


@dataclass(frozen=True, slots=True)
class BackendCapabilities:
    """What a backend can actually do.

    ModSim degrades explicitly rather than silently. A backend that cannot
    create constraints at runtime reports it here, and the runtime refuses
    docking instead of committing logical connections that no physics engine is
    enforcing.
    """

    name: str
    supports_runtime_constraints: bool = False
    supports_constraint_removal: bool = False
    supports_constraint_forces: bool = False
    """Snapshots carry the force part of each connection's wrench."""
    supports_constraint_torques: bool = False
    """Snapshots also carry the moment part of each connection's wrench."""
    supports_contact_forces: bool = False
    supports_module_pose_write: bool = False
    supports_joint_commands: bool = False
    supports_external_viewer: bool = False
    supports_gravity: bool = False
    """The backend honours a non-zero ``WorldSettings.gravity_m_s2``."""
    supports_ground_plane: bool = False
    """The backend honours ``WorldSettings.ground``."""
    supports_contact_dynamics: bool = False
    """Bodies collide with each other and the ground and respond with friction.

    Physically driven demonstrations (rolling, pivoting, pushing) need this. A
    kinematic backend that only moves bodies where it is told reports False.
    """
    supported_joint_control_modes: frozenset[ControlMode] = field(
        default_factory=frozenset[ControlMode]
    )
    supported_constraint_types: frozenset[PhysicalConstraintType] = field(
        default_factory=frozenset[PhysicalConstraintType]
    )
    """Physical connection kinds ``create_physical_connection`` can realise.

    Empty with ``supports_runtime_constraints`` means fixed connections only.
    """
    supports_connection_retarget: bool = False
    """The adapter implements :class:`SupportsConnectionRetarget` for fixed connections."""
    connection_change_delay_substeps: int = 0
    """Internal steps before a connection change takes physical effect.

    See rule 5 of the timing contract in this module's docstring.
    """

    def __post_init__(self) -> None:
        """Normalize the sets and keep the legacy booleans meaningful."""
        modes = frozenset(ControlMode(mode) for mode in self.supported_joint_control_modes)
        object.__setattr__(self, "supported_joint_control_modes", modes)
        if modes and not self.supports_joint_commands:
            object.__setattr__(self, "supports_joint_commands", True)
        kinds = frozenset(PhysicalConstraintType(kind) for kind in self.supported_constraint_types)
        if not kinds and self.supports_runtime_constraints:
            kinds = frozenset({PhysicalConstraintType.FIXED})
        object.__setattr__(self, "supported_constraint_types", kinds)
        if self.supports_constraint_torques and not self.supports_constraint_forces:
            raise ValueError("supports_constraint_torques requires supports_constraint_forces")
        if self.connection_change_delay_substeps < 0:
            raise ValueError("connection_change_delay_substeps must be non-negative")

    def world_support_problems(self, world: WorldSettings) -> tuple[str, ...]:
        """Return why this backend cannot honour ``world``, or nothing if it can."""
        problems: list[str] = []
        if world.requests_gravity and not self.supports_gravity:
            problems.append(f"backend '{self.name}' does not simulate gravity")
        if world.ground and not self.supports_ground_plane:
            problems.append(f"backend '{self.name}' does not provide a ground plane")
        return tuple(problems)


@dataclass(frozen=True, slots=True)
class ConnectionRequest:
    """A request to realise one logical connection as a physical constraint.

    Both connectors' link-local frames are supplied alongside the desired
    connector-frame relative pose, so the backend can convert to whatever
    body-relative form its constraint model needs without re-deriving ModSim
    geometry.
    """

    connection_id: ConnectionId
    module_a: ModuleInstanceId
    link_a: str
    connector_a: ConnectorInstanceId
    connector_a_local: Transform
    module_b: ModuleInstanceId
    link_b: str
    connector_b: ConnectorInstanceId
    connector_b_local: Transform
    relative_transform: Transform
    physical_connection: PhysicalConnectionSpec
    orientation_rad: float = 0.0
    orientation_index: int | None = None
    snap_to_nominal: bool = False
    snap_side: Literal["a", "b"] = "b"
    """Which side a nominal snap moves. ``"b"`` (the default) moves module B.

    A point-to-surface connection sets ``"a"``, so the sliding strut moves to
    the surface instead of dragging the node and every other strut on it.
    """
    surface_realization: SurfaceRealization | None = None
    """For a point-to-surface connection, how it is realised; informational.

    The backend never needs it: side A is the point connector, side B's
    ``connector_b_local`` is the virtual attachment frame on the surface, and
    ``physical_connection`` is already the realised ``fixed`` or ``ball`` spec.
    """
    surface_center_link_b: Vec3 | None = None
    """The surface's sphere centre in link B's frame; informational."""


@dataclass(frozen=True, slots=True)
class ConnectionRetarget:
    """A new desired relative pose for an existing fixed connection.

    The fields mean what they mean in :class:`ConnectionRequest`. For a
    point-to-surface connection, ``connector_b_local`` is the new virtual
    attachment frame on the surface, in link B's frame.
    """

    handle: ConstraintHandle
    connector_a_local: Transform
    connector_b_local: Transform
    relative_transform: Transform


@dataclass(frozen=True, slots=True)
class ConnectionOutcome:
    """A backend's answer to a connection request."""

    success: bool
    handle: ConstraintHandle | None = None
    detail: str | None = None

    @classmethod
    def accepted(cls, handle: ConstraintHandle) -> ConnectionOutcome:
        """Return a successful outcome."""
        return cls(success=True, handle=handle)

    @classmethod
    def refused(cls, detail: str) -> ConnectionOutcome:
        """Return a failed outcome carrying the backend's explanation."""
        return cls(success=False, detail=detail)


@dataclass(frozen=True, slots=True)
class BackendHandleRegistry:
    """Mapping from ModSim identifiers to backend-native handles."""

    bodies: Mapping[tuple[ModuleInstanceId, str], str] = field(
        default_factory=dict[tuple[ModuleInstanceId, str], str]
    )
    joints: Mapping[tuple[ModuleInstanceId, str], str] = field(
        default_factory=dict[tuple[ModuleInstanceId, str], str]
    )
    connector_frames: Mapping[ConnectorInstanceId, str] = field(
        default_factory=dict[ConnectorInstanceId, str]
    )


@runtime_checkable
class SupportsJointCommands(Protocol):
    """Optional backend ability to command scalar robot joints.

    Commands arrive only after :class:`~modsim.runtime.session.RuntimeSession`
    has resolved their stable IDs and validated their declared modes and
    limits. Implementations must apply each tuple as one batch: either every
    target is accepted or no target is changed.
    """

    def set_joint_commands(self, commands: tuple[JointCommand, ...]) -> None:
        """Set a validated batch of persistent joint targets."""
        ...

    def clear_joint_commands(
        self,
        joints: tuple[JointInstanceId, ...] | None = None,
    ) -> None:
        """Clear selected persistent targets, or every target when omitted."""
        ...


@runtime_checkable
class SupportsConnectionRetarget(Protocol):
    """Optional backend ability to change an existing fixed connection's relative pose.

    It is advertised by ``BackendCapabilities.supports_connection_retarget``.
    One call carries every update for a step. A retarget is a connection change
    under rule 5 of the timing contract: it acts within
    ``connection_change_delay_substeps`` of the next ``step``. Implementations
    validate the whole batch before applying any of it, and return one flag per
    update, ``False`` marking an update whose handle is not held or that was
    refused. If any flag is ``False``, no update in the batch was applied.
    """

    def retarget_physical_connections(
        self,
        updates: tuple[ConnectionRetarget, ...],
    ) -> tuple[bool, ...]:
        """Apply a batch of relative-pose updates atomically."""
        ...


@runtime_checkable
class SupportsModuleKinematics(Protocol):
    """Optional backend ability to place and drive whole modules directly.

    This is deliberately not part of :class:`BackendAdapter`. Writing a pose or
    velocity bypasses whatever dynamics a backend has, so it belongs to
    scenario setup and testing rather than to the simulation contract. A caller
    should check for it rather than assume it.
    """

    def set_module_pose(self, module_id: ModuleInstanceId, pose: Transform) -> None:
        """Place one module, clearing any velocity it had."""
        ...

    def set_module_twist(
        self,
        module_id: ModuleInstanceId,
        *,
        linear_m_s: Vec3 = ZERO_VEC3,
        angular_rad_s: Vec3 = ZERO_VEC3,
    ) -> None:
        """Set one module's world-frame linear and angular velocity."""
        ...


@runtime_checkable
class BackendAdapter(Protocol):
    """Minimal interface a physics backend must provide to ModSim."""

    def capabilities(self) -> BackendCapabilities:
        """Return what this backend supports."""
        ...

    def load(
        self,
        pack: RobotPack,
        scene: SceneSpec,
        *,
        root: Path | None = None,
    ) -> BackendHandleRegistry:
        """Instantiate every module placement and return the handle registry.

        ``scene.world`` carries gravity, ground and timestep, and
        ``scene.connection_budget`` sizes any storage reserved for runtime
        connections. A backend that cannot honour ``scene.world`` raises
        :class:`BackendError` (the runtime checks capabilities first).

        ``root`` is the Robot Pack directory. A backend that must read
        mechanical assets from disk requires it; one that works purely from the
        semantic model, such as the mock, ignores it.
        """
        ...

    def step(self, dt_s: float) -> None:
        """Advance the simulation by ``dt_s`` seconds.

        On return, ``snapshot()`` must describe the state at the new time; see
        the timing contract in this module's docstring.
        """
        ...

    def snapshot(self) -> BackendStateSnapshot:
        """Return the current backend state, all at one instant (``time_s``)."""
        ...

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Attempt to create the requested constraint.

        The constraint takes physical effect within the declared
        ``connection_change_delay_substeps`` of the next ``step``. Its wrench is
        reported from module A's side, about connector A's origin.
        """
        ...

    def remove_physical_connection(self, handle: ConstraintHandle) -> bool:
        """Remove a previously created constraint, returning whether it existed."""
        ...

    def shutdown(self) -> None:
        """Release backend resources."""
        ...
