"""The contract between ModSim semantics and a physics backend.

The boundary rule is narrow on purpose. ModSim asks a backend to load a scene,
step, report state, and create or remove a physical connection. The backend
never decides what a modular connector *means*: it does not evaluate
compatibility, acceptance, or lifecycle, and it does not create logical
connections. It only reports whether the constraint it was asked for exists.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
)
from modsim.core.scene import SceneSpec
from modsim.core.snapshot import BackendStateSnapshot
from modsim.core.transforms import Transform
from modsim.robot_packs.schema import PhysicalConnectionSpec, RobotPack


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
    supports_contact_forces: bool = False
    supports_module_pose_write: bool = False
    supports_joint_commands: bool = False
    supports_external_viewer: bool = False


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
class BackendAdapter(Protocol):
    """Minimal interface a physics backend must provide to ModSim."""

    def capabilities(self) -> BackendCapabilities:
        """Return what this backend supports."""
        ...

    def load(self, pack: RobotPack, scene: SceneSpec) -> BackendHandleRegistry:
        """Instantiate every module placement and return the handle registry."""
        ...

    def step(self, dt_s: float) -> None:
        """Advance the simulation by ``dt_s`` seconds."""
        ...

    def snapshot(self) -> BackendStateSnapshot:
        """Return the current backend state."""
        ...

    def create_physical_connection(self, request: ConnectionRequest) -> ConnectionOutcome:
        """Attempt to create the requested constraint."""
        ...

    def remove_physical_connection(self, handle: ConstraintHandle) -> bool:
        """Remove a previously created constraint, returning whether it existed."""
        ...

    def shutdown(self) -> None:
        """Release backend resources."""
        ...
