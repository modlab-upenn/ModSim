"""Mutable runtime entities tracked by a ModSim world.

Robot Pack models are frozen Pydantic documents describing *types*. The objects
here are the per-step mutable *instances* those types produce, and they are
plain dataclasses so that a simulation loop can update poses without paying
validation cost on every step.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Generic, TypeVar

from modsim.core.ids import (
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    JointInstanceId,
    ModuleInstanceId,
)
from modsim.core.snapshot import JointState
from modsim.core.surface import SurfaceAttachment
from modsim.core.transforms import ZERO_VEC3, Transform, Vec3
from modsim.robot_packs.schema import ControlMode, PhysicalConstraintType, SurfaceRealization


@dataclass(frozen=True, slots=True)
class JointCommand:
    """One scalar command for one module joint.

    The command stays backend-neutral: its target is identified by the stable
    Robot Pack joint ID, its units follow the selected control mode and joint
    type, and a backend translates it to its native actuator representation.
    """

    joint: JointInstanceId
    mode: ControlMode
    value: float


class ConnectorLifecycleState(StrEnum):
    """Lifecycle of a single connector instance.

    ``ALIGNING`` and ``LOAD_BEARING`` are part of the modelled lifecycle but are
    not driven by the format-0.1 docking engine, because deciding that a
    connector is aligning or load bearing needs controller intent and measured
    constraint forces respectively. They are legal states that an adapter or a
    controller may set.
    """

    FREE = "free"
    CANDIDATE_DETECTED = "candidate_detected"
    ALIGNING = "aligning"
    IN_ACCEPTANCE_REGION = "in_acceptance_region"
    LATCHING = "latching"
    DOCKED = "docked"
    LOAD_BEARING = "load_bearing"
    RELEASING = "releasing"
    FAILED = "failed"


ENGAGED_STATES = frozenset(
    {
        ConnectorLifecycleState.LATCHING,
        ConnectorLifecycleState.DOCKED,
        ConnectorLifecycleState.LOAD_BEARING,
        ConnectorLifecycleState.RELEASING,
    }
)
"""States in which a connector holds or is releasing a logical connection."""


class ModuleInstance:
    """One module instance and its most recently ingested backend state.

    ``pose``, ``linear_velocity_m_s`` and ``angular_velocity_rad_s`` describe
    the root link and are read from the latest ingested sample on first access,
    then cached until the next sample; values match an eager per-step copy
    exactly. ``link_poses`` and ``joint_states`` may be read-only views over the
    latest backend snapshot rather than dictionaries; treat both as immutable.
    """

    __slots__ = (
        "_angular_velocity_rad_s",
        "_clock",
        "_linear_velocity_m_s",
        "_pose",
        "_stamp",
        "id",
        "joint_states",
        "link_poses",
        "module_type_id",
    )

    def __init__(
        self,
        id: ModuleInstanceId,
        module_type_id: str,
        pose: Transform | None = None,
        linear_velocity_m_s: Vec3 = ZERO_VEC3,
        angular_velocity_rad_s: Vec3 = ZERO_VEC3,
        link_poses: Mapping[str, Transform] | None = None,
        joint_states: Mapping[str, JointState] | None = None,
    ) -> None:
        self.id = id
        self.module_type_id = module_type_id
        self.link_poses: Mapping[str, Transform] = {} if link_poses is None else link_poses
        self.joint_states: Mapping[str, JointState] = {} if joint_states is None else joint_states
        self._pose = pose if pose is not None else Transform.identity()
        self._linear_velocity_m_s = linear_velocity_m_s
        self._angular_velocity_rad_s = angular_velocity_rad_s
        self._clock: SampleClock[ModuleInstance] | None = None
        self._stamp = _UNSET_STAMP

    def __repr__(self) -> str:
        return f"ModuleInstance(id={self.id!r}, module_type_id={self.module_type_id!r})"

    def bind_sample_clock(self, clock: SampleClock[ModuleInstance] | None) -> None:
        """Attach the owning world's clock; ``None`` freezes the current values."""
        self._clock = clock
        self._stamp = _UNSET_STAMP

    def store_root_state(
        self,
        pose: Transform | None = None,
        linear_velocity_m_s: Vec3 | None = None,
        angular_velocity_rad_s: Vec3 | None = None,
    ) -> None:
        """Record the root state for the current sample (owner only).

        ``None`` keeps the previous value, matching a sample that omits the root.
        """
        if pose is not None:
            self._pose = pose
        if linear_velocity_m_s is not None:
            self._linear_velocity_m_s = linear_velocity_m_s
        if angular_velocity_rad_s is not None:
            self._angular_velocity_rad_s = angular_velocity_rad_s
        if self._clock is not None:
            self._stamp = self._clock.stamp

    def settle(self) -> None:
        """Resolve against the current sample now, before a new one replaces it."""
        clock = self._clock
        if clock is not None and self._stamp != clock.stamp:
            clock.resolve(self)

    @property
    def pose(self) -> Transform:
        self.settle()
        return self._pose

    @pose.setter
    def pose(self, value: Transform) -> None:
        self.settle()
        self._pose = value

    @property
    def linear_velocity_m_s(self) -> Vec3:
        self.settle()
        return self._linear_velocity_m_s

    @linear_velocity_m_s.setter
    def linear_velocity_m_s(self, value: Vec3) -> None:
        self.settle()
        self._linear_velocity_m_s = value

    @property
    def angular_velocity_rad_s(self) -> Vec3:
        self.settle()
        return self._angular_velocity_rad_s

    @angular_velocity_rad_s.setter
    def angular_velocity_rad_s(self, value: Vec3) -> None:
        self.settle()
        self._angular_velocity_rad_s = value


_E = TypeVar("_E")


class SampleClock(Generic[_E]):
    """Shared stamp that tells lazily derived entities when they are stale.

    ``WorldState`` advances ``stamp`` whenever it ingests a new kinematic
    sample. An entity (module root state, connector frame) recomputes its
    derived values only when one of them is read after the stamp has moved, so
    ingesting a sample costs O(1) per entity instead of rebuilding every value.
    """

    __slots__ = ("resolve", "stamp")

    def __init__(self, resolve: Callable[[_E], None]) -> None:
        self.stamp = 0
        self.resolve = resolve


_UNSET_STAMP = -1


class ConnectorInstance:
    """One connector instance, its lifecycle state, and its resolved world frame.

    The world-frame fields (``world_pose``, ``world_docking_axis``,
    ``world_approach_axis``, ``world_velocity_m_s``, ``resolved``) are derived
    from the latest ingested sample on first read, then cached until the next
    sample. Reading them always returns the same values an eager per-step
    refresh would have produced. Assigning one first brings the whole frame
    up to date and then overrides that field until the next sample.
    """

    __slots__ = (
        "_clock",
        "_resolved",
        "_stamp",
        "_world_approach_axis",
        "_world_docking_axis",
        "_world_pose",
        "_world_velocity_m_s",
        "available_at_s",
        "capacity",
        "connection_ids",
        "connector_id",
        "connector_type_id",
        "id",
        "lifecycle_state",
        "local_pose",
        "module_id",
        "parent_link",
        "surface",
    )

    def __init__(
        self,
        id: ConnectorInstanceId,
        module_id: ModuleInstanceId,
        connector_id: str,
        connector_type_id: str,
        parent_link: str,
        lifecycle_state: ConnectorLifecycleState = ConnectorLifecycleState.FREE,
        connection_id: ConnectionId | None = None,
        world_pose: Transform | None = None,
        world_docking_axis: Vec3 = (1.0, 0.0, 0.0),
        world_approach_axis: Vec3 = (1.0, 0.0, 0.0),
        world_velocity_m_s: Vec3 = ZERO_VEC3,
        local_pose: Transform | None = None,
        resolved: bool = False,
        available_at_s: float = 0.0,
        *,
        surface: bool = False,
        capacity: int = 1,
    ) -> None:
        self.id = id
        self.module_id = module_id
        self.connector_id = connector_id
        self.connector_type_id = connector_type_id
        self.parent_link = parent_link
        self.lifecycle_state = lifecycle_state
        self.surface = surface
        """Whether this is a surface connector that can hold several connections."""
        self.capacity = capacity if surface else 1
        self.connection_ids: tuple[ConnectionId, ...] = (
            () if connection_id is None else (connection_id,)
        )
        """Every connection this connector holds, sorted. At most one unless ``surface``."""
        self.local_pose = local_pose if local_pose is not None else Transform.identity()
        self.available_at_s = available_at_s
        self._world_pose = world_pose if world_pose is not None else Transform.identity()
        self._world_docking_axis = world_docking_axis
        self._world_approach_axis = world_approach_axis
        self._world_velocity_m_s = world_velocity_m_s
        self._resolved = resolved
        self._clock: SampleClock[ConnectorInstance] | None = None
        self._stamp = _UNSET_STAMP

    def __repr__(self) -> str:
        return (
            f"ConnectorInstance(id={self.id!r}, lifecycle_state={self.lifecycle_state!r}, "
            f"connection_ids={self.connection_ids!r}, resolved={self.resolved!r})"
        )

    # -- lazy world frame ---------------------------------------------------

    def bind_frame_clock(self, clock: SampleClock[ConnectorInstance] | None) -> None:
        """Attach the owning world's clock; ``None`` freezes the current values."""
        self._clock = clock
        self._stamp = _UNSET_STAMP

    def store_frame(
        self,
        *,
        resolved: bool,
        world_pose: Transform | None = None,
        world_docking_axis: Vec3 | None = None,
        world_approach_axis: Vec3 | None = None,
        world_velocity_m_s: Vec3 | None = None,
    ) -> None:
        """Record a freshly derived frame for the current sample (owner only)."""
        if world_pose is not None:
            self._world_pose = world_pose
        if world_docking_axis is not None:
            self._world_docking_axis = world_docking_axis
        if world_approach_axis is not None:
            self._world_approach_axis = world_approach_axis
        if world_velocity_m_s is not None:
            self._world_velocity_m_s = world_velocity_m_s
        self._resolved = resolved
        if self._clock is not None:
            self._stamp = self._clock.stamp

    def settle(self) -> None:
        """Resolve against the current sample now, before a new one replaces it."""
        clock = self._clock
        if clock is not None and self._stamp != clock.stamp:
            clock.resolve(self)

    @property
    def world_pose(self) -> Transform:
        self.settle()
        return self._world_pose

    @world_pose.setter
    def world_pose(self, value: Transform) -> None:
        self.settle()
        self._world_pose = value

    @property
    def world_docking_axis(self) -> Vec3:
        self.settle()
        return self._world_docking_axis

    @world_docking_axis.setter
    def world_docking_axis(self, value: Vec3) -> None:
        self.settle()
        self._world_docking_axis = value

    @property
    def world_approach_axis(self) -> Vec3:
        self.settle()
        return self._world_approach_axis

    @world_approach_axis.setter
    def world_approach_axis(self, value: Vec3) -> None:
        self.settle()
        self._world_approach_axis = value

    @property
    def world_velocity_m_s(self) -> Vec3:
        self.settle()
        return self._world_velocity_m_s

    @world_velocity_m_s.setter
    def world_velocity_m_s(self, value: Vec3) -> None:
        self.settle()
        self._world_velocity_m_s = value

    @property
    def resolved(self) -> bool:
        self.settle()
        return self._resolved

    @resolved.setter
    def resolved(self, value: bool) -> None:
        self.settle()
        self._resolved = value

    @property
    def is_engaged(self) -> bool:
        """Whether this connector currently holds or is releasing a connection.

        For a surface connector this means "holds at least one connection";
        use :attr:`has_capacity` to ask whether it can accept another.
        """
        return self.lifecycle_state in ENGAGED_STATES

    @property
    def has_capacity(self) -> bool:
        """Whether this connector can accept one more connection now.

        A point connector has capacity while it is not engaged. A surface
        connector has capacity while it holds fewer connections than its
        declared capacity.
        """
        if self.surface:
            return len(self.connection_ids) < self.capacity
        return not self.is_engaged

    @property
    def connection_id(self) -> ConnectionId | None:
        """Return the connection this connector holds, if it holds exactly one."""
        return self.connection_ids[0] if len(self.connection_ids) == 1 else None


@dataclass(slots=True)
class ConnectionRuntime:
    """One committed logical connection backed by a backend constraint."""

    id: ConnectionId
    connector_a: ConnectorInstanceId
    connector_b: ConnectorInstanceId
    module_a: ModuleInstanceId
    module_b: ModuleInstanceId
    relative_transform: Transform
    orientation_rad: float
    orientation_index: int | None
    constraint_handle: ConstraintHandle
    created_at_s: float
    measured_force_n: float | None = None
    # Keep new defaulted fields after the original positional surface.  This
    # preserves callers that passed ``measured_force_n`` positionally before
    # constraint type became part of canonical runtime state.
    constraint: PhysicalConstraintType = PhysicalConstraintType.FIXED
    realization: SurfaceRealization | None = None
    """How a point-to-surface connection is realised; ``None`` for point-to-point."""
    committed_attachment: SurfaceAttachment | None = None
    """The attachment at commit. The live attachment is ``WorldState.surface_attachment``."""

    @property
    def connectors(self) -> tuple[ConnectorInstanceId, ConnectorInstanceId]:
        """Return both connector ends in canonical order."""
        return (self.connector_a, self.connector_b)
