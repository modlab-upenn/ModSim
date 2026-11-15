"""Canonical runtime state for modular robot worlds."""

from modsim_robotics.core.assemblies import AssemblyIndex, MergeResult, SplitResult
from modsim_robotics.core.entities import (
    ConnectionRuntime,
    ConnectorInstance,
    ConnectorLifecycleState,
    ModuleInstance,
)
from modsim_robotics.core.events import (
    AssemblyMerged,
    AssemblySplit,
    ConnectorOverloaded,
    DockCandidateDetected,
    DockCommitted,
    DockFailed,
    DockFailureReason,
    Event,
    EventLog,
    UndockCommitted,
    UndockFailed,
)
from modsim_robotics.core.ids import (
    AssemblyId,
    ConnectionId,
    ConnectorInstanceId,
    ConstraintHandle,
    ModuleInstanceId,
    assembly_id,
    connection_id,
    connector_instance_id,
    split_connector_instance_id,
)
from modsim_robotics.core.scene import ModulePlacement, SceneError, SceneSpec
from modsim_robotics.core.snapshot import BackendStateSnapshot, BodyState, JointState
from modsim_robotics.core.state import WorldState, WorldStateError, WorldStateRevision
from modsim_robotics.core.transforms import Transform, Vec3

__all__ = [
    "AssemblyId",
    "AssemblyIndex",
    "AssemblyMerged",
    "AssemblySplit",
    "BackendStateSnapshot",
    "BodyState",
    "ConnectionId",
    "ConnectionRuntime",
    "ConnectorInstance",
    "ConnectorInstanceId",
    "ConnectorLifecycleState",
    "ConnectorOverloaded",
    "ConstraintHandle",
    "DockCandidateDetected",
    "DockCommitted",
    "DockFailed",
    "DockFailureReason",
    "Event",
    "EventLog",
    "JointState",
    "MergeResult",
    "ModuleInstance",
    "ModuleInstanceId",
    "ModulePlacement",
    "SceneError",
    "SceneSpec",
    "SplitResult",
    "Transform",
    "UndockCommitted",
    "UndockFailed",
    "Vec3",
    "WorldState",
    "WorldStateError",
    "WorldStateRevision",
    "assembly_id",
    "connection_id",
    "connector_instance_id",
    "split_connector_instance_id",
]
