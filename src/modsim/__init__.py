"""ModSim public package."""

from modsim.backends import BackendAdapter, BackendCapabilities, MockBackendAdapter
from modsim.connectors import DockingManager, DockProposal, evaluate_acceptance
from modsim.core import (
    AssemblyIndex,
    BackendStateSnapshot,
    ConnectorLifecycleState,
    EventLog,
    ModulePlacement,
    SceneSpec,
    Transform,
    WorldState,
    WorldStateRevision,
)
from modsim.model_views import (
    GraphModelView,
    ModelView,
    ModelViewBuilder,
    ModelViewContext,
    ModelViewDescriptor,
    ModelViewError,
    ModelViewFactory,
    ModuleTopologyGraphView,
)
from modsim.robot_packs import (
    LoadedRobotPack,
    RobotPack,
    RobotPackLoader,
    RobotPackValidator,
    RobotPackWriter,
    ValidationProfile,
    load_robot_pack,
    validate_robot_pack,
    write_robot_pack,
)
from modsim.runtime import DockingMetrics, RuntimeSession, collect_metrics

__all__ = [
    "AssemblyIndex",
    "BackendAdapter",
    "BackendCapabilities",
    "BackendStateSnapshot",
    "ConnectorLifecycleState",
    "DockProposal",
    "DockingManager",
    "DockingMetrics",
    "EventLog",
    "GraphModelView",
    "LoadedRobotPack",
    "MockBackendAdapter",
    "ModelView",
    "ModelViewBuilder",
    "ModelViewContext",
    "ModelViewDescriptor",
    "ModelViewError",
    "ModelViewFactory",
    "ModulePlacement",
    "ModuleTopologyGraphView",
    "RobotPack",
    "RobotPackLoader",
    "RobotPackValidator",
    "RobotPackWriter",
    "RuntimeSession",
    "SceneSpec",
    "Transform",
    "ValidationProfile",
    "WorldState",
    "WorldStateRevision",
    "__version__",
    "collect_metrics",
    "evaluate_acceptance",
    "load_robot_pack",
    "validate_robot_pack",
    "write_robot_pack",
]

__version__ = "0.1.0"
