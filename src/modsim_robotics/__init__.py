"""ModSim public package."""

from modsim_robotics.backends import (
    BackendAdapter,
    BackendCapabilities,
    MockBackendAdapter,
    SupportsJointCommands,
)
from modsim_robotics.connectors import DockingManager, DockProposal, evaluate_acceptance
from modsim_robotics.core import (
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
from modsim_robotics.core.entities import JointCommand
from modsim_robotics.core.ids import JointInstanceId
from modsim_robotics.model_views import (
    CubicLatticeView,
    GraphModelView,
    ModelView,
    ModelViewBuilder,
    ModelViewContext,
    ModelViewDescriptor,
    ModelViewError,
    ModelViewFactory,
    ModuleTopologyGraphView,
)
from modsim_robotics.robot_packs import (
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
from modsim_robotics.runtime import DockingMetrics, RuntimeSession, collect_metrics
from modsim_robotics.runtime.session import JointCommandError

__all__ = [
    "AssemblyIndex",
    "BackendAdapter",
    "BackendCapabilities",
    "BackendStateSnapshot",
    "ConnectorLifecycleState",
    "CubicLatticeView",
    "DockProposal",
    "DockingManager",
    "DockingMetrics",
    "EventLog",
    "GraphModelView",
    "JointCommand",
    "JointCommandError",
    "JointInstanceId",
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
    "SupportsJointCommands",
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
