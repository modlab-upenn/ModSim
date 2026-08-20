"""Runtime session loop and modular-robot metrics."""

from modsim.runtime.inspection import (
    RuntimeEventRow,
    RuntimeInspectionError,
    RuntimeInspectorFrame,
    build_runtime_inspector_frame,
    event_row,
    format_event_detail,
)
from modsim.runtime.metrics import DockingMetrics, collect_metrics
from modsim.runtime.scenarios import (
    DockingPairPhase,
    DockingPairScenario,
    DockingPairScenarioConfig,
    DockingPairScenarioStatus,
    DockingPairSetup,
    ScenarioSetupError,
    stage_docking_pair,
)
from modsim.runtime.session import RuntimeSession

__all__ = [
    "DockingMetrics",
    "DockingPairPhase",
    "DockingPairScenario",
    "DockingPairScenarioConfig",
    "DockingPairScenarioStatus",
    "DockingPairSetup",
    "RuntimeEventRow",
    "RuntimeInspectionError",
    "RuntimeInspectorFrame",
    "RuntimeSession",
    "ScenarioSetupError",
    "build_runtime_inspector_frame",
    "collect_metrics",
    "event_row",
    "format_event_detail",
    "stage_docking_pair",
]
