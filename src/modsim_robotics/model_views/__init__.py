"""Backend-neutral mathematical model-view generation."""

from modsim_robotics.model_views.base import (
    DuplicateModelViewBuilderError,
    ModelViewBuilder,
    ModelViewBuildError,
    ModelViewContext,
    ModelViewDescriptor,
    ModelViewError,
    ModelViewUnavailableError,
    UnknownModelViewBuilderError,
)
from modsim_robotics.model_views.factory import ModelViewFactory
from modsim_robotics.model_views.lattice import CubicLatticeBuilder
from modsim_robotics.model_views.models import (
    MODEL_VIEW_SCHEMA_VERSION,
    CubicLatticeConnectionEdge,
    CubicLatticeNode,
    CubicLatticeOccupancyConflict,
    CubicLatticeOrientation,
    CubicLatticePoseResidual,
    CubicLatticeView,
    DockedConnectionEdge,
    GraphEdge,
    GraphModelView,
    GraphNode,
    LatticeFace,
    ModelView,
    ModelViewSourceStamp,
    ModuleGraphNode,
    ModuleTopologyGraphView,
)
from modsim_robotics.model_views.topology import ModuleTopologyGraphBuilder

__all__ = [
    "MODEL_VIEW_SCHEMA_VERSION",
    "CubicLatticeBuilder",
    "CubicLatticeConnectionEdge",
    "CubicLatticeNode",
    "CubicLatticeOccupancyConflict",
    "CubicLatticeOrientation",
    "CubicLatticePoseResidual",
    "CubicLatticeView",
    "DockedConnectionEdge",
    "DuplicateModelViewBuilderError",
    "GraphEdge",
    "GraphModelView",
    "GraphNode",
    "LatticeFace",
    "ModelView",
    "ModelViewBuildError",
    "ModelViewBuilder",
    "ModelViewContext",
    "ModelViewDescriptor",
    "ModelViewError",
    "ModelViewFactory",
    "ModelViewSourceStamp",
    "ModelViewUnavailableError",
    "ModuleGraphNode",
    "ModuleTopologyGraphBuilder",
    "ModuleTopologyGraphView",
    "UnknownModelViewBuilderError",
]
