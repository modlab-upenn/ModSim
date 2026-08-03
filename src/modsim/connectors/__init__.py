"""Connector semantics: compatibility, acceptance, guards, and docking execution."""

from modsim.connectors.acceptance import (
    AcceptanceCriterion,
    AcceptanceResult,
    effective_acceptance_region,
    evaluate_acceptance,
    nominal_relative_transform,
)
from modsim.connectors.broadphase import SpatialHash, candidate_pairs
from modsim.connectors.compatibility import (
    CompatibilityResult,
    evaluate_compatibility,
    genders_match,
)
from modsim.connectors.docking import DockingManager, DockProposal, free_connectors
from modsim.connectors.guards import (
    SUPPORTED_CONSTRAINTS,
    GuardResult,
    evaluate_dock_guards,
    evaluate_undock_guards,
    resolve_physical_connection,
)

__all__ = [
    "SUPPORTED_CONSTRAINTS",
    "AcceptanceCriterion",
    "AcceptanceResult",
    "CompatibilityResult",
    "DockProposal",
    "DockingManager",
    "GuardResult",
    "SpatialHash",
    "candidate_pairs",
    "effective_acceptance_region",
    "evaluate_acceptance",
    "evaluate_compatibility",
    "evaluate_dock_guards",
    "evaluate_undock_guards",
    "free_connectors",
    "genders_match",
    "nominal_relative_transform",
    "resolve_physical_connection",
]
