"""Authored five-module M-Blocks kinematic traversal example.

The four-module base remains connected while ``block_5`` follows three rigid
quarter-circle arcs across its top.  This is an illustrative kinematic route
inspired by the published 3D M-Blocks pivot primitive; it is not a replay of a
measured hardware trajectory and does not model flywheel momentum, magnetic
edge hinges, or capture forces.
"""

import math

from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, connector_instance_id
from modsim.runtime.kinematic_pivot import KinematicPivotRoute
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationAction,
    ReconfigurationPlan,
)

SOURCE_URL = "https://doi.org/10.1109/ICRA.2015.7139450"
_HALF_PITCH_M = 0.025


def build_plan() -> ReconfigurationPlan:
    """Return the five-module face-connection replacement plan."""
    return ReconfigurationPlan(
        id="mblocks_five_module_pivot",
        name="M-Blocks Five-Module Kinematic Pivot",
        module_ids=tuple(_module(index) for index in range(1, 6)),
        initial_connections=(
            _pair(1, "pos_x", 2, "neg_x"),
            _pair(2, "pos_x", 3, "neg_x"),
            _pair(3, "pos_x", 4, "neg_x"),
            _pair(1, "pos_z", 5, "neg_z"),
        ),
        actions=(
            ReconfigurationAction(
                label="Pivot block 5 from block 1 onto block 2",
                undock=_pair(1, "pos_z", 5, "neg_z"),
                dock=_pair(2, "pos_z", 5, "pos_x"),
            ),
            ReconfigurationAction(
                label="Pivot block 5 from block 2 onto block 3",
                undock=_pair(2, "pos_z", 5, "pos_x"),
                dock=_pair(3, "pos_z", 5, "pos_z"),
            ),
            ReconfigurationAction(
                label="Pivot block 5 from block 3 onto block 4",
                undock=_pair(3, "pos_z", 5, "pos_z"),
                dock=_pair(4, "pos_z", 5, "neg_x"),
            ),
        ),
        source_url=SOURCE_URL,
    )


def build_routes() -> tuple[KinematicPivotRoute, ...]:
    """Return three reference-frame edge rotations for :func:`build_plan`."""
    return tuple(
        KinematicPivotRoute(
            moving_module=_module(5),
            reference_module=_module(reference_index),
            pivot_point_m=(_HALF_PITCH_M, 0.0, _HALF_PITCH_M),
            pivot_axis=(0.0, 1.0, 0.0),
            angle_rad=math.pi / 2.0,
            duration_s=1.25,
        )
        for reference_index in range(1, 4)
    )


def _module(index: int) -> ModuleInstanceId:
    return ModuleInstanceId(f"block_{index}")


def _connector(module: int, local_connector: str) -> ConnectorInstanceId:
    return connector_instance_id(_module(module), local_connector)


def _pair(
    fixed_module: int,
    fixed_connector: str,
    moving_module: int,
    moving_connector: str,
) -> ConnectorPairRef:
    return ConnectorPairRef(
        fixed_connector=_connector(fixed_module, fixed_connector),
        moving_connector=_connector(moving_module, moving_connector),
    )


__all__ = ["SOURCE_URL", "build_plan", "build_routes"]
