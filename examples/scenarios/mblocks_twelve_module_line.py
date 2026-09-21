"""Twelve-module M-Blocks structure-to-line reference plan.

Eleven modules form a horizontal substrate and ``block_12`` starts above its
first cell.  Ten quarter-turn surface traverses carry the moving module across
the substrate; one final half-turn convex pivot places it beside ``block_11``
to complete a twelve-module line.

The 2019 M-Blocks work demonstrated decentralized line formation with physical
robots, but it does not publish a replayable per-module move trace.  This file
therefore provides a deterministic, paper-inspired ModSim benchmark rather
than claiming to reproduce the exact hardware trial.  Its connector plan and
edge rotations are backend-neutral and can be shared by kinematic and physical
executors.
"""

import math

from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, connector_instance_id
from modsim.runtime.kinematic_pivot import KinematicPivotRoute
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationAction,
    ReconfigurationPlan,
)

SOURCE_URL = "https://doi.org/10.1109/IROS40897.2019.8967810"
MODULE_COUNT = 12
_SUBSTRATE_COUNT = MODULE_COUNT - 1
_HALF_PITCH_M = 0.025
_MOVING_FACE_AFTER_QUARTER_TURNS = ("neg_z", "pos_x", "pos_z", "neg_x")


def build_plan() -> ReconfigurationPlan:
    """Return a connected structure-to-line edge-replacement plan."""
    initial_connections = (
        *(_pair(index, "pos_x", index + 1, "neg_x") for index in range(1, _SUBSTRATE_COUNT)),
        _pair(1, "pos_z", MODULE_COUNT, "neg_z"),
    )

    actions: list[ReconfigurationAction] = []
    current_support = 1
    current_face = "neg_z"
    for quarter_turn in range(1, _SUBSTRATE_COUNT):
        next_support = current_support + 1
        next_face = _MOVING_FACE_AFTER_QUARTER_TURNS[quarter_turn % 4]
        actions.append(
            ReconfigurationAction(
                label=(
                    f"Traverse block {MODULE_COUNT} from block {current_support} "
                    f"onto block {next_support}"
                ),
                undock=_pair(current_support, "pos_z", MODULE_COUNT, current_face),
                dock=_pair(next_support, "pos_z", MODULE_COUNT, next_face),
            )
        )
        current_support = next_support
        current_face = next_face

    actions.append(
        ReconfigurationAction(
            label=f"Roll block {MODULE_COUNT} down to complete the line",
            undock=_pair(current_support, "pos_z", MODULE_COUNT, current_face),
            dock=_pair(current_support, "pos_x", MODULE_COUNT, "neg_x"),
        )
    )

    return ReconfigurationPlan(
        id="mblocks_twelve_module_line",
        name="M-Blocks Twelve-Module Structure-to-Line",
        module_ids=tuple(_module(index) for index in range(1, MODULE_COUNT + 1)),
        initial_connections=initial_connections,
        actions=tuple(actions),
        source_url=SOURCE_URL,
    )


def build_routes() -> tuple[KinematicPivotRoute, ...]:
    """Return the ten surface traverses and final convex half-turn."""
    traverses = tuple(
        KinematicPivotRoute(
            moving_module=_module(MODULE_COUNT),
            reference_module=_module(reference_index),
            pivot_point_m=(_HALF_PITCH_M, 0.0, _HALF_PITCH_M),
            pivot_axis=(0.0, 1.0, 0.0),
            angle_rad=math.pi / 2.0,
            duration_s=1.25,
        )
        for reference_index in range(1, _SUBSTRATE_COUNT)
    )
    return (
        *traverses,
        KinematicPivotRoute(
            moving_module=_module(MODULE_COUNT),
            reference_module=_module(_SUBSTRATE_COUNT),
            pivot_point_m=(_HALF_PITCH_M, 0.0, _HALF_PITCH_M),
            pivot_axis=(0.0, 1.0, 0.0),
            angle_rad=math.pi,
            duration_s=2.5,
        ),
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


__all__ = ["MODULE_COUNT", "SOURCE_URL", "build_plan", "build_routes"]
