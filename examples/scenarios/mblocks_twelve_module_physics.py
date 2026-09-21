"""Twelve-module one-plane M-Blocks physics demonstration.

Eleven face-connected modules form a grounded substrate. ``block_12`` starts
above ``block_1``, performs ten +Y quarter-turn surface traverses, and finishes
with one +Y half-turn beside ``block_11``. Every move uses the internal
flywheel, a temporary two-point edge hinge, gravity/contact integration, and
measured target-face capture. No module root is controlled after the initial
tree is staged at simulation time zero.

The route is a deterministic ModSim realization inspired by the 2019
M-Blocks line-formation work. It is not the paper's unpublished hardware move
trace, an autonomous planner, a continuous magnetic model, or a model of the
real three-plane actuator carrier.
"""

from __future__ import annotations

import math

from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform
from modsim.runtime.momentum_pivot import MomentumPivotConfig
from modsim.runtime.momentum_sequence import (
    MomentumPivotSequencePlan,
    MomentumPivotSequenceScenario,
)
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationAction,
    ReconfigurationPlan,
)
from modsim.runtime.session import RuntimeSession

SOURCE_URL = "https://doi.org/10.1109/IROS40897.2019.8967810"
MODULE_TYPE = "mblocks_3d"
MODULE_COUNT = 12
MOVING_MODULE = ModuleInstanceId("block_12")
NOMINAL_PITCH_M = 0.05

# Q_k = R_y(k*pi/2) before move k. Each connector below then points from the
# moving corner toward world -Z and can oppose the upright support's +Z port.
_MOVING_BOTTOM_FACE_CYCLE = ("neg_z", "pos_x", "pos_z", "neg_x")
_MOVING_EDGE_CYCLE = (
    "edge_pos_x_neg_z",
    "edge_pos_x_pos_z_from_pos_x",
    "edge_neg_x_pos_z",
    "edge_neg_x_neg_z_from_neg_x",
)
_FIXED_TOP_EDGE = "edge_pos_x_pos_z"
_QUARTER_TURN_FLYWHEEL_SPEED_RAD_S = 6_000.0 * math.tau / 60.0
_HALF_TURN_FLYWHEEL_SPEED_RAD_S = 9_000.0 * math.tau / 60.0


def build_scene() -> SceneSpec:
    """Place the substrate and moving cube on the 50 mm grounded lattice."""
    half_pitch = NOMINAL_PITCH_M / 2.0
    placements = [
        ModulePlacement(
            instance_id=_module(index),
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation(((index - 1) * NOMINAL_PITCH_M, 0.0, half_pitch)),
        )
        for index in range(1, MODULE_COUNT)
    ]
    placements.append(
        ModulePlacement(
            instance_id=MOVING_MODULE,
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation((0.0, 0.0, 3.0 * half_pitch)),
        )
    )
    return SceneSpec.of(placements)


def build_reconfiguration_plan() -> ReconfigurationPlan:
    """Return the same endpoint topology plan as the kinematic benchmark."""
    substrate_count = MODULE_COUNT - 1
    initial_connections = (
        *(_pair(index, "pos_x", index + 1, "neg_x") for index in range(1, substrate_count)),
        _pair(1, "pos_z", MODULE_COUNT, _MOVING_BOTTOM_FACE_CYCLE[0]),
    )
    actions: list[ReconfigurationAction] = []
    for move_index in range(substrate_count - 1):
        current_support = move_index + 1
        next_support = current_support + 1
        actions.append(
            ReconfigurationAction(
                label=(
                    f"Momentum-traverse block {MODULE_COUNT} from block "
                    f"{current_support} onto block {next_support}"
                ),
                undock=_pair(
                    current_support,
                    "pos_z",
                    MODULE_COUNT,
                    _MOVING_BOTTOM_FACE_CYCLE[move_index % 4],
                ),
                dock=_pair(
                    next_support,
                    "pos_z",
                    MODULE_COUNT,
                    _MOVING_BOTTOM_FACE_CYCLE[(move_index + 1) % 4],
                ),
            )
        )
    actions.append(
        ReconfigurationAction(
            label=f"Momentum-roll block {MODULE_COUNT} down to complete the line",
            undock=_pair(
                substrate_count,
                "pos_z",
                MODULE_COUNT,
                _MOVING_BOTTOM_FACE_CYCLE[(substrate_count - 1) % 4],
            ),
            dock=_pair(substrate_count, "pos_x", MODULE_COUNT, "neg_x"),
        )
    )
    return ReconfigurationPlan(
        id="mblocks_physical_twelve_module_line",
        name="M-Blocks Twelve-Module Physics Line Formation",
        module_ids=tuple(_module(index) for index in range(1, MODULE_COUNT + 1)),
        initial_connections=initial_connections,
        actions=tuple(actions),
        source_url=SOURCE_URL,
    )


def build_plan(*, dt_s: float = 0.0005) -> MomentumPivotSequencePlan:
    """Pair every topology replacement with its physical hinge/impulse input."""
    reconfiguration = build_reconfiguration_plan()
    pivots: list[MomentumPivotConfig] = []
    for move_index, action in enumerate(reconfiguration.actions):
        assert action.undock is not None and action.dock is not None
        current_support = min(move_index + 1, MODULE_COUNT - 1)
        half_turn = move_index == len(reconfiguration.actions) - 1
        pivots.append(
            MomentumPivotConfig(
                initial_face=action.undock,
                edge_hinge=_pair(
                    current_support,
                    _FIXED_TOP_EDGE,
                    MODULE_COUNT,
                    _MOVING_EDGE_CYCLE[move_index % 4],
                ),
                target_face=action.dock,
                dt_s=dt_s,
                settle_s=0.25 if move_index == 0 else 0.1,
                target_flywheel_speed_rad_s=(
                    _HALF_TURN_FLYWHEEL_SPEED_RAD_S
                    if half_turn
                    else _QUARTER_TURN_FLYWHEEL_SPEED_RAD_S
                ),
                pivot_timeout_s=3.0,
                connected_hold_s=0.35,
                target_pivot_angle_rad=math.pi if half_turn else math.pi / 2.0,
                plan_id=reconfiguration.id,
                plan_name=reconfiguration.name,
            )
        )
    return MomentumPivotSequencePlan(reconfiguration, tuple(pivots))


def build_scenario(
    session: RuntimeSession,
    *,
    dt_s: float = 0.0005,
) -> MomentumPivotSequenceScenario:
    """Create the complete physical sequence in an already-loaded session."""
    return MomentumPivotSequenceScenario.create(session, build_plan(dt_s=dt_s))


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


__all__ = [
    "MODULE_COUNT",
    "MODULE_TYPE",
    "MOVING_MODULE",
    "NOMINAL_PITCH_M",
    "SOURCE_URL",
    "build_plan",
    "build_reconfiguration_plan",
    "build_scenario",
    "build_scene",
]
