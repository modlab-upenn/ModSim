"""Twelve M-Blocks reconfigure from a 2 x 6 mat into a 3-step staircase.

The route is a deterministic ModSim demonstration built from the published
M-Blocks edge-pivot primitives.  It is not a replay of a paper's unpublished
hardware trace and it is not an autonomous planner.  Six rigid two-module
slabs move in one plane about +Y; this produces a genuinely three-dimensional,
two-block-deep staircase while remaining compatible with the Robot Pack's
current one-plane flywheel bootstrap.

Both executors below consume the same exact topology and route:

* ``build_kinematic_scenario`` writes the detached slab roots along analytical
  arcs and is the deterministic reference/regression oracle.
* ``build_physical_scenario`` commands both slab flywheels, releases the fixed
  faces onto a temporary edge hinge, and relies on MuJoCo gravity, contacts,
  constraints, and momentum for all motion after initialization.
"""

from __future__ import annotations

import math

from modsim.core.ids import (
    ConnectorInstanceId,
    ModuleInstanceId,
    connector_instance_id,
    split_connector_instance_id,
)
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import Transform, quat_from_axis_angle
from modsim.runtime.coordinated_pivot import (
    CoordinatedKinematicPivotScenario,
    CoordinatedMomentumPivotScenario,
    CoordinatedPivotAction,
    CoordinatedPivotPlan,
)
from modsim.runtime.kinematic_pivot import KinematicPivotConfig, KinematicPivotRoute
from modsim.runtime.momentum_pivot import MomentumPivotConfig
from modsim.runtime.reconfiguration import ConnectorPairRef
from modsim.runtime.session import RuntimeSession

SOURCE_URL = "https://doi.org/10.1109/ICRA.2015.7139450"
PLANNING_SOURCE_URL = "https://doi.org/10.1109/ICRA.2015.7139451"
MODULE_TYPE = "mblocks_3d"
MODULE_COUNT = 12
NOMINAL_PITCH_M = 0.05
HALF_PITCH_M = NOMINAL_PITCH_M / 2.0

KINEMATIC_PLAN_ID = "mblocks_twelve_module_staircase"
PHYSICAL_PLAN_ID = "mblocks_physical_twelve_module_staircase"
PLAN_NAME = "M-Blocks Twelve-Module Mat-to-Staircase"

# Slabs are listed front row then rear row. Keeping the familiar block_1..6
# and block_7..12 row ordering makes the lattice view easy to read.
SLABS: dict[str, tuple[ModuleInstanceId, ModuleInstanceId]] = {
    "A": (ModuleInstanceId("block_1"), ModuleInstanceId("block_7")),
    "B": (ModuleInstanceId("block_2"), ModuleInstanceId("block_8")),
    "C": (ModuleInstanceId("block_3"), ModuleInstanceId("block_9")),
    "D": (ModuleInstanceId("block_4"), ModuleInstanceId("block_10")),
    "E": (ModuleInstanceId("block_5"), ModuleInstanceId("block_11")),
    "F": (ModuleInstanceId("block_6"), ModuleInstanceId("block_12")),
}

INITIAL_CELLS: dict[ModuleInstanceId, tuple[int, int, int]] = {
    module: (x, row, 0)
    for x, slab in enumerate(("A", "B", "C", "D", "E", "F"))
    for row, module in enumerate(SLABS[slab])
}
FINAL_CELLS: dict[ModuleInstanceId, tuple[int, int, int]] = {
    **{module: (3, row, 0) for row, module in enumerate(SLABS["D"])},
    **{module: (4, row, 0) for row, module in enumerate(SLABS["E"])},
    **{module: (5, row, 0) for row, module in enumerate(SLABS["F"])},
    **{module: (4, row, 1) for row, module in enumerate(SLABS["B"])},
    **{module: (5, row, 1) for row, module in enumerate(SLABS["A"])},
    **{module: (5, row, 2) for row, module in enumerate(SLABS["C"])},
}

_QUARTER_TURN_SPEED_RAD_S = 6_000.0 * math.tau / 60.0
_HALF_TURN_SPEED_RAD_S = 15_000.0 * math.tau / 60.0
_ELEVATED_HALF_TURN_SPEED_RAD_S = 19_500.0 * math.tau / 60.0


def build_scene() -> SceneSpec:
    """Place the exact grounded 2 x 6 starting mat."""
    return SceneSpec.of(
        ModulePlacement(
            instance_id=module,
            module_type_id=MODULE_TYPE,
            pose=Transform.from_translation(
                (
                    cell[0] * NOMINAL_PITCH_M,
                    cell[1] * NOMINAL_PITCH_M,
                    HALF_PITCH_M,
                )
            ),
        )
        for module, cell in INITIAL_CELLS.items()
    )


def build_plan(
    *,
    dt_s: float = 0.0005,
    physical: bool = False,
) -> CoordinatedPivotPlan:
    """Return the shared cyclic topology and eleven collision-free pivots."""
    plan_id = PHYSICAL_PLAN_ID if physical else KINEMATIC_PLAN_ID
    initial = (
        *(_pair(SLABS[slab][0], "pos_y", SLABS[slab][1], "neg_y") for slab in SLABS),
        *(
            pair
            for left, right in zip("ABCDE", "BCDEF", strict=True)
            for pair in _paired_faces(left, "pos_x", right, "neg_x")
        ),
    )

    specs = (
        # moving, reference, start faces, hinge connectors, target faces,
        # pivot point, angle, and target speed.
        (
            "A",
            "B",
            _paired_faces("B", "neg_x", "A", "pos_x"),
            ("edge_neg_x_pos_z_from_neg_x", "edge_pos_x_pos_z_from_pos_x"),
            _paired_faces("B", "pos_z", "A", "pos_z"),
            (-HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi,
            _HALF_TURN_SPEED_RAD_S,
            "Lift slab A over slab B",
        ),
        (
            "A",
            "B",
            _paired_faces("B", "pos_z", "A", "pos_z"),
            ("edge_pos_x_pos_z", "edge_neg_x_pos_z"),
            _paired_faces("C", "pos_z", "A", "neg_x"),
            (HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi / 2.0,
            _QUARTER_TURN_SPEED_RAD_S,
            "Traverse slab A from B to C",
        ),
        (
            "A",
            "C",
            _paired_faces("C", "pos_z", "A", "neg_x"),
            ("edge_pos_x_pos_z", "edge_neg_x_neg_z_from_neg_x"),
            _paired_faces("D", "pos_z", "A", "neg_z"),
            (HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi / 2.0,
            _QUARTER_TURN_SPEED_RAD_S,
            "Traverse slab A from C to D",
        ),
        (
            "A",
            "D",
            _paired_faces("D", "pos_z", "A", "neg_z"),
            ("edge_pos_x_pos_z", "edge_pos_x_neg_z"),
            _paired_faces("E", "pos_z", "A", "pos_x"),
            (HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi / 2.0,
            _QUARTER_TURN_SPEED_RAD_S,
            "Traverse slab A from D to E",
        ),
        (
            "A",
            "E",
            _paired_faces("E", "pos_z", "A", "pos_x"),
            ("edge_pos_x_pos_z", "edge_pos_x_pos_z_from_pos_x"),
            _paired_faces("F", "pos_z", "A", "pos_z"),
            (HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi / 2.0,
            _QUARTER_TURN_SPEED_RAD_S,
            "Traverse slab A from E to F",
        ),
        (
            "B",
            "C",
            _paired_faces("C", "neg_x", "B", "pos_x"),
            ("edge_neg_x_pos_z_from_neg_x", "edge_pos_x_pos_z_from_pos_x"),
            _paired_faces("C", "pos_z", "B", "pos_z"),
            (-HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi,
            _HALF_TURN_SPEED_RAD_S,
            "Lift slab B over slab C",
        ),
        (
            "B",
            "C",
            _paired_faces("C", "pos_z", "B", "pos_z"),
            ("edge_pos_x_pos_z", "edge_neg_x_pos_z"),
            _paired_faces("D", "pos_z", "B", "neg_x"),
            (HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi / 2.0,
            _QUARTER_TURN_SPEED_RAD_S,
            "Traverse slab B from C to D",
        ),
        (
            "B",
            "D",
            _paired_faces("D", "pos_z", "B", "neg_x"),
            ("edge_pos_x_pos_z", "edge_neg_x_neg_z_from_neg_x"),
            (
                *_paired_faces("E", "pos_z", "B", "neg_z"),
                *_paired_faces("A", "pos_x", "B", "pos_x"),
            ),
            (HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi / 2.0,
            _QUARTER_TURN_SPEED_RAD_S,
            "Traverse slab B from D to E and close its side faces",
        ),
        (
            "C",
            "D",
            _paired_faces("D", "neg_x", "C", "pos_x"),
            ("edge_neg_x_pos_z_from_neg_x", "edge_pos_x_pos_z_from_pos_x"),
            (
                *_paired_faces("D", "pos_z", "C", "pos_z"),
                *_paired_faces("B", "neg_x", "C", "neg_x"),
            ),
            (-HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi,
            _HALF_TURN_SPEED_RAD_S,
            "Lift slab C over D and close its side faces to B",
        ),
        (
            "C",
            "B",
            (
                *_paired_faces("D", "pos_z", "C", "pos_z"),
                *_paired_faces("B", "neg_x", "C", "neg_x"),
            ),
            ("edge_neg_x_pos_z_from_neg_x", "edge_neg_x_neg_z_from_neg_x"),
            _paired_faces("B", "pos_z", "C", "neg_z"),
            (-HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi,
            _ELEVATED_HALF_TURN_SPEED_RAD_S,
            "Lift slab C from beside B to the top tier",
        ),
        (
            "C",
            "B",
            _paired_faces("B", "pos_z", "C", "neg_z"),
            ("edge_pos_x_pos_z", "edge_pos_x_neg_z"),
            _paired_faces("A", "neg_z", "C", "pos_x"),
            (HALF_PITCH_M, 0.0, HALF_PITCH_M),
            math.pi / 2.0,
            _QUARTER_TURN_SPEED_RAD_S,
            "Traverse slab C onto A to complete the staircase",
        ),
    )

    actions: list[CoordinatedPivotAction] = []
    for index, spec in enumerate(specs):
        (
            moving_slab,
            reference_slab,
            releases,
            hinge_connectors,
            targets,
            pivot_point,
            angle,
            target_speed,
            label,
        ) = spec
        moving_modules = SLABS[moving_slab]
        reference_modules = SLABS[reference_slab]
        route = KinematicPivotRoute(
            moving_module=moving_modules[0],
            reference_module=reference_modules[0],
            pivot_point_m=pivot_point,
            pivot_axis=(0.0, 1.0, 0.0),
            angle_rad=angle,
            duration_s=2.5 if math.isclose(angle, math.pi) else 1.25,
        )
        primary_release = next(
            pair
            for pair in releases
            if split_module(pair.fixed_connector) == reference_modules[0]
            and split_module(pair.moving_connector) == moving_modules[0]
        )
        primary_target = targets[0]
        momentum = MomentumPivotConfig(
            initial_face=primary_release,
            edge_hinge=_pair(
                reference_modules[0],
                hinge_connectors[0],
                moving_modules[0],
                hinge_connectors[1],
            ),
            target_face=primary_target,
            dt_s=dt_s,
            settle_s=0.25 if index == 0 else 0.1,
            target_flywheel_speed_rad_s=target_speed,
            capture_distance_m=0.0028,
            pivot_timeout_s=3.0,
            connected_hold_s=0.35,
            target_pivot_angle_rad=angle,
            plan_id=plan_id,
            plan_name=PLAN_NAME,
        )
        actions.append(
            CoordinatedPivotAction(
                label=label,
                moving_modules=moving_modules,
                release_faces=releases,
                target_faces=targets,
                route=route,
                momentum=momentum,
            )
        )

    return CoordinatedPivotPlan(
        id=plan_id,
        name=PLAN_NAME,
        module_ids=tuple(ModuleInstanceId(f"block_{index}") for index in range(1, 13)),
        initial_connections=initial,
        actions=tuple(actions),
        source_url=SOURCE_URL,
    )


def build_kinematic_scenario(
    session: RuntimeSession,
    *,
    dt_s: float = 0.002,
) -> CoordinatedKinematicPivotScenario:
    """Create the deterministic route/reference phase."""
    return CoordinatedKinematicPivotScenario.create(
        session,
        build_plan(),
        KinematicPivotConfig(dt_s=dt_s),
    )


def build_physical_scenario(
    session: RuntimeSession,
    *,
    dt_s: float = 0.0005,
) -> CoordinatedMomentumPivotScenario:
    """Create the full MuJoCo momentum/contact phase."""
    return CoordinatedMomentumPivotScenario.create(
        session,
        build_plan(dt_s=dt_s, physical=True),
    )


def expected_pose(cell: tuple[int, int, int], quarter_turns: int = 0) -> Transform:
    """Return a grounded lattice pose useful to verification and analysis."""
    return Transform(
        translation=(
            cell[0] * NOMINAL_PITCH_M,
            cell[1] * NOMINAL_PITCH_M,
            HALF_PITCH_M + cell[2] * NOMINAL_PITCH_M,
        ),
        rotation=quat_from_axis_angle((0.0, 1.0, 0.0), quarter_turns * math.pi / 2.0),
    )


def _paired_faces(
    fixed_slab: str,
    fixed_connector: str,
    moving_slab: str,
    moving_connector: str,
) -> tuple[ConnectorPairRef, ConnectorPairRef]:
    return (
        _pair(
            SLABS[fixed_slab][0],
            fixed_connector,
            SLABS[moving_slab][0],
            moving_connector,
        ),
        _pair(
            SLABS[fixed_slab][1],
            fixed_connector,
            SLABS[moving_slab][1],
            moving_connector,
        ),
    )


def _connector(module: ModuleInstanceId, local_connector: str) -> ConnectorInstanceId:
    return connector_instance_id(module, local_connector)


def _pair(
    fixed_module: ModuleInstanceId,
    fixed_connector: str,
    moving_module: ModuleInstanceId,
    moving_connector: str,
) -> ConnectorPairRef:
    return ConnectorPairRef(
        fixed_connector=_connector(fixed_module, fixed_connector),
        moving_connector=_connector(moving_module, moving_connector),
    )


def split_module(connector: ConnectorInstanceId) -> ModuleInstanceId:
    """Return the module portion of one full connector ID."""
    return split_connector_instance_id(connector)[0]


__all__ = [
    "FINAL_CELLS",
    "HALF_PITCH_M",
    "INITIAL_CELLS",
    "KINEMATIC_PLAN_ID",
    "MODULE_COUNT",
    "MODULE_TYPE",
    "NOMINAL_PITCH_M",
    "PHYSICAL_PLAN_ID",
    "PLANNING_SOURCE_URL",
    "PLAN_NAME",
    "SLABS",
    "SOURCE_URL",
    "build_kinematic_scenario",
    "build_physical_scenario",
    "build_plan",
    "build_scene",
    "expected_pose",
]
