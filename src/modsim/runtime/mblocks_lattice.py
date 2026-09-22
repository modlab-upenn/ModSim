"""Measured tabletop lattice observations and generated connector actions.

The planner's XY frame follows a named stationary anchor. The pack's local
+Y flywheel points along world +Z, so every quarter turn stays in one plane.
This module only observes poses; it never changes a module's root state.
"""

from __future__ import annotations

import math

from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, connector_instance_id
from modsim.core.scene import ModulePlacement, SceneSpec
from modsim.core.transforms import (
    Transform,
    Vec3,
    quat_angle,
    quat_conjugate,
    quat_from_axis_angle,
    quat_multiply,
    quat_rotate,
    vec_dot,
    vec_norm,
    vec_sub,
)
from modsim.planning.mblocks.geometry import apply_pivot, neighbors
from modsim.planning.mblocks.models import (
    Cell,
    LatticeBlock,
    LatticeGoal,
    LatticePivot,
    LatticeState,
)
from modsim.runtime.coordinated_pivot import CoordinatedPivotAction, CoordinatedPivotPlan
from modsim.runtime.kinematic_pivot import KinematicPivotRoute
from modsim.runtime.momentum_pivot import MomentumPivotConfig
from modsim.runtime.reconfiguration import ConnectorPairRef, ReconfigurationScenarioError
from modsim.runtime.session import RuntimeSession

TABLETOP_ROTATION = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi / 2)
PLAN_ID = "mblocks_online_lattice"
PLAN_NAME = "M-Blocks online planar reconfiguration"
SOURCE_URL = "https://doi.org/10.1109/ICRA.2015.7139451"
_FACE_AXES: dict[str, Vec3] = {
    "pos_x": (1.0, 0.0, 0.0),
    "neg_x": (-1.0, 0.0, 0.0),
    "pos_y": (0.0, 1.0, 0.0),
    "neg_y": (0.0, -1.0, 0.0),
    "pos_z": (0.0, 0.0, 1.0),
    "neg_z": (0.0, 0.0, -1.0),
}


def rectangle_state(width: int = 3, height: int = 2) -> LatticeState:
    if width < 1 or height < 1 or width * height > 32:
        raise ValueError("rectangle must contain between 2 and 32 blocks")
    return LatticeState(
        blocks=tuple(
            LatticeBlock(id=f"block_{y * width + x + 1}", cell=(x, y))
            for y in range(height)
            for x in range(width)
        )
    )


def line_goal(state: LatticeState) -> LatticeGoal:
    x, y = max(state.cells, key=lambda cell: (cell[1], cell[0]))
    return LatticeGoal(id="line", cells=tuple((x, y + i) for i in range(len(state.blocks))))


def starter_state() -> LatticeState:
    """Small physical baseline; larger rectangles are geometric/stress fixtures."""
    return rectangle_state(2, 2)


def larger_state() -> LatticeState:
    """Six-cube rectangle with five moving cubes and 21 generated pivots."""
    return rectangle_state(2, 3)


def elbow_goal(state: LatticeState | None = None) -> LatticeGoal:
    selected = starter_state() if state is None else state
    x, y = max(selected.cells, key=lambda cell: (cell[1], cell[0]))
    width = min(3, len(selected.blocks) - 1, max(2, x + 1))
    return LatticeGoal(
        id="elbow",
        cells=tuple((x - i, y) for i in range(width))
        + tuple((x - width + 1, y - i) for i in range(1, len(selected.blocks) - width + 1)),
    )


def block_pose(block: LatticeBlock, frame: Transform, pitch_m: float) -> Transform:
    return frame.compose(
        Transform(
            translation=(block.cell[0] * pitch_m, block.cell[1] * pitch_m, 0.0),
            rotation=quat_multiply(
                quat_from_axis_angle((0.0, 0.0, 1.0), block.quarter_turns * math.pi / 2),
                TABLETOP_ROTATION,
            ),
        )
    )


def tabletop_scene(
    state: LatticeState | None = None,
    *,
    pitch_m: float = 0.05,
    height_m: float = 0.025,
    module_type: str = "mblocks_3d",
) -> SceneSpec:
    selected = starter_state() if state is None else state
    frame = Transform.from_translation((0.0, 0.0, height_m))
    return SceneSpec.of(
        ModulePlacement(
            instance_id=ModuleInstanceId(block.id),
            module_type_id=module_type,
            pose=block_pose(block, frame, pitch_m),
        )
        for block in selected.blocks
    )


def observation_frame(
    session: RuntimeSession,
    anchor: str,
    anchor_cell: Cell,
    anchor_turns: int,
    pitch_m: float,
) -> Transform:
    pose = session.world.modules[ModuleInstanceId(anchor)].pose
    axis = quat_rotate(pose.rotation, (1.0, 0.0, 0.0))
    yaw = math.atan2(axis[1], axis[0]) - anchor_turns * math.pi / 2
    rotation = quat_from_axis_angle((0.0, 0.0, 1.0), yaw)
    offset = quat_rotate(rotation, (anchor_cell[0] * pitch_m, anchor_cell[1] * pitch_m, 0.0))
    return Transform(translation=vec_sub(pose.translation, offset), rotation=rotation)


def observe_lattice(
    session: RuntimeSession,
    frame: Transform,
    pitch_m: float,
    *,
    position_tolerance_m: float = 0.003,
    orientation_tolerance_rad: float = math.radians(5),
) -> tuple[LatticeState, float]:
    blocks: list[LatticeBlock] = []
    residual = 0.0
    for module in session.world.modules.values():
        local = module.pose.relative_to(frame)
        x, y, _ = local.translation
        cell = (_round_cell(x / pitch_m), _round_cell(y / pitch_m))
        distance = vec_norm(vec_sub(local.translation, (cell[0] * pitch_m, cell[1] * pitch_m, 0.0)))
        angles = tuple(
            quat_angle(
                quat_multiply(
                    quat_conjugate(local.rotation),
                    block_pose(
                        LatticeBlock(id=module.id, cell=(0, 0), quarter_turns=turns),
                        Transform.identity(),
                        pitch_m,
                    ).rotation,
                )
            )
            for turns in range(4)
        )
        turns = min(range(4), key=lambda index: angles[index])
        if distance > position_tolerance_m or angles[turns] > orientation_tolerance_rad:
            raise ReconfigurationScenarioError(
                f"{module.id} is off the tabletop lattice: {distance * 1000:.2f} mm, "
                f"{math.degrees(angles[turns]):.2f} degrees"
            )
        blocks.append(LatticeBlock(id=module.id, cell=cell, quarter_turns=turns))
        residual = max(residual, distance)
    return LatticeState(blocks=tuple(blocks)), residual


def face_pairs(
    state: LatticeState, frame: Transform, pitch_m: float
) -> tuple[ConnectorPairRef, ...]:
    poses = {block.id: block_pose(block, frame, pitch_m) for block in state.blocks}
    pairs: list[ConnectorPairRef] = []
    for block in state.blocks:
        for adjacent in sorted(set(neighbors(block.cell)) & state.cells):
            other = state.at(adjacent)
            if block.id >= other.id:
                continue
            direction = frame.apply_direction(
                (
                    float(adjacent[0] - block.cell[0]),
                    float(adjacent[1] - block.cell[1]),
                    0.0,
                )
            )
            pairs.append(
                ConnectorPairRef(
                    fixed_connector=connector_instance_id(
                        ModuleInstanceId(block.id), _face(poses[block.id], direction)
                    ),
                    moving_connector=connector_instance_id(
                        ModuleInstanceId(other.id),
                        _face(poses[other.id], (-direction[0], -direction[1], -direction[2])),
                    ),
                )
            )
    return tuple(pairs)


def make_pivot_plan(
    session: RuntimeSession,
    state: LatticeState,
    action: LatticePivot,
    frame: Transform,
    *,
    pitch_m: float = 0.05,
    dt_s: float = 0.0005,
    quarter_rpm: float = 14000.0,
    half_rpm: float = 17000.0,
) -> CoordinatedPivotPlan:
    """Compile a legal geometric move into measured lifecycle/effort commands."""
    after = apply_pivot(state, action)
    current = face_pairs(state, frame, pitch_m)
    expected = {pair.connection_id for pair in current}
    if expected != set(session.world.connections):
        raise ReconfigurationScenarioError(
            "Measured face topology does not match lattice adjacency"
        )
    moving, support = ModuleInstanceId(action.moving), ModuleInstanceId(action.support)
    releases = tuple(_toward(pair, moving) for pair in current if _contains(pair, moving))
    targets = tuple(
        _toward(pair, moving)
        for pair in face_pairs(after, frame, pitch_m)
        if _contains(pair, moving)
    )
    initial_face = next(pair for pair in releases if pair.fixed_connector.startswith(f"{support}/"))
    pivot = frame.apply_point(
        (action.pivot_twice[0] * pitch_m / 2, action.pivot_twice[1] * pitch_m / 2, 0.0)
    )
    support_pose = block_pose(next(b for b in state.blocks if b.id == support), frame, pitch_m)
    moving_pose = block_pose(next(b for b in state.blocks if b.id == moving), frame, pitch_m)
    support_hinge = _hinge(session, support, support_pose, pivot, initial_face.fixed_connector)
    moving_hinge = _hinge(session, moving, moving_pose, pivot, initial_face.moving_connector)
    sign = 1 if action.turns > 0 else -1
    fixed_rotation = session.world.modules[support].pose.rotation
    axis = quat_rotate(quat_conjugate(fixed_rotation), (0.0, 0.0, float(sign)))
    config = MomentumPivotConfig(
        initial_face=initial_face,
        edge_hinge=ConnectorPairRef(fixed_connector=support_hinge, moving_connector=moving_hinge),
        target_face=targets[0],
        dt_s=dt_s,
        target_flywheel_speed_rad_s=(quarter_rpm if abs(action.turns) == 1 else half_rpm)
        * math.tau
        / 60,
        spin_direction=sign,
        pivot_axis_in_fixed_frame=axis,
        pivot_reference_in_moving_frame=(1.0, 0.0, 0.0),
        target_pivot_angle_rad=abs(action.turns) * math.pi / 2,
        capture_distance_m=0.001,
        settle_s=0.05,
        connected_hold_s=0.3,
        pivot_timeout_s=3.0,
        plan_id=PLAN_ID,
        plan_name=PLAN_NAME,
    )
    route = KinematicPivotRoute(
        moving_module=moving,
        reference_module=support,
        pivot_point_m=session.world.modules[support].pose.inverse().apply_point(pivot),
        pivot_axis=axis,
        angle_rad=abs(action.turns) * math.pi / 2,
        duration_s=1.0,
    )
    return CoordinatedPivotPlan(
        id=PLAN_ID,
        name=PLAN_NAME,
        module_ids=tuple(session.world.modules),
        initial_connections=current,
        actions=(
            CoordinatedPivotAction(
                label=f"{action.moving}: {action.source} → {action.destination}",
                moving_modules=(moving,),
                release_faces=releases,
                target_faces=targets,
                route=route,
                momentum=config,
            ),
        ),
        source_url=SOURCE_URL,
    )


def _face(pose: Transform, direction: Vec3) -> str:
    local = quat_rotate(quat_conjugate(pose.rotation), direction)
    name = max(_FACE_AXES, key=lambda key: vec_dot(_FACE_AXES[key], local))
    if vec_dot(_FACE_AXES[name], local) < 0.999:
        raise ReconfigurationScenarioError("Face direction is not aligned to the lattice")
    return name


def _hinge(
    session: RuntimeSession,
    module: ModuleInstanceId,
    pose: Transform,
    pivot: Vec3,
    face: ConnectorInstanceId,
) -> ConnectorInstanceId:
    desired = pose.inverse().apply_point(pivot)
    face_axis = session.world.connector(face).world_docking_axis
    candidates = [
        connector
        for connector in session.world.connectors.values()
        if connector.module_id == module
        and connector.connector_type_id == "mblock_magnetic_edge_hinge"
        and vec_norm(vec_sub(connector.local_pose.translation, desired)) < 1e-6
        and vec_dot(connector.world_docking_axis, face_axis) > 0.98
    ]
    if len(candidates) != 1:
        raise ReconfigurationScenarioError(f"No unique supported hinge at {desired} on {module}")
    return candidates[0].id


def _contains(pair: ConnectorPairRef, module: ModuleInstanceId) -> bool:
    return any(connector.startswith(f"{module}/") for connector in pair.connectors)


def _toward(pair: ConnectorPairRef, moving: ModuleInstanceId) -> ConnectorPairRef:
    if pair.moving_connector.startswith(f"{moving}/"):
        return pair
    return ConnectorPairRef(
        fixed_connector=pair.moving_connector, moving_connector=pair.fixed_connector
    )


def _round_cell(value: float) -> int:
    return math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5)
