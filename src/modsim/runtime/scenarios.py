"""Reusable setup helpers for small runtime demonstrations.

These helpers deliberately operate on connector frames reported by a loaded
backend.  That makes them work for connectors attached to articulated links as
well as root-link connectors, without duplicating URDF forward kinematics in
the ModSim core.
"""

from __future__ import annotations

from dataclasses import dataclass

from modsim.backends.base import BackendError, SupportsModuleKinematics
from modsim.connectors.acceptance import nominal_relative_transform
from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId
from modsim.core.transforms import Transform, Vec3, vec_scale
from modsim.runtime.session import RuntimeSession


class ScenarioSetupError(ValueError):
    """Raised when a runtime scene cannot be arranged as requested."""


@dataclass(frozen=True, slots=True)
class DockingPairSetup:
    """Result of arranging one moving connector in front of a fixed connector."""

    fixed_connector: ConnectorInstanceId
    moving_connector: ConnectorInstanceId
    moving_module: ModuleInstanceId
    approach_direction: Vec3
    gap_m: float


def stage_docking_pair(
    session: RuntimeSession,
    fixed_connector: ConnectorInstanceId,
    moving_connector: ConnectorInstanceId,
    *,
    gap_m: float,
    orientation_rad: float = 0.0,
) -> DockingPairSetup:
    """Place ``moving_connector`` directly in front of ``fixed_connector``.

    The moving module is rigidly repositioned so that the connector axes are
    antiparallel, the requested roll is applied, and the connector origins are
    separated by ``gap_m`` along the fixed connector's outward docking axis.
    The returned approach direction points from the moving connector toward
    the fixed connector.

    This is scenario setup, not a docking decision. Compatibility, acceptance,
    policy, and the physical-constraint two-phase commit are still evaluated by
    the normal runtime pipeline.
    """
    if gap_m < 0.0:
        raise ScenarioSetupError("connector gap must not be negative")
    if not isinstance(session.adapter, SupportsModuleKinematics):
        raise ScenarioSetupError(
            f"backend '{session.adapter.capabilities().name}' cannot reposition modules"
        )

    try:
        fixed = session.world.connector(fixed_connector)
        moving = session.world.connector(moving_connector)
    except KeyError as error:
        raise ScenarioSetupError(str(error)) from error
    if fixed.module_id == moving.module_id:
        raise ScenarioSetupError("a docking demo requires connectors on different modules")
    if not fixed.resolved or not moving.resolved:
        raise ScenarioSetupError("connector frames must be resolved before arranging a pair")

    moving_module = session.world.modules[moving.module_id]
    module_to_connector = moving.world_pose.relative_to(moving_module.pose)
    nominal = nominal_relative_transform(fixed, moving, orientation_rad)
    target_connector = fixed.world_pose.compose(nominal)
    target_connector = Transform(
        translation=(
            fixed.world_pose.translation[0] + fixed.world_docking_axis[0] * gap_m,
            fixed.world_pose.translation[1] + fixed.world_docking_axis[1] * gap_m,
            fixed.world_pose.translation[2] + fixed.world_docking_axis[2] * gap_m,
        ),
        rotation=target_connector.rotation,
    )
    target_module = target_connector.compose(module_to_connector.inverse())

    try:
        session.adapter.set_module_pose(moving.module_id, target_module)
        session.world.ingest(session.adapter.snapshot())
    except BackendError as error:
        raise ScenarioSetupError(
            f"backend could not arrange the connector pair: {error}"
        ) from error

    return DockingPairSetup(
        fixed_connector=fixed_connector,
        moving_connector=moving_connector,
        moving_module=moving.module_id,
        approach_direction=vec_scale(fixed.world_docking_axis, -1.0),
        gap_m=gap_m,
    )
