"""SMORES-EP Driver-to-Snake example plan from Liu, Whitzer, and Yim (2019)."""

from modsim.core.ids import ConnectorInstanceId, ModuleInstanceId, connector_instance_id
from modsim.runtime.reconfiguration import (
    ConnectorPairRef,
    ReconfigurationAction,
    ReconfigurationPlan,
)

SOURCE_URL = (
    "https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf"
)


def build_plan() -> ReconfigurationPlan:
    """Return the seven-module Driver-to-Snake demonstration plan."""
    return ReconfigurationPlan(
        id="smores_driver_to_snake",
        name="SMORES-EP Driver to Snake",
        module_ids=tuple(_module(index) for index in range(1, 8)),
        initial_connections=(
            _pair(2, "pan", 1, "bottom"),
            _pair(2, "bottom", 3, "pan"),
            _pair(2, "right", 4, "left"),
            _pair(4, "right", 5, "left"),
            _pair(5, "pan", 6, "bottom"),
            _pair(5, "bottom", 7, "pan"),
        ),
        actions=(
            ReconfigurationAction(
                label="Move module 1 from module 2 to module 3",
                undock=_pair(2, "pan", 1, "bottom"),
                dock=_pair(3, "bottom", 1, "pan"),
            ),
            ReconfigurationAction(
                label="Move module 7 from module 5 to module 6",
                undock=_pair(5, "bottom", 7, "pan"),
                dock=_pair(6, "pan", 7, "bottom"),
            ),
            ReconfigurationAction(
                label="Move the 1-3-2 subassembly onto module 4",
                undock=_pair(4, "left", 2, "right"),
                dock=_pair(4, "bottom", 2, "pan"),
            ),
            ReconfigurationAction(
                label="Move the 5-6-7 subassembly onto module 4",
                undock=_pair(4, "right", 5, "left"),
                dock=_pair(4, "pan", 5, "bottom"),
            ),
        ),
        source_url=SOURCE_URL,
    )


def _module(index: int) -> ModuleInstanceId:
    return ModuleInstanceId(f"module_{index}")


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


__all__ = ["SOURCE_URL", "build_plan"]
