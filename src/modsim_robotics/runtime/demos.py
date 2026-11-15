"""Stable names for Runtime Inspector demonstration entry points."""

from enum import StrEnum


class RuntimeDemo(StrEnum):
    """Demonstrations selectable by the Runtime Inspector CLI."""

    DOCK = "dock"
    DOCK_UNDOCK = "dock_undock"
    SMORES_DIFF_DRIVE_DOCK_UNDOCK = "smores_diff_drive_dock_undock"
    SMORES_DRIVER_TO_SNAKE = "smores_driver_to_snake"
    SMORES_PHYSICAL_DRIVER_TO_SNAKE = "smores_physical_driver_to_snake"
    SMORES_ONLINE_ASSEMBLY = "smores_online_assembly"
    SMORES_ONLINE_DRIVER_TO_SNAKE = "smores_online_driver_to_snake"
    MBLOCKS_FIVE_MODULE_PIVOT = "mblocks_five_module_pivot"
    MBLOCKS_MOMENTUM_PIVOT = "mblocks_momentum_pivot"
    MBLOCKS_PHYSICAL_TWELVE_MODULE_LINE = "mblocks_physical_twelve_module_line"
    MBLOCKS_PHYSICAL_TWELVE_MODULE_STAIRCASE = "mblocks_physical_twelve_module_staircase"
    MBLOCKS_TWELVE_MODULE_LINE = "mblocks_twelve_module_line"
    MBLOCKS_TWELVE_MODULE_STAIRCASE = "mblocks_twelve_module_staircase"
    MBLOCKS_ONLINE_LATTICE = "mblocks_online_lattice"
    MBLOCKS_ONLINE_LATTICE_LARGE = "mblocks_online_lattice_large"
    MBLOCKS_ONLINE_LATTICE_ELBOW = "mblocks_online_lattice_elbow"


__all__ = ["RuntimeDemo"]
