"""Stable names for Runtime Inspector demonstration entry points."""

from enum import StrEnum


class RuntimeDemo(StrEnum):
    """Demonstrations selectable by the Runtime Inspector CLI."""

    DOCK = "dock"
    DOCK_UNDOCK = "dock_undock"
    SMORES_DRIVER_TO_SNAKE = "smores_driver_to_snake"


__all__ = ["RuntimeDemo"]
