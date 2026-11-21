"""Define supported physical quantity kinds and their immutable unit scales.

Angles remain a separate semantic kind from dimensionless numbers. In
particular, angular velocity is distinct from cyclic frequency. Unit conversion
does not change coordinate frames, geometry scale, or rotation conventions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from types import MappingProxyType


class UnitError(ValueError):
    """Reject an unsupported unit, incompatible quantity, or invalid numeric input."""


class QuantityKind(StrEnum):
    """Distinguish physical quantities accepted at ModSim input/output boundaries."""

    LENGTH = "length"
    ANGLE = "angle"
    LINEAR_VELOCITY = "linear_velocity"
    ANGULAR_VELOCITY = "angular_velocity"
    TIME = "time"
    MASS = "mass"
    FORCE = "force"
    TORQUE = "torque"
    INERTIA = "inertia"
    LINEAR_ACCELERATION = "linear_acceleration"
    LINEAR_STIFFNESS = "linear_stiffness"
    ANGULAR_STIFFNESS = "angular_stiffness"
    LINEAR_DAMPING = "linear_damping"
    ANGULAR_DAMPING = "angular_damping"
    FREQUENCY = "frequency"


@dataclass(frozen=True, slots=True)
class UnitDefinition:
    """Describe an immutable supported unit's mapping to canonical SI.

    Args:
        symbol: Canonical symbol used when formatting a magnitude.
        kind: Semantic physical quantity, with angles distinct from scalar ratios.
        factor: Multiplicative factor from this unit to the quantity's canonical SI unit.
        aliases: Additional accepted symbols. Registry lookup remains case-sensitive.
    """

    symbol: str
    kind: QuantityKind
    factor: float
    aliases: tuple[str, ...] = ()


_DEFINITIONS = (
    UnitDefinition("m", QuantityKind.LENGTH, 1.0),
    UnitDefinition("cm", QuantityKind.LENGTH, 1e-2),
    UnitDefinition("mm", QuantityKind.LENGTH, 1e-3),
    UnitDefinition("rad", QuantityKind.ANGLE, 1.0),
    UnitDefinition("deg", QuantityKind.ANGLE, math.pi / 180.0, ("°",)),
    UnitDefinition("m/s", QuantityKind.LINEAR_VELOCITY, 1.0),
    UnitDefinition("cm/s", QuantityKind.LINEAR_VELOCITY, 1e-2),
    UnitDefinition("mm/s", QuantityKind.LINEAR_VELOCITY, 1e-3),
    UnitDefinition("rad/s", QuantityKind.ANGULAR_VELOCITY, 1.0),
    UnitDefinition("deg/s", QuantityKind.ANGULAR_VELOCITY, math.pi / 180.0, ("°/s",)),
    UnitDefinition("rpm", QuantityKind.ANGULAR_VELOCITY, math.tau / 60.0, ("RPM",)),
    UnitDefinition("s", QuantityKind.TIME, 1.0),
    UnitDefinition("ms", QuantityKind.TIME, 1e-3),
    UnitDefinition("kg", QuantityKind.MASS, 1.0),
    UnitDefinition("g", QuantityKind.MASS, 1e-3),
    UnitDefinition("N", QuantityKind.FORCE, 1.0),
    UnitDefinition("mN", QuantityKind.FORCE, 1e-3),
    UnitDefinition("kN", QuantityKind.FORCE, 1e3),
    UnitDefinition("N*m", QuantityKind.TORQUE, 1.0, ("N·m", "Nm", "N m")),
    UnitDefinition("N*mm", QuantityKind.TORQUE, 1e-3, ("N·mm", "Nmm", "N mm")),
    UnitDefinition("kg*m^2", QuantityKind.INERTIA, 1.0, ("kg·m²", "kg*m²", "kg m^2")),
    UnitDefinition("kg*mm^2", QuantityKind.INERTIA, 1e-6, ("kg·mm²", "kg*mm²", "kg mm^2")),
    UnitDefinition("m/s^2", QuantityKind.LINEAR_ACCELERATION, 1.0, ("m/s²",)),
    UnitDefinition("cm/s^2", QuantityKind.LINEAR_ACCELERATION, 1e-2, ("cm/s²",)),
    UnitDefinition("mm/s^2", QuantityKind.LINEAR_ACCELERATION, 1e-3, ("mm/s²",)),
    UnitDefinition("N/m", QuantityKind.LINEAR_STIFFNESS, 1.0),
    UnitDefinition("N/cm", QuantityKind.LINEAR_STIFFNESS, 1e2),
    UnitDefinition("N/mm", QuantityKind.LINEAR_STIFFNESS, 1e3),
    UnitDefinition("N*m/rad", QuantityKind.ANGULAR_STIFFNESS, 1.0, ("N·m/rad",)),
    UnitDefinition("N*m/deg", QuantityKind.ANGULAR_STIFFNESS, 180.0 / math.pi, ("N·m/deg",)),
    UnitDefinition("N*mm/rad", QuantityKind.ANGULAR_STIFFNESS, 1e-3, ("N·mm/rad",)),
    UnitDefinition("N*mm/deg", QuantityKind.ANGULAR_STIFFNESS, 0.18 / math.pi, ("N·mm/deg",)),
    UnitDefinition("N*s/m", QuantityKind.LINEAR_DAMPING, 1.0, ("N·s/m",)),
    UnitDefinition("N*s/cm", QuantityKind.LINEAR_DAMPING, 1e2, ("N·s/cm",)),
    UnitDefinition("N*s/mm", QuantityKind.LINEAR_DAMPING, 1e3, ("N·s/mm",)),
    UnitDefinition("N*m*s/rad", QuantityKind.ANGULAR_DAMPING, 1.0, ("N·m·s/rad",)),
    UnitDefinition("N*m*s/deg", QuantityKind.ANGULAR_DAMPING, 180.0 / math.pi, ("N·m·s/deg",)),
    UnitDefinition("N*mm*s/rad", QuantityKind.ANGULAR_DAMPING, 1e-3, ("N·mm·s/rad",)),
    UnitDefinition("N*mm*s/deg", QuantityKind.ANGULAR_DAMPING, 0.18 / math.pi, ("N·mm·s/deg",)),
    UnitDefinition("Hz", QuantityKind.FREQUENCY, 1.0),
    UnitDefinition("kHz", QuantityKind.FREQUENCY, 1e3),
)

_UNITS = MappingProxyType(
    {name: unit for unit in _DEFINITIONS for name in (unit.symbol, *unit.aliases)}
)


def _checked_symbol(value: object) -> str:
    """Reject nontext unit identifiers at public API boundaries."""
    if not isinstance(value, str):
        raise UnitError("Unit must be a supported unit symbol.")
    return value.strip()


def get_unit(unit: str, *, kind: QuantityKind | None = None) -> UnitDefinition:
    """Look up immutable unit metadata without changing the supported registry.

    Args:
        unit: Supported canonical symbol or alias. Surrounding whitespace is ignored.
        kind: Expected physical quantity, or ``None`` to accept the unit's declared kind.

    Returns:
        The shared frozen definition containing the canonical symbol and SI scale.

    Raises:
        UnitError: The symbol is unsupported or belongs to another physical quantity.
    """
    try:
        definition = _UNITS[_checked_symbol(unit)]
    except KeyError as error:
        raise UnitError(f"Unsupported unit {unit!r}.") from error
    if kind is not None and definition.kind != kind:
        raise UnitError(f"Unit {definition.symbol!r} measures {definition.kind.value}, not {kind}.")
    return definition


def get_units(kind: QuantityKind) -> tuple[UnitDefinition, ...]:
    """List supported canonical unit definitions for a semantic quantity.

    Args:
        kind: Quantity whose unit choices should be offered to a user.

    Returns:
        An immutable tuple in registry order. Aliases are not duplicated.

    Raises:
        UnitError: The requested quantity kind is unsupported.
    """
    definitions = tuple(unit for unit in _DEFINITIONS if unit.kind == kind)
    if not definitions:
        raise UnitError(f"Unsupported quantity kind {kind!r}.")
    return definitions
