"""Represent user-selected authoring/display units without changing pack semantics."""

from __future__ import annotations

from dataclasses import dataclass, fields
from types import MappingProxyType

from modsim_robotics.units.definitions import QuantityKind, UnitError, get_unit

_PREFERENCE_KINDS = MappingProxyType({f"{kind.value}_unit": kind for kind in QuantityKind})
_DERIVED = frozenset({"linear_velocity_unit", "angular_velocity_unit", "linear_acceleration_unit"})


def _preference_key(kind: object) -> str:
    if not isinstance(kind, QuantityKind):
        raise UnitError(f"Unsupported quantity kind {kind!r}.")
    return f"{kind.value}_unit"


@dataclass(frozen=True, slots=True)
class UnitPreferences:
    """Immutable user display choices for physical quantities.

    Preferences describe input and presentation only. They do not reinterpret
    ``xyz_m`` or ``rpy_rad`` fields, change mesh scale, or choose a rotation frame.

    Args:
        length_unit: One of ``mm``, ``cm``, or ``m``. Defaults to millimeters.
        angle_unit: One of ``deg`` or ``rad``. Defaults to degrees.
        time_unit: Seconds or milliseconds; defaults to seconds.
        mass_unit: Kilograms or grams; defaults to kilograms.
        force_unit: Supported force symbol; defaults to N.
        torque_unit: Supported torque symbol; defaults to N*m.
        inertia_unit: Supported moment-of-inertia symbol; defaults to kg*m^2.
        linear_velocity_unit: Speed unit, or None to follow length per second.
        angular_velocity_unit: Angular speed, or None to follow angle per second.
        linear_acceleration_unit: Acceleration, or None to follow length per second squared.
        linear_stiffness_unit: Linear spring constant unit; defaults to N/m.
        angular_stiffness_unit: Angular spring constant unit; defaults to N*m/rad.
        linear_damping_unit: Linear viscous damping unit; defaults to N*s/m.
        angular_damping_unit: Angular viscous damping unit; defaults to N*m*s/rad.
        frequency_unit: Cyclic frequency unit; defaults to Hz, distinct from angular velocity.

    Raises:
        UnitError: A preference is not one of the supported canonical symbols.
    """

    length_unit: str = "mm"
    angle_unit: str = "deg"
    time_unit: str = "s"
    mass_unit: str = "kg"
    force_unit: str = "N"
    torque_unit: str = "N*m"
    inertia_unit: str = "kg*m^2"
    linear_velocity_unit: str | None = None
    angular_velocity_unit: str | None = None
    linear_acceleration_unit: str | None = None
    linear_stiffness_unit: str = "N/m"
    angular_stiffness_unit: str = "N*m/rad"
    linear_damping_unit: str = "N*s/m"
    angular_damping_unit: str = "N*m*s/rad"
    frequency_unit: str = "Hz"

    def __post_init__(self) -> None:
        """Validate display preferences without accessing or creating user files.

        Raises:
            UnitError: A length or angular preference is unsupported.
        """
        for field in fields(self):
            value = getattr(self, field.name)
            if value is None and field.name in _DERIVED:
                continue
            if not isinstance(value, str):
                raise UnitError(f"{field.name} requires a supported unit symbol.")
            definition = get_unit(value, kind=_PREFERENCE_KINDS[field.name])
            if definition.symbol != value:
                raise UnitError(f"{field.name} requires canonical symbol {definition.symbol!r}.")

    def unit_for(self, kind: QuantityKind) -> str:
        """Resolve a physical quantity to its explicit or derived display unit.

        Args:
            kind: Quantity being edited or presented, established by typed field metadata.

        Returns:
            A validated canonical registry symbol. Derived speeds use seconds even
            when the time display preference is milliseconds.

        Raises:
            UnitError: The requested quantity kind is unsupported.
        """
        key = _preference_key(kind)
        if key not in _PREFERENCE_KINDS:
            raise UnitError(f"Unsupported quantity kind {kind!r}.")
        value = getattr(self, key)
        if isinstance(value, str):
            return value
        if kind == QuantityKind.LINEAR_VELOCITY:
            return self.length_unit + "/s"
        if kind == QuantityKind.ANGULAR_VELOCITY:
            return self.angle_unit + "/s"
        return self.length_unit + "/s^2"
