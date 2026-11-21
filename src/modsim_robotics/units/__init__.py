"""Map explicit authoring and display units onto ModSim's canonical SI numbers.

Use conversion at input/output boundaries, with ordinary floats remaining in
runtime state. Rotation representation and coordinate frame changes belong to
the existing transform utilities rather than to unit conversion.
"""

from modsim_robotics.units.conversion import (
    convert,
    format_quantity,
    from_si,
    from_si_vector,
    parse_quantity,
    to_si,
    to_si_vector,
)
from modsim_robotics.units.definitions import (
    QuantityKind,
    UnitDefinition,
    UnitError,
    get_unit,
    get_units,
)
from modsim_robotics.units.fields import joint_quantity_kind, joint_value_to_si, quantity_for_field
from modsim_robotics.units.preferences import UnitPreferences

__all__ = [
    "QuantityKind",
    "UnitDefinition",
    "UnitError",
    "UnitPreferences",
    "convert",
    "format_quantity",
    "from_si",
    "from_si_vector",
    "get_unit",
    "get_units",
    "joint_quantity_kind",
    "joint_value_to_si",
    "parse_quantity",
    "quantity_for_field",
    "to_si",
    "to_si_vector",
]
