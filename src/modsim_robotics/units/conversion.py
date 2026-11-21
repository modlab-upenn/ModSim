"""Convert explicit user units to and from canonical SI numbers.

These helpers belong at authoring and presentation boundaries. Their returned
values are ordinary floats; runtime state and simulator stepping do not carry
quantity wrappers. Converting angular units preserves complete revolutions.
"""

from __future__ import annotations

import math
import re
from collections.abc import Sequence
from numbers import Real

from modsim_robotics.units.definitions import QuantityKind, UnitError, get_unit

_NUMBER = r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?"
_QUANTITY = re.compile(rf"\s*(?P<value>{_NUMBER})\s*(?P<unit>.*?)\s*\Z")


def _match_quantity(text: object) -> re.Match[str]:
    """Require numeric text, retaining dynamic validation for Python callers."""
    if not isinstance(text, str):
        raise UnitError("Quantity input must be numeric text with an optional unit suffix.")
    match = _QUANTITY.fullmatch(text)
    if match is None:
        raise UnitError("Enter a finite number followed by an optional supported unit.")
    return match


def _display_precision(value: object) -> int:
    """Validate a significant-digit count independently of typed signatures."""
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 17:
        raise UnitError("Display precision must be an integer from 1 through 17.")
    return value


def _finite(value: float) -> float:
    """Return a finite numeric magnitude, excluding booleans and numeric strings."""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise UnitError("Quantity value must be a finite number, not a boolean or string.")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise UnitError("Quantity value must be a finite number.") from error
    if not math.isfinite(result):
        raise UnitError("Quantity value must be finite; conversion may have overflowed.")
    return result


def convert(value: float, from_unit: str, to_unit: str) -> float:
    """Convert a numeric magnitude between compatible supported units.

    Args:
        value: Finite source magnitude. Booleans and numeric strings are rejected.
        from_unit: Supported source symbol, such as ``mm`` or ``deg``.
        to_unit: Supported destination symbol for the same physical quantity.

    Returns:
        A finite float in the destination unit. Angles retain multiple turns.

    Raises:
        UnitError: A unit is unknown, quantities differ, or the value/result is non-finite.
    """
    source = get_unit(from_unit)
    target = get_unit(to_unit, kind=source.kind)
    return _finite(_finite(value) * (source.factor / target.factor))


def to_si(value: float, unit: str, *, kind: QuantityKind | None = None) -> float:
    """Normalize an explicit magnitude to the quantity's canonical SI unit.

    Args:
        value: Finite magnitude expressed in ``unit``; booleans and strings are rejected.
        unit: Supported unit symbol. For example, ``rpm`` maps to radians per second.
        kind: Expected quantity kind, or ``None`` to infer it from the unit.

    Returns:
        The finite canonical magnitude: m, rad, m/s, rad/s, s, kg, N, N*m,
        kg*m^2, m/s^2, N/m, N*m/rad, N*s/m, N*m*s/rad, or Hz.

    Raises:
        UnitError: The unit is unknown/incompatible, or the value/result is non-finite.
    """
    definition = get_unit(unit, kind=kind)
    return _finite(_finite(value) * definition.factor)


def from_si(value: float, unit: str, *, kind: QuantityKind | None = None) -> float:
    """Express a canonical SI magnitude in a supported display unit.

    Args:
        value: Finite magnitude in the quantity's canonical SI unit.
        unit: Supported destination symbol, such as ``cm`` or ``deg``.
        kind: Expected quantity kind, or ``None`` to infer it from the unit.

    Returns:
        A finite magnitude in the requested unit; the input is never mutated or wrapped.

    Raises:
        UnitError: The unit is unknown/incompatible, or the value/result is non-finite.
    """
    definition = get_unit(unit, kind=kind)
    return _finite(_finite(value) / definition.factor)


def parse_quantity(text: str, *, kind: QuantityKind, default_unit: str) -> float:
    """Parse a number with an optional supported suffix into canonical SI.

    Scientific notation and attached suffixes such as ``2.5e1mm`` are accepted.
    This parser never evaluates expressions or infers units from value size.

    Args:
        text: Numeric text, optionally followed by a unit such as ``90 deg`` or ``50mm``.
        kind: Physical quantity expected by the field, independent of display preferences.
        default_unit: Supported unit used only when the text has no suffix.

    Returns:
        A finite canonical SI magnitude, preserving signs and complete revolutions.

    Raises:
        UnitError: Text is malformed, units are unsupported/incompatible, or conversion
            produces a non-finite value. The default unit must match ``kind`` even when
            an explicit suffix is supplied.

    Examples:
        >>> parse_quantity("50 mm", kind=QuantityKind.LENGTH, default_unit="m")
        0.05
    """
    get_unit(default_unit, kind=kind)
    match = _match_quantity(text)
    unit = match.group("unit").strip() or default_unit
    return to_si(float(match.group("value")), unit, kind=kind)


def format_quantity(value_si: float, unit: str, *, precision: int = 12) -> str:
    """Format a canonical SI value with a supported unit's canonical symbol.

    Args:
        value_si: Finite canonical magnitude to display. Its stored value is untouched.
        unit: Supported display symbol or alias, such as ``mm`` or ``°``.
        precision: Number of significant display digits, from 1 through 17.

    Returns:
        Numeric text followed by a space and the unit symbol, for example ``50 mm``.
        Degree aliases are displayed using the canonical ``deg`` symbol.

    Raises:
        UnitError: A value/unit is invalid or precision is not an integer in 1..17.
    """
    digits = _display_precision(precision)
    definition = get_unit(unit)
    magnitude = from_si(value_si, definition.symbol)
    # Suppress a distracting negative zero without changing the caller's value.
    number = 0.0 if magnitude == 0.0 else magnitude
    return f"{number:.{digits}g} {definition.symbol}"


def to_si_vector(
    values: Sequence[float], unit: str, *, kind: QuantityKind
) -> tuple[float, float, float]:
    """Normalize three components of one physical quantity into canonical SI.

    Args:
        values: Three finite components expressed in ``unit``; the input is not mutated.
        unit: Supported source unit applied independently to each component.
        kind: Expected physical quantity, normally length or angle for authoring.

    Returns:
        A new three-component tuple in canonical SI. Coordinate frames are unchanged.

    Raises:
        UnitError: There are not exactly three components, the unit is incompatible,
            or a component/result is non-finite or nonnumeric.
    """
    if len(values) != 3:
        raise UnitError("A physical vector requires exactly three components.")
    return (
        to_si(values[0], unit, kind=kind),
        to_si(values[1], unit, kind=kind),
        to_si(values[2], unit, kind=kind),
    )


def from_si_vector(
    values: Sequence[float], unit: str, *, kind: QuantityKind
) -> tuple[float, float, float]:
    """Express three canonical components in a common display unit.

    Args:
        values: Three finite canonical SI components; the input is not mutated.
        unit: Supported display unit applied independently to each component.
        kind: Expected physical quantity, normally length or angle for authoring.

    Returns:
        A new three-component tuple in the chosen unit. This does not convert a
        quaternion, change a coordinate frame, or wrap an angle.

    Raises:
        UnitError: There are not exactly three components, the unit is incompatible,
            or a component/result is non-finite or nonnumeric.
    """
    if len(values) != 3:
        raise UnitError("A physical vector requires exactly three components.")
    return (
        from_si(values[0], unit, kind=kind),
        from_si(values[1], unit, kind=kind),
        from_si(values[2], unit, kind=kind),
    )
