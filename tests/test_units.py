"""Verify explicit unit conversion preserves physical values at boundaries."""

from __future__ import annotations

import math
from dataclasses import FrozenInstanceError
from typing import cast

import pytest

from modsim_robotics.units import (
    QuantityKind,
    UnitError,
    UnitPreferences,
    convert,
    format_quantity,
    from_si,
    from_si_vector,
    parse_quantity,
    to_si,
    to_si_vector,
)


@pytest.mark.parametrize(
    ("value", "unit", "kind", "expected"),
    [
        (25.0, "mm", QuantityKind.LENGTH, 0.025),
        (5.0, "cm", QuantityKind.LENGTH, 0.05),
        (90.0, "deg", QuantityKind.ANGLE, math.pi / 2),
        (90.0, "°", QuantityKind.ANGLE, math.pi / 2),
        (500.0, "mm/s", QuantityKind.LINEAR_VELOCITY, 0.5),
        (180.0, "deg/s", QuantityKind.ANGULAR_VELOCITY, math.pi),
        (60.0, "rpm", QuantityKind.ANGULAR_VELOCITY, math.tau),
        (10.0, "ms", QuantityKind.TIME, 0.01),
        (250.0, "g", QuantityKind.MASS, 0.25),
        (10.0, "N", QuantityKind.FORCE, 10.0),
        (1000.0, "N*mm", QuantityKind.TORQUE, 1.0),
        (1e6, "kg*mm^2", QuantityKind.INERTIA, 1.0),
    ],
)
def test_quantities_normalize_to_the_correct_si_dimension(
    value: float, unit: str, kind: QuantityKind, expected: float
) -> None:
    assert to_si(value, unit, kind=kind) == pytest.approx(expected)
    assert from_si(expected, unit, kind=kind) == pytest.approx(value)


@pytest.mark.parametrize("unit", ["m", "cm", "mm"])
def test_length_preferences_do_not_change_the_physical_value(unit: str) -> None:
    original_m = 0.012345678912345
    displayed = from_si(original_m, unit, kind=QuantityKind.LENGTH)
    assert to_si(displayed, unit, kind=QuantityKind.LENGTH) == pytest.approx(original_m)
    assert convert(displayed, unit, "mm") == pytest.approx(original_m * 1000)


def test_torque_and_inertia_scale_by_their_own_dimensions() -> None:
    assert convert(1.0, "N*m", "N*mm") == pytest.approx(1000.0)
    assert convert(1.0, "kg*m^2", "kg*mm^2") == pytest.approx(1e6)
    assert convert(60.0, "RPM", "deg/s") == pytest.approx(360.0)


def test_scalar_angle_conversion_preserves_multiple_revolutions() -> None:
    assert to_si(450.0, "deg") == pytest.approx(2.5 * math.pi)
    assert from_si(4 * math.tau, "deg") == pytest.approx(1440.0)


@pytest.mark.parametrize(
    ("text", "unit", "expected"),
    [
        ("25 mm", "m", 0.025),
        ("2.5cm", "mm", 0.025),
        ("  +2.5e1 mm  ", "cm", 0.025),
        ("-.5e2mm", "m", -0.05),
        ("50", "mm", 0.05),
        (".025", "m", 0.025),
        ("25.", "mm", 0.025),
    ],
)
def test_authoring_text_accepts_explicit_units_or_the_display_default(
    text: str, unit: str, expected: float
) -> None:
    assert parse_quantity(text, kind=QuantityKind.LENGTH, default_unit=unit) == pytest.approx(
        expected
    )


def test_degree_suffixes_use_the_same_parser_as_numeric_fields() -> None:
    assert parse_quantity("90°", kind=QuantityKind.ANGLE, default_unit="rad") == pytest.approx(
        math.pi / 2
    )


@pytest.mark.parametrize("text", ["", "nan", "inf", "1e999 mm", "1 + 2 mm", "1/2 m", "5 unknown"])
def test_malformed_or_non_finite_input_is_rejected(text: str) -> None:
    with pytest.raises(UnitError):
        parse_quantity(text, kind=QuantityKind.LENGTH, default_unit="mm")


def test_incompatible_quantities_are_never_silently_converted() -> None:
    with pytest.raises(UnitError, match="not"):
        convert(1.0, "mm", "deg")
    with pytest.raises(UnitError, match="not"):
        to_si(1.0, "rad", kind=QuantityKind.LENGTH)
    with pytest.raises(UnitError, match="not"):
        from_si(1.0, "N*m", kind=QuantityKind.FORCE)
    with pytest.raises(UnitError, match="not"):
        parse_quantity("90 deg", kind=QuantityKind.LENGTH, default_unit="mm")
    with pytest.raises(UnitError, match="not"):
        parse_quantity("50 mm", kind=QuantityKind.LENGTH, default_unit="deg")


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, True, "2", None])
def test_conversion_boundaries_reject_invalid_numeric_values(value: object) -> None:
    for operation in (to_si, from_si):
        with pytest.raises(UnitError):
            operation(cast("float", value), "m")
    with pytest.raises(UnitError):
        convert(cast("float", value), "m", "cm")


def test_conversion_overflow_is_rejected_even_for_finite_inputs() -> None:
    with pytest.raises(UnitError, match="finite"):
        convert(1e308, "m", "mm")
    with pytest.raises(UnitError, match="finite"):
        from_si(1e308, "mm")
    with pytest.raises(UnitError, match="finite"):
        to_si(10**400, "m")


def test_vector_conversion_retains_components_and_input_values() -> None:
    values = [50.0, -25.0, 0.0]
    canonical = to_si_vector(values, "mm", kind=QuantityKind.LENGTH)
    assert canonical == pytest.approx((0.05, -0.025, 0.0))
    assert from_si_vector(canonical, "cm", kind=QuantityKind.LENGTH) == pytest.approx(
        (5.0, -2.5, 0.0)
    )
    assert values == [50.0, -25.0, 0.0]


def test_vectors_reject_incorrect_sizes_and_invalid_components() -> None:
    for values in ((), (1.0, 2.0), (1.0, 2.0, 3.0, 4.0)):
        with pytest.raises(UnitError, match="three"):
            to_si_vector(values, "mm", kind=QuantityKind.LENGTH)
        with pytest.raises(UnitError, match="three"):
            from_si_vector(values, "mm", kind=QuantityKind.LENGTH)
    with pytest.raises(UnitError, match="finite"):
        to_si_vector((1.0, math.nan, 2.0), "mm", kind=QuantityKind.LENGTH)


def test_formatting_only_affects_display_text() -> None:
    value_m = 0.050123456789
    assert format_quantity(value_m, "mm", precision=4) == "50.12 mm"
    assert value_m == 0.050123456789
    assert format_quantity(math.pi / 2, "°") == "90 deg"
    assert format_quantity(-0.0, "m") == "0 m"


@pytest.mark.parametrize("precision", [0, -1, 18, True, 1.5])
def test_formatting_rejects_invalid_precision(precision: object) -> None:
    with pytest.raises(UnitError, match="precision"):
        format_quantity(1.0, "m", precision=cast("int", precision))


def test_preferences_are_validated_immutable_display_values() -> None:
    preferences = UnitPreferences()
    assert preferences.length_unit == "mm"
    assert preferences.angle_unit == "deg"
    assert UnitPreferences(length_unit="cm", angle_unit="rad").length_unit == "cm"
    with pytest.raises(FrozenInstanceError):
        preferences.length_unit = "m"  # type: ignore[misc]
    with pytest.raises(UnitError, match="length"):
        UnitPreferences(length_unit="deg")
    with pytest.raises(UnitError, match="angle"):
        UnitPreferences(angle_unit="mm")
